"""양팔 팔로워 · 리더(dual.py) 시험 — 공식 CLI 자동 발견 · 키 · 보정 · 제어 루프."""

from contextlib import redirect_stdout
import io
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

from lerobot.robots.so_follower import SOFollowerConfig
from lerobot.teleoperators.so_leader import SOLeaderConfig

from lerobot_robot_dual_so102 import DualSO102Follower
from lerobot_robot_dual_so102 import DualSO102FollowerConfig
from lerobot_robot_dual_so102 import DualSO102Leader
from lerobot_robot_dual_so102 import DualSO102LeaderConfig
from tests.fake_bus import camera
from tests.fake_bus import patch_make_bus
from tests.fake_bus import use_temp_calibration_root
from tests.fake_bus import write_calibration_file

SEVEN_JOINTS = (
	'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
	'wrist_yaw', 'wrist_roll', 'gripper',
)
FOURTEEN_KEYS = {f'{side}_{joint}.pos'
	for side in ('left', 'right') for joint in SEVEN_JOINTS}


def follower_config(**kwargs: object) -> DualSO102FollowerConfig:
	"""두 팔 포트가 정해진 양팔 팔로워 설정."""
	return DualSO102FollowerConfig(
		left_arm_config=SOFollowerConfig(port='/dev/so102_follower_left'),
		right_arm_config=SOFollowerConfig(port='/dev/so102_follower_right'),
		**kwargs)


def leader_config(**kwargs: object) -> DualSO102LeaderConfig:
	"""두 팔 포트가 정해진 양팔 리더 설정."""
	return DualSO102LeaderConfig(
		left_arm_config=SOLeaderConfig(port='/dev/so102_leader_left'),
		right_arm_config=SOLeaderConfig(port='/dev/so102_leader_right'),
		**kwargs)


def run_python(script: str, cwd: object) -> subprocess.CompletedProcess:
	"""새 파이썬 프로세스에서 스크립트를 돌린다 (import 상태가 깨끗하다)."""
	return subprocess.run([sys.executable, '-c', script], cwd=cwd,
		capture_output=True, text=True, timeout=120)


class DualTest(unittest.TestCase):
	"""공식 Bi 클래스를 상속한 양팔이 공식 CLI 에서 그대로 도는지 본다."""

	def setUp(self) -> None:
		self.root = use_temp_calibration_root(self)

	def test_fresh_process_discovery(self) -> None:
		"""사용자 import 없이 배포 이름만으로 공식 팩터리가 양팔을 만든다."""
		script = '\n'.join((
			'from pathlib import Path',
			'from lerobot.utils.import_utils import register_third_party_plugins',
			'from lerobot.robots import RobotConfig, make_robot_from_config',
			'from lerobot.robots.so_follower import SOFollowerConfig',
			'from lerobot.teleoperators import TeleoperatorConfig',
			'from lerobot.teleoperators import make_teleoperator_from_config',
			'from lerobot.teleoperators.so_leader import SOLeaderConfig',
			'register_third_party_plugins()',
			"robot = make_robot_from_config(RobotConfig.get_choice_class(",
			"	'dual_so102_follower')(id='bi_so102', calibration_dir=Path('c'),",
			"	left_arm_config=SOFollowerConfig(port='/dev/missing_l'),",
			"	right_arm_config=SOFollowerConfig(port='/dev/missing_r')))",
			"assert robot.left_arm.bus.motors['wrist_yaw'].id == 5",
			"assert robot.right_arm.bus.motors['gripper'].id == 7",
			"assert robot.right_arm.bus.port == '/dev/missing_r'",
			'assert not robot.is_connected',
			"teleop = make_teleoperator_from_config(TeleoperatorConfig",
			"	.get_choice_class('dual_so102_leader')(id='bi_so102_leader',",
			"	calibration_dir=Path('c'),",
			"	left_arm_config=SOLeaderConfig(port='/dev/missing_l'),",
			"	right_arm_config=SOLeaderConfig(port='/dev/missing_r')))",
			"assert teleop.left_arm.bus.motors['wrist_yaw'].id == 5",
			"print('AUTO_DISCOVERY_OK')",
		))
		result = run_python(script, self.root)
		self.assertEqual(result.returncode, 0, result.stderr)
		self.assertIn('AUTO_DISCOVERY_OK', result.stdout)

	def test_official_help(self) -> None:
		"""공식 teleoperate · calibrate 도움말이 두 타입을 알아본다."""
		for module, option in (('lerobot_teleoperate', '--robot.type'),
			('lerobot_calibrate', '--teleop.type')):
			kind = ('dual_so102_follower' if option == '--robot.type'
				else 'dual_so102_leader')
			with self.subTest(module=module):
				result = subprocess.run([sys.executable, '-m',
					f'lerobot.scripts.{module}', f'{option}={kind}', '--help'],
					cwd=self.root, capture_output=True, text=True, timeout=120)
				self.assertEqual(result.returncode, 0, result.stderr)
				self.assertIn('left_arm_config', result.stdout)

	def test_feature_keys_are_14(self) -> None:
		"""관측 · 행동 키가 left_/right_ × 7관절 .pos, 카메라는 공식 규칙대로다."""
		robot = DualSO102Follower(DualSO102FollowerConfig(
			id='bi_so102',
			left_arm_config=SOFollowerConfig(port='/dev/l',
				cameras={'wrist': camera(240, 320)}),
			right_arm_config=SOFollowerConfig(port='/dev/r'),
			cameras={'cam_top': camera(480, 640)}))
		self.assertEqual(set(robot.action_features), FOURTEEN_KEYS)
		self.assertEqual(robot.observation_features, {
			**dict.fromkeys(FOURTEEN_KEYS, float),
			'cam_top': (480, 640, 3), 'left_wrist': (240, 320, 3)})
		leader = DualSO102Leader(leader_config(id='bi_so102_leader'))
		self.assertEqual(set(leader.action_features), FOURTEEN_KEYS)
		with self.assertRaisesRegex(ValueError, 'cam_top'):
			DualSO102Follower(DualSO102FollowerConfig(
				id='bi_so102',
				left_arm_config=SOFollowerConfig(port='/dev/l',
					cameras={'cam_top': camera(240, 320)}),
				right_arm_config=SOFollowerConfig(port='/dev/r'),
				cameras={'cam_top': camera(480, 640)}))

	def test_reuses_existing_calibration_files(self) -> None:
		"""--robot.id=bi_so102 면 기존 so102_follower/bi_so102_{left,right}.json 을 읽는다."""
		robots = self.root / 'robots' / 'so102_follower'
		teleops = self.root / 'teleoperators' / 'so102_leader'
		write_calibration_file(robots / 'bi_so102_left.json', range_min=1101)
		write_calibration_file(robots / 'bi_so102_right.json', range_min=1202)
		write_calibration_file(teleops / 'bi_so102_leader_left.json',
			range_min=1303)
		write_calibration_file(teleops / 'bi_so102_leader_right.json',
			range_min=1404)
		robot = DualSO102Follower(follower_config(id='bi_so102'))
		leader = DualSO102Leader(leader_config(id='bi_so102_leader'))
		self.assertEqual(robot.left_arm.calibration_fpath,
			robots / 'bi_so102_left.json')
		for arm, expected in ((robot.left_arm, 1101), (robot.right_arm, 1202),
			(leader.left_arm, 1303), (leader.right_arm, 1404)):
			with self.subTest(arm=str(arm)):
				self.assertEqual(
					arm.bus.calibration['wrist_yaw'].range_min, expected)

	def test_teleoperate_loop(self) -> None:
		"""공식 lerobot-teleoperate 로 리더 왼쪽 → 팔로워 왼쪽, 오른쪽 → 오른쪽."""
		from lerobot.scripts import lerobot_teleoperate

		buses = patch_make_bus(self, {
			'/dev/so102_leader_left': {'shoulder_pan': 11.0},
			'/dev/so102_leader_right': {'shoulder_pan': -22.0},
		})
		for role, side in (('robot', 'left'), ('robot', 'right'),
			('teleop', 'left'), ('teleop', 'right')):
			device_id = 'bi_so102' if role == 'robot' else 'bi_so102_leader'
			write_calibration_file(
				self.root / role / f'{device_id}_{side}.json')
		args = [
			'lerobot-teleoperate',
			'--robot.type=dual_so102_follower', '--robot.id=bi_so102',
			'--robot.left_arm_config.port=/dev/so102_follower_left',
			'--robot.right_arm_config.port=/dev/so102_follower_right',
			f'--robot.calibration_dir={self.root / "robot"}',
			'--teleop.type=dual_so102_leader', '--teleop.id=bi_so102_leader',
			'--teleop.left_arm_config.port=/dev/so102_leader_left',
			'--teleop.right_arm_config.port=/dev/so102_leader_right',
			f'--teleop.calibration_dir={self.root / "teleop"}',
			'--display_data=false', '--teleop_time_s=0.02',
		]
		with patch('sys.argv', args), redirect_stdout(io.StringIO()):
			lerobot_teleoperate.main()
		self.assertEqual(len(buses), 4)
		left_goals = [call[1] for call in
			buses['/dev/so102_follower_left'].calls if call[0] == 'goal']
		right_goals = [call[1] for call in
			buses['/dev/so102_follower_right'].calls if call[0] == 'goal']
		self.assertTrue(left_goals and right_goals)
		self.assertEqual(left_goals[0], {
			'shoulder_pan': 11.0, 'shoulder_lift': 0.0, 'elbow_flex': 0.0,
			'wrist_flex': 0.0, 'wrist_yaw': 1.0, 'wrist_roll': 0.0,
			'gripper': 50.0})
		self.assertEqual(right_goals[0]['shoulder_pan'], -22.0)
		self.assertTrue(all(not bus.is_connected for bus in buses.values()))

	def test_calibrate_writes_left_and_right(self) -> None:
		"""공식 lerobot-calibrate 가 <id>_left.json · <id>_right.json 7관절을 쓴다."""
		from lerobot.scripts import lerobot_calibrate

		buses = patch_make_bus(self)
		for role, kind, device_id in (
			('robot', 'dual_so102_follower', 'bi_so102'),
			('teleop', 'dual_so102_leader', 'bi_so102_leader'),
		):
			with self.subTest(role=role):
				directory = self.root / role
				args = [
					'lerobot-calibrate', f'--{role}.type={kind}',
					f'--{role}.id={device_id}',
					f'--{role}.left_arm_config.port=/dev/{role}_left',
					f'--{role}.right_arm_config.port=/dev/{role}_right',
					f'--{role}.calibration_dir={directory}',
				]
				with patch('sys.argv', args), patch('builtins.input',
					return_value=''), redirect_stdout(io.StringIO()):
					lerobot_calibrate.main()
				for side in ('left', 'right'):
					data = json.loads(
						(directory / f'{device_id}_{side}.json').read_text())
					self.assertEqual(set(data), set(SEVEN_JOINTS))
					self.assertEqual(data['gripper']['id'], 7)
					self.assertEqual(data['wrist_yaw']['range_min'], 1000)
					self.assertEqual(data['wrist_yaw']['range_max'], 3000)
					self.assertEqual(data['wrist_roll']['range_min'], 0)
					self.assertEqual(data['wrist_roll']['range_max'], 4095)
					self.assertFalse(buses[f'/dev/{role}_{side}'].is_connected)

	def test_requires_id(self) -> None:
		"""id 가 없으면 두 팔이 None.json 을 같이 덮어쓰기 전에 멈춘다."""
		with self.assertRaisesRegex(ValueError, '--robot.id'):
			DualSO102Follower(follower_config())
		with self.assertRaisesRegex(ValueError, '--teleop.id'):
			DualSO102Leader(leader_config())
		self.assertFalse(any(self.root.rglob('None.json')))

	def test_coexists_with_overwritten_lerobot(self) -> None:
		"""solution 을 덮어쓴 LeRobot 처럼 so102_* · bi_so102_* 가 이미 있어도 된다."""
		script = '\n'.join((
			'from dataclasses import dataclass',
			'from lerobot.robots import RobotConfig',
			'from lerobot.teleoperators import TeleoperatorConfig',
			"for name in ('so102_follower', 'bi_so102_follower',",
			"	'bi_so102_client'):",
			'	RobotConfig.register_subclass(name)(',
			"		dataclass(type('Old' + name, (RobotConfig,), {})))",
			"for name in ('so102_leader', 'bi_so102_leader'):",
			'	TeleoperatorConfig.register_subclass(name)(',
			"		dataclass(type('Old' + name, (TeleoperatorConfig,), {})))",
			'import lerobot_robot_dual_so102',
			"assert RobotConfig.get_choice_class('dual_so102_follower')",
			"assert TeleoperatorConfig.get_choice_class('dual_so102_leader')",
			"assert RobotConfig.get_choice_class('dual_so102_client')",
			"print('COEXIST_OK')",
		))
		result = run_python(script, self.root)
		self.assertEqual(result.returncode, 0, result.stderr)
		self.assertIn('COEXIST_OK', result.stdout)


if __name__ == '__main__':
	unittest.main()
