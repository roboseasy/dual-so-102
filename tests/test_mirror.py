"""미러 모드 시험 — 1 · 5 · 6번 부호 반전, 좌우 교체, 보정 파일 불변."""

import unittest
from unittest.mock import patch

from lerobot.motors import MotorCalibration
from lerobot.robots.so_follower import SOFollowerConfig
import scservo_sdk as scs

from lerobot_robot_dual_so102 import DualSO102Follower
from lerobot_robot_dual_so102 import DualSO102FollowerConfig
from lerobot_robot_dual_so102.arm import make_bus
from tests.fake_bus import camera
from tests.fake_bus import patch_make_bus
from tests.fake_bus import use_temp_calibration_root
from tests.fake_bus import write_calibration_file

# 원시값 2500, 보정 범위 1000~3000 (가운데 2000) → 도 단위 500 * 360 / 4095
DEGREES_AT_2500 = 43.956044
# 그리퍼(0~100, drive_mode=1): 100 - (2500 - 1000) / 2000 * 100
GRIPPER_AT_2500 = 25.0


class MirrorTest(unittest.TestCase):
	"""기존 양팔 코드의 --robot.mirror_mode=true 동작을 그대로 옮겼는지 본다."""

	def setUp(self) -> None:
		self.root = use_temp_calibration_root(self)

	def test_mirror_swaps_ports_ids_and_arm_cameras(self) -> None:
		"""left_* 팔이 오른팔 포트 · <id>_right 보정 · 오른팔 설정 · 팔별 카메라를 쓴다."""
		config = DualSO102FollowerConfig(
			id='bi_so102', mirror_mode=True,
			left_arm_config=SOFollowerConfig(port='/dev/L',
				cameras={'cam_left': camera(240, 320)}),
			right_arm_config=SOFollowerConfig(port='/dev/R',
				max_relative_target=7.0),
			cameras={'cam_top': camera(480, 640)})
		with self.assertLogs('lerobot_robot_dual_so102.dual', 'INFO') as logs:
			robot = DualSO102Follower(config)
		self.assertEqual((robot.left_arm.config.port, robot.left_arm.id),
			('/dev/R', 'bi_so102_right'))
		self.assertEqual((robot.right_arm.config.port, robot.right_arm.id),
			('/dev/L', 'bi_so102_left'))
		self.assertEqual(robot.left_arm.config.max_relative_target, 7.0)
		self.assertIsNone(robot.right_arm.config.max_relative_target)
		self.assertEqual(set(robot.left_arm.cameras), {'cam_top'})
		self.assertEqual(set(robot.right_arm.cameras), {'cam_left'})
		self.assertEqual({k for k in robot.observation_features
			if not k.endswith('.pos')}, {'cam_top', 'right_cam_left'})
		self.assertTrue(robot.left_arm.bus.is_mirror)
		self.assertTrue(robot.right_arm.bus.is_mirror)
		self.assertIn('Mirror mode: left_* -> /dev/R (bi_so102_right), '
			'right_* -> /dev/L (bi_so102_left)', '\n'.join(logs.output))

	def read_and_write(self, is_mirror: bool) -> tuple[dict, dict]:
		"""원시값 2500 을 읽어 정규화하고, 그 값을 다시 써서 원시값을 받는다."""
		calibration = {
			name: MotorCalibration(id=index, drive_mode=1, homing_offset=0,
				range_min=1000, range_max=3000)
			for index, name in enumerate((
				'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
				'wrist_yaw', 'wrist_roll', 'gripper'), 1)
		}
		bus = make_bus('/dev/x', True, calibration, is_mirror=is_mirror)
		bus.port_handler.is_open = True  # 포트를 열지 않고 연결된 상태로
		self.addCleanup(setattr, bus.port_handler, 'is_open', False)
		raw = {motor_id: 2500 for motor_id in range(1, 8)}
		with patch.object(bus, '_sync_read',
			return_value=(raw, scs.COMM_SUCCESS)):
			degrees = bus.sync_read('Present_Position')
		with patch.object(bus, '_sync_write',
			return_value=scs.COMM_SUCCESS) as sync_write:
			bus.sync_write('Goal_Position', degrees)
		return degrees, sync_write.call_args.args[2]

	def test_mirror_flips_only_mirror_joints_in_degrees(self) -> None:
		"""drive_mode=1 이어도 도 단위 부호는 미러 버스의 1 · 5 · 6번만 뒤집힌다."""
		plain, plain_raw = self.read_and_write(is_mirror=False)
		mirror, mirror_raw = self.read_and_write(is_mirror=True)
		expected = {
			'shoulder_pan': (DEGREES_AT_2500, -DEGREES_AT_2500),
			'shoulder_lift': (DEGREES_AT_2500, DEGREES_AT_2500),
			'elbow_flex': (DEGREES_AT_2500, DEGREES_AT_2500),
			'wrist_flex': (DEGREES_AT_2500, DEGREES_AT_2500),
			'wrist_yaw': (DEGREES_AT_2500, -DEGREES_AT_2500),
			'wrist_roll': (DEGREES_AT_2500, -DEGREES_AT_2500),
			'gripper': (GRIPPER_AT_2500, GRIPPER_AT_2500),
		}
		for joint, (plain_value, mirror_value) in expected.items():
			with self.subTest(joint=joint):
				self.assertAlmostEqual(plain[joint], plain_value, places=5)
				self.assertAlmostEqual(mirror[joint], mirror_value, places=5)
		# 읽은 값을 그대로 쓰면 같은 원시 위치로 돌아간다 (정수 변환 오차 1 이내)
		for raw in (plain_raw, mirror_raw):
			for motor_id, value in raw.items():
				self.assertLessEqual(abs(value - 2500), 1, (motor_id, raw))

	def test_mirror_keeps_calibration_file(self) -> None:
		"""연결하면 버스 보정 사본만 1 · 5 · 6번 drive_mode=1, 파일 · 원본은 그대로다."""
		files = self.root / 'robots' / 'so102_follower'
		write_calibration_file(files / 'bi_so102_left.json')
		write_calibration_file(files / 'bi_so102_right.json')
		before = {path.name: path.read_bytes() for path in files.iterdir()}
		patch_make_bus(self)
		robot = DualSO102Follower(DualSO102FollowerConfig(
			id='bi_so102', mirror_mode=True,
			left_arm_config=SOFollowerConfig(port='/dev/L'),
			right_arm_config=SOFollowerConfig(port='/dev/R')))
		with self.assertLogs('lerobot_robot_dual_so102.arm', 'INFO') as logs:
			robot.connect()
		for arm in (robot.left_arm, robot.right_arm):
			with self.subTest(arm=arm.id):
				self.assertEqual({joint: c.drive_mode
					for joint, c in arm.bus.calibration.items()}, {
					'shoulder_pan': 1, 'shoulder_lift': 0, 'elbow_flex': 0,
					'wrist_flex': 0, 'wrist_yaw': 1, 'wrist_roll': 1,
					'gripper': 0})
				self.assertEqual(arm.calibration['wrist_yaw'].drive_mode, 0)
		self.assertIn('mirror mode: drive_mode=1 on shoulder_pan, wrist_yaw, '
			'wrist_roll (calibration file unchanged)', '\n'.join(logs.output))
		robot.disconnect()
		self.assertEqual(
			{path.name: path.read_bytes() for path in files.iterdir()}, before)

	def test_mirror_off_by_default(self) -> None:
		"""mirror_mode 를 주지 않으면 좌우 그대로, 연결해도 drive_mode 를 바꾸지 않는다."""
		files = self.root / 'robots' / 'so102_follower'
		write_calibration_file(files / 'bi_so102_left.json')
		write_calibration_file(files / 'bi_so102_right.json')
		patch_make_bus(self)
		robot = DualSO102Follower(DualSO102FollowerConfig(
			id='bi_so102',
			left_arm_config=SOFollowerConfig(port='/dev/L'),
			right_arm_config=SOFollowerConfig(port='/dev/R')))
		self.assertEqual((robot.left_arm.config.port, robot.left_arm.id),
			('/dev/L', 'bi_so102_left'))
		self.assertFalse(robot.left_arm.bus.is_mirror)
		robot.connect()
		self.assertEqual(
			robot.left_arm.bus.calibration['wrist_yaw'].drive_mode, 0)
		robot.disconnect()


if __name__ == '__main__':
	unittest.main()
