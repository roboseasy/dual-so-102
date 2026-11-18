"""7축 단팔(arm.py) 시험 — 버스 구성 · 보정 경로 · 재시도 · 미등록."""

import errno
import json
import os
import subprocess
import sys
import unittest
from unittest.mock import patch

from lerobot.motors import MotorNormMode
from lerobot.robots.so_follower import SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig
import scservo_sdk as scs

from lerobot_robot_dual_so102.arm import make_bus
from lerobot_robot_dual_so102.arm import SO102ArmFollower
from lerobot_robot_dual_so102.arm import SO102ArmLeader
from tests.fake_bus import use_temp_calibration_root

SEVEN_JOINTS = [
	'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
	'wrist_yaw', 'wrist_roll', 'gripper',
]


class ArmTest(unittest.TestCase):
	"""단팔 팔로워 · 리더가 공식 클래스에 7축 버스만 바꿔 끼웠는지 본다."""

	def setUp(self) -> None:
		self.root = use_temp_calibration_root(self)

	def test_follower_bus_is_seven_axis(self) -> None:
		"""팔로워 · 리더 버스가 ID 1~7 의 7축이고 정규화가 공식 규칙대로다."""
		follower = SO102ArmFollower(
			SOFollowerRobotConfig(port='/dev/x', id='bi_so102_left'))
		leader = SO102ArmLeader(
			SOLeaderTeleopConfig(port='/dev/y', id='bi_so102_leader_left'))
		for arm, port in ((follower, '/dev/x'), (leader, '/dev/y')):
			with self.subTest(arm=arm.name):
				self.assertEqual(list(arm.bus.motors), SEVEN_JOINTS)
				self.assertEqual([m.id for m in arm.bus.motors.values()],
					[1, 2, 3, 4, 5, 6, 7])
				self.assertEqual(arm.bus.port, port)
				self.assertEqual(arm.bus.motors['gripper'].norm_mode,
					MotorNormMode.RANGE_0_100)
				self.assertEqual(arm.bus.motors['wrist_yaw'].norm_mode,
					MotorNormMode.DEGREES)
		ranged = SO102ArmFollower(SOFollowerRobotConfig(
			port='/dev/x', id='bi_so102_left', use_degrees=False))
		self.assertEqual(ranged.bus.motors['wrist_yaw'].norm_mode,
			MotorNormMode.RANGE_M100_100)

	def test_calibration_paths_reuse_so102_names(self) -> None:
		"""보정 파일이 기존 단팔 이름(so102_follower · so102_leader) 폴더에 있다."""
		follower = SO102ArmFollower(
			SOFollowerRobotConfig(port='/dev/x', id='bi_so102_left'))
		leader = SO102ArmLeader(
			SOLeaderTeleopConfig(port='/dev/y', id='bi_so102_leader_left'))
		self.assertEqual(follower.calibration_fpath,
			self.root / 'robots' / 'so102_follower' / 'bi_so102_left.json')
		self.assertEqual(leader.calibration_fpath,
			self.root / 'teleoperators' / 'so102_leader'
			/ 'bi_so102_leader_left.json')

	def test_bus_retries_twice_by_default(self) -> None:
		"""통신 실패 시 sync_read · sync_write 를 처음 + 재시도 2번, 모두 3번 한다."""
		bus = make_bus('/dev/x', True, {})
		bus.port_handler.is_open = True  # 포트를 열지 않고 연결된 상태로
		self.addCleanup(setattr, bus.port_handler, 'is_open', False)
		with patch.object(bus.sync_reader, 'txRxPacket',
			return_value=scs.COMM_RX_TIMEOUT) as read_packet:
			with self.assertRaises(ConnectionError):
				bus.sync_read('Present_Position')
		with patch.object(bus.sync_writer, 'txPacket',
			return_value=scs.COMM_TX_FAIL) as write_packet:
			with self.assertRaises(ConnectionError):
				bus.sync_write('Goal_Position', {'gripper': 100},
					normalize=False)
		self.assertEqual(read_packet.call_count, 3)
		self.assertEqual(write_packet.call_count, 3)

	def test_register_writes_retry_twice_by_default(self) -> None:
		"""연결 때 쓰는 레지스터 쓰기 · 토크 켜기도 3번 한다 (같은 값이라 다시 써도 안전).

		실장비(2026-11-18): host 연결 중 Torque_Enable 쓰기 한 번이 'Incorrect status
		packet' 으로 실패해 연결 전체가 끝났다. 바로 다음 판별에서는 두 팔 모두 정상.
		"""
		bus = make_bus('/dev/x', True, {})
		bus.port_handler.is_open = True  # 포트를 열지 않고 연결된 상태로
		self.addCleanup(setattr, bus.port_handler, 'is_open', False)
		for name, action in (
			('write', lambda: bus.write('P_Coefficient', 'gripper', 16)),
			('enable_torque', lambda: bus.enable_torque('gripper')),
		):
			with self.subTest(call=name):
				with patch.object(bus.packet_handler, 'writeTxRx',
					return_value=(scs.COMM_RX_CORRUPT, 0)) as write_packet:
					with self.assertRaises(ConnectionError):
						action()
				self.assertEqual(write_packet.call_count, 3)

	@unittest.skipIf(os.geteuid() == 0, 'root 는 배타 포트도 열 수 있다')
	def test_bus_holds_port_exclusively(self) -> None:
		"""연결한 버스의 포트는 다른 프로그램이 열 수 없고, 닫으면 다시 열린다.

		다른 프로그램(포트 판별 · 두 번째 host)이 같은 포트를 열면 서보 응답을
		가로채 host 가 끝나고 두 팔 토크가 풀린다.
		"""
		master, slave = os.openpty()
		self.addCleanup(os.close, master)
		port = os.ttyname(slave)
		os.close(slave)
		bus = make_bus(port, True, {})
		bus.connect(handshake=False)
		try:
			with self.assertRaises(OSError) as raised:
				os.close(os.open(port, os.O_RDWR | os.O_NOCTTY))
			self.assertEqual(raised.exception.errno, errno.EBUSY)
		finally:
			bus.disconnect(disable_torque=False)
		os.close(os.open(port, os.O_RDWR | os.O_NOCTTY))

	def test_registers_nothing_on_import(self) -> None:
		"""import 해도 dual_so102_* 말고는 아무 이름도 등록하지 않는다.

		덮어쓴 LeRobot 이 이미 등록한 so102_* · bi_so102_* 와 부딪히면 안 된다.
		"""
		script = '\n'.join((
			'from lerobot.robots import RobotConfig',
			'from lerobot.teleoperators import TeleoperatorConfig',
			# 이 패키지가 끌어오는 공식 모듈은 자기 이름(so100_* · bi_so_* 등)을
			# 스스로 등록하므로 기준에 먼저 넣는다
			'import lerobot.robots.so_follower',
			'import lerobot.robots.bi_so_follower',
			'import lerobot.teleoperators.so_leader',
			'import lerobot.teleoperators.bi_so_leader',
			'def known():',
			'	return (set(RobotConfig.get_known_choices())',
			'		| set(TeleoperatorConfig.get_known_choices()))',
			'before = known()',
			'import lerobot_robot_dual_so102',
			'import lerobot_robot_dual_so102.arm',
			'import json',
			'print(json.dumps(sorted(known() - before)))',
		))
		result = subprocess.run([sys.executable, '-c', script],
			cwd=self.root, capture_output=True, text=True, timeout=60)
		self.assertEqual(result.returncode, 0, result.stderr)
		added = json.loads(result.stdout.strip().splitlines()[-1])
		self.assertLessEqual(set(added), {
			'dual_so102_follower', 'dual_so102_client', 'dual_so102_leader'})


if __name__ == '__main__':
	unittest.main()
