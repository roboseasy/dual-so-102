"""Pi host(dual-so102-host) 시험 — 기존 host 동작 · 종료 정리 · 포트 사용 중."""

import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

from lerobot.robots.so_follower import SOFollowerConfig
import numpy as np
import zmq

from lerobot_robot_dual_so102 import DualSO102Client
from lerobot_robot_dual_so102 import DualSO102ClientConfig
from lerobot_robot_dual_so102 import DualSO102Follower
from lerobot_robot_dual_so102 import DualSO102FollowerConfig
from lerobot_robot_dual_so102.config import DualSO102HostConfig
from lerobot_robot_dual_so102.host import main
from lerobot_robot_dual_so102.host import run_host
from tests.fake_bus import patch_make_bus
from tests.fake_bus import use_temp_calibration_root
from tests.fake_bus import write_calibration_file

REPO = Path(__file__).resolve().parents[1]
SEVEN_JOINTS = (
	'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
	'wrist_yaw', 'wrist_roll', 'gripper',
)
FOURTEEN_KEYS = {f'{side}_{joint}.pos'
	for side in ('left', 'right') for joint in SEVEN_JOINTS}


def free_port() -> int:
	"""지금 비어 있는 TCP 포트."""
	with socket.socket() as probe:
		probe.bind(('127.0.0.1', 0))
		return probe.getsockname()[1]


class FakeCamera:
	"""늦거나 빠질 수 있는 가짜 카메라 (OpenCVCamera.read_latest 와 같은 예외).

	Attributes:
		is_stalled: True 면 read_latest 가 failure 를 낸다.
		failure: 늦음은 TimeoutError, USB 가 빠져 읽기 스레드가 멈추면 RuntimeError.
	"""

	def __init__(self, failure: Exception | None = None) -> None:
		self.is_connected = True
		self.is_stalled = False
		self.failure = failure or TimeoutError('가짜 카메라: 프레임이 500 ms 넘게 늦음')

	def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
		"""가장 최근 프레임 (멈췄으면 failure)."""
		if self.is_stalled:
			raise self.failure
		return np.full((48, 64, 3), 200, np.uint8)

	def disconnect(self) -> None:
		"""가상 해제."""
		self.is_connected = False


class HostTest(unittest.TestCase):
	"""가짜 버스 양팔을 host 로 띄워 client 와 주고받는다."""

	def setUp(self) -> None:
		self.root = use_temp_calibration_root(self)
		files = self.root / 'robots' / 'so102_follower'
		write_calibration_file(files / 'bi_so102_left.json')
		write_calibration_file(files / 'bi_so102_right.json')
		self.buses = patch_make_bus(self)
		self.host_config = DualSO102HostConfig(port_zmq_cmd=free_port(),
			port_zmq_observations=free_port(), watchdog_timeout_ms=500)

	def connected_robot(self) -> DualSO102Follower:
		"""가짜 버스로 연결한 양팔."""
		robot = DualSO102Follower(DualSO102FollowerConfig(id='bi_so102',
			left_arm_config=SOFollowerConfig(port='/dev/L'),
			right_arm_config=SOFollowerConfig(port='/dev/R')))
		robot.connect()
		self.addCleanup(lambda: robot.disconnect()
			if robot.is_connected else None)
		return robot

	def start_host(self, robot: DualSO102Follower) -> threading.Event:
		"""run_host 를 스레드에서 돌린다. 돌려준 이벤트를 set 하면 멈춘다."""
		stop = threading.Event()
		thread = threading.Thread(target=run_host,
			args=(robot, self.host_config, stop.is_set), daemon=True)
		thread.start()

		def stop_host() -> None:
			stop.set()
			thread.join(timeout=5)
		self.addCleanup(stop_host)
		stop.stop_host = stop_host  # 시험 안에서 먼저 멈출 때
		stop.thread = thread
		return stop

	def observation_receiver(self) -> zmq.Socket:
		"""host 관측을 받는 소켓 (client 하나처럼)."""
		context = zmq.Context()
		self.addCleanup(context.term)
		receiver = context.socket(zmq.PULL)
		self.addCleanup(receiver.close, 0)
		receiver.setsockopt(zmq.CONFLATE, 1)
		receiver.connect(
			f'tcp://127.0.0.1:{self.host_config.port_zmq_observations}')
		return receiver

	def latest_observation(self, receiver: zmq.Socket) -> dict:
		"""host 가 보낸 가장 최근 관측."""
		self.assertTrue(receiver.poll(3000))
		return json.loads(receiver.recv_string())

	def start_host_process(self, ignore_sighup: bool = False,
		) -> subprocess.Popen:
		"""가짜 버스 host 를 새 프로세스로 띄우고, 루프가 돌 때까지 기다린다.

		버스를 닫으면 'BUS_DISCONNECTED <포트> <토크 해제>' 를 찍는다.
		"""
		calibration = self.root / 'cal'
		write_calibration_file(calibration / 'bi_so102_left.json')
		write_calibration_file(calibration / 'bi_so102_right.json')
		cmd_port, obs_port = free_port(), free_port()
		script = '\n'.join((
			'import sys',
			'from unittest.mock import patch',
			'from tests.fake_bus import FakeBus',
			'class LoudBus(FakeBus):',
			'	def disconnect(self, disable_torque=True):',
			'		super().disconnect(disable_torque)',
			"		print('BUS_DISCONNECTED', self.port, disable_torque,",
			'			flush=True)',
			"with patch('lerobot_robot_dual_so102.arm.make_bus', LoudBus):",
			'	from lerobot_robot_dual_so102.host import main',
			"	sys.argv = ['dual-so102-host', '--robot.id=bi_so102',",
			f"		'--robot.calibration_dir={calibration}',",
			"		'--robot.left_arm_config.port=/dev/L',",
			"		'--robot.right_arm_config.port=/dev/R',",
			f"		'--host.port_zmq_cmd={cmd_port}',",
			f"		'--host.port_zmq_observations={obs_port}']",
			'	main()',
		))

		def ignore_hangup() -> None:
			signal.signal(signal.SIGHUP, signal.SIG_IGN)  # nohup 과 같다
		process = subprocess.Popen([sys.executable, '-c', script],
			cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
			text=True, preexec_fn=ignore_hangup if ignore_sighup else None)
		self.addCleanup(lambda: process.poll() is None and process.kill())
		client = DualSO102Client(DualSO102ClientConfig(
			remote_ip='127.0.0.1', port_zmq_cmd=cmd_port,
			port_zmq_observations=obs_port, connect_timeout_s=30))
		client.connect()  # host 루프가 돈다
		client.disconnect()
		return process

	def test_host_client_roundtrip(self) -> None:
		"""client 명령이 왼팔 버스 목표로 가고, client 는 14 관절 관측을 받는다."""
		robot = self.connected_robot()
		self.start_host(robot)
		client = DualSO102Client(DualSO102ClientConfig(remote_ip='127.0.0.1',
			port_zmq_cmd=self.host_config.port_zmq_cmd,
			port_zmq_observations=self.host_config.port_zmq_observations))
		client.connect()
		self.addCleanup(client.disconnect)
		self.assertEqual(set(client.get_observation()), FOURTEEN_KEYS)
		client.send_action({**dict.fromkeys(FOURTEEN_KEYS, 0.0),
			'left_shoulder_pan.pos': 12.5})
		deadline = time.monotonic() + 3
		goals = []
		while time.monotonic() < deadline and not goals:
			goals = [call[1] for call in self.buses['/dev/L'].calls
				if call[0] == 'goal' and call[1].get('shoulder_pan') == 12.5]
			time.sleep(0.02)
		self.assertTrue(goals)

	def test_watchdog_holds_position(self) -> None:
		"""명령이 watchdog 보다 오래 없으면 경고 한 번, 새 목표를 쓰지 않는다."""
		self.host_config.watchdog_timeout_ms = 100
		robot = self.connected_robot()
		with self.assertLogs('lerobot_robot_dual_so102.host', 'WARNING') as logs:
			stop = self.start_host(robot)
			time.sleep(0.4)
			stop.stop_host()
		holds = [line for line in logs.output if '자세를 유지' in line]
		self.assertEqual(len(holds), 1, logs.output)
		goals = [call for bus in self.buses.values() for call in bus.calls
			if call[0] == 'goal']
		self.assertEqual(goals, [])

	def test_read_failures_stop_host(self) -> None:
		"""관측 읽기가 연속 5번 실패하면 경고 4번 · 오류 1번 뒤 ConnectionError."""
		robot = self.connected_robot()
		self.buses['/dev/L'].fail_reads = 10
		with self.assertLogs('lerobot_robot_dual_so102.host', 'WARNING') as logs:
			with self.assertRaises(ConnectionError):
				run_host(robot, self.host_config)
		failures = [line for line in logs.output if '관측 읽기' in line]
		self.assertEqual([line.split(':')[0] for line in failures],
			['WARNING'] * 4 + ['ERROR'])

	def test_read_failure_streak_resets_after_success(self) -> None:
		"""실패 사이에 한 번이라도 읽히면 연속 횟수는 0 부터 다시 센다."""
		robot = self.connected_robot()
		bus = self.buses['/dev/L']
		original_read = bus.sync_read
		pattern = iter([True, True, True, True, False] * 4)

		def flaky_read(*args: object, **kwargs: object) -> dict:
			if next(pattern, False):
				raise ConnectionError('가짜 순간 끊김')
			return original_read(*args, **kwargs)
		bus.sync_read = flaky_read
		with self.assertLogs('lerobot_robot_dual_so102.host', 'WARNING') as logs:
			stop = self.start_host(robot)
			time.sleep(1.0)
			stop.stop_host()
		self.assertFalse([line for line in logs.output if 'ERROR' in line])

	def test_sigterm_runs_cleanup(self) -> None:
		"""SIGTERM · SIGHUP(SSH 끊김)에도 Ctrl+C 처럼 두 팔을 토크 해제하고 끝난다."""
		for signum in (signal.SIGTERM, signal.SIGHUP):
			with self.subTest(signal=signum.name):
				process = self.start_host_process()
				process.send_signal(signum)
				output, _ = process.communicate(timeout=30)
				self.assertEqual(process.returncode, 0, output)
				self.assertIn('BUS_DISCONNECTED /dev/L True', output)
				self.assertIn('BUS_DISCONNECTED /dev/R True', output)

	def test_sighup_ignored_under_nohup(self) -> None:
		"""nohup 처럼 SIGHUP 을 무시하게 띄운 host 는 SSH 가 끊겨도 계속 돈다."""
		process = self.start_host_process(ignore_sighup=True)
		process.send_signal(signal.SIGHUP)
		time.sleep(1.0)
		self.assertIsNone(process.poll())
		process.send_signal(signal.SIGTERM)
		output, _ = process.communicate(timeout=30)
		self.assertEqual(process.returncode, 0, output)
		self.assertIn('BUS_DISCONNECTED /dev/L True', output)

	def test_camera_stall_keeps_host_and_sends_empty_frame(self) -> None:
		"""카메라가 늦거나 빠져도 host 는 끝나지 않고 그 카메라만 빈 프레임으로 보낸다.

		실장비(2026-11-18): Pi USB 가 흔들려 카메라 3대가 빠지자 읽기 스레드가 멈추고
		(RuntimeError) host 가 끝나 두 팔 토크가 풀렸다.
		"""
		for failure in (TimeoutError('늦음'),
			RuntimeError('OpenCVCamera(/dev/cam_top) read thread is not running.')):
			with self.subTest(failure=type(failure).__name__):
				self.check_camera_failure_keeps_host(failure)

	def check_camera_failure_keeps_host(self, failure: Exception) -> None:
		"""카메라 하나가 failure 를 내는 동안과 회복 뒤의 host 관측을 본다."""
		self.host_config = DualSO102HostConfig(port_zmq_cmd=free_port(),
			port_zmq_observations=free_port())
		robot = self.connected_robot()
		camera = FakeCamera(failure)
		robot.left_arm.cameras['cam_top'] = camera
		robot.cameras = {'cam_top': camera}
		robot._top_level_cam_keys = {'cam_top'}
		receiver = self.observation_receiver()
		with self.assertLogs('lerobot_robot_dual_so102.host', 'INFO') as logs:
			stop = self.start_host(robot)
			self.assertNotEqual(self.latest_observation(receiver)['cam_top'], '')
			camera.is_stalled = True
			time.sleep(0.5)
			stalled = self.latest_observation(receiver)
			self.assertEqual(stalled['cam_top'], '')
			self.assertEqual({k for k in stalled if k.endswith('.pos')},
				FOURTEEN_KEYS)
			self.assertTrue(stop.thread.is_alive())
			camera.is_stalled = False
			time.sleep(0.3)
			self.assertNotEqual(self.latest_observation(receiver)['cam_top'], '')
			stop.stop_host()
		camera_lines = [line for line in logs.output if '카메라 cam_top' in line]
		self.assertEqual(len(camera_lines), 2, logs.output)  # 늦음 1번 · 회복 1번

	def test_two_command_clients_hold_position(self) -> None:
		"""명령 client 가 둘이면 어느 쪽도 따르지 않고, 하나로 돌아오면 다시 따른다."""
		robot = self.connected_robot()
		context = zmq.Context()
		self.addCleanup(context.term)
		senders = []
		for pan in (10.0, -10.0):
			sender = context.socket(zmq.PUSH)
			self.addCleanup(sender.close, 0)
			senders.append((sender, pan))

		def send_for(seconds: float, pairs: list) -> None:
			end = time.monotonic() + seconds
			while time.monotonic() < end:
				for sender, pan in pairs:
					try:
						sender.send_string(json.dumps({
							**dict.fromkeys(FOURTEEN_KEYS, 0.0),
							'left_shoulder_pan.pos': pan}), zmq.NOBLOCK)
					except zmq.Again:
						pass
				time.sleep(0.02)

		def left_pans() -> list[float]:
			return [call[1]['shoulder_pan'] for call in
				self.buses['/dev/L'].calls if call[0] == 'goal']

		with self.assertLogs('lerobot_robot_dual_so102.host', 'INFO') as logs:
			stop = self.start_host(robot)
			for sender, _ in senders:
				sender.connect(f'tcp://127.0.0.1:{self.host_config.port_zmq_cmd}')
			time.sleep(0.3)  # 두 연결이 잡힐 때까지
			send_for(0.5, senders)
			self.assertEqual(left_pans(), [])
			senders[1][0].close(linger=0)
			time.sleep(0.3)
			send_for(0.5, senders[:1])
			stop.stop_host()
		self.assertIn(10.0, left_pans())
		self.assertNotIn(-10.0, left_pans())
		crowded = [line for line in logs.output
			if line.startswith('ERROR') and '명령 client' in line]
		self.assertEqual(len(crowded), 1, logs.output)

	def test_port_in_use_fails_before_robot_connect(self) -> None:
		"""기존 host 가 포트를 잡고 있으면 팔을 건드리기 전에 포트를 적은 오류."""
		context = zmq.Context()
		self.addCleanup(context.term)
		busy = context.socket(zmq.PULL)
		self.addCleanup(busy.close, 0)
		busy.bind(f'tcp://*:{self.host_config.port_zmq_cmd}')
		argv = ['dual-so102-host', '--robot.id=bi_so102',
			'--robot.left_arm_config.port=/dev/L',
			'--robot.right_arm_config.port=/dev/R',
			f'--host.port_zmq_cmd={self.host_config.port_zmq_cmd}',
			f'--host.port_zmq_observations='
			f'{self.host_config.port_zmq_observations}']
		with patch('sys.argv', argv):
			with self.assertRaisesRegex(RuntimeError,
				str(self.host_config.port_zmq_cmd)):
				main()
		self.assertEqual(self.buses, {})

	def test_cli_help(self) -> None:
		"""host 도움말에 미러 · 포트 옵션이 있다."""
		result = subprocess.run([sys.executable, '-m',
			'lerobot_robot_dual_so102.host', '--help'], cwd=self.root,
			capture_output=True, text=True, timeout=120)
		self.assertEqual(result.returncode, 0, result.stderr)
		self.assertIn('robot.mirror_mode', result.stdout)
		self.assertIn('host.port_zmq_cmd', result.stdout)


if __name__ == '__main__':
	unittest.main()
