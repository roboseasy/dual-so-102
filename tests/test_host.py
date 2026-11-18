"""Pi host(dual-so102-host) 시험 — 기존 host 동작 · 종료 정리 · 포트 사용 중."""

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
		return stop

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
		calibration = self.root / 'cal'
		write_calibration_file(calibration / 'bi_so102_left.json')
		write_calibration_file(calibration / 'bi_so102_right.json')
		for signum in (signal.SIGTERM, signal.SIGHUP):
			with self.subTest(signal=signum.name):
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
				process = subprocess.Popen([sys.executable, '-c', script],
					cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
					text=True)
				self.addCleanup(lambda p=process: p.poll() is None and p.kill())
				client = DualSO102Client(DualSO102ClientConfig(
					remote_ip='127.0.0.1', port_zmq_cmd=cmd_port,
					port_zmq_observations=obs_port, connect_timeout_s=30))
				client.connect()  # host 루프가 돈다
				client.disconnect()
				process.send_signal(signum)
				output, _ = process.communicate(timeout=30)
				self.assertEqual(process.returncode, 0, output)
				self.assertIn('BUS_DISCONNECTED /dev/L True', output)
				self.assertIn('BUS_DISCONNECTED /dev/R True', output)

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
