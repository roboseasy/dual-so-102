"""PC client(dual_so102_client) 시험 — 기존 전송 형식, 관측 끊김, 카메라 검증."""

from contextlib import redirect_stdout
import importlib.util
import io
import json
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

from lerobot.utils.errors import DeviceNotConnectedError
import numpy as np
import zmq

from lerobot_robot_dual_so102 import DualSO102Client
from lerobot_robot_dual_so102 import DualSO102ClientConfig
from lerobot_robot_dual_so102.wire import decode_frame
from lerobot_robot_dual_so102.wire import encode_observation
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
# 녹화(lerobot-record)는 dataset extra 가 있어야 한다 (Pi 기본 설치에는 없다)
HAS_DATASETS = importlib.util.find_spec('datasets') is not None


def free_port() -> int:
	"""지금 비어 있는 TCP 포트."""
	with socket.socket() as probe:
		probe.bind(('127.0.0.1', 0))
		return probe.getsockname()[1]


def joints(left_pan: float = 12.5) -> dict[str, float]:
	"""14 관절값 (left_shoulder_pan 만 따로, 나머지는 키마다 다른 값)."""
	values = {key: float(index) for index, key in enumerate(sorted(FOURTEEN_KEYS))}
	values['left_shoulder_pan.pos'] = left_pan
	return values


class FakeHost:
	"""dual-so102-host 처럼 관측을 PUSH 하고 명령을 PULL 하는 가짜 host.

	Attributes:
		message: 보낼 관측 (이미 전송 형식으로 인코딩된 dict).
	"""

	def __init__(self, test: unittest.TestCase) -> None:
		self.context = zmq.Context()
		self.cmd_port = free_port()
		self.obs_port = free_port()
		self.observations = self.context.socket(zmq.PUSH)
		self.observations.setsockopt(zmq.CONFLATE, 1)
		self.observations.setsockopt(zmq.LINGER, 0)
		self.observations.bind(f'tcp://127.0.0.1:{self.obs_port}')
		self.commands = self.context.socket(zmq.PULL)
		self.commands.setsockopt(zmq.LINGER, 0)
		self.commands.bind(f'tcp://127.0.0.1:{self.cmd_port}')
		self.message: dict = {}
		self._stop = threading.Event()
		self._thread: threading.Thread | None = None
		self.is_closed = False
		test.addCleanup(self.close)

	def keep_sending(self, period_s: float = 0.02) -> None:
		"""self.message 를 주기적으로 보낸다 (받는 client 가 없으면 버린다)."""
		def loop() -> None:
			while not self._stop.is_set():
				try:
					self.observations.send_string(json.dumps(self.message),
						zmq.NOBLOCK)
				except zmq.Again:
					pass
				time.sleep(period_s)
		self._thread = threading.Thread(target=loop, daemon=True)
		self._thread.start()

	def stop_sending(self) -> None:
		"""보내기를 멈춘다 (host 가 멈춘 것처럼)."""
		self._stop.set()
		if self._thread is not None:
			self._thread.join()

	def receive_command(self, timeout_s: float = 2.0) -> dict | None:
		"""client 가 보낸 명령 하나."""
		if self.commands.poll(int(timeout_s * 1000)):
			return json.loads(self.commands.recv_string())
		return None

	def close(self) -> None:
		"""소켓을 닫는다 (host 가 꺼진 것처럼)."""
		if self.is_closed:
			return
		self.is_closed = True
		self.stop_sending()
		self.observations.close(linger=0)
		self.commands.close(linger=0)
		self.context.term()


class ClientTest(unittest.TestCase):
	"""기존 host 와 같은 형식으로 주고받고, 끊김 · 설정 불일치에서 멈추는지 본다."""

	def setUp(self) -> None:
		self.root = use_temp_calibration_root(self)

	def make_client(self, host: FakeHost | None, **kwargs: object,
		) -> DualSO102Client:
		"""가짜 host 에 붙는 client (연결은 시험이 한다)."""
		options = {'remote_ip': '127.0.0.1', 'connect_timeout_s': 3.0,
			'stale_timeout_s': 0.2,
			'cameras': {'cam_top': camera(480, 640)}}
		if host is not None:
			options.update(port_zmq_cmd=host.cmd_port,
				port_zmq_observations=host.obs_port)
		options.update(kwargs)
		client = DualSO102Client(DualSO102ClientConfig(**options))
		self.addCleanup(lambda: client.disconnect()
			if client.is_connected else None)
		return client

	def alive_host(self, frame_shape: tuple[int, int] | None = (480, 640),
		) -> FakeHost:
		"""14 관절 + cam_top 을 계속 보내는 host."""
		host = FakeHost(self)
		observation: dict = joints()
		if frame_shape is not None:
			observation['cam_top'] = np.full((*frame_shape, 3), 200, np.uint8)
		host.message = encode_observation(observation)
		host.keep_sending()
		return host

	def test_wire_roundtrip(self) -> None:
		"""관절은 float, 프레임은 shape 유지, 빈 · 깨진 프레임은 None."""
		frame = np.full((480, 640, 3), 120, np.uint8)
		encoded = encode_observation(
			{'left_gripper.pos': np.float32(40.5), 'cam_top': frame})
		self.assertEqual(json.loads(json.dumps(encoded))['left_gripper.pos'],
			40.5)
		self.assertIsInstance(encoded['cam_top'], str)
		decoded = decode_frame(encoded['cam_top'])
		self.assertEqual(decoded.shape, (480, 640, 3))
		self.assertLess(abs(int(decoded.mean()) - 120), 3)
		for broken in ('', 'not base64!', 'AAAA', 7.0):
			with self.subTest(broken=broken):
				self.assertIsNone(decode_frame(broken))
		self.assertEqual(encode_observation({'cam_top': None})['cam_top'], '')

	def test_connect_times_out_without_host(self) -> None:
		"""host 가 없으면 connect_timeout_s 뒤 주소를 적은 연결 오류."""
		client = self.make_client(None, connect_timeout_s=0.3,
			port_zmq_cmd=free_port(), port_zmq_observations=free_port())
		start = time.monotonic()
		with self.assertRaisesRegex(DeviceNotConnectedError, '127.0.0.1'):
			client.connect()
		self.assertLess(time.monotonic() - start, 1.5)
		self.assertFalse(client.is_connected)

	def test_default_waits_for_slow_host_start(self) -> None:
		"""기본 설정 client 는 Pi host 가 카메라를 여는 동안 기다렸다가 붙는다.

		실장비(2026-11-18): Pi host 는 카메라 3대를 여는 데 약 16초 걸린다. host 를 띄우자마자
		PC 텔레옵을 켜면 예전 기본값 5초 안에 준비되지 않아 '응답이 없습니다' 로 끝났다.
		"""
		host = FakeHost(self)
		observation: dict = joints()
		observation['cam_top'] = np.full((480, 640, 3), 200, np.uint8)
		host.message = encode_observation(observation)
		late_start = threading.Timer(6.0, host.keep_sending)
		late_start.start()
		self.addCleanup(late_start.cancel)
		client = DualSO102Client(DualSO102ClientConfig(remote_ip='127.0.0.1',
			port_zmq_cmd=host.cmd_port, port_zmq_observations=host.obs_port,
			cameras={'cam_top': camera(480, 640)}))
		self.addCleanup(lambda: client.disconnect()
			if client.is_connected else None)
		client.connect()
		self.assertIn('cam_top', client.get_observation())

	def test_roundtrip_state_camera_and_action(self) -> None:
		"""관측 14키 + cam_top, 명령은 14키(NaN 제외)만 host 에 간다."""
		host = self.alive_host()
		client = self.make_client(host)
		client.connect()
		observation = client.get_observation()
		self.assertEqual(set(observation), FOURTEEN_KEYS | {'cam_top'})
		self.assertEqual(observation['left_shoulder_pan.pos'], 12.5)
		self.assertEqual(observation['cam_top'].shape, (480, 640, 3))
		action = {**dict.fromkeys(FOURTEEN_KEYS, 1.5), 'left_foo': 9.0,
			'right_gripper.pos': float('nan')}
		sent = client.send_action(action)
		received = host.receive_command()
		self.assertEqual(set(received), FOURTEEN_KEYS - {'right_gripper.pos'})
		self.assertEqual(received, sent)

	def test_missing_camera_at_connect(self) -> None:
		"""설정한 카메라를 host 가 안 보내면 그 이름을 적은 연결 오류."""
		host = self.alive_host(frame_shape=None)
		client = self.make_client(host, connect_timeout_s=0.5)
		with self.assertRaisesRegex(DeviceNotConnectedError, 'cam_top'):
			client.connect()
		self.assertFalse(client.is_connected)

	def test_camera_shape_mismatch(self) -> None:
		"""host 프레임 크기가 설정과 다르면 이름 · 두 크기를 적은 ValueError."""
		host = self.alive_host(frame_shape=(240, 320))
		client = self.make_client(host)
		with self.assertRaisesRegex(ValueError, 'cam_top'):
			client.connect()
		self.assertFalse(client.is_connected)

	def test_stale_host_raises(self) -> None:
		"""host 가 멈추면 stale_timeout_s 뒤 관측 · 명령 모두 연결 오류."""
		host = self.alive_host()
		client = self.make_client(host)
		client.connect()
		host.stop_sending()
		time.sleep(0.4)
		with self.assertRaises(DeviceNotConnectedError):
			client.get_observation()
		with self.assertRaises(DeviceNotConnectedError):
			client.send_action(dict.fromkeys(FOURTEEN_KEYS, 0.0))

	def test_pause_longer_than_stale_is_ok_when_host_alive(self) -> None:
		"""에피소드 저장처럼 루프만 오래 멈춰도 host 가 살아 있으면 괜찮다."""
		host = self.alive_host()
		client = self.make_client(host)
		client.connect()
		time.sleep(0.5)
		self.assertIn('cam_top', client.get_observation())
		client.send_action(dict.fromkeys(FOURTEEN_KEYS, 0.0))

	def test_host_died_during_pause_raises(self) -> None:
		"""루프가 멈춘 사이 host 가 죽었으면, 멈추기 전 메시지를 새것으로 쓰지 않는다."""
		host = self.alive_host()
		client = self.make_client(host)
		client.connect()
		client.get_observation()
		time.sleep(0.1)  # 루프는 멈췄고 host 는 아직 보낸다 (대기열에 하나 남는다)
		host.stop_sending()
		time.sleep(0.5)
		with self.assertRaises(DeviceNotConnectedError):
			client.get_observation()

	def test_dropped_frames_reuse_previous_then_raise(self) -> None:
		"""빈 프레임("")은 직전 프레임으로 메우고(검은 화면 X), 계속 빠지면 오류."""
		host = self.alive_host()
		client = self.make_client(host)
		client.connect()
		host.message = {**host.message, 'cam_top': ''}
		time.sleep(0.05)
		frame = client.get_observation()['cam_top']
		self.assertGreater(int(frame.mean()), 150)
		time.sleep(0.4)
		with self.assertRaisesRegex(DeviceNotConnectedError, 'cam_top'):
			client.get_observation()

	def test_disconnect_does_not_hang_without_host(self) -> None:
		"""host 가 사라진 뒤에도 disconnect 가 바로 끝난다."""
		host = self.alive_host()
		client = self.make_client(host)
		client.connect()
		host.close()
		client.send_action(dict.fromkeys(FOURTEEN_KEYS, 0.0))
		closer = threading.Thread(target=client.disconnect, daemon=True)
		closer.start()
		closer.join(timeout=2.0)
		self.assertFalse(closer.is_alive())
		self.assertFalse(client.is_connected)

	@unittest.skipUnless(HAS_DATASETS, 'dataset extra 없음 (Pi 기본 설치)')
	def test_record_one_episode_through_client(self) -> None:
		"""공식 lerobot-record 가 client 로 한 에피소드를 녹화한다 (14키 + 카메라)."""
		from lerobot.scripts import lerobot_record

		host = self.alive_host(frame_shape=(48, 64))
		patch_make_bus(self)
		calibration = self.root / 'teleop'
		for side in ('left', 'right'):
			write_calibration_file(calibration / f'bi_so102_leader_{side}.json')
		dataset_root = self.root / 'dataset'
		cameras = json.dumps({'cam_top': {'type': 'opencv', 'index_or_path': 0,
			'width': 64, 'height': 48, 'fps': 30}})
		argv = [
			'lerobot-record', '--robot.type=dual_so102_client',
			'--robot.remote_ip=127.0.0.1',
			f'--robot.port_zmq_cmd={host.cmd_port}',
			f'--robot.port_zmq_observations={host.obs_port}',
			f'--robot.cameras={cameras}',
			'--teleop.type=dual_so102_leader', '--teleop.id=bi_so102_leader',
			'--teleop.left_arm_config.port=/dev/so102_leader_left',
			'--teleop.right_arm_config.port=/dev/so102_leader_right',
			f'--teleop.calibration_dir={calibration}',
			'--dataset.repo_id=local/dual_so102_test',
			f'--dataset.root={dataset_root}', '--dataset.single_task=test',
			'--dataset.num_episodes=1', '--dataset.episode_time_s=1',
			'--dataset.reset_time_s=0', '--dataset.push_to_hub=false',
			'--display_data=false', '--play_sounds=false',
		]
		# 시험 중 데스크톱 키 입력(→ · Esc)이 녹화를 끊지 않게 키보드 감시를 뺀다
		events = {'exit_early': False, 'rerecord_episode': False,
			'stop_recording': False}
		with (patch('sys.argv', argv),
			patch.object(lerobot_record, 'init_keyboard_listener',
				return_value=(None, events)),
			redirect_stdout(io.StringIO())):
			lerobot_record.main()
		info = json.loads((dataset_root / 'meta' / 'info.json').read_text())
		self.assertEqual(info['robot_type'], 'dual_so102_client')
		self.assertGreater(info['total_frames'], 0)
		self.assertEqual(set(info['features']['observation.state']['names']),
			FOURTEEN_KEYS)
		self.assertIn('observation.images.cam_top', info['features'])
		self.assertIsNotNone(host.receive_command())

	@unittest.skipUnless(HAS_DATASETS, 'dataset extra 없음 (Pi 기본 설치)')
	def test_record_help(self) -> None:
		"""공식 lerobot-record 도움말이 client 타입과 remote_ip 를 보여 준다."""
		result = subprocess.run([sys.executable, '-m',
			'lerobot.scripts.lerobot_record', '--robot.type=dual_so102_client',
			'--help'], cwd=self.root, capture_output=True, text=True,
			timeout=120)
		self.assertEqual(result.returncode, 0, result.stderr)
		self.assertIn('remote_ip', result.stdout)


if __name__ == '__main__':
	unittest.main()
