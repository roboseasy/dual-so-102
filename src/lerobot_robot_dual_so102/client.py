"""PC 에서 Pi 의 dual-so102-host 에 ZMQ 로 붙는 원격 양팔 (dual_so102_client).

전송 형식은 기존 양팔 host 와 같다 (wire.py). 기존 client 와 다른 점:
- 수신 스레드가 관측이 도착한 시각을 찍는다. host 관측이 stale_timeout_s
	이상 끊기면 관측 · 명령 모두 DeviceNotConnectedError 로 멈춘다
	(멈춘 화면을 녹화하거나 응답 없는 host 에 명령하지 않게).
- 연결할 때 설정한 카메라가 모두 오고 크기가 맞는지 확인한다.
- 빠진 프레임은 직전 프레임으로 메우고 검은 화면을 만들지 않는다.
	한 카메라가 stale_timeout_s 이상 안 오면 멈춘다.
"""

from functools import cached_property
import json
import logging
import math
import threading
import time
from typing import Final

from lerobot.robots import Robot
from lerobot.types import RobotAction
from lerobot.types import RobotObservation
from lerobot.utils.decorators import check_if_already_connected
from lerobot.utils.decorators import check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
import numpy as np
import zmq

from .arm import JOINT_NAMES
from .config import DualSO102ClientConfig
from .wire import decode_frame

# 연결할 때 첫 관측을 확인하는 간격
WAIT_POLL_S: Final[float] = 0.01
# 끊을 때 수신 스레드를 기다리는 시간
RECEIVER_JOIN_TIMEOUT_S: Final[float] = 1.0

logger = logging.getLogger(__name__)


class DualSO102Client(Robot):
	"""Pi 의 dual-so102-host 와 짝이 되는 원격 양팔.

	보정 · 모터 설정은 host 쪽 로봇이 하므로 여기서는 아무것도 하지 않는다.
	"""

	config_class = DualSO102ClientConfig
	name = 'dual_so102_client'

	def __init__(self, config: DualSO102ClientConfig) -> None:
		super().__init__(config)
		self.config = config
		# lerobot-record 가 카메라 수(len(robot.cameras))로 이미지 저장 스레드를 정한다.
		# 카메라는 Pi 의 host 가 열므로 이름만 둔다.
		self.cameras: dict[str, None] = dict.fromkeys(config.cameras)
		self._lock = threading.Lock()
		self._stop_event = threading.Event()
		self._receiver: threading.Thread | None = None
		self._context: zmq.Context | None = None
		self._cmd_socket: zmq.Socket | None = None
		self._observation_socket: zmq.Socket | None = None
		self._is_connected = False
		self._state: dict[str, float] = {}
		self._frames: dict[str, np.ndarray] = {}
		self._frame_times: dict[str, float] = {}
		self._last_receive_time: float | None = None
		self._error: Exception | None = None
		self._has_warned_drop = False
		self._has_warned_bad_message = False

	@cached_property
	def _state_ft(self) -> dict[str, type]:
		return {f'{side}_{joint}.pos': float
			for side in ('left', 'right') for joint in JOINT_NAMES}

	@cached_property
	def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
		return {name: (camera.height, camera.width, 3)
			for name, camera in self.config.cameras.items()}

	@cached_property
	def observation_features(self) -> dict[str, type | tuple]:
		return {**self._state_ft, **self._cameras_ft}

	@cached_property
	def action_features(self) -> dict[str, type]:
		return self._state_ft

	@property
	def is_connected(self) -> bool:
		return self._is_connected

	@property
	def is_calibrated(self) -> bool:
		return True

	def calibrate(self) -> None:
		"""host 쪽 로봇이 보정하므로 할 일이 없다."""

	def configure(self) -> None:
		"""host 쪽 로봇이 모터를 설정하므로 할 일이 없다."""

	@check_if_already_connected
	def connect(self, calibrate: bool = True) -> None:
		"""host 에 붙고 첫 관측으로 구성(카메라 이름 · 크기)을 확인한다.

		Args:
			calibrate: 공식 인터페이스 호환용 (client 는 보정하지 않는다).

		Raises:
			DeviceNotConnectedError: connect_timeout_s 안에 관측이 없거나,
				설정한 카메라를 host 가 보내지 않을 때.
			ValueError: 카메라 크기나 관절 키가 host 와 다를 때.
		"""
		with self._lock:
			self._state, self._frames, self._frame_times = {}, {}, {}
			self._last_receive_time = None
			self._error = None
		self._context = zmq.Context()
		self._cmd_socket = self._open_socket(zmq.PUSH,
			self.config.port_zmq_cmd, is_command=True)
		self._observation_socket = self._open_socket(zmq.PULL,
			self.config.port_zmq_observations)
		self._stop_event.clear()
		self._receiver = threading.Thread(target=self._receive_loop,
			name='dual_so102_client_rx', daemon=True)
		self._receiver.start()
		try:
			self._wait_for_first_observation()
		except BaseException:
			# 시간 초과뿐 아니라 기다리는 중 Ctrl+C 에도 스레드 · 소켓을 정리한다
			self._close()
			raise
		self._is_connected = True
		logger.info(f'{self} connected to dual-so102-host at '
			f'{self.config.remote_ip}.')

	def _open_socket(self, kind: int, port: int,
		is_command: bool = False) -> zmq.Socket:
		"""옵션을 건 뒤 host 에 연결한 소켓 (connect 뒤에 건 옵션은 안 먹을 수 있다)."""
		sock = self._context.socket(kind)
		sock.setsockopt(zmq.CONFLATE, 1)  # 가장 최근 메시지 하나만 둔다
		sock.setsockopt(zmq.LINGER, 0)  # 닫을 때 못 보낸 것을 기다리지 않는다
		if is_command:
			# host 와 연결됐을 때만 큐에 넣는다 → 없으면 zmq.Again 으로 버림을 안다
			sock.setsockopt(zmq.IMMEDIATE, 1)
		sock.connect(f'tcp://{self.config.remote_ip}:{port}')
		return sock

	def _wait_for_first_observation(self) -> None:
		"""설정한 카메라가 모두 든 첫 관측을 connect_timeout_s 동안 기다린다."""
		deadline = time.monotonic() + self.config.connect_timeout_s
		while True:
			with self._lock:
				error = self._error
				has_state = self._last_receive_time is not None
				missing = sorted(set(self._cameras_ft) - set(self._frames))
			if error is not None:
				raise error
			if has_state and not missing:
				return
			if time.monotonic() >= deadline:
				break
			time.sleep(WAIT_POLL_S)
		if not has_state:
			raise DeviceNotConnectedError(
				f'{self.config.remote_ip} 의 dual-so102-host 응답이 없습니다 '
				f'({self.config.connect_timeout_s}초). Pi 에서 host 가 돌고 '
				"'명령을 기다립니다' 가 찍혔는지, --robot.remote_ip · 포트가 맞는지 "
				'확인하세요.')
		raise DeviceNotConnectedError(
			f'host 가 카메라 {missing} 를 보내지 않습니다. host 와 client 의 '
			'--robot.cameras 이름을 맞추세요.')

	def _receive_loop(self) -> None:
		"""수신 스레드: 소켓을 늘 비우고 가장 최근 관측을 반영한다.

		관측 소켓은 이 스레드만 만진다. 메시지 하나가 잘못돼도 죽지 않는다.
		"""
		sock = self._observation_socket
		poller = zmq.Poller()
		poller.register(sock, zmq.POLLIN)
		while not self._stop_event.is_set():
			try:
				if sock not in dict(poller.poll(self.config.polling_timeout_ms)):
					continue
				latest: str | None = None
				while True:
					try:
						latest = sock.recv_string(zmq.NOBLOCK)
					except zmq.Again:
						break
				if latest is not None:
					self._apply_message(latest)
			except zmq.ZMQError as error:
				if not self._stop_event.is_set():
					logger.error(f'관측 소켓 오류: {error}')
				return
			except Exception as error:  # 메시지 하나 때문에 스레드가 죽지 않게
				if not self._has_warned_bad_message:
					self._has_warned_bad_message = True
					logger.error(f'host 관측을 읽지 못해 건너뜁니다: {error}')

	def _apply_message(self, message: str) -> None:
		"""관측 메시지 하나를 풀어 상태 · 프레임 · 도착 시각을 갱신한다."""
		received_at = time.monotonic()
		data = json.loads(message)
		if not isinstance(data, dict):
			raise ValueError('관측이 JSON 객체가 아닙니다')
		state: dict[str, float] = {}
		missing_keys = []
		for key in self._state_ft:
			value = data.get(key)
			if isinstance(value, bool) or not isinstance(value, (int, float)):
				missing_keys.append(key)
			else:
				state[key] = float(value)
		if missing_keys:
			self._set_error(ValueError(
				f'host 관측에 관절 값이 없습니다: {missing_keys} — Pi 에서 '
				'dual-so102-host(7축 양팔)를 쓰는지 확인하세요.'))
			return
		frames: dict[str, np.ndarray] = {}
		for name, shape in self._cameras_ft.items():
			frame = decode_frame(data.get(name))
			if frame is None:
				continue  # 빠진 프레임: 직전 프레임을 쓴다 (계속 빠지면 멈춘다)
			if frame.shape != shape:
				self._set_error(ValueError(
					f'카메라 {name}: host {frame.shape} != 설정 {shape} — '
					'client 의 --robot.cameras width · height 를 host 와 맞추세요.'))
				return
			frames[name] = frame
		with self._lock:
			self._state = state
			self._frames.update(frames)
			self._frame_times.update(dict.fromkeys(frames, received_at))
			self._last_receive_time = received_at

	def _set_error(self, error: Exception) -> None:
		"""다음 호출에서 낼 설정 오류를 남긴다 (처음 것만)."""
		with self._lock:
			if self._error is None:
				self._error = error

	def _raise_if_unusable(self) -> None:
		"""설정 오류가 있거나 관측 · 카메라가 stale_timeout_s 이상 끊겼으면 멈춘다."""
		with self._lock:
			error = self._error
			last_receive_time = self._last_receive_time
			frame_times = dict(self._frame_times)
		if error is not None:
			raise error
		now = time.monotonic()
		timeout_s = self.config.stale_timeout_s
		age_s = now - last_receive_time
		if age_s > timeout_s:
			raise DeviceNotConnectedError(
				f'host 관측이 {age_s:.1f}초째 오지 않습니다 '
				f'(stale_timeout_s={timeout_s}). host 가 멈췄거나 네트워크가 '
				'끊겼습니다 — 멈춘 화면을 녹화하지 않게 중단합니다.')
		for name in self._cameras_ft:
			frame_age_s = now - frame_times[name]
			if frame_age_s > timeout_s:
				raise DeviceNotConnectedError(
					f'카메라 {name} 프레임이 {frame_age_s:.1f}초째 오지 않습니다 '
					f'(stale_timeout_s={timeout_s}). Pi 의 카메라 연결을 확인하세요.')

	@check_if_not_connected
	def get_observation(self) -> RobotObservation:
		"""수신 스레드가 받아 둔 가장 최근 관측 (14 관절 + 카메라).

		Raises:
			DeviceNotConnectedError: host 관측이나 카메라가 끊겼을 때.
			ValueError: host 와 설정이 다를 때.
		"""
		self._raise_if_unusable()
		with self._lock:
			observation: RobotObservation = dict(self._state)
			for name in self._cameras_ft:
				observation[name] = self._frames[name]
		return observation

	@check_if_not_connected
	def send_action(self, action: RobotAction) -> RobotAction:
		"""14 관절 목표를 host 로 보낸다 (다른 키 · NaN · inf 는 뺀다).

		Args:
			action: '<left|right>_<관절>.pos' → 목표 위치.

		Returns:
			보낸 명령.

		Raises:
			DeviceNotConnectedError: host 관측이 끊겼을 때.
		"""
		self._raise_if_unusable()
		sent: dict[str, float] = {}
		for key in self._state_ft:
			if key in action:
				value = float(action[key])
				if math.isfinite(value):
					sent[key] = value
		try:
			self._cmd_socket.send_string(json.dumps(sent), zmq.NOBLOCK)
			if self._has_warned_drop:
				self._has_warned_drop = False
				logger.info('host 로 명령이 다시 갑니다.')
		except zmq.Again:
			if not self._has_warned_drop:
				self._has_warned_drop = True
				logger.warning('host 와 연결이 없어 명령을 버렸습니다.')
		return sent

	@check_if_not_connected
	def disconnect(self) -> None:
		"""소켓을 닫는다 (Pi 의 팔은 host 가 계속 잡고 있다)."""
		self._close()
		self._is_connected = False
		logger.info(f'{self} disconnected.')

	def _close(self) -> None:
		"""수신 스레드를 멈춘 뒤 소켓 · 컨텍스트를 닫는다.

		ZMQ 소켓은 스레드 안전하지 않아 스레드가 끝난 뒤 닫는다. 스레드가
		제때 안 끝나면 관측 소켓 · 컨텍스트는 그대로 둔다 (다른 스레드의
		소켓을 닫는 것보다 새는 편이 낫다).
		"""
		self._stop_event.set()
		is_receiver_alive = False
		if self._receiver is not None:
			self._receiver.join(timeout=RECEIVER_JOIN_TIMEOUT_S)
			is_receiver_alive = self._receiver.is_alive()
			self._receiver = None
		if self._cmd_socket is not None:
			self._cmd_socket.close(linger=0)
			self._cmd_socket = None
		if is_receiver_alive:
			logger.warning('수신 스레드가 제때 끝나지 않아 관측 소켓을 그대로 둡니다.')
			return
		if self._observation_socket is not None:
			self._observation_socket.close(linger=0)
			self._observation_socket = None
		if self._context is not None:
			self._context.term()
			self._context = None
