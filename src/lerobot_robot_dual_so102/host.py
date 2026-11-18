"""Pi 에서 도는 dual-so102-host — 팔로워 양팔 · 카메라를 잡고 PC client 와 ZMQ 로 주고받는다.

동작은 기존 양팔 host 와 같다 (전송 형식 · watchdog · 연속 실패 종료).
더한 것:
- 포트를 먼저 잡는다. 기존 host 가 아직 돌고 있으면 팔을 건드리기 전에 멈춘다.
- SIGTERM · SIGHUP(SSH 끊김)도 Ctrl+C 처럼 정리(버스 · 카메라 해제, 토크 해제)한다.
	nohup 처럼 SIGHUP 을 무시하게 띄웠으면 그대로 둔다.
- 카메라 프레임이 늦으면 그 카메라만 빈 프레임으로 보낸다 (host 는 계속 돈다).
- 명령 client 가 둘 이상 붙으면 명령을 따르지 않고 자세를 유지한다.

	dual-so102-host --robot.id=bi_so102 \\
		--robot.left_arm_config.port=/dev/so102_follower_left \\
		--robot.right_arm_config.port=/dev/so102_follower_right
"""

from collections.abc import Callable
from dataclasses import dataclass
from dataclasses import field
import json
import logging
import signal
import time
from types import FrameType
from typing import Final

import draccus
from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401 (opencv 타입 등록)
from lerobot.utils.utils import init_logging
import numpy as np
import zmq
from zmq.utils.monitor import recv_monitor_message

from .config import DualSO102FollowerConfig
from .config import DualSO102HostConfig
from .dual import DualSO102Follower
from .wire import encode_observation

# 관측 읽기가 이만큼 연속 실패하면 host 를 끝낸다 (순간 끊김은 건너뛴다)
MAX_CONSECUTIVE_READ_FAILURES: Final[int] = 5

logger = logging.getLogger(__name__)


@dataclass
class DualSO102ServerConfig:
	"""dual-so102-host 의 CLI 설정 (--robot.* · --host.*)."""

	robot: DualSO102FollowerConfig
	host: DualSO102HostConfig = field(default_factory=DualSO102HostConfig)


class HostSockets:
	"""host 의 ZMQ 소켓 두 개 — 명령 PULL · 관측 PUSH (둘 다 CONFLATE).

	명령 소켓에 붙은 client 수를 센다. PULL 은 보낸 쪽을 구분하지 못해, client 가
	둘이면 두 리더의 명령이 번갈아 들어와 팔이 흔들린다.

	Attributes:
		commands: client 명령을 받는 소켓.
		observations: client 로 관측을 보내는 소켓.
	"""

	def __init__(self, cfg: DualSO102HostConfig) -> None:
		self._context = zmq.Context()
		self.commands: zmq.Socket | None = None
		self.observations: zmq.Socket | None = None
		self._peer_monitor: zmq.Socket | None = None
		self._command_peers = 0
		try:
			self.commands = self._bind(zmq.PULL, cfg.port_zmq_cmd)
			self.observations = self._bind(zmq.PUSH, cfg.port_zmq_observations)
			self._peer_monitor = self.commands.get_monitor_socket(
				zmq.EVENT_ACCEPTED | zmq.EVENT_DISCONNECTED)
		except BaseException:
			self.close()
			raise

	def count_command_peers(self) -> int:
		"""지금 명령 소켓에 붙어 있는 client 수 (연결 · 끊김 알림을 반영)."""
		while True:
			try:
				event = recv_monitor_message(self._peer_monitor, zmq.NOBLOCK)
			except zmq.Again:
				return self._command_peers
			if event['event'] == zmq.EVENT_ACCEPTED:
				self._command_peers += 1
			elif event['event'] == zmq.EVENT_DISCONNECTED:
				self._command_peers = max(self._command_peers - 1, 0)

	def _bind(self, kind: int, port: int) -> zmq.Socket:
		"""포트를 잡는다. 이미 쓰는 중이면 알아볼 수 있는 오류를 낸다."""
		sock = self._context.socket(kind)
		sock.setsockopt(zmq.CONFLATE, 1)
		sock.setsockopt(zmq.LINGER, 0)
		# 인사 없이 사라진 client(와이파이 끊김)를 몇 초 안에 끊긴 것으로 본다
		sock.setsockopt(zmq.TCP_KEEPALIVE, 1)
		sock.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 2)
		sock.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
		sock.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
		try:
			sock.bind(f'tcp://*:{port}')
		except zmq.ZMQError as error:
			sock.close(linger=0)
			if error.errno == zmq.EADDRINUSE:
				raise RuntimeError(
					f'포트 {port} 사용 중 — 다른 host(기존 bi_so102_host 등)를 '
					'먼저 끄세요.') from error
			raise
		return sock

	def close(self) -> None:
		"""소켓 · 컨텍스트를 닫는다."""
		if self._peer_monitor is not None:
			self.commands.disable_monitor()
			self._peer_monitor.close(linger=0)
			self._peer_monitor = None
		for sock in (self.commands, self.observations):
			if sock is not None:
				sock.close(linger=0)
		self.commands = self.observations = None
		self._context.term()

	def __enter__(self) -> 'HostSockets':
		return self

	def __exit__(self, *exc_info: object) -> None:
		self.close()


def run_host(
	robot: DualSO102Follower,
	cfg: DualSO102HostConfig,
	should_stop: Callable[[], bool] = lambda: False,
	sockets: HostSockets | None = None,
) -> None:
	"""연결된 양팔로 host 루프를 돈다 (로봇 연결 · 해제는 부르는 쪽이 한다).

	- 명령이 오면 send_action. watchdog_timeout_ms 동안 없으면 경고를 한 번
	  남기고 새 목표를 쓰지 않는다 (마지막 자세 유지).
	- 관측 읽기 실패(ConnectionError)는 그 주기만 건너뛰고, 연속
	  MAX_CONSECUTIVE_READ_FAILURES 번이면 오류를 남기고 다시 던진다.
	- 카메라 프레임이 늦으면(TimeoutError) 그 카메라만 빈 프레임으로 보낸다.
	- 받는 client 가 없으면 관측을 버리고, 붙으면 다시 보낸다.
	- 명령 client 가 둘 이상이면 명령을 따르지 않고 자세를 유지한다.

	Args:
		robot: 연결된 양팔.
		cfg: host 설정.
		should_stop: True 를 돌려주면 루프를 끝낸다 (시험용).
		sockets: 이미 잡은 소켓. 없으면 여기서 잡고 닫는다.

	Raises:
		ConnectionError: 관측 읽기가 연속으로 실패했을 때.
		RuntimeError: 포트가 사용 중일 때.
	"""
	is_own_sockets = sockets is None
	if is_own_sockets:
		sockets = HostSockets(cfg)
	_tolerate_camera_stalls(robot)
	try:
		_serve(robot, cfg, sockets, should_stop)
	finally:
		if is_own_sockets:
			sockets.close()


def _tolerate_camera_stalls(robot: DualSO102Follower) -> None:
	"""카메라 프레임이 늦으면(TimeoutError) 그 카메라만 None(빈 프레임)으로 읽게 한다.

	카메라 하나가 잠깐 늦었다고 host 가 끝나 두 팔 토크가 풀리면 안 된다.
	client 는 빈 프레임을 직전 프레임으로 메우고, 계속 오지 않으면
	stale_timeout_s 뒤 멈춘다 (그동안 host 는 자세를 유지한다).
	카메라 스레드가 죽는 등 다른 오류는 그대로 낸다.

	Args:
		robot: 카메라를 연 양팔 (같은 카메라 객체를 팔이 읽는다).
	"""
	stalled: set[str] = set()
	for name, camera in robot.cameras.items():
		if getattr(camera.read_latest, 'is_stall_tolerant', False):
			continue

		def read_or_none(*args: object, _read: Callable = camera.read_latest,
			_name: str = name, **kwargs: object) -> np.ndarray | None:
			try:
				frame = _read(*args, **kwargs)
			except TimeoutError as error:
				if _name not in stalled:
					stalled.add(_name)
					logger.warning(f'카메라 {_name} 프레임이 늦어 빈 프레임으로 '
						f'보냅니다: {error}')
				return None
			if _name in stalled:
				stalled.discard(_name)
				logger.info(f'카메라 {_name} 프레임이 다시 옵니다.')
			return frame
		read_or_none.is_stall_tolerant = True
		camera.read_latest = read_or_none


def _serve(
	robot: DualSO102Follower,
	cfg: DualSO102HostConfig,
	sockets: HostSockets,
	should_stop: Callable[[], bool],
) -> None:
	"""host 루프 본체 (run_host 참고)."""
	period_s = 1 / cfg.max_loop_freq_hz
	watchdog_s = cfg.watchdog_timeout_ms / 1000
	start = time.monotonic()
	last_command_time = start
	is_holding = False
	is_client_connected = True
	is_crowded = False
	read_failures = 0
	logger.info(f'명령을 기다립니다 (명령 포트 {cfg.port_zmq_cmd}, '
		f'관측 포트 {cfg.port_zmq_observations}).')
	while not should_stop():
		loop_start = time.monotonic()
		if (cfg.connection_time_s > 0
				and loop_start - start >= cfg.connection_time_s):
			logger.info('connection_time_s 가 지나 host 를 끝냅니다.')
			return

		peers = sockets.count_command_peers()
		if peers > 1 and not is_crowded:
			is_crowded = True
			logger.error(f'명령 client 가 {peers}개 붙었습니다 — 하나만 남기세요. '
				'그때까지 명령을 따르지 않고 자세를 유지합니다.')
		elif peers <= 1 and is_crowded:
			is_crowded = False
			logger.info('명령 client 가 하나로 돌아와 명령을 다시 따릅니다.')

		try:
			message = sockets.commands.recv_string(zmq.NOBLOCK)
		except zmq.Again:
			message = None
		if message is not None and not is_crowded:
			try:
				action = json.loads(message)
				if not isinstance(action, dict):
					raise ValueError('명령이 JSON 객체가 아닙니다')
				robot.send_action(action)
				last_command_time = time.monotonic()
				if is_holding:
					is_holding = False
					logger.info('명령이 다시 옵니다.')
			except Exception as error:
				logger.error(f'명령 처리 실패: {error}')

		if (not is_holding and not is_crowded
				and time.monotonic() - last_command_time > watchdog_s):
			is_holding = True
			logger.warning(f'명령이 {cfg.watchdog_timeout_ms} ms 넘게 오지 않아 '
				'자세를 유지합니다.')

		observation = None
		try:
			observation = robot.get_observation()
			read_failures = 0
		except ConnectionError as error:
			read_failures += 1
			if read_failures >= MAX_CONSECUTIVE_READ_FAILURES:
				logger.error(f'관측 읽기가 {read_failures}번 연속 실패해 host 를 '
					f'끝냅니다: {error}')
				raise
			logger.warning(f'관측 읽기 실패 ({read_failures}/'
				f'{MAX_CONSECUTIVE_READ_FAILURES}), 이번 주기는 건너뜁니다: {error}')

		if observation is not None:
			encoded = json.dumps(encode_observation(observation, cfg.jpeg_quality))
			try:
				sockets.observations.send_string(encoded, zmq.NOBLOCK)
				if not is_client_connected:
					is_client_connected = True
					logger.info('client 가 붙어 관측을 보냅니다.')
			except zmq.Again:
				if is_client_connected:
					is_client_connected = False
					logger.warning('받는 client 가 없어 관측을 버립니다 '
						'(client 가 붙으면 다시 보냅니다).')

		time.sleep(max(period_s - (time.monotonic() - loop_start), 0))


def _stop_on_signal(signum: int, frame: FrameType | None) -> None:
	"""SIGTERM · SIGHUP 을 Ctrl+C 처럼 바꾼다 (정리하는 동안 같은 신호는 무시)."""
	signal.signal(signal.SIGTERM, signal.SIG_IGN)
	signal.signal(signal.SIGHUP, signal.SIG_IGN)
	raise KeyboardInterrupt(signal.Signals(signum).name)


@draccus.wrap()
def main(cfg: DualSO102ServerConfig) -> None:
	"""dual-so102-host 진입점: 포트 → 양팔 연결 → 루프 → 해제.

	Args:
		cfg: CLI 에서 읽은 설정.

	Raises:
		RuntimeError: 포트가 사용 중일 때 (팔은 건드리지 않는다).
	"""
	init_logging()
	signal.signal(signal.SIGTERM, _stop_on_signal)
	# nohup 처럼 SIGHUP 을 무시하게 띄웠으면 SSH 가 끊겨도 계속 돌게 둔다
	if signal.getsignal(signal.SIGHUP) is not signal.SIG_IGN:
		signal.signal(signal.SIGHUP, _stop_on_signal)
	# 포트부터 잡는다: 다른 host 가 돌고 있으면 팔(같은 시리얼 포트)을 건드리기 전에 멈춘다
	with HostSockets(cfg.host) as sockets:
		robot = DualSO102Follower(cfg.robot)
		logger.info(f'{robot} 연결 중')
		robot.connect()
		try:
			run_host(robot, cfg.host, sockets=sockets)
		except KeyboardInterrupt as interrupt:
			logger.info(f'종료 신호({interrupt or "Ctrl+C"})를 받아 정리합니다.')
		finally:
			robot.disconnect()
	logger.info('dual-so102-host 를 끝냈습니다.')


if __name__ == '__main__':
	main()
