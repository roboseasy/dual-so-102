"""Pi 에서 도는 dual-so102-host — 팔로워 양팔 · 카메라를 잡고 PC client 와 ZMQ 로 주고받는다.

동작은 기존 양팔 host 와 같다 (전송 형식 · watchdog · 연속 실패 종료).
더한 것:
- 포트를 먼저 잡는다. 기존 host 가 아직 돌고 있으면 팔을 건드리기 전에 멈춘다.
- SIGTERM · SIGHUP(SSH 끊김)도 Ctrl+C 처럼 정리(버스 · 카메라 해제, 토크 해제)한다.

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
from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401 (--robot.cameras 의 opencv 타입 등록)
from lerobot.utils.utils import init_logging
import zmq

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

	Attributes:
		commands: client 명령을 받는 소켓.
		observations: client 로 관측을 보내는 소켓.
	"""

	def __init__(self, cfg: DualSO102HostConfig) -> None:
		self._context = zmq.Context()
		self.commands: zmq.Socket | None = None
		self.observations: zmq.Socket | None = None
		try:
			self.commands = self._bind(zmq.PULL, cfg.port_zmq_cmd)
			self.observations = self._bind(zmq.PUSH, cfg.port_zmq_observations)
		except BaseException:
			self.close()
			raise

	def _bind(self, kind: int, port: int) -> zmq.Socket:
		"""포트를 잡는다. 이미 쓰는 중이면 알아볼 수 있는 오류를 낸다."""
		sock = self._context.socket(kind)
		sock.setsockopt(zmq.CONFLATE, 1)
		sock.setsockopt(zmq.LINGER, 0)
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
	- 받는 client 가 없으면 관측을 버리고, 붙으면 다시 보낸다.

	Args:
		robot: 연결된 양팔.
		cfg: host 설정.
		should_stop: True 를 돌려주면 루프를 끝낸다 (시험용).
		sockets: 이미 잡은 소켓. 없으면 여기서 잡고 닫는다.

	Raises:
		ConnectionError: 관측 읽기가 연속으로 실패했을 때.
		RuntimeError: 포트가 사용 중일 때.
	"""
	own_sockets = sockets is None
	if own_sockets:
		sockets = HostSockets(cfg)
	try:
		_serve(robot, cfg, sockets, should_stop)
	finally:
		if own_sockets:
			sockets.close()


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
	read_failures = 0
	logger.info(f'명령을 기다립니다 (명령 포트 {cfg.port_zmq_cmd}, '
		f'관측 포트 {cfg.port_zmq_observations}).')
	while not should_stop():
		loop_start = time.monotonic()
		if (cfg.connection_time_s > 0
				and loop_start - start >= cfg.connection_time_s):
			logger.info('connection_time_s 가 지나 host 를 끝냅니다.')
			return

		try:
			message = sockets.commands.recv_string(zmq.NOBLOCK)
		except zmq.Again:
			message = None
		if message is not None:
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

		if not is_holding and time.monotonic() - last_command_time > watchdog_s:
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
