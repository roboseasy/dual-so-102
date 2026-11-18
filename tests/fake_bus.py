"""시험 공용 도구 — 시리얼을 열지 않는 7축 가짜 버스와 임시 보정 폴더.

`lerobot_robot_dual_so102.arm.make_bus` 를 `FakeBus` 로 바꿔 끼우면
공식 SOFollower · SOLeader 의 연결 · 보정 · 제어 코드가 그대로 돈다.
"""

from collections.abc import Iterator
from contextlib import contextmanager
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.motors import Motor, MotorCalibration, MotorNormMode

JOINTS = (
	'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
	'wrist_yaw', 'wrist_roll', 'gripper',
)


class FakeBus:
	"""`make_bus` 와 같은 시그니처의 가짜 7축 버스.

	Attributes:
		values: 정규화된 현재 위치 (sync_read 가 돌려주는 값).
		calls: 버스에 한 일의 기록. 목표 쓰기는 ('goal', 값, normalize).
		fail_reads: 남은 횟수만큼 sync_read 가 ConnectionError 를 낸다.
		setup_order: setup_motor 가 불린 관절 순서.
	"""

	def __init__(
		self,
		port: str,
		use_degrees: bool,
		calibration: dict[str, MotorCalibration],
		is_mirror: bool = False,
	) -> None:
		body_mode = (MotorNormMode.DEGREES if use_degrees
			else MotorNormMode.RANGE_M100_100)
		self.motors = {
			name: Motor(index, 'sts3215',
				MotorNormMode.RANGE_0_100 if name == 'gripper' else body_mode)
			for index, name in enumerate(JOINTS, 1)
		}
		self.port = port
		self.calibration = calibration
		self.is_mirror = is_mirror
		self.is_connected = False
		self.calls: list[tuple] = []
		self.fail_reads = 0
		self.setup_order: list[str] = []
		self.values = {name: 50.0 if name == 'gripper' else 0.0
			for name in JOINTS}
		if 'leader' in port:
			# 리더 값이 팔로워로 넘어갔는지 구별하는 표식
			self.values['wrist_yaw'] = 1.0

	@property
	def is_calibrated(self) -> bool:
		"""7관절 보정값이 모두 있는지."""
		return set(self.calibration) == set(JOINTS)

	def connect(self, handshake: bool = True) -> None:
		"""가상 연결."""
		self.calls.append(('connect',))
		self.is_connected = True

	def disconnect(self, disable_torque: bool = True) -> None:
		"""가상 해제 (토크 해제 여부 기록)."""
		self.calls.append(('disconnect', disable_torque))
		self.is_connected = False

	def configure_motors(self, *args: object, **kwargs: object) -> None:
		"""모터 설정 기록."""
		self.calls.append(('configure',))

	def disable_torque(self, motors: object = None,
		num_retry: int = 0) -> None:
		"""토크 해제 기록."""
		self.calls.append(('disable_torque',))

	def enable_torque(self, motors: object = None,
		num_retry: int = 0) -> None:
		"""토크 켜기 기록."""
		self.calls.append(('enable_torque',))

	@contextmanager
	def torque_disabled(self, motors: object = None) -> Iterator[None]:
		"""공식 configure 가 쓰는 토크 해제 구간."""
		self.disable_torque()
		try:
			yield
		finally:
			self.enable_torque()

	def write(self, data_name: str, motor: str, value: object, *,
		normalize: bool = True, num_retry: int = 0) -> None:
		"""레지스터 쓰기 기록."""
		self.calls.append(('write', data_name, motor, value))

	def sync_read(self, data_name: str, motors: object = None, *,
		normalize: bool = True, num_retry: int = 0) -> dict[str, float]:
		"""현재 위치 (fail_reads 가 남았으면 ConnectionError)."""
		if self.fail_reads > 0:
			self.fail_reads -= 1
			raise ConnectionError(f'{self.port}: 가짜 읽기 실패')
		if normalize:
			return dict(self.values)
		return dict.fromkeys(JOINTS, 2047)

	def sync_write(self, data_name: str, values: dict[str, float], *,
		normalize: bool = True, num_retry: int = 0) -> None:
		"""목표 쓰기 기록, 정규화 목표는 현재 위치에 반영."""
		self.calls.append(('goal', dict(values), normalize))
		if normalize and data_name == 'Goal_Position':
			self.values.update(values)

	def set_half_turn_homings(self, motors: object = None) -> dict[str, int]:
		"""모든 관절의 가상 홈 오프셋."""
		return dict.fromkeys(JOINTS, 0)

	def record_ranges_of_motion(self, motors: list[str],
		display_values: bool = True) -> tuple[dict, dict]:
		"""공식 보정처럼 wrist_roll 을 뺀 6축의 범위를 잰다."""
		assert set(motors) == set(JOINTS) - {'wrist_roll'}, motors
		return dict.fromkeys(motors, 1000), dict.fromkeys(motors, 3000)

	def write_calibration(self, calibration_dict: dict, cache: bool = True,
		) -> None:
		"""가상 모터에 보정값 쓰기."""
		self.calls.append(('write_calibration',))
		if cache:
			self.calibration = calibration_dict

	def setup_motor(self, motor: str, initial_baudrate: int | None = None,
		initial_id: int | None = None) -> None:
		"""ID 설정 순서 기록."""
		self.setup_order.append(motor)


def use_temp_calibration_root(test: unittest.TestCase) -> Path:
	"""LeRobot 기본 보정 폴더를 시험 동안 임시 폴더로 바꾼다.

	Robot · Teleoperator 는 생성할 때 보정 폴더를 만들므로,
	실제 ~/.cache/huggingface/lerobot/calibration 을 건드리지 않게 한다.

	Args:
		test: 정리(cleanup)를 맡길 시험.

	Returns:
		임시 보정 루트 (그 아래 robots/ · teleoperators/ 가 생긴다).
	"""
	directory = tempfile.TemporaryDirectory()
	test.addCleanup(directory.cleanup)
	root = Path(directory.name)
	for target in ('lerobot.robots.robot.HF_LEROBOT_CALIBRATION',
		'lerobot.teleoperators.teleoperator.HF_LEROBOT_CALIBRATION'):
		patcher = patch(target, root)
		patcher.start()
		test.addCleanup(patcher.stop)
	return root


def patch_make_bus(test: unittest.TestCase,
	initial_values: dict[str, dict[str, float]] | None = None,
	) -> dict[str, FakeBus]:
	"""시험 동안 `arm.make_bus` 를 FakeBus 로 바꾼다.

	Args:
		test: 정리(cleanup)를 맡길 시험.
		initial_values: 포트 → 처음 위치 (일부 관절만 줘도 된다).

	Returns:
		포트 → 만들어진 FakeBus (만들 때마다 채워진다).
	"""
	buses: dict[str, FakeBus] = {}

	def make_fake_bus(port: str, use_degrees: bool,
		calibration: dict[str, MotorCalibration],
		is_mirror: bool = False) -> FakeBus:
		bus = FakeBus(port, use_degrees, calibration, is_mirror)
		bus.values.update((initial_values or {}).get(port, {}))
		buses[port] = bus
		return bus

	patcher = patch('lerobot_robot_dual_so102.arm.make_bus', make_fake_bus)
	patcher.start()
	test.addCleanup(patcher.stop)
	return buses


def write_calibration_file(path: Path, range_min: int = 1000,
	range_max: int = 3000) -> None:
	"""LeRobot 형식의 7관절 보정 파일을 쓴다 (폴더도 만든다).

	Args:
		path: 보정 파일 경로 (예: .../so102_follower/bi_so102_left.json).
		range_min: 모든 관절의 최소 위치.
		range_max: 모든 관절의 최대 위치.
	"""
	data = {
		name: {'id': index, 'drive_mode': 0, 'homing_offset': 0,
			'range_min': range_min, 'range_max': range_max}
		for index, name in enumerate(JOINTS, 1)
	}
	path.parent.mkdir(parents=True, exist_ok=True)
	path.write_text(json.dumps(data, indent=4))


def camera(height: int, width: int) -> OpenCVCameraConfig:
	"""열지 않는 카메라 설정 (생성만 하고 장치는 열지 않는다)."""
	return OpenCVCameraConfig(index_or_path='/dev/null', fps=30,
		width=width, height=height)
