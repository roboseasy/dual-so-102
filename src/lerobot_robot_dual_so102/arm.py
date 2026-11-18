"""SO-102 7축 단팔 — 공식 SO 팔로워 · 리더를 상속하고 모터 버스만 7축으로 바꾼다.

이 모듈의 클래스는 LeRobot 에 등록하지 않는다 (양팔 클래스가 안에서만 쓴다).
클래스 `name` 을 so102_follower · so102_leader 로 두어 보정 파일 위치가
기존 양팔 코드 · so-102 플러그인과 같다 → 기존 보정 파일을 그대로 읽는다.
"""

from dataclasses import replace
import logging
from typing import Final

from lerobot.motors import Motor
from lerobot.motors import MotorCalibration
from lerobot.motors import MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so_follower import SOFollower
from lerobot.robots.so_follower import SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeader
from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig

JOINT_NAMES: Final[tuple[str, ...]] = (
	'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
	'wrist_yaw', 'wrist_roll', 'gripper',
)
# 미러 모드에서 부호를 뒤집는 관절 (모터 1 · 5 · 6)
MIRROR_JOINTS: Final[tuple[str, ...]] = (
	'shoulder_pan', 'wrist_yaw', 'wrist_roll')
# USB 순간 끊김을 흡수한다: 처음 1번 + 재시도 2번
BUS_NUM_RETRY: Final[int] = 2

logger = logging.getLogger(__name__)


class SO102Bus(FeetechMotorsBus):
	"""SO-102 7축 Feetech 버스.

	공식 버스와 두 가지가 다르다.
	- sync_read · sync_write 의 재시도 기본값이 BUS_NUM_RETRY 다.
	- 미러 버스는 도 단위에서도 MIRROR_JOINTS 의 drive_mode 로 부호를 뒤집는다.
	  LeRobot 은 -100~100 · 0~100 단위에서만 drive_mode 를 쓰고 도 단위에서는
	  무시한다. 같은 기준(보정 범위 가운데)으로 뒤집고, 대상을 MIRROR_JOINTS 로
	  한정해 다른 관절의 drive_mode 가 1 이어도 바꾸지 않는다.

	Attributes:
		is_mirror: 미러 모드 버스인지.
	"""

	def __init__(
		self,
		port: str,
		motors: dict[str, Motor],
		calibration: dict[str, MotorCalibration] | None = None,
		is_mirror: bool = False,
	) -> None:
		super().__init__(port, motors, calibration)
		self.is_mirror = is_mirror

	def _is_mirrored_degrees(self, motor_id: int) -> bool:
		"""이 모터의 도 단위 값을 뒤집어야 하는지."""
		motor = self._id_to_name(motor_id)
		calibration = self.calibration.get(motor)
		return (self.is_mirror
			and motor in MIRROR_JOINTS
			and calibration is not None
			and bool(calibration.drive_mode)
			and self.motors[motor].norm_mode is MotorNormMode.DEGREES)

	def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
		normalized = super()._normalize(ids_values)
		return {motor_id: -value if self._is_mirrored_degrees(motor_id)
			else value for motor_id, value in normalized.items()}

	def _unnormalize(self, ids_values: dict[int, float]) -> dict[int, int]:
		flipped = {motor_id: -value if self._is_mirrored_degrees(motor_id)
			else value for motor_id, value in ids_values.items()}
		return super()._unnormalize(flipped)

	def sync_read(
		self,
		data_name: str,
		motors: str | list[str] | None = None,
		*,
		normalize: bool = True,
		num_retry: int = BUS_NUM_RETRY,
	) -> dict[str, float]:
		"""공식 sync_read 와 같다. 재시도 기본값만 BUS_NUM_RETRY 다."""
		return super().sync_read(data_name, motors, normalize=normalize,
			num_retry=num_retry)

	def sync_write(
		self,
		data_name: str,
		values: float | dict[str, float],
		*,
		normalize: bool = True,
		num_retry: int = BUS_NUM_RETRY,
	) -> None:
		"""공식 sync_write 와 같다. 재시도 기본값만 BUS_NUM_RETRY 다."""
		super().sync_write(data_name, values, normalize=normalize,
			num_retry=num_retry)


def make_bus(
	port: str,
	use_degrees: bool,
	calibration: dict[str, MotorCalibration],
	is_mirror: bool = False,
) -> SO102Bus:
	"""SO-102 7축(ID 1~7, STS3215) 버스를 만든다. 시험이 바꿔 끼우는 지점이다.

	Args:
		port: 시리얼 포트 (예: /dev/so102_follower_left). 아직 열지 않는다.
		use_degrees: True 면 몸통 6관절을 도 단위, False 면 -100~100 으로.
		calibration: 관절 이름 → 보정값 (보정 파일 내용).
		is_mirror: 미러 모드 버스로 만들지.

	Returns:
		그리퍼만 0~100 으로 정규화하는 7축 버스.
	"""
	body_mode = (MotorNormMode.DEGREES if use_degrees
		else MotorNormMode.RANGE_M100_100)
	motors = {
		name: Motor(index, 'sts3215',
			MotorNormMode.RANGE_0_100 if name == 'gripper' else body_mode)
		for index, name in enumerate(JOINT_NAMES, 1)
	}
	return SO102Bus(port, motors, calibration=calibration,
		is_mirror=is_mirror)


class SO102ArmFollower(SOFollower):
	"""공식 SOFollower 에 7축 버스를 끼운 단팔 팔로워 (등록 안 함).

	연결 · 보정 · 관측 · 명령은 공식 구현을 그대로 쓴다.

	Attributes:
		is_mirror: 미러 모드 팔인지. 연결할 때 버스 보정 사본에만
			MIRROR_JOINTS 의 drive_mode=1 을 넣는다.
	"""

	name = 'so102_follower'

	def __init__(self, config: SOFollowerRobotConfig,
		is_mirror: bool = False) -> None:
		super().__init__(config)
		self.is_mirror = is_mirror
		self.bus = make_bus(config.port, config.use_degrees, self.calibration,
			is_mirror=is_mirror)

	def connect(self, calibrate: bool = True) -> None:
		"""공식 연결(필요하면 보정 포함) 뒤, 미러 팔이면 미러 보정을 건다."""
		super().connect(calibrate)
		if self.is_mirror:
			self._apply_mirror_drive_mode()

	def _apply_mirror_drive_mode(self) -> None:
		"""버스 보정을 MIRROR_JOINTS 만 drive_mode=1 인 사본으로 바꾼다.

		self.calibration(파일에 저장되는 원본)은 그대로 두므로 보정 파일은
		바뀌지 않는다. Feetech write_calibration 은 drive_mode 를 모터에 쓰지 않는다.
		"""
		self.bus.calibration = {
			motor: replace(calibration, drive_mode=1)
			if motor in MIRROR_JOINTS else calibration
			for motor, calibration in self.bus.calibration.items()
		}
		logger.info(f'{self} mirror mode: drive_mode=1 on '
			f'{", ".join(MIRROR_JOINTS)} (calibration file unchanged)')


class SO102ArmLeader(SOLeader):
	"""공식 SOLeader 에 7축 버스를 끼운 단팔 리더 (등록 안 함)."""

	name = 'so102_leader'

	def __init__(self, config: SOLeaderTeleopConfig) -> None:
		super().__init__(config)
		self.bus = make_bus(config.port, config.use_degrees, self.calibration)
