"""SO-102 7축 단팔 — 공식 SO 팔로워 · 리더를 상속하고 모터 버스만 7축으로 바꾼다.

이 모듈의 클래스는 LeRobot 에 등록하지 않는다 (양팔 클래스가 안에서만 쓴다).
클래스 `name` 을 so102_follower · so102_leader 로 두어 보정 파일 위치가
기존 양팔 코드 · so-102 플러그인과 같다 → 기존 보정 파일을 그대로 읽는다.
"""

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
# USB 순간 끊김을 흡수한다: 처음 1번 + 재시도 2번
BUS_NUM_RETRY: Final[int] = 2


class SO102Bus(FeetechMotorsBus):
	"""SO-102 7축 Feetech 버스 — sync_read · sync_write 재시도 기본값만 다르다."""

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
) -> SO102Bus:
	"""SO-102 7축(ID 1~7, STS3215) 버스를 만든다. 시험이 바꿔 끼우는 지점이다.

	Args:
		port: 시리얼 포트 (예: /dev/so102_follower_left). 아직 열지 않는다.
		use_degrees: True 면 몸통 6관절을 도 단위, False 면 -100~100 으로.
		calibration: 관절 이름 → 보정값 (보정 파일 내용).

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
	return SO102Bus(port, motors, calibration=calibration)


class SO102ArmFollower(SOFollower):
	"""공식 SOFollower 에 7축 버스를 끼운 단팔 팔로워 (등록 안 함).

	연결 · 보정 · 관측 · 명령은 공식 구현을 그대로 쓴다.
	"""

	name = 'so102_follower'

	def __init__(self, config: SOFollowerRobotConfig) -> None:
		super().__init__(config)
		self.bus = make_bus(config.port, config.use_degrees, self.calibration)


class SO102ArmLeader(SOLeader):
	"""공식 SOLeader 에 7축 버스를 끼운 단팔 리더 (등록 안 함)."""

	name = 'so102_leader'

	def __init__(self, config: SOLeaderTeleopConfig) -> None:
		super().__init__(config)
		self.bus = make_bus(config.port, config.use_degrees, self.calibration)
