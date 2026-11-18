"""SO-102 7축 양팔 — 공식 BiSOFollower · BiSOLeader 를 상속하고 팔만 7축으로 바꾼다.

관측 · 명령 · 특징(features) · 연결 · 보정은 공식 구현을 그대로 쓴다.
공식 __init__ 은 팔을 6축 SOFollower · SOLeader 로 만들므로 부르지 않고,
같은 규칙(보정 id <id>_left/right, 최상위 카메라는 왼팔이 연다)으로 직접 만든다.
"""

from lerobot.cameras import CameraConfig
from lerobot.robots import Robot
from lerobot.robots.bi_so_follower import BiSOFollower
from lerobot.robots.so_follower import SOFollowerConfig
from lerobot.robots.so_follower import SOFollowerRobotConfig
from lerobot.teleoperators import Teleoperator
from lerobot.teleoperators.bi_so_leader import BiSOLeader
from lerobot.teleoperators.so_leader import SOLeaderConfig
from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig

from .arm import SO102ArmFollower
from .arm import SO102ArmLeader
from .config import DualSO102FollowerConfig
from .config import DualSO102LeaderConfig


def _require_id(device_id: str | None, option: str) -> None:
	"""id 가 없으면 멈춘다.

	공식 Bi 클래스는 id 가 없으면 두 팔 id 를 모두 None 으로 두어
	왼팔 · 오른팔이 같은 None.json 보정 파일을 덮어쓴다.

	Args:
		device_id: 설정의 id.
		option: 사용자에게 알려 줄 CLI 옵션 이름.

	Raises:
		ValueError: id 가 없거나 빈 문자열인 경우.
	"""
	if not device_id:
		raise ValueError(f'{option} 를 지정하세요 '
			'(보정 파일 <id>_left.json · <id>_right.json 의 이름, 예: bi_so102)')


def _follower_arm_config(
	config: DualSO102FollowerConfig,
	arm: SOFollowerConfig,
	side: str,
	cameras: dict[str, CameraConfig],
) -> SOFollowerRobotConfig:
	"""양팔 설정에서 한 팔의 공식 단팔 설정을 만든다.

	Args:
		config: 양팔 설정 (id · 보정 폴더).
		arm: 이 팔에 쓸 포트 · 토크 · 제한 설정.
		side: 보정 id 접미사 ('left' 또는 'right').
		cameras: 이 팔이 열 카메라.

	Returns:
		보정 id 가 <id>_<side> 인 단팔 설정.
	"""
	return SOFollowerRobotConfig(
		id=f'{config.id}_{side}',
		calibration_dir=config.calibration_dir,
		port=arm.port,
		disable_torque_on_disconnect=arm.disable_torque_on_disconnect,
		max_relative_target=arm.max_relative_target,
		use_degrees=arm.use_degrees,
		cameras=cameras,
	)


class DualSO102Follower(BiSOFollower):
	"""SO-102 7축 양팔 팔로워 (dual_so102_follower).

	Attributes:
		left_arm: left_* 키를 맡는 단팔.
		right_arm: right_* 키를 맡는 단팔.
		cameras: 두 팔이 연 카메라 전부.
	"""

	config_class = DualSO102FollowerConfig
	name = 'dual_so102_follower'

	def __init__(self, config: DualSO102FollowerConfig) -> None:
		_require_id(config.id, '--robot.id')
		Robot.__init__(self, config)
		self.config = config
		self._top_level_cam_keys = set(config.cameras)
		arm_cam_keys = (set(config.left_arm_config.cameras)
			| set(config.right_arm_config.cameras))
		collisions = self._top_level_cam_keys & arm_cam_keys
		if collisions:
			raise ValueError(
				f'카메라 이름이 최상위 · 팔별 설정에 겹칩니다: {sorted(collisions)}')
		self.left_arm = SO102ArmFollower(_follower_arm_config(
			config, config.left_arm_config, 'left',
			{**config.left_arm_config.cameras, **config.cameras}))
		self.right_arm = SO102ArmFollower(_follower_arm_config(
			config, config.right_arm_config, 'right',
			config.right_arm_config.cameras))
		self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}


def _leader_arm_config(
	config: DualSO102LeaderConfig,
	arm: SOLeaderConfig,
	side: str,
) -> SOLeaderTeleopConfig:
	"""양팔 리더 설정에서 한 팔의 공식 단팔 설정을 만든다.

	Args:
		config: 양팔 리더 설정 (id · 보정 폴더).
		arm: 이 팔의 포트 · 단위 설정.
		side: 보정 id 접미사 ('left' 또는 'right').

	Returns:
		보정 id 가 <id>_<side> 인 단팔 리더 설정.
	"""
	return SOLeaderTeleopConfig(
		id=f'{config.id}_{side}',
		calibration_dir=config.calibration_dir,
		port=arm.port,
		use_degrees=arm.use_degrees,
	)


class DualSO102Leader(BiSOLeader):
	"""SO-102 7축 양팔 리더 (dual_so102_leader)."""

	config_class = DualSO102LeaderConfig
	name = 'dual_so102_leader'

	def __init__(self, config: DualSO102LeaderConfig) -> None:
		_require_id(config.id, '--teleop.id')
		Teleoperator.__init__(self, config)
		self.config = config
		self.left_arm = SO102ArmLeader(
			_leader_arm_config(config, config.left_arm_config, 'left'))
		self.right_arm = SO102ArmLeader(
			_leader_arm_config(config, config.right_arm_config, 'right'))
