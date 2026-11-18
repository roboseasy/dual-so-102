"""LeRobot CLI 에 등록하는 dual-so-102 설정 타입.

등록 이름은 dual_so102_* 뿐이다. solution 을 덮어쓴 LeRobot · so-102 플러그인이
쓰는 so102_* · bi_so102_* 와 겹치지 않아 같은 환경에 같이 설치된다.
"""

from dataclasses import dataclass

from lerobot.robots import RobotConfig
from lerobot.robots.bi_so_follower import BiSOFollowerConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot.teleoperators.bi_so_leader import BiSOLeaderConfig


@RobotConfig.register_subclass('dual_so102_follower')
@dataclass
class DualSO102FollowerConfig(BiSOFollowerConfig):
	"""SO-102 7축 양팔 팔로워. 필드는 공식 bi_so_follower 에 mirror_mode 를 더했다.

	--robot.id 는 필수다. 두 팔의 보정 파일 이름이 <id>_left.json ·
	<id>_right.json 이다 (예: bi_so102 → bi_so102_left.json).

	Attributes:
		mirror_mode: True 면 거울처럼 움직인다. left_* 키가 오른팔을 움직이고
			(포트 · 보정 · 팔별 카메라를 짝으로 교체), 1 · 5 · 6번 관절의 부호를
			뒤집는다. 보정 파일은 바꾸지 않는다.
	"""

	mirror_mode: bool = False


@TeleoperatorConfig.register_subclass('dual_so102_leader')
@dataclass
class DualSO102LeaderConfig(BiSOLeaderConfig):
	"""SO-102 7축 양팔 리더. 필드는 공식 bi_so_leader 와 같고 --teleop.id 는 필수다."""
