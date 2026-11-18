"""LeRobot 0.6.0 이 자동 발견하는 SO-102 7축 양팔 플러그인.

공식 팩터리는 XxxConfig 에 맞는 장치 클래스 Xxx 를 이 패키지에서 찾으므로
등록한 설정과 장치 클래스를 여기서 내보낸다.
"""

from .client import DualSO102Client
from .config import DualSO102ClientConfig
from .config import DualSO102FollowerConfig
from .config import DualSO102LeaderConfig
from .dual import DualSO102Follower
from .dual import DualSO102Leader

__all__ = [
	'DualSO102Client',
	'DualSO102ClientConfig',
	'DualSO102Follower',
	'DualSO102FollowerConfig',
	'DualSO102Leader',
	'DualSO102LeaderConfig',
]
