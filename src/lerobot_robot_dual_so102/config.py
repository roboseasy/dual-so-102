"""LeRobot CLI 에 등록하는 dual-so-102 설정 타입.

등록 이름은 dual_so102_* 뿐이다. solution 을 덮어쓴 LeRobot · so-102 플러그인이
쓰는 so102_* · bi_so102_* 와 겹치지 않아 같은 환경에 같이 설치된다.
"""

from dataclasses import dataclass
from dataclasses import field

from lerobot.cameras import CameraConfig
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


@RobotConfig.register_subclass('dual_so102_client')
@dataclass
class DualSO102ClientConfig(RobotConfig):
	"""PC 에서 Pi 의 dual-so102-host 에 붙는 원격 양팔 (dual_so102_client).

	카메라는 Pi 의 host 가 연다. cameras 에는 host 와 같은 이름 · 크기를 적는다
	(PC 는 카메라를 열지 않고, 녹화 데이터셋의 모양을 정하는 데만 쓴다).

	Attributes:
		remote_ip: Pi 주소. AP 모드 192.168.4.1, 유선 · 공유기 모드는 Pi 에서
			hostname -I 로 확인 (예: 10.42.0.78, setip 하면 192.168.0.(100+킷 번호)).
		port_zmq_cmd: 명령 포트 (host 와 같게).
		port_zmq_observations: 관측 포트 (host 와 같게).
		polling_timeout_ms: 수신 스레드가 한 번에 기다리는 시간.
		connect_timeout_s: 연결할 때 첫 관측(카메라 모두)을 기다리는 시간. Pi host 는
			카메라 3대를 여는 데 약 16초 걸리므로 넉넉히 둔다.
		stale_timeout_s: host 관측이나 한 카메라가 이만큼 끊기면 멈춘다.
		cameras: host 카메라와 같은 이름 · 크기 (width · height · fps 필수).
	"""

	remote_ip: str
	port_zmq_cmd: int = 5555
	port_zmq_observations: int = 5556
	polling_timeout_ms: int = 15
	connect_timeout_s: float = 60.0
	stale_timeout_s: float = 1.0
	cameras: dict[str, CameraConfig] = field(default_factory=dict)


@dataclass
class DualSO102HostConfig:
	"""Pi 의 dual-so102-host 설정 (CLI 의 --host.*, 등록하지 않는다).

	Attributes:
		port_zmq_cmd: 명령을 받는 포트 (client 와 같게).
		port_zmq_observations: 관측을 보내는 포트 (client 와 같게).
		connection_time_s: 이 시간 뒤 끝낸다. 0 이면 끝없이 돈다.
		watchdog_timeout_ms: 명령이 이만큼 없으면 경고하고 자세를 유지한다.
		max_loop_freq_hz: 루프 최대 주기.
		jpeg_quality: 카메라 프레임 JPEG 품질.
	"""

	port_zmq_cmd: int = 5555
	port_zmq_observations: int = 5556
	connection_time_s: int = 0
	watchdog_timeout_ms: int = 500
	max_loop_freq_hz: int = 30
	jpeg_quality: int = 90
