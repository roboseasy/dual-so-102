"""공식 CLI 가 플러그인을 읽지 못하는 두 가지를 채우는 보조 명령.

LeRobot 0.6.0 의 lerobot-setup-motors · lerobot-find-joint-limits 는 정해진
로봇 목록만 받아 dual_so102_* 타입을 쓸 수 없다.

- dual-so102-check [PORT ...]: 포트마다 모터 ID · 전압을 읽어 팔로워(12 V) ·
	리더(5 V)를 가린다. 읽기만 한다 (모터가 움직이거나 토크가 바뀌지 않는다).
- dual-so102-setup-motors --role {follower,leader} --port PORT:
	팔 하나의 모터 ID 1~7 을 한 모터씩 쓴다.
"""

import argparse
from dataclasses import dataclass
import glob
import os
from typing import Final

from lerobot.motors import Motor
from lerobot.motors import MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so_follower import SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig

from .arm import JOINT_NAMES
from .arm import SO102ArmFollower
from .arm import SO102ArmLeader

# 팔로워(12 V) / 리더(5 V) 전원 판별 기준 (기존 양팔 코드의 포트 점검과 같다)
FOLLOWER_VOLTAGE_MIN_V: Final[float] = 10.8
FOLLOWER_VOLTAGE_MAX_V: Final[float] = 13.2
LEADER_VOLTAGE_MAX_V: Final[float] = 8.0
PING_NUM_RETRY: Final[int] = 3
EXPECTED_IDS: Final[frozenset[int]] = frozenset(
	range(1, len(JOINT_NAMES) + 1))


@dataclass
class PortReport:
	"""포트 하나를 읽은 결과.

	Attributes:
		port: 시리얼 포트.
		ids: 응답한 모터 ID (작은 것부터).
		voltage: 첫 모터의 입력 전압 [V]. 못 읽으면 None.
		error: 포트를 못 열었거나 ping 이 실패한 이유. 정상이면 None.
	"""

	port: str
	ids: list[int]
	voltage: float | None
	error: str | None = None


def port_users(port: str) -> list[int]:
	"""그 포트를 열어 둔 다른 프로세스 번호 (/proc 에서 찾는다, 이 프로세스는 뺀다).

	host · 텔레옵이 쓰는 포트를 같이 열면 서보 응답을 가로채 그 프로그램이
	끝나고 두 팔 토크가 풀린다. 그래서 판별 전에 확인한다.

	Args:
		port: 시리얼 포트 (별칭이어도 된다).

	Returns:
		프로세스 번호 목록 (못 읽는 다른 사용자 프로세스는 빠진다).
	"""
	target = os.path.realpath(port)
	users = []
	try:
		entries = [entry for entry in os.scandir('/proc')
			if entry.name.isdigit() and int(entry.name) != os.getpid()]
	except OSError:
		return users
	for entry in entries:
		try:
			links = [os.readlink(fd.path)
				for fd in os.scandir(f'/proc/{entry.name}/fd')]
		except OSError:
			continue  # 끝난 프로세스 · 다른 사용자 프로세스
		if target in links:
			users.append(int(entry.name))
	return users


def probe_port(port: str) -> PortReport:
	"""포트의 모터 ID 와 전압을 읽는다. 쓰기는 하지 않는다.

	닫을 때도 disable_torque=False 로 닫는다 (기본값은 모든 모터에 토크
	해제를 쓴다 — host 가 잡고 있는 팔이 풀릴 수 있다).

	Args:
		port: 시리얼 포트 (예: /dev/ttyACM0).

	Returns:
		읽은 결과.
	"""
	scanner = FeetechMotorsBus(port, {})
	try:
		scanner.connect(handshake=False)
	except Exception as error:
		return PortReport(port, [], None, ' '.join(str(error).split())
			or type(error).__name__)
	try:
		found = scanner.broadcast_ping(num_retry=PING_NUM_RETRY) or {}
	except Exception as error:
		return PortReport(port, [], None, f'ping 실패: {error}')
	finally:
		scanner.disconnect(disable_torque=False)
	ids = sorted(found)
	voltage = _read_voltage(port, ids[0]) if ids else None
	return PortReport(port, ids, voltage)


def _read_voltage(port: str, motor_id: int) -> float | None:
	"""모터 하나의 입력 전압 [V] (못 읽으면 None)."""
	bus = FeetechMotorsBus(port,
		{'probe': Motor(motor_id, 'sts3215', MotorNormMode.DEGREES)})
	try:
		bus.connect(handshake=False)
	except Exception:
		return None
	try:
		raw = bus.read('Present_Voltage', 'probe', normalize=False,
			num_retry=PING_NUM_RETRY)
		return raw / 10
	except Exception:
		return None
	finally:
		bus.disconnect(disable_torque=False)


def _format_ids(ids: list[int]) -> str:
	"""ID 목록을 짧게 적는다 ([1..7] → '1..7', [1, 2, 4] → '1,2,4')."""
	if not ids:
		return '없음'
	runs: list[list[int]] = []
	for motor_id in ids:
		if runs and motor_id == runs[-1][-1] + 1:
			runs[-1].append(motor_id)
		else:
			runs.append([motor_id])
	return ','.join(f'{run[0]}..{run[-1]}' if len(run) >= 3
		else ','.join(map(str, run)) for run in runs)


def _role(voltage: float | None) -> str:
	"""전압으로 가른 역할."""
	if voltage is not None:
		if FOLLOWER_VOLTAGE_MIN_V <= voltage <= FOLLOWER_VOLTAGE_MAX_V:
			return '팔로워'
		if voltage <= LEADER_VOLTAGE_MAX_V:
			return '리더'
	return '?'


def _verdict(ids: list[int]) -> str:
	"""ID 가 정확히 1~7 이면 'OK', 아니면 빠지거나 남는 ID."""
	if not ids:
		return '모터 응답 없음 — 전원(팔로워 12 V · 리더 5 V) · 케이블 확인'
	missing = sorted(EXPECTED_IDS - set(ids))
	extra = sorted(set(ids) - EXPECTED_IDS)
	problems = []
	if missing:
		problems.append(f'빠진 ID {_format_ids(missing)}')
	if extra:
		problems.append(f'남는 ID {_format_ids(extra)}')
	return ', '.join(problems) or 'OK'


def check_main(argv: list[str] | None = None) -> int:
	"""dual-so102-check: 포트마다 역할 · ID · 전압 · 판정을 한 줄씩 적는다.

	Args:
		argv: 명령줄 인자 (None 이면 sys.argv).

	Returns:
		모든 포트가 OK 면 0, 아니면 1.
	"""
	parser = argparse.ArgumentParser(prog='dual-so102-check',
		description='SO-102 팔 포트 판별 (읽기 전용: ping · 전압만 읽는다).')
	parser.add_argument('ports', nargs='*', metavar='PORT',
		help='검사할 포트 (기본: /dev/ttyACM* 전부)')
	args = parser.parse_args(argv)
	ports = args.ports or sorted(glob.glob('/dev/ttyACM*'))
	if not ports:
		print('/dev/ttyACM* 포트가 없습니다 — USB 케이블과 보드 전원을 확인하세요.')
		return 1
	is_all_ok = True
	for port in ports:
		users = port_users(port)
		if users:
			is_all_ok = False
			print(f'{port}  사용 중 (pid {",".join(map(str, users))}) — '
				'host · 텔레옵을 끄고 다시 실행하세요 (열지 않았습니다)')
			continue
		report = probe_port(port)
		if report.error is not None:
			is_all_ok = False
			print(f'{port}  열 수 없음 — 케이블 · 권한(dialout 그룹) · 다른 '
				f'프로그램(host 등)이 쓰는지 확인 ({report.error})')
			continue
		verdict = _verdict(report.ids)
		is_all_ok = is_all_ok and verdict == 'OK'
		volts = ('?' if report.voltage is None
			else f'{report.voltage:.1f}')
		print(f'{port}  {_role(report.voltage)}  '
			f'ids={_format_ids(report.ids)}  {volts}V  {verdict}')
	print('왼팔 · 오른팔은 ID · 전압으로 알 수 없습니다. 한 팔씩 뽑아 보며 '
		'확인하고 udev 별칭(/dev/so102_{follower,leader}_{left,right})을 '
		'만드세요 — docs/2.ports.md')
	return 0 if is_all_ok else 1


def setup_motors_main(argv: list[str] | None = None) -> int:
	"""dual-so102-setup-motors: 팔 하나의 모터 ID 1~7 을 한 모터씩 쓴다.

	Args:
		argv: 명령줄 인자 (None 이면 sys.argv).

	Returns:
		0 (실패하면 예외).
	"""
	parser = argparse.ArgumentParser(prog='dual-so102-setup-motors',
		description='SO-102 팔 하나의 모터 ID(1~7)와 통신 속도를 쓴다.')
	parser.add_argument('--role', choices=('follower', 'leader'),
		required=True, help='follower(12 V 팔로워) 또는 leader(5 V 리더)')
	parser.add_argument('--port', required=True,
		help='이 팔 보드의 포트 (예: /dev/ttyACM0)')
	args = parser.parse_args(argv)
	if args.role == 'follower':
		arm = SO102ArmFollower(SOFollowerRobotConfig(port=args.port))
	else:
		arm = SO102ArmLeader(SOLeaderTeleopConfig(port=args.port))
	order = ' → '.join(f'{name}({arm.bus.motors[name].id})'
		for name in reversed(arm.bus.motors))
	print(f'모터를 하나씩 보드에 연결해 ID 를 씁니다: {order}')
	print("안내(Connect the controller board to the '...' motor only)가 나오면 "
		'그 모터 하나만 보드에 연결하고 Enter 를 누르세요.')
	arm.setup_motors()
	print('끝났습니다. 모터를 모두 이어 연결하고 dual-so102-check 로 확인하세요.')
	return 0
