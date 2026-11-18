"""보조 명령(tools.py) 시험 — 모터 ID 설정 순서, 포트 판별(읽기 전용)."""

from contextlib import redirect_stdout
import io
import subprocess
import sys
import unittest
from unittest.mock import MagicMock
from unittest.mock import patch

from lerobot.motors.feetech import FeetechMotorsBus

from lerobot_robot_dual_so102.tools import check_main
from lerobot_robot_dual_so102.tools import setup_motors_main
from tests.fake_bus import patch_make_bus
from tests.fake_bus import use_temp_calibration_root

# 포트 → (응답하는 모터 ID, 전압 V). None 이면 포트를 못 연다
PORTS = {
	'A': (range(1, 8), 12.1),
	'B': (range(1, 8), 5.0),
	'C': (range(1, 7), 12.0),
	'D': (range(1, 8), 9.5),
	'E': (range(1, 9), 12.0),
	'F': None,
}
# 쓰기 · 토크를 바꾸는 버스 동작 (판별 도구는 하나도 부르면 안 된다)
WRITERS = ('write', 'sync_write', 'write_calibration', 'enable_torque',
	'disable_torque', 'setup_motor')


class ToolsTest(unittest.TestCase):
	"""공식 CLI 가 플러그인을 못 읽는 두 가지(모터 ID · 포트 판별)를 채운다."""

	def setUp(self) -> None:
		self.root = use_temp_calibration_root(self)

	def fake_feetech(self) -> dict[str, MagicMock]:
		"""FeetechMotorsBus 의 시리얼 동작을 포트별 가짜 응답으로 바꾼다."""
		def connect(bus: FeetechMotorsBus, handshake: bool = True) -> None:
			if PORTS[bus.port] is None:
				raise ConnectionError(f"Could not connect on port '{bus.port}'")

		def broadcast_ping(bus: FeetechMotorsBus, num_retry: int = 0,
			raise_on_error: bool = False) -> dict[int, int]:
			ids, _ = PORTS[bus.port]
			return dict.fromkeys(ids, 777)

		def read(bus: FeetechMotorsBus, data_name: str, motor: str, *,
			normalize: bool = True, num_retry: int = 0) -> int:
			self.assertEqual(data_name, 'Present_Voltage')
			_, volts = PORTS[bus.port]
			return round(volts * 10)

		effects = {'connect': connect, 'broadcast_ping': broadcast_ping,
			'read': read, 'disconnect': None,
			**dict.fromkeys(WRITERS)}
		mocks = {}
		for name, effect in effects.items():
			patcher = patch.object(FeetechMotorsBus, name, autospec=True,
				side_effect=effect)
			mocks[name] = patcher.start()
			self.addCleanup(patcher.stop)
		return mocks

	def run_check(self, ports: list[str]) -> tuple[int, list[str]]:
		"""check_main 의 종료 코드와 출력 줄."""
		output = io.StringIO()
		with redirect_stdout(output):
			code = check_main(ports)
		return code, output.getvalue().splitlines()

	def test_setup_motors_order(self) -> None:
		"""그리퍼(7) → shoulder_pan(1) 순서로 한 모터씩 ID 를 쓴다."""
		buses = patch_make_bus(self)
		for role in ('follower', 'leader'):
			with self.subTest(role=role):
				with (patch('builtins.input', return_value=''),
					redirect_stdout(io.StringIO())):
					code = setup_motors_main(
						['--role', role, '--port', f'/dev/{role}'])
				self.assertEqual(code, 0)
				self.assertEqual(buses[f'/dev/{role}'].setup_order, [
					'gripper', 'wrist_roll', 'wrist_yaw', 'wrist_flex',
					'elbow_flex', 'shoulder_lift', 'shoulder_pan'])

	def test_check_classifies_ports(self) -> None:
		"""전압으로 역할을, ID 1~7 로 판정을 내고, 하나라도 OK 가 아니면 1."""
		self.fake_feetech()
		code, lines = self.run_check(['A', 'B', 'C', 'D', 'E', 'F'])
		self.assertEqual(code, 1)
		self.assertIn('A  팔로워  ids=1..7  12.1V  OK', lines)
		self.assertIn('B  리더  ids=1..7  5.0V  OK', lines)
		self.assertIn('C  팔로워  ids=1..6  12.0V  빠진 ID 7', lines)
		self.assertIn('D  ?  ids=1..7  9.5V  OK', lines)
		self.assertIn('E  팔로워  ids=1..8  12.0V  남는 ID 8', lines)
		self.assertTrue([line for line in lines
			if line.startswith('F  열 수 없음')], lines)
		ok_code, _ = self.run_check(['A', 'B'])
		self.assertEqual(ok_code, 0)

	def test_check_skips_port_in_use(self) -> None:
		"""다른 프로세스(host · 텔레옵)가 연 포트는 열지 않고 '사용 중' 으로 알린다."""
		mocks = self.fake_feetech()
		port = self.root / 'ttyFAKE'
		port.write_text('')
		holder = subprocess.Popen([sys.executable, '-c',
			f"f = open({str(port)!r}); print('OPEN', flush=True); "
			'import time; time.sleep(60)'], stdout=subprocess.PIPE, text=True)
		self.addCleanup(holder.wait)
		self.addCleanup(holder.kill)
		self.assertEqual(holder.stdout.readline().strip(), 'OPEN')
		code, lines = self.run_check([str(port)])
		self.assertEqual(code, 1)
		self.assertTrue([line for line in lines if line.startswith(
			f'{port}  사용 중 (pid {holder.pid})')], lines)
		mocks['connect'].assert_not_called()

	def test_check_is_read_only(self) -> None:
		"""판별은 ping · 전압 읽기만 하고, 닫을 때도 토크를 건드리지 않는다."""
		mocks = self.fake_feetech()
		self.run_check(['A', 'B', 'C', 'F'])
		for name in WRITERS:
			with self.subTest(writer=name):
				mocks[name].assert_not_called()
		self.assertTrue(mocks['disconnect'].called)
		self.assertTrue(all(call.kwargs == {'disable_torque': False}
			for call in mocks['disconnect'].call_args_list),
			mocks['disconnect'].call_args_list)


if __name__ == '__main__':
	unittest.main()
