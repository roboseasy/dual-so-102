"""레포 · 문서 시험 — 옛 이름 없음, 문서 명령이 공식 CLI 로 읽힘, udev 예시."""

import importlib
from pathlib import Path
import re
import shlex
import subprocess
import unittest

import draccus

import lerobot_robot_dual_so102  # noqa: F401 (dual_so102_* 등록)

REPO = Path(__file__).resolve().parents[1]
OUR_TYPES = {'dual_so102_follower', 'dual_so102_client', 'dual_so102_leader'}
# 문서 명령 → (설정 클래스가 있는 모듈, 클래스 이름)
COMMAND_CONFIGS = {
	'lerobot-calibrate': ('lerobot.scripts.lerobot_calibrate', 'CalibrateConfig'),
	'lerobot-teleoperate': ('lerobot.scripts.lerobot_teleoperate',
		'TeleoperateConfig'),
	'lerobot-record': ('lerobot.scripts.lerobot_record', 'RecordConfig'),
	'lerobot-replay': ('lerobot.scripts.lerobot_replay', 'ReplayConfig'),
	'dual-so102-host': ('lerobot_robot_dual_so102.host',
		'DualSO102ServerConfig'),
}


def doc_files() -> list[Path]:
	"""README 와 docs/ 의 문서."""
	return [REPO / 'README.md', *sorted((REPO / 'docs').glob('*.md'))]


def documented_commands() -> list[tuple[str, list[str]]]:
	"""문서 코드 블록의 명령을 (파일 이름, argv) 로 꺼낸다 (줄 끝 \\ 는 잇는다)."""
	commands = []
	for path in doc_files():
		blocks = re.findall(r'^```[a-z]*\n(.*?)^```', path.read_text(),
			re.MULTILINE | re.DOTALL)
		for block in blocks:
			for line in block.replace('\\\n', ' ').splitlines():
				argv = shlex.split(line, comments=True)
				if argv and argv[0] in COMMAND_CONFIGS:
					commands.append((path.name, argv))
	return commands


class RepoTest(unittest.TestCase):
	"""문서가 코드와 어긋나지 않는지 본다."""

	def test_no_old_repo_name(self) -> None:
		"""레포에 올라가는 파일(이름 · 내용) 어디에도 옛 교육 레포 이름이 없다."""
		old_name = 'a' + '-ba'  # 이 파일에도 그대로 적지 않는다
		try:
			listed = subprocess.run(['git', 'ls-files', '--cached', '--others',
				'--exclude-standard'], cwd=REPO, capture_output=True, text=True,
				check=True).stdout.splitlines()
		except (OSError, subprocess.CalledProcessError):
			self.skipTest('git 저장소에서만 확인한다')
		paths = [REPO / name for name in listed if (REPO / name).is_file()]
		self.assertTrue(paths)
		hits = [str(path.relative_to(REPO)) for path in paths
			if old_name in str(path.relative_to(REPO))
			or old_name in path.read_text(errors='ignore')]
		self.assertEqual(hits, [])

	def test_readme_types_are_registered(self) -> None:
		"""문서의 --robot.type · --teleop.type 은 이 패키지의 세 타입뿐이다."""
		found = {(path.name, value) for path in doc_files()
			for value in re.findall(r'--(?:robot|teleop)\.type[= ]([\w-]+)',
				path.read_text())}
		self.assertTrue(found)
		self.assertEqual({value for _, value in found} - OUR_TYPES, set(),
			sorted(found))

	def test_documented_commands_parse(self) -> None:
		"""문서의 LeRobot · host 명령이 공식 설정 클래스로 오류 없이 읽힌다.

		--policy.path 가 든 명령(학습한 모델이 있어야 읽힘)은 실장비 단계에서,
		--help 만 붙은 설치 확인 명령은 test_dual · test_client 에서 본다.
		"""
		commands = [(name, argv) for name, argv in documented_commands()
			if not any(arg.startswith('--policy.path') or arg == '--help'
				for arg in argv)]
		self.assertGreaterEqual(len({argv[0] for _, argv in commands}),
			len(COMMAND_CONFIGS))
		for name, argv in commands:
			with self.subTest(file=name, command=' '.join(argv[:3])):
				module, class_name = COMMAND_CONFIGS[argv[0]]
				try:
					config_class = getattr(importlib.import_module(module),
						class_name)
				except ImportError as error:
					self.skipTest(f'{module} import 불가 ({error}) — '
						'PC 는 pip install -e ".[dual-so-102]"')
				draccus.parse(config_class, args=argv[1:], exit_on_error=False)

	def test_udev_example_names(self) -> None:
		"""udev 예시는 팔 4개 · 카메라 3개 별칭을 하나씩 만든다."""
		rules = (REPO / 'config' / '99-dual-so102.rules.example').read_text()
		self.assertEqual(sorted(re.findall(r'SYMLINK\+="([^"]+)"', rules)), [
			'cam_left', 'cam_right', 'cam_top',
			'so102_follower_left', 'so102_follower_right',
			'so102_leader_left', 'so102_leader_right'])


if __name__ == '__main__':
	unittest.main()
