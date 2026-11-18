# 로보시지 Dual SO-102 × LeRobot

로보시지 **7축 SO-102 양팔 키트**(팔로워 양팔 + 리더 양팔)를 **LeRobot v0.6.0** 공식 명령
(`lerobot-calibrate` · `lerobot-teleoperate` · `lerobot-record` · `lerobot-replay` · `lerobot-rollout`)에서
그대로 쓰게 하는 플러그인입니다. 팔로워 양팔과 카메라를 라즈베리파이에 꽂고 PC 와 네트워크(ZMQ)로
주고받는 구성(LeKiwi 방식)도 됩니다.

| 이름 | 쓰는 곳 | 하는 일 |
|---|---|---|
| `--robot.type=dual_so102_follower` | 팔로워가 꽂힌 컴퓨터 | 팔로워 양팔 (`--robot.mirror_mode=true` 로 거울 조종) |
| `--teleop.type=dual_so102_leader` | 리더가 꽂힌 PC | 리더 양팔 |
| `--robot.type=dual_so102_client` | PC | Pi 에 꽂힌 팔로워 양팔 · 카메라를 네트워크로 |
| `dual-so102-host` | Pi | 팔로워 양팔 · 카메라를 네트워크에 연다 |
| `dual-so102-check` | 어디서나 | 포트마다 팔로워 · 리더 구분과 모터 ID 확인 (읽기만, 다른 프로그램이 쓰는 포트는 건드리지 않음) |
| `dual-so102-setup-motors` | 어디서나 | 새 팔의 모터 ID 1~7 쓰기 |

관측 · 행동 키는 14개(`left_*` · `right_*` × 관절 7개 `.pos`)로 기존 양팔 코드(`bi_so102_*`)와 같고,
기존 보정 파일을 다시 보정하지 않고 그대로 읽습니다.

## 1. 설치

PC — LeRobot v0.6.0 이 설치된 `lerobot` 환경에 추가로 설치합니다.

```bash
conda activate lerobot
cd ~
git clone https://github.com/roboseasy/dual-so-102.git
cd dual-so-102
pip install -e ".[dual-so-102]"
```

Pi — 로그인하면 `~/lerobot060_venv` 가 켜져 있습니다. 새 Pi 라면 [Pi 분리 구성](docs/3.pi.md)의 준비부터 합니다.

```bash
cd ~/workspace
git clone https://github.com/roboseasy/dual-so-102.git
cd dual-so-102
pip install -e .
```

설치를 확인합니다. 도움말에 `left_arm_config` 가 보이면 됩니다.

```bash
lerobot-teleoperate --robot.type=dual_so102_follower --help
```

## 2. 포트 확인 · 고정

팔 4개의 보드를 꽂고 확인합니다. 팔로워는 12 V, 리더는 5 V 라 전압으로 역할을 가립니다.

```bash
dual-so102-check
```

```text
/dev/ttyACM0  팔로워  ids=1..7  12.1V  OK
/dev/ttyACM1  팔로워  ids=1..7  12.0V  OK
```

왼팔 · 오른팔은 이것으로 알 수 없으므로 한 팔씩 뽑아 보며 확인하고, udev 별칭
`/dev/so102_{follower,leader}_{left,right}` 을 만듭니다 → [USB 포트 고정](docs/2.ports.md).
3 · 4절 예시는 이 별칭을 씁니다.

## 3. 보정

팔을 받친 상태에서 안내를 따릅니다. 왼팔을 먼저, 이어서 오른팔을 보정합니다.

리더암 (내 PC)
```bash
lerobot-calibrate \
  --robot.type=dual_so102_follower \
  --robot.id=bi_so102 \
  --robot.left_arm_config.port=/dev/so102_follower_left \
  --robot.right_arm_config.port=/dev/so102_follower_right

```


팔로워암 (라즈베리파이)
```bash
lerobot-calibrate \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/so102_leader_left \
  --teleop.right_arm_config.port=/dev/so102_leader_right
```

`--robot.id` · `--teleop.id` 는 꼭 줍니다. 보정 파일 이름이 `<id>_left.json` · `<id>_right.json` 입니다
(`~/.cache/huggingface/lerobot/calibration/robots/so102_follower/bi_so102_left.json` 등).
기존 양팔 코드로 보정한 키트는 같은 id 를 주면 그 파일을 그대로 씁니다.
Pi 분리 구성에서는 팔로워 보정을 Pi 에서 합니다.

## 4. 텔레오퍼레이션 — 팔 4개를 PC 에

리더를 팔로워와 비슷한 자세로 맞춘 뒤 실행합니다. `Ctrl+C` 로 끝냅니다.

```bash
lerobot-teleoperate \
  --robot.type=dual_so102_follower \
  --robot.id=bi_so102 \
  --robot.left_arm_config.port=/dev/so102_follower_left \
  --robot.right_arm_config.port=/dev/so102_follower_right \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/so102_leader_left \
  --teleop.right_arm_config.port=/dev/so102_leader_right \
  --display_data=false
```

## 5. Pi 분리 — 라즈베리파이 · PC 에서 칠 명령

팔로워 양팔 · 카메라 3대는 라즈베리파이에, 리더 양팔은 PC 에 꽂습니다. Pi 에서 host 를 먼저 띄우고, PC 에서는
로봇 타입만 `dual_so102_client` 로 바꿔 공식 명령을 씁니다. 주소 · 포트는 키트 1호(유선 연결) 기준 예시입니다.

### 라즈베리파이 (팔로워 · 카메라)

PC 에서 접속합니다 (유선 `10.42.0.78`, AP 모드면 `192.168.4.1`). 로그인하면 `lerobot060_venv` 가 켜집니다.

```bash
ssh roboseasy@10.42.0.78
```

```bash
dual-so102-host \
  --robot.id=bi_so102 \
  --robot.left_arm_config.port=/dev/so102_follower_left \
  --robot.right_arm_config.port=/dev/so102_follower_right \
  --robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": "/dev/cam_top", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}, "cam_left": {"type": "opencv", "index_or_path": "/dev/cam_left", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}, "cam_right": {"type": "opencv", "index_or_path": "/dev/cam_right", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}}'
```

- `명령을 기다립니다` 가 찍히면 준비된 것입니다. 연결될 때 팔을 받쳐 주세요.
- 기본은 리더와 같은 쪽을 그대로 따라 합니다. 거울 모드는 끝에 `--robot.mirror_mode=true` 를 붙입니다.
- `Ctrl+C` 로 끝내면 토크가 풀리니 팔을 받칩니다. SSH 가 끊겨도 계속 돌게 하려면 `tmux` 안에서 실행합니다.
- 다른 host 가 이미 돌고 있으면 팔을 건드리지 않고 `포트 5555 사용 중` 으로 멈춥니다. 그 host 를 먼저 끕니다.
- 코드를 갱신할 때만 `cd ~/workspace/dual-so-102 && git pull` 을 합니다.

### PC (리더) — `conda activate lerobot`

**텔레옵:**

```bash
lerobot-teleoperate \
  --robot.type=dual_so102_client \
  --robot.remote_ip=10.42.0.78 \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/bi_so102_leaders_left \
  --teleop.right_arm_config.port=/dev/bi_so102_leaders_right \
  --display_data=false
```

**카메라 화면을 보며 텔레옵** — `--robot.cameras` 를 Pi 와 같은 이름 · 크기로 적어야 client 가 프레임을 받아 rerun 창에
띄웁니다. Pi 는 30 fps 로 보내므로 `--fps=30` 으로 맞춥니다 (텔레옵 기본은 60).

```bash
lerobot-teleoperate \
  --robot.type=dual_so102_client \
  --robot.remote_ip=10.42.0.78 \
  --robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/bi_so102_leaders_left \
  --teleop.right_arm_config.port=/dev/bi_so102_leaders_right \
  --fps=30 \
  --display_data=true
```

**녹화** — 카메라 이름 · 크기는 Pi 와 같게 적습니다 (PC 는 카메라를 열지 않으므로 `index_or_path` 는 아무 값이나 됩니다).

```bash
lerobot-record \
  --robot.type=dual_so102_client \
  --robot.remote_ip=10.42.0.78 \
  --robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/bi_so102_leaders_left \
  --teleop.right_arm_config.port=/dev/bi_so102_leaders_right \
  --dataset.repo_id=local/dual_so102_pick \
  --dataset.single_task="블록을 상자에 넣기" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false \
  --display_data=true
```

**재생** — LeRobot 0.6.0 은 녹화할 때마다 이름 끝에 날짜 · 시각을 붙여 저장합니다
(예: 폴더 `~/.cache/huggingface/lerobot/local/dual_so102_pick_20261118_211947`). 재생 · 학습에는 붙은 이름까지 씁니다.

```bash
lerobot-replay \
  --robot.type=dual_so102_client \
  --robot.remote_ip=10.42.0.78 \
  --dataset.repo_id=local/dual_so102_pick_20261118_211947 \
  --dataset.episode=0
```

- `--robot.remote_ip` 는 PC 와 Pi 를 잇는 방식에 따라 다릅니다.

  | PC ↔ Pi 연결 | `--robot.remote_ip` |
  |---|---|
  | 유선 (PC 의 "다른 컴퓨터와 공유" 연결) | Pi 에서 `hostname -I` 로 확인 (키트 1호: `10.42.0.78`) |
  | Pi 의 AP 모드 와이파이(`dualso102-NN`)에 PC 접속 | `192.168.4.1` |
  | 같은 공유기 | Pi 에서 `hostname -I` 로 확인. Pi 에서 `setip <킷 번호>` 를 하면 `192.168.0.(100+킷 번호)` 로 고정 |

- 리더 포트는 Physical Labs 앱이 PC 에 만든 별칭(`/dev/bi_so102_leaders_*`)입니다. [USB 포트 고정](docs/2.ports.md)의
  예시 규칙으로 만들었다면 `/dev/so102_leader_left` · `/dev/so102_leader_right` 입니다.
- 처음 실행하기 전에 리더를 팔로워와 비슷한 자세로 맞춥니다.
- PC 는 Pi host 가 준비될 때까지 60초 기다립니다 (`--robot.connect_timeout_s`). Pi host 는 카메라 3대를 여는 데
  약 16초 걸리니, Pi 에서 `명령을 기다립니다` 가 찍힌 뒤 PC 명령을 실행하면 바로 붙습니다.
- Hugging Face 에 올리려면 `--dataset.repo_id=<HF 아이디>/dual_so102_pick` 과 `--dataset.push_to_hub=true` 를 씁니다.
  학습 · 추론은 [사용법](docs/1.usage.md), 카메라 · 운용 규칙은 [Pi 분리 구성](docs/3.pi.md)에 있습니다.

## 안전

- 끝내면 토크가 풀립니다 (공식 동작). **팔을 받친 채** 끝냅니다. Pi 분리 구성에서는 PC 를 끝내도 팔이
  자세를 유지하고, Pi 의 host 를 끝낼 때(Ctrl+C · `kill` · SSH 끊김) 토크가 풀립니다.
- host 에는 인증이 없습니다. 같은 네트워크의 누구나 명령을 보낼 수 있으므로 **폐쇄망에서만** 씁니다.
- 팔별 `--robot.left_arm_config.max_relative_target` · `--robot.right_arm_config.max_relative_target` 은 기본값(제한 없음)으로
  둡니다. 5 로 두면 30 Hz 에서 관절 속도가 150°/s 로 잘려 팔로워가 리더를 둔하게 따라옵니다.
- 첫 연결 전에 리더를 팔로워와 비슷한 자세로 맞춥니다. 다르면 첫 명령에서 팔이 크게 움직입니다.

## 문서

[사용법](docs/1.usage.md) · [USB 포트 고정](docs/2.ports.md) · [Pi 분리 구성](docs/3.pi.md) ·
[문제 해결](docs/4.troubleshooting.md) · [개발 · 시험](docs/5.development.md)

## 검증 기록

- 가짜 버스 시험 50개(공식 LeRobot 명령 · 파서 · 제어 루프를 그대로 돌리고 시리얼만 가짜): PC LeRobot 0.6.0
  (`so-102` 플러그인이 같이 있는 환경 · 없는 환경)과 Pi(기존 코드를 덮어쓴 LeRobot 과 같이 설치) 모두 통과.
  Pi 기본 설치에는 녹화 패키지가 없어 녹화 시험 6개는 건너뜁니다.
- 실장비 (2026-11-18, 키트 1호: Pi 4 에 팔로워 2 + 카메라 3, PC 에 리더 2, 유선 연결):
  - `dual-so102-check` — Pi 팔로워 2대, PC 리더 2대 모두 `ids=1..7 OK`.
  - 기존 양팔 코드의 host + 새 `dual_so102_client` — 관절 14개 · 카메라 3대를 받고 텔레옵 동작 (전송 형식 호환).
  - 새 `dual-so102-host`(카메라 3대 MJPG) + PC client — 텔레옵 60초, 녹화 1 에피소드(597 프레임 · 30 fps ·
    카메라 3대), replay.
  - Pi 팔로워의 기존 보정 파일(`bi_so102_{left,right}.json`)을 다시 보정하지 않고 그대로 사용.
  - host 를 `kill -TERM` 으로 끝내면 카메라 · 두 팔 버스를 정리(토크 해제)하고 종료 코드 0.
  - 실장비에서 찾아 고친 것: Pi USB 가 흔들려 카메라가 빠지면 host 가 끝나 토크가 풀리던 문제, 연결 중 순간
    통신 오류 한 번에 연결이 실패하고 정리 없이 끝나던 문제, 녹화 이름에 붙는 날짜 · 시각을 몰랐던 문서.
- 아직 실장비로 확인하지 않은 것: PC 직결(팔 4개를 PC 에), 새 host 의 미러 모드(기존 host 에서는 동작),
  `lerobot-rollout` 추론.
