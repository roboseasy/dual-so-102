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
아래 예시는 모두 이 별칭을 씁니다.

## 3. 보정

팔을 받친 상태에서 안내를 따릅니다. 왼팔을 먼저, 이어서 오른팔을 보정합니다.

```bash
lerobot-calibrate \
  --robot.type=dual_so102_follower \
  --robot.id=bi_so102 \
  --robot.left_arm_config.port=/dev/so102_follower_left \
  --robot.right_arm_config.port=/dev/so102_follower_right

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

## 5. Pi 분리 — 팔로워 · 카메라는 Pi, 리더는 PC

Pi 에서 host 를 띄웁니다 (카메라 3대를 MJPG 로 엽니다).

```bash
dual-so102-host \
  --robot.id=bi_so102 \
  --robot.left_arm_config.port=/dev/so102_follower_left \
  --robot.right_arm_config.port=/dev/so102_follower_right \
  --robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": "/dev/cam_top", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}, "cam_left": {"type": "opencv", "index_or_path": "/dev/cam_left", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}, "cam_right": {"type": "opencv", "index_or_path": "/dev/cam_right", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}}'
```

PC 에서는 로봇 타입만 `dual_so102_client` 로 바꿉니다. `--robot.remote_ip` 는 Pi 주소입니다.

```bash
lerobot-teleoperate \
  --robot.type=dual_so102_client \
  --robot.remote_ip=192.168.4.1 \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/so102_leader_left \
  --teleop.right_arm_config.port=/dev/so102_leader_right \
  --display_data=false
```

| PC ↔ Pi 연결 | `--robot.remote_ip` |
|---|---|
| Pi 의 AP 모드 와이파이(`dualso102-NN`)에 PC 접속 | `192.168.4.1` |
| 유선(PC 의 "다른 컴퓨터와 공유" 연결) | Pi 에서 `hostname -I` 로 확인 (예: `10.42.0.78`) |
| 같은 공유기 | Pi 에서 `hostname -I` 로 확인, `setip <킷 번호>` 로 고정하면 `192.168.0.(100+킷 번호)` |

카메라 · 미러 모드 · 운용 규칙은 [Pi 분리 구성](docs/3.pi.md)에 있습니다.

## 6. 녹화 · 재생 · 추론

PC 에서 녹화합니다. `--robot.cameras` 는 host 와 같은 이름 · 크기를 적습니다 (PC 는 카메라를 열지 않으므로
`index_or_path` 는 아무 값이나 됩니다). 학습 · 추론 명령은 [사용법](docs/1.usage.md)에 있습니다.

```bash
HF_USER=my_name  # Hugging Face 아이디 (올리지 않으면 아무 이름)

lerobot-record \
  --robot.type=dual_so102_client \
  --robot.remote_ip=192.168.4.1 \
  --robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
  --teleop.type=dual_so102_leader \
  --teleop.id=bi_so102_leader \
  --teleop.left_arm_config.port=/dev/so102_leader_left \
  --teleop.right_arm_config.port=/dev/so102_leader_right \
  --dataset.repo_id=${HF_USER}/dual_so102_test \
  --dataset.single_task="블록을 상자에 넣기" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false \
  --display_data=true

lerobot-replay \
  --robot.type=dual_so102_client \
  --robot.remote_ip=192.168.4.1 \
  --dataset.repo_id=${HF_USER}/dual_so102_test \
  --dataset.episode=0
```

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

- 가짜 버스 시험(공식 LeRobot 명령 · 파서 · 제어 루프를 그대로 돌리고 시리얼만 가짜):
  PC LeRobot 0.6.0 에서 통과.
- 실장비 검증은 아직 하지 않았습니다.
