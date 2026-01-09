# G1 Inspire Pipeline

Unitree G1 EDU + Inspire Hand (5-finger) 로봇 제어 파이프라인

## 빠른 시작

```bash
# 1. 클론 및 서브모듈 초기화
git clone https://github.com/tgwoo-robotics/UnitreeG1.git
cd UnitreeG1
git submodule update --init --recursive

# 2. 환경 설정 (자동)
chmod +x setup.sh
./setup.sh

# 3. 시뮬레이션 시작
cd external/unitree_mujoco/simulate_python
python unitree_mujoco.py

# 4. 새 터미널에서 상체 제어
python scripts/sim_upper_body_control.py
```

## 프로젝트 구조

```
G1/
├── external/                    # 공식 저장소 (git submodules)
│   ├── unitree_mujoco/         # MuJoCo 시뮬레이션 + DDS 브릿지
│   ├── unitree_rl_gym/         # Isaac Gym RL 학습
│   ├── unitree_IL_lerobot/     # 모방학습 (ACT, Diffusion)
│   ├── xr_teleoperate/         # XR 텔레오퍼레이션 (Meta Quest)
│   ├── lerobot/                # HuggingFace LeRobot
│   ├── unitree_ros/            # G1 + Inspire Hand URDF
│   └── mujoco_menagerie/       # MuJoCo 로봇 모델
│
├── scripts/                     # 실행 스크립트
│   ├── sim_upper_body_control.py  # 상체 키보드 제어 (시뮬레이션)
│   ├── sim_keyboard_control.py    # 전체 키보드 제어 (시뮬레이션)
│   ├── teleop_hybrid.py           # 하이브리드 텔레옵 (XR + 컨트롤러)
│   ├── hybrid_televuer.py         # 하이브리드 TeleVuer
│   ├── run_simulation.py          # 시뮬레이션 설정/실행
│   └── run_lerobot.py             # LeRobot 실행
│
├── configs/                     # 설정 파일
│   └── g1_inspire.yaml
│
├── docs/                        # 문서
│   └── SIMULATION.md
│
├── setup.sh                     # 환경 설정 스크립트
└── requirements.txt             # Python 의존성
```

## 설치

### 자동 설치 (권장)

```bash
# Conda 환경 생성
conda create -n g1 python=3.10 -y
conda activate g1

# 자동 설치
./setup.sh
```

### 수동 설치

```bash
# 1. 서브모듈 초기화
git submodule update --init --recursive

# 2. 기본 의존성
pip install -r requirements.txt

# 3. Unitree SDK2
cd external/unitree_mujoco/unitree_sdk2_python
pip install -e .
cd ../../..

# 4. (선택) LeRobot
cd external/lerobot
pip install -e '.[unitree_g1]'
cd ../..

# 5. (선택) XR Teleoperation
pip install -r external/xr_teleoperate/requirements.txt
```

## 시뮬레이션

### MuJoCo 시뮬레이션

```bash
# 터미널 1: 시뮬레이션 시작
cd external/unitree_mujoco/simulate_python
python unitree_mujoco.py

# 터미널 2: 상체 제어
python scripts/sim_upper_body_control.py
```

**상체 제어 키:**
- `↑/↓`: 선택된 관절 +/- 이동
- `←/→`: 다른 관절 선택
- `1`: 왼팔 인사 자세
- `2`: 오른팔 인사 자세
- `3`: 양팔 들기
- `0`: 기본 자세
- `ESC`: 종료

**시뮬레이션 창 키:**
- `7/8`: ElasticBand 길이 조절
- `9`: ElasticBand 켜기/끄기

### ElasticBand

시뮬레이션에서는 실제 로봇처럼 내장 균형 제어기가 없습니다.
**ElasticBand**는 로봇을 가상 밴드로 매달아서 넘어지지 않게 해줍니다.

`external/unitree_mujoco/simulate_python/config.py`에서:
```python
ENABLE_ELASTIC_BAND = True  # 균형 유지
```

## XR 텔레오퍼레이션 (Meta Quest)

### 하이브리드 텔레옵

손 추적 + 컨트롤러 동시 사용:

```bash
python scripts/teleop_hybrid.py --arm G1_29 --ee inspire_ftp
```

**기능:**
- **손 추적**: 손 움직임 → 로봇 팔/손 제어
- **헤드셋 이동 감지**: 실제로 걸어다니면 로봇도 이동
- **컨트롤러 조이스틱**: 추가 이동/회전 제어

**옵션:**
- `--sim`: 시뮬레이션 모드
- `--no-locomotion`: 하체 제어 비활성화 (상체만)
- `--head-velocity-scale`: 헤드셋 이동 속도 스케일
- `--ctrl-velocity-scale`: 컨트롤러 속도 스케일

## RL 학습 (Isaac Gym)

```bash
cd external/unitree_rl_gym

# G1 보행 학습
python legged_gym/scripts/train.py --task=g1

# MuJoCo에서 검증
python deploy/deploy_mujoco/deploy_mujoco.py g1.yaml
```

## 모방학습 (LeRobot)

```bash
cd external/unitree_IL_lerobot

# 데이터 변환
python convert_unitree_json_to_lerobot.py --input data/ --output lerobot_data/

# ACT 학습
python train.py --policy act --dataset lerobot_data/

# 평가
python eval_g1.py --policy checkpoints/act_latest.pt
```

## 실제 로봇 배포

```bash
# G1 Orin에서 서버 실행
ssh unitree@192.168.123.164
python external/lerobot/src/lerobot/robots/unitree_g1/run_g1_server.py

# PC에서 정책 실행
python scripts/run_lerobot.py --policy gr00t
```

## 로봇 모델

| 모델 | 경로 | DOF |
|------|------|-----|
| G1 기본 | `external/mujoco_menagerie/unitree_g1/` | 29 |
| G1 + 기본손 | `external/unitree_mujoco/unitree_robots/g1/` | 43 |
| G1 + Inspire Hand | `external/unitree_ros/robots/g1_description/` | 53 |

## 사전학습 모델

| 모델 | 용도 | 실행 |
|------|------|------|
| GR00T-WholeBodyControl | 보행 | `--repo-id nepyope/GR00T-WholeBodyControl_g1` |
| Holosoma | 보행 | `holosoma_locomotion.py` |

## 참고 자료

- [LeRobot G1 가이드](https://huggingface.co/docs/lerobot/unitree_g1)
- [Unitree SDK2 문서](https://github.com/unitreerobotics/unitree_sdk2_python)
- [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [Isaac Lab 문서](https://isaac-sim.github.io/IsaacLab/)
