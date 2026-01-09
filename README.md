# G1 Inspire Pipeline

Unitree G1 EDU + Inspire Hand (5-finger) 로봇 제어 파이프라인

## 프로젝트 구조

```
G1/
├── external/                    # 공식 저장소 (git submodules)
│   ├── unitree_mujoco/         # MuJoCo 시뮬레이션 + DDS 브릿지
│   ├── unitree_rl_gym/         # Isaac Gym RL 학습
│   ├── unitree_IL_lerobot/     # 모방학습 (ACT, Diffusion)
│   ├── xr_teleoperate/         # XR 텔레오퍼레이션
│   ├── lerobot/                # HuggingFace LeRobot
│   ├── unitree_ros/            # G1 + Inspire Hand URDF
│   └── mujoco_menagerie/       # MuJoCo 로봇 모델
│
├── configs/                     # 프로젝트 설정
│   └── g1_inspire.yaml
│
├── scripts/                     # 실행 스크립트
│   ├── setup_env.sh            # 환경 설정
│   ├── run_mujoco_sim.py       # MuJoCo 시뮬레이션
│   └── run_lerobot.py          # LeRobot 실행
│
└── docs/                        # 문서
```

## 설치

### 1. 저장소 클론 및 서브모듈 초기화

```bash
git clone https://github.com/your-repo/G1.git
cd G1
git submodule update --init --recursive
```

### 2. 환경 설정

```bash
# Conda 환경 생성
conda create -n g1 python=3.10 -y
conda activate g1

# 기본 의존성
pip install -r requirements.txt

# LeRobot (G1 지원 포함)
cd external/lerobot
pip install -e '.[unitree_g1]'

# Unitree SDK2
cd ../unitree_mujoco
pip install -e unitree_sdk2_python/
```

## 워크플로우

### 1. MuJoCo 시뮬레이션 (검증용)

```bash
# 기본 시뮬레이션
python scripts/run_mujoco_sim.py

# Inspire Hand 모델
python scripts/run_mujoco_sim.py --model inspire
```

### 2. RL 학습 (Isaac Gym)

```bash
cd external/unitree_rl_gym

# G1 보행 학습
python legged_gym/scripts/train.py --task=g1

# MuJoCo에서 검증
python deploy/deploy_mujoco/deploy_mujoco.py g1
```

### 3. 모방학습 (LeRobot)

```bash
cd external/unitree_IL_lerobot

# 데이터 변환 (xr_teleoperate에서 수집한 데이터)
python convert_unitree_json_to_lerobot.py --input data/ --output lerobot_data/

# ACT 학습
python train.py --policy act --dataset lerobot_data/

# 평가
python eval_g1.py --policy checkpoints/act_latest.pt
```

### 4. 실제 로봇 배포 (LeRobot 공식)

```bash
# G1 Orin에서 서버 실행
ssh unitree@192.168.123.164
python external/lerobot/src/lerobot/robots/unitree_g1/run_g1_server.py

# PC에서 정책 실행
python scripts/run_lerobot.py --policy gr00t
```

### 5. XR 텔레오퍼레이션

```bash
cd external/xr_teleoperate

# Quest 3 텔레옵 시작
python teleop.py --robot g1 --hand inspire
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
- [Isaac Lab 문서](https://isaac-sim.github.io/IsaacLab/)
