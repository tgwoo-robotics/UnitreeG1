# G1 Inspire Pipeline 워크플로우

## 전체 파이프라인

```
데이터 수집 → 학습 → 검증 → 배포
(XR Teleop)   (RL/IL) (MuJoCo) (Real Robot)
```

---

## 1. 데이터 수집 (XR Teleoperation)

### 지원 장치
- Meta Quest 3
- Apple Vision Pro
- PICO 4 Ultra

### 실행 방법
```bash
cd external/xr_teleoperate

# 텔레오퍼레이션 시작
python teleop.py --robot g1 --hand inspire

# 데이터는 JSON 형식으로 저장됨
# data/episode_0/, data/episode_1/, ...
```

### 데이터 형식 변환 (LeRobot)
```bash
cd external/unitree_IL_lerobot

# JSON → LeRobot v3.0 형식
python convert_unitree_json_to_lerobot.py \
    --input ../xr_teleoperate/data/ \
    --output lerobot_dataset/ \
    --hand inspire  # inspire, dex3, brainco
```

---

## 2. 학습

### 2.1 강화학습 (RL) - 보행

```bash
cd external/unitree_rl_gym

# Isaac Gym 환경 필요
# https://developer.nvidia.com/isaac-gym

# G1 보행 학습
python legged_gym/scripts/train.py \
    --task=g1 \
    --num_envs=4096 \
    --headless

# 결과: logs/g1/<timestamp>/model_<iter>.pt
```

### 2.2 모방학습 (IL) - 조작

```bash
cd external/unitree_IL_lerobot

# ACT (Action Chunking Transformer)
python train.py \
    --policy act \
    --dataset lerobot_dataset/ \
    --epochs 100

# Diffusion Policy
python train.py \
    --policy diffusion \
    --dataset lerobot_dataset/

# 결과: checkpoints/act_latest.pt
```

---

## 3. 검증 (Sim2Sim)

### MuJoCo에서 정책 검증

```bash
# unitree_rl_gym 정책 검증
cd external/unitree_rl_gym
python deploy/deploy_mujoco/deploy_mujoco.py g1

# 또는 간단한 시뮬레이션
cd /path/to/G1
python scripts/run_mujoco_sim.py --model g1_29dof --mode viewer
```

### unitree_mujoco (DDS 브릿지)

```bash
cd external/unitree_mujoco/simulate_python

# config.py 수정
# ROBOT = "g1"
# ROBOT_SCENE = "unitree_robots/g1/scene_29dof.xml"

# 시뮬레이터 실행
python unitree_mujoco.py

# 별도 터미널에서 제어 프로그램 실행
# (동일한 DDS API로 실제 로봇과 통신 가능)
```

---

## 4. 실제 로봇 배포

### 4.1 네트워크 설정

```bash
# PC 이더넷 설정 (G1과 같은 서브넷)
sudo ip addr add 192.168.123.200/24 dev eth0

# G1 SSH 접속
ssh unitree@192.168.123.164  # 비밀번호: 123
```

### 4.2 LeRobot 방식 (권장)

```bash
# [G1 Orin에서] 서버 실행
cd lerobot
python src/lerobot/robots/unitree_g1/run_g1_server.py

# [PC에서] 정책 실행
cd external/lerobot

# GR00T 보행 정책
python examples/unitree_g1/gr00t_locomotion.py \
    --repo-id "nepyope/GR00T-WholeBodyControl_g1"

# Holosoma 보행 정책
python examples/unitree_g1/holosoma_locomotion.py
```

### 4.3 직접 배포 (unitree_rl_gym)

```bash
cd external/unitree_rl_gym

# 실제 로봇 배포
python deploy/deploy_real/deploy.py g1 \
    --policy logs/g1/latest/model.pt
```

---

## 5. Inspire Hand 사용

### 모델 경로
```
external/unitree_ros/robots/g1_description/
├── g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf  # Force-Torque
└── g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf  # Simplified
```

### MuJoCo에서 로드
```python
import mujoco

# URDF 직접 로드 (actuator 없음, 시각화용)
model = mujoco.MjModel.from_xml_path(
    "external/unitree_ros/robots/g1_description/"
    "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"
)
```

### 손 관절 구조 (12 DoF per hand)
- Thumb: 4 joints (회전 + 3단 굴곡)
- Index: 2 joints
- Middle: 2 joints
- Ring: 2 joints
- Little: 2 joints

---

## 사전학습 모델

| 모델 | 용도 | HuggingFace |
|------|------|-------------|
| GR00T-WholeBodyControl | 전신 보행 | `nepyope/GR00T-WholeBodyControl_g1` |
| Holosoma | 보행 | Amazon FAR |

---

## 참고 링크

- [LeRobot G1 가이드](https://huggingface.co/docs/lerobot/unitree_g1)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)
- [MuJoCo](https://mujoco.readthedocs.io/)
