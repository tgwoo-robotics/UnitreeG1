# G1 Inspire Pipeline

Unitree G1 EDU + Inspire Hand (5-finger) 로봇 제어 파이프라인

## 빠른 시작

```bash
# 1. 클론 및 서브모듈 초기화
git clone https://github.com/tgwoo-robotics/UnitreeG1.git
cd UnitreeG1
git submodule update --init --recursive

# 2. Conda 환경 생성
conda create -n G1 python=3.10 pinocchio=3.1.0 -c conda-forge -y

# 3. 패키지 설치
conda activate G1
pip install mujoco pygame pynput meshcat sshkeyboard
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
pip install -e external/xr_teleoperate/teleop/televuer/
pip install -r external/xr_teleoperate/requirements.txt

# 4. libstdc++ 호환 설정 (필수)
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'export LD_PRELOAD=$CONDA_PREFIX/lib/libstdc++.so.6' > $CONDA_PREFIX/etc/conda/activate.d/fix_libstdcxx.sh

# 5. NumPy 다운그레이드 (pinocchio 호환)
pip install 'numpy<2'

# 6. 시뮬레이션 테스트
cd external/unitree_mujoco/simulate_python
python unitree_mujoco.py
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
│   └── hybrid_televuer.py         # 하이브리드 TeleVuer
│
├── configs/                     # 설정 파일
├── docs/                        # 문서
├── setup.sh                     # 환경 설정 스크립트 (기본)
└── requirements.txt             # Python 의존성
```

## Conda 환경 설정

### 자동 설정 (권장)

```bash
# 환경 생성 (pinocchio + casadi 바인딩 포함)
conda create -n G1 python=3.10 pinocchio=3.1.0 -c conda-forge -y
conda activate G1

# 필수 패키지
pip install mujoco pygame pynput meshcat sshkeyboard
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
pip install -e external/xr_teleoperate/teleop/televuer/
pip install -r external/xr_teleoperate/requirements.txt

# libstdc++ 호환 설정
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'export LD_PRELOAD=$CONDA_PREFIX/lib/libstdc++.so.6' > $CONDA_PREFIX/etc/conda/activate.d/fix_libstdcxx.sh

# NumPy 다운그레이드 (pinocchio 호환)
pip install 'numpy<2'
```

### 환경 검증

```bash
conda activate G1
python -c "
from pinocchio import casadi as cpin
print('pinocchio + casadi: OK')
import mujoco
print('mujoco: OK')
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
print('unitree_sdk2py: OK')
"
```

## 시뮬레이션

### MuJoCo 시뮬레이션 시작

```bash
conda activate G1
cd external/unitree_mujoco/simulate_python
python unitree_mujoco.py
```

### 상체 키보드 제어

```bash
# 다른 터미널
conda activate G1
python scripts/sim_upper_body_control.py
```

**조작키:**
| 키 | 기능 |
|----|------|
| `↑/↓` | 선택된 관절 +/- 이동 |
| `←/→` | 다른 관절 선택 |
| `1` | 왼팔 인사 자세 |
| `2` | 오른팔 인사 자세 |
| `3` | 양팔 들기 |
| `0` | 기본 자세 |
| `ESC` | 종료 |

**시뮬레이션 창 키:**
| 키 | 기능 |
|----|------|
| `7/8` | ElasticBand 길이 조절 |
| `9` | ElasticBand 켜기/끄기 |

### ElasticBand

시뮬레이션에서는 실제 로봇의 내장 균형 제어기가 없습니다.
**ElasticBand**가 로봇을 가상 밴드로 매달아 균형을 유지합니다.

설정: `external/unitree_mujoco/simulate_python/config.py`
```python
ENABLE_ELASTIC_BAND = True  # 균형 유지
```

## XR 텔레오퍼레이션 (Meta Quest)

### 기본 실행

```bash
conda activate G1
cd external/xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm G1_29 --ee inspire_ftp
```

### 옵션

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--arm` | 로봇 타입 | `G1_29` |
| `--ee` | 손 타입 | `inspire_ftp` |
| `--input-mode` | 입력 모드 (`hand`/`controller`) | `hand` |
| `--display-mode` | 디스플레이 모드 | `immersive` |
| `--sim` | 시뮬레이션 모드 | - |
| `--motion` | 하체 이동 활성화 | - |
| `--headless` | 헤드리스 모드 | - |

### 하이브리드 텔레옵 (손 추적 + 컨트롤러)

```bash
python scripts/teleop_hybrid.py --arm G1_29 --ee inspire_ftp
```

**기능:**
- **손 추적**: 손 움직임 → 로봇 팔/손 제어
- **헤드셋 이동 감지**: 실제로 걸어다니면 로봇도 이동
- **컨트롤러 조이스틱**: 추가 이동/회전 제어

## G1 관절 구조 (29 DOF)

| 인덱스 | 관절 |
|--------|------|
| 0-5 | 왼쪽 다리 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll) |
| 6-11 | 오른쪽 다리 |
| 12-14 | 허리 (yaw, roll, pitch) |
| 15-21 | 왼팔 (shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw) |
| 22-28 | 오른팔 |

## DDS 통신

| Domain ID | 용도 |
|-----------|------|
| 0 | 실제 로봇 |
| 1 | 시뮬레이션 |

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
```

## 실제 로봇 배포

```bash
# G1 Orin에서 서버 실행
ssh unitree@192.168.123.164
python external/lerobot/src/lerobot/robots/unitree_g1/run_g1_server.py

# PC에서 정책 실행
python scripts/run_lerobot.py --policy gr00t
```

## 트러블슈팅

### 1. libstdc++ 버전 오류
```
ImportError: CXXABI_1.3.15 not found
```
**해결:**
```bash
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'export LD_PRELOAD=$CONDA_PREFIX/lib/libstdc++.so.6' > $CONDA_PREFIX/etc/conda/activate.d/fix_libstdcxx.sh
conda deactivate && conda activate G1
```

### 2. unitree_sdk2py import 오류
```
ImportError: cannot import name 'b2'
```
**해결:** `unitree_sdk2py/__init__.py`에서 b2 import 제거
```python
# 수정 전
from . import idl, utils, core, rpc, go2, b2
# 수정 후
from . import idl, utils, core, rpc, go2
```

### 3. NumPy 버전 충돌
```
A module compiled using NumPy 1.x cannot run in NumPy 2.x
```
**해결:**
```bash
pip install 'numpy<2'
```

## 참고 자료

- [LeRobot G1 가이드](https://huggingface.co/docs/lerobot/unitree_g1)
- [Unitree SDK2 문서](https://github.com/unitreerobotics/unitree_sdk2_python)
- [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [Isaac Lab 문서](https://isaac-sim.github.io/IsaacLab/)
