# G1 시뮬레이션 테스트 가이드

## 의존성 설치

```bash
# MuJoCo
pip install mujoco

# 키보드 제어용
pip install pynput

# Unitree SDK2 Python
pip install unitree_sdk2py

# xr_teleoperate 의존성
pip install vuer numpy opencv-python
```

---

## 1. 키보드 제어 테스트 (Meta Quest 없이)

### Terminal 1: MuJoCo 시뮬레이션 실행
```bash
cd /home/taegyu/G1
python scripts/run_simulation.py
```

### Terminal 2: 키보드 제어
```bash
cd /home/taegyu/G1
python scripts/teleop_keyboard.py
```

### 조작법
| 키 | 동작 |
|----|------|
| W | 전진 |
| S | 후진 |
| A | 좌측 이동 |
| D | 우측 이동 |
| Q | 좌회전 |
| E | 우회전 |
| ESC | 종료 |

---

## 2. Meta Quest 텔레오퍼레이션 테스트

### 사전 준비
1. Meta Quest가 PC와 같은 네트워크에 연결
2. SSL 인증서 설정 (`~/.config/xr_teleoperate/`)
3. Image server 실행 (또는 pass-through 모드 사용)

### Terminal 1: MuJoCo 시뮬레이션
```bash
cd /home/taegyu/G1
python scripts/run_simulation.py
```

### Terminal 2: 하이브리드 텔레오퍼레이션
```bash
cd /home/taegyu/G1
python scripts/teleop_hybrid.py --sim --display-mode pass-through
```

### 조작법
| 입력 | 동작 |
|------|------|
| 손 추적 | 로봇 팔/손 제어 |
| 실제로 걷기 | 로봇 이동 (헤드셋 속도 감지) |
| 왼쪽 조이스틱 | 전진/후진, 좌/우 |
| 오른쪽 조이스틱 | 회전 |
| 양쪽 조이스틱 동시 클릭 | 비상 정지 |
| A 버튼 | 종료 |

---

## 3. 빠른 시작 스크립트

```bash
# 키보드 모드
./scripts/start_sim_test.sh keyboard

# Meta Quest 모드
./scripts/start_sim_test.sh quest
```

---

## 옵션

### run_simulation.py
| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--robot` | g1 | 로봇 종류 |
| `--scene` | auto | Scene XML 경로 |
| `--domain-id` | 1 | DDS Domain ID |

### teleop_hybrid.py
| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--sim` | - | 시뮬레이션 모드 |
| `--arm` | G1_29 | 로봇 팔 타입 |
| `--ee` | inspire_ftp | 손 타입 |
| `--head-velocity-scale` | 1.0 | 헤드셋 이동 속도 스케일 |
| `--no-locomotion` | - | 하체 비활성화 |

### teleop_keyboard.py
| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--linear-speed` | 0.2 | 이동 속도 (m/s) |
| `--angular-speed` | 0.3 | 회전 속도 (rad/s) |

---

## 문제 해결

### DDS 통신 오류
```bash
# 네트워크 인터페이스 확인
ip addr

# loopback 사용 (시뮬레이션)
python teleop_keyboard.py --network-interface lo
```

### MuJoCo 렌더링 오류
```bash
# DISPLAY 환경변수 확인
echo $DISPLAY

# headless 모드
python scripts/run_mujoco_sim.py --mode headless
```
