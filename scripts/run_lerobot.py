#!/usr/bin/env python3
"""
LeRobot G1 Runner

HuggingFace LeRobot을 사용하여 G1 로봇 제어
- 사전학습 모델 실행 (GR00T, Holosoma)
- 시뮬레이션 모드 지원
"""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "external/lerobot"))


def run_gr00t_locomotion(robot_ip: str, is_simulation: bool = False):
    """GR00T-WholeBodyControl 보행 정책 실행"""
    print("Running GR00T-WholeBodyControl locomotion...")
    print("")
    print("실행 방법:")
    print(f"  cd {PROJECT_ROOT / 'external/lerobot'}")
    print("")

    if is_simulation:
        print("  # 시뮬레이션 모드")
        print("  # config_unitree_g1.py에서 is_simulation=True 설정 후:")
        print('  python examples/unitree_g1/gr00t_locomotion.py --repo-id "nepyope/GR00T-WholeBodyControl_g1"')
    else:
        print("  # 1) G1 Orin에서 서버 실행")
        print(f"  ssh unitree@{robot_ip}")
        print("  python src/lerobot/robots/unitree_g1/run_g1_server.py")
        print("")
        print("  # 2) PC에서 정책 실행")
        print(f"  # config_unitree_g1.py에서 robot_ip='{robot_ip}' 설정 후:")
        print('  python examples/unitree_g1/gr00t_locomotion.py --repo-id "nepyope/GR00T-WholeBodyControl_g1"')


def run_holosoma_locomotion(robot_ip: str, is_simulation: bool = False):
    """Holosoma 보행 정책 실행"""
    print("Running Holosoma locomotion...")
    print("")
    print("실행 방법:")
    print(f"  cd {PROJECT_ROOT / 'external/lerobot'}")
    print("")

    if is_simulation:
        print("  # 시뮬레이션 모드")
        print("  python examples/unitree_g1/holosoma_locomotion.py")
    else:
        print("  # 1) G1 Orin에서 서버 실행")
        print(f"  ssh unitree@{robot_ip}")
        print("  python src/lerobot/robots/unitree_g1/run_g1_server.py")
        print("")
        print("  # 2) PC에서 정책 실행")
        print("  python examples/unitree_g1/holosoma_locomotion.py")


def show_info():
    """LeRobot G1 정보 표시"""
    print("=" * 60)
    print("LeRobot G1 Integration")
    print("=" * 60)
    print("")
    print("지원 기능:")
    print("  - G1 29 DoF / 23 DoF EDU 버전")
    print("  - ZMQ 소켓 브릿지 (원격 통신)")
    print("  - MuJoCo 시뮬레이션 모드")
    print("  - GR00T-WholeBodyControl 보행 정책")
    print("  - Holosoma 보행 정책")
    print("")
    print("설정 파일:")
    print(f"  {PROJECT_ROOT / 'external/lerobot/src/lerobot/robots/unitree_g1/config_unitree_g1.py'}")
    print("")
    print("주요 설정:")
    print("  robot_ip: str = '...'  # 로봇 WiFi IP")
    print("  is_simulation: bool = True/False")
    print("")
    print("참고:")
    print("  https://huggingface.co/docs/lerobot/unitree_g1")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="LeRobot G1 Runner")
    parser.add_argument(
        "--policy", "-p",
        choices=["gr00t", "holosoma", "info"],
        default="info",
        help="Policy to run"
    )
    parser.add_argument(
        "--robot-ip",
        default="192.168.123.164",
        help="G1 robot IP address"
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Run in simulation mode"
    )
    args = parser.parse_args()

    if args.policy == "info":
        show_info()
    elif args.policy == "gr00t":
        run_gr00t_locomotion(args.robot_ip, args.sim)
    elif args.policy == "holosoma":
        run_holosoma_locomotion(args.robot_ip, args.sim)


if __name__ == "__main__":
    main()
