#!/usr/bin/env python3
"""
G1 Simulation Keyboard Control (Low-level)

MuJoCo 시뮬레이션에서 직접 LowCmd로 로봇 제어
"""

import time
import sys
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "external/unitree_mujoco/simulate_python"))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HG_LowCmd
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HG_LowState
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False

# G1 29DOF Joint indices
# Legs: 0-11 (6 per leg)
# Waist: 12-14 (3 joints)
# Arms: 15-28 (7 per arm)

class G1SimController:
    """G1 시뮬레이션 저수준 제어기"""

    def __init__(self):
        # DDS 초기화
        ChannelFactoryInitialize(1, "lo")

        # Publisher/Subscriber
        self.low_cmd_pub = ChannelPublisher("rt/lowcmd", HG_LowCmd)
        self.low_cmd_pub.Init()

        self.low_state = None
        self.low_state_sub = ChannelSubscriber("rt/lowstate", HG_LowState)
        self.low_state_sub.Init(self._low_state_callback, 10)

        self.crc = CRC()

        # 기본 명령
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.cmd.mode_pr = 0
        self.cmd.mode_machine = 0

        # 모터 초기화 (35개)
        for i in range(35):
            self.cmd.motor_cmd[i].mode = 1  # PMSM mode
            self.cmd.motor_cmd[i].q = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].kd = 0.0

        # 제어 상태
        self.target_positions = np.zeros(29)  # 29 DOF
        self.running = True

        # 키 상태
        self.keys = set()

    def _low_state_callback(self, msg):
        self.low_state = msg

    def set_joint_position(self, joint_idx, position, kp=50.0, kd=2.0):
        """관절 위치 제어"""
        if 0 <= joint_idx < 29:
            self.cmd.motor_cmd[joint_idx].q = position
            self.cmd.motor_cmd[joint_idx].kp = kp
            self.cmd.motor_cmd[joint_idx].kd = kd
            self.cmd.motor_cmd[joint_idx].dq = 0.0
            self.cmd.motor_cmd[joint_idx].tau = 0.0

    def send_command(self):
        """명령 전송"""
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.low_cmd_pub.Write(self.cmd)

    def stand_pose(self):
        """기본 서있는 자세"""
        # 다리 관절 (인덱스 0-11)
        leg_positions = [
            0.0,   # left_hip_pitch
            0.0,   # left_hip_roll
            0.0,   # left_hip_yaw
            0.0,   # left_knee
            0.0,   # left_ankle_pitch
            0.0,   # left_ankle_roll
            0.0,   # right_hip_pitch
            0.0,   # right_hip_roll
            0.0,   # right_hip_yaw
            0.0,   # right_knee
            0.0,   # right_ankle_pitch
            0.0,   # right_ankle_roll
        ]

        # 허리 (인덱스 12-14)
        waist_positions = [0.0, 0.0, 0.0]

        # 팔 (인덱스 15-28)
        arm_positions = [
            0.0,    # left_shoulder_pitch
            0.3,    # left_shoulder_roll
            0.0,    # left_shoulder_yaw
            0.5,    # left_elbow
            0.0,    # left_wrist_roll
            0.0,    # left_wrist_pitch
            0.0,    # left_wrist_yaw
            0.0,    # right_shoulder_pitch
            -0.3,   # right_shoulder_roll
            0.0,    # right_shoulder_yaw
            0.5,    # right_elbow
            0.0,    # right_wrist_roll
            0.0,    # right_wrist_pitch
            0.0,    # right_wrist_yaw
        ]

        positions = leg_positions + waist_positions + arm_positions

        for i, pos in enumerate(positions):
            self.set_joint_position(i, pos, kp=100.0, kd=5.0)

    def wave_arm(self, side='left', angle=0.5):
        """팔 흔들기"""
        if side == 'left':
            # 왼팔 어깨 pitch
            self.set_joint_position(15, angle, kp=50.0, kd=2.0)
        else:
            # 오른팔 어깨 pitch
            self.set_joint_position(22, angle, kp=50.0, kd=2.0)


def main():
    if not PYNPUT_AVAILABLE:
        print("Error: pynput required. Install with: pip install pynput")
        sys.exit(1)

    print("=" * 60)
    print("G1 Simulation Keyboard Control")
    print("=" * 60)
    print("\nControls:")
    print("  1: Stand pose (기본 자세)")
    print("  2: Wave left arm (왼팔 흔들기)")
    print("  3: Wave right arm (오른팔 흔들기)")
    print("  0: Zero pose (모든 관절 0)")
    print("  ESC: Quit")
    print("=" * 60)

    controller = G1SimController()

    # 현재 동작
    current_action = 'stand'
    wave_angle = 0.0
    wave_dir = 1

    def on_press(key):
        nonlocal current_action
        try:
            if key.char == '1':
                current_action = 'stand'
                print("\n[Action] Stand pose")
            elif key.char == '2':
                current_action = 'wave_left'
                print("\n[Action] Wave left arm")
            elif key.char == '3':
                current_action = 'wave_right'
                print("\n[Action] Wave right arm")
            elif key.char == '0':
                current_action = 'zero'
                print("\n[Action] Zero pose")
        except AttributeError:
            pass

    def on_release(key):
        nonlocal controller
        if key == keyboard.Key.esc:
            controller.running = False
            return False

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("\n[Ready] Waiting for simulation connection...")
    time.sleep(1)
    print("[Running] Press keys to control robot\n")

    try:
        while controller.running:
            # 동작 실행
            if current_action == 'stand':
                controller.stand_pose()
            elif current_action == 'zero':
                for i in range(29):
                    controller.set_joint_position(i, 0.0, kp=50.0, kd=2.0)
            elif current_action == 'wave_left':
                controller.stand_pose()
                wave_angle += 0.05 * wave_dir
                if wave_angle > 1.5 or wave_angle < -0.5:
                    wave_dir *= -1
                controller.wave_arm('left', wave_angle)
            elif current_action == 'wave_right':
                controller.stand_pose()
                wave_angle += 0.05 * wave_dir
                if wave_angle > 1.5 or wave_angle < -0.5:
                    wave_dir *= -1
                controller.wave_arm('right', wave_angle)

            controller.send_command()
            time.sleep(0.01)  # 100Hz

    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        listener.stop()
        print("[Done]")


if __name__ == "__main__":
    main()
