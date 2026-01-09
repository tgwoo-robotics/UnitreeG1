#!/usr/bin/env python3
"""
G1 Upper Body Control (with ElasticBand for balance)

ElasticBand가 하체 균형을 유지해주고, 키보드로 상체만 제어
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
# Legs: 0-11 (6 per leg) - ElasticBand handles balance
# Waist: 12-14 (3 joints)
# Arms: 15-28 (7 per arm) - We control these

# Joint names for reference
JOINT_NAMES = {
    # Left Arm (15-21)
    15: "left_shoulder_pitch",
    16: "left_shoulder_roll",
    17: "left_shoulder_yaw",
    18: "left_elbow",
    19: "left_wrist_roll",
    20: "left_wrist_pitch",
    21: "left_wrist_yaw",
    # Right Arm (22-28)
    22: "right_shoulder_pitch",
    23: "right_shoulder_roll",
    24: "right_shoulder_yaw",
    25: "right_elbow",
    26: "right_wrist_roll",
    27: "right_wrist_pitch",
    28: "right_wrist_yaw",
}


class G1UpperBodyController:
    """G1 상체 제어기 - ElasticBand로 균형 유지"""

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

        # 상체 타겟 위치 (기본 자세)
        self.arm_targets = {
            # Left Arm
            15: 0.0,    # shoulder_pitch
            16: 0.3,    # shoulder_roll (약간 벌림)
            17: 0.0,    # shoulder_yaw
            18: 0.5,    # elbow (약간 굽힘)
            19: 0.0,    # wrist_roll
            20: 0.0,    # wrist_pitch
            21: 0.0,    # wrist_yaw
            # Right Arm
            22: 0.0,    # shoulder_pitch
            23: -0.3,   # shoulder_roll (약간 벌림, 반대방향)
            24: 0.0,    # shoulder_yaw
            25: 0.5,    # elbow (약간 굽힘)
            26: 0.0,    # wrist_roll
            27: 0.0,    # wrist_pitch
            28: 0.0,    # wrist_yaw
        }

        self.running = True

        # 현재 선택된 관절
        self.selected_joint = 15  # 기본: 왼쪽 어깨 pitch
        self.step_size = 0.1  # 관절 이동량

    def _low_state_callback(self, msg):
        self.low_state = msg

    def set_joint_position(self, joint_idx, position, kp=50.0, kd=3.0):
        """관절 위치 제어"""
        if 0 <= joint_idx < 29:
            self.cmd.motor_cmd[joint_idx].q = position
            self.cmd.motor_cmd[joint_idx].kp = kp
            self.cmd.motor_cmd[joint_idx].kd = kd
            self.cmd.motor_cmd[joint_idx].dq = 0.0
            self.cmd.motor_cmd[joint_idx].tau = 0.0

    def send_command(self):
        """명령 전송"""
        # 하체(0-11)는 ElasticBand가 균형 유지하므로 낮은 게인으로 유지
        for i in range(12):
            self.set_joint_position(i, 0.0, kp=20.0, kd=2.0)

        # 허리(12-14)는 중간 게인
        for i in range(12, 15):
            self.set_joint_position(i, 0.0, kp=30.0, kd=2.0)

        # 상체(15-28)는 타겟 위치로
        for joint_idx, target_pos in self.arm_targets.items():
            self.set_joint_position(joint_idx, target_pos, kp=50.0, kd=3.0)

        self.cmd.crc = self.crc.Crc(self.cmd)
        self.low_cmd_pub.Write(self.cmd)

    def move_joint(self, delta):
        """선택된 관절 이동"""
        if self.selected_joint in self.arm_targets:
            self.arm_targets[self.selected_joint] += delta
            # 관절 범위 제한
            self.arm_targets[self.selected_joint] = np.clip(
                self.arm_targets[self.selected_joint], -2.0, 2.0
            )

    def preset_wave_left(self):
        """왼팔 인사 자세"""
        self.arm_targets[15] = -0.5   # shoulder pitch up
        self.arm_targets[16] = 0.8    # shoulder roll out
        self.arm_targets[18] = 1.2    # elbow bend

    def preset_wave_right(self):
        """오른팔 인사 자세"""
        self.arm_targets[22] = -0.5   # shoulder pitch up
        self.arm_targets[23] = -0.8   # shoulder roll out
        self.arm_targets[25] = 1.2    # elbow bend

    def preset_neutral(self):
        """기본 자세로 복귀"""
        self.arm_targets = {
            15: 0.0, 16: 0.3, 17: 0.0, 18: 0.5, 19: 0.0, 20: 0.0, 21: 0.0,
            22: 0.0, 23: -0.3, 24: 0.0, 25: 0.5, 26: 0.0, 27: 0.0, 28: 0.0,
        }

    def preset_arms_up(self):
        """양팔 들기"""
        self.arm_targets[15] = -1.5   # left shoulder pitch
        self.arm_targets[22] = -1.5   # right shoulder pitch


def main():
    if not PYNPUT_AVAILABLE:
        print("Error: pynput required. Install with: pip install pynput")
        sys.exit(1)

    print("=" * 60)
    print("G1 Upper Body Control (ElasticBand Balance)")
    print("=" * 60)
    print("\nControls:")
    print("  Arrow UP/DOWN: Move selected joint +/-")
    print("  Arrow LEFT/RIGHT: Select previous/next joint")
    print("  ")
    print("  1: Left arm wave pose")
    print("  2: Right arm wave pose")
    print("  3: Both arms up")
    print("  0: Neutral pose")
    print("  ")
    print("  ESC: Quit")
    print("=" * 60)

    controller = G1UpperBodyController()

    def on_press(key):
        try:
            # 숫자 키로 프리셋
            if hasattr(key, 'char'):
                if key.char == '1':
                    controller.preset_wave_left()
                    print("\n[Pose] Left arm wave")
                elif key.char == '2':
                    controller.preset_wave_right()
                    print("\n[Pose] Right arm wave")
                elif key.char == '3':
                    controller.preset_arms_up()
                    print("\n[Pose] Arms up")
                elif key.char == '0':
                    controller.preset_neutral()
                    print("\n[Pose] Neutral")
        except AttributeError:
            pass

        # 화살표 키
        if key == keyboard.Key.up:
            controller.move_joint(controller.step_size)
            joint_name = JOINT_NAMES.get(controller.selected_joint, "?")
            print(f"\r[Joint {controller.selected_joint}] {joint_name}: {controller.arm_targets[controller.selected_joint]:.2f}  ", end="")
        elif key == keyboard.Key.down:
            controller.move_joint(-controller.step_size)
            joint_name = JOINT_NAMES.get(controller.selected_joint, "?")
            print(f"\r[Joint {controller.selected_joint}] {joint_name}: {controller.arm_targets[controller.selected_joint]:.2f}  ", end="")
        elif key == keyboard.Key.right:
            # 다음 관절
            joints = list(controller.arm_targets.keys())
            idx = joints.index(controller.selected_joint)
            controller.selected_joint = joints[(idx + 1) % len(joints)]
            joint_name = JOINT_NAMES.get(controller.selected_joint, "?")
            print(f"\n[Selected] Joint {controller.selected_joint}: {joint_name}")
        elif key == keyboard.Key.left:
            # 이전 관절
            joints = list(controller.arm_targets.keys())
            idx = joints.index(controller.selected_joint)
            controller.selected_joint = joints[(idx - 1) % len(joints)]
            joint_name = JOINT_NAMES.get(controller.selected_joint, "?")
            print(f"\n[Selected] Joint {controller.selected_joint}: {joint_name}")

    def on_release(key):
        if key == keyboard.Key.esc:
            controller.running = False
            return False

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("\n[Ready] Waiting for simulation...")
    print("[Info] Make sure simulation is running with ElasticBand enabled")
    print(f"[Selected] Joint {controller.selected_joint}: {JOINT_NAMES.get(controller.selected_joint, '?')}")
    time.sleep(1)

    try:
        while controller.running:
            controller.send_command()
            time.sleep(0.01)  # 100Hz

    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        listener.stop()
        print("[Done]")


if __name__ == "__main__":
    main()
