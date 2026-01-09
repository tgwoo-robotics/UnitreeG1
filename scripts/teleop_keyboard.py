#!/usr/bin/env python3
"""
Keyboard Teleoperation for Simulation Testing

Meta Quest 없이 키보드로 로봇 제어 테스트
"""

import argparse
import sys
import time
import threading
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
XR_TELEOP_PATH = PROJECT_ROOT / "external/xr_teleoperate"
sys.path.insert(0, str(XR_TELEOP_PATH))

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False


class KeyboardController:
    """키보드 입력을 속도 명령으로 변환"""

    def __init__(self, linear_speed: float = 0.2, angular_speed: float = 0.3):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        # 현재 속도
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        # 키 상태
        self.keys_pressed = set()

        # 종료 플래그
        self.running = True

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = key

        self.keys_pressed.add(k)
        self._update_velocity()

    def on_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = key

        self.keys_pressed.discard(k)
        self._update_velocity()

        # ESC로 종료
        if key == keyboard.Key.esc:
            self.running = False
            return False

    def _update_velocity(self):
        """키 상태에 따라 속도 업데이트"""
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        # WASD: 이동
        if 'w' in self.keys_pressed:
            self.vx = self.linear_speed
        if 's' in self.keys_pressed:
            self.vx = -self.linear_speed
        if 'a' in self.keys_pressed:
            self.vy = self.linear_speed
        if 'd' in self.keys_pressed:
            self.vy = -self.linear_speed

        # QE: 회전
        if 'q' in self.keys_pressed:
            self.vyaw = self.angular_speed
        if 'e' in self.keys_pressed:
            self.vyaw = -self.angular_speed

    def get_velocity(self):
        return self.vx, self.vy, self.vyaw


def main():
    parser = argparse.ArgumentParser(description="Keyboard Teleoperation")

    parser.add_argument("--arm", choices=["G1_29", "G1_23"], default="G1_29")
    parser.add_argument("--linear-speed", type=float, default=0.2,
                        help="Linear speed (m/s)")
    parser.add_argument("--angular-speed", type=float, default=0.3,
                        help="Angular speed (rad/s)")
    parser.add_argument("--network-interface", default="lo",
                        help="Network interface")
    parser.add_argument("--frequency", type=float, default=50.0)

    args = parser.parse_args()

    if not PYNPUT_AVAILABLE:
        print("Error: pynput not installed")
        print("Install with: pip install pynput")
        sys.exit(1)

    # Import
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController
    from teleop.utils.motion_switcher import LocoClientWrapper

    print("=" * 60)
    print("Keyboard Teleoperation (Simulation)")
    print("=" * 60)
    print("\nControls:")
    print("  W/S: Forward/Backward")
    print("  A/D: Left/Right")
    print("  Q/E: Rotate Left/Right")
    print("  ESC: Quit")
    print("=" * 60)

    # DDS 초기화 (시뮬레이션용 domain_id=1)
    ChannelFactoryInitialize(1, networkInterface=args.network_interface)
    print("[DDS] Domain ID: 1 (Simulation)")

    # Locomotion
    loco_wrapper = LocoClientWrapper()
    print("[Locomotion] Ready")

    # Arm (선택적)
    if args.arm == "G1_29":
        arm_ctrl = G1_29_ArmController(motion_mode=True, simulation_mode=True)
    else:
        arm_ctrl = G1_23_ArmController(motion_mode=True, simulation_mode=True)
    print(f"[Arm] {args.arm} Ready")

    # Keyboard controller
    kb_ctrl = KeyboardController(
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed
    )

    # Keyboard listener
    listener = keyboard.Listener(
        on_press=kb_ctrl.on_press,
        on_release=kb_ctrl.on_release
    )
    listener.start()

    print("\n[READY] Press any movement key to start...")

    loop_dt = 1.0 / args.frequency

    try:
        while kb_ctrl.running:
            loop_start = time.time()

            # Get velocity
            vx, vy, vyaw = kb_ctrl.get_velocity()

            # Send command
            loco_wrapper.Move(vx, vy, vyaw)

            # Status display (occasionally)
            if int(time.time() * 2) % 2 == 0:
                if vx != 0 or vy != 0 or vyaw != 0:
                    print(f"\r[Velocity] vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}    ", end="")

            # Loop timing
            elapsed = time.time() - loop_start
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)

    except KeyboardInterrupt:
        print("\n[Stopped] Ctrl+C")
    finally:
        loco_wrapper.Damp()
        listener.stop()
        print("[Cleanup] Done")


if __name__ == "__main__":
    main()
