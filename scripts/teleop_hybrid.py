#!/usr/bin/env python3
"""
Hybrid Teleoperation Script

- 상체: 손 추적 (hand tracking)으로 제어
- 하체: 헤드셋 이동 속도 + 컨트롤러 조이스틱 (둘 다 사용 가능)

Usage:
    python teleop_hybrid.py --arm G1_29 --ee inspire_ftp --sim
"""

import argparse
import sys
import time
import numpy as np
from pathlib import Path

# Add paths
PROJECT_ROOT = Path(__file__).parent.parent
XR_TELEOP_PATH = PROJECT_ROOT / "external/xr_teleoperate"
sys.path.insert(0, str(XR_TELEOP_PATH))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from multiprocessing import Array, Lock


class VelocityBasedLocomotion:
    """
    속도 기반 하체 제어

    - 헤드셋 이동 속도를 감지해서 로봇 이동
    - 컨트롤러 조이스틱 입력도 합산
    - 멈추면 로봇도 즉시 정지
    """

    def __init__(self,
                 head_velocity_scale: float = 1.0,
                 ctrl_velocity_scale: float = 0.3,
                 max_velocity: float = 0.3,
                 max_rotation: float = 0.5,
                 velocity_deadzone: float = 0.03,
                 smoothing: float = 0.5):
        """
        Args:
            head_velocity_scale: 헤드셋 이동 속도 → 로봇 속도 스케일
            ctrl_velocity_scale: 컨트롤러 조이스틱 → 로봇 속도 스케일
            max_velocity: 최대 이동 속도 (m/s)
            max_rotation: 최대 회전 속도 (rad/s)
            velocity_deadzone: 속도 데드존 (m/s)
            smoothing: 스무딩 계수 (0-1)
        """
        self.head_velocity_scale = head_velocity_scale
        self.ctrl_velocity_scale = ctrl_velocity_scale
        self.max_velocity = max_velocity
        self.max_rotation = max_rotation
        self.velocity_deadzone = velocity_deadzone
        self.smoothing = smoothing

        # 이전 상태
        self.prev_head_pos = None
        self.prev_time = None

        # 스무딩된 속도
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_vyaw = 0.0

        # 속도 필터 (노이즈 제거)
        self.velocity_buffer = []
        self.buffer_size = 5

    def update(self, head_pose: np.ndarray,
               left_thumbstick: np.ndarray,
               right_thumbstick: np.ndarray) -> tuple:
        """
        속도 계산

        Args:
            head_pose: (4,4) SE(3) 헤드셋 pose (Robot Convention)
            left_thumbstick: [x, y] 왼쪽 조이스틱 (-1 ~ 1)
            right_thumbstick: [x, y] 오른쪽 조이스틱 (-1 ~ 1)

        Returns:
            (vx, vy, vyaw): 로봇 속도 명령 (m/s, m/s, rad/s)
        """
        current_time = time.time()
        current_pos = head_pose[:3, 3].copy()

        # 초기화
        if self.prev_head_pos is None:
            self.prev_head_pos = current_pos
            self.prev_time = current_time
            return 0.0, 0.0, 0.0

        dt = current_time - self.prev_time
        if dt < 0.001:  # 너무 짧은 간격
            return self.smoothed_vx, self.smoothed_vy, self.smoothed_vyaw

        # === 1. 헤드셋 이동 속도 계산 ===
        delta_pos = current_pos - self.prev_head_pos

        # 속도 = 위치 변화 / 시간
        head_vx = delta_pos[0] / dt  # 전진/후진
        head_vy = delta_pos[1] / dt  # 좌/우

        # 속도 버퍼에 추가 (노이즈 필터링)
        self.velocity_buffer.append([head_vx, head_vy])
        if len(self.velocity_buffer) > self.buffer_size:
            self.velocity_buffer.pop(0)

        # 평균 속도 계산
        avg_velocity = np.mean(self.velocity_buffer, axis=0)
        head_vx, head_vy = avg_velocity[0], avg_velocity[1]

        # 데드존 적용
        if abs(head_vx) < self.velocity_deadzone:
            head_vx = 0.0
        if abs(head_vy) < self.velocity_deadzone:
            head_vy = 0.0

        # 스케일 적용
        head_vx *= self.head_velocity_scale
        head_vy *= self.head_velocity_scale

        # === 2. 컨트롤러 조이스틱 입력 ===
        # 왼쪽 조이스틱: 전진/후진 (y), 좌/우 (x)
        # 오른쪽 조이스틱: 회전 (x)
        ctrl_vx = -left_thumbstick[1] * self.ctrl_velocity_scale
        ctrl_vy = -left_thumbstick[0] * self.ctrl_velocity_scale
        ctrl_vyaw = -right_thumbstick[0] * self.max_rotation

        # === 3. 합산 ===
        target_vx = head_vx + ctrl_vx
        target_vy = head_vy + ctrl_vy
        target_vyaw = ctrl_vyaw  # 회전은 컨트롤러만

        # === 4. 속도 제한 ===
        target_vx = np.clip(target_vx, -self.max_velocity, self.max_velocity)
        target_vy = np.clip(target_vy, -self.max_velocity, self.max_velocity)
        target_vyaw = np.clip(target_vyaw, -self.max_rotation, self.max_rotation)

        # === 5. 스무딩 ===
        self.smoothed_vx = self.smoothing * self.smoothed_vx + (1 - self.smoothing) * target_vx
        self.smoothed_vy = self.smoothing * self.smoothed_vy + (1 - self.smoothing) * target_vy
        self.smoothed_vyaw = self.smoothing * self.smoothed_vyaw + (1 - self.smoothing) * target_vyaw

        # 상태 업데이트
        self.prev_head_pos = current_pos
        self.prev_time = current_time

        return self.smoothed_vx, self.smoothed_vy, self.smoothed_vyaw

    def reset(self):
        """상태 초기화"""
        self.prev_head_pos = None
        self.prev_time = None
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_vyaw = 0.0
        self.velocity_buffer = []


def main():
    parser = argparse.ArgumentParser(description="G1 Hybrid Teleoperation")

    # 로봇 설정
    parser.add_argument("--arm", choices=["G1_29", "G1_23", "H1_2", "H1"], default="G1_29")
    parser.add_argument("--ee", choices=["dex3", "dex1", "inspire_dfx", "inspire_ftp", "brainco"],
                        default="inspire_ftp", help="End-effector (hand) type")

    # 디스플레이 설정
    parser.add_argument("--display-mode", choices=["immersive", "pass-through", "ego"],
                        default="pass-through")
    parser.add_argument("--img-server-ip", default="192.168.123.164")
    parser.add_argument("--network-interface", default="")
    parser.add_argument("--frequency", type=float, default=30.0)

    # 시뮬레이션
    parser.add_argument("--sim", action="store_true", help="Simulation mode")

    # 속도 설정
    parser.add_argument("--head-velocity-scale", type=float, default=1.0,
                        help="헤드셋 이동 속도 스케일")
    parser.add_argument("--ctrl-velocity-scale", type=float, default=0.3,
                        help="컨트롤러 조이스틱 속도 스케일")
    parser.add_argument("--max-velocity", type=float, default=0.3,
                        help="최대 이동 속도 (m/s)")

    # 하체 비활성화 옵션
    parser.add_argument("--no-locomotion", action="store_true",
                        help="하체 제어 비활성화 (상체만 제어)")

    args = parser.parse_args()

    # Import
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from hybrid_televuer import HybridTeleVuerWrapper
    from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController
    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK
    from teleimager.image_client import ImageClient
    from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper

    print("=" * 60)
    print("G1 Hybrid Teleoperation")
    print("=" * 60)
    print(f"  Robot: {args.arm}")
    print(f"  Hand: {args.ee}")
    print(f"  Locomotion: {'Disabled' if args.no_locomotion else 'Enabled'}")
    print(f"  Simulation: {args.sim}")
    print("=" * 60)
    print("\nControls:")
    print("  [Hand Tracking]")
    print("    - 손 움직임 → 로봇 팔 제어")
    print("    - 손가락 움직임 → 로봇 손 제어")
    if not args.no_locomotion:
        print("  [Locomotion]")
        print("    - 실제로 걸어다니기 → 로봇 이동 (헤드셋 속도 감지)")
        print("    - 왼쪽 조이스틱 → 전진/후진, 좌/우")
        print("    - 오른쪽 조이스틱 → 회전")
        print("    - 양쪽 조이스틱 동시 클릭 → 비상 정지")
    print("  [System]")
    print("    - A 버튼 → 종료")
    print("=" * 60)

    # DDS 초기화
    domain_id = 1 if args.sim else 0
    ChannelFactoryInitialize(domain_id, networkInterface=args.network_interface)
    print(f"[DDS] Domain ID: {domain_id}")

    # Image client
    img_client = ImageClient(host=args.img_server_ip)
    camera_config = img_client.get_cam_config()
    print(f"[Camera] Connected to {args.img_server_ip}")

    # Hybrid TeleVuer
    tv_wrapper = HybridTeleVuerWrapper(
        binocular=camera_config['head_camera']['binocular'],
        img_shape=camera_config['head_camera']['image_shape'],
        display_mode=args.display_mode,
        zmq=camera_config['head_camera']['enable_zmq'],
        webrtc=camera_config['head_camera']['enable_webrtc'],
        webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer" if camera_config['head_camera']['enable_webrtc'] else None,
    )
    print("[TeleVuer] Hybrid mode initialized (Hand + Controller)")

    # Locomotion
    loco_wrapper = None
    loco_tracker = None

    if not args.no_locomotion:
        loco_wrapper = LocoClientWrapper()
        loco_tracker = VelocityBasedLocomotion(
            head_velocity_scale=args.head_velocity_scale,
            ctrl_velocity_scale=args.ctrl_velocity_scale,
            max_velocity=args.max_velocity,
        )
        print("[Locomotion] Enabled - Velocity based")
    else:
        motion_switcher = MotionSwitcher()
        status, result = motion_switcher.Enter_Debug_Mode()
        print(f"[Locomotion] Disabled - Debug mode: {'OK' if status == 0 else 'Failed'}")

    # Arm IK & Controller
    motion_mode = not args.no_locomotion
    if args.arm == "G1_29":
        arm_ik = G1_29_ArmIK()
        arm_ctrl = G1_29_ArmController(motion_mode=motion_mode, simulation_mode=args.sim)
    elif args.arm == "G1_23":
        arm_ik = G1_23_ArmIK()
        arm_ctrl = G1_23_ArmController(motion_mode=motion_mode, simulation_mode=args.sim)
    print(f"[Arm] {args.arm} initialized")

    # Hand Controller
    left_hand_pos_array = Array('d', 75, lock=True)
    right_hand_pos_array = Array('d', 75, lock=True)
    dual_hand_data_lock = Lock()

    hand_dof = 14 if args.ee == "dex3" else 12
    dual_hand_state_array = Array('d', hand_dof, lock=False)
    dual_hand_action_array = Array('d', hand_dof, lock=False)

    if args.ee == "inspire_ftp":
        from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
        hand_ctrl = Inspire_Controller_FTP(
            left_hand_pos_array, right_hand_pos_array,
            dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
            simulation_mode=args.sim
        )
    elif args.ee == "inspire_dfx":
        from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
        hand_ctrl = Inspire_Controller_DFX(
            left_hand_pos_array, right_hand_pos_array,
            dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
            simulation_mode=args.sim
        )
    elif args.ee == "dex3":
        from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
        hand_ctrl = Dex3_1_Controller(
            left_hand_pos_array, right_hand_pos_array,
            dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,
            simulation_mode=args.sim
        )
    print(f"[Hand] {args.ee} initialized")

    # Start
    hand_ctrl.start()
    arm_ctrl.speed_gradual_max()

    print("\n" + "=" * 60)
    print("Press Enter to start teleoperation...")
    print("=" * 60)
    input()
    print("\n[STARTED] Teleoperation running...\n")

    running = True
    loop_dt = 1.0 / args.frequency

    try:
        while running:
            loop_start = time.time()

            # === 1. Get image and render ===
            try:
                head_img = img_client.get_image()
                if head_img is not None:
                    tv_wrapper.render_to_xr(head_img)
            except Exception as e:
                pass

            # === 2. Get teleop data ===
            tele_data = tv_wrapper.get_tele_data()

            # === 3. Update hand positions ===
            with left_hand_pos_array.get_lock():
                left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
            with right_hand_pos_array.get_lock():
                right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()

            # === 4. Locomotion control ===
            if loco_wrapper is not None and loco_tracker is not None:
                # 종료 체크
                if tele_data.right_ctrl_aButton:
                    print("\n[EXIT] A button pressed")
                    running = False
                    break

                # 비상 정지 체크
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    print("[DAMP] Emergency stop!")
                    loco_wrapper.Damp()
                    loco_tracker.reset()
                else:
                    # 속도 계산 및 전송
                    vx, vy, vyaw = loco_tracker.update(
                        tele_data.head_pose,
                        tele_data.left_ctrl_thumbstickValue,
                        tele_data.right_ctrl_thumbstickValue
                    )
                    loco_wrapper.Move(vx, vy, vyaw)

            # === 5. Arm control ===
            current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            sol_q, sol_tauff = arm_ik.solve_ik(
                tele_data.left_wrist_pose,
                tele_data.right_wrist_pose,
                current_lr_arm_q,
                current_lr_arm_dq
            )
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # === 6. Loop timing ===
            elapsed = time.time() - loop_start
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)

    except KeyboardInterrupt:
        print("\n[STOPPED] Ctrl+C pressed")
    finally:
        print("[CLEANUP] Stopping controllers...")
        hand_ctrl.stop()
        tv_wrapper.close()
        if loco_wrapper is not None:
            loco_wrapper.Damp()
        print("[CLEANUP] Done")


if __name__ == "__main__":
    main()
