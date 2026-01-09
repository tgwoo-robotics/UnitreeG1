#!/usr/bin/env python3
"""
Hybrid TeleVuer Wrapper

손 추적 + 컨트롤러 동시 사용 지원
- 손 추적: 상체 (팔 + 손) 제어
- 컨트롤러 조이스틱: 하체 속도 제어
- 헤드셋 이동: 하체 속도 제어 (속도 기반)
"""

import numpy as np
from multiprocessing import Value, Array
from dataclasses import dataclass, field
from typing import Literal
import sys
from pathlib import Path

# Add televuer to path
XR_TELEOP_PATH = Path(__file__).parent.parent / "external/xr_teleoperate"
sys.path.insert(0, str(XR_TELEOP_PATH / "teleop/televuer/src"))

from televuer.televuer import TeleVuer


@dataclass
class HybridTeleData:
    """손 추적 + 컨트롤러 데이터 통합"""
    # Head
    head_pose: np.ndarray  # (4,4) SE(3)

    # Arms (from hand tracking)
    left_wrist_pose: np.ndarray   # (4,4) SE(3)
    right_wrist_pose: np.ndarray  # (4,4) SE(3)

    # Hands (from hand tracking)
    left_hand_pos: np.ndarray = None   # (25,3)
    right_hand_pos: np.ndarray = None  # (25,3)
    left_hand_pinchValue: float = 10.0
    right_hand_pinchValue: float = 10.0

    # Controller (for locomotion)
    left_ctrl_thumbstickValue: np.ndarray = field(default_factory=lambda: np.zeros(2))
    right_ctrl_thumbstickValue: np.ndarray = field(default_factory=lambda: np.zeros(2))
    left_ctrl_thumbstick: bool = False
    right_ctrl_thumbstick: bool = False
    right_ctrl_aButton: bool = False
    right_ctrl_bButton: bool = False


class HybridTeleVuer(TeleVuer):
    """손 추적 + 컨트롤러 동시 지원하는 TeleVuer 확장"""

    def __init__(self, binocular: bool = True, img_shape: tuple = None, display_fps: float = 30.0,
                 display_mode: Literal["immersive", "pass-through", "ego"] = "pass-through",
                 zmq: bool = False, webrtc: bool = False, webrtc_url: str = None,
                 cert_file: str = None, key_file: str = None):

        # 손 추적 모드로 기본 초기화
        self.use_hand_tracking = True
        self.binocular = binocular

        if img_shape is None:
            raise ValueError("[HybridTeleVuer] img_shape must be provided.")
        self.img_shape = (img_shape[0], img_shape[1], 3)
        self.display_fps = display_fps
        self.img_height = self.img_shape[0]
        if self.binocular:
            self.img_width = self.img_shape[1] // 2
        else:
            self.img_width = self.img_shape[1]
        self.aspect_ratio = self.img_width / self.img_height

        # SSL 설정
        import os
        env_cert = os.getenv("XR_TELEOP_CERT")
        env_key = os.getenv("XR_TELEOP_KEY")
        if cert_file is None or key_file is None:
            if env_cert and env_key:
                cert_file = cert_file or env_cert
                key_file = key_file or env_key
            else:
                user_conf_dir = Path.home() / ".config" / "xr_teleoperate"
                cert_path_user = user_conf_dir / "cert.pem"
                key_path_user = user_conf_dir / "key.pem"

                if cert_path_user.exists() and key_path_user.exists():
                    cert_file = cert_file or str(cert_path_user)
                    key_file = key_file or str(key_path_user)
                else:
                    current_module_dir = Path(__file__).resolve().parent.parent / "external/xr_teleoperate/teleop/televuer"
                    cert_file = cert_file or str(current_module_dir / "cert.pem")
                    key_file = key_file or str(current_module_dir / "key.pem")

        from vuer import Vuer
        self.vuer = Vuer(host='0.0.0.0', cert=cert_file, key=key_file, queries=dict(grid=False), queue_len=3)

        # 핸들러 등록 - 카메라, 손, 컨트롤러 모두
        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
        self.vuer.add_handler("HAND_MOVE")(self.on_hand_move)
        self.vuer.add_handler("CONTROLLER_MOVE")(self.on_controller_move)

        self.display_mode = display_mode
        self.zmq = zmq
        self.webrtc = webrtc
        self.webrtc_url = webrtc_url

        # 디스플레이 모드 설정
        import asyncio
        import threading
        import cv2
        from multiprocessing import shared_memory

        if self.display_mode == "immersive":
            if self.webrtc:
                fn = self.main_image_binocular_webrtc if self.binocular else self.main_image_monocular_webrtc
            elif self.zmq:
                self.img2display_shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
                self.img2display = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.img2display_shm.buf)
                self.latest_frame = None
                self.new_frame_event = threading.Event()
                self.stop_writer_event = threading.Event()
                self.writer_thread = threading.Thread(target=self._xr_render_loop, daemon=True)
                self.writer_thread.start()
                fn = self.main_image_binocular_zmq if self.binocular else self.main_image_monocular_zmq
            else:
                raise ValueError("[HybridTeleVuer] immersive mode requires zmq=True or webrtc=True.")
        elif self.display_mode == "ego":
            if self.webrtc:
                fn = self.main_image_binocular_webrtc_ego if self.binocular else self.main_image_monocular_webrtc_ego
            elif self.zmq:
                self.img2display_shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
                self.img2display = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.img2display_shm.buf)
                self.latest_frame = None
                self.new_frame_event = threading.Event()
                self.stop_writer_event = threading.Event()
                self.writer_thread = threading.Thread(target=self._xr_render_loop, daemon=True)
                self.writer_thread.start()
                fn = self.main_image_binocular_zmq_ego if self.binocular else self.main_image_monocular_zmq_ego
            else:
                raise ValueError("[HybridTeleVuer] ego mode requires zmq=True or webrtc=True.")
        elif self.display_mode == "pass-through":
            fn = self.main_hybrid_pass_through
        else:
            raise ValueError(f"[HybridTeleVuer] Unknown display_mode: {self.display_mode}")

        self.vuer.spawn(start=False)(fn)

        # Shared memory - 공통
        self.head_pose_shared = Array('d', 16, lock=True)
        self.left_arm_pose_shared = Array('d', 16, lock=True)
        self.right_arm_pose_shared = Array('d', 16, lock=True)

        # 손 추적 데이터
        self.left_hand_position_shared = Array('d', 75, lock=True)
        self.right_hand_position_shared = Array('d', 75, lock=True)
        self.left_hand_orientation_shared = Array('d', 25 * 9, lock=True)
        self.right_hand_orientation_shared = Array('d', 25 * 9, lock=True)
        self.left_hand_pinch_shared = Value('b', False, lock=True)
        self.left_hand_pinchValue_shared = Value('d', 0.0, lock=True)
        self.left_hand_squeeze_shared = Value('b', False, lock=True)
        self.left_hand_squeezeValue_shared = Value('d', 0.0, lock=True)
        self.right_hand_pinch_shared = Value('b', False, lock=True)
        self.right_hand_pinchValue_shared = Value('d', 0.0, lock=True)
        self.right_hand_squeeze_shared = Value('b', False, lock=True)
        self.right_hand_squeezeValue_shared = Value('d', 0.0, lock=True)

        # 컨트롤러 데이터
        self.left_ctrl_trigger_shared = Value('b', False, lock=True)
        self.left_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
        self.left_ctrl_squeeze_shared = Value('b', False, lock=True)
        self.left_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
        self.left_ctrl_thumbstick_shared = Value('b', False, lock=True)
        self.left_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
        self.left_ctrl_aButton_shared = Value('b', False, lock=True)
        self.left_ctrl_bButton_shared = Value('b', False, lock=True)
        self.right_ctrl_trigger_shared = Value('b', False, lock=True)
        self.right_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
        self.right_ctrl_squeeze_shared = Value('b', False, lock=True)
        self.right_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
        self.right_ctrl_thumbstick_shared = Value('b', False, lock=True)
        self.right_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
        self.right_ctrl_aButton_shared = Value('b', False, lock=True)
        self.right_ctrl_bButton_shared = Value('b', False, lock=True)

        from multiprocessing import Process
        self.process = Process(target=self._vuer_run)
        self.process.daemon = True
        self.process.start()

    async def main_hybrid_pass_through(self, session):
        """손 추적 + 컨트롤러 동시 스트리밍"""
        from vuer.schemas import Hands, MotionControllers
        import asyncio

        # 손 추적과 컨트롤러 모두 활성화
        session.upsert(
            Hands(
                stream=True,
                key="hands",
                hideLeft=True,
                hideRight=True
            ),
            to="bgChildren",
        )
        session.upsert(
            MotionControllers(
                stream=True,
                key="motionControllers",
                left=True,
                right=True,
            ),
            to="bgChildren",
        )

        while True:
            await asyncio.sleep(1.0 / self.display_fps)


class HybridTeleVuerWrapper:
    """손 추적 + 컨트롤러 래퍼"""

    # 좌표 변환 상수
    T_ROBOT_OPENXR = np.array([[ 0, 0,-1, 0],
                               [-1, 0, 0, 0],
                               [ 0, 1, 0, 0],
                               [ 0, 0, 0, 1]])

    T_OPENXR_ROBOT = np.array([[ 0,-1, 0, 0],
                               [ 0, 0, 1, 0],
                               [-1, 0, 0, 0],
                               [ 0, 0, 0, 1]])

    T_TO_UNITREE_HUMANOID_LEFT_ARM = np.array([[1, 0, 0, 0],
                                               [0, 0,-1, 0],
                                               [0, 1, 0, 0],
                                               [0, 0, 0, 1]])

    T_TO_UNITREE_HUMANOID_RIGHT_ARM = np.array([[1, 0, 0, 0],
                                                [0, 0, 1, 0],
                                                [0,-1, 0, 0],
                                                [0, 0, 0, 1]])

    T_TO_UNITREE_HAND = np.array([[0,  0, 1, 0],
                                  [-1, 0, 0, 0],
                                  [0, -1, 0, 0],
                                  [0,  0, 0, 1]])

    CONST_HEAD_POSE = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 1.5],
                                [0, 0, 1, -0.2],
                                [0, 0, 0, 1]])

    CONST_LEFT_ARM_POSE = np.array([[1, 0, 0, -0.15],
                                    [0, 1, 0, 1.13],
                                    [0, 0, 1, -0.3],
                                    [0, 0, 0, 1]])

    CONST_RIGHT_ARM_POSE = np.array([[1, 0, 0, 0.15],
                                     [0, 1, 0, 1.13],
                                     [0, 0, 1, -0.3],
                                     [0, 0, 0, 1]])

    def __init__(self, binocular: bool = True, img_shape: tuple = (480, 1280),
                 display_fps: float = 30.0,
                 display_mode: Literal["immersive", "pass-through", "ego"] = "pass-through",
                 zmq: bool = False, webrtc: bool = False, webrtc_url: str = None,
                 cert_file: str = None, key_file: str = None):

        self.tvuer = HybridTeleVuer(
            binocular=binocular,
            img_shape=img_shape,
            display_fps=display_fps,
            display_mode=display_mode,
            zmq=zmq,
            webrtc=webrtc,
            webrtc_url=webrtc_url,
            cert_file=cert_file,
            key_file=key_file
        )

    def _safe_mat_update(self, prev_mat, mat):
        det = np.linalg.det(mat)
        if not np.isfinite(det) or np.isclose(det, 0.0, atol=1e-6):
            return prev_mat, False
        return mat, True

    def _fast_mat_inv(self, mat):
        ret = np.eye(4)
        ret[:3, :3] = mat[:3, :3].T
        ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
        return ret

    def get_tele_data(self) -> HybridTeleData:
        """손 추적 + 컨트롤러 데이터 통합 반환"""

        # Head pose
        Bxr_world_head, head_is_valid = self._safe_mat_update(
            self.CONST_HEAD_POSE, self.tvuer.head_pose)
        Brobot_world_head = self.T_ROBOT_OPENXR @ Bxr_world_head @ self.T_OPENXR_ROBOT

        # Arm poses (from hand tracking)
        left_IPxr_Bxr_world_arm, left_arm_is_valid = self._safe_mat_update(
            self.CONST_LEFT_ARM_POSE, self.tvuer.left_arm_pose)
        right_IPxr_Bxr_world_arm, right_arm_is_valid = self._safe_mat_update(
            self.CONST_RIGHT_ARM_POSE, self.tvuer.right_arm_pose)

        # Transform to robot convention
        left_IPxr_Brobot_world_arm = self.T_ROBOT_OPENXR @ left_IPxr_Bxr_world_arm @ self.T_OPENXR_ROBOT
        right_IPxr_Brobot_world_arm = self.T_ROBOT_OPENXR @ right_IPxr_Bxr_world_arm @ self.T_OPENXR_ROBOT

        # Transform to Unitree convention
        left_IPunitree_Brobot_world_arm = left_IPxr_Brobot_world_arm @ (
            self.T_TO_UNITREE_HUMANOID_LEFT_ARM if left_arm_is_valid else np.eye(4))
        right_IPunitree_Brobot_world_arm = right_IPxr_Brobot_world_arm @ (
            self.T_TO_UNITREE_HUMANOID_RIGHT_ARM if right_arm_is_valid else np.eye(4))

        # Transfer from WORLD to HEAD
        left_IPunitree_Brobot_head_arm = left_IPunitree_Brobot_world_arm.copy()
        right_IPunitree_Brobot_head_arm = right_IPunitree_Brobot_world_arm.copy()
        left_IPunitree_Brobot_head_arm[0:3, 3] -= Brobot_world_head[0:3, 3]
        right_IPunitree_Brobot_head_arm[0:3, 3] -= Brobot_world_head[0:3, 3]

        # Offset to waist
        left_wrist = left_IPunitree_Brobot_head_arm.copy()
        right_wrist = right_IPunitree_Brobot_head_arm.copy()
        left_wrist[0, 3] += 0.15
        right_wrist[0, 3] += 0.15
        left_wrist[2, 3] += 0.45
        right_wrist[2, 3] += 0.45

        # Hand positions
        if left_arm_is_valid and right_arm_is_valid:
            left_hand_pos_xr = np.concatenate([
                self.tvuer.left_hand_positions.T,
                np.ones((1, 25))
            ])
            right_hand_pos_xr = np.concatenate([
                self.tvuer.right_hand_positions.T,
                np.ones((1, 25))
            ])

            left_hand_pos_robot = self.T_ROBOT_OPENXR @ left_hand_pos_xr
            right_hand_pos_robot = self.T_ROBOT_OPENXR @ right_hand_pos_xr

            left_hand_pos_arm = self._fast_mat_inv(left_IPxr_Brobot_world_arm) @ left_hand_pos_robot
            right_hand_pos_arm = self._fast_mat_inv(right_IPxr_Brobot_world_arm) @ right_hand_pos_robot

            left_hand_pos = (self.T_TO_UNITREE_HAND @ left_hand_pos_arm)[0:3, :].T
            right_hand_pos = (self.T_TO_UNITREE_HAND @ right_hand_pos_arm)[0:3, :].T
        else:
            left_hand_pos = np.zeros((25, 3))
            right_hand_pos = np.zeros((25, 3))

        return HybridTeleData(
            head_pose=Brobot_world_head,
            left_wrist_pose=left_wrist,
            right_wrist_pose=right_wrist,
            left_hand_pos=left_hand_pos,
            right_hand_pos=right_hand_pos,
            left_hand_pinchValue=self.tvuer.left_hand_pinchValue * 100.0,
            right_hand_pinchValue=self.tvuer.right_hand_pinchValue * 100.0,
            left_ctrl_thumbstickValue=self.tvuer.left_ctrl_thumbstickValue,
            right_ctrl_thumbstickValue=self.tvuer.right_ctrl_thumbstickValue,
            left_ctrl_thumbstick=self.tvuer.left_ctrl_thumbstick,
            right_ctrl_thumbstick=self.tvuer.right_ctrl_thumbstick,
            right_ctrl_aButton=self.tvuer.right_ctrl_aButton,
            right_ctrl_bButton=self.tvuer.right_ctrl_bButton,
        )

    def render_to_xr(self, img):
        self.tvuer.render_to_xr(img)

    def close(self):
        self.tvuer.close()
