#!/usr/bin/env python3
"""
Central Configuration Manager

configs/robot.yaml 설정을 읽어서 전체 프로젝트에 적용
"""

import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import Optional


PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_PATH = PROJECT_ROOT / "configs/robot.yaml"


@dataclass
class HandConfig:
    type: str
    dof_per_hand: int
    total_dof: int
    urdf_path: Optional[Path]
    mjcf_path: Optional[Path]


@dataclass
class RobotConfig:
    name: str
    version: str
    hand: HandConfig
    body_dof: int
    total_dof: int


def load_config() -> dict:
    """Load central config file"""
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)


def get_hand_config(hand_type: str) -> HandConfig:
    """Get hand configuration based on type"""

    hand_configs = {
        "inspire_ftp": HandConfig(
            type="inspire_ftp",
            dof_per_hand=12,
            total_dof=24,
            urdf_path=PROJECT_ROOT / "external/unitree_ros/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf",
            mjcf_path=None,  # URDF를 MuJoCo로 직접 로드
        ),
        "inspire_dfq": HandConfig(
            type="inspire_dfq",
            dof_per_hand=12,
            total_dof=24,
            urdf_path=PROJECT_ROOT / "external/unitree_ros/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf",
            mjcf_path=None,
        ),
        "dex3": HandConfig(
            type="dex3",
            dof_per_hand=7,
            total_dof=14,
            urdf_path=None,
            mjcf_path=PROJECT_ROOT / "external/mujoco_menagerie/unitree_g1/g1_with_hands.xml",
        ),
        "none": HandConfig(
            type="none",
            dof_per_hand=0,
            total_dof=0,
            urdf_path=None,
            mjcf_path=PROJECT_ROOT / "external/unitree_mujoco/unitree_robots/g1/g1_29dof.xml",
        ),
    }

    return hand_configs.get(hand_type, hand_configs["none"])


def get_robot_config() -> RobotConfig:
    """Get full robot configuration"""
    config = load_config()

    hand_type = config["hand"]["type"]
    hand = get_hand_config(hand_type)

    body_dof = 29 if "29dof" in config["robot"]["version"] else 23

    return RobotConfig(
        name=config["robot"]["name"],
        version=config["robot"]["version"],
        hand=hand,
        body_dof=body_dof,
        total_dof=body_dof + hand.total_dof,
    )


def get_model_path() -> Path:
    """Get the appropriate model path based on config"""
    robot = get_robot_config()

    if robot.hand.urdf_path and robot.hand.urdf_path.exists():
        return robot.hand.urdf_path
    elif robot.hand.mjcf_path and robot.hand.mjcf_path.exists():
        return robot.hand.mjcf_path
    else:
        # Fallback to basic model
        return PROJECT_ROOT / "external/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"


def get_simulation_config() -> dict:
    """Get simulation settings"""
    config = load_config()
    return config.get("simulation", {})


def get_training_config() -> dict:
    """Get training settings"""
    config = load_config()
    return config.get("training", {})


def get_teleop_config() -> dict:
    """Get teleoperation settings"""
    config = load_config()
    return config.get("teleop", {})


def print_config():
    """Print current configuration"""
    robot = get_robot_config()
    config = load_config()

    print("=" * 50)
    print("G1 Robot Configuration")
    print("=" * 50)
    print(f"Robot: {robot.name} ({robot.version})")
    print(f"Hand: {robot.hand.type}")
    print(f"Total DOF: {robot.total_dof} (body: {robot.body_dof}, hands: {robot.hand.total_dof})")
    print(f"Model: {get_model_path()}")
    print("-" * 50)
    print(f"Simulation: {config['simulation']['engine']}")
    print(f"Training: {config['training']['type']}")
    print(f"Teleop: {config['teleop']['device']}")
    print("=" * 50)


if __name__ == "__main__":
    print_config()
