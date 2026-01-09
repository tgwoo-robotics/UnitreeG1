#!/usr/bin/env python3
"""
G1 MuJoCo Simulation Runner

시뮬레이션 실행 후 teleop_hybrid.py --sim으로 텔레오퍼레이션 테스트
"""

import argparse
import subprocess
import sys
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
UNITREE_MUJOCO = PROJECT_ROOT / "external/unitree_mujoco"


def update_config(robot: str = "g1", scene: str = None, domain_id: int = 1):
    """unitree_mujoco config.py 업데이트"""
    config_path = UNITREE_MUJOCO / "simulate_python/config.py"

    if scene is None:
        if robot == "g1":
            scene = "../unitree_robots/g1/scene_29dof.xml"
        else:
            scene = f"../unitree_robots/{robot}/scene.xml"

    config_content = f'''ROBOT = "{robot}" # Robot name
ROBOT_SCENE = "{scene}" # Robot scene
DOMAIN_ID = {domain_id} # Domain id (1=sim, 0=real)
INTERFACE = "lo" # Interface

USE_JOYSTICK = 0 # Disable joystick for teleop mode
JOYSTICK_TYPE = "xbox"
JOYSTICK_DEVICE = 0

PRINT_SCENE_INFORMATION = True
ENABLE_ELASTIC_BAND = False # Virtual spring band

SIMULATE_DT = 0.005
VIEWER_DT = 0.02  # 50 fps
'''

    with open(config_path, 'w') as f:
        f.write(config_content)

    print(f"[Config] Updated: {config_path}")
    print(f"  Robot: {robot}")
    print(f"  Scene: {scene}")
    print(f"  Domain ID: {domain_id}")


def run_simulation():
    """MuJoCo 시뮬레이션 실행"""
    sim_script = UNITREE_MUJOCO / "simulate_python/unitree_mujoco.py"

    print("\n" + "=" * 60)
    print("Starting MuJoCo Simulation...")
    print("=" * 60)
    print("\nControls:")
    print("  - Mouse drag: Camera rotation")
    print("  - Scroll: Zoom in/out")
    print("  - Space: Pause/Resume")
    print("  - ESC: Quit")
    print("=" * 60 + "\n")

    os.chdir(UNITREE_MUJOCO / "simulate_python")
    subprocess.run([sys.executable, str(sim_script)])


def main():
    parser = argparse.ArgumentParser(description="G1 Simulation Runner")

    parser.add_argument("--robot", default="g1", choices=["g1", "h1", "go2", "b2"],
                        help="Robot type")
    parser.add_argument("--scene", default=None,
                        help="Custom scene XML path (relative to unitree_robots/)")
    parser.add_argument("--domain-id", type=int, default=1,
                        help="DDS Domain ID (1=sim, 0=real)")
    parser.add_argument("--config-only", action="store_true",
                        help="Only update config, don't run simulation")

    args = parser.parse_args()

    # Update config
    update_config(
        robot=args.robot,
        scene=args.scene,
        domain_id=args.domain_id
    )

    if not args.config_only:
        run_simulation()


if __name__ == "__main__":
    main()
