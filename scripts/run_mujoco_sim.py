#!/usr/bin/env python3
"""
MuJoCo Simulation Runner

configs/robot.yaml 설정에 따라 자동으로 모델 선택
"""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from config import get_robot_config, get_model_path, get_simulation_config, print_config


def run_viewer(model_path: Path):
    """MuJoCo 뷰어 실행"""
    import mujoco
    import mujoco.viewer

    print(f"Loading: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    robot = get_robot_config()

    print(f"\n{'='*50}")
    print(f"Model Loaded: {robot.hand.type} hand")
    print(f"{'='*50}")
    print(f"  DOF (nq): {model.nq}")
    print(f"  Actuators (nu): {model.nu}")
    print(f"  Bodies: {model.nbody}")
    print(f"{'='*50}")
    print("\nControls: Mouse=Camera, Space=Pause, ESC=Quit\n")

    mujoco.viewer.launch(model, data)


def run_headless(model_path: Path):
    """헤드리스 테스트"""
    import mujoco
    import numpy as np

    print(f"Loading: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print(f"Model: nq={model.nq}, nu={model.nu}")
    print("Running 1000 steps...")

    for _ in range(1000):
        mujoco.mj_step(model, data)

    print(f"Done. Time: {data.time:.2f}s")

    # 렌더링
    try:
        from PIL import Image

        model.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
        model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]

        renderer = mujoco.Renderer(model, 480, 640)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, 0.9]
        camera.distance = 2.5
        camera.azimuth = 150
        camera.elevation = -15

        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        img = renderer.render()

        output = PROJECT_ROOT / "test_render.png"
        Image.fromarray(img).save(output)
        print(f"Saved: {output}")
        renderer.close()
    except Exception as e:
        print(f"Render skipped: {e}")


def main():
    parser = argparse.ArgumentParser(description="G1 MuJoCo Simulation")
    parser.add_argument("--mode", choices=["viewer", "headless", "info"], default="viewer")
    parser.add_argument("--model", help="Override model path")
    args = parser.parse_args()

    # 설정 출력
    print_config()
    print()

    if args.mode == "info":
        return

    # 모델 경로 결정
    model_path = Path(args.model) if args.model else get_model_path()

    if not model_path.exists():
        print(f"Model not found: {model_path}")
        return

    if args.mode == "viewer":
        run_viewer(model_path)
    elif args.mode == "headless":
        run_headless(model_path)


if __name__ == "__main__":
    main()
