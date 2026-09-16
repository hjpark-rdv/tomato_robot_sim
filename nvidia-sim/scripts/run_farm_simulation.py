#!/usr/bin/env python3
"""Interactive & Headless Simulation Runner for RB5 Farmily Robot in Smart Farm Greenhouse.

Features:
  - Loads the smart farm greenhouse USD stage with the RB5 harvesting robot.
  - Controls the 7-DOF articulation (vertical lift + 6-axis RB5 manipulator).
  - Demonstrates tomato harvesting posture cycles (lift adjustment, approach, retract).
  - Supports GUI mode (on DISPLAY) or headless mode.
  - Can capture camera snapshots of the scene.
"""

import argparse
import math
import os
import sys
import time

# Ensure immediate unbuffered console logging
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Parse command line arguments before starting SimulationApp
parser = argparse.ArgumentParser(description="Run Smart Farm Greenhouse & Harvesting Robot Simulation")
parser.add_argument(
    "--headless",
    action="store_true",
    default=False,
    help="Run simulation headless (without GUI window)",
)
parser.add_argument(
    "--steps",
    type=int,
    default=0,
    help="Number of simulation steps to run (0 = run indefinitely until user closes window)",
)
parser.add_argument(
    "--scene",
    type=str,
    default="/root/farmily_tomato/nvidia-sim/scenes/farmily_greenhouse_robot.usd",
    help="Path to composite scene USD",
)
parser.add_argument(
    "--capture-frame",
    type=str,
    default=None,
    help="Path to save captured RGB image of the scene",
)
args = parser.parse_args()

# Check display environment
if not args.headless and not os.environ.get("DISPLAY"):
    print("[WARN] DISPLAY not found in environment, falling back to headless mode.")
    args.headless = True

print(f"[SIM] Initializing Isaac Sim (headless={args.headless})...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np
import omni.kit.viewport.utility as vpu
import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux


def setup_overview_camera(stage: Usd.Stage):
    """Ensure overview camera is present in the stage."""
    cam_path = Sdf.Path("/World/OverviewCamera")
    cam_prim = UsdGeom.Camera.Get(stage, cam_path)
    if not cam_prim:
        cam_prim = UsdGeom.Camera.Define(stage, cam_path)
        xformable = UsdGeom.Xformable(cam_prim.GetPrim())
        xformable.ClearXformOpOrder()
        eye = Gf.Vec3d(0.30, -0.65, 1.35)
        target = Gf.Vec3d(0.00, 0.90, 0.55)
        up = Gf.Vec3d(0.0, 0.0, 1.0)
        view_mat = Gf.Matrix4d().SetLookAt(eye, target, up)
        xformable.AddTransformOp().Set(view_mat.GetInverse())
        cam_prim.CreateFocalLengthAttr().Set(24.0)
        cam_prim.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 100.0))
    return "/World/OverviewCamera"


def main():
    scene_path = os.path.abspath(args.scene)
    if not os.path.exists(scene_path):
        print(f"[ERROR] Scene USD does not exist: {scene_path}")
        sys.exit(1)

    print(f"[SIM] Opening greenhouse stage: {scene_path}")
    omni.usd.get_context().open_stage(scene_path)
    stage = omni.usd.get_context().get_stage()

    # Set up overview camera
    overview_cam_path = setup_overview_camera(stage)

    # Initialize World
    print("[SIM] Initializing simulation world physics...")
    world = World(stage_units_in_meters=1.0)

    # Register robot articulation
    robot_prim_path = "/World/Robot"
    robot = world.scene.add(
        SingleArticulation(
            prim_path=robot_prim_path,
            name="rb5_farmily",
        )
    )

    # Reset world
    print("[SIM] Resetting world and starting physics...")
    world.reset()

    # In GUI mode, switch viewport to OverviewCamera
    vp = vpu.get_active_viewport()
    if vp is not None:
        try:
            vp.set_active_camera(overview_cam_path)
            print(f"[SIM] Set viewport camera to {overview_cam_path}")
        except Exception as e:
            print(f"[WARN] Could not set viewport camera: {e}")

    dof_names = list(robot.dof_names)
    num_dofs = robot.num_dof
    print(f"[SIM] Robot ready with {num_dofs} DOFs: {dof_names}")

    # Joint trajectories for demonstration (Harvesting Cycle)
    # DOFs: [farmily_lift_height_joint, base, shoulder, elbow, wrist1, wrist2, wrist3]
    postures = {
        # wrist3 is aligned with the real-robot ROS 2 PICK_READY state
        # (1.541930975 rad). The previous zero value made the fixed gripper
        # look like it was mounted roughly 90 degrees off in Isaac Sim.
        "READY": np.array([0.10, 0.0, -0.30, 1.20, -0.90, 1.57, 1.541930975]),
        "LIFT_HIGH": np.array([0.45, 0.0, -0.30, 1.20, -0.90, 1.57, 1.541930975]),
        "APPROACH_TOMATO": np.array([0.45, 0.35, -0.15, 1.40, -1.25, 1.57, 1.741930975]),
        "GRASP_SIM": np.array([0.45, 0.38, -0.10, 1.45, -1.35, 1.57, 1.741930975]),
        "RETRACT": np.array([0.45, 0.0, -0.35, 1.10, -0.75, 1.57, 1.541930975]),
        "LIFT_LOW": np.array([0.05, 0.0, -0.30, 1.20, -0.90, 1.57, 1.541930975]),
    }

    current_target = postures["READY"].copy()
    robot.apply_action(ArticulationAction(joint_positions=current_target))

    step_count = 0
    phase_idx = 0
    phase_keys = list(postures.keys())
    phase_duration_steps = 150  # ~2.5 seconds per phase at 60Hz

    print("\n" + "=" * 60)
    print(" Smart Farm Robot Simulation Running!")
    print(f" Mode: {'HEADLESS' if args.headless else 'GUI (Interactive)'}")
    print(" Target: Autonomous Harvesting Cycle in Greenhouse")
    print("=" * 60 + "\n")

    frame_captured = False

    while simulation_app.is_running():
        # Cycle through demonstration phases
        if step_count > 0 and step_count % phase_duration_steps == 0:
            phase_idx = (phase_idx + 1) % len(phase_keys)
            phase_name = phase_keys[phase_idx]
            current_target = postures[phase_name].copy()
            current_pos = robot.get_joint_positions()
            lift_m = current_pos[0]
            print(
                f"[Step {step_count:04d}] Phase: {phase_name:<16} | Lift: {lift_m:6.3f} m | Target: {current_target[0]:6.3f} m"
            )

        # Apply action to articulation controller
        robot.apply_action(ArticulationAction(joint_positions=current_target))

        # Step physics and rendering
        world.step(render=not args.headless)
        step_count += 1

        # Capture frame if requested
        if args.capture_frame and not frame_captured and step_count >= 100:
            os.makedirs(os.path.dirname(os.path.abspath(args.capture_frame)), exist_ok=True)
            if vp is not None:
                vpu.capture_viewport_to_file(vp, args.capture_frame)
                print(f"[SIM OK] Captured viewport screenshot to: {args.capture_frame}")
                frame_captured = True
            else:
                print("[WARN] Viewport not available for capture in headless mode without display.")

        # If user specified fixed step count, terminate when reached
        if args.steps > 0 and step_count >= args.steps:
            print(f"[SIM] Reached requested step limit ({args.steps} steps). Exiting.")
            break

    print("[SIM] Simulation finished cleanly.")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
