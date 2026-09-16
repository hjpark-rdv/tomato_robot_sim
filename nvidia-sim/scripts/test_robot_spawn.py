#!/usr/bin/env python3
"""Automated verification test for rb5_farmily harvesting robot in smart farm greenhouse."""

import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Usd


def run_test():
    scene_path = "/root/farmily_tomato/nvidia-sim/scenes/farmily_greenhouse_robot.usd"
    print(f"[TEST] Loading composite scene: {scene_path}")
    assert os.path.exists(scene_path), f"Scene file not found: {scene_path}"

    omni.usd.get_context().open_stage(scene_path)
    stage = omni.usd.get_context().get_stage()
    assert stage is not None, "Failed to open USD stage"

    # Verify Greenhouse prims
    greenhouse_prim = stage.GetPrimAtPath("/World/Greenhouse")
    assert greenhouse_prim.IsValid(), "Greenhouse prim /World/Greenhouse not found"

    plants_prim = stage.GetPrimAtPath("/World/Greenhouse/Plants")
    if not plants_prim.IsValid():
        plants_prim = stage.GetPrimAtPath("/World/Greenhouse/World/Plants")
    assert plants_prim.IsValid(), f"Greenhouse plants not found (checked /World/Greenhouse/Plants and /World/Greenhouse/World/Plants)"
    num_plants = len(list(plants_prim.GetChildren()))
    print(f"[TEST] Found {num_plants} tomato plants in greenhouse ({plants_prim.GetPath()}).")
    assert num_plants >= 50, f"Expected >= 50 plants, found {num_plants}"

    # Initialize World
    world = World(stage_units_in_meters=1.0)

    # Add Robot Articulation
    robot_prim_path = "/World/Robot"
    robot = world.scene.add(SingleArticulation(prim_path=robot_prim_path, name="rb5_farmily"))

    print("[TEST] Resetting world and initializing physics...")
    world.reset()

    # Verify Articulation DOFs
    num_dofs = robot.num_dof
    dof_names = list(robot.dof_names)
    print(f"[TEST] Robot DOFs ({num_dofs}): {dof_names}")
    assert num_dofs == 7, f"Expected 7 DOFs, got {num_dofs}"

    expected_joints = [
        "farmily_lift_height_joint",
        "base",
        "shoulder",
        "elbow",
        "wrist1",
        "wrist2",
        "wrist3",
    ]
    for ej in expected_joints:
        assert ej in dof_names, f"Expected joint '{ej}' in robot DOFs: {dof_names}"

    # Step simulation
    print("[TEST] Stepping physics simulation (60 steps)...")
    for _ in range(60):
        world.step(render=False)

    initial_pos = robot.get_joint_positions()
    print(f"[TEST] Initial resting joint positions:\n{initial_pos}")
    assert np.all(np.isfinite(initial_pos)), "NaN or Inf detected in initial joint positions!"

    # Test joint command
    target_pos = np.array([0.25, 0.0, -0.3, 1.2, -0.9, 1.57, 0.0])
    print(f"[TEST] Commanding target joint positions:\n{target_pos}")
    action = ArticulationAction(joint_positions=target_pos)
    robot.apply_action(action)

    print("[TEST] Stepping physics simulation with active command (100 steps)...")
    for _ in range(100):
        robot.apply_action(action)
        world.step(render=False)

    final_pos = robot.get_joint_positions()
    print(f"[TEST] Final joint positions after 100 steps:\n{final_pos}")
    assert np.all(np.isfinite(final_pos)), "NaN or Inf detected in final joint positions!"

    # Verify lift motion towards target
    lift_height = final_pos[0]
    print(f"[TEST] Lift height: {lift_height:.4f} m (target: 0.25 m)")
    assert lift_height > 0.15, f"Lift did not respond properly: height={lift_height}"

    print("[TEST SUCCESS] All checks passed successfully! Robot is fully functional in greenhouse.")
    return True


if __name__ == "__main__":
    success = False
    try:
        success = run_test()
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
    if not success:
        sys.exit(1)
