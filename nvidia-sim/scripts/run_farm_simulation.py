#!/usr/bin/env python3
"""Interactive & Headless Simulation Runner for RB5 Farmily Robot in Smart Farm Greenhouse.

Features:
  - Loads the smart farm greenhouse USD stage with the RB5 harvesting robot.
  - Controls the 7-DOF articulation (vertical lift + 6-axis RB5 manipulator).
  - Demonstrates tomato harvesting posture cycles (lift adjustment, approach, retract).
  - Publishes the eye-in-hand D435 RGB-D camera over ROS 2.
  - Supports GUI mode (on DISPLAY) or headless mode.
  - Can capture camera snapshots of the scene.
"""

import argparse
import math
import os
import sys
import time
import traceback

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
parser.add_argument(
    "--no-ros2-camera",
    dest="ros2_camera",
    action="store_false",
    help="Disable D435 RGB-D ROS 2 publishers",
)
parser.add_argument(
    "--camera-width",
    type=int,
    default=640,
    help="D435 render and ROS image width (default: 640)",
)
parser.add_argument(
    "--camera-height",
    type=int,
    default=480,
    help="D435 render and ROS image height (default: 480)",
)
parser.set_defaults(ros2_camera=True)
args = parser.parse_args()

if args.camera_width <= 0 or args.camera_height <= 0:
    parser.error("--camera-width and --camera-height must be positive")

# Check display environment
if not args.headless and not os.environ.get("DISPLAY"):
    print("[WARN] DISPLAY not found in environment, falling back to headless mode.")
    args.headless = True

print(f"[SIM] Initializing Isaac Sim (headless={args.headless})...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.kit.viewport.utility as vpu
import omni.usd
import usdrt.Sdf as UsdRtSdf
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux


D435_ROOT_PATH = (
    "/World/Robot/link6/tcp/tomato_gripper/"
    "d435_bottom_screw_frame/d435_link"
)
D435_COLOR_OPTICAL_PATH = f"{D435_ROOT_PATH}/d435_color_frame/d435_color_optical_frame"
D435_DEPTH_OPTICAL_PATH = f"{D435_ROOT_PATH}/d435_depth_frame/d435_depth_optical_frame"
ROS2_CAMERA_GRAPH_PATH = "/World/ROS2D435Graph"

ROS2_CAMERA_TOPICS = {
    "color_image": "/camera/d435/color/image_raw",
    "color_info": "/camera/d435/color/camera_info",
    "depth_image": "/camera/d435/depth/image_raw",
    "depth_info": "/camera/d435/depth/camera_info",
    "point_cloud": "/camera/d435/depth/points",
    "clock": "/clock",
}


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


def enable_ros2_bridge():
    """Enable Isaac Sim's ROS 2 bridge and fail early if it cannot start."""
    extension_name = "isaacsim.ros2.bridge"
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if not extension_manager.is_extension_enabled(extension_name):
        extension_manager.set_extension_enabled_immediate(extension_name, True)
        for _ in range(5):
            simulation_app.update()

    if not extension_manager.is_extension_enabled(extension_name):
        raise RuntimeError(
            "Isaac Sim ROS 2 Bridge did not start. Run through nvidia-sim/run_sim.sh "
            "so the bundled ROS 2 libraries are configured."
        )


def setup_d435_sensor_camera(stage: Usd.Stage, optical_frame_path: str, camera_name: str):
    """Create a USD camera under a D435 ROS optical frame.

    ROS optical frames look along +Z with +Y down, while USD cameras look along
    -Z with +Y up. A 180 degree local X rotation maps those conventions.
    """
    optical_frame = stage.GetPrimAtPath(optical_frame_path)
    if not optical_frame.IsValid():
        raise RuntimeError(f"D435 optical frame was not found: {optical_frame_path}")

    camera_path = f"{optical_frame_path}/{camera_name}"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddRotateXOp().Set(180.0)

    # Approximate Intel RealSense D435 intrinsics at 640x480 (69.4 degree HFOV).
    # Aperture and focal length scale together, so the ratio controls the FOV.
    horizontal_aperture = 20.955
    vertical_aperture = horizontal_aperture * args.camera_height / args.camera_width
    focal_length = horizontal_aperture / (2.0 * math.tan(math.radians(69.4) / 2.0))
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.CreateHorizontalApertureAttr().Set(horizontal_aperture)
    camera.CreateVerticalApertureAttr().Set(vertical_aperture)
    camera.CreateFocalLengthAttr().Set(focal_length)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.10, 20.0))
    camera.CreateFocusDistanceAttr().Set(0.50)
    return camera_path


def setup_ros2_d435_graph(color_camera_path: str, depth_camera_path: str):
    """Publish D435 RGB, depth, camera info, point cloud, and simulation clock."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": ROS2_CAMERA_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("CreateColorRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CreateDepthRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishColor", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("PublishPointCloud", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("CreateColorRenderProduct.inputs:cameraPrim", [UsdRtSdf.Path(color_camera_path)]),
                ("CreateColorRenderProduct.inputs:width", args.camera_width),
                ("CreateColorRenderProduct.inputs:height", args.camera_height),
                ("CreateDepthRenderProduct.inputs:cameraPrim", [UsdRtSdf.Path(depth_camera_path)]),
                ("CreateDepthRenderProduct.inputs:width", args.camera_width),
                ("CreateDepthRenderProduct.inputs:height", args.camera_height),
                ("PublishColor.inputs:topicName", ROS2_CAMERA_TOPICS["color_image"]),
                ("PublishColor.inputs:frameId", "d435_color_optical_frame"),
                ("PublishColor.inputs:type", "rgb"),
                ("PublishColor.inputs:resetSimulationTimeOnStop", False),
                ("PublishColorInfo.inputs:topicName", ROS2_CAMERA_TOPICS["color_info"]),
                ("PublishColorInfo.inputs:frameId", "d435_color_optical_frame"),
                ("PublishColorInfo.inputs:resetSimulationTimeOnStop", False),
                ("PublishDepth.inputs:topicName", ROS2_CAMERA_TOPICS["depth_image"]),
                ("PublishDepth.inputs:frameId", "d435_depth_optical_frame"),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:resetSimulationTimeOnStop", False),
                ("PublishDepthInfo.inputs:topicName", ROS2_CAMERA_TOPICS["depth_info"]),
                ("PublishDepthInfo.inputs:frameId", "d435_depth_optical_frame"),
                ("PublishDepthInfo.inputs:resetSimulationTimeOnStop", False),
                ("PublishPointCloud.inputs:topicName", ROS2_CAMERA_TOPICS["point_cloud"]),
                ("PublishPointCloud.inputs:frameId", "d435_depth_optical_frame"),
                ("PublishPointCloud.inputs:type", "depth_pcl"),
                ("PublishPointCloud.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "CreateColorRenderProduct.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateDepthRenderProduct.inputs:execIn"),
                ("CreateColorRenderProduct.outputs:execOut", "PublishColor.inputs:execIn"),
                ("CreateColorRenderProduct.outputs:execOut", "PublishColorInfo.inputs:execIn"),
                ("CreateColorRenderProduct.outputs:renderProductPath", "PublishColor.inputs:renderProductPath"),
                (
                    "CreateColorRenderProduct.outputs:renderProductPath",
                    "PublishColorInfo.inputs:renderProductPath",
                ),
                ("CreateDepthRenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("CreateDepthRenderProduct.outputs:execOut", "PublishDepthInfo.inputs:execIn"),
                ("CreateDepthRenderProduct.outputs:execOut", "PublishPointCloud.inputs:execIn"),
                ("CreateDepthRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                (
                    "CreateDepthRenderProduct.outputs:renderProductPath",
                    "PublishDepthInfo.inputs:renderProductPath",
                ),
                (
                    "CreateDepthRenderProduct.outputs:renderProductPath",
                    "PublishPointCloud.inputs:renderProductPath",
                ),
            ],
        },
    )


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

    if args.ros2_camera:
        print("[ROS2] Enabling Isaac Sim ROS 2 Bridge...")
        enable_ros2_bridge()
        color_camera_path = setup_d435_sensor_camera(
            stage, D435_COLOR_OPTICAL_PATH, "IsaacColorCamera"
        )
        depth_camera_path = setup_d435_sensor_camera(
            stage, D435_DEPTH_OPTICAL_PATH, "IsaacDepthCamera"
        )
        setup_ros2_d435_graph(color_camera_path, depth_camera_path)
        print(f"[ROS2] D435 RGB-D publishers ready ({args.camera_width}x{args.camera_height})")
        for topic in ROS2_CAMERA_TOPICS.values():
            print(f"[ROS2]   {topic}")

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
        # Camera annotators require rendering even when the GUI is headless.
        world.step(render=args.ros2_camera or not args.headless)
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
    exit_code = 0
    try:
        main()
    except Exception:
        exit_code = 1
        traceback.print_exc()
    finally:
        simulation_app.close()
    sys.exit(exit_code)
