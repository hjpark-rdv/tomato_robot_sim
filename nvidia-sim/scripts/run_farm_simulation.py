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
parser.add_argument(
    "--spawn-stem",
    action="store_true",
    default=True,
    help="Dynamically spawn harvestable tomato stem at runtime (default: True)",
)
parser.add_argument(
    "--no-spawn-stem",
    action="store_false",
    dest="spawn_stem",
    help="Do not spawn harvestable tomato stem at startup",
)
parser.add_argument(
    "--stem-pos",
    nargs=3,
    type=float,
    default=None,
    metavar=("X", "Y", "Z"),
    help="Position (x y z) to spawn harvestable stem (e.g. --stem-pos -0.75 1.2 0.32)",
)
parser.add_argument(
    "--stem-yaw",
    type=float,
    default=0.0,
    help="Yaw rotation in degrees for harvestable stem (default: 0.0)",
)
parser.add_argument(
    "--stem-scale",
    type=float,
    default=0.5,
    help="Scale factor for harvestable stem (default: 0.5)",
)
parser.add_argument(
    "--stem-usd",
    type=str,
    default="/root/farmily_tomato/nvidia-sim/env_usd/tomato_stem_v8_with_rotated90_cluster_harvestable_FIXED.usd",
    help="Path to harvestable tomato stem USD",
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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


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

TOMATO_JOINT_BREAK_TORQUE_NM = 1.0


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


def spawn_harvestable_stem(
    stage: Usd.Stage,
    stem_usd_path: str,
    pos: tuple,
    yaw_deg: float = 0.0,
    scale: float = 0.5,
    prim_path: str = "/World/HarvestableStem",
):
    """Dynamically spawn or relocate the harvestable tomato stem asset at runtime."""
    stem_usd_path = os.path.abspath(stem_usd_path)
    if not os.path.exists(stem_usd_path):
        print(f"[WARN] Harvestable stem USD not found: {stem_usd_path}")
        return None

    stem_prim = stage.GetPrimAtPath(prim_path)
    if not stem_prim.IsValid():
        stem_xform = UsdGeom.Xform.Define(stage, Sdf.Path(prim_path))
        stem_prim = stem_xform.GetPrim()
        stem_prim.GetReferences().AddReference(stem_usd_path)
        print(f"[SIM] Added harvestable stem reference: {stem_usd_path}")
    else:
        stem_xform = UsdGeom.Xform(stem_prim)

    xformable = UsdGeom.Xformable(stem_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xformable.AddRotateZOp().Set(float(yaw_deg))
    xformable.AddScaleOp().Set(Gf.Vec3f(float(scale), float(scale), float(scale)))

    # The source asset's 0.08 N*m torque limit is below the constraint impulse
    # produced during PhysX initialization, causing all fruit joints to break on
    # the first frame. Keep the 3 N force limit harvestable, while allowing the
    # fixed joints to survive initialization and gravity.
    joint_root = stage.GetPrimAtPath(f"{prim_path}/Joints")
    configured_joint_count = 0
    if joint_root.IsValid():
        for joint_prim in Usd.PrimRange(joint_root):
            if joint_prim.IsA(UsdPhysics.Joint) and joint_prim.GetName().startswith("TomatoJoint_"):
                UsdPhysics.Joint(joint_prim).GetBreakTorqueAttr().Set(
                    TOMATO_JOINT_BREAK_TORQUE_NM
                )
                configured_joint_count += 1

    if configured_joint_count == 0:
        print(f"[WARN] No harvestable tomato joints found below {prim_path}/Joints")
    else:
        print(
            f"[SIM] Configured {configured_joint_count} tomato joints: "
            f"breakTorque={TOMATO_JOINT_BREAK_TORQUE_NM:.2f} N*m"
        )

    print(f"\n[STEM RELOCATED] HarvestableStem successfully moved to:")
    print(f"                 Position (X, Y, Z) = ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    print(f"                 Yaw = {yaw_deg:.1f}° | Scale = {scale:.2f}x\n")
    return stem_prim


import queue
import threading

stem_cmd_queue = queue.Queue()
IPC_CMD_FILE = "/tmp/farmily_stem_cmd.txt"


def stdin_command_listener():
    """Background listener reading user coordinate inputs from terminal while simulation runs."""
    while True:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    yaw = float(parts[3]) if len(parts) >= 4 else 0.0
                    stem_cmd_queue.put((x, y, z, yaw))
                    print(f"\n[입력 접수] 새 좌표로 이동 요청: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, Yaw={yaw:.1f}°")
                except ValueError:
                    print(f"\n[입력 오류] 숫자 형식이 올바르지 않습니다: '{line}'. (형식: X Y Z [YAW], 예: -0.75 1.5 0.32)")
            elif line.lower() in ["q", "quit", "exit"]:
                stem_cmd_queue.put("quit")
            else:
                print("\n[실시간 좌표 입력 안내] 'X Y Z [YAW]' 형식으로 입력하세요. (예: -0.75 1.5 0.32 또는 -0.75 1.5 0.32 90)")
        except Exception:
            break


def create_stem_spawner_ui(stage, stem_usd_path, initial_pos=(-0.75, 1.20, 0.32), scale=0.5):
    """Create a floating UI panel in Isaac Sim GUI to adjust stem coordinates in real-time."""
    try:
        import omni.ui as ui

        window = ui.Window("Tomato Stem Spawner", width=340, height=230)
        with window.frame:
            with ui.VStack(spacing=5):
                ui.Label("🍅 Harvestable Stem Spawner", height=22, style={"color": 0xFF55FF55, "font_size": 16})
                ui.Label("시뮬레이션 실행 중 실시간 좌표 이동", height=16, style={"color": 0xFFAAAAAA, "font_size": 12})
                with ui.HStack(height=24):
                    ui.Label("X (m):", width=55)
                    x_model = ui.SimpleFloatModel(initial_pos[0])
                    ui.FloatDrag(x_model, min=-3.0, max=3.0, step=0.01)
                with ui.HStack(height=24):
                    ui.Label("Y (m):", width=55)
                    y_model = ui.SimpleFloatModel(initial_pos[1])
                    ui.FloatDrag(y_model, min=-2.0, max=6.0, step=0.01)
                with ui.HStack(height=24):
                    ui.Label("Z (m):", width=55)
                    z_model = ui.SimpleFloatModel(initial_pos[2])
                    ui.FloatDrag(z_model, min=0.0, max=3.0, step=0.01)
                with ui.HStack(height=24):
                    ui.Label("Yaw (°):", width=55)
                    yaw_model = ui.SimpleFloatModel(0.0)
                    ui.FloatDrag(yaw_model, min=-180.0, max=180.0, step=5.0)

                def on_click():
                    stem_cmd_queue.put((x_model.as_float, y_model.as_float, z_model.as_float, yaw_model.as_float))

                ui.Spacer(height=3)
                ui.Button("Move Stem (실시간 이동)", clicked_fn=on_click, height=32, style={"background_color": 0xFF228822})
        return window
    except Exception as e:
        print(f"[WARN] Failed to create Stem Spawner UI: {e}")
        return None


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

    # Dynamic spawn of harvestable tomato stem if requested
    stem_pos = args.stem_pos
    if stem_pos is not None or args.spawn_stem:
        if stem_pos is None:
            # Default to left gutter within reachable workspace in front of robot
            stem_pos = (-0.75, 1.20, 0.32)
        spawn_harvestable_stem(
            stage=stage,
            stem_usd_path=args.stem_usd,
            pos=tuple(stem_pos),
            yaw_deg=args.stem_yaw,
            scale=args.stem_scale,
        )

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
            vp.camera_path = overview_cam_path
            vp.set_active_camera(overview_cam_path)
            print(f"[SIM] Set viewport camera to {overview_cam_path}")
        except Exception as e:
            print(f"[WARN] Failed to set viewport camera: {e}")

    dof_names = list(robot.dof_names)
    num_dofs = robot.num_dof
    print(f"[SIM] Robot ready with {num_dofs} DOFs: {dof_names}")

    def check_and_enqueue_ipc():
        """Check for and enqueue coordinate updates sent via move_stem.sh / IPC file."""
        if os.path.exists(IPC_CMD_FILE):
            try:
                with open(IPC_CMD_FILE, "r") as f:
                    lines = f.readlines()
                os.remove(IPC_CMD_FILE)
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.replace(",", " ").split()
                    if len(parts) >= 3:
                        cx, cy, cz = float(parts[0]), float(parts[1]), float(parts[2])
                        cyaw = float(parts[3]) if len(parts) >= 4 else 0.0
                        stem_cmd_queue.put((cx, cy, cz, cyaw))
                        print(f"\n[IPC 수신] 외부 명령 파일 수신: X={cx:.3f}, Y={cy:.3f}, Z={cz:.3f}, Yaw={cyaw:.1f}°")
            except Exception:
                pass

    # Check for any IPC commands queued during startup
    check_and_enqueue_ipc()

    # Start background stdin command listener thread for real-time terminal inputs
    try:
        stdin_thread = threading.Thread(target=stdin_command_listener, daemon=True)
        stdin_thread.start()
        print("[SIM] Terminal stdin coordinate listener active.")
    except Exception as e:
        print(f"[WARN] Could not start terminal stdin listener: {e}")

    # Create GUI spawner window if running in interactive UI mode
    spawner_window = None
    if not args.headless:
        try:
            spawner_window = create_stem_spawner_ui(
                stage=stage,
                stem_usd_path=args.stem_usd,
                initial_pos=tuple(stem_pos) if stem_pos else (-0.75, 1.20, 0.32),
                scale=args.stem_scale,
            )
            if spawner_window is not None:
                print("[SIM] Interactive 'Tomato Stem Spawner' GUI window ready on screen.")
        except Exception as e:
            print(f"[WARN] Failed to create GUI window: {e}")

    # Print live stem control instructions
    print("\n" + "=" * 70)
    print(" 🍅 [LIVE STEM CONTROL] 실시간 토마토 줄기/열매 위치 제어 활성화!")
    print(" 시뮬레이션을 재시작(runsim)할 필요 없이 아래 방법으로 실시간 이동:")
    print("   [방법 1] 현재 터미널에 좌표 입력 후 Enter: 예) -0.75 1.5 0.32")
    print("            (회전각도 포함: -0.75 1.5 0.32 90)")
    print("   [방법 2] 다른 터미널에서 명령어 실행:   ./nvidia-sim/move_stem.sh X Y Z [YAW]")
    if not args.headless:
        print("   [방법 3] Isaac Sim 화면의 'Tomato Stem Spawner' 패널에서 슬라이더 조작 후 클릭")
    print("=" * 70 + "\n")

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
        # Check IPC command file (/tmp/farmily_stem_cmd.txt)
        check_and_enqueue_ipc()

        # Process any pending real-time stem movement commands
        while not stem_cmd_queue.empty():
            cmd = stem_cmd_queue.get_nowait()
            if cmd == "quit":
                print("[SIM] Quit command received. Shutting down...")
                return
            if isinstance(cmd, (tuple, list)) and len(cmd) >= 3:
                cx, cy, cz = cmd[0], cmd[1], cmd[2]
                cyaw = cmd[3] if len(cmd) > 3 else 0.0
                spawn_harvestable_stem(
                    stage=stage,
                    stem_usd_path=args.stem_usd,
                    pos=(cx, cy, cz),
                    yaw_deg=cyaw,
                    scale=args.stem_scale,
                )

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
                for _ in range(20):
                    simulation_app.update()
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
