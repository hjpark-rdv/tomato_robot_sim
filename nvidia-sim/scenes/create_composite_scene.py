#!/usr/bin/env python3
"""Programmatically create a composite USD stage combining the greenhouse environment and the harvesting robot."""

import argparse
import os
import sys

from isaacsim import SimulationApp

# Start headless simulation app to access pxr API
simulation_app = SimulationApp({"headless": True})

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


def create_composite_scene(
    output_usd_path: str,
    greenhouse_usd_path: str,
    robot_usd_path: str,
    robot_pos: tuple = (0.00, 1.00, 0.075),
    robot_yaw_deg: float = 180.0,
    greenhouse_scale: float = 0.5,
):
    print(f"Creating composite stage at: {output_usd_path} (greenhouse_scale={greenhouse_scale})")
    os.makedirs(os.path.dirname(os.path.abspath(output_usd_path)), exist_ok=True)

    # 1. Create Stage
    stage = Usd.Stage.CreateNew(output_usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # 2. Root Prim: /World
    world_path = Sdf.Path("/World")
    world_prim = UsdGeom.Xform.Define(stage, world_path)
    stage.SetDefaultPrim(world_prim.GetPrim())

    # 3. Physics Scene: /World/PhysicsScene
    physics_scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    # 4. Ground Plane with collision (at z=0)
    ground_path = Sdf.Path("/World/GroundPlane")
    ground_prim = UsdGeom.Plane.Define(stage, ground_path)
    ground_prim.CreateAxisAttr().Set("Z")
    ground_prim.CreateLengthAttr().Set(30.0)
    ground_prim.CreateWidthAttr().Set(30.0)
    UsdPhysics.CollisionAPI.Apply(ground_prim.GetPrim())

    # 5. Lighting
    # Distant / Sunlight
    sun_path = Sdf.Path("/World/SunLight")
    sun_light = UsdLux.DistantLight.Define(stage, sun_path)
    sun_light.CreateIntensityAttr().Set(2500.0)
    sun_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.92))
    sun_xform = UsdGeom.XformOp(sun_light.GetPrim().GetAttribute("xformOp:rotateXYZ"))
    if not sun_xform:
        UsdGeom.Xformable(sun_light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

    # Dome Light for greenhouse ambient diffuse
    dome_path = Sdf.Path("/World/DomeLight")
    dome_light = UsdLux.DomeLight.Define(stage, dome_path)
    dome_light.CreateIntensityAttr().Set(1000.0)
    dome_light.CreateColorAttr().Set(Gf.Vec3f(0.9, 0.95, 1.0))

    # 6. Reference Greenhouse Environment (with scale)
    greenhouse_path = Sdf.Path("/World/Greenhouse")
    greenhouse_xform = UsdGeom.Xform.Define(stage, greenhouse_path)
    greenhouse_prim = greenhouse_xform.GetPrim()
    greenhouse_prim.GetReferences().AddReference(
        assetPath=os.path.relpath(greenhouse_usd_path, os.path.dirname(output_usd_path))
        if os.path.isabs(greenhouse_usd_path)
        else greenhouse_usd_path
    )
    if greenhouse_scale != 1.0:
        greenhouse_xform.AddScaleOp().Set(Gf.Vec3f(greenhouse_scale, greenhouse_scale, greenhouse_scale))
        print(f"[OK] Applied greenhouse scale: {greenhouse_scale}")
    print(f"[OK] Added Greenhouse reference: {greenhouse_usd_path}")

    # 7. Reference Harvesting Robot (rb5_farmily) - Kept 1:1 Scale
    robot_path = Sdf.Path("/World/Robot")
    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    robot_prim = robot_xform.GetPrim()
    robot_prim.GetReferences().AddReference(
        assetPath=os.path.relpath(robot_usd_path, os.path.dirname(output_usd_path))
        if os.path.isabs(robot_usd_path)
        else robot_usd_path
    )

    # Set Robot Position and Orientation
    xformable = UsdGeom.Xformable(robot_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(robot_pos[0], robot_pos[1], robot_pos[2]))
    xformable.AddRotateZOp().Set(float(robot_yaw_deg))
    print(f"[OK] Added Robot reference at pos={robot_pos}, yaw={robot_yaw_deg} deg")

    # 8. Overview Camera inside aisle looking at robot & tomato crop
    cam_path = Sdf.Path("/World/OverviewCamera")
    cam_prim = UsdGeom.Camera.Define(stage, cam_path)
    # Camera positioned in the aisle looking towards robot and scaled crop
    eye = Gf.Vec3d(0.30, -0.65, 1.35)
    target = Gf.Vec3d(0.00, 0.90, 0.55)
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    view_mat = Gf.Matrix4d().SetLookAt(eye, target, up)
    cam_xform = view_mat.GetInverse()
    cam_xformable = UsdGeom.Xformable(cam_prim.GetPrim())
    cam_xformable.ClearXformOpOrder()
    cam_xformable.AddTransformOp().Set(cam_xform)
    cam_prim.CreateFocalLengthAttr().Set(24.0)
    cam_prim.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 100.0))
    print(f"[OK] Added OverviewCamera at eye={eye} looking at robot {target}")

    # 9. Save Stage
    stage.GetRootLayer().Save()
    print(f"[SUCCESS] Composite stage saved to: {output_usd_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate composite greenhouse + robot USD stage")
    parser.add_argument(
        "--output",
        type=str,
        default="/root/farmily_tomato/nvidia-sim/scenes/farmily_greenhouse_robot.usd",
        help="Path for output composite USD",
    )
    parser.add_argument(
        "--greenhouse-usd",
        type=str,
        default="/root/farmily_tomato/nvidia-sim/env_usd/tomato_greenhouse_upgraded_with_stems_and_clusters_v2_isaac.usd",
        help="Path to greenhouse USD",
    )
    parser.add_argument(
        "--robot-usd",
        type=str,
        default="/root/farmily_tomato/nvidia-sim/robot_usd/rb5_farmily.usd",
        help="Path to robot USD",
    )
    parser.add_argument(
        "--greenhouse-scale",
        type=float,
        default=0.5,
        help="Scale factor for the greenhouse environment (default: 0.5)",
    )
    parser.add_argument(
        "--robot-pos",
        nargs=3,
        type=float,
        default=[0.00, 1.00, 0.075],
        help="Robot spawn position (x y z)",
    )
    parser.add_argument(
        "--robot-yaw",
        type=float,
        default=180.0,
        help="Robot spawn yaw angle in degrees",
    )
    args = parser.parse_args()

    create_composite_scene(
        output_usd_path=args.output,
        greenhouse_usd_path=args.greenhouse_usd,
        robot_usd_path=args.robot_usd,
        robot_pos=tuple(args.robot_pos),
        robot_yaw_deg=args.robot_yaw,
        greenhouse_scale=args.greenhouse_scale,
    )


if __name__ == "__main__":
    main()
    simulation_app.close()
