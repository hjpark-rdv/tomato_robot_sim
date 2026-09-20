"""Learning-only robot override and procedural fruit; requires a running Kit app."""

from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema

from geometry import RING_CENTER, WIRE_RADIUS, arc_points

SIM_DIR = Path(__file__).resolve().parents[1]
ROBOT_SOURCE = SIM_DIR / "robot_usd/rb5_farmily.usd"
ROBOT_ASSET = Path(__file__).parent / "generated/rb5_ring.usda"


def collision(prim):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(0.0005)
    api.CreateRestOffsetAttr(0.0)


def capsule(stage, path, a, b, radius, color=(0.2, 0.6, 0.2)):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    delta = b - a
    shape = UsdGeom.Capsule.Define(stage, path)
    shape.CreateRadiusAttr(radius)
    shape.CreateHeightAttr(float(np.linalg.norm(delta)))
    shape.CreateAxisAttr("Z")
    shape.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2)))
    quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*delta)).GetQuat()
    shape.AddOrientOp().Set(Gf.Quatf(quat))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    collision(shape.GetPrim())
    return shape


def build_robot(output_path=None):
    """Author an override, leaving the source USD and ROS model untouched."""
    import trimesh

    output_path = Path(output_path) if output_path is not None else ROBOT_ASSET
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_path))
    root = stage.DefinePrim("/Robot", "Xform")
    root.GetReferences().AddReference(str(ROBOT_SOURCE))
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    # The imported collision geometry lacks CollisionAPI in this checkout.
    # Make collision references editable and restore their physics explicitly.
    for prim in list(stage.Traverse()):
        if prim.IsInstance():
            prim.SetInstanceable(False)
    restored = 0
    for prim in list(stage.Traverse()):
        if "/collisions/" not in str(prim.GetPath()):
            continue
        if "assy_gripper" in str(prim.GetPath()):
            if prim.IsA(UsdGeom.Mesh):
                prim.SetActive(False)
            continue
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube):
            collision(prim)
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            UsdGeom.Imageable(prim).MakeInvisible()
            restored += 1
    tool_path = "/Robot/link6/tcp/tomato_gripper"
    assert stage.GetPrimAtPath(tool_path), "Robot tool hierarchy has changed"
    points = arc_points()
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        shape = capsule(stage, f"{tool_path}/RingCollision/segment_{i:02d}", a, b, WIRE_RADIUS)
        shape.MakeInvisible()
    # CAD has two straight wires between the half-ring ends and the proximal
    # assembly. Keep the aperture open, but make these visible wires physical.
    # Separate paths preserve the existing rear-arc hook/contact definition.
    for i, a in enumerate((points[0], points[-1])):
        b = (-0.078, a[1], a[2])
        shape = capsule(stage, f"{tool_path}/RailCollision/rail_{i:02d}", a, b, WIRE_RADIUS)
        shape.MakeInvisible()
    # Convex approximation of the proximal assembly only, clear of the aperture.
    mesh_path = SIM_DIR.parent / "ros2_ws/src/rbpodo_ros2/rbpodo_description/meshes/tomato_gripper/assy_gripper_ver_6.stl"
    mesh = trimesh.load(mesh_path, force="mesh")
    vertices = mesh.vertices[mesh.vertices[:, 0] > -78] * 0.001
    hull = trimesh.convex.convex_hull(vertices)
    proxy = UsdGeom.Mesh.Define(stage, f"{tool_path}/ProximalCollision")
    proxy.CreatePointsAttr(hull.vertices.tolist())
    proxy.CreateFaceVertexCountsAttr([3] * len(hull.faces))
    proxy.CreateFaceVertexIndicesAttr(hull.faces.reshape(-1).tolist())
    collision(proxy.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(proxy.GetPrim()).CreateApproximationAttr("convexHull")
    proxy.MakeInvisible()
    frame = UsdGeom.Xform.Define(stage, f"{tool_path}/ring_center")
    frame.AddTranslateOp().Set(Gf.Vec3d(*RING_CENTER))
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    stage.GetRootLayer().Save()
    print(f"[ASSET] {output_path}: {restored} restored colliders, {len(points)-1} ring segments", flush=True)
    return output_path


def make_joint(stage, path, fruit_path, position, quaternion, local_anchor, break_force, break_torque, anchor_path=None):
    """Standalone joint at the pedicel attachment, optionally relative to a stem body."""
    # Publish a complete joint in one USD change. Authoring its live attributes
    # individually makes the GUI PhysX parser process partial joint updates and
    # call wakeUp on the kinematic attachment (invalid in PhysX).
    joint_stage = Usd.Stage.CreateInMemory()
    joint = UsdPhysics.FixedJoint.Define(joint_stage, path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(fruit_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*position))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_anchor))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(quaternion[0]), Gf.Vec3f(*quaternion[1:])))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    if anchor_path:
        joint.CreateBody0Rel().SetTargets([Sdf.Path(anchor_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateBreakForceAttr().Set(float(break_force))
    joint.CreateBreakTorqueAttr().Set(float(break_torque))
    joint.CreateExcludeFromArticulationAttr(True)
    edit_target = stage.GetEditTarget()
    if not Sdf.CopySpec(joint_stage.GetRootLayer(), Sdf.Path(path), edit_target.GetLayer(),
                        edit_target.MapToSpecPath(Sdf.Path(path))):
        raise RuntimeError(f"Failed to publish harvest joint: {path}")
    return UsdPhysics.FixedJoint.Get(stage, path)
