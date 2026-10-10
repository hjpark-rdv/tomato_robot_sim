"""Loaded native belt-transport control using the unchanged V5.1 tool.

This opt-in diagnostic adds a synthetic sphere, not a farm fruit. Its center is
guided along the authored CAD belt axis; contacts with the actual moving belts
must supply transport. There is no attachment to a belt, jaw, or plant, and no
harvest or grasp-success claim. Call before native physics initialization.
"""
from __future__ import annotations

import math

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema


def build_belt_control(stage, manifest, *, tool_root='/World/ActuatedTool',
                       diameter_m=.03, density_kg_m3=1000.):
    """Author one cloneable fixture and return its concrete native body path.

    A world-anchored D6 guide locks CAD X/Z translation only. CAD Y translation
    and all rotation remain free, so the body can move and spin under real belt
    contact. Source tool geometry, materials, drives, and surface velocities are
    neither edited nor copied by this helper. The guide and sphere live beneath
    ``tool_root`` so the existing motion replication adapter copies them too.

    The 30 mm default leaves 7 mm radial clearance in the raw initial 44 mm CAD
    gap and loads contact at the original 28 mm jaw minimum. A historical 40 mm
    control had invalid native startup contact despite 2 mm raw CAD clearance;
    raw dimensions do not prove clearance of the cooked collision shapes. The
    caller must reject penetration during native initialization before a trial.
    """
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        return _build_belt_control(stage, manifest, tool_root, diameter_m, density_kg_m3)


def _build_belt_control(stage, manifest, tool_root, diameter_m, density_kg_m3):
    if not math.isfinite(diameter_m) or diameter_m <= 0:
        raise ValueError('Sphere diameter must be finite and positive')
    if not math.isfinite(density_kg_m3) or density_kg_m3 <= 0:
        raise ValueError('Sphere density must be finite and positive')
    tool = manifest['tool']
    bounds = {row['name'][0]: row for row in tool['belt_bounds']}
    if set(bounds) != {'L', 'R'}:
        raise ValueError('Both original CAD belt bounds are required')
    left_lo = np.asarray(bounds['L']['minimum'], float)
    left_hi = np.asarray(bounds['L']['maximum'], float)
    right_lo = np.asarray(bounds['R']['minimum'], float)
    right_hi = np.asarray(bounds['R']['maximum'], float)
    if any(x.shape != (3,) or not np.isfinite(x).all()
           for x in (left_lo, left_hi, right_lo, right_hi)):
        raise ValueError('Invalid CAD belt bounds')
    if np.any(left_lo >= left_hi) or np.any(right_lo >= right_hi):
        raise ValueError('Empty CAD belt bounds')
    gap_m = float(right_lo[0] - left_hi[0])
    if not math.isclose(gap_m, tool['nominal_gap_mm'] / 1000., abs_tol=1e-6):
        raise ValueError('CAD inner belt gap does not match the authored tool')
    minimum_gap_m = tool['jaw_limits_mm'][0] / 1000.
    if not minimum_gap_m < diameter_m < gap_m:
        raise ValueError('Sphere must fit the initial gap and contact during native closing')
    radius = diameter_m / 2.
    # The same CAD grasp center used by motion_kinematics.grasp_pose. Verify its
    # entire contact band fits both original belt spans before authoring a body.
    local_center = np.array([(left_hi[0] + right_lo[0]) / 2., .093, .037])
    overlap_lo = np.maximum(left_lo[1:], right_lo[1:])
    overlap_hi = np.minimum(left_hi[1:], right_hi[1:])
    if np.any(local_center[1:] - radius < overlap_lo) or np.any(local_center[1:] + radius > overlap_hi):
        raise ValueError('Diagnostic sphere does not fit the original CAD belt contact span')
    fixed_path = tool_root + '/fixed'
    fixed = stage.GetPrimAtPath(fixed_path)
    if not fixed or not fixed.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Native fixed tool body is missing')
    namespace = tool_root + '/Diagnostics/BeltControl'
    if stage.GetPrimAtPath(namespace):
        raise ValueError('Belt-control fixture already exists')
    cache = UsdGeom.XformCache()
    fixed_world = np.asarray(cache.GetLocalToWorldTransform(fixed), float).T
    rotation = fixed_world[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < 0:
        raise ValueError('Tool transform must have unit scale and proper rotation')
    center_world = rotation @ local_center + fixed_world[:3, 3]
    fixture_pose = fixed_world.copy()
    fixture_pose[:3, 3] = center_world
    body_path = namespace + '/Sphere'
    guide_path = namespace + '/Guide'
    parent = UsdGeom.Xform.Define(stage, namespace)
    parent_pose = np.asarray(cache.GetLocalToWorldTransform(parent.GetPrim()), float).T
    body_local = np.linalg.inv(parent_pose) @ fixture_pose
    sphere = UsdGeom.Sphere.Define(stage, body_path)
    sphere.CreateRadiusAttr(radius)
    sphere.CreateDisplayColorAttr([Gf.Vec3f(.15, .7, .95)])
    sphere.MakeMatrixXform().Set(Gf.Matrix4d(body_local.T.tolist()))
    body = sphere.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    UsdPhysics.CollisionAPI.Apply(body).CreateCollisionEnabledAttr(True)
    mass_kg = density_kg_m3 * 4. * math.pi * radius ** 3 / 3.
    inertia_kg_m2 = 2. * mass_kg * radius ** 2 / 5.
    mass = UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr(mass_kg)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia_kg_m2))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    native_body = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    native_body.CreateDisableGravityAttr(True)
    native_body.CreateEnableCCDAttr(True)
    native_body.CreateSleepThresholdAttr(0.)
    # Use the existing collision offsets and default material. No friction
    # changes are made to help transport or to pass a CPU/GPU comparison.
    collider = PhysxSchema.PhysxCollisionAPI.Apply(body)
    collider.CreateContactOffsetAttr(.0005)
    collider.CreateRestOffsetAttr(0.)
    PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.)
    guide = UsdPhysics.Joint.Define(stage, guide_path)
    guide.CreateBody1Rel().SetTargets([body_path])
    guide.CreateExcludeFromArticulationAttr(True)
    guide.CreateCollisionEnabledAttr(False)
    # Missing body0 means the authored slot-0 frame is in world coordinates.
    frame = Gf.Matrix4d(fixture_pose.T.tolist()).ExtractRotationQuat()
    guide.CreateLocalPos0Attr(Gf.Vec3f(*map(float, center_world)))
    guide.CreateLocalRot0Attr(Gf.Quatf(frame))
    guide.CreateLocalPos1Attr(Gf.Vec3f(0))
    guide.CreateLocalRot1Attr(Gf.Quatf(1))
    for axis in ('transX', 'transZ'):
        limit = UsdPhysics.LimitAPI.Apply(guide.GetPrim(), axis)
        limit.CreateLowAttr(1.)
        limit.CreateHighAttr(-1.)
    body.SetCustomDataByKey('diagnosticFixture', 'native_belt_transport_control')
    body.SetCustomDataByKey('farmFruit', False)
    return dict(body_path=body_path, guide_path=guide_path,
                start_center_world=center_world.tolist(),
                start_center_tool_local=local_center.tolist(),
                transport_axis_world=rotation[:, 1].tolist(),
                tool_frame_world=rotation.tolist(),
                source_belt_paths=list(tool['belts']),
                diameter_m=diameter_m, density_kg_m3=density_kg_m3,
                mass_kg=mass_kg, inertia_kg_m2=inertia_kg_m2,
                nominal_gap_m=gap_m, requested_closed_gap_m=minimum_gap_m,
                command_closed_gap_m=minimum_gap_m,
                initial_raw_cad_radial_clearance_m=(gap_m - diameter_m) / 2.,
                native_startup_validation='required; raw CAD clearance is not cooked-shape clearance',
                fixture_only=True, harvest_success=None,
                guide='world D6: CAD X/Z translation locked; CAD Y and all rotation free',
                limitations=['Synthetic sphere, not a farm fruit',
                             'No belt or plant attachment; transport requires native contact',
                             'Gravity disabled for this diagnostic body only',
                             'Raw CAD gap does not establish native cooked collision clearance',
                             'Loaded transport and CPU/GPU equivalence require actual execution'])
