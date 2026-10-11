"""Opt-in physical world-Y rail axis for the existing S300 motion assembly.

Full farm and visual CAD are preserved. Static S300 triangle colliders become
per-part convex decompositions because they now belong to a moving rigid body.
Rail drive and chassis mass are explicit simulator assumptions, not calibrated
wheel traction or a new harvesting pipeline. Source assets are never modified.
"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_assets import ROBOT, attr
from motion_stage_metrics import copy_stage_metrics, require_motion_units

ROOT_JOINT = ROBOT + '/root_joint'
BASE_BODY = ROBOT + '/world'
OLD_CHASSIS = '/World/HarvestRobot/S300'
NEW_CHASSIS = BASE_BODY + '/S300Chassis'
RAIL_ANCHOR = ROBOT + '/rail_anchor'
RAIL_FIXED_ROOT = ROBOT + '/rail_world_fixed'
LAYOUT = Path(__file__).resolve().parents[2] / 'mujoco-benchmark/validation/covered_d1_server/covered_scene_metadata.json'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def world_meshes(stage, root=None):
    """Actual original mesh vertices, including instance proxy world transforms."""
    cache = UsdGeom.XformCache()
    prim = stage.GetPrimAtPath(root or '/World')
    if not prim:
        raise ValueError('Missing mesh root: ' + str(root))
    result = {}
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        transform = np.asarray(cache.GetLocalToWorldTransform(p))
        vertices = np.asarray(m.GetPointsAttr().Get(), dtype=float)
        result[str(p.GetPath())] = vertices @ transform[:3, :3] + transform[3, :3]
    return result


def enabled_colliders(stage):
    return {str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath('/World'), Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.CollisionAPI) and
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False}


def _inertia_tensor(api):
    inertia = np.asarray(api.GetDiagonalInertiaAttr().Get(), dtype=float)
    if inertia.shape != (3,) or not np.isfinite(inertia).all() or (inertia <= 0).any():
        raise ValueError('Source moving base needs explicit positive inertia')
    q = api.GetPrincipalAxesAttr().Get()
    r = Rotation.from_quat([*q.GetImaginary(), q.GetReal()]).as_matrix()
    return r @ np.diag(inertia) @ r.T


def _parallel_axis(mass, offset):
    return mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))


def internalize_rail_root(stage, root, root_pose, base_pose):
    # A movable joint directly against world is an external root constraint in
    # native PhysX and is not exposed as an articulation DOF. Keep a fixed,
    # collider-free root link and put the driven rail joint inside the existing
    # articulation so native tensor control actually owns the travel axis.
    anchor = UsdGeom.Xform.Define(stage, RAIL_ANCHOR)
    anchor_pose = np.eye(4);anchor_pose[:3,3] = base_pose[:3,3]
    # Robot is mounted below HarvestRobot and has its own lift/base transform.
    # Use the actual immediate parent, never the chassis root transform.
    parent_pose = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(anchor.GetPrim().GetParent())).T
    anchor.AddTransformOp().Set(Gf.Matrix4d((np.linalg.inv(parent_pose) @ anchor_pose).T.tolist()))
    UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(anchor.GetPrim());mass.CreateMassAttr(1.)
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.01,.01,.01))
    fixed = UsdPhysics.FixedJoint.Define(stage, RAIL_FIXED_ROOT)
    fixed.CreateBody1Rel().SetTargets([RAIL_ANCHOR])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*map(float, anchor_pose[:3,3])))
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0.))
    fixed.CreateLocalRot0Attr(Gf.Quatf(1.));fixed.CreateLocalRot1Attr(Gf.Quatf(1.))
    UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim())
    for attribute in root.GetAttributes():
        if attribute.GetName().startswith('physxArticulation:') and attribute.Get() is not None:
            attr(fixed.GetPrim(),attribute.GetName(),attribute.GetTypeName(),attribute.Get())
    root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    root.SetTypeName('PhysicsPrismaticJoint')
    joint = UsdPhysics.PrismaticJoint(root)
    joint.CreateBody0Rel().SetTargets([RAIL_ANCHOR])
    return joint


def convert(stage, *, rail_y_range=(0., 5.), chassis_mass_kg=150.,
            stiffness_n_m=100000., damping_n_s_m=5000., max_force_n=10000.):
    """Modify only a caller-owned derived stage; the caller saves immutably."""
    low, high = map(float, rail_y_range)
    scalars = [low, high, chassis_mass_kg, stiffness_n_m, damping_n_s_m, max_force_n]
    if not np.isfinite(scalars).all() or low >= high or min(scalars[2:]) <= 0:
        raise ValueError('Finite increasing rail range and positive mass/drive values required')
    root = stage.GetPrimAtPath(ROOT_JOINT)
    base = stage.GetPrimAtPath(BASE_BODY)
    chassis = stage.GetPrimAtPath(OLD_CHASSIS)
    robot_root = stage.GetPrimAtPath('/World/HarvestRobot')
    if not root or not root.IsA(UsdPhysics.FixedJoint) or not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise ValueError('Expected original fixed articulation root joint')
    j = UsdPhysics.Joint(root)
    if j.GetBody0Rel().GetTargets() or list(map(str, j.GetBody1Rel().GetTargets())) != [BASE_BODY]:
        raise ValueError('Original root must connect world to exact robot base')
    if not base or not base.HasAPI(UsdPhysics.RigidBodyAPI) or not chassis or not chassis.IsActive():
        raise ValueError('Missing original robot base or active S300 CAD')
    cache = UsdGeom.XformCache()
    root_pose = np.asarray(cache.GetLocalToWorldTransform(robot_root)).T
    base_pose = np.asarray(cache.GetLocalToWorldTransform(base)).T
    chassis_pose = np.asarray(cache.GetLocalToWorldTransform(chassis)).T
    if not np.allclose(root_pose[:3, :3], np.eye(3), atol=1e-9):
        raise ValueError('Current rail fixture requires the original world-aligned chassis')
    station_y = float(root_pose[1, 3])
    if not low <= station_y <= high:
        raise ValueError('Original station is outside the requested rail range')
    original_meshes = world_meshes(stage)
    original_colliders = enabled_colliders(stage)
    chassis_vertices = np.concatenate([v for p, v in original_meshes.items() if p.startswith(OLD_CHASSIS + '/')])
    chassis_local = (chassis_vertices - base_pose[:3, 3]) @ base_pose[:3, :3]
    bounds = np.array([chassis_local.min(0), chassis_local.max(0)])
    added_com = bounds.mean(0)
    size = bounds[1] - bounds[0]
    added_inertia = chassis_mass_kg / 12 * np.diag([
        size[1]**2 + size[2]**2, size[0]**2 + size[2]**2, size[0]**2 + size[1]**2])
    mass_api = UsdPhysics.MassAPI(base)
    previous_mass = float(mass_api.GetMassAttr().Get())
    previous_com = np.asarray(mass_api.GetCenterOfMassAttr().Get(), dtype=float)
    if previous_mass <= 0 or previous_com.shape != (3,) or not np.isfinite(previous_com).all():
        raise ValueError('Source moving base needs finite explicit mass and COM')
    source_inertia = _inertia_tensor(mass_api)
    combined_mass = previous_mass + chassis_mass_kg
    combined_com = (previous_mass * previous_com + chassis_mass_kg * added_com) / combined_mass
    combined_inertia = source_inertia + added_inertia + _parallel_axis(previous_mass, previous_com - combined_com) + _parallel_axis(chassis_mass_kg, added_com - combined_com)
    eigenvalues, eigenvectors = np.linalg.eigh(combined_inertia)
    if np.linalg.det(eigenvectors) < 0:
        eigenvectors[:, -1] *= -1
    q = Rotation.from_matrix(eigenvectors).as_quat()
    mass_api.CreateMassAttr(combined_mass)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, combined_com)))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, eigenvalues)))
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
    editor = Usd.NamespaceEditor(stage)
    if not editor.ReparentPrim(chassis, base, 'S300Chassis') or not editor.CanApplyEdits() or not editor.ApplyEdits():
        raise RuntimeError('Cannot attach original S300 CAD to moving base')
    new = stage.GetPrimAtPath(NEW_CHASSIS)
    UsdGeom.Xformable(new).MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(base_pose) @ chassis_pose).T.tolist()))
    approximations = []
    for p in Usd.PrimRange(new):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Unexpected additional S300 rigid body')
        if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Mesh):
            old = p.GetAttribute('physics:approximation').Get()
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexDecomposition')
            approximations.append(dict(path=str(p.GetPath()),source=old,new='convexDecomposition'))
    joint = internalize_rail_root(stage, root, root_pose, base_pose)
    joint.CreateAxisAttr('Y')
    joint.CreateLowerLimitAttr(low - station_y)
    joint.CreateUpperLimitAttr(high - station_y)
    # The fixed anchor is world-aligned; its +Y matches the physical rail.
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
    q = Rotation.from_matrix(base_pose[:3, :3].T).as_quat()
    joint.CreateLocalRot1Attr(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
    drive = UsdPhysics.DriveAPI.Apply(root, 'linear')
    drive.CreateTypeAttr('force')
    drive.CreateTargetPositionAttr(0.)
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateStiffnessAttr(stiffness_n_m)
    drive.CreateDampingAttr(damping_n_s_m)
    drive.CreateMaxForceAttr(max_force_n)
    root.AddAppliedSchema('PhysicsJointStateAPI:linear')
    attr(root, 'state:linear:physics:position', Sdf.ValueTypeNames.Float, 0.)
    attr(root, 'state:linear:physics:velocity', Sdf.ValueTypeNames.Float, 0.)
    def mapped(path):
        return NEW_CHASSIS + path[len(OLD_CHASSIS):] if path.startswith(OLD_CHASSIS + '/') else path
    result_meshes = world_meshes(stage)
    error = max(float(np.max(np.abs(result_meshes[mapped(p)] - v))) for p, v in original_meshes.items())
    if error > 1e-8 or set(result_meshes) != {mapped(p) for p in original_meshes}:
        raise RuntimeError('Rail attachment changed original visual geometry')
    if enabled_colliders(stage) != {mapped(p) for p in original_colliders}:
        raise RuntimeError('Rail attachment changed collider coverage')
    return dict(schema='s300-world-y-rail-v2',dof_name='root_joint',joint=ROOT_JOINT,
        fixed_root_joint=RAIL_FIXED_ROOT,anchor_body=RAIL_ANCHOR,
        base_body=BASE_BODY,old_chassis_root=OLD_CHASSIS,chassis_root=NEW_CHASSIS,
        axis_world=[0.,1.,0.],initial_station_y_m=station_y,
        limits_world_y_m=[low,high],limits_relative_m=[low-station_y,high-station_y],
        source_base_position_world_m=base_pose[:3,3].tolist(),
        source_robot_root_position_world_m=root_pose[:3,3].tolist(),
        initial_displacement_m=0.,added_dofs=1,
        mass=dict(original_base_kg=previous_mass,assumed_chassis_kg=chassis_mass_kg,
                  combined_base_kg=combined_mass,combined_com_base_m=combined_com.tolist(),
                  principal_inertia_kg_m2=eigenvalues.tolist(),
                  assumption='S300 mass is explicit uncalibrated input; uniform CAD bounding-box inertia with source inertia/parallel-axis combination'),
        drive=dict(stiffness_n_m=stiffness_n_m,damping_n_s_m=damping_n_s_m,max_force_n=max_force_n,
                   model='Ideal driven rail prismatic actuator; wheel traction/slip and rail-wheel mechanism unmodeled'),
        visual_meshes=len(result_meshes),visual_world_vertex_error_m=error,
        collider_count=len(original_colliders),chassis_collider_approximation_changes=approximations,
        collision_note='Only moving S300 mesh colliders converted from static triangle mesh to per-part convex decomposition; all farm/tool/active plant colliders preserved',
        native_validation='pending',training_eligible=False,harvest_success=None)


def revise(assembly, output, **kwargs):
    assembly, output = Path(assembly).resolve(), Path(output).resolve()
    source = assembly / 'scene.usda'
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    digest = sha(source)
    if manifest.get('scene_sha256') != digest:
        raise ValueError('Source assembly hash mismatch')
    if 'rail' in manifest or output.exists():
        raise ValueError('Use an untouched source and a new output directory')
    original = Usd.Stage.Open(str(source))
    require_motion_units(original)
    stage = Usd.Stage.Open(original.Flatten())
    copy_stage_metrics(original, stage)
    rail = convert(stage, **kwargs)
    output.mkdir(parents=True, exist_ok=False)
    stage.Flatten().Export(str(output / 'scene.usda'))
    shutil.copy2(assembly / 'skin.npz', output / 'skin.npz')
    manifest['rail'] = dict(rail,source_assembly=str(assembly),source_scene_sha256=digest)
    manifest['scene_sha256'] = sha(output / 'scene.usda')
    manifest['training_eligible'] = False
    (output / 'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    if sha(source) != digest:
        raise RuntimeError('Source unexpectedly changed')
    return rail


def near_side_inventory(stage, layout=LAYOUT, *, active_manifest=None):
    """All actual trusses, with near/far root policy; reachability stays unknown."""
    data = json.loads(Path(layout).read_text()) if isinstance(layout,(str,Path)) else layout
    cache = UsdGeom.XformCache()
    rows = []
    for plant in data['neighbor_placements']:
        position = np.asarray(plant['position'],float)
        near = bool(abs(position[0]) < abs(float(plant['gutter_center_x'])))
        for i,placement in enumerate(plant['trusses']):
            path = f"/World/Greenhouse/Plants/Plant_{plant['id']:02d}/Truss_{i:02d}"
            fruits = {}
            plant_path = path.rsplit('/',1)[0]
            active = active_manifest and active_manifest['plant']['source_plant'] == plant_path
            if active:
                fruits = {f['name']:list(f['center']) for f in active_manifest['plant']['fruits']
                          if f['name'].startswith(f'Truss_{i:02d}_')}
            else:
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsActive():
                    raise ValueError('Missing actual truss for inventory: ' + path)
                bounds = {}
                for mesh in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()):
                    name = mesh.GetCustomDataByKey('sourceName') or ''
                    if mesh.IsA(UsdGeom.Mesh) and re.fullmatch(r'Fruit_\d+',name):
                        tf = np.asarray(cache.GetLocalToWorldTransform(mesh))
                        vertices = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
                        world = vertices @ tf[:3,:3] + tf[3,:3]
                        lo,hi = world.min(0),world.max(0)
                        if name in bounds:
                            lo,hi = np.minimum(lo,bounds[name][0]),np.maximum(hi,bounds[name][1])
                        bounds[name] = (lo,hi)
                fruits = {name:((lo+hi)/2).tolist() for name,(lo,hi) in bounds.items()}
            if not fruits:
                raise ValueError('No actual fruit geometry for inventory: '+path)
            centers = np.asarray(list(fruits.values()))
            rows.append(dict(plant_id=plant['id'],truss_index=i,path=path,stem_variant=plant['stem_variant'],
                root_position_world_m=position.tolist(),gutter_center_x=plant['gutter_center_x'],side=plant['side'],
                rail_near=near,selected_by_side_policy=near,
                exclusion_reason=None if near else 'opposite-side stem across gutter from central rail',
                reachability='unknown; placement/IK/path/physics must be tested',
                source_glb=str(placement['source_glb']),y_deg=placement['y_deg'],
                fruits=fruits,fruit_count=len(fruits),fruit_centroid_world_m=centers.mean(0).tolist(),
                fruit_height_minmax_m=[float(centers[:,2].min()),float(centers[:,2].max())]))
    return dict(schema='s300-rail-near-truss-inventory-v1',rail_axis_world=[0.,1.,0.],
        policy='Near-side stem roots only: abs(root x) < abs(gutter center x); no reachability inference',
        trusses=rows,total_trusses=len(rows),side_selected_trusses=sum(r['rail_near'] for r in rows),
        side_excluded_trusses=sum(not r['rail_near'] for r in rows),training_eligible=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--rail-y-range',type=float,nargs=2,default=[0.,5.])
    parser.add_argument('--chassis-mass-kg',type=float,default=150.)
    parser.add_argument('--inventory',type=Path)
    args = parser.parse_args()
    result = revise(args.assembly,args.output,rail_y_range=args.rail_y_range,chassis_mass_kg=args.chassis_mass_kg)
    if args.inventory:
        stage = Usd.Stage.Open(str(args.output/'scene.usda'))
        manifest = json.loads((args.output/'motion_manifest.json').read_text())
        args.inventory.write_text(json.dumps(near_side_inventory(stage,active_manifest=manifest),indent=2)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))
