"""Replace the virtual shelf with the existing CAD carriage, in a NEW assembly.

Run with .asset-venv. The existing prismatic joint/drives/body identities stay
in use. No harvesting trajectory is reused after the mounting correction.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

ROBOT = '/World/HarvestRobot/Robot'
PLATFORM = ROBOT + '/farmily_lift_platform'
LIFT_MESH = ROBOT + '/world/visuals/assy_lift/node_STL_BINARY_/mesh'
MOVING_IDS = {1: 'back_carriage', 4: 'mounting_plate', 10: 'cantilever_bracket'}


def resolve_carrier_archive(manifest, data_root=Path('/root/docker_share/mujoko_debugging_data')):
    """Relocate only missing pinned source evidence; keep exact SHA checks."""
    lineage=manifest.get('rail_chassis_body',{})
    original=Path(lineage.get('source_assembly',''))
    if not original.is_relative_to(data_root) or (original/'scene.usda').is_file():return
    archived=data_root.with_name(data_root.name+'_bak')/original.relative_to(data_root)
    if not (archived/'scene.usda').is_file():return
    if sha(archived/'scene.usda')!=lineage.get('source_scene_sha256'):
        raise ValueError('Archived carrier evidence SHA mismatch')
    lineage['source_assembly']=str(archived)
    manifest['real_lift_source_archive']=dict(original=str(original),resolved=str(archived),
        source_scene_sha256=lineage['source_scene_sha256'],geometry_changed=False)


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def matrix(stage, path):
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def components(stage):
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(LIFT_MESH))
    if not mesh or not np.all(np.asarray(mesh.GetFaceVertexCountsAttr().Get()) == 3):
        raise ValueError('Expected original triangulated assy_lift mesh')
    points = np.asarray(mesh.GetPointsAttr().Get(), float)
    faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), int).reshape(-1, 3)
    tf = matrix(stage, LIFT_MESH)
    vertices = points @ tf[:3, :3].T + tf[:3, 3]
    welded = trimesh.Trimesh(vertices, faces, process=False)
    welded.merge_vertices()
    cc = trimesh.graph.connected_components(welded.face_adjacency, nodes=np.arange(len(faces)), min_len=1)
    cc = sorted(cc, key=lambda ids: (-len(ids), *vertices[faces[ids]].reshape(-1, 3).min(0)))
    # Fail closed if a different CAD revision would invalidate the reviewed map.
    if len(faces) != 120881 or len(cc) != 48 or [len(cc[x]) for x in MOVING_IDS] != [13510, 6988, 3282]:
        raise ValueError('Lift CAD topology changed: review moving component map')
    return mesh, vertices, faces, cc


def apply(stage):
    if stage.GetPrimAtPath(PLATFORM + '/visuals/real_lift_carriage'):
        raise ValueError('Real-lift correction is already present')
    mesh, vertices, faces, cc = components(stage)
    old_tf = {str(p.GetPath()): matrix(stage, str(p.GetPath()))
              for root in (ROBOT, '/World/ActuatedTool')
              for p in Usd.PrimRange(stage.GetPrimAtPath(root))
              if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    # The CAD top plate is almost horizontal (original import used 1.57 rad).
    # Fit its actual top face; seat the upright RB5 base without interpenetration.
    plate = vertices[faces[cc[4]]]
    normals = np.cross(plate[:, 1]-plate[:, 0], plate[:, 2]-plate[:, 0])
    area = np.linalg.norm(normals, axis=1)
    unit = normals / np.maximum(area[:, None], 1e-30)
    top = (np.abs(unit[:, 2]) > .9999) & (plate.mean(1)[:, 2] > np.median(plate[:, :, 2]))
    plane_points = plate[top].reshape(-1, 3)
    coef = np.linalg.lstsq(np.c_[plane_points[:, :2], np.ones(len(plane_points))], plane_points[:, 2], rcond=None)[0]
    if len(plane_points) < 3 or np.max(np.abs(np.c_[plane_points[:, :2], np.ones(len(plane_points))] @ coef - plane_points[:, 2])) > 1e-6:
        raise ValueError('Could not identify a single mounting plane')
    base_path = PLATFORM + '/visuals/link0/Scene/mesh'
    base_tf = matrix(stage, base_path)
    base = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(base_path)).GetPointsAttr().Get())
    base = base @ base_tf[:3, :3].T + base_tf[:3, 3]
    foot = base[base[:, 2] < base[:, 2].min() + 1e-6]
    surface_z = np.c_[foot[:, :2], np.ones(len(foot))] @ coef
    dz = float(surface_z.max() - foot[:, 2].min())
    if not -.01 < dz < 0:
        raise ValueError('Unexpected RB5 mount offset: ' + str(dz))
    # Move the arm/tool zero-pose assembly, and the joint's parent frame, together.
    moved = [p for p in old_tf if p == PLATFORM or p.startswith(ROBOT+'/link') or p.startswith('/World/ActuatedTool/')]
    for path in moved:
        prim = stage.GetPrimAtPath(path)
        parent = str(prim.GetParent().GetPath())
        target = old_tf[path].copy(); target[2, 3] += dz
        local = np.linalg.inv(matrix(stage, parent)) @ target
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))
    joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(ROBOT+'/joints/farmily_lift_height_joint'))
    p = np.array(joint.GetLocalPos0Attr().Get(), float)
    p += old_tf[ROBOT+'/world'][:3, :3].T @ np.array([0, 0, dz])
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*p))
    source_tf = matrix(stage, LIFT_MESH)
    source_normals = np.asarray(mesh.GetNormalsAttr().Get(), float)
    if mesh.GetNormalsInterpolation() != 'faceVarying' or len(source_normals) != len(faces)*3:
        raise ValueError('Expected original face-varying CAD normals')
    normals_world = source_normals @ np.linalg.inv(source_tf[:3, :3])
    normals_world /= np.linalg.norm(normals_world, axis=1)[:, None]
    colors = mesh.GetDisplayColorAttr().Get()
    records = []
    for i, ids in enumerate(cc):
        owner = PLATFORM if i in MOVING_IDS else ROBOT+'/world'
        group = 'real_lift_carriage' if i in MOVING_IDS else 'real_lift_fixed'
        name = MOVING_IDS.get(i, f'fixed_{i:02d}')
        dest = owner+'/visuals/'+group+'/'+name
        inv = np.linalg.inv(matrix(stage, owner))
        unique, inverse = np.unique(faces[ids].ravel(), return_inverse=True)
        local = vertices[unique] @ inv[:3, :3].T + inv[:3, 3]
        m = UsdGeom.Mesh.Define(stage, dest)
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(local.astype(np.float32)))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3]*len(ids)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(inverse.tolist()))
        m.CreateSubdivisionSchemeAttr('none')
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy((normals_world.reshape(-1,3,3)[ids].reshape(-1,3) @ inv[:3,:3].T).astype(np.float32)))
        m.SetNormalsInterpolation('faceVarying')
        m.CreateDisplayColorAttr(colors if colors else [(0.72, 0.72, 0.72)])
        records.append(dict(component=i, name=name, owner=owner, path=dest, triangles=len(ids),
                            source_world_bounds_m=[vertices[unique].min(0).tolist(), vertices[unique].max(0).tolist()]))
        if i == 4:
            # Only replace the pre-existing shelf collider. Other lift CAD was
            # visual-only before and remains so; no invented solid convex truss.
            c = UsdGeom.Mesh.Define(stage, PLATFORM+'/collisions/real_mounting_plate')
            c.CreatePointsAttr(m.GetPointsAttr().Get())
            c.CreateFaceVertexCountsAttr(m.GetFaceVertexCountsAttr().Get())
            c.CreateFaceVertexIndicesAttr(m.GetFaceVertexIndicesAttr().Get())
            c.CreateSubdivisionSchemeAttr('none')
            c.CreateVisibilityAttr('invisible')
            UsdPhysics.CollisionAPI.Apply(c.GetPrim()).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(c.GetPrim()).CreateApproximationAttr('convexHull')
    for path in (LIFT_MESH, PLATFORM+'/visuals/mesh_0/box', PLATFORM+'/collisions/mesh_0/box'):
        stage.GetPrimAtPath(path).SetActive(False)
    return dict(schema='s300-real-lift-mount-v1', source_mesh=LIFT_MESH,
                source_triangles=len(faces), output_triangles=sum(r['triangles'] for r in records),
                components=records, moving_components=sorted(MOVING_IDS),
                joint=str(joint.GetPath()), range_m=[joint.GetLowerLimitAttr().Get(),joint.GetUpperLimitAttr().Get()],
                arm_mount_offset_z_m=dz, mounting_plane_z_ax_by_c=coef.tolist(),
                rb5_base_gap_range_m=[0., float(surface_z.max()-surface_z.min())],
                virtual_shelf_removed=True, existing_joint_drives_and_mass_estimates_preserved=True,
                collision_change='Virtual shelf box replaced by actual top-plate convex hull. Remaining lift CAD stays visual-only as before.',
                hardware_stroke_and_mass_calibration_verified=False,
                old_harvest_plans_revalidated=False, training_eligible=False)


def export_robot_check(stage, manifest, output):
    """Reuse the existing USD articulation adapter, with collision geometry only."""
    repo = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo/'mujoco-benchmark/scripts/s300_smoke'))
    from robot_adapter import add_robot
    output.mkdir()
    root = ET.Element('mujoco', model='s300_real_lift_mount_check')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false', meshdir='meshes')
    opt = ET.SubElement(root,'option', timestep=str(1/240),integrator='implicitfast',solver='Newton',iterations='64',tolerance='1e-8',cone='elliptic')
    ET.SubElement(opt,'flag',filterparent='disable',autoreset='disable')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'geom',friction='.5 0 0',condim='3',margin='0',gap='.0005',solref='.002 1',solimp='.99 .999 .001',density='0')
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prefix in (ROBOT, '/World/ActuatedTool'):
            for prim in Usd.PrimRange(stage.GetPrimAtPath(prefix)):
                if prim.IsA(UsdGeom.Gprim) and not prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdGeom.Imageable(prim).MakeInvisible()
        result = add_robot(stage, manifest, root=root, world=ET.SubElement(root,'worldbody'),
                           assets=ET.SubElement(root,'asset'), contact=ET.SubElement(root,'contact'),meshdir=output/'meshes')
    ET.indent(root)
    ET.ElementTree(root).write(output/'scene.xml',encoding='unicode')
    (output/'robot.json').write_text(json.dumps(result['metadata'],indent=2)+'\n')


def prepare(assembly, output, robot_check=False):
    source=assembly/'composed.usdc'; source_sha=sha(source)
    output.mkdir(parents=True,exist_ok=False)
    stage=Usd.Stage.CreateNew(str(output/'scene.usda'))
    stage.GetRootLayer().subLayerPaths=[str(source.resolve())]
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    result=apply(stage)
    stage.Flatten().Export(str(output/'composed.usdc'))
    # Publish a pure local wrapper, so native Isaac and replay open exactly the
    # same self-contained materialization. Do not retain old source overrides.
    stage.GetRootLayer().Clear()
    stage.GetRootLayer().subLayerPaths=['composed.usdc']
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    stage.GetRootLayer().Save()
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    resolve_carrier_archive(manifest)
    manifest['real_lift_mount']=result
    manifest['scene_sha256']=sha(output/'scene.usda')
    manifest.setdefault('compact_assets',{})['materialized_dependency']=dict(relative_path='composed.usdc',sha256=sha(output/'composed.usdc'),format='usdc',materialized=True,external_usd_sublayers=[])
    manifest['compact_assets']['geometry_unchanged']=False
    manifest['compact_assets']['native_validation']='See separate real-lift robot check; previous harvest validation does not transfer'
    manifest['real_lift_source']=dict(assembly=str(assembly),sha256=source_sha)
    (output/'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    for path in assembly.glob('skin*.npz'):
        shutil.copy2(path,output/path.name)
    (output/'real_lift_mount.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if robot_check:
        export_robot_check(Usd.Stage.Open(str(output/'composed.usdc')),manifest,output/'robot_check')
    if sha(source)!=source_sha:raise RuntimeError('Source changed')
    print(json.dumps({k:result[k] for k in ('moving_components','arm_mount_offset_z_m','range_m','rb5_base_gap_range_m')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('assembly',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--export-robot-check',action='store_true');a=p.parse_args()
    prepare(a.assembly,a.output,a.export_robot_check)
