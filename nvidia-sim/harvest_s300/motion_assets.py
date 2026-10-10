"""Opt-in native PhysX motion adapter; source CAD and farm remain untouched.

Original visual triangles, original elastic full segmentation and stiffness.
Jaw sliders are equivalent actuators for the single crank transmission. Belts
use PhysX surface velocity; belt deformation and motor transmission are not FEM.
"""
import argparse
import json
import re
import sys
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from build_scene import REPO, mesh, camera, sha
from motion_stage_metrics import copy_stage_metrics, require_motion_units
sys.path.insert(0, str(REPO / 'nvidia-sim/rl'))
from elastic_geometry import tube_centerline, resample_rod, point_segment_distances, mesh_components

ROBOT = '/World/HarvestRobot/Robot'
TOOL = ROBOT + '/link6/tcp/GripperV51'


def attr(prim, name, kind, value):
    prim.CreateAttribute(name, kind).Set(value)


def physx(prim, schema, **values):
    prim.AddAppliedSchema(schema)
    for name, (kind, value) in values.items():
        attr(prim, name, kind, value)


def transform(stage, path, pose):
    x = UsdGeom.Xform.Define(stage, path)
    x.MakeMatrixXform().Set(Gf.Matrix4d(np.asarray(pose).T.tolist()))
    return x.GetPrim()


def rigid(stage, path, pose, mass, inertia=None):
    prim = transform(stage, path, pose)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(float(mass))
    if inertia is not None:
        m.CreateCenterOfMassAttr(Gf.Vec3f(0))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    physx(prim, 'PhysxRigidBodyAPI', **{
        'physxRigidBody:enableCCD': (Sdf.ValueTypeNames.Bool, True),
        'physxRigidBody:sleepThreshold': (Sdf.ValueTypeNames.Float, 0.),
    })
    physx(prim, 'PhysxContactReportAPI', **{'physxContactReport:threshold': (Sdf.ValueTypeNames.Float, 0.)})
    return prim


def collision(prim):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    physx(prim, 'PhysxCollisionAPI', **{
        'physxCollision:contactOffset': (Sdf.ValueTypeNames.Float, .0005),
        'physxCollision:restOffset': (Sdf.ValueTypeNames.Float, 0.),
    })


def joint(stage, path, body0, body1, pose0, pose1, pivot, kind='spring', k=0., d=0., rotation=None):
    cls = {'spring': UsdPhysics.Joint, 'fixed': UsdPhysics.FixedJoint,
           'slider': UsdPhysics.PrismaticJoint}[kind]
    j = cls.Define(stage, path)
    if body0:
        j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    frame = np.eye(4)
    frame[:3, 3] = pivot
    if rotation is not None:
        frame[:3, :3] = rotation
    for slot, pose in enumerate((pose0, pose1)):
        t = np.linalg.inv(pose) @ frame
        q = R.from_matrix(t[:3, :3]).as_quat()
        getattr(j, f'CreateLocalPos{slot}Attr')().Set(Gf.Vec3f(*map(float, t[:3, 3])))
        getattr(j, f'CreateLocalRot{slot}Attr')().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
    j.CreateCollisionEnabledAttr(False)
    if kind == 'spring':
        for axis in ('transX', 'transY', 'transZ'):
            limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            limit.CreateLowAttr(1.); limit.CreateHighAttr(-1.)
        for axis in ('rotX', 'rotY', 'rotZ'):
            limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            limit.CreateLowAttr(-45.); limit.CreateHighAttr(45.)
            drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(float(k * np.pi / 180))
            drive.CreateDampingAttr(float(d * np.pi / 180))
            drive.CreateTargetPositionAttr(0.); drive.CreateMaxForceAttr(100.)
    return j


def geometry(prim, cache):
    m = UsdGeom.Mesh(prim)
    t = np.asarray(cache.GetLocalToWorldTransform(prim))
    v = np.asarray(m.GetPointsAttr().Get(), float) @ t[:3, :3] + t[3, :3]
    f = np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    return v, f, tuple(m.GetDisplayColorAttr().Get()[0])


def build_tool(stage, report):
    root_joint=stage.GetPrimAtPath(ROBOT+'/root_joint')
    attr(root_joint,'physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool,True)
    report['robot_self_collision_enabled']=True
    cache = UsdGeom.XformCache()
    pose = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(TOOL))).T
    parent_pose = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT + '/link6'))).T
    parts = {x['name']: x for x in json.loads((REPO / 'gripper-design/inaho_gripper_v5/parts_manifest.json').read_text())}
    source = []
    for p in stage.GetPrimAtPath(TOOL).GetChildren():
        if p.IsA(UsdGeom.Mesh):
            source.append((p.GetCustomDataByKey('sourceName'), *geometry(p, cache)))
    stage.OverridePrim(TOOL).SetActive(False)
    root = '/World/ActuatedTool'
    bodies = {}; mass_by_group = {}
    densities = {'steel': 7850., 'A2 stainless steel': 7900., 'PETG': 1270.,
                 'TPU 95A': 1200., 'aluminum': 2700., 'rubber': 1100.}
    def part_mass(part):
        return part['volume_mm3'] * 1e-9 * densities.get(part['material'], 1270.)
    for group in ('fixed', 'L', 'R'):
        # CAD part volume/material based estimate, explicitly not a measured payload.
        mass = sum(part_mass(x) for x in parts.values()
                   if (x['group'] == group or group == 'fixed' and x['group'] in ('crank', 'linkL', 'linkR'))
                   and not x['name'].endswith('C07_contact_belt_TPU'))
        mass_by_group[group] = mass
        path = root + '/' + group
        rigid(stage, path, pose, mass)
        bodies[group] = path
        j = joint(stage, root + '/Joints/jaw_' + group,
                  ROBOT + '/link6' if group == 'fixed' else bodies['fixed'], path,
                  parent_pose if group == 'fixed' else pose, pose, pose[:3, 3], 'fixed' if group == 'fixed' else 'slider',
                  rotation=pose[:3, :3])
        if group != 'fixed':
            j.CreateAxisAttr('X')
            j.CreateLowerLimitAttr(-.026 if group == 'L' else -.008)
            j.CreateUpperLimitAttr(.008 if group == 'L' else .026)
            drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), 'linear')
            drive.CreateTypeAttr('force'); drive.CreateStiffnessAttr(1500.)
            drive.CreateDampingAttr(30.); drive.CreateMaxForceAttr(10.)
            drive.CreateTargetPositionAttr(0.)
    # Each moving belt has its own fixed child body, so velocity applies only
    # to the belt's collider, never the motor, cover, or sliding jaw frame.
    belt_paths = []
    for group in ('L', 'R'):
        path = root + '/' + group + '_belt'
        belt_mass = part_mass(parts[group + '_C07_contact_belt_TPU'])
        mass_by_group[group + '_belt'] = belt_mass
        rigid(stage, path, pose, belt_mass)
        joint(stage, root + '/Joints/belt_' + group, bodies[group], path, pose, pose, pose[:3, 3], 'fixed')
        physx(stage.GetPrimAtPath(path), 'PhysxSurfaceVelocityAPI', **{
            'physxSurfaceVelocity:surfaceVelocity': (Sdf.ValueTypeNames.Vector3f, Gf.Vec3f(0)),
            'physxSurfaceVelocity:surfaceVelocityLocalSpace': (Sdf.ValueTypeNames.Bool, True),
        })
        belt_paths.append(path)
    inv = np.linalg.inv(pose)
    linkage = []
    belt_bounds = []
    for i, (name, v, f, color) in enumerate(source):
        spec = parts[name]; group = spec['group']
        v = v @ inv[:3, :3].T + inv[:3, 3]
        if group in ('crank', 'linkL', 'linkR'):
            owner = bodies['fixed'] + '/Transmission/' + group
            UsdGeom.Xform.Define(stage, owner)
        else:
            owner = bodies[group] + ('_belt' if name.endswith('C07_contact_belt_TPU') else '')
        m = mesh(stage, owner + '/part_' + str(i), v, f, color, approximation='convexDecomposition')
        m.GetPrim().SetCustomDataByKey('sourceName', name)
        collision(m.GetPrim())
        if group in ('crank', 'linkL', 'linkR'):
            UsdPhysics.CollisionAPI(m).CreateCollisionEnabledAttr(False)
            linkage.append(dict(name=name, group=group, path=str(m.GetPath()), motion=spec.get('motion')))
        if name.endswith('C07_contact_belt_TPU'):
            belt_bounds.append(dict(name=name, minimum=v.min(0).tolist(), maximum=v.max(0).tolist()))
    report['tool'] = dict(bodies=bodies, belts=belt_paths, mass_estimates_kg=mass_by_group,
                         mass_model='CAD volume times material density; motor/switch density fallback 1270 kg/m3; not measured',
                         jaw_limits_mm=[28., 96.], nominal_gap_mm=44., belt_bounds=belt_bounds,
                         linkage=linkage, transmission='equivalent coupled sliders; crank/link visuals follow measured gap; transmission collider disabled',
                         belt_model='rigid CAD belt hull decomposition + tangential surface velocity; no TPU deformation',
                         contact_material='PhysX default material, no friction tuning to obtain success')


def build_plant(stage, plant_path, report, output):
    """Bind actual placed mesh geometry to full spring/rod topology from ElasticPlant."""
    cache = UsdGeom.XformCache()
    plant = stage.GetPrimAtPath(plant_path)
    objects = {}
    for part in plant.GetChildren():
        group = {}
        for p in Usd.PrimRange(part, Usd.TraverseInstanceProxies()):
            if p.IsA(UsdGeom.Mesh):
                name = p.GetCustomDataByKey('sourceName')
                group.setdefault(name, []).append(geometry(p, cache))
        objects[part.GetName()] = group
    stage.OverridePrim(plant_path).SetActive(False)
    root = '/World/ElasticTarget'
    rp = UsdGeom.Xform.Define(stage, root).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(rp)
    physx(rp, 'PhysxArticulationAPI', **{
        'physxArticulation:enabledSelfCollisions': (Sdf.ValueTypeNames.Bool, True),
        'physxArticulation:solverPositionIterationCount': (Sdf.ValueTypeNames.UInt, 64),
        'physxArticulation:solverVelocityIterationCount': (Sdf.ValueTypeNames.UInt, 4),
        'physxArticulation:sleepThreshold': (Sdf.ValueTypeNames.Float, 0.),
        'physxArticulation:stabilizationThreshold': (Sdf.ValueTypeNames.Float, 0.),
    })
    records = []; chains = {}; fruit_records = []; skin = {}; exclusions = []
    def add_body(name, position, mass):
        pose = np.eye(4); pose[:3, 3] = position
        path = root + '/' + name
        rigid(stage, path, pose, mass)
        row = dict(name=name, path=path, rest_position=np.asarray(position).tolist(), mass=mass)
        records.append(row)
        return row, pose
    def exclude(a, b):
        rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel()
        rel.AddTarget(b); exclusions.append([a, b])
    def chain(group, label, sides, count, parent, k, d):
        v = objects[group][label][0][0]
        points, radii = resample_rod(*tube_centerline(v, sides), count)
        rows = []
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            radius = float(max(radii[i:i+2]))
            mass = max(.0005, 1000*np.pi*radius**2*np.linalg.norm(b-a))
            row, pose = add_body(group + '_' + label + '_' + str(i), (a+b)/2, float(mass))
            # Full ElasticPlant inertial model: foliage colliders do not move
            # the rod COM or replace its inertia by the leaf envelope inertia.
            length=float(np.linalg.norm(b-a));api=UsdPhysics.MassAPI(stage.GetPrimAtPath(row['path']))
            api.CreateCenterOfMassAttr(Gf.Vec3f(0))
            api.CreateDiagonalInertiaAttr(Gf.Vec3f(mass*(3*radius**2+length**2)/12,
                mass*(3*radius**2+length**2)/12,mass*radius**2/2))
            api.CreatePrincipalAxesAttr(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*(b-a))).GetQuat()))
            row.update(parent=parent['path'] if parent else None, pivot=a.tolist(), stiffness=k, damping=d)
            pp = np.eye(4)
            if parent: pp[:3, 3] = parent['rest_position']
            joint(stage, root + '/Joints/' + row['name'], row['parent'], row['path'], pp, pose, a,
                  'spring' if parent else 'fixed', k, d)
            cap = UsdGeom.Capsule.Define(stage, row['path'] + '/Collider')
            cap.CreateAxisAttr('Z'); cap.CreateRadiusAttr(radius); cap.CreateHeightAttr(float(np.linalg.norm(b-a)))
            cap.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1), Gf.Vec3d(*(b-a))).GetQuat()))
            cap.MakeInvisible(); collision(cap.GetPrim())
            if parent: exclude(row['path'], parent['path'])
            rows.append(row); parent=row
        chains[(group, label)] = (points, rows)
        return rows
    def nearest(rows, point):
        return min(rows, key=lambda x: np.linalg.norm(np.array(x['rest_position'])-point))
    main = chain('Stem', 'MainStem', 24, 16, None, 100., 3.)
    for group, meshes in objects.items():
        if group == 'Stem': continue
        ped_start = tube_centerline(meshes['Truss_01_Peduncle'][0][0], 14)[0][0]
        parent = nearest(main, ped_start)
        ped = chain(group, 'Truss_01_Peduncle', 14, 5, parent, 25., 1.)
        for neighbor in main:
            if abs(main.index(neighbor)-main.index(parent))<=1:
                exclude(ped[0]['path'], neighbor['path'])
        rachis = chain(group, 'Rachis', 14, 14, ped[-1], 2., .15)
        for i in range(1,12):
            suffix=f'{i:02d}'; label='Pedicel_proximal_'+suffix
            points, _=tube_centerline(meshes[label][0][0],12)
            owner=nearest(rachis,points[0])
            prox=chain(group,label,12,3,owner,.25,.02)
            dist=chain(group,'Pedicel_distal_'+suffix,12,1,prox[-1],.25,.02)
            # Same narrow junction exclusion used in existing GLB migration.
            from build_glb_physics import junction_neighbors
            cp, cr=resample_rod(*tube_centerline(meshes['Rachis'][0][0],14),14)
            pr=tube_centerline(meshes[label][0][0],12)[1]
            for j in junction_neighbors(cp,cr,points[0],pr[0],rachis.index(owner)):
                exclude(prox[0]['path'],rachis[j]['path'])
            fruit='Fruit_'+suffix
            if fruit not in meshes: continue
            v,f,color=meshes[fruit][0]; center=(v.max(0)+v.min(0))/2
            # Existing reference fruit mass scaled cubically by approved GLB .5.
            ref=json.loads((REPO/'mujoco-benchmark/assets/reference/reference.json').read_text())
            mass=next(b['mass'] for b in ref['bodies'] if b['name']=='Tomato_'+suffix)*.5**3
            row,pose=add_body(group+'_'+fruit,center,float(mass))
            pp=np.eye(4); pp[:3,3]=dist[-1]['rest_position']
            neck=tube_centerline(meshes['Pedicel_distal_'+suffix][0][0],12)[0][-1]
            j=joint(stage,root+'/HarvestJoints/'+row['name'],dist[-1]['path'],row['path'],pp,pose,neck,'fixed')
            j.CreateExcludeFromArticulationAttr(True);j.CreateBreakForceAttr(3.);j.CreateBreakTorqueAttr(.08)
            m=mesh(stage,row['path']+'/Collider',v-center,f,color,approximation='convexHull')
            m.MakeInvisible();collision(m.GetPrim());exclude(row['path'],dist[-1]['path'])
            row.update(joint=str(j.GetPath()),anchor=dist[-1]['path'],center=center.tolist(),
                       dimensions_m=np.ptp(v,axis=0).tolist(),break_force_n=3.,break_torque_nm=.08)
            fruit_records.append(row)
    # Preserve every original triangle. Tube skin uses the existing helper at render time;
    # leaf/calyx and fruit meshes follow the corresponding simulated body transform.
    # Match ElasticPlant._main_appendages: common attachment for each compound
    # leaf, connected-component convex proxies, no hairs/fine-vein convex hull.
    appendage_owner={};appendage_colliders=[];groups={}
    for label, primitives in objects['Stem'].items():
        if label in ('MainStem','MainStem_Trichomes'):continue
        match=re.match(r'((?:CompoundLeaf|Pruned_Branch|Apical_Young_Leaf)_\d+)',label)
        group=match.group(1) if match else ('Removed_Truss' if label.startswith('Removed_Truss') else label.removesuffix('_Trichomes'))
        groups.setdefault(group,[]).append(label)
    centers=np.array([r['rest_position'] for r in main])
    for labels in groups.values():
        vertices=np.concatenate([v for label in labels for v,_,_ in objects['Stem'][label]])
        owner=main[int(np.linalg.norm(vertices[:,None]-centers,axis=2).min(axis=0).argmin())]
        for label in labels:appendage_owner[label]=owner
    nvis=0
    for group, meshes in objects.items():
        for label, primitives in meshes.items():
            base=label.removesuffix('_Trichomes')
            for v,f,color in primitives:
                key=(group,base)
                if key in chains:
                    pts, rows=chains[key]
                    distances=point_segment_distances(v,pts[:-1],pts[1:])
                    ids=np.argsort(distances,axis=1)[:,:2]
                    weights=1/np.maximum(np.take_along_axis(distances,ids,axis=1),1e-6)**2
                    weights/=weights.sum(1)[:,None]
                    paths=[r['path'] for r in rows]
                    path=root+'/Skin/mesh_'+str(nvis)
                    m=mesh(stage,path,v,f,color)
                    UsdPhysics.CollisionAPI(m).CreateCollisionEnabledAttr(False)
                    skin[path]=dict(vertices=v,ids=ids,weights=weights,body_paths=np.array(paths),
                                    rest=np.array([r['rest_position'] for r in rows]))
                else:
                    if group!='Stem' and label.startswith(('Fruit_','Calyx_')):
                        tokens=label.split('_');suffix=tokens[2] if tokens[1]=='Hub' else tokens[1]
                        owner=next(x for x in fruit_records if x['name']==group+'_Fruit_'+suffix)
                    else:
                        owner=appendage_owner[label] if group=='Stem' else nearest(chains[(group,'Rachis')][1],v.mean(0))
                    path=owner['path']+'/Visual_'+str(nvis)
                    m=mesh(stage,path,v-np.array(owner['rest_position']),f,color,approximation='convexDecomposition')
                    UsdPhysics.CollisionAPI(m).CreateCollisionEnabledAttr(False)
                    if group=='Stem' and not any(x in label for x in ('Trichomes','Cut_Fibers')):
                        components=mesh_components(np.full(len(f),3),f.ravel(),len(v))
                        if len(components)<=20:
                            for ci,indices in enumerate(components):
                                if len(indices)<4:continue
                                mapping=np.full(len(v),-1);mapping[indices]=np.arange(len(indices))
                                ff=f[mapping[f[:,0]]>=0]
                                cp=mesh(stage,owner['path']+f'/Appendage_{nvis}_{ci}',
                                        v[indices]-np.array(owner['rest_position']),mapping[ff],color,approximation='convexHull')
                                cp.MakeInvisible();collision(cp.GetPrim());appendage_colliders.append(str(cp.GetPath()))
                m.GetPrim().SetCustomDataByKey('sourceName',label)
                nvis+=1
    for i,path in enumerate(appendage_colliders):
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path)).CreateFilteredPairsRel().SetTargets(appendage_colliders[i+1:])
    arrays={}
    skin_meta=[]
    for i,(path,data) in enumerate(skin.items()):
        for k,v in data.items():arrays[f'{i}_{k}']=v
        skin_meta.append(path)
    np.savez_compressed(output/'skin.npz',**arrays)
    report['plant']=dict(source_plant=plant_path,root=root,bodies=records,fruits=fruit_records,
                         skin_paths=skin_meta,visual_primitives=nvis,exclusions=exclusions,
                         stiffness='full ElasticPlant: main100/peduncle25/rachis2/pedicel.25 Nm/rad',
                         deformation='spring-linked rods, not calibrated continuum',breakage='native breakable fixed joints, original 3N/.08Nm thresholds')


def build(scene, output, plant=22):
    source_stage=Usd.Stage.Open(str(scene.resolve()))
    require_motion_units(source_stage)
    output.mkdir(parents=True,exist_ok=False)
    s=Usd.Stage.CreateNew(str(output/'scene.usda'))
    s.GetRootLayer().subLayerPaths=[str(scene.resolve())]
    metrics=copy_stage_metrics(source_stage,s)
    s.SetDefaultPrim(s.GetPrimAtPath('/World'))
    report=dict(source_scene=str(scene),source_sha256=sha(scene),training_eligible=False,harvest_success=None)
    report['stage_metrics']=metrics
    robot_xform=UsdGeom.Xformable(s.GetPrimAtPath(ROBOT))
    old=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(s.GetPrimAtPath(ROBOT))).T
    op=next(x for x in robot_xform.GetOrderedXformOps() if x.GetOpType()==UsdGeom.XformOp.TypeTranslate)
    p=np.asarray(op.Get(),float);p[1]-=.20;op.Set(Gf.Vec3d(*p))
    report['mount_revision']=dict(robot_translation_y_m=-.20,reason='moving 0.95m lift shelf crosses restored S300 rear panel at original mount',
                                  rear_panel_preserved=True,source_assembly_unchanged=True)
    build_tool(s,report)
    sys.path.insert(0,str(REPO/'mujoco-benchmark/scripts'))
    build_plant(s,f'/World/Greenhouse/Plants/Plant_{plant:02d}',report,output)
    # Original stable diagnostic rate; no reduction in plant segmentation or collision coverage.
    attr(s.GetPrimAtPath('/World/PhysicsScene'),'physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt,240)
    target=next(x for x in report['plant']['fruits'] if x['name']=='Truss_02_Fruit_02')
    c=np.array(target['center'])
    camera(s,'/World/HarvestCloseCamera',tuple(c+[.42,-.42,.23]),tuple(c))
    report['target']=target
    s.GetRootLayer().Save()
    report['scene_sha256']=sha(output/'scene.usda')
    (output/'motion_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(scene=str(output/'scene.usda'),bodies=len(report['plant']['bodies']),target=target),indent=2),flush=True)


def revise_for_horizontal_ready(assembly, output, distance=.30, yaw=30., seed_index=-1, target=None):
    """User-requested hardware/initial-state revision of an existing assembly.

    Retains the existing farm and motion runner. The receiver and its dedicated
    four fasteners are removed together, including their CAD mass estimate.
    JointStateAPI initializes a level, retracted ready pose before simulation;
    it is not a teleport or a pose correction during the harvesting motion.
    """
    from motion_kinematics import ToolKinematics, grasp_pose, plan_motion
    from motion_self_collision import MotionSelfCollision,check_plan
    if not np.isfinite([distance,yaw]).all() or not .15<=distance<=.5 or not -180<=yaw<=180:
        raise ValueError('Ready distance must be .15.. .5 m and yaw -180..180 degrees')
    source=assembly/'scene.usda'
    report=json.loads((assembly/'motion_manifest.json').read_text())
    if target is not None:
        choices=[p for p in report['plant']['fruits'] if p['name']==target]
        if len(choices)!=1:raise ValueError('Target must match exactly one existing active fruit')
        report['target']=choices[0]
    if sha(source)!=report['scene_sha256']:raise ValueError('Source assembly hash mismatch')
    original=Usd.Stage.Open(str(source.resolve()));require_motion_units(original)
    output.mkdir(parents=True,exist_ok=False)
    # Keep the adapter's authored rigid/mesh specs in its root layer. A further
    # sublayer introduces a different native mesh-update composition boundary.
    shutil.copy2(source,output/'scene.usda')
    s=Usd.Stage.Open(str(output/'scene.usda'))
    s.GetRootLayer().subLayerPaths=[str(Path(Sdf.ComputeAssetPathRelativeToLayer(
        original.GetRootLayer(),p)).resolve()) for p in original.GetRootLayer().subLayerPaths]
    expected={'L_T01_sloped_receiver','L_T01_post_bolt_32','L_T01_post_nut_32',
              'L_T01_post_bolt_130','L_T01_post_nut_130'}
    parts={p['name']:p for p in json.loads((REPO/'gripper-design/inaho_gripper_v5/parts_manifest.json').read_text())}
    removed=[]
    for prim in list(Usd.PrimRange(s.GetPrimAtPath('/World/ActuatedTool'))):
        name=prim.GetCustomDataByKey('sourceName')
        if name in expected:
            path=str(prim.GetPath());s.OverridePrim(path).SetActive(False)
            density=1270. if parts[name]['material']=='PETG' else 7850.
            removed.append(dict(name=name,path=path,mass_kg=parts[name]['volume_mm3']*1e-9*density))
    if {p['name'] for p in removed}!=expected or len(removed)!=5:
        raise ValueError('Receiver part inventory does not exactly match the five requested parts')
    removed_mass=sum(p['mass_kg'] for p in removed)
    previous_mass=report['tool']['mass_estimates_kg']['L']
    report['tool']['mass_estimates_kg']['L']=previous_mass-removed_mass
    UsdPhysics.MassAPI(s.GetPrimAtPath('/World/ActuatedTool/L')).CreateMassAttr(previous_mass-removed_mass)
    # The jaw carriage slides on the fixed gripper frame, not directly on the
    # robot flange. Preserve its world joint frame/axis and drive; the existing
    # collisionEnabled=false now excludes only this actual adjacent mechanism.
    # This does not exclude the gripper from any non-adjacent robot arm link.
    tool_joint_revisions=[]
    cache=UsdGeom.XformCache()
    for side in ('L','R'):
        j=UsdPhysics.Joint(s.GetPrimAtPath('/World/ActuatedTool/Joints/jaw_'+side))
        previous=str(j.GetBody0Rel().GetTargets()[0]);new='/World/ActuatedTool/fixed'
        old_pose=np.asarray(cache.GetLocalToWorldTransform(s.GetPrimAtPath(previous))).T
        new_pose=np.asarray(cache.GetLocalToWorldTransform(s.GetPrimAtPath(new))).T
        qrot=j.GetLocalRot0Attr().Get();local=np.eye(4)
        local[:3,:3]=R.from_quat([*qrot.GetImaginary(),qrot.GetReal()]).as_matrix()
        local[:3,3]=j.GetLocalPos0Attr().Get()
        changed=np.linalg.inv(new_pose)@old_pose@local;rot=R.from_matrix(changed[:3,:3]).as_quat()
        j.CreateBody0Rel().SetTargets([new]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,changed[:3,3])))
        j.CreateLocalRot0Attr(Gf.Quatf(float(rot[3]),Gf.Vec3f(*map(float,rot[:3]))))
        tool_joint_revisions.append(dict(joint=str(j.GetPath()),old_body0=previous,new_body0=new,
            world_frame_max_abs_error=float(np.max(np.abs(new_pose@changed-old_pose@local)))))
    kin=ToolKinematics(s);center=np.array(report['target']['center'])
    direction=center-np.r_[kin.world[:2,3],center[2]]
    direction=R.from_euler('z',yaw,degrees=True).apply(direction)
    pose=grasp_pose(center,direction);pose[:3,3]-=pose[:3,1]*distance
    seeds=[np.array([.05,b,u,e,w,v,0.]) for b in (-np.pi/2,np.pi/2)
           for u,e,w in [(-1.1,1.8,-.7),(.9,-1.7,.8)] for v in (-np.pi/2,np.pi/2)]
    seeds += [np.array([lift,b,u,e,w,v,0.]) for lift in (.3,.6,.9) for b in (-np.pi/2,np.pi/2)
              for u,e,w in [(-1.1,1.8,-.7),(.9,-1.7,.8)] for v in (-np.pi/2,np.pi/2)]
    if not -1<=seed_index<len(seeds):raise ValueError('Ready seed index must be -1 (search) or 0..31')
    attr(s.GetPrimAtPath(ROBOT+'/root_joint'),'physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool,True)
    checker=MotionSelfCollision(s,kin,clearance_m=.002);attempts=[];selected=None
    for index in (range(len(seeds)) if seed_index==-1 else [seed_index]):
        q,pe,re=kin.ik(pose,seeds[index]);attempt=dict(seed_index=index,position_error_m=pe,orientation_error_rad=re)
        attempts.append(attempt)
        if pe>.002 or re>.02 or not np.isfinite(q).all():attempt['status']='ik_failed';continue
        hits=checker.check(q,.044) or checker.check(q,.096)
        if hits:attempt.update(status='ready_self_collision',hits=hits);continue
        try:
            phases,plan=plan_motion(kin,center,start=q,preparation='horizontal_ready',yaw_offset_deg=yaw,entry_only=True)
        except RuntimeError as exc:attempt.update(status='path_ik_failed',error=str(exc));continue
        audit=check_plan(checker,plan,hz=120)
        attempt.update(status='clear' if audit['passed'] else 'path_self_collision',audit=audit)
        if audit['passed'] and audit['complete']:selected=index;break
    (output/'ready_self_collision_search.json').write_text(json.dumps(attempts,indent=2)+'\n')
    if selected is None:raise RuntimeError('No self-collision-free horizontal ready branch found; see ready_self_collision_search.json')
    seed_index=selected
    for name,value in zip(kin.names,q):
        p=s.GetPrimAtPath(ROBOT+'/joints/'+name)
        slider=p.IsA(UsdPhysics.PrismaticJoint);kind='linear' if slider else 'angular'
        units=float(value if slider else np.rad2deg(value))
        p.AddAppliedSchema('PhysicsJointStateAPI:'+kind)
        attr(p,'state:'+kind+':physics:position',Sdf.ValueTypeNames.Float,units)
        attr(p,'state:'+kind+':physics:velocity',Sdf.ValueTypeNames.Float,0.)
        UsdPhysics.DriveAPI(p,kind).CreateTargetPositionAttr(units)
    shutil.copy2(assembly/'skin.npz',output/'skin.npz')
    report['initial_robot_q']=q.tolist()
    report['robot_self_collision_enabled']=True
    report['tool_joint_parent_revision']=tool_joint_revisions
    report['ready_revision']=dict(source_assembly=str(assembly.resolve()),source_scene_sha256=sha(source),
        removed_parts=removed,removed_mass_kg=removed_mass,ready_pose=pose.tolist(),
        ready_distance_m=distance,yaw_offset_deg=yaw,seed_index=seed_index,
        ik_position_error_m=pe,ik_orientation_error_rad=re,
        initialization='JointStateAPI before native startup; no runtime teleport',
        reference_video='https://www.youtube.com/watch?v=-V8sVICfdxg&t=49',reference_seconds=[49,69])
    s.GetRootLayer().Save();report['scene_sha256']=sha(output/'scene.usda')
    (output/'motion_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (output/'ready_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    print(json.dumps(report['ready_revision'],indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plant',type=int,default=22)
    p.add_argument('--horizontal-ready-revision',action='store_true',help='Input is an existing assembly directory; remove lower receiver and author horizontal ready pose')
    p.add_argument('--ready-distance',type=float,default=.30)
    p.add_argument('--ready-yaw',type=float,default=30.)
    p.add_argument('--ready-seed',type=int,default=-1,help='-1 searches IK branches with self-collision screening; 0..31 selects one checked seed')
    p.add_argument('--target',help='Existing active fruit name, for the ready revision only')
    a=p.parse_args()
    if a.target and not a.horizontal_ready_revision:p.error('--target requires --horizontal-ready-revision')
    if a.horizontal_ready_revision:revise_for_horizontal_ready(a.scene,a.output,a.ready_distance,a.ready_yaw,a.ready_seed,a.target)
    else:build(a.scene,a.output,a.plant)
