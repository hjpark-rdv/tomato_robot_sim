"""Current S300 USD -> MuJoCo adapter (run with the existing asset Python).

Default remains the prescribed-tool smoke; opt-in robot/background ports reuse
the authored joint topology, full visual triangles and finite drive caps.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Usd, UsdGeom, UsdPhysics

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from convert_model import nums, quat


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def export(assembly, trial, output, truss='Truss_00', *, robot=False,
           background=False, excluded_background_roots=(), plant_only=False,
           ground_collision='source-box'):
    start = time.perf_counter()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'meshes').mkdir()
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    source = assembly / 'composed.usdc'
    assert sha(source) == manifest['compact_assets']['materialized_dependency']['sha256']
    assert sha(assembly / 'scene.usda') == manifest['scene_sha256']
    stage = Usd.Stage.Open(str(source))
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.
    cache = UsdGeom.XformCache()
    def matrix(path):
        return np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
    def qattr(j, i):
        q = getattr(j, f'GetLocalRot{i}Attr')().Get()
        return [*q.GetImaginary(), q.GetReal()]
    def position(j, i):
        return np.asarray(getattr(j, f'GetLocalPos{i}Attr')().Get(), float)

    rods = [b for b in manifest['plant']['bodies']
            if (b['name'].startswith(('Stem_MainStem_', truss+'_')) or
                (robot or plant_only) and b['name'].startswith('Leaf_')) and 'Fruit_' not in b['name']]
    fruits = [f for f in manifest['plant']['fruits'] if f['name'].startswith(truss+'_')]
    paths = {b['path']: b for b in rods + fruits}
    tools = [] if robot or plant_only else list(manifest['tool']['bodies'].values()) + manifest['tool']['belts']
    names = {p: b['name'] for p, b in paths.items()}
    names.update({p: 'Tool_' + p.rsplit('/', 1)[1] for p in tools})
    transforms = {p: matrix(p) for p in names}
    joint_by_child = {}
    for p in Usd.PrimRange(stage.GetPrimAtPath(manifest['plant']['root'])):
        if p.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(p)
            for child in j.GetBody1Rel().GetTargets():
                joint_by_child[str(child)] = j

    root = ET.Element('mujoco', model='s300_dynamic_robot_harvest' if robot else 's300_one_truss_prescribed_tool')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false', meshdir='meshes')
    opt = ET.SubElement(root, 'option', timestep=str(1/240), integrator='discrete',
                        solver='Newton', iterations='64', tolerance='1e-8',
                        jacobian='sparse', cone='elliptic')
    ET.SubElement(opt, 'flag', filterparent='disable', autoreset='disable')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.SubElement(visual, 'map', znear='.001', zfar='20')
    default = ET.SubElement(root, 'default')
    # Source PhysX restOffset=0, contactOffset=.5mm: force at the real surface,
    # with a separate detection buffer. Modern MuJoCo gap adds detection range;
    # it does not subtract margin. Setting both to .5mm inflates active contact.
    ET.SubElement(default, 'geom', friction='.5 0 0', condim='3', margin='0', gap='.0005',
                  solref='.002 1', solimp='.99 .999 .001', density='0')
    assets = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    contact = ET.SubElement(root, 'contact')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(world, 'light', pos='0 3 4', dir='0 0 -1', diffuse='.8 .8 .8', ambient='.4 .4 .4')
    ET.SubElement(world, 'light', pos='2 1 2', dir='-1 1 -1', diffuse='.6 .6 .6')
    # A declared catch floor; chassis/farm are intentionally outside this test.
    ET.SubElement(world, 'geom', name='CatchFloor', type='plane', pos='0 0 0',
                  size='5 5 .1', rgba='.8 .82 .8 1', contype='1', conaffinity='3')
    nodes = {}
    for p in tools:
        t = transforms[p]
        nodes[p] = ET.SubElement(world, 'body', name=names[p], mocap='true',
                                 pos=nums(t[:3, 3]), quat=nums(quat(R.from_matrix(t[:3, :3]))))

    def body(p):
        if p in nodes:
            return nodes[p]
        b, t = paths[p], transforms[p]
        fruit = b in fruits
        parent = None if fruit else b.get('parent')
        if parent and parent not in paths:
            raise ValueError('Missing rod ancestor ' + parent)
        node = body(parent) if parent else world
        local = np.linalg.inv(transforms[parent]) @ t if parent else t
        node = ET.SubElement(node, 'body', name=names[p], pos=nums(local[:3, 3]),
                             quat=nums(quat(R.from_matrix(local[:3, :3]))))
        nodes[p] = node
        if fruit:
            ET.SubElement(node, 'freejoint', name=names[p] + '_free')
        elif parent:
            j = joint_by_child[p]
            for i in range(3):
                ET.SubElement(node, 'joint', name=names[p] + '_' + str(i), type='hinge',
                              pos=nums(position(j, 1)), axis=nums(R.from_quat(qattr(j, 1)).apply(np.eye(3)[i])),
                              stiffness=str(b['stiffness']), damping=str(b['damping']),
                              limited='true', range=nums(np.deg2rad([-45, 45])), armature='0')
        prim = stage.GetPrimAtPath(p)
        mass = float(prim.GetAttribute('physics:mass').Get())
        inertia = prim.GetAttribute('physics:diagonalInertia').Get()
        if inertia and min(inertia) > 0:
            com = prim.GetAttribute('physics:centerOfMass').Get()
            q = prim.GetAttribute('physics:principalAxes').Get()
            axes=np.array([q.GetReal(),*q.GetImaginary()])
            if np.linalg.norm(axes)<1e-12:axes=np.array([1.,0.,0.,0.])
            ET.SubElement(node, 'inertial', pos=nums(com), mass=str(mass), diaginertia=nums(inertia),
                          quat=nums(axes))
        elif fruit:
            # Explicit ellipsoid estimate; PhysX auto-computed hull inertia was not saved.
            a, bb, c = np.array(b['dimensions_m']) / 2
            diag = mass / 5 * np.array([bb*bb+c*c, a*a+c*c, a*a+bb*bb])
            ET.SubElement(node, 'inertial', pos='0 0 0', mass=str(mass), diaginertia=nums(diag))
        else:
            raise ValueError('Missing inertia ' + p)
        return node
    for p in paths:
        body(p)

    inventory = []
    for owner in names:
        inv = np.linalg.inv(transforms[owner])
        for prim in Usd.PrimRange(stage.GetPrimAtPath(owner)):
            ismesh = prim.IsA(UsdGeom.Mesh)
            iscap = prim.IsA(UsdGeom.Capsule)
            if not (ismesh or iscap):
                continue
            path = str(prim.GetPath())
            collides = bool(prim.HasAPI(UsdPhysics.CollisionAPI) and
                            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
            # Explicit selected policy: foliage has no collision, stem capsules do.
            if '/Appendage_' in path:
                continue
            if names[owner].startswith('Leaf_'):collides=False
            visible = UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible'
            if not collides and not visible:
                continue
            local = inv @ matrix(path)
            name = f'g{len(inventory):04d}'
            color = [0.18, 0.42, 0.05]
            ga = dict(name=name, contype='2' if owner in tools else '1',
                      conaffinity='1' if owner in tools else '3', group='0')
            if not collides:
                ga.update(contype='0', conaffinity='0', group='2')
            if iscap:
                cap = UsdGeom.Capsule(prim)
                axis = np.eye(3)['XYZ'.index(cap.GetAxisAttr().Get())]
                ends = np.array([-axis, axis])*float(cap.GetHeightAttr().Get())/2
                ends = ends @ local[:3, :3].T + local[:3, 3]
                ga.update(type='capsule', fromto=nums(ends), size=str(cap.GetRadiusAttr().Get()))
                approximation = 'source_capsule'
            else:
                mesh = UsdGeom.Mesh(prim)
                if not np.all(np.asarray(mesh.GetFaceVertexCountsAttr().Get()) == 3):
                    raise ValueError('Non-triangle source ' + path)
                v = np.asarray(mesh.GetPointsAttr().Get(), float) @ local[:3, :3].T + local[:3, 3]
                f = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
                color_attr = mesh.GetDisplayColorAttr().Get()
                if color_attr:
                    color = list(color_attr[0])
                obj = output / 'meshes' / (name + '.obj')
                with obj.open('w') as stream:
                    for vertex in v: stream.write('v ' + nums(vertex) + '\n')
                    for face in f: stream.write('f ' + ' '.join(str(int(i)+1) for i in face) + '\n')
                ET.SubElement(assets, 'mesh', name=name, file=obj.name)
                ga.update(type='mesh', mesh=name)
                approximation = str(prim.GetAttribute('physics:approximation').Get()) if collides else 'visual_triangles'
            ga['rgba'] = nums([*color, 1 if visible or iscap else 0])
            ET.SubElement(nodes[owner], 'geom', **ga)
            inventory.append(dict(name=name, source=path, owner=names[owner], collision=collides,
                                  source_approximation=approximation,
                                  mujoco_collision='convex_hull_per_source_mesh' if ismesh and collides else approximation))
    exclusions = set()
    for a, b in manifest['plant']['exclusions']:
        if a in paths and b in paths:
            exclusions.add(tuple(sorted((names[a], names[b]))))
    for a, b in sorted(exclusions):
        ET.SubElement(contact, 'exclude', body1=a, body2=b)
    attachments = []
    for f in fruits:
        p, anchor = f['path'], f['anchor']
        j = joint_by_child[p]
        for i, owner in enumerate([anchor, p]):
            ET.SubElement(nodes[owner], 'site', name=f['name'] + '_a' + str(i),
                          pos=nums(position(j, i)), size='.001', rgba='1 .8 0 0')
        ET.SubElement(equality, 'connect', name=f['name'] + '_attachment',
                      site1=f['name']+'_a0', site2=f['name']+'_a1', solref='.002 1', solimp='.999 .9999 .001')
        gap = np.linalg.norm((transforms[anchor] @ np.r_[position(j, 0), 1])[:3] -
                             (transforms[p] @ np.r_[position(j, 1), 1])[:3])
        if gap > 1e-6: raise ValueError(f'Attachment transform gap {gap}: {p}')
        attachments.append(dict(name=f['name'], anchor=names[anchor], local_rot0=qattr(j,0),
                                local_rot1=qattr(j,1), initial_gap_m=float(gap),
                                force_n=f['break_force_n'], torque_nm=f['break_torque_nm'],
                                stiffness=[j.GetPrim().GetAttribute('physxLimit:rot'+a+':stiffness').Get() for a in 'XYZ'],
                                damping=j.GetPrim().GetAttribute('physxLimit:rotX:damping').Get(), deadband_deg=1.))
    robot_report=None;farm_report=None
    if robot:
        from robot_adapter import add_robot
        robot_port=add_robot(stage,manifest,root=root,world=world,assets=assets,
                               contact=contact,meshdir=output/'meshes',matrix=matrix)
        inventory.extend(robot_port['geometry'])
        robot_report=robot_port['metadata']
    if background:
        world.remove(world.find("geom[@name='CatchFloor']"))
        from farm_adapter import add_background
        farm_report=add_background(stage,manifest,world=world,assets=assets,output=output,
                                    excluded_roots=excluded_background_roots,ground_collision=ground_collision)
    center = np.mean([transforms[f['path']][:3, 3] for f in fruits], axis=0)
    for name, offset in [('top', [0,-.12,.75]), ('side', [.65,-.5,.28])]:
        eye = center + offset
        z = eye - center; z /= np.linalg.norm(z)
        x = np.cross([0,0,1], z); x /= np.linalg.norm(x)
        ET.SubElement(world, 'camera', name=name, pos=nums(eye), xyaxes=nums([x,np.cross(z,x)]), fovy='48')
    ET.indent(root)
    ET.ElementTree(root).write(output/'scene.xml', encoding='unicode')
    states = np.load(trial/'states.npz')
    ids = [states['body_paths'].tolist().index(p) for p in tools]
    np.savez_compressed(output/'tool_trace.npz', time=states['time'], poses=states['poses'][:,ids],
                        phase=states['phase'], belt_speed=states['belt_surface_velocity_local_y_m_s'])
    report = dict(schema='s300-mujoco-small-smoke-v1', source_assembly=str(assembly), source_trial=str(trial),
                  source_usdc_sha256=sha(source), source_states_sha256=sha(trial/'states.npz'),
                  model_sha256=sha(output/'scene.xml'), trace_sha256=sha(output/'tool_trace.npz'),
                  tools=[names[p] for p in tools], rods=[b['name'] for b in rods],
                  fruits=[f['name'] for f in fruits], attachments=attachments, geometry=inventory,
                  export_wall_s=time.perf_counter()-start, training_eligible=False, harvest_success=None,
                  contact_offset_mapping=dict(source_rest_offset_m=0.,source_contact_offset_m=.0005,
                      native_margin_m=0.,native_gap_m=.0005,
                      interpretation='Force at surface; detection buffer per geom. Earlier .5mm margin was an export mapping error.'),
                  differences=[
                      'Prescribed recorded tool body poses; robot/lift/rail/chassis/farm absent. Not full-system speed comparison.',
                      'One selected truss and its 16-segment main stem; other truss mass/dynamics omitted. Leaves visual-only.',
                      'D6 rods use existing MuJoCo three-hinge convention with source stiffness/damping; no DOF merging.',
                      'Tool per-mesh convex hull replaces PhysX convex decomposition. Concave-part collision equivalence NOT established.',
                      'Fruit inertia is estimated from source dimensions/mass; native cooked inertia not exported.',
                      'Contact friction .5, MuJoCo solver-specific solref/solimp. Neither contact equivalence nor tissue calibration claimed.',
                      'Fruit connect constraint + passive angular spring + measured force/torque threshold is a MuJoCo adapter, not native PhysX breaking.',
                      'Native surfacevel supplies belt and prescribed tool tangential velocity; mocap normal-velocity and finite robot inertia are not represented.',
                      'Discrete integrator accounts for damping in constraint effective inertia. No gravity preload: this selected plant settles under gravity.',
                      'Source replay ends at its original 36.8s stop; no unrecorded withdrawal is invented.'])
    report['fruit_specs']=fruits
    report['source_manifest']=str(assembly/'motion_manifest.json')
    report['source_manifest_sha256']=sha(assembly/'motion_manifest.json')
    if robot:
        report['robot']=robot_report
        report['tools']=robot_report['tool_names']
        report['schema']='s300-mujoco-dynamic-robot-v1'
        report['differences']=report['differences'][1:]
        report['differences']=[s for s in report['differences'] if not s.startswith(('Native surfacevel supplies','Source replay ends'))]
        report['differences']=[s for s in report['differences'] if not s.startswith(('One selected truss','Discrete integrator'))]
        report['differences'].append('One selected truss, 16-segment stem and 27 leaf bodies per exported target plant; no merging. Leaves retain dynamics/visuals but do not collide.')
        report['differences'].append('Native discrete integrator and fixed initial plant gravity preload; bias feedforward through finite robot actuators. Engine contact equivalence remains unverified.')
        report['differences'].append('Rail/lift/arm/jaws use finite source PD actuators; no physical pose playback.')
        plan=json.loads((trial/'plan.json').read_text())
        (output/'source_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        report['source_plan_sha256']=sha(output/'source_plan.json')
    if background:report['farm']=farm_report
    (output/'export.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['export_wall_s','model_sha256','fruits','differences']},indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('assembly', type=Path); p.add_argument('trial', type=Path); p.add_argument('output', type=Path)
    p.add_argument('--robot',action='store_true');p.add_argument('--background',action='store_true')
    p.add_argument('--plant-only',action='store_true')
    p.add_argument('--exclude-background-root',action='append',default=[])
    p.add_argument('--ground-collision',choices=['source-box','top-plane'],default='source-box')
    a = p.parse_args(); export(a.assembly, a.trial, a.output,robot=a.robot,background=a.background,
                              excluded_background_roots=a.exclude_background_root,plant_only=a.plant_only,
                              ground_collision=a.ground_collision)
