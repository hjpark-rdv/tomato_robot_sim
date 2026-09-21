"""Read-only inspection of authored native collision shapes and plant joints."""
import json
import numpy as np
from scipy.spatial.transform import Rotation


def point_segment(point,a,b):
    delta=b-a
    t=np.clip(np.sum((point-a)*delta,axis=-1)/np.maximum(np.sum(delta*delta,axis=-1),1e-30),0,1)
    return np.linalg.norm(point-a-t[...,None]*delta,axis=-1)


def segment_distance(a,b,c,d):
    """Exact segment distance, including parallel and zero-length sphere axes."""
    u=b-a;v=d-c;w=a-c
    aa=np.sum(u*u,axis=-1);bb=np.sum(u*v,axis=-1);cc=np.sum(v*v,axis=-1)
    dd=np.sum(u*w,axis=-1);ee=np.sum(v*w,axis=-1);denom=aa*cc-bb*bb
    s=(bb*ee-cc*dd)/np.maximum(denom,1e-30);t=(aa*ee-bb*dd)/np.maximum(denom,1e-30)
    interior=np.linalg.norm(w+s[...,None]*u-t[...,None]*v,axis=-1)
    interior=np.where((denom>1e-24)&(s>=0)&(s<=1)&(t>=0)&(t<=1),interior,np.inf)
    return np.minimum.reduce([point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b),interior])


def world_shapes(poses,shapes):
    indices=np.asarray([s['body'] for s in shapes]);local=np.asarray([s['local_endpoints'] for s in shapes])
    selected=poses[:,indices];rot=Rotation.from_quat(selected[:,:,[4,5,6,3]].reshape(-1,4)).as_matrix().reshape(*selected.shape[:2],3,3)
    return np.einsum('tnij,nkj->tnki',rot,local)+selected[:,:,:3,None].transpose(0,1,3,2)


def analyze(directory,recordings):
    from pathlib import Path
    import csv
    directory=Path(directory);recordings=Path(recordings)
    live=json.loads((directory/'live_connections.json').read_text())
    visual_manifest=json.loads((recordings/'scene.json').read_text())
    assert live['body_paths']==visual_manifest['body_paths']
    shapes=live['shapes'];lookup={s['path']:i for i,s in enumerate(shapes)}
    by_name={s['path'].split('/')[-2]:i for i,s in enumerate(shapes) if s['path'].endswith('/StemCollider')}
    radius=np.array([s['radius_m'] for s in shapes]);rest=np.asarray(live['body_rest'])
    world=world_shapes(rest[None],shapes)
    def pair_gap(points,aa,bb):
        a=points[:,aa];b=points[:,bb]
        return segment_distance(a[:,:,None,0],a[:,:,None,1],b[:,None,:,0],b[:,None,:,1])-radius[aa][None,:,None]-radius[bb][None,None,:]
    rachis=[by_name[f'TRUSS_Rachis_{i:02d}'] for i in range(14)]
    connections=[]
    for i in range(1,12):
        name=f'Tomato_{i:02d}';chain=[by_name[f'TRUSS_Pedicel_proximal_{i:02d}_{j:02d}'] for j in range(3)]
        fruit=f'/World/envs/env_0/HarvestableStem/Harvestables/{name}'
        distal=lookup[fruit+'/PedicelCollider'];sphere=lookup[fruit+'/FruitCollider']
        connections.append(dict(target=name,root_shapes=[chain[0]],parent_shapes=rachis,
            terminal_shapes=[chain[-1]],distal_shapes=[distal],fruit_shapes=[distal,sphere],
            root_gap_initial_mm=float(pair_gap(world,[chain[0]],rachis).min()*1000),
            terminal_to_distal_gap_initial_mm=float(pair_gap(world,[chain[-1]],[distal]).min()*1000),
            terminal_to_fruit_any_gap_initial_mm=float(pair_gap(world,[chain[-1]],[distal,sphere]).min()*1000)))
    results=[];ring=[i for i,s in enumerate(shapes) if '/RingCollision/' in s['path']]
    for capture in sorted((recordings/'captures').glob('*/motion.npz')):
        data=np.load(capture);poses=data['poses'];times=data['times_s'];points=world_shapes(poses,shapes)
        assert np.max(abs(poses[0]-rest))<1e-6
        for entry in connections:
            root_gap=pair_gap(points,entry['root_shapes'],entry['parent_shapes']).min(axis=(1,2))
            end_gap=pair_gap(points,entry['terminal_shapes'],entry['distal_shapes']).min(axis=(1,2))
            any_gap=pair_gap(points,entry['terminal_shapes'],entry['fruit_shapes']).min(axis=(1,2))
            results.append(dict(candidate=capture.parent.name,target=entry['target'],
                root_gap_max_mm=float(root_gap.max()*1000),root_gap_max_time_s=float(times[root_gap.argmax()]),
                terminal_gap_max_mm=float(end_gap.max()*1000),terminal_gap_max_time_s=float(times[end_gap.argmax()]),
                terminal_any_fruit_gap_max_mm=float(any_gap.max()*1000)))
        if capture.parent.name=='coarse_0027':
            chosen=connections[4];target_shapes=[by_name[f'TRUSS_Pedicel_proximal_05_{j:02d}'] for j in range(3)]+chosen['distal_shapes']
            gap=pair_gap(points,ring,target_shapes);min_gap=gap.min(axis=(1,2));ids=np.unravel_index(gap.argmin(),gap.shape)
            case27=dict(min_ring_target_pedicel_gap_mm=float(gap[ids]*1000),at_time_s=float(times[ids[0]]),
                ring_shape=shapes[ring[ids[1]]]['path'],pedicel_shape=shapes[target_shapes[ids[2]]]['path'])
            np.savez_compressed(directory/'case27_collision_geometry.npz',times_s=times,endpoints=points,radius=radius,min_ring_target_gap_m=min_gap)
    for entry in connections:
        selected=[r for r in results if r['target']==entry['target']]
        for key in ('root_gap_max_mm','terminal_gap_max_mm','terminal_any_fruit_gap_max_mm'):
            worst=max(selected,key=lambda r:r[key]);entry[key]=worst[key];entry[key+'_candidate']=worst['candidate']
    summary=dict(connections=connections,case27=case27,shape_count=len(shapes),
        all_shapes_enabled=all(s['enabled'] is not False for s in shapes),
        scope='All 11 fruit connections in rest state and 15 Hz recorded physics states of all 31 Tomato_05 candidates; not 11 separate target approach trials',
        gap_definition='capsule/sphere surface separation in mm: positive=empty space, negative=overlap; contact-offset inflation excluded',
        recording_sample_rate_hz=15,ring_wire_diameter_mm=2.)
    (directory/'connection_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (directory/'connection_gaps.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(results[0]));writer.writeheader();writer.writerows(results)
    print(json.dumps(summary,indent=2))



def export_connections(env, directory, reset=True):
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
    from omni.physx import get_physx_interface
    from pose_video_capture import body_poses
    if reset: env.reset()
    env.elastic.sync_visuals()
    env.sim.physics_sim_view.update_articulations_kinematic()
    get_physx_interface().update_transformations(False,True,True)
    paths=list(env.robot.root_physx_view.link_paths[0])+[s['path'] for s in env.fruit_specs]+env.elastic.paths+[s['anchor'] for s in env.fruit_specs]
    poses=body_poses(env); rotations=Rotation.from_quat(poses[:,[4,5,6,3]]).as_matrix()
    cache=UsdGeom.XformCache(); shapes=[]; filters=[];joints=[]
    # This export describes one clone. Traversing the entire stage here made
    # the new per-run audit setup needlessly scan every replicated robot.
    for prim in Usd.PrimRange(env.stage.GetPrimAtPath(env.root),Usd.TraverseInstanceProxies()):
        path=str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            filters.append(dict(path=path,targets=[str(p) for p in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()]))
        if prim.IsA(UsdPhysics.Joint):
            joint=UsdPhysics.Joint(prim);a=joint.GetBody0Rel().GetTargets();b=joint.GetBody1Rel().GetTargets()
            if a and b and str(a[0]) in paths and str(b[0]) in paths:
                joints.append(dict(path=path,body0=paths.index(str(a[0])),body1=paths.index(str(b[0])),
                    p0=list(joint.GetLocalPos0Attr().Get()),p1=list(joint.GetLocalPos1Attr().Get()),
                    collision_enabled=joint.GetCollisionEnabledAttr().Get()))
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        if not prim.IsA(UsdGeom.Capsule) and not prim.IsA(UsdGeom.Sphere):continue
        ancestor=prim
        while ancestor and str(ancestor.GetPath()) not in paths:ancestor=ancestor.GetParent()
        if not ancestor:continue
        body=paths.index(str(ancestor.GetPath()));tf=np.asarray(cache.GetLocalToWorldTransform(prim))
        scale=np.linalg.norm(tf[:3,:3],axis=1)
        if not np.allclose(scale,scale[0],atol=1e-6):raise ValueError('Nonuniform capsule or sphere scale: '+path)
        if prim.IsA(UsdGeom.Capsule):
            shape=UsdGeom.Capsule(prim);axis='XYZ'.index(str(shape.GetAxisAttr().Get()));h=float(shape.GetHeightAttr().Get())
            ends=np.zeros((2,3));ends[:,axis]=[-h/2,h/2];kind='capsule'
        else:shape=UsdGeom.Sphere(prim);ends=np.zeros((2,3));kind='sphere'
        world=ends@tf[:3,:3]+tf[3,:3];local=(world-poses[body,:3])@rotations[body]
        api=PhysxSchema.PhysxCollisionAPI(prim)
        shapes.append(dict(path=path,body=body,kind=kind,local_endpoints=local.tolist(),
            radius_m=float(shape.GetRadiusAttr().Get())*float(scale[0]),
            enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),
            contact_offset_attr=api.GetContactOffsetAttr().Get(),rest_offset_attr=api.GetRestOffsetAttr().Get()))
    visual=[]
    for name,sides in [('STEM_MainStem',24),('TRUSS_Truss_01_Peduncle',14),('TRUSS_Rachis',14),*[(f'TRUSS_Pedicel_proximal_{i:02}',12) for i in range(1,12)]]:
        from elastic_geometry import tube_centerline
        mesh=env.elastic.static.GetChild(name);points,_=env.elastic._world_points(mesh)
        centers,radii=tube_centerline(points,sides)
        visual.append(dict(name=name,centers=centers.tolist(),radii=radii.tolist()))
    report=dict(body_paths=paths,body_rest=poses.tolist(),shapes=shapes,joints=joints,filtered_pairs=filters,
        chains={k:[env.elastic.paths[i] for i in v] for k,v in env.elastic.chains.items()},visual_centerlines=visual,
        model=env.elastic.model,physics_dt=env.physics_dt,
        scope='Live USD shapes in the unchanged elastic scene; no model edits or motion')
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'live_connections.json').write_text(json.dumps(report,indent=2)+'\n')
    print('[CONNECTION AUDIT]',len(shapes),'shapes',len(joints),'joints',flush=True)


def analyze_missing_rails(directory, recordings):
    """Screen motion against CAD rails omitted by assets.build_robot.

    These capsules are inspection probes only; no USD or physics is changed.
    Dimensions match assy_gripper_ver_6.stl and the x > -78 mm hull cutoff.
    """
    from pathlib import Path
    out, video = Path(directory), Path(recordings)
    live = json.loads((out / 'live_connections.json').read_text())
    manifest = json.loads((video / 'scene.json').read_text())
    assert live['body_paths'] == manifest['body_paths']
    shapes = live['shapes']
    arc = {shape['path'].split('/')[-1]: shape for shape in shapes
           if '/RingCollision/' in shape['path']}
    p0 = np.array(arc['segment_00']['local_endpoints'])[0]
    p1 = np.array(arc['segment_31']['local_endpoints'])[1]
    center = (p0 + p1) / 2
    back = np.array(arc['segment_15']['local_endpoints'])[1]
    axis = (center - back) / np.linalg.norm(center - back)
    rails = [dict(path=f'missing_rail_{i}', body=arc['segment_00']['body'],
                  local_endpoints=[p.tolist(), (p + axis * (.10640287 - .078)).tolist()],
                  radius_m=.001) for i, p in enumerate([p0, p1])]
    all_shapes = shapes + rails
    radii = np.array([shape['radius_m'] for shape in all_shapes])
    target = [i for i, shape in enumerate(shapes)
              if 'TRUSS_Pedicel_proximal_05_' in shape['path']
              or '/Tomato_05/PedicelCollider' in shape['path']]
    plant = [i for i, shape in enumerate(shapes) if '/Robot/' not in shape['path']]
    findings, target_intervals = [], []
    for path in sorted((video / 'captures').glob('*/motion.npz')):
        data = np.load(path)
        assert np.max(abs(data['poses'][0] - np.asarray(live['body_rest']))) < 1e-6
        points = world_shapes(data['poses'], all_shapes)
        a, b = points[:, -2:], points[:, plant]
        gaps = (segment_distance(a[:, :, None, 0], a[:, :, None, 1],
                                 b[:, None, :, 0], b[:, None, :, 1])
                - radii[-2:][None, :, None] - radii[plant][None, None, :])
        hit = np.unravel_index(gaps.argmin(), gaps.shape)
        findings.append(dict(candidate=path.parent.name,
                             min_gap_mm=float(gaps[hit] * 1000),
                             time_s=float(data['times_s'][hit[0]]),
                             plant_shape=shapes[plant[hit[2]]]['path'],
                             rail=int(hit[1])))
        if path.parent.name == 'coarse_0027':
            for index in target:
                gap = gaps[:, :, plant.index(index)]
                ticks = np.flatnonzero(gap.min(1) < -.0001)
                target_intervals.append(dict(
                    shape=shapes[index]['path'], min_gap_mm=float(gap.min() * 1000),
                    first_last_overlap_time_s=[float(data['times_s'][ticks[0]]),
                                               float(data['times_s'][ticks[-1]])]
                    if len(ticks) else None))
            np.savez_compressed(out / 'case27_missing_rails.npz', endpoints=points,
                                times=data['times_s'], gaps=gaps)
    report = dict(rails=rails, findings=findings, plant_shapes=plant,
                  case27_target_intervals=target_intervals,
                  threshold_mm=-.1, probe_only=True,
                  scope='31 Tomato_05 trajectories, all plant capsule/sphere shapes, 15 Hz')
    (out / 'missing_rails.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Missing rail intersections:', [(r['candidate'], round(r['min_gap_mm'], 3))
                                          for r in findings if r['min_gap_mm'] < -.1])


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('recordings')
    args = parser.parse_args()
    analyze(args.directory, args.recordings)
    analyze_missing_rails(args.directory, args.recordings)
