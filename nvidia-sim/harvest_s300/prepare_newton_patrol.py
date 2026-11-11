"""Prepare bounded entrance patrol assets from source USD, never old trials.

Asset Python only. Existing converters/robot/plant/controller are reused.
All trusses of selected near-side plants are retained in a canonical state bank.
The requested row extent is an explicit test budget, not an eligibility claim.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).parent
sys.path.insert(0, str(ROOT/'mujoco-benchmark/scripts/s300_smoke'))


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def idle_plan(manifest):
    from plan_adapter import DOF_NAMES
    q=manifest['initial_robot_q']
    return dict(schema='s300-camera-patrol-hold-v1', dof_names=DOF_NAMES,
        initial_command=[0.,*q,0.,0.], initial_gap_m=.044, settle_seconds=0.,
        phases=[dict(name='ready_hold',duration_s=1.,waypoint_q=[q,q],end_gap_m=.044,
                     rail_waypoint_positions_m=[0.,0.],belt_local_y_m_s=[0.,0.])],
        total_duration_s=1.,training_eligible=False,harvest_success=None)


def relocate_camera_geometry(tree,geometry,body_names,previous,mounted):
    """Move only the existing non-colliding housing/bracket in the visual XML."""
    from scipy.spatial.transform import Rotation
    old_body=body_names[previous['body_path']];new_body=body_names[mounted['body_path']]
    bodies={b.get('name'):b for b in tree.iter('body')}
    delta=np.asarray(mounted['body_from_camera_optical'])@np.linalg.inv(previous['body_from_camera_optical'])
    selected=[g for g in geometry if g['source'].startswith(previous['root_path']+'/')]
    if len(selected)!=2 or any(g['collision'] for g in selected):
        raise ValueError('Exactly the original visual housing and bracket required')
    for row in selected:
        geom=bodies[old_body].find("geom[@name='%s']"%row['name'])
        if geom is None or geom.get('contype')!='0' or geom.get('conaffinity')!='0':
            raise ValueError('Camera relocation must not move a collider')
        pose=np.eye(4);pose[:3,3]=np.fromstring(geom.get('pos','0 0 0'),sep=' ')
        quat=np.fromstring(geom.get('quat','1 0 0 0'),sep=' ')
        pose[:3,:3]=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
        pose=delta@pose;q=Rotation.from_matrix(pose[:3,:3]).as_quat()[[3,0,1,2]]
        geom.set('pos',' '.join(map(str,pose[:3,3])));geom.set('quat',' '.join(map(str,q)))
        bodies[old_body].remove(geom);bodies[new_body].append(geom)
        row['owner']=new_body
        row['source']=mounted['root_path']+row['source'][len(previous['root_path']):]
    return [g['name'] for g in selected]


def relocate_camera_assets(source,output,*,ready_q=None):
    """Sensor-only revision: share verified physics, reuse farm and planner."""
    import copy
    import xml.etree.ElementTree as ET
    from pxr import Usd
    from motion_kinematics import ToolKinematics
    from camera_patrol_runner import ready_camera_service_calibration
    from mounted_rgbd import calibrated_mount
    from mounted_rgbd import sha
    source=Path(source).resolve(strict=True);output=Path(output).resolve()
    config=json.loads((source/'patrol_assets.json').read_text())
    assembly=Path(config['plants'][0]['assembly'])
    previous=config.get('mounted_rgbd') or json.loads((assembly/'mounted_rgbd.json').read_text())
    # Green box: crop-facing side of the distal forearm housing, immediately
    # below the wrist. Attach to link3 so its cover and sensor stay rigid.
    body='/World/HarvestRobot/Robot/link3';distal='/World/HarvestRobot/Robot/link4'
    kin=ToolKinematics(Usd.Stage.Open(str(assembly/'scene.usda')))
    reference=json.loads(Path(config['plants'][0]['camera_calibration']).read_text())
    if ready_q is not None:reference['ready_q']=np.asarray(ready_q,float).tolist()
    frames=kin.frames(np.asarray(reference['ready_q']))
    normal=np.array([-1.,0.,0.]);down=np.array([0.,0.,-1.]);right=np.cross(down,normal)
    local=np.eye(4);local[:3,:3]=frames[body][:3,:3].T@np.column_stack((right,down,normal))
    pivot=np.linalg.inv(frames[body])@frames[distal]
    local[:3,3]=pivot[:3,3]+.098*local[:3,2]-.0325*local[:3,1]
    local=calibrated_mount(local)
    mounted=dict(previous,body_path=body,root_path=body+'/ForearmRGBD',
        camera_path=body+'/ForearmRGBD/Camera',body_from_camera_optical=local.tolist(),
        mount_description='Crop-facing distal forearm cover at wrist joint, user green box; portrait housing, horizontal crop-facing optical axis at ready pose; rigidly follows measured link3',
        mount_calibration_revision='20261110 distal forearm camera relocation',
        reference_ready_q=reference['ready_q'])
    output.mkdir(parents=True,exist_ok=False);(output/'model').mkdir()
    for name in ('inputs','inventory.json','sensor_catalog.json'):(output/name).symlink_to(source/name)
    for path in (source/'model').iterdir():
        if path.name not in ('scene.xml','export.json'):(output/'model'/path.name).symlink_to(path)
    export=json.loads((source/'model/export.json').read_text())
    tree=ET.parse(source/'model/scene.xml')
    moved=relocate_camera_geometry(tree,export['geometry'],export['robot']['body_names'],previous,mounted)
    tree.write(output/'model/scene.xml',encoding='unicode')
    visual_sha=sha(output/'model/scene.xml')
    export.update(model_sha256=visual_sha,camera_relocation=dict(source_assets=str(source),
        source_visual_sha256=sha(source/'model/scene.xml'),moved_geoms=moved,
        renderer_only=True,physics_inputs_unchanged=True))
    write(output/'model/export.json',export)
    revised=copy.deepcopy(config)
    for plant in revised['plants']:
        current_assembly=Path(plant['assembly']);manifest=json.loads((current_assembly/'motion_manifest.json').read_text())
        if ready_q is not None:manifest=dict(manifest,initial_robot_q=np.asarray(ready_q,float).tolist())
        calibration=ready_camera_service_calibration(current_assembly,manifest,
            ['--ready-lift-offset-mm','0','--grasp-depth-mm','35'],config['bed_gutter_center_x'],mounted_camera=mounted)
        calibration.update(mounted_body_path=body,body_from_camera_optical=local.tolist())
        target=output/f"plant_{plant['plant_id']:02d}_camera_calibration.json";write(target,calibration)
        plant['camera_calibration']=str(target)
    revised.update(mounted_rgbd=mounted,camera_relocation=dict(source_assets=str(source),
        visual_xml_sha256=visual_sha,moved_geoms=moved,previous_mount=previous,
        physical_model_sha256=sha(source/'inputs/input_model.mjb'),
        renderer_only=True,physics_inputs_unchanged=True,
        calibration_source='Existing exact assembly FK with the explicit new optical body/extrinsic'))
    write(output/'patrol_assets.json',revised)
    print('Prepared camera relocation:',output,flush=True)
    return revised


def repose_robot_inputs(source,output,ready_path):
    """Author a separate zero-time robot start; never alter a running state."""
    import mujoco as mj
    from newton_inputs import sha256,command_timeline
    from newton_prepared_assets import verify_manifest_files,link_manifest_assets
    source=Path(source).resolve(strict=True);output=Path(output).resolve()
    proof=json.loads((source/'inputs.json').read_text());verify_manifest_files(source,proof['input_files'])
    meta=json.loads((source/'source_export.json').read_text())
    if meta.get('active_window',{}).get('all_plants_static') is not True:
        raise ValueError('New ready profile requires the existing static-plant patrol model')
    q=np.asarray(json.loads(Path(ready_path).read_text())['ready_q'],float)
    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Finite seven-joint start required')
    initial=dict(np.load(source/'initial_state.npz',allow_pickle=False))
    if float(initial['time'][0])!=0 or np.any(initial['qvel']):raise ValueError('Only fresh zero-time initial assets may be reposed')
    model=mj.MjModel.from_binary_path(str(source/'input_model.mjb'));data=mj.MjData(model)
    command=np.r_[0.,q,0.,0.];data.qpos[:]=initial['qpos'];data.qpos[initial['robot_qpos_addresses']]=command
    mj.mj_forward(model,data)
    for contact in data.contact:
        robot=any(model.body(int(model.geom_bodyid[g])).name.startswith(('Robot_','Tool_')) for g in contact.geom)
        if robot and contact.dist<-.001:raise ValueError('New ready state collides in the unchanged native model')
    output.mkdir(parents=True,exist_ok=False)
    generated={'initial_state.npz','canonical_full_state.npz','sampled_plan.json','commands.npz','source_export.json'}
    link_manifest_assets(source,output,set(proof['input_files'])-generated)
    for name in ('initial_state.npz','canonical_full_state.npz'):
        state=dict(np.load(source/name,allow_pickle=False))
        state['qpos'][state['robot_qpos_addresses']]=command
        state['ctrl'][state['actuator_ids']]=command
        indices=[model.body(str(n)).id for n in state['body_names']]
        state['body_positions']=data.xpos[indices].copy();state['body_quaternions_wxyz']=data.xquat[indices].copy()
        np.savez_compressed(output/name,**state)
    plan=idle_plan(dict(initial_robot_q=q.tolist()));write(output/'sampled_plan.json',plan)
    revision=dict(source_initial_state_sha256=sha256(source/'initial_state.npz'),ready_q=q.tolist(),
        fresh_zero_time_start=True,runtime_teleport=False,physical_model_sha256=sha256(source/'input_model.mjb'))
    meta.update(prepared_source_plan=str(output/'sampled_plan.json'),initial_pose_revision=revision)
    write(output/'source_export.json',meta)
    timeline,description,_=command_timeline(meta,command,220,free_hz=60)
    np.savez_compressed(output/'commands.npz',**timeline);write(output/'timeline.json',description)
    proof.update(timeline=description,initial_pose_revision=revision)
    for name in proof['input_files']:proof['input_files'][name]=dict(path=str(output/name),sha256=sha256(output/name))
    write(output/'inputs.json',proof)


def prepare_elbow_up_assets(source,output):
    from pxr import Usd
    from motion_kinematics import ToolKinematics,elbow_up_clearance
    from motion_self_collision import MotionSelfCollision
    source=Path(source).resolve(strict=True);output=Path(output).resolve()
    config=json.loads((source/'patrol_assets.json').read_text());plant=config['plants'][0]
    kin=ToolKinematics(Usd.Stage.Open(str(Path(plant['assembly'])/'scene.usda')))
    old=np.asarray(json.loads(Path(plant['camera_calibration']).read_text())['ready_q']);goal=kin.fk(old)
    checker=MotionSelfCollision(Usd.Stage.Open(str(Path(plant['assembly'])/'scene.usda')),kin,clearance_m=.002)
    proposals=[]
    for shoulder in (1.,1.5,2.):
        for elbow in (-2.5,-1.5):
            seed=old.copy();seed[2]=shoulder;seed[3]=elbow;seed[4]=sum(old[2:5])-shoulder-elbow
            q,pe,re=kin.ik_at_lift(goal,seed,old[0]);clearance=elbow_up_clearance(kin,q)
            hits=checker.check(q,.096)+checker.check(q,.044) if pe<=.002 and re<=.02 else []
            proposals.append(dict(ready_q=q.tolist(),position_error_m=pe,rotation_error_rad=re,
                elbow_above_shoulder_and_tool_m=clearance,collisions=hits,
                accepted=bool(pe<=.002 and re<=.02 and clearance>0 and not hits)))
    valid=[r for r in proposals if r['accepted']]
    if not valid:raise RuntimeError('Elbow-up ready IK not found with existing self-collision checks')
    selected=max(valid,key=lambda r:r['elbow_above_shoulder_and_tool_m'])
    revised=relocate_camera_assets(source,output,ready_q=selected['ready_q'])
    ready=json.loads(Path(revised['plants'][0]['camera_calibration']).read_text())
    write(output/'elbow_up_ready.json',dict(ready,original_ready_q=old.tolist(),proposals=proposals,
        selected=selected,initialization='New independent zero-time spawn; no running pose write'))
    (output/'inputs').unlink()
    subprocess.run(list(map(str,[ROOT/'nvidia-sim/.newton-venv/bin/python',HERE/'prepare_newton_patrol.py',output/'inputs',
        '--repose-inputs-from',source/'inputs','--ready-q-json',output/'elbow_up_ready.json'])),check=True)
    revised.update(elbow_posture='up',inputs=str(output/'inputs'),initial_pose_revision=str(output/'elbow_up_ready.json'))
    write(output/'patrol_assets.json',revised)
    print('Prepared elbow-up start and camera:',output,flush=True)


def prepare_elbow_modes_assets(source,output,down_reference):
    """Package the current up hardware and original down start as selectable modes."""
    from pxr import Usd
    from motion_kinematics import ToolKinematics,elbow_posture_clearance
    from motion_self_collision import MotionSelfCollision
    from mounted_rgbd import sha
    source=Path(source).resolve(strict=True);output=Path(output).resolve()
    config=json.loads((source/'patrol_assets.json').read_text())
    if config.get('elbow_posture')!='up':raise ValueError('Use the current elbow-up assets as source')
    reference=json.loads((Path(down_reference)/'patrol_assets.json').read_text())
    ready=json.loads(Path(reference['plants'][0]['camera_calibration']).read_text())
    q=np.asarray(ready['ready_q'])
    stage=Usd.Stage.Open(str(Path(config['plants'][0]['assembly'])/'scene.usda'))
    kin=ToolKinematics(stage);checker=MotionSelfCollision(stage,kin,clearance_m=.002)
    if elbow_posture_clearance(kin,q,'down')<=0 or checker.check(q,.096)+checker.check(q,.044):
        raise ValueError('Original elbow-down ready posture does not pass the current robot checks')
    output.mkdir(parents=True,exist_ok=False)
    for path in source.iterdir():
        if path.name!='patrol_assets.json':(output/path.name).symlink_to(path)
    down=output/'down'
    revised=relocate_camera_assets(source,down,ready_q=q)
    calibrated=json.loads(Path(revised['plants'][0]['camera_calibration']).read_text())
    write(down/'elbow_down_ready.json',dict(calibrated,reference_assets=str(Path(down_reference).resolve()),
        elbow_clearance_m=elbow_posture_clearance(kin,q,'down'),fresh_zero_time_start=True))
    (down/'inputs').unlink()
    subprocess.run(list(map(str,[ROOT/'nvidia-sim/.newton-venv/bin/python',HERE/'prepare_newton_patrol.py',down/'inputs',
        '--repose-inputs-from',source/'inputs','--ready-q-json',down/'elbow_down_ready.json'])),check=True)
    revised.update(elbow_posture='down',inputs=str(down/'inputs'),initial_pose_revision=str(down/'elbow_down_ready.json'))
    write(down/'patrol_assets.json',revised)
    config.update(start_modes=dict(up=str(source),down=str(down)),default_start_mode='up')
    write(output/'patrol_assets.json',config)
    write(output/'elbow_modes.json',dict(up=str(source),down=str(down),
        physical_model_same=sha(source/'inputs/input_model.mjb')==sha(down/'inputs/input_model.mjb'),
        sensor_note='Each authored start profile includes its previously calibrated forearm mount; no sensor transform changes after START',
        runtime_switch_allowed=False))
    print('Prepared selectable elbow modes:',output,flush=True)


def remove_lift_braces_assets(source,output):
    """Remove only the two diagonal webs; retain the CAD vertical/base legs.

    Native Python has the existing manifold mesh cutter. The reviewed CAD
    parts are visual-only in both the planner and the render-free GPU model.
    """
    import copy
    import trimesh
    import mujoco as mj
    import xml.etree.ElementTree as ET
    from newton_inputs import sha256
    from newton_prepared_assets import verify_manifest_files
    source=Path(source).resolve(strict=True);output=Path(output).resolve()
    config=json.loads((source/'patrol_assets.json').read_text())
    export=json.loads((source/'model/export.json').read_text())
    if sha256(source/'model/scene.xml')!=export['model_sha256']:
        raise ValueError('Source visual model SHA mismatch')
    proof=json.loads((source/'inputs/inputs.json').read_text())
    verify_manifest_files(source/'inputs',proof['input_files'])
    physical=mj.MjModel.from_binary_path(str(source/'inputs/input_model.mjb'))
    tree=ET.parse(source/'model/scene.xml')
    reviewed={
        'fixed_16':'c18b6f4dcd4a72e52962c01de3eb4f826895cf1a80d40a9627787d6c879f4767',
        'fixed_17':'ee52493ffc14f2dc0e31fcded618e3f87167710222255c7117e7885add3cb294'}
    rows=[r for r in export['geometry'] if r['source'].rsplit('/',1)[-1] in reviewed
          and '/real_lift_fixed/' in r['source']]
    if len(rows)!=2:raise ValueError('Expected the two reviewed fixed lift side supports')
    revised_meshes=[]
    for row in rows:
        geom=tree.find(".//geom[@name='%s']"%row['name'])
        asset=tree.find("./asset/mesh[@name='%s']"%geom.get('mesh'))
        path=source/'model'/tree.find('compiler').get('meshdir')/asset.get('file')
        if row['collision'] or (geom.get('contype'),geom.get('conaffinity'))!=('0','0'):
            raise ValueError('Lift support unexpectedly has a collider')
        if mj.mj_name2id(physical,mj.mjtObj.mjOBJ_GEOM,row['name'])!=-1:
            raise ValueError('Lift support is present in the physical MJB; explicit re-export required')
        if sha256(path)!=reviewed[row['source'].rsplit('/',1)[-1]]:
            raise ValueError('Lift CAD revision changed; review the diagonal cut before applying')
        mesh=trimesh.load(path,force='mesh',process=True)
        # Recover the small original CAD import rotation from its planar faces.
        x=mesh.face_normals[np.argmax(mesh.area_faces*(abs(mesh.face_normals[:,0])>.99))].copy();x*=np.sign(x[0])
        y=mesh.face_normals[np.argmax(mesh.area_faces*(abs(mesh.face_normals[:,1])>.99))].copy();y*=np.sign(y[1])
        z=np.cross(x,y);z/=np.linalg.norm(z);y=np.cross(z,x)
        rotation=np.column_stack((x,y,z));mesh.vertices=mesh.vertices@rotation
        lo,hi=mesh.bounds
        if not np.allclose(hi-lo,[.02,.49,.45],atol=1e-6):
            raise ValueError('Unexpected lift support dimensions')
        # Original upright and horizontal feet are 40 mm wide. Remove only
        # the material above the foot and in front of the upright; cap cuts.
        cut_lo=lo-1.;cut_hi=hi+1.;cut_hi[1]=hi[1]-.04;cut_lo[2]=lo[2]+.04
        cutter=trimesh.creation.box(extents=cut_hi-cut_lo,
            transform=trimesh.transformations.translation_matrix((cut_lo+cut_hi)/2))
        trimmed=trimesh.boolean.difference([mesh,cutter],engine='manifold')
        if not trimmed.is_volume or not 0<trimmed.volume<mesh.volume:
            raise ValueError('Diagonal removal did not produce a closed remaining support')
        trimmed.vertices=trimmed.vertices@rotation.T
        revised_meshes.append((row,asset,trimmed,dict(source_mesh_sha256=sha256(path),
            original_triangles=len(mesh.faces),remaining_triangles=len(trimmed.faces),
            original_volume_m3=float(mesh.volume),remaining_volume_m3=float(trimmed.volume),
            retained_leg_width_m=.04,watertight=True,source=row['source'],geom=row['name'])))
    output.mkdir(parents=True,exist_ok=False);(output/'model').mkdir()
    for path in source.iterdir():
        if path.name not in ('model','patrol_assets.json'):(output/path.name).symlink_to(path)
    for path in (source/'model').iterdir():
        if path.name not in ('scene.xml','export.json'):(output/'model'/path.name).symlink_to(path)
    (output/'model/lift_revision').mkdir()
    records=[]
    for row,asset,mesh,record in revised_meshes:
        target=output/'model/lift_revision'/(row['name']+'.obj');mesh.export(target)
        asset.set('file',str(target))
        row['mesh_sha256']=sha256(target);row['visual_revision']='diagonal removed; upright/base retained'
        records.append(dict(record,mesh_sha256=row['mesh_sha256']))
    tree.write(output/'model/scene.xml',encoding='unicode')
    visual_sha=sha256(output/'model/scene.xml')
    revision=dict(source_assets=str(source),source_visual_sha256=export['model_sha256'],
        visual_xml_sha256=visual_sha,parts=records,physics_inputs_unchanged=True,
        physical_model_sha256=sha256(source/'inputs/input_model.mjb'),
        affects=['browser geometry','RGB-D occlusion geometry'],
        collision_note='Both source parts are visual-only; no planner or GPU colliders existed for the removed diagonals')
    export.update(model_sha256=visual_sha,lift_brace_removal=revision)
    write(output/'model/export.json',export)
    revised=copy.deepcopy(config);revised.update(inputs=str(output/'inputs'),lift_brace_removal=revision)
    # The camera mount is unchanged, but its scene identity includes this edit.
    if 'camera_relocation' in revised:revised['camera_relocation']['visual_xml_sha256']=visual_sha
    write(output/'patrol_assets.json',revised);write(output/'lift_brace_removal.json',revision)
    print('Prepared lift diagonal removal:',output,flush=True)


def prepare(output, source_scene, *, end_station=.6, bed_x=-.775, reuse_hardware=None, physical_harvest=False, resume=False):
    from pxr import Usd
    from rail_assets import near_side_inventory, revise as rail
    from row_harvest_targets import select_one_bed
    from motion_assets import build, revise_for_horizontal_ready
    from fruit_attachment import revise as attachment
    from compliant_foliage import revise as foliage
    from compact_motion_assets import compact_assembly
    from side_robot_mount import prepare as side
    from prepare_harvest_group_policy import prepare as group_policy
    from mounted_gt import build_catalog
    from export_scene import export
    from merge_windows import merge
    from compact_motion_assets import verify_compact_dependencies, file_sha256
    import time
    resume_actions=[]
    def recover(folder, required, validate=None):
        """Reuse only published, verified assets; preserve unfinished folders."""
        folder=Path(folder)
        if not folder.exists():return False
        if not resume:raise FileExistsError(folder)
        if all((folder/n).is_file() for n in required):
            if validate:validate(folder)
            resume_actions.append(dict(path=str(folder),action='verified_reuse'))
            return True
        backup=folder.with_name(folder.name+'_interrupted_'+str(time.time_ns()))
        folder.rename(backup)
        resume_actions.append(dict(path=str(folder),action='preserved_incomplete',backup=str(backup)))
        return False
    def validate_assembly(folder):
        m=json.loads((folder/'motion_manifest.json').read_text())
        if file_sha256(folder/'scene.usda')!=m['scene_sha256']:raise ValueError('Resume scene SHA mismatch')
        verify_compact_dependencies(folder,m)
        expected=f'/World/Greenhouse/Plants/{folder.parent.name.replace("plant_","Plant_")}'
        if m['plant']['source_plant']!=expected:raise ValueError('Resume source plant identity mismatch')
    def assembly_step(folder, function, *args, **kwargs):
        if not recover(folder,('scene.usda','motion_manifest.json','skin.npz'),validate_assembly):
            function(*args,folder,**kwargs)
    output=Path(output).resolve();source_scene=Path(source_scene).resolve(strict=True)
    if not 0 < end_station <= 5: raise ValueError('End station must be within (0,5]')
    output.mkdir(parents=True,exist_ok=resume)
    inv=near_side_inventory(Usd.Stage.Open(str(source_scene)))
    # Whole selected bed inventory remains in the observer. Only explicit
    # route-budget plants get dynamic assets; out-of-budget targets stay logged.
    proposals=[dict(plant_id=r['plant_id'],truss_index=r['truss_index']) for r in inv['trusses']]
    selection=select_one_bed(proposals,inv,bed_x)
    identities={(r['plant_id'],r['truss_index']) for r in selection['stations']}
    chosen=sorted({r['plant_id'] for r in inv['trusses']
        if (r['plant_id'],r['truss_index']) in identities and
           0 <= r['fruit_centroid_world_m'][1] <= end_station})
    if not chosen: raise ValueError('No near-side source plants within this bounded patrol')
    write(output/'inventory.json',inv)
    sources=[];catalogs={};maps={};models=[]
    for index,plant in enumerate(chosen):
        p=output/f'plant_{plant:02d}';p.mkdir(exist_ok=resume)
        print('Preparing source plant',plant,flush=True)
        hardware=Path(reuse_hardware)/f'plant_{plant:02d}/hardware' if reuse_hardware else p/'hardware'
        if reuse_hardware and hardware.exists():
            saved=json.loads((hardware/'motion_manifest.json').read_text())
            if saved['plant']['source_plant']!=f'/World/Greenhouse/Plants/Plant_{plant:02d}':
                raise ValueError('Reusable hardware plant identity mismatch')
        else:
            hardware=p/'hardware'
            assembly_step(p/'motion',build,source_scene,plant=plant)
            assembly_step(hardware,revise_for_horizontal_ready,p/'motion',hardware_only=True)
        assembly_step(p/'attachment',attachment,hardware,kind='elastic',force_n=3.,torque_nm=.008,all_trusses=True)
        print('Plant',plant,'attachments prepared; assembling foliage, rail and side mount',flush=True)
        assembly_step(p/'foliage',foliage,p/'attachment')
        assembly_step(p/'rail',rail,p/'foliage')
        assembly_step(p/'compact',compact_assembly,p/'rail')
        assembly_step(p/'assembly',side,p/'compact',initial_station_y_m=0.)
        assembly=p/'assembly';m=json.loads((assembly/'motion_manifest.json').read_text())
        from camera_patrol_runner import ready_camera_service_calibration
        calibration=ready_camera_service_calibration(assembly,m,
            ['--ready-lift-offset-mm','0','--grasp-depth-mm','35'],bed_x)
        write(p/'camera_calibration.json',calibration)
        if resume and (p/'policy.json').is_file():
            policy=json.loads((p/'policy.json').read_text())
            if (policy['minimum_fruits']!=2 or any(file_sha256(assembly/n)!=h
                    for n,h in policy['source_sha256'].items())):
                raise ValueError('Resume policy source mismatch')
        else:group_policy(assembly,p/'policy.json',minimum=2)
        write(p/'plan.json',idle_plan(m))
        print('Plant',plant,'exporting native geometry and camera identities',flush=True)
        def validate_model(folder):
            meta=json.loads((folder/'export.json').read_text())
            if file_sha256(folder/'scene.xml')!=meta['model_sha256']:raise ValueError('Resume model SHA mismatch')
            if meta['source_manifest_sha256']!=file_sha256(assembly/'motion_manifest.json'):
                raise ValueError('Resume model manifest mismatch')
        if not recover(p/'model',('scene.xml','export.json','source_plan.json'),validate_model):
            export(assembly,p,p/'model',truss='*',robot=index==0,plant_only=index!=0,
                background=index==0,ground_collision='top-plane',source_plan=p/'plan.json',
                excluded_background_roots=[f'/World/Greenhouse/Plants/Plant_{x:02d}' for x in chosen],
                semantic_background=index==0)
        stage=Usd.Stage.Open(str(assembly/'composed.usdc'))
        meshes,objects=build_catalog(stage,assembly)
        prefix='' if index==0 else f'p{plant:02d}_'
        meta=json.loads((p/'model/export.json').read_text())
        for g in meta['geometry']:
            identity=meshes.get(g['source'])
            if identity is not None:
                maps[prefix+g['name']]=identity;catalogs[str(identity)]=objects[identity]
        if index==0:
            for g in meta['farm']['geometry_inventory']:
                ids={meshes[x] for x in g.get('source_paths',[]) if x in meshes}
                if len(ids)>1: raise ValueError('Semantic batch mixed object identities: '+g['name'])
                if ids:
                    identity=next(iter(ids));maps[g['name']]=identity;catalogs[str(identity)]=objects[identity]
        models.append((prefix,p/'model'))
        sources.append(dict(plant_id=plant,assembly=str(assembly),prefix=prefix,policy=str(p/'policy.json'),
                            camera_calibration=str(p/'camera_calibration.json')))
    merge(models[0][1],models[1:],output/'model')
    print('Compiling canonical Newton patrol inputs for plants',chosen,flush=True)
    import shutil
    shutil.copy2(models[0][1]/'source_plan.json',output/'model/source_plan.json')
    merged=json.loads((output/'model/export.json').read_text())
    for window in merged['windows']:
        window['display_root']=f"/World/NewtonPlants/Plant_{int(window['id']):02d}"
    write(output/'model/export.json',merged)
    write(output/'sensor_catalog.json',dict(objects=catalogs,geom_to_instance=maps))
    py=ROOT/'nvidia-sim/.newton-venv/bin/python'
    for command in ([py,HERE/'newton_inputs.py',output/'model',output/'inputs_full','--hz','220','--discard-visual-geoms','--allow-multiple-windows'],
                    [py,HERE/'newton_plant_variant.py',output/'inputs_full',output/'inputs_coarse','--mode','coarse4'],
                    [py,HERE/'newton_fruit_proxy.py',output/'inputs_coarse',output/('inputs' if physical_harvest else 'inputs_proxy')]):
        subprocess.run(list(map(str,command)),check=True)
    if not physical_harvest:
        subprocess.run(list(map(str,[py,HERE/'newton_geometric_harvest.py','--inputs-only',output/'inputs_proxy',output/'inputs'])),check=True)
    config=dict(schema='s300-newton-camera-patrol-assets-v1',
        start_station_m=0.,end_station_m=end_station,bed_gutter_center_x=bed_x,plants=sources,
        source_scene=str(source_scene),inputs=str(output/'inputs'),
        collision_scope='Prepared plants have authored stem/truss colliders; other farm plants visual only. Active-window mode freezes non-active prepared plants; released fruit stays dynamic.',
        dynamics='Existing coarse4 fruit-proxy adapter; every truss of each prepared plant retained',
        old_experiment_required=False,training_eligible=False,harvest_success=None)
    if not physical_harvest:
        config.update(harvest_mode='main_stem_only_geometric_harvest',
            dynamics='Robot coordinates only; plant bodies static, every original visual body retained',
            collision_scope='Only thick main stems collide; fruit and thin plant geometry have zero masks. Robot/structure unchanged.',
            geometric_harvest=json.loads((output/'inputs/geometric_harvest.json').read_text()))
    write(output/'patrol_assets.json',config)
    if resume:write(output/'prepare_resume.json',dict(source_scene=str(source_scene),actions=resume_actions))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--source-scene',type=Path,default=Path('/root/farmily_runtime/newton/assets/base_20261108_v2/scene.usda'))
    p.add_argument('--end-station',type=float,default=.6)
    p.add_argument('--bed-x',type=float,default=-.775)
    p.add_argument('--reuse-hardware',type=Path,help='Optional source asset cache from a previous preparation; never trial states')
    p.add_argument('--physical-harvest',action='store_true',help='Explicit legacy comparison: plant contacts and physical release')
    p.add_argument('--relocate-camera-from',type=Path,help='Create a sensor-only distal-forearm revision of existing patrol assets; retain physics and original assets')
    p.add_argument('--elbow-up-from',type=Path,help='New elbow-up ready spawn and camera calibration; preserve source assets')
    p.add_argument('--remove-lift-braces-from',type=Path,help='Remove the two visual diagonal lift supports, retaining their upright/base legs')
    p.add_argument('--elbow-modes-from',type=Path,help='Package the current up profile and an original down reference for GUI selection')
    p.add_argument('--elbow-down-reference',type=Path,help='Existing original elbow-down patrol assets')
    p.add_argument('--repose-inputs-from',type=Path,help=argparse.SUPPRESS)
    p.add_argument('--ready-q-json',type=Path,help=argparse.SUPPRESS)
    a=p.parse_args()
    if a.elbow_modes_from:
        if a.elbow_down_reference is None:p.error('--elbow-down-reference is required')
        prepare_elbow_modes_assets(a.elbow_modes_from,a.output,a.elbow_down_reference)
    elif a.remove_lift_braces_from:remove_lift_braces_assets(a.remove_lift_braces_from,a.output)
    elif a.repose_inputs_from:repose_robot_inputs(a.repose_inputs_from,a.output,a.ready_q_json)
    elif a.elbow_up_from:prepare_elbow_up_assets(a.elbow_up_from,a.output)
    elif a.relocate_camera_from:relocate_camera_assets(a.relocate_camera_from,a.output)
    else:prepare(a.output,a.source_scene,end_station=a.end_station,bed_x=a.bed_x,reuse_hardware=a.reuse_hardware,physical_harvest=a.physical_harvest)
