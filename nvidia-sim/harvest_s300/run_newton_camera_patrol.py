"""Entrance -> short rail move -> mounted observation -> harvest -> reobserve.

Uses CameraPatrol/CameraRegionSelector, the measured plan adapter, and the
existing Newton GPU executor/verified active-window state bank. No Isaac/Kit,
saved trajectory playback, detector, or environment reset between decisions.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import mujoco as mj
import numpy as np

from camera_patrol import CameraPatrol
from camera_region_policy import CameraRegionSelector
from newton_inputs import sha256, command_timeline
from newton_active_window import canonical_state, prepare as active_prepare, _window_selection
from newton_window_continuation import checked_window_parent, create_transition
from newton_continuation import load_resume,VerificationContext
from newton_prepared_assets import verify_manifest_files, link_manifest_assets
from plan_adapter import sample_plan
from newton_geometric_harvest import MODE, geometric_plan, capture_receipt, sensor_poses, volume_bounds
from newton_patrol_motion import fluid_plan, fluid_scan, cruise_route
from newton_scene_chunks import stream_manifest, PackedScene

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).parent
MAX_ENTRY_YAW_DEG=15.
ENTRY_YAW_CANDIDATES=(0.,-7.5,7.5,-15.,15.)


def read(p):return json.loads(Path(p).read_text())
def write(p,v):Path(p).write_text(json.dumps(v,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def record_scan_reacquisition(patrol,decision,observation,station):
    """Reuse the native runner's finite forward-view recovery after a scan.

    Remembered invisible fruit earns no harvest credit. A fresh unsuccessful
    scan must consume a view attempt, rather than repeatedly requesting the
    identical ready pose at the identical rail position.
    """
    key=decision.get('target_key')
    if (decision.get('action')!='scan' or key not in patrol.queue or
            not patrol.observations or patrol.observations[-1]['stale']):return None
    item=patrol.queue[key]
    if item['visible_now'] or item['status'] not in ('pending','revisit_required'):return None
    return patrol.record_local_failure(key,'fresh_scan_did_not_reacquire_minimum_visible_red_group',
        dict(actual_observation=observation,observed_attached_fruit_keys=item['observed_attached_fruit_keys'],
             no_harvest_motion_attempted=True),current_station_y_m=station,
        native_observation_executed=True,max_forward_views=2)


def entry_yaw_audit(directions,forward):
    vectors=np.asarray(directions,float);reference=np.asarray(forward,float)
    if (vectors.ndim!=2 or vectors.shape[1]!=3 or not len(vectors) or
            not np.isfinite(vectors).all() or reference.shape!=(3,) or not np.isfinite(reference).all() or
            np.linalg.norm(reference[:2])<1e-9 or np.any(np.linalg.norm(vectors[:,:2],axis=1)<1e-9)):
        raise ValueError('Finite nondegenerate entry axes and robot reference required')
    angles=np.rad2deg(np.arctan2(reference[0]*vectors[:,1]-reference[1]*vectors[:,0],
                                vectors[:,:2]@reference[:2]))
    maximum=float(np.abs(angles).max())
    return dict(passed=maximum<=MAX_ENTRY_YAW_DEG+1e-8,samples=len(angles),
        reference_forward_world=reference.tolist(),maximum_allowed_deg=MAX_ENTRY_YAW_DEG,
        minimum_yaw_deg=float(angles.min()),maximum_yaw_deg=float(angles.max()),maximum_absolute_yaw_deg=maximum,
        scope='Horizontal Tool_fixed +Y yaw during approach, manipulation hold and withdraw; ready transfers excluded',
        numerical_roundoff_tolerance_deg=1e-8)


def entry_phase_names(plan):
    names=[p['name'] for p in plan['phases']]
    return names[names.index('approach'):names.index('withdraw')+1]


def posture_evidence(elbow,shoulder,tool,mode):
    if mode not in ('up','down'):raise ValueError('Unknown elbow posture')
    clearance=(np.asarray(elbow)-np.maximum(shoulder,tool) if mode=='up' else
               np.minimum(shoulder,tool)-np.asarray(elbow))
    minimum=float(np.min(clearance))
    report=dict(required=mode,minimum_posture_clearance_m=minimum,passed=minimum>0.)
    if mode=='up':report['minimum_height_above_shoulder_and_tool_m']=minimum
    else:report['minimum_depth_below_shoulder_and_tool_m']=minimum
    return report


def planned_entry_yaw(model,bank,qadr,plan,forward,*,require_elbow_up=False,elbow_posture=None):
    mode=elbow_posture or ('up' if require_elbow_up else None)
    data=mj.MjData(model);data.qpos[:]=bank['qpos'];tool=model.body('Tool_fixed').id
    axes=[];heights=[];phases=entry_phase_names(plan)
    for t in np.arange(0,plan['total_duration_s']+1/60,1/60):
        sample=sample_plan(plan,min(t,plan['total_duration_s']))
        if sample['phase'] not in phases:continue
        data.qpos[qadr]=sample['command'];mj.mj_kinematics(model,data)
        axes.append(data.xmat[tool].reshape(3,3)[:,1].copy())
        if mode:
            heights.append([data.xpos[model.body('Robot_link3').id,2],
                data.xpos[model.body('Robot_link2').id,2],data.xpos[tool,2]])
    report=entry_yaw_audit(axes,forward)
    report['requested_direction']=entry_yaw_audit([plan['approach_direction_world']],forward)
    report['passed']=report['passed'] and report['requested_direction']['passed']
    if mode:
        report['elbow_posture']=posture_evidence(*np.asarray(heights).T,mode)
        report['passed']=report['passed'] and report['elbow_posture']['passed']
    report['source']='Actual native FK of sampled joint commands at60Hz'
    return report


def recorded_entry_yaw(states,plan,source_mapping,forward,*,require_elbow_up=False,elbow_posture=None):
    mode=elbow_posture or ('up' if require_elbow_up else None)
    from scipy.spatial.transform import Rotation
    with np.load(states,allow_pickle=False) as saved:
        path=next(k for k,v in source_mapping.items() if v=='Tool_fixed')
        index=list(map(str,saved['body_paths'])).index(path)
        selected=np.isin(saved['phase'],entry_phase_names(plan))
        if not selected.any():raise ValueError('No recorded entry-angle samples')
        rotations=Rotation.from_quat(saved['poses'][selected,index,3:]).as_matrix()
        report=entry_yaw_audit(rotations[:,:,1],forward)
        if mode:
            paths=list(map(str,saved['body_paths']))
            def z(name):
                source=next(k for k,v in source_mapping.items() if v==name)
                return saved['poses'][selected,paths.index(source),2]
            report['elbow_posture']=posture_evidence(z('Robot_link3'),z('Robot_link2'),z('Tool_fixed'),mode)
            report['passed']=report['passed'] and report['elbow_posture']['passed']
        report.update(source='Recorded actual GPU body poses; no additional physics',
            start_timestamp_s=float(saved['time'][selected][0]),end_timestamp_s=float(saved['time'][selected][-1]),
            maximum_recorded_interval_s=float(np.diff(saved['time'][selected]).max()) if selected.sum()>1 else 0.)
    return report


def planning_failure(log):
    lines=Path(log).read_text().strip().splitlines()
    message=lines[-1] if lines else 'Empty planner log'
    if message.startswith('RuntimeError:'):
        if 'IK not found' in message:return 'ik_not_found',message
        if 'joint limits exceeded' in message:return 'joint_limit',message
        if 'FK orientation check failed' in message:return 'fk_path_quality',message
    raise RuntimeError('Planner input/adapter failure; not an IK search result: '+message)


def ordered_proposals(headings,connectors,preferred,view_round=0):
    rows=[(yaw,group,name,options) for yaw,group in headings for name,options in connectors]
    if preferred is None:return rows
    directions=list(dict.fromkeys([preferred[0]]+[row[0] for row in headings]))
    direction=directions[view_round%len(directions)]
    # A later 5cm view explores another direction instead of repeating the
    # same blocked branch three times. All finite alternatives remain present.
    return sorted(rows,key=lambda row:(row[0]!=direction,row[2]!=preferred[1]))


class PlannerProcess:
    """One existing USD/IK adapter process for a patrol, not a new planner."""
    def __init__(self, python, log):
        self.log=Path(log).open('w')
        self.process=subprocess.Popen([str(python),str(ROOT/'mujoco-benchmark/scripts/s300_smoke/plan_adapter.py'),'--serve'],
            cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)

    def plan(self, assembly, request, output, log):
        started=time.perf_counter()
        self.process.stdin.write(json.dumps(dict(assembly=str(assembly),request=str(request),output=str(output)))+'\n')
        self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('Resident IK adapter exited; see planning_worker.log')
        result=json.loads(line)
        Path(log).write_text(result.get('traceback','Resident existing IK adapter completed\n'))
        return result['ok'],time.perf_counter()-started

    def close(self):
        self.process.stdin.close()
        try:self.process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            self.process.terminate();self.process.wait(timeout=5.)
        self.log.close()


def scan_plan(command, *, destination=None, ready_q=None, speed=.08):
    command=np.asarray(command,float)
    if command.shape!=(10,) or not np.isfinite(command).all():raise ValueError('Finite measured ten-axis command required')
    if not np.isfinite(speed) or speed<=0:raise ValueError('Positive rail speed required')
    destination=float(command[0] if destination is None else destination)
    if abs(destination-command[0])>.200001:raise ValueError('Camera patrol rail step exceeds 20cm')
    q=command[1:8];goal=q if ready_q is None else np.asarray(ready_q,float)
    gap=float(.044+command[9]-command[8])
    duration=max(1.,1.5*abs(destination-command[0])/speed,3. if ready_q is not None else 0.)
    return dict(schema='s300-camera-patrol-measured-motion-v1',initial_command=command.tolist(),
        initial_gap_m=gap,settle_seconds=.05,total_duration_s=duration+.55,
        phases=[dict(name='rail_transfer' if destination!=command[0] else 'ready_scan',duration_s=duration,
            waypoint_q=[q.tolist(),goal.tolist()],rail_waypoint_positions_m=[float(command[0]),destination],
            end_gap_m=gap,belt_local_y_m_s=[0.,0.]),
            dict(name='ready_hold',duration_s=.5,waypoint_q=[goal.tolist(),goal.tolist()],
                 rail_waypoint_positions_m=[destination,destination],end_gap_m=gap,belt_local_y_m_s=[0.,0.])],
        collision_certified=False,training_eligible=False,harvest_success=None)


def segment_inputs(common, out, plan, bank, plant, index, *, verification_context=None,source_metadata=None):
    checked_sha=sha256 if verification_context is None else verification_context.file_sha256
    proof=read(common/'inputs.json');verify_manifest_files(common,proof['input_files'],verification_context=verification_context)
    out.mkdir()
    changed={'commands.npz','timeline.json','source_export.json','sampled_plan.json','initial_state.npz'}
    meta=read(common/'source_export.json') if source_metadata is None else dict(source_metadata)
    fixed_metadata=meta.get('active_window',{}).get('all_plants_static') is True
    if fixed_metadata:changed.remove('source_export.json')
    shared={k:v for k,v in proof['input_files'].items() if k not in changed}
    link_manifest_assets(common,out,shared)
    initial=dict(np.load(common/'initial_state.npz',allow_pickle=False))
    for field in ('qpos','qvel','eq_active','ctrl','runtime_dof_damping','time','qacc_warmstart'):
        if field in bank:initial[field]=bank[field].copy()
    np.savez_compressed(out/'initial_state.npz',**initial)
    meta['released_gutter_diagnostic']=True
    write(out/'sampled_plan.json',plan)
    if not fixed_metadata:write(out/'source_export.json',meta)
    timeline,description,_=command_timeline(dict(meta,prepared_source_plan=str(out/'sampled_plan.json')),
        initial['qpos'][initial['robot_qpos_addresses']],220,free_hz=60)
    np.savez_compressed(out/'commands.npz',**timeline);write(out/'timeline.json',description)
    proof['timeline']=description
    for name in (*proof['input_files'],'sampled_plan.json'):
        proof['input_files'][name]=dict(path=str(out/name),sha256=checked_sha(out/name))
    proof['same_bed_sequence']=dict(index=index,plant_id=plant,source_common=str(common),training_eligible=False)
    write(out/'inputs.json',proof)


def poses_from_bank(model,bank):
    data=mj.MjData(model);data.qpos[:]=bank['qpos'];mj.mj_kinematics(model,data)
    return {model.body(i).name:np.r_[data.xpos[i],data.xquat[i,[1,2,3,0]]] for i in range(1,model.nbody)}


def share_compiled_assets(inputs, pool):
    """Deduplicate only newly generated byte-identical immutable model files."""
    pool.mkdir(exist_ok=True)
    proof=read(inputs/'inputs.json');saved=0
    for name in ('input_model.mjb','input_scene.xml'):
        original=inputs/name;digest=sha256(original)
        if digest!=proof['input_files'][name]['sha256']:raise ValueError('Generated model SHA mismatch')
        shared=pool/(digest+original.suffix)
        if shared.exists():
            if sha256(shared)!=digest:raise ValueError('Shared model cache SHA mismatch')
            saved+=original.stat().st_size
        else:
            # Hard-link before atomically replacing only this run's duplicate.
            os.link(original,shared)
        link=inputs/(name+'.shared-link');link.symlink_to(shared)
        os.replace(link,original)
    return saved


def screen_preparation(model, bank, qadr, plan, *, hz=60, active_body_names=None, entire_path=False, sample_observer=None):
    """Sample the existing plan against the identical native collision model.

    This only rejects intersecting preparation commands. It does not certify
    tracking/contact dynamics, and it never moves a plant to make room.
    """
    data=mj.MjData(model);data.qpos[:]=bank['qpos']
    robot=np.array([model.body(int(b)).name.startswith(('Robot_', 'RB5_', 'Tool_', 'S300_', 'Lift_'))
                    for b in model.geom_bodyid])
    count=0;elastic_hits=[];seen=set()
    for t in np.arange(0,plan['total_duration_s']+1/hz,1/hz):
        sample=sample_plan(plan,min(t,plan['total_duration_s']))
        if not entire_path and sample['phase']=='approach':break
        data.qpos[qadr]=sample['command']
        mj.mj_fwdPosition(model,data);count+=1
        if sample_observer is not None:sample_observer(data,sample)
        for c in data.contact:
            a,b=map(int,c.geom)
            if c.dist < -.001 and (robot[a] or robot[b]):
                bodies=[model.body(int(model.geom_bodyid[g])).name for g in (a,b)]
                hit=dict(collision_certified=False,samples=count,
                    phase=sample['phase'],time_s=float(t),penetration_m=float(-c.dist),
                    geoms=[model.geom(g).name for g in (a,b)],
                    bodies=bodies,
                    criterion='Existing 1mm preparation limit; measured plant frozen only for screening')
                # Frozen geometry cannot predict a flexible branch pushing
                # aside. Record this overlap; the real GPU guard still uses
                # its unchanged 1mm criterion on the deformed scene.
                elastic=any((name in active_body_names if active_body_names is not None else
                    ('Truss_' in name or name.startswith(('Stem_', 'MainStem')))) for name in bodies)
                if elastic and not (robot[a] and robot[b]):
                    key=tuple(hit['geoms'])
                    if key not in seen:elastic_hits.append(hit);seen.add(key)
                    continue
                return dict(hit,passed=False,elastic_intersections=elastic_hits)
    return dict(passed=True,collision_certified=False,samples=count,
        elastic_intersections=elastic_hits,
        scope='Rigid preparation intersections screened; flexible overlap diagnostic only. Actual GPU guard remains authoritative')


def audit_static_plan(model,bank,qadr,plan,forward,*,elbow_posture=None):
    """One unchanged 60Hz native pass for contacts, entry yaw and elbow pose.

    Stop at the first forbidden collision exactly as the original screen did.
    In that case the remaining angle check is explicitly incomplete. Valid
    paths retain every original sample, body, collider and 1mm threshold.
    """
    tool=model.body('Tool_fixed').id
    elbow=model.body('Robot_link3').id;shoulder=model.body('Robot_link2').id
    phases=set(entry_phase_names(plan));axes=[];heights=[]
    # Materialize numeric waypoint arrays once, retaining the existing sampler
    # and its validation/arithmetic at every timestamp.
    prepared=copy.deepcopy(plan)
    for row in prepared['phases']:
        for key in ('waypoint_q','rail_native_command_positions_m','rail_waypoint_positions_m'):
            if key in row:row[key]=np.asarray(row[key],float)
    def observe(data,sample):
        if sample['phase'] not in phases:return
        axes.append(data.xmat[tool].reshape(3,3)[:,1].copy())
        if elbow_posture:heights.append([data.xpos[elbow,2],data.xpos[shoulder,2],data.xpos[tool,2]])
    screen=screen_preparation(model,bank,qadr,prepared,active_body_names=set(),entire_path=True,
                              sample_observer=observe)
    if not screen['passed']:
        return dict(passed=False,complete=False,reason='Forbidden collision stopped path audit',
                    samples=len(axes)),screen
    angles=entry_yaw_audit(axes,forward)
    angles['requested_direction']=entry_yaw_audit([plan['approach_direction_world']],forward)
    angles['passed']=angles['passed'] and angles['requested_direction']['passed']
    if elbow_posture:
        angles['elbow_posture']=posture_evidence(*np.asarray(heights).T,elbow_posture)
        angles['passed']=angles['passed'] and angles['elbow_posture']['passed']
    angles.update(complete=True,source='Actual native FK in the same complete60Hz collision pass')
    return angles,screen


def source_fruit_manifest(source):
    if source.get('fruit_catalog'):
        path=Path(source['fruit_catalog'])
        if sha256(path)!=source['fruit_catalog_sha256']:raise ValueError('Static fruit catalog changed')
        manifest=read(path)
        if manifest.get('physical_attachments') is not False:raise ValueError('Expected logical static fruit registration')
        return manifest
    return read(Path(source['assembly'])/'motion_manifest.json')


def select_group(observation, source, meta, poses, breaks, attempts, target, output, *, yaw_offset_deg=0.,entry_forward_world=None):
    manifest=source_fruit_manifest(source);fruits=manifest['plant']['fruits']
    prefix=source['prefix'];by_name={f['name']:f for f in fruits}
    paths=[f['path'] for f in fruits];fruit_poses=np.array([poses[prefix+f['name']] for f in fruits])
    local_breaks=[dict(name=f['name'],joint=f['joint']) for f in fruits if prefix+f['name'] in breaks]
    policy=read(source['policy'])
    if source.get('fruit_catalog') and policy.get('static_catalog_sha256')!=source['fruit_catalog_sha256']:
        raise ValueError('Static fruit eligibility and catalog mismatch')
    if meta.get('geometric_harvest',{}).get('mode')==MODE:policy['harvest_mode']=MODE
    ideal=meta.get('geometric_harvest',{}).get('mode')==MODE
    if ideal:
        low,high=volume_bounds(meta['geometric_harvest'])
        policy['tool_height_interval_m']=[float(low[2]),float(high[2])]
    selector=CameraRegionSelector(policy,lambda *_:np.array([-1.,0.,0.]),
        lambda *_:observation,plant_source=manifest['plant']['source_plant'],selected_truss=target,
        depth_m=.015 if ideal else .035,height_offset_m=-.01,output=output,yaw_offset_deg=yaw_offset_deg,
        entry_forward_world=entry_forward_world,max_entry_yaw_deg=MAX_ENTRY_YAW_DEG)
    group=selector(fruits,local_breaks,paths,fruit_poses,attempts,max_attempts=2)
    return group,selector.last_decision


def start_profiles(assets,requested='auto'):
    """Resolve authored zero-time modes; never edit a running physics state."""
    assets=Path(assets).resolve(strict=True);config=read(assets/'patrol_assets.json')
    profiles={mode:Path(path).resolve(strict=True) for mode,path in config.get('start_modes',{}).items()}
    if profiles:
        if set(profiles)!={'up','down'}:raise ValueError('Start profiles must contain up and down')
        default=config.get('default_start_mode','up')
        for mode,path in profiles.items():
            if read(path/'patrol_assets.json').get('elbow_posture')!=mode:
                raise ValueError('Start profile and entry posture do not match')
    else:
        default=config.get('elbow_posture')
        profiles={default:assets}
    selected=default if requested=='auto' else requested
    if selected not in profiles:raise ValueError('Requested elbow mode has no prepared start profile')
    return profiles,selected


def route_end(config,requested):
    prepared=float(config['end_station_m'])
    end=prepared if requested is None else float(requested)
    if not np.isfinite(prepared) or prepared<=0:
        raise ValueError('Prepared route must have a positive finite end station')
    if not np.isfinite(end) or not 0<end<=prepared:
        raise ValueError(
            f'Requested --end-station {end:g}m exceeds the usable route 0 < end <= {prepared:g}m. '
            f'Use --end-station {prepared:g} (or omit it), or select extended ASSETS. '
            'To extend existing farm instances: prepare-route REFERENCE_ASSETS OUTPUT --end-station METRES.')
    return end


def preflight_patrol(args):
    """Reject incompatible route options before loading geometry or waiting for START."""
    profiles,selected=start_profiles(args.assets,getattr(args,'elbow_mode','auto'))
    # The GUI permits either posture, even if --elbow-mode specified its default.
    choices=profiles if not args.headless else {selected:profiles[selected]}
    return {mode:dict(assets=str(path),end_station_m=route_end(read(path/'patrol_assets.json'),args.end_station))
            for mode,path in choices.items()}


def select_start_assets(args,out):
    profiles,selected=start_profiles(args.assets,getattr(args,'elbow_mode','auto'))
    viewer=None;wait_s=0.
    if not args.headless and set(profiles)=={'up','down'}:
        from newton_patrol_viewer import PatrolViewer
        preview_models={};preview_states={}
        for mode,path in profiles.items():
            config=read(path/'patrol_assets.json');common=(path/'inputs').resolve()
            proof=read(common/'inputs.json');verify_manifest_files(common,proof['input_files'])
            model=mj.MjModel.from_binary_path(str(common/'input_model.mjb'))
            initial=dict(np.load(common/'initial_state.npz',allow_pickle=False))
            if float(initial['time'][0])!=0:raise ValueError('Start selection requires fresh zero-time states')
            preview_states[mode]=poses_from_bank(model,canonical_state(model,initial))
            packed=stream_manifest(path,getattr(args,'scene_loading','auto'))
            if packed:
                section=read(packed)['chunks'][0]
                preview_models[mode]=PackedScene(packed.parent/section['geometry'],section['geometry_sha256'])
            else:
                visual=path/'model/scene.xml'
                if sha256(visual)!=read(path/'model/export.json')['model_sha256']:
                    raise ValueError('Start preview visual SHA mismatch')
                preview_models[mode]=mj.MjModel.from_xml_path(str(visual))
        viewer=PatrolViewer(preview_models[selected],port=args.port,geometric=config.get('harvest_mode')==MODE,
            entry_yaw_limit_deg=MAX_ENTRY_YAW_DEG,elbow_mode=selected,
            harvest_volume=read(profiles[selected]/'patrol_assets.json').get('geometric_harvest'))
        current=[selected]
        def preview(mode):
            if mode!=current[0]:viewer.replace_robot_model(preview_models[mode]);current[0]=mode
            viewer.update(preview_states[mode])
            limit=route_end(read(profiles[mode]/'patrol_assets.json'),args.end_station)
            viewer.status.content=(f'**시작 자세: {viewer.mode_labels[mode]}**\n\n'
                f'순회 구간: 입구 → {limit:g}m\n\n자세를 선택한 뒤 순회 시작을 누르세요.')
        started=time.perf_counter()
        selected=viewer.wait_start(preview);wait_s=time.perf_counter()-started
        viewer.status.content=f'**{viewer.mode_labels[selected]} · 카메라 준비 중**\n\n준비가 끝나면 순회를 시작합니다.'
    path=profiles[selected]
    write(out/'start_selection.json',dict(requested=getattr(args,'elbow_mode','auto'),selected=selected,
        assets=str(path),available_modes=[m for m in profiles if m],selection_wait_s=wait_s,
        initial_state_sha256=sha256(path/'inputs/initial_state.npz'),
        physical_model_sha256=sha256(path/'inputs/input_model.mjb'),runtime_switch_allowed=False))
    return path,viewer,wait_s


def campaign(args):
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    write(out/'launch.json',dict(options={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}))
    try:
        write(out/'preflight.json',preflight_patrol(args))
    except Exception as error:
        write(out/'summary.json',dict(status='error',stage='configuration',error=str(error),
            physics_started=False,assets=str(args.assets),requested_end_station_m=str(args.end_station)))
        raise
    try:
        return run_campaign(args,out)
    except Exception as error:
        # Early model/camera failures used to leave only start_selection.json.
        # Preserve the full-run report when the physics loop already wrote one.
        import traceback
        write(out/'error.json',dict(error=repr(error),traceback=traceback.format_exc()))
        if not (out/'summary.json').exists():
            write(out/'summary.json',dict(status='error',stage='startup',error=repr(error)))
        raise


def run_campaign(args,out):
    launch_start=time.perf_counter()
    assets,viewer,selection_wait_s=select_start_assets(args,out)
    config=read(assets/'patrol_assets.json');common=(assets/'inputs').resolve()
    ideal=config.get('harvest_mode')==MODE
    elbow_posture=config.get('elbow_posture')
    proof=read(common/'inputs.json');verify_manifest_files(common,proof['input_files'])
    meta=read(common/'source_export.json');meta['released_gutter_diagnostic']=True
    model=mj.MjModel.from_binary_path(str(common/'input_model.mjb'))
    static_plants=ideal and meta.get('active_window',{}).get('all_plants_static') is True
    verification_context=VerificationContext(scope='patrol') if static_plants else None
    if static_plants and model.nv != len(np.load(common/'initial_state.npz')['robot_dof_addresses']):
        raise ValueError('Static-plant model must integrate only the original robot coordinates')
    initial=dict(np.load(common/'initial_state.npz',allow_pickle=False));bank=canonical_state(model,initial)
    sources={int(s['plant_id']):s for s in config['plants']}
    first=next(iter(sources.values()));active=first['plant_id']
    calibration=read(first['camera_calibration'])
    end=route_end(config,args.end_station)
    patrol=CameraPatrol(read(assets/'inventory.json'),config['bed_gutter_center_x'],step_m=args.step,
        station_limits_m=(0.,end),minimum_fruits=2,
        camera_foreaft_m=calibration['camera_foreaft_m'],camera_to_grasp_height_m=calibration['camera_to_grasp_height_m'])
    qadr=initial['robot_qpos_addresses'];q0=bank['qpos'][qadr]
    reference=mj.MjData(model);reference.qpos[:]=bank['qpos'];mj.mj_kinematics(model,reference)
    entry_forward=reference.xmat[model.body('Tool_fixed').id].reshape(3,3)[:,1].copy()
    entry_forward[2]=0.;entry_forward/=np.linalg.norm(entry_forward)
    if abs(q0[0])>1e-9 or config['start_station_m']!=0:raise ValueError('Patrol must start at the authored rail entrance')
    from newton_mounted_camera import MountedCamera
    packed=stream_manifest(assets,getattr(args,'scene_loading','auto'))
    if packed:
        from newton_scene_stream import StreamingCamera
        stream=read(packed)
        camera_body=proof['source_body_to_native_name'][stream['mount']['body_path']]
        if not args.headless and viewer is None:
            from newton_patrol_viewer import PatrolViewer
            section=stream['chunks'][0]
            visual=PackedScene(packed.parent/section['geometry'],section['geometry_sha256'])
            viewer=PatrolViewer(visual,port=args.port,geometric=ideal,entry_yaw_limit_deg=MAX_ENTRY_YAW_DEG,
                                harvest_volume=config.get('geometric_harvest'))
            viewer.start.disabled=True
            viewer.update(poses_from_bank(model,bank))
        print('Preparing current/next camera sections; subsequent preloads run in background.',flush=True)
        camera=StreamingCamera(assets,packed,poses_from_bank(model,bank),camera_body,
            scale=args.camera_scale,output=out/'scene_stream',viewer=viewer,
            reuse_identical=not getattr(args,'scene_reload_identical',False))
    else:
        print('Loading full-farm camera geometry once; physics starts after preparation.',flush=True)
        camera=MountedCamera(assets,scale=args.camera_scale)
    cold_preparation_wall_s=time.perf_counter()-launch_start-selection_wait_s
    camera_body=proof['source_body_to_native_name'][camera.mount['body_path']]
    if not args.headless:
        from newton_patrol_viewer import PatrolViewer
        if viewer is None:viewer=PatrolViewer(camera.visual,port=args.port,geometric=ideal,entry_yaw_limit_deg=MAX_ENTRY_YAW_DEG,
            harvest_volume=config.get('geometric_harvest'))
        ready_poses=poses_from_bank(model,bank);viewer.update(ready_poses)
        _,images=camera.capture(ready_poses,camera_body,float(bank['time'][0]),station=float(q0[0]))
        viewer.observation(images,0.,0)
        if not viewer.event.is_set():
            viewer.start.disabled=False
            viewer.wait_start()
    parent=None;rows=[];observations=[];attempts={p:{} for p in sources};break_names=set();harvests=0;started_harvests=0
    collected=set();capture_events=[]
    preferred_connector=(0.,'joint_branch') if ideal else None
    planning_rounds={}
    from newton_gpu_session import NewtonGPUContinuationCache
    gpu_cache=NewtonGPUContinuationCache()
    planner=PlannerProcess(args.planning_python,out/'planning_worker.log')
    next_plan=scan_plan(q0,ready_q=calibration['ready_q']);decision=dict(action='initial_scan_pose')
    fluid=ideal and getattr(args,'motion_profile','fluid')=='fluid'
    if fluid:next_plan=fluid_scan(next_plan)
    start=time.perf_counter();physics_index=0
    report=dict(schema='s300-newton-camera-patrol-v1',status='running',assets=str(assets),
        actual_gpu_physics=True,actual_gpu_camera=True,start_station_m=0.,end_station_m=end,
        maximum_forward_step_m=args.step,rows=rows,observations=observations,training_eligible=False,harvest_success=None)
    report.update(harvest_mode=MODE if ideal else 'physical_detachment',
        assumed_harvest_count=0, assumed_harvest_names=[], capture_events=capture_events,
        static_plants=static_plants, model_counts=dict(nq=model.nq,nv=model.nv,nbody=model.nbody,ngeom=model.ngeom))
    report.update(cold_preparation_wall_s=cold_preparation_wall_s,camera_setup_wall_s=camera.setup_wall_s,
                  proposals_per_view=args.proposals_per_view)
    report['entry_angle_policy']=dict(reference_forward_world=entry_forward.tolist(),maximum_yaw_deg=MAX_ENTRY_YAW_DEG,
        candidate_yaws_deg=list(ENTRY_YAW_CANDIDATES),reference='Fixed authored robot ready Tool_fixed +Y in world XY')
    report['elbow_posture']=elbow_posture or 'unconstrained'
    report['selection_wait_s']=selection_wait_s
    report['motion_profile']='fluid' if fluid else 'legacy'
    write(out/'invocation.json',dict(options={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        source_code_sha256={n:sha256(HERE/n) for n in ('run_newton_camera_patrol.py','newton_mounted_camera.py',
            'camera_patrol.py','camera_region_policy.py','harvest_group_policy.py','newton_geometric_harvest.py',
            'newton_kernels.py','newton_contact_policy.py','newton_patrol_viewer.py','newton_patrol_motion.py',
            'newton_scene_chunks.py','newton_scene_stream.py')}))
    try:
        for cycle in range(args.max_decisions):
            if viewer and viewer.stop_event.is_set():report['status']='user_stopped';break
            step=out/f'step_{cycle:03d}';step.mkdir();write(step/'decision.json',decision)
            if next_plan is not None:
                if decision['action']=='harvest':started_harvests+=1
                write(step/'plan.json',next_plan)
                preparation_start=time.perf_counter()
                segment_inputs(common,step/('inputs' if static_plants else 'canonical_inputs'),next_plan,bank,active,physics_index,
                               verification_context=verification_context,source_metadata=meta)
                if not static_plants:
                    active_prepare(step/'canonical_inputs',step/'inputs',active,bank_state=bank)
                    shared_bytes=share_compiled_assets(step/'inputs',out/'shared_models')
                else:shared_bytes=0
                transition=create_transition(step/'inputs',parent,common) if parent and not static_plants else None
                input_preparation_wall_s=time.perf_counter()-preparation_start
                command=[str(ROOT/'nvidia-sim/run_s300_newton.sh'),'gpu-realtime',str(step/'inputs'),
                    '--output',str(step/'physics'),'--nconmax',str(args.nconmax),'--njmax',str(args.njmax),
                    '--dense-hessian-capacity','128','--dense-hessian-overflow-capacity','512',
                    '--solver-tolerance','1e-8','--released-gutter-diagnostic',
                    '--viewer-backend','viser','--recording','full']
                if viewer:command+=['--publish-live','--viewer-fps','30','--presentation-rate',str(args.display_rate)]
                if transition:command+=['--window-transition',str(transition)]
                if parent and static_plants:command+=['--resume-state',str(parent)]
                if static_plants:
                    # The old 355-DOF blocked cuSOLVER override is unsuitable
                    # for a ten-axis system. Use Newton/MuJoCo Warp's native
                    # small GPU factorization; integration remains on CUDA.
                    command+=['--linear-algebra','warp','--inertia-linear-algebra','warp']
                    command+=['--batch',str(args.graph_batch)]
                write(step/'execution_command.json',command)
                print('GPU',cycle,decision['action'],'plant',active,flush=True)
                execution_start=time.perf_counter()
                rolling_callback=None
                if next_plan.get('rolling_camera_required'):
                    if args.fresh_process:raise ValueError('Moving camera requires the resident GPU executor')
                    last_rolling_station=[float(bank['qpos'][qadr[0]])];last_check=[-1.];rolling_count=[0]
                    def rolling_callback(data,tick,elapsed):
                        if elapsed-last_check[0]<.05:return None
                        last_check[0]=elapsed
                        actual_q=data.qpos.numpy()[0]
                        actual_station=float(actual_q[qadr[0]])
                        if actual_station-last_rolling_station[0]<args.step-.005:return None
                        stamp=float(data.time.numpy()[0]);rolling_count[0]+=1
                        folder=step/'rolling_camera'/f'frame_{rolling_count[0]:03d}'
                        actual_poses=poses_from_bank(model,dict(qpos=actual_q))
                        rolling_obs,rolling_images=camera.capture(sensor_poses(actual_poses,collected),camera_body,stamp,output=folder,station=actual_station)
                        rolling_obs['observation_id']=str(folder/'observation.json')
                        ingested=patrol.observe(rolling_obs,current_station_y_m=actual_station,
                            current_lift_m=float(actual_q[qadr[1]]),native_detached_keys=detached+assumed_keys)
                        probe=patrol.decide(current_station_y_m=actual_station,
                            active_source_plant=f'/World/Greenhouse/Plants/Plant_{active:02d}')
                        write(folder/'decision.json',probe)
                        distance=actual_station-last_rolling_station[0]
                        if distance>.2:raise RuntimeError('Moving-camera observation spacing exceeded20cm')
                        observations.append(dict(index=cycle,in_motion=True,station_m=actual_station,
                            physics_clock_s=stamp,campaign_wall_s=time.perf_counter()-start,
                            visible_red_fruits=len(ingested['accepted']),queued=ingested['queued'],
                            camera=str(folder),sensor_wall_s=rolling_obs['render_wall_s'],
                            distance_since_observation_m=distance,
                            actual_rail_velocity_m_s=float(data.qvel.numpy()[0,initial['robot_dof_addresses'][0]])))
                        last_rolling_station[0]=actual_station
                        if viewer:viewer.observation(rolling_images,actual_station,harvests,len(collected))
                        print('ROLLING OBS',cycle,round(actual_station,3),probe['action'],flush=True)
                        if probe['action']!='move' or probe['next_station_y_m']-actual_station<args.step-.01:
                            return dict(observation=str(folder/'observation.json'),decision=probe,
                                        actual_station_m=actual_station,solver_step=tick)
                        return None
                if not args.fresh_process:
                    from run_newton_harvest import arguments,run
                    import contextlib,threading
                    options=subprocess.run(command,cwd=ROOT,env=dict(os.environ,S300_NEWTON_PRINT_ARGS='1'),
                        capture_output=True,text=True,check=True)
                    runner_args=arguments(json.loads(options.stdout))
                    done=threading.Event();display_errors=[]
                    def poll_display():
                        sent=False
                        while not done.wait(1/60):
                            try:
                                if viewer:viewer.poll(step/'physics')
                                if viewer and viewer.stop_event.is_set() and not sent:
                                    import signal
                                    os.kill(os.getpid(),signal.SIGINT);sent=True
                            except (FileNotFoundError,json.JSONDecodeError):pass
                            except Exception as error:display_errors.append(repr(error));return
                    thread=threading.Thread(target=poll_display,daemon=True);thread.start()
                    try:
                        with (step/'physics.log').open('w') as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
                            run(runner_args,session_cache=gpu_cache,verification_context=verification_context,
                                progress_callback=rolling_callback)
                    finally:done.set();thread.join(timeout=2.)
                    if display_errors:raise RuntimeError('Independent patrol display: '+str(display_errors))
                    returncode=0
                else:
                    with (step/'physics.log').open('w') as log:
                        process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                        stop_sent=False
                        while process.poll() is None:
                            if viewer:viewer.poll(step/'physics')
                            if viewer and viewer.stop_event.is_set() and not stop_sent:
                                import signal
                                process.send_signal(signal.SIGINT);stop_sent=True
                            time.sleep(.05)
                        returncode=process.returncode
                if viewer:viewer.poll(step/'physics')
                summary=read(step/'physics/summary.json')
                row=dict(index=cycle,action=decision['action'],target=decision.get('target_key'),
                    physics=str(step/'physics'),status=summary['status'],stop=summary.get('stop'),
                    simulated_s=summary.get('simulated_s'),realtime_factor=summary.get('realtime_factor'),
                    orchestration_wall_s=time.perf_counter()-execution_start,
                    input_preparation_wall_s=input_preparation_wall_s,
                    duplicate_model_bytes_avoided=shared_bytes,
                    solver_capacity=summary.get('solver_capacity'),
                    new_detachments=summary.get('breaks',[]),
                    first_active_violations=summary.get('first_active_violations',[]))
                rows.append(row)
                report['terminal_physics_clock_s']=summary.get('final_physics_clock_s')
                if returncode or summary['status']!='completed':
                    report.update(status='physics_stopped',stop=row,
                        releases_observed_in_stopped_segment=summary.get('breaks',[]),
                        stopped_segment_releases_counted_as_success=False);break
                parent=step/'physics'
                if static_plants:
                    from newton_contact_policy import validate_config
                    _,terminal,history,lineage=load_resume(parent,common,initial,meta,
                        policy=validate_config(read(HERE/'profiles/belt_strokes_2mm.json')),
                        verification_context=verification_context)
                    bank=canonical_state(model,initial,terminal)
                    write(step/'continuation_check.json',lineage)
                else:
                    bank,_,history,_,_,_=checked_window_parent(parent,common,initial,meta,
                        HERE/'profiles/belt_strokes_2mm.json',physics_index)
                physics_index+=1;break_names={r['name'] for r in history}
                if decision['action']=='harvest':
                    actual_angles=recorded_entry_yaw(step/'physics/states.npz',next_plan,
                        proof['source_body_to_native_name'],entry_forward,elbow_posture=elbow_posture)
                    write(step/'actual_entry_angle.json',actual_angles);row['entry_angle']=actual_angles
                    if not actual_angles['passed']:
                        report.update(status='entry_angle_stopped',reason='Actual entry violated yaw or required elbow-up posture',stop=row)
                        break
                    harvests+=1
                    for f in decision['group']['fruits']:attempts[active][f]=attempts[active].get(f,0)+1
                    if ideal:
                        segment_model=model if static_plants else mj.MjModel.from_binary_path(str(step/'inputs/input_model.mjb'))
                        segment_initial=dict(np.load(step/'inputs/initial_state.npz',allow_pickle=False))
                        receipt=capture_receipt(segment_model,segment_initial,next_plan,
                            step/'physics/states.npz',meta.get('geometric_fruit_names',[a['name'] for a in meta['attachments']]),collected,
                            volume=meta.get('geometric_harvest'))
                        write(step/'geometric_capture.json',receipt)
                        capture_events.append(dict(step=cycle,**receipt))
                        collected.update(r['name'] for r in receipt['harvested'])
                        print('CAPTURE',cycle,'reached',receipt['reached'],'new',len(receipt['harvested']),
                              'total',len(collected),flush=True)
                        if viewer:viewer.mark_harvested(collected)
            measured=bank['qpos'][qadr];station=float(measured[0]);poses=poses_from_bank(model,bank)
            if viewer:viewer.update(poses)
            obs,images=camera.capture(sensor_poses(poses,collected),camera_body,float(bank['time'][0]),output=step/'camera',station=station)
            obs['observation_id']=str(step/'camera/observation.json')
            if viewer:viewer.observation(images,station,harvests,len(collected))
            detached=[];assumed_keys=[]
            for s in sources.values():
                for f in source_fruit_manifest(s)['plant']['fruits']:
                    key=f"/World/Greenhouse/Plants/Plant_{s['plant_id']:02d}/"+f['name'].replace('_Fruit_','/Fruit_')
                    if s['prefix']+f['name'] in break_names:detached.append(key)
                    if s['prefix']+f['name'] in collected:assumed_keys.append(key)
            ingestion=patrol.observe(obs,current_station_y_m=station,current_lift_m=float(measured[1]),native_detached_keys=detached+assumed_keys)
            observations.append(dict(index=cycle,station_m=station,physics_clock_s=float(bank['time'][0]),
                campaign_wall_s=time.perf_counter()-start,
                visible_red_fruits=len(ingestion['accepted']),queued=ingestion['queued'],camera=str(step/'camera'),
                sensor_wall_s=obs['render_wall_s']))
            reacquisition=record_scan_reacquisition(patrol,decision,obs,station)
            if reacquisition is not None:observations[-1]['camera_reacquisition']=reacquisition
            print('OBS',cycle,'station',round(station,3),'visible red',len(ingestion['accepted']),'queue',ingestion['queued'],flush=True)
            report.update(last_station_m=station,harvest_attempts=started_harvests,completed_harvest_motions=harvests,detached_fruit_keys=detached,
                          assumed_harvest_count=len(collected),assumed_harvest_names=sorted(collected),
                          assumed_harvest_keys=assumed_keys,
                          physics_clock_s=float(bank['time'][0]),last_valid_state_clock_s=float(bank['time'][0]),
                          wall_s=time.perf_counter()-start)
            write(out/'patrol.json',patrol.to_dict());write(out/'summary.json',report)
            if harvests>=args.max_harvests:
                report['status']='harvest_attempt_budget_exhausted';break
            decision=patrol.decide(current_station_y_m=station,active_source_plant=f'/World/Greenhouse/Plants/Plant_{active:02d}')
            next_plan=None
            rail_velocity=float(bank['qvel'][initial['robot_dof_addresses'][0]])
            if fluid and decision['action'] in ('harvest','scan') and rail_velocity>.005:
                # The mounted observation may find a target while cruising.
                # Brake physically, then reacquire at the measured stopped pose.
                destination=min(end,station+max(.03,rail_velocity**2/.3))
                next_plan=fluid_scan(scan_plan(measured,destination=destination),rail_velocity=rail_velocity)
                decision=dict(action='brake_for_observation',reason='Observed target during forward cruise',
                              target_key=decision.get('target_key'),next_station_y_m=destination)
                continue
            if decision['action']=='stop':
                reason=decision['reason']
                if reason=='physical_rail_end_reached' and end<5:reason='configured_route_budget_end_reached'
                report.update(status='stopped',reason=reason);break
            if decision['action'] in ('move','scan'):
                next_plan=scan_plan(measured,destination=decision['next_station_y_m'],
                    ready_q=calibration['ready_q'] if decision['action']=='scan' else None)
                if fluid:next_plan=fluid_scan(next_plan,rail_velocity=rail_velocity,
                    cruise=decision['action']=='move' and not decision.get('target_key') and
                           decision['next_station_y_m']<end-.03)
                if fluid and not args.fresh_process and decision['action']=='move' and not decision.get('target_key'):
                    next_plan=cruise_route(measured,end,step_m=args.step,rail_velocity=rail_velocity)
            elif decision['action']=='harvest':
                plant=decision['target']['plant_id'];target=decision['target']['truss_key']
                if plant not in sources:
                    patrol.record_blocker(decision['target_key'],'outside_prepared_route_budget',dict(plant_id=plant,prepared_plants=list(sources)))
                    continue
                headings=[];evidence=None
                for yaw in (ENTRY_YAW_CANDIDATES if ideal else (0.,)):
                    group,evidence=select_group(obs,sources[plant],meta,poses,break_names|collected,attempts[plant],
                        target,step/f'group_selection_yaw_{yaw:g}.json',yaw_offset_deg=yaw,entry_forward_world=entry_forward)
                    if group is not None:headings.append((yaw,group))
                if not headings:
                    patrol.record_local_failure(decision['target_key'],'visible_group_footprint_rejected',evidence,
                        current_station_y_m=station,max_forward_views=2)
                    continue
                proposals=[];proposed=None
                # Existing planner connectors only; finite alternatives when
                # a vertical-first connector traps the current IK branch.
                connectors=[('level_then_translate',dict(preparation_height_first=True)),
                            ('combined_cartesian',dict(preparation_height_first=False)),
                            ('joint_branch',dict(horizontal_ready_connector='joint',horizontal_ready_normalize_turns=True))]
                alternatives=ordered_proposals(headings,connectors,preferred_connector,
                    planning_rounds.get(decision['target_key'],0) if static_plants else 0)
                # Reuse the most recently collision-screened direction first.
                # Initial lateral preference comes from the preserved baseline;
                # every alternative remains available with identical checks.
                proposal_budget=args.proposals_per_view if static_plants else len(alternatives)
                for proposal_index,(yaw,group,name,options) in enumerate(alternatives):
                    if proposal_index>=proposal_budget:break
                    connector_name=name
                    if viewer:
                        viewer.status.content=f'**경로 확인 중**\n\n송이 {target} · 방향 {yaw:g}° · 연결 {name}'
                    request=dict(measured_joint_q=measured[1:8].tolist(),measured_jaw_q=measured[8:].tolist(),
                        measured_station_y_m=max(0.,min(5.,station)),rail_origin_station_m=0.,
                        target_center=group['aim_world_m'],direction=group['group_selection']['direction_world'],
                        ready_lift_offset_m=0.,grasp_depth_m=group['group_selection']['depth_m'],grasp_height_offset_m=-.01,settle_seconds=.05,
                        horizontal_ready_connector='cartesian')
                    if elbow_posture:request['elbow_posture']=elbow_posture
                    if static_plants:request['ik_solver']='analytic'
                    if ideal:name=f'yaw_{yaw:g}_{name}'
                    attempt=step/name;attempt.mkdir()
                    req=dict(request,**options);write(attempt/'request.json',req)
                    planned,planning_wall=planner.plan(sources[plant]['assembly'],attempt/'request.json',
                        attempt/'plan.json',attempt/'planning.log')
                    record=dict(name=name,yaw_offset_deg=yaw,directory=str(attempt),planned=planned,
                                planning_wall_s=planning_wall)
                    proposals.append(record)
                    if not planned:
                        record['failure_kind'],record['failure_message']=planning_failure(attempt/'planning.log')
                        print('PLAN',cycle,name,'IK/connector not found; trying next bounded proposal',flush=True)
                        continue
                    candidate=read(attempt/'plan.json')
                    if ideal:candidate=geometric_plan(candidate)
                    if fluid:candidate=fluid_plan(candidate)
                    audit_start=time.perf_counter()
                    if static_plants:
                        angles,screen=audit_static_plan(model,bank,qadr,candidate,entry_forward,elbow_posture=elbow_posture)
                    else:
                        angles=planned_entry_yaw(model,bank,qadr,candidate,entry_forward,elbow_posture=elbow_posture)
                        selected,owned=_window_selection(model,meta,plant)
                        screen=screen_preparation(model,bank,qadr,candidate,
                            active_body_names=set() if ideal else owned[selected]|break_names,entire_path=ideal)
                    write(attempt/'planned_entry_angle.json',angles);record['entry_angle']=angles
                    write(attempt/'preparation_screen.json',screen);record['screen']=screen
                    record['audit_wall_s']=time.perf_counter()-audit_start
                    record['planning_and_audit_wall_s']=planning_wall+record['audit_wall_s']
                    if angles.get('complete',True) and not angles['passed']:
                        record['failure_kind']='entry_angle_limit'
                        print('PLAN',cycle,name,'outside robot forward +/-15deg',flush=True)
                        continue
                    if screen['passed']:
                        proposed=candidate;preferred_connector=(yaw,connector_name)
                        write(step/'next_plan.json',candidate);break
                    print('PLAN',cycle,name,'rigid intersection',screen['bodies'],flush=True)
                    record['failure_kind']='obstacle_collision'
                write(step/'planning_proposals.json',proposals)
                write(step/'planning_budget.json',dict(available=len(alternatives),evaluated=len(proposals),
                    per_view_limit=proposal_budget,untested=len(alternatives)-len(proposals),
                    purpose='Bound planning pause per camera view, then retain target and obtain next 5cm view'))
                if proposed is None:
                    planning_rounds[decision['target_key']]=planning_rounds.get(decision['target_key'],0)+1
                    reason='view_planning_budget_exhausted' if len(proposals)<len(alternatives) else 'finite_preparation_proposals_exhausted'
                    patrol.record_local_failure(decision['target_key'],reason,
                        dict(proposals=proposals),current_station_y_m=station,max_forward_views=2)
                    continue
                planning_rounds[decision['target_key']]=0
                next_plan=proposed;active=plant;decision['group']=group
                write(step/'proposed_next_decision.json',decision)
        else:report['status']='decision_budget_exhausted'
    except BaseException as error:
        report.update(status='error',error=repr(error))
        print('Patrol stopped by error: '+repr(error),file=sys.stderr,flush=True)
        raise
    finally:
        planner.close()
        gpu_cache.close()
        report.update(harvest_attempts=started_harvests,completed_harvest_motions=harvests)
        report['wall_s']=time.perf_counter()-start
        if packed:
            report['scene_stream']=camera.evidence()
            write(out/'scene_stream.json',report['scene_stream'])
            camera.close()
        report['end_to_end_wall_s']=report['wall_s']+cold_preparation_wall_s
        if viewer:
            write(out/'display_frame_timing.json',viewer.frame_timing)
            report['display_rate']=args.display_rate
            report['display_target_fps']=30
        if verification_context is not None:report['verification_scope']=verification_context.evidence()
        write(out/'summary.json',report);write(out/'patrol.json',patrol.to_dict())
        if viewer:viewer.finish(report)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('assets',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--headless',action='store_true');p.add_argument('--port',type=int,default=5905)
    p.add_argument('--display-rate',type=float,default=1.,help='Live display speed; default1x, 0 maximum throughput')
    p.add_argument('--elbow-mode',choices=['auto','up','down'],default='auto',help='Start posture; GUI permits selection before START, headless uses the specified profile')
    p.add_argument('--fresh-process',action='store_true',help='Comparison only: reload GPU physics for each segment')
    p.add_argument('--motion-profile',choices=['fluid','legacy'],default='fluid',
                   help='Geometric patrol timing: distance-based movement without repeated close/open or fixed scan waits')
    p.add_argument('--step',type=float,default=.15);p.add_argument('--end-station',type=float)
    p.add_argument('--max-decisions',type=int,default=100);p.add_argument('--max-harvests',type=int,default=6)
    p.add_argument('--camera-scale',type=float,default=.5,help='Default validated patrol sensor: 240x320; 1.0 uses mounted 480x640')
    p.add_argument('--scene-loading',choices=['auto','stream','full'],default='auto',
                   help='Use prepared scene_stream sections when available; full is the comparison baseline')
    p.add_argument('--scene-reload-identical',action='store_true',
                   help='Diagnostic: exercise actual background uploads even if adjacent section contents are identical')
    p.add_argument('--graph-batch',type=int,default=2,
                   help='Two steps allow30Hz display even during60Hz rail motion; keep GUI/headless the same')
    # Broadphase candidate pairs share the contact buffer even when narrowphase
    # produces no real contacts. The upright wrist's fifth harvest needs310
    # candidates; the old256 allocation truncated pairs and correctly stopped.
    p.add_argument('--nconmax',type=int,default=1024,help='GPU contact and broadphase candidate capacity per environment')
    p.add_argument('--njmax',type=int,default=1024,help='GPU constraint capacity per environment')
    p.add_argument('--proposals-per-view',type=int,default=3,help='Bound static-mode planning pause, then reobserve after a short forward probe; 15 restores full view search')
    p.add_argument('--planning-python',type=Path,default=ROOT/'nvidia-sim/.asset-venv/bin/python')
    a=p.parse_args()
    if not np.isfinite(a.display_rate) or a.display_rate<0:p.error('Nonnegative finite display rate required')
    if not 0<a.step<=.2 or min(a.max_decisions,a.max_harvests)<1:p.error('Positive budgets and step<=.2m required')
    if a.graph_batch<2 or a.graph_batch%2:p.error('GPU graph batch must be a positive even number')
    if min(a.nconmax,a.njmax)<1:p.error('Positive GPU contact and constraint capacities required')
    if a.proposals_per_view<1:p.error('Positive per-view planning budget required')
    result=campaign(a);print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
