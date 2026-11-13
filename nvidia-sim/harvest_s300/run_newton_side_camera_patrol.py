"""Live V6: real RGB-D observation -> visible group -> native motion -> reobserve."""
import argparse
from contextlib import ExitStack,redirect_stdout,redirect_stderr
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

from camera_patrol import CameraPatrol
from newton_active_window import canonical_state
from newton_continuation import VerificationContext,load_resume
from newton_contact_policy import validate_config
from newton_geometric_harvest import capture_receipt
from newton_inputs import sha256
from newton_scene_chunks import stream_manifest
from newton_scene_stream import StreamingCamera
from plan_adapter import sample_plan
from run_newton_camera_patrol import segment_inputs,select_group,source_fruit_manifest,record_scan_reacquisition
from side_farm import read,write
from side_camera_assets import prepare
from side_camera_scene import SideCameraScene
from side_camera_motion import SideCameraMotion,PlanningError
from side_camera_coverage import SideCoveragePatrol,plan_visible_alternatives

ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).parent


def reuse_stationary_scan(patrol,decision,observation,q,home):
    """The just-rendered measured view already fulfils an identical scan."""
    if (decision.get('action')!='scan' or
            abs(decision.get('next_station_y_m',q[0])-q[0])>1e-6 or
            not np.allclose(q[1:8],home[1:8],rtol=0,atol=.002) or
            not observation.get('physics_state_authoritative')):
        return None
    return record_scan_reacquisition(patrol,decision,observation,float(q[0]))


def campaign(args,out,resources):
    assets=args.assets.resolve();config=read(assets/'patrol_assets.json')
    if config.get('side_camera_patrol',{}).get('profile_sha256')!=sha256(args.ready_profile):
        assets=out/'assets';config=prepare(args.assets,assets,args.ready_profile)
    scene=SideCameraScene(assets);common=assets/'inputs'
    initial=dict(np.load(common/'initial_state.npz',allow_pickle=False));model=scene.native
    bank=canonical_state(model,initial);home=np.array(config['side_camera_patrol']['ready_command'])
    motion=SideCameraMotion(scene,initial,home)
    if not motion.planner.valid_full(bank['qpos']):raise ValueError('Configured V6 camera start rejected: '+str(motion.planner.stats))
    end=float(config['end_station_m']if args.end_station is None else args.end_station)
    if not 0<end<=config['end_station_m']:raise ValueError('Requested camera route exceeds prepared farm')
    viewer=None
    if not args.headless:
        from side_camera_viewer import SideCameraViewer
        viewer=SideCameraViewer(scene,out,port=args.port,end_station=end)
        viewer.update(bank['qpos'],0.,'camera_preparation')
    path=stream_manifest(assets,'stream')
    poses=scene.measured_poses(bank['qpos'])
    print('Preparing V6 GPU RGB-D sensor; subsequent sections prefetch in background',flush=True)
    camera=StreamingCamera(assets,path,poses,'Tool_fixed',scale=args.camera_scale,output=out/'scene_stream',viewer=viewer)
    resources.callback(camera.close)
    calibration=config['side_camera_patrol']
    patrol=SideCoveragePatrol(read(assets/'inventory.json'),config['bed_gutter_center_x'],step_m=args.step,
        station_limits_m=(0.,end),minimum_fruits=2,camera_foreaft_m=calibration['camera_foreaft_m'],
        camera_to_grasp_height_m=calibration['camera_to_grasp_height_m'],visible_service_offset_proposal_m=0.)
    sources={int(s['plant_id']):s for s in config['plants']};active=next(iter(sources))
    for identity in list(patrol.eligible):
        if identity[0]not in sources:
            patrol.eligible.remove(identity);patrol.excluded[identity]='outside_prepared_native_route_budget'
    fruit_keys={}
    for source in sources.values():
        for fruit in source_fruit_manifest(source)['plant']['fruits']:
            fruit_keys[source['prefix']+fruit['name']]=f"/World/Greenhouse/Plants/Plant_{source['plant_id']:02d}/"+fruit['name'].replace('_Fruit_','/Fruit_')
    meta=read(common/'source_export.json');meta['released_gutter_diagnostic']=True
    policy=validate_config(read(HERE/'profiles/belt_strokes_2mm.json'))
    verification=VerificationContext(scope='patrol')
    from newton_gpu_session import NewtonGPUContinuationCache
    from run_newton_harvest import arguments,run
    cache=NewtonGPUContinuationCache();resources.callback(cache.close)
    command=[str(ROOT/'nvidia-sim/run_s300_newton.sh'),'gpu-realtime',str(common),'--output',str(out/'unused_template'),
        '--nconmax','2048','--njmax','8192','--dense-hessian-capacity','128','--dense-hessian-overflow-capacity','512',
        '--solver-tolerance','1e-8','--released-gutter-diagnostic','--linear-algebra','warp',
        '--inertia-linear-algebra','warp','--batch','2','--recording','full']
    options=subprocess.run(command,env=dict(os.environ,S300_NEWTON_PRINT_ARGS='1'),cwd=ROOT,capture_output=True,text=True,check=True)
    template=arguments(json.loads(options.stdout))
    rows=[];observations=[];events=[];performance=[];reused_scans=[];latency=[];pending_latency=None
    collected=set();attempts={p:{}for p in sources};harvests=0;parent=None;physics_index=0
    report=dict(schema='s300-v6-live-camera-patrol-v1',status='ready',assets=str(assets),actual_gpu_physics=True,
        actual_gpu_camera=True,recorded_replay=False,uses_authored_fruit_target_centres=False,
        perception='Existing visible SIM-GT labels and optical-Z depth; no learned detector',
        harvest_mode='main_stem_only_geometric_harvest',physical_fruit_detachment=False,
        start_station_m=0.,end_station_m=end,maximum_observation_step_m=args.step,
        grip_cycles_per_target=2,rows=rows,observations=observations,capture_events=events,
        completed_harvest_motions=0,assumed_harvest_count=0,training_eligible=False,harvest_success=None,
        source_model_mjb_sha256=sha256(common/'input_model.mjb'),sensor_geometry=scene.provenance,
        performance=performance,camera_to_motion_latency=latency,reused_stationary_scans=reused_scans,entry_policy='level only; no tilted IK alternatives',
        visible_changed_view_retries=patrol.reopened_views,single_visible_fruit_cleanup=True,
        maximum_distinct_groups_per_view=args.proposals_per_view)
    write(out/'summary.json',report)
    if viewer:
        obs,images=camera.capture(poses,'Tool_fixed',0.,output=out/'initial_camera',station=0.)
        obs.update(initial_preview_only=True,pose_source='Declared fresh native initial state; not used for patrol decisions')
        write(out/'initial_camera/observation.json',obs)
        viewer.observation(images,0.,0,0)
    started=time.perf_counter();report['status']='running'
    decision=dict(action='initial_scan');plan=motion.scan(bank['qpos'])

    def expose(folder,q,stamp,*,in_motion=False):
        exposure_started=time.perf_counter()
        observation,images=camera.capture(scene.measured_poses(q,collected),'Tool_fixed',stamp,
            output=folder,station=float(q[0]))
        camera_returned=time.perf_counter()
        observation.update(observation_id=str(folder/'observation.json'),physics_state_authoritative=True)
        write(folder/'observation.json',observation)
        if observation.get('uses_hidden_gt_centers')is not False:raise ValueError('Sensor must not project hidden target centres')
        assumed=[fruit_keys[n]for n in collected]
        ingested=patrol.observe(observation,current_station_y_m=float(q[0]),current_lift_m=float(q[1]),native_detached_keys=assumed)
        stationary=not in_motion and abs(float(bank['qvel'][initial['robot_dof_addresses'][0]]))<=.005
        reopened=patrol.reconsider_visible(float(q[0]))if stationary else []
        observations.append(dict(camera=str(folder),physics_clock_s=stamp,station_m=float(q[0]),in_motion=in_motion,
            visible_red_fruits=len(ingested['accepted']),queued=ingested['queued'],reopened=reopened,render_wall_s=observation['render_wall_s'],
            capture_rpc_wall_s=camera_returned-exposure_started,camera_returned_monotonic_s=camera_returned))
        coverage=dict(visible_ripe_fruits=len(ingested['accepted']),
            visible_pending_trusses=sum(item.get('visible_now',False)and item['status']in('pending','revisit_required')for item in patrol.queue.values()),
            visible_deferred_trusses=sum(item.get('visible_now',False)and item['status']=='finite_blocked'for item in patrol.queue.values()),
            changed_view_retries=len(patrol.reopened_views))
        report['visible_coverage']=coverage
        if viewer:viewer.observation(images,float(q[0]),harvests,len(collected),coverage=coverage)
        return observation,ingested

    try:
        for cycle in range(args.max_decisions):
            if viewer and cycle>0 and not viewer.gate():report['status']='user_stopped';break
            step=out/f'step_{cycle:03d}';step.mkdir();write(step/'decision.json',decision)
            if plan is not None:
                write(step/'plan.json',plan);stage_started=time.perf_counter()
                segment_inputs(common,step/'inputs',plan,bank,active,physics_index,
                    verification_context=verification,source_metadata=meta)
                if pending_latency is not None:
                    pending_latency['input_preparation_s']=time.perf_counter()-stage_started
                runner=copy.copy(template);runner.inputs=step/'inputs';runner.output=step/'physics';runner.resume_state=parent
                write(step/'execution.json',dict(native_gpu=True,inputs=str(runner.inputs),resume_state=str(parent)if parent else None))
                last_display=[-1.];last_station=[float(bank['qpos'][0])];rolling=[0];presentation=[None]
                def progress(data,tick,elapsed):
                    now=time.monotonic()
                    if viewer:
                        pause_start=time.monotonic()
                        if not viewer.gate():return dict(reason='user_stop',count_as_harvest=False)
                        paused=time.monotonic()-pause_start
                        if presentation[0]is None:presentation[0]=time.monotonic()-elapsed
                        else:presentation[0]+=paused
                        delay=presentation[0]+elapsed-time.monotonic()
                        while delay>.002:
                            time.sleep(min(delay,.03))
                            if viewer.stop_event.is_set():return dict(reason='user_stop',count_as_harvest=False)
                            delay=presentation[0]+elapsed-time.monotonic()
                    measured_q=None
                    if pending_latency is not None and 'first_actual_arm_change_s'not in pending_latency:
                        measured_q=data.qpos.numpy()[0]
                        commanded=sample_plan(plan,elapsed)['command']
                        if (np.max(np.abs(measured_q[2:8]-plan['initial_command'][2:8]))>.002 and
                                np.max(np.abs(commanded[2:8]-plan['initial_command'][2:8]))>.002):
                            pending_latency.update(first_actual_arm_change_s=time.perf_counter()-pending_latency['camera_returned_monotonic_s'],
                                measured_motion_threshold_rad=.002,status='actual_arm_motion_observed')
                    rolling_due=bool(plan.get('rolling_camera_required'))
                    if not rolling_due and (not viewer or now-last_display[0]<1/30.):return None
                    q=data.qpos.numpy()[0] if measured_q is None else measured_q;stamp=float(data.time.numpy()[0])
                    if viewer and now-last_display[0]>=1/30.:
                        viewer.update(q,stamp,sample_plan(plan,elapsed)['phase']);last_display[0]=now
                    if rolling_due and float(q[0])-last_station[0]>=args.step-.001:
                        if float(q[0])-last_station[0]>.2:raise ValueError('Moving camera observation spacing exceeded20cm')
                        rolling[0]+=1;folder=step/'rolling_camera'/f'frame_{rolling[0]:03d}'
                        _,_=expose(folder,q,stamp,in_motion=True);last_station[0]=float(q[0])
                        probe=patrol.decide(current_station_y_m=float(q[0]))
                        write(folder/'decision.json',probe)
                        if probe['action']!='move'or probe.get('target_key'):
                            return dict(observation=str(folder/'observation.json'),decision=probe,unexecuted_suffix_is_not_executed=True)
                    return None
                print('GPU',cycle,decision['action'],'station',round(float(bank['qpos'][0]),3),flush=True)
                def segment_ready():
                    nonlocal started,stage_started
                    if pending_latency is not None:
                        pending_latency['physics_ready_s']=time.perf_counter()-pending_latency['camera_returned_monotonic_s']
                    if not viewer or cycle!=0:return True
                    viewer.ready()
                    if args.autostart:viewer.run_event.set()
                    waiting=time.perf_counter();proceed=viewer.gate()
                    wait_s=time.perf_counter()-waiting;report['operator_wait_s']=wait_s
                    started=time.perf_counter();stage_started+=wait_s
                    return proceed
                with (step/'physics.log').open('w')as log,redirect_stdout(log),redirect_stderr(log):
                    run(runner,session_cache=cache,verification_context=verification,progress_callback=progress,
                        ready_callback=segment_ready)
                summary=read(step/'physics/summary.json')
                rows.append(dict(index=cycle,action=decision['action'],target=decision.get('target_key'),physics=str(step/'physics'),
                    status=summary['status'],stop=summary.get('stop'),simulated_s=summary.get('simulated_s'),
                    max_active_penetration_m=summary.get('max_active_penetration_m'),
                    source_model_mjb_sha256=summary.get('source_model_mjb_sha256')))
                if viewer and summary['status']=='cancelled_before_start':report['status']='user_stopped';break
                if summary['status']!='completed':
                    report.update(status='physics_stopped',reason='Actual physics stopped; no reset or next-target fallback',stop=rows[-1]);break
                parent=step/'physics'
                _,terminal,history,lineage=load_resume(parent,common,initial,meta,policy=policy,verification_context=verification)
                bank=canonical_state(model,initial,terminal);physics_index+=1;write(step/'continuation_check.json',lineage)
                if viewer:viewer.update(bank['qpos'],float(bank['time'][0]),'motion_complete')
                if viewer and viewer.stop_event.is_set():report['status']='user_stopped';break
                performance.append(dict(step=cycle,stage='physics_and_continuation',wall_s=time.perf_counter()-stage_started))
                stage_started=time.perf_counter()
                audit=motion.audit_measured(parent,plan);write(step/'measured_motion_audit.json',audit)
                performance.append(dict(step=cycle,stage='measured_audit',wall_s=time.perf_counter()-stage_started))
                pending_latency=None
                if decision['action']=='harvest':
                    harvests+=1
                    for name in decision['group']['fruits']:attempts[active][name]=attempts[active].get(name,0)+1
                    receipt=capture_receipt(model,initial,plan,parent/'states.npz',list(fruit_keys),collected,
                        volume=config.get('geometric_harvest'))
                    write(step/'geometric_capture.json',receipt);events.append(dict(step=cycle,**receipt))
                    collected.update(r['name']for r in receipt['harvested'])
                    if viewer:viewer.mark_harvested(collected)
                    print('CAMERA HARVEST',harvests,'reached',receipt['reached'],'new',len(receipt['harvested']),flush=True)
            q=bank['qpos'];station=float(q[0]);stamp=float(bank['time'][0])
            observation,ingestion=expose(step/'camera',q,stamp)
            record_scan_reacquisition(patrol,decision,observation,station)
            report.update(last_station_m=station,physics_clock_s=stamp,completed_harvest_motions=harvests,
                assumed_harvest_count=len(collected),assumed_harvest_names=sorted(collected),wall_s=time.perf_counter()-started)
            write(out/'patrol.json',patrol.to_dict());write(out/'summary.json',report)
            print('OBS',cycle,round(station,3),'visible ripe',len(ingestion['accepted']),flush=True)
            if harvests>=args.max_harvests:report['status']='harvest_budget_reached';break
            for _ in range(len(patrol.queue)+1):
                decision=patrol.decide(current_station_y_m=station)
                reused=reuse_stationary_scan(patrol,decision,observation,q,home)
                if reused is None:break
                reused_scans.append(dict(step=cycle,target=decision.get('target_key'),
                    observation=observation['observation_id'],result=reused,physical_hold_requested=False))
            plan=None
            velocity=float(bank['qvel'][initial['robot_dof_addresses'][0]])
            if velocity>.005 and decision['action']in ('harvest','scan','stop'):
                destination=min(end,station+max(.03,velocity**2/.3))
                plan=motion.scan(q,destination,rail_velocity=velocity);decision=dict(action='brake_for_camera',next_station_y_m=destination);continue
            if decision['action']=='stop':report.update(status='route_stopped',reason=decision['reason']);break
            if decision['action']in ('move','scan'):
                if decision['action']=='move'and not decision.get('target_key'):
                    plan=motion.scan(q,cruise_end=end,step=args.step,rail_velocity=velocity)
                else:plan=motion.scan(q,decision['next_station_y_m'],rail_velocity=velocity)
            elif decision['action']=='harvest':
                planning_started=time.perf_counter()
                active=decision['target']['plant_id'];key=decision['target_key']
                if active not in sources:
                    patrol.record_blocker(key,'outside_prepared_native_route',dict(plant_id=active));continue
                poses=scene.measured_poses(q)
                plan,group,proposals=plan_visible_alternatives(motion,q,observation,sources[active],meta,poses,
                    collected,attempts[active],decision['target']['truss_key'],step/'selection',stamp,
                    budget=args.proposals_per_view)
                if plan is not None:
                    decision['group']=group;write(step/'selected_group.json',group);write(step/'next_plan.json',plan)
                    pending_latency=dict(step=cycle,observation=observation['observation_id'],status='planned',
                        camera_returned_monotonic_s=observations[-1]['camera_returned_monotonic_s'],
                        capture_rpc_wall_s=observations[-1]['capture_rpc_wall_s'],
                        harvest_planning_s=time.perf_counter()-planning_started,
                        plan_ready_s=time.perf_counter()-observations[-1]['camera_returned_monotonic_s'])
                    latency.append(pending_latency)
                if plan is None:
                    patrol.record_local_failure(key,'finite_camera_goal_planning_exhausted',dict(proposals=proposals),
                        current_station_y_m=station,max_forward_views=2)
                    print('FINITE PLAN RETRY',key,flush=True)
                write(step/'planning_proposals.json',proposals)
                performance.append(dict(step=cycle,stage='harvest_planning',wall_s=time.perf_counter()-planning_started))
        else:report['status']='decision_budget_reached'
    except Exception as error:
        if viewer:
            viewer.run_event.clear();viewer.phase='error';viewer._runtime()
        report.update(status='error',error=repr(error));write(out/'error.json',dict(error=repr(error),traceback=traceback.format_exc()));raise
    finally:
        report.update(completed_harvest_motions=harvests,assumed_harvest_count=len(collected),
            last_station_m=float(bank['qpos'][0]),physics_clock_s=float(bank['time'][0]),wall_s=time.perf_counter()-started)
        write(out/'patrol.json',patrol.to_dict());write(out/'summary.json',report);write(out/'scene_stream_evidence.json',camera.evidence())
    return report,viewer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets','--prepared-assets',dest='assets',type=Path,default=Path('/root/farmily_runtime/newton/assets/side_farm_20261113_v3'))
    parser.add_argument('--ready-profile',type=Path,default=HERE/'profiles/v6_lift_centre_scan.json')
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--port',type=int,default=5906)
    parser.add_argument('--end-station',type=float,default=1.2);parser.add_argument('--step',type=float,default=.15)
    parser.add_argument('--max-harvests','--max-grips',dest='max_harvests',type=int,default=4)
    parser.add_argument('--max-decisions',type=int,default=100);parser.add_argument('--camera-scale',type=float,default=.5)
    parser.add_argument('--proposals-per-view',type=int,default=6)
    parser.add_argument('--grip-cycles',type=int,default=2)
    parser.add_argument('--headless',action='store_true');parser.add_argument('--autostart',action='store_true')
    args=parser.parse_args()
    if not 0<args.step<=.2 or min(args.max_harvests,args.max_decisions)<1:parser.error('Positive budgets and step<=.20m required')
    if args.grip_cycles!=2:parser.error('The current V6 profile performs two full jaw cycles')
    if not 1<=args.proposals_per_view<=8:parser.error('One to eight distinct observed groups per view supported')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    write(args.output/'launch.json',dict(options={k:str(v)if isinstance(v,Path)else v for k,v in vars(args).items()}))
    try:
        with ExitStack()as resources:report,viewer=campaign(args,args.output,resources)
    except Exception as error:
        if not(args.output/'error.json').exists():write(args.output/'error.json',dict(error=repr(error),traceback=traceback.format_exc()))
        if not(args.output/'summary.json').exists():write(args.output/'summary.json',dict(status='error',stage='startup',error=repr(error)))
        raise
    print(json.dumps({k:v for k,v in report.items()if k not in ('rows','observations','capture_events','sensor_geometry')},ensure_ascii=False,indent=2),flush=True)
    if viewer:viewer.finish(report)


if __name__=='__main__':main()
