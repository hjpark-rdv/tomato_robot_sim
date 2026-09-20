"""Single-process Isaac Lab candidate dataset worker (launched by runner)."""
import argparse
import json
from pathlib import Path
from isaaclab.app import AppLauncher
parser=argparse.ArgumentParser()
parser.add_argument('--run-dir',type=Path,required=True)
parser.add_argument('--rebuild',action='store_true')
parser.add_argument('--profile',action='store_true',help='Write Python CPU profile before simulator shutdown')
parser.add_argument('--benchmark-candidates',type=int,default=0)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device='cpu')
args=parser.parse_args();launcher=AppLauncher(args);app=launcher.app

import copy
import csv
import hashlib
import time
import numpy as np
import torch
from assets import build_robot
from harvest_env import HarvestEnvCfg
from dataset_scene import DatasetScene
from dataset_camera import choose_observation,capture_observation
from dataset_motion import plan,execute_steps
from dataset_design import classify_result,write_json,throughput_estimate,waypoints as design_waypoints
from pose_collision import SelfCollisionCheck
from pose_worker import state,state_comparison
from pose_candidates import DEFAULT_LIMITS

torch.set_num_threads(4)


def snapshot(slot,canonical=False):
    result=state(slot)
    if canonical:
        for key in ('robot_root','elastic_bodies','fruits'):
            result[key][...,:3]-=slot.dataset_origin
    return result


def identity(slot):
    root=slot.root+'/Robot/link6/tcp/tomato_gripper'
    proximal=[slot.elastic.paths[i]+'/StemCollider' for i in slot.elastic.chains['TRUSS_Pedicel_proximal_'+slot.target_spec['name'][-2:]]]
    distal=slot.target_spec['path']+'/PedicelCollider'
    return dict(hook_prim=root,target_proximal_pedicels=proximal,target_distal_pedicel=distal,
                accepted_hook_pedicels=[distal,proximal[-1]],
                intended_ring_colliders=[root+f'/RingCollision/segment_{i:02d}' for i in range(8,24)])


def verify_resets(world):
    world.reset_all();first=[snapshot(s) for s in world.slots]
    world.reset_all();second=[snapshot(s) for s in world.slots]
    checks=[state_comparison(a,b) for a,b in zip(first,second)]
    if not all(c['passed'] for c in checks): raise RuntimeError('Repeated reset mismatch')
    # Deliberately perturb robot AND elastic joints of one slot, then reset only
    # that slot without stepping time. Neighbour state must stay bitwise stable.
    slot=world.slots[0];q=slot.robot.data.joint_pos.clone();q[0,slot.arm_ids[0]]+=.01
    slot.robot.write_joint_state_to_sim(q,torch.zeros_like(q))
    plant=slot.elastic.articulation.data.joint_pos.clone();plant[0,0]+=.005
    slot.elastic.articulation.write_joint_state_to_sim(plant,torch.zeros_like(plant))
    world.sim.physics_sim_view.update_articulations_kinematic();world.scene.update(slot.physics_dt)
    neighbors=[snapshot(s) for s in world.slots[1:]]
    slot.reset_slot()
    independent=[state_comparison(a,snapshot(s)) for a,s in zip(neighbors,world.slots[1:])]
    restored=state_comparison(second[0],snapshot(slot))
    if not restored['passed'] or not all(c['passed'] for c in independent):
        raise RuntimeError('Independent reset/elastic deformation restoration failed')
    canonical=[snapshot(s,True) for s in world.slots]
    clone_checks=[]
    for row in canonical:
        delta={k:float(np.max(abs(row[k]-canonical[0][k]))) for k in row}
        # Float32 translated body coordinates; preserve sub-millimetre accuracy.
        clone_checks.append(dict(max_abs_by_field=delta,passed=max(delta.values())<=1e-4,tolerance=1e-4))
    if not all(c['passed'] for c in clone_checks): raise RuntimeError('Clone initial states differ after removing origin offset')
    return dict(repeated_reset=checks,independent_reset=independent,perturbed_slot_restored=restored,
                clone_equivalence=clone_checks,contact_routing_faults=world.routing_faults)


def export(root,rows,elapsed):
    ordered=sorted(rows,key=lambda r:r['candidate_id'])
    with (root/'candidates.jsonl').open('w') as f:
        for row in ordered:f.write(json.dumps(row,allow_nan=False)+'\n')
    fields=['candidate_id','target_id','observation_id','result','hook_success','executed','env_index',
            'azimuth_deg','elevation_deg','roll_deg','pitch_deg','offset_x_m','offset_y_m','offset_z_m',
            'pre_hook_distance_m','insertion_distance_m','lift_offset_m','target_max_displacement_mm',
            'main_stem_max_displacement_mm','first_contact_object','first_contact_time_s','episode_duration_s']
    with (root/'candidates.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for row in ordered:
            values={k:row.get(k) for k in fields};p=row['parameters']
            values.update({k:p[k] for k in fields if k in p})
            values.update({f'offset_{a}_m':v for a,v in zip('xyz',p['offset_xyz_m'])})
            writer.writerow(values)
    counts={label:sum(r['result']==label for r in rows) for label in
            ('success_target_hook','miss','non_target_contact','excessive_displacement','ik_failure','planning_failure','incomplete')}
    executed=sum(r['executed'] for r in rows);complete=sum(r['executed'] and r['result']!='incomplete' for r in rows)
    summary=dict(total_candidates=len(rows),executed_candidates=executed,completed_physics_candidates=complete,
                 counts=counts,success_rate_of_completed_physics=counts['success_target_hook']/complete if complete else None,
                 success_rate_of_all_candidates=counts['success_target_hook']/len(rows) if rows and not counts['incomplete'] else None,
                 non_target_contact_event_count=sum('non_target_contact' in r['events'] for r in rows),
                 elapsed_wall_s=elapsed,candidates_per_second=len(rows)/elapsed if elapsed else None)
    write_json(root/'summary.json',summary)
    return summary


def run(root):
    config=json.loads((root/'config.json').read_text());started=time.monotonic()
    torch.set_num_threads(config.get('torch_threads',4))
    import carb
    settings=carb.settings.get_settings()
    if 'physics_threads' in config:
        settings.set_int('/persistent/physics/numThreads',config['physics_threads'])
    print('[DATASET CPU]',dict(torch_threads=torch.get_num_threads(),
        physics_threads=settings.get('/persistent/physics/numThreads')),flush=True)
    sources=['assets.py','geometry.py','harvest_env.py','greenhouse_env.py','elastic_plant.py','pose_worker.py',
             'pose_candidates.py','pose_collision.py','hook_motion.py','contact_planner.py','rgb_camera.py',
             'dataset_design.py','dataset_scene.py','dataset_motion.py','dataset_camera.py','dataset_sim.py']
    fingerprints={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest() for name in sources}
    from assets import SIM_DIR, ROBOT_SOURCE
    from greenhouse_env import SCENE_SOURCE, STEM_SOURCE
    # A changed USD/CAD must not silently reuse images and old result labels.
    for asset in (SCENE_SOURCE,STEM_SOURCE,ROBOT_SOURCE,SIM_DIR/'robot_usd/rb5_farmily.urdf',
                  SIM_DIR.parent/'ros2_ws/src/rbpodo_ros2/rbpodo_description/meshes/tomato_gripper/assy_gripper_ver_6.stl'):
        digest=hashlib.sha256()
        with asset.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
        fingerprints['asset:'+str(asset)]=digest.hexdigest()
    provenance_file=root/'source_sha256.json'
    if provenance_file.exists() and json.loads(provenance_file.read_text())!=fingerprints:
        raise RuntimeError('Source changed since this dataset started; choose a new run directory')
    write_json(provenance_file,fingerprints)
    build_robot()  # Generated override only; always include repaired straight wires.
    cfg=HarvestEnvCfg();cfg.sim.device='cpu';cfg.scene.num_envs=1;cfg.curriculum='approach'
    cfg.sim.dt=1/960;cfg.decimation=16;cfg.sim.render_interval=16
    cfg.sim.physx.min_position_iteration_count=64;cfg.sim.physx.enable_external_forces_every_iteration=True
    cfg.position_jitter=0.;cfg.break_randomization=0.;cfg.plant_model='elastic';cfg.elastic_stiffness_scale=1.
    cfg.target_fruit=config['target'];cfg.stem_position=(-.75,.55,.32);cfg.stem_yaw=0.;cfg.stem_scale=.5
    cfg.lift_start_below=.4;cfg.lift_height_reference='mount';cfg.lift_speed=.1
    cfg.override_break_force=False;cfg.override_break_torque=False
    cfg.robot.actuators['lift'].velocity_limit_sim=.25;cfg.dataset_max_steps=config['max_control_steps']
    cfg.dataset_rise_speed=config.get('rise_speed',.002)
    cfg.dataset_pull_speed=config.get('pull_speed',.004)
    cfg.dataset_physics_sync=config.get('physics_sync','legacy')
    world=DatasetScene(cfg,config['num_envs'])
    try:
        validation=verify_resets(world);write_json(root/'reset_validation.json',validation)
        first=world.slots[0]
        checker=SelfCollisionCheck(first,first.pose_search_kin)
        write_json(root/'self_collision_model.json',checker.manifest)
        q,selection=choose_observation(first,checker,config)
        # The observation pose becomes the common episode initial robot state.
        for slot in world.slots:
            slot.start_q=torch.tensor(q[None],dtype=torch.float32)
        world.reset_all()
        validation['observation_pose_resets']=verify_resets(world)
        references=[snapshot(s) for s in world.slots]
        observation_dir=root/'scene_0001/observation_0001'
        if (observation_dir/'observation.json').exists():
            observation=json.loads((observation_dir/'observation.json').read_text())
            if not np.allclose(observation['joint_positions_at_capture'],q,atol=1e-6,rtol=0):
                raise RuntimeError('Resume observation pose differs')
            for key in ('rgb','depth','depth_valid','local_rgb','local_depth','local_depth_valid'):
                if not (observation_dir/observation[key]).exists():
                    raise RuntimeError('Resume observation incomplete: '+key)
        else:
            observation=capture_observation(first,observation_dir,config,selection)
        checks=[state_comparison(reference,snapshot(slot)) for reference,slot in zip(references,world.slots)]
        if not all(c['passed'] for c in checks):raise RuntimeError('RGB-D rendering modified physics state')
        validation['render_preserves_state']=checks
        print('[DATASET CAMERA] saved; checking dynamic clone equivalence',flush=True)
        # Exercise all clones briefly; no training/rollout labels are generated.
        for _ in range(16):world.step()
        canonical=[snapshot(s,True) for s in world.slots]
        dynamic=[]
        for value in canonical:
            delta={k:float(np.max(abs(value[k]-canonical[0][k]))) for k in value}
            dynamic.append(dict(max_abs_by_field=delta,passed=max(delta.values())<.002))
        validation['identical_command_clone_check']=dynamic
        write_json(root/'reset_validation.json',validation)
        if not all(c['passed'] for c in dynamic):raise RuntimeError('Clones diverge during identical no-action step; see reset_validation.json')
        world.reset_all()
        validation['after_step_reset']=[state_comparison(r,snapshot(s)) for r,s in zip(references,world.slots)]
        if not all(c['passed'] for c in validation['after_step_reset']):raise RuntimeError('State not restored after physics stepping')
        write_json(root/'reset_validation.json',validation)
        print('[DATASET VALIDATED]',config['num_envs'],'clones; RGB-D',observation['crop'],flush=True)
        if config['validate_only']:
            write_json(root/'validation_complete.json',dict(passed=True,candidate_rollouts_executed=0))
            return
        params=json.loads((root/'candidates.json').read_text())
        rows=[json.loads(p.read_text()) for p in sorted((root/'results').glob('*/candidate.json'))]
        done={r['candidate_id'] for r in rows}
        pending=[p for p in params if p['candidate_id'] not in done]
        if args.benchmark_candidates:pending=pending[:args.benchmark_candidates]
        rollout_started=time.monotonic();initial_rows=len(rows);last_progress=rollout_started
        def progress_summary():
            elapsed=time.monotonic()-rollout_started
            finished=len(rows)-initial_rows
            result=throughput_estimate(len(params),len(rows),finished,rollout_started-started,
                                       elapsed,bool(config['max_control_steps']))
            result['physics_step_timings']=world.step_timings.copy()
            return result
        for batch_start in range(0,len(pending),config['num_envs']):
            world.reset_all()
            resets=[state_comparison(r,snapshot(s)) for r,s in zip(references,world.slots)]
            if not all(c['passed'] for c in resets):raise RuntimeError('Candidate reset contamination')
            active={};contexts={}
            for slot,param in zip(world.slots,pending[batch_start:batch_start+config['num_envs']]):
                p=dict(param,goal=config['goal'])
                target_center,target_neck,_=[v[0].cpu().numpy() for v in first._target_geometry()]
                desired_rotation,desired_waypoints,desired_direction=design_waypoints(target_center,target_neck,p)
                desired_position=desired_waypoints[0][1]
                desired_quaternion=desired_rotation.as_quat().tolist()
                plan_started=time.monotonic()
                # All clones share canonical geometry/joints; IK/FCL is computed
                # in env_0 coordinates, then identical commands run in each slot.
                planned,preflight=plan(first,first.pose_search_kin,checker,p)
                folder=root/'results'/p['candidate_id'];folder.mkdir(parents=True,exist_ok=True)
                row=dict(scene_id='scene_0001',target_id=config['target'],candidate_id=p['candidate_id'],
                    observation_id='observation_0001',observation_path='scene_0001/observation_0001/observation.json',
                    parameters=p,env_index=slot.index,env_origin_world=slot.dataset_origin.tolist(),
                    robot_joint_positions_at_start=slot.robot.data.joint_pos[0].tolist(),
                    preflight=preflight,reset_validation=resets[slot.index],planning_wall_s=time.monotonic()-plan_started,
                    candidate_pose_target_frame=dict(position_xyz=(desired_position-target_center).tolist(),orientation_xyzw=desired_quaternion),
                    candidate_position_xyz=desired_position.tolist(),candidate_orientation_quaternion=desired_quaternion,
                    quaternion_order='xyzw',candidate_world_frame='canonical env_0; add env_origin_world for executed clone',
                    approach_direction=desired_direction.tolist(),
                    target_frame_definition='origin at initial fruit centre, axes aligned with world; GT centre for generation only',
                    physical_inputs=dict(physics_dt=slot.physics_dt,control_dt=slot.step_dt,
                        solver_position_iterations=cfg.sim.physx.min_position_iteration_count,
                        approach_speed_m_s=.035,rise_speed_m_s=cfg.dataset_rise_speed,
                        pull_speed_m_s=cfg.dataset_pull_speed,hold_s=DEFAULT_LIMITS['hold_seconds'],limits=DEFAULT_LIMITS),
                    label_policy='dataset_v1: gentle target-fruit contact allowed; excessive displacement, break, dangerous non-target contact and non-target hook disqualify')
                if planned is None:
                    row.update(classify_result(dict(planning_failure_reason=preflight),DEFAULT_LIMITS))
                    write_json(folder/'candidate.json',row);rows.append(row);continue
                position=planned['prehook'];quat=planned['orientation'].as_quat().tolist()
                row.update(candidate_position_xyz=position.tolist(),candidate_orientation_quaternion=quat,
                    quaternion_order='xyzw',approach_direction=planned['direction'].tolist(),
                    candidate_pose_target_frame=dict(position_xyz=(position-planned['target_center']).tolist(),orientation_xyzw=quat),
                    commanded_waypoints=planned['waypoints'],planned_command_file=str((folder/'planned_commands.npy').relative_to(root)))
                np.save(folder/'planned_commands.npy',planned['commands'])
                slot.contact_diagnostics=[]
                active[slot.index]=execute_steps(slot,planned,identity(slot),app,DEFAULT_LIMITS.copy())
                contexts[slot.index]=(row,folder)
                print('[DATASET START]',p['candidate_id'],'env',slot.index,flush=True)
            ticks=0
            while active:
                for index in list(active):
                    try:next(active[index])
                    except StopIteration as finished:
                        metrics,trace=finished.value;slot=world.slots[index];row,folder=contexts[index]
                        row.update(metrics);row.update(classify_result(metrics,DEFAULT_LIMITS))
                        first_contact=metrics['first_contact']
                        row.update(first_contact_time_s=(first_contact['step']+1)*slot.step_dt if first_contact else None,
                            target_max_displacement_mm=metrics['target_max_displacement_m']*1000,
                            main_stem_max_displacement_mm=metrics['main_stem_max_displacement_m']*1000,
                            episode_duration_s=len(trace)*slot.step_dt,hooked_target=metrics['retained_hook'])
                        write_json(folder/'trace.json',trace);write_json(folder/'contacts.json',slot.contact_diagnostics)
                        write_json(folder/'candidate.json',row);rows.append(row);del active[index]
                        # Completed slot may evolve but is not re-used mid-batch.
                        # Batch barrier avoids changing constraints under another
                        # slot's ongoing trial; all slots reset before next batch.
                        print('[DATASET RESULT]',row['candidate_id'],row['result'],metrics['abort_reason'],flush=True)
                if active:world.step();ticks+=1
                if time.monotonic()-last_progress>=15:
                    progress=progress_summary();progress.update(active_candidates=len(active),batch_sim_seconds=round(ticks*cfg.sim.dt,2))
                    write_json(root/'progress.json',progress)
                    print('[DATASET PROGRESS]',json.dumps(progress),flush=True);last_progress=time.monotonic()
            summary=export(root,rows,time.monotonic()-started)
            summary.update(progress_summary());write_json(root/'summary.json',summary)
            print('[DATASET BATCH]',json.dumps(summary),flush=True)
        summary=export(root,rows,time.monotonic()-started)
        summary.update(progress_summary())
        summary['benchmark_only']=bool(args.benchmark_candidates)
        summary['execution_complete']=True
        summary['dataset_complete']=len(rows)==len(params) and not summary['counts']['incomplete']
        summary['num_envs']=config['num_envs'];summary['target']=config['target']
        summary['reset_and_camera_validation_passed']=True
        write_json(root/'summary.json',summary)
        try:
            import matplotlib;matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig,ax=plt.subplots(figsize=(7,5))
            for label in sorted({r['result'] for r in rows}):
                selected=[r for r in rows if r['result']==label]
                ax.scatter([r['parameters']['azimuth_deg'] for r in selected],
                           [r['parameters']['elevation_deg'] for r in selected],label=label,s=18)
            ax.set(xlabel='Approach azimuth (deg)',ylabel='Approach elevation (deg)');ax.legend()
            fig.tight_layout();fig.savefig(root/'candidate_results.png',dpi=150);plt.close(fig)
        except ImportError:pass
        print('[DATASET COMPLETE]',root,flush=True)
    finally:world.close()


profiler=None
if args.profile:
    import cProfile
    profiler=cProfile.Profile();profiler.enable()
try:
    run(args.run_dir)
except BaseException as error:
    import traceback
    write_json(args.run_dir/'execution_error.json',dict(type=type(error).__name__,message=str(error),dataset_complete=False))
    traceback.print_exc()
    raise
finally:
    if profiler is not None:
        profiler.disable();profiler.dump_stats(str(args.run_dir/'cpu_profile.pstats'))
    app.close()
