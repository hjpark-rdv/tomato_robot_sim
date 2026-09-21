"""Experimental GPU PhysX dataset worker. Shared planning, geometry and label rules remain unchanged."""
import argparse
import json
from pathlib import Path
from isaaclab.app import AppLauncher
parser=argparse.ArgumentParser()
parser.add_argument('--run-dir',type=Path,required=True)
parser.add_argument('--rebuild',action='store_true')
parser.add_argument('--observation-only',action='store_true',help='Capture the common pre-action image in a single environment, then exit')
parser.add_argument('--profile',action='store_true',help='Write Python CPU profile before simulator shutdown')
parser.add_argument('--benchmark-candidates',type=int,default=0)
parser.add_argument('--app-threads',type=int,default=32,help='Kit startup thread cap; physics threads are configured separately')
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device='cpu')
args=parser.parse_args();launcher=AppLauncher(args,limit_cpu_threads=args.app_threads);app=launcher.app

import copy
import csv
import hashlib
import time
import numpy as np
import torch
from assets import build_robot
from harvest_env import HarvestEnvCfg
from gpu_dataset_scene import GpuDatasetScene as DatasetScene
from gpu_validation import check_clone
from dataset_camera import choose_observation,capture_observation
from dataset_motion import plan,execute_steps
from dataset_design import classify_result,write_json,throughput_estimate,waypoints as design_waypoints
from pose_collision import SelfCollisionCheck
from pose_worker import state,state_comparison
from pose_candidates import DEFAULT_LIMITS

torch.set_num_threads(4)


def snapshot(slot,canonical=False):
    result=state(slot)
    # q and -q describe the same orientation. GPU link transforms may choose
    # either sign; normalize only representation, never position or velocity.
    for key in ('robot_root','elastic_bodies','fruits'):
        quaternion=result[key][...,3:7]
        pivot=np.take_along_axis(quaternion,np.argmax(np.abs(quaternion),axis=-1)[...,None],axis=-1)
        quaternion*=np.where(pivot<0,-1.,1.)
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
            'physics_valid','max_tool_penetration_mm','exclude_from_valid_trajectory_analysis',
            'azimuth_deg','elevation_deg','roll_deg','pitch_deg','offset_x_m','offset_y_m','offset_z_m',
            'pre_hook_distance_m','insertion_distance_m','lift_offset_m','target_max_displacement_mm',
            'main_stem_max_displacement_mm','first_contact_object','first_contact_time_s','episode_duration_s']
    fields+=['approach_azimuth_deg','entry_clearance_m','lateral_offset_m','lift_forward_angle_deg','lift_distance_m',
             'entry_reached','center_entry_achieved','center_entry_safe','center_entry_time_s','center_entry_duration_s','lift_completed','highest_stage']
    with (root/'candidates.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for row in ordered:
            values={k:row.get(k) for k in fields};p=row['parameters']
            values.update({k:p[k] for k in fields if k in p})
            values.update({f'offset_{a}_m':v for a,v in zip('xyz',p.get('offset_xyz_m',[]))})
            writer.writerow(values)
    counts={label:sum(r['result']==label for r in rows) for label in
            ('success_target_hook','miss','non_target_contact','excessive_displacement','ik_failure','planning_failure','incomplete','invalid_physics')}
    executed=sum(r['executed'] for r in rows);complete=sum(r['executed'] and r['result'] not in ('incomplete','invalid_physics') for r in rows)
    summary=dict(total_candidates=len(rows),executed_candidates=executed,completed_physics_candidates=complete,
                 counts=counts,success_rate_of_completed_physics=counts['success_target_hook']/complete if complete else None,
                 success_rate_of_all_candidates=counts['success_target_hook']/len(rows) if rows and not counts['incomplete'] and not counts['invalid_physics'] else None,
                 invalid_physics_candidates=counts['invalid_physics'],
                 non_target_contact_event_count=sum('non_target_contact' in r['events'] for r in rows),
                 elapsed_wall_s=elapsed,candidates_per_second=len(rows)/elapsed if elapsed else None)
    summary.update(physics_device='gpu',tensor_device='cpu',solver='PGS',experimental=True)
    write_json(root/'summary.json',summary)
    return summary


def run(root):
    config=json.loads((root/'config.json').read_text());started=time.monotonic()
    if args.observation_only:
        config['num_envs']=1
    validation_path=root/('observation_reset_validation.json' if args.observation_only else 'reset_validation.json')
    torch.set_num_threads(config.get('torch_threads',4))
    import carb
    settings=carb.settings.get_settings()
    if 'physics_threads' in config:
        settings.set_int('/persistent/physics/numThreads',config['physics_threads'])
    print('[DATASET HOST THREADS]',dict(torch_threads=torch.get_num_threads(),
        physics_threads=settings.get('/persistent/physics/numThreads')),flush=True)
    sources=['assets.py','geometry.py','harvest_env.py','greenhouse_env.py','elastic_plant.py','pose_worker.py',
             'pose_candidates.py','pose_collision.py','hook_motion.py','contact_planner.py','rgb_camera.py',
             'dataset_design.py','dataset_scene.py','dataset_motion.py','dataset_camera.py','gpu_dataset_sim.py',
             'gpu_dataset_scene.py','gpu_batch_views.py','gpu_validation.py','gpu_prim_lookup.py','gpu_replication.py','gpu_physics_errors.py','gpu_initialization.py','gpu_grid_view.py',
             'gpu_planning.py','gpu_planning_worker.py','trajectory_search.py','trajectory_report.py',
             'tool_contact_audit.py','connection_audit.py','contact_policy.py','display_skin.py']
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
    # Workers own their overrides so concurrent runs cannot truncate/reload a
    # USD that another simulator has open.
    robot_asset=build_robot(root/'generated/rb5_ring.usda')
    cfg=HarvestEnvCfg();cfg.sim.device='cpu';cfg.scene.num_envs=1;cfg.curriculum='approach'
    cfg.robot.spawn.usd_path=str(robot_asset)
    physics_hz=config.get('physics_hz',960)
    cfg.sim.dt=1/physics_hz;cfg.decimation=physics_hz//60;cfg.sim.render_interval=cfg.decimation
    # FCL initialization and source-mesh RGB-D currently read USD transforms.
    # Keep their readback current; GPU dynamics is independent of Fabric.
    cfg.sim.use_fabric=False
    cfg.gpu_batched_joint_commands=config.get('command_uploads','batched')=='batched'
    cfg.sim.physx.solver_type=0
    # Multiple GPU constraint partitions diverge for this branched plant with
    # standalone break joints. One partition keeps environments independent
    # without changing drives, contacts, timestep, iterations or tolerances.
    cfg.sim.physx.gpu_max_num_partitions=1
    cfg.sim.physx.enable_ccd=False
    position_iterations=config.get('position_iterations',64)
    cfg.sim.physx.max_position_iteration_count=position_iterations
    velocity_iterations=config.get('velocity_iterations',4)
    cfg.sim.physx.min_velocity_iteration_count=velocity_iterations
    cfg.sim.physx.max_velocity_iteration_count=velocity_iterations
    cfg.sim.physx.min_position_iteration_count=position_iterations;cfg.sim.physx.enable_external_forces_every_iteration=True
    cfg.position_jitter=0.;cfg.break_randomization=0.;cfg.plant_model='elastic';cfg.elastic_stiffness_scale=1.
    cfg.elastic_joint_armature=config.get('elastic_joint_armature',1e-5)
    cfg.contact_policy=config.get('contact_policy','legacy')
    cfg.target_fruit=config['target'];cfg.stem_position=(-.75,.55,.32);cfg.stem_yaw=0.;cfg.stem_scale=.5
    cfg.lift_start_below=.4;cfg.lift_height_reference='mount';cfg.lift_speed=.1
    cfg.override_break_force=False;cfg.override_break_torque=False
    cfg.robot.actuators['lift'].velocity_limit_sim=.25;cfg.dataset_max_steps=config['max_control_steps']
    cfg.dataset_rise_speed=config.get('rise_speed',.002)
    cfg.dataset_pull_speed=config.get('pull_speed',.004)
    cfg.dataset_physics_sync=config.get('physics_sync','legacy')
    cfg.gpu_native_replication=True
    world=DatasetScene(cfg,config['num_envs'])
    viewer=None;planning_service=None
    try:
        context=world.sim.get_physics_context()
        env_id_attr=world.scene.stage.GetPrimAtPath(context.prim_path).GetAttribute('physxScene:envIdInBoundsBitCount')
        backend=dict(physics_device='gpu',tensor_device='cpu',solver='PGS',position_iterations=position_iterations,
                     physics_preset=config.get('physics_preset','reference960'),
                     physics_hz=physics_hz,control_hz=60,
                     elastic_joint_armature_kg_m2=cfg.elastic_joint_armature,
                     command_uploads=config.get('command_uploads','batched'),
                     velocity_iterations=velocity_iterations,contact_policy=world.contact_policy,
                     tool_penetration_guard=True,tool_penetration_tolerance_m=.0005,
                     gpu_max_num_partitions=context.get_gpu_max_num_partitions(),ccd=False,native_collider_reports=True,
                     gpu_dynamics=context.is_gpu_dynamics_enabled(),broadphase=context.get_broadphase_type(),
                     suppress_readback=world.sim.carb_settings.get('/physics/suppressReadback'),
                     experimental=True,cpu_tgs_equivalence_guaranteed=False,
                     native_physics_replication=config['num_envs']>1,
                     environment_id_bounds_bits=env_id_attr.Get() if env_id_attr else None)
        if not backend['gpu_dynamics'] or backend['broadphase']!='GPU' or backend['suppress_readback'] or backend['gpu_max_num_partitions']!=1:
            raise RuntimeError('GPU physics / native collider readback backend assertion failed')
        for slot in world.slots:
            actual=slot.elastic.articulation.root_physx_view.get_dof_armatures()
            if not torch.equal(actual,torch.full_like(actual,cfg.elastic_joint_armature)):
                raise RuntimeError('Native elastic joint armature differs from requested configuration')
        if config['num_envs']>1:
            from gpu_replication import environment_id_bits
            if backend['environment_id_bounds_bits']!=environment_id_bits(config['num_envs']):
                raise RuntimeError('Native environment-ID broadphase filtering not configured')
        write_json(root/('observation_backend.json' if args.observation_only else 'backend.json'),backend)
        print('[DATASET GPU]',backend,flush=True)
        validation=verify_resets(world);write_json(validation_path,validation)
        from tool_contact_audit import initialize_world_audits
        initialize_world_audits(world,root/'collision_model')
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
        if args.observation_only:
            np.savez_compressed(root/'observation_initial_state.npz',**references[0])
            write_json(validation_path,validation)
            write_json(root/'observation_complete.json',dict(passed=True,num_envs=1,source_sha256=fingerprints,
                       files_sha256={key:hashlib.sha256((observation_dir/observation[key]).read_bytes()).hexdigest()
                                     for key in ('rgb','depth','depth_valid','local_rgb','local_depth','local_depth_valid')}))
            print('[DATASET OBSERVATION] saved in isolated one-environment renderer',flush=True)
            return
        if (root/'observation_complete.json').exists():
            observed=json.loads((root/'observation_complete.json').read_text())
            if observed['source_sha256']!=fingerprints:
                raise RuntimeError('Observation process source differs from physics process')
            if any(hashlib.sha256((observation_dir/observation[key]).read_bytes()).hexdigest()!=digest
                   for key,digest in observed['files_sha256'].items()):
                raise RuntimeError('Observation image/depth changed after capture')
            with np.load(root/'observation_initial_state.npz') as observed_state:
                validation['isolated_observation_state_match']=state_comparison(dict(observed_state),references[0])
            if not validation['isolated_observation_state_match']['passed']:
                raise RuntimeError('Single-env camera state differs from batch initial state')
        print('[DATASET CAMERA] saved; checking dynamic clone equivalence',flush=True)
        # Exercise all clones briefly; no training/rollout labels are generated.
        for _ in range(cfg.decimation):world.step()
        canonical=[snapshot(s,True) for s in world.slots]
        dynamic=[]
        for value in canonical:
            delta={k:float(np.max(abs(value[k]-canonical[0][k]))) for k in value}
            dynamic.append(dict(max_abs_by_field=delta,passed=max(delta.values())<.002))
        validation['legacy_mixed_unit_clone_check']=dynamic
        dynamic=[check_clone(canonical[0],value,broken=bool(slot._broken[0] or slot._other_broken))
                 for slot,value in zip(world.slots,canonical)]
        validation['identical_command_clone_check']=dynamic
        physical_deltas=[]
        for slot,value in zip(world.slots,canonical):
            physical_deltas.append(dict(env_index=slot.index,
                max_body_position_delta_m=float(np.max(np.linalg.norm(value['elastic_bodies'][...,:3]-canonical[0]['elastic_bodies'][...,:3],axis=-1))),
                max_fruit_position_delta_m=float(np.max(np.linalg.norm(value['fruits'][...,:3]-canonical[0]['fruits'][...,:3],axis=-1))),
                max_plant_joint_position_delta_rad=float(np.max(abs(value['elastic_q']-canonical[0]['elastic_q']))),
                max_plant_joint_velocity_delta_rad_s=float(np.max(abs(value['elastic_dq']-canonical[0]['elastic_dq']))),
                target_broken=bool(slot._broken[0]),other_broken=bool(slot._other_broken),
                break_events=[e for e in slot.contact_diagnostics if e['event']=='break']))
        validation['clone_physical_deltas']=physical_deltas
        if not all(c['passed'] for c in dynamic):
            np.savez_compressed(root/'clone_divergence_states.npz',**{key:np.stack([s[key] for s in canonical]) for key in canonical[0]})
        write_json(validation_path,validation)
        if not all(c['passed'] for c in dynamic):raise RuntimeError('Clones diverge during identical no-action step; see reset_validation.json')
        world.reset_all()
        validation['after_step_reset']=[state_comparison(r,snapshot(s)) for r,s in zip(references,world.slots)]
        if not all(c['passed'] for c in validation['after_step_reset']):raise RuntimeError('State not restored after physics stepping')
        write_json(validation_path,validation)
        print('[DATASET VALIDATED]',config['num_envs'],'clones; RGB-D',observation['crop'],flush=True)
        if config.get('view_grid') and not args.observation_only:
            from gpu_grid_view import GridView
            before=[snapshot(s) for s in world.slots]
            viewer=GridView(world,app,count=16,fps=config.get('view_fps',5.),output=root)
            checks=[state_comparison(a,snapshot(s)) for a,s in zip(before,world.slots)]
            write_json(root/'display_validation.json',dict(physics_state_preserved=all(c['passed'] for c in checks),
                checks=checks,displayed_environments=viewer.count,
                layout='native single environment' if viewer.direct else 'read-only visual copies; original physics co-located',
                physics_dt_s=viewer.physics_dt,requested_fps=viewer.fps,display_budget_fraction=viewer.cadence.budget,
                usd_writeback='explicit at display refresh; native physics readback unchanged',
                display_has_physics_schemas=False))
            if not all(c['passed'] for c in checks):raise RuntimeError('Display rendering modified physics state')
            viewer.capture();viewer.update(force=True)
            print('[실시간 화면] '+('원본 단일 환경' if viewer.direct else '다중 환경 격자')+' 준비 완료; 환경 행을 누르면 고리 주변을 확대합니다',flush=True)
        if config['validate_only']:
            write_json(root/'validation_complete.json',dict(passed=True,candidate_rollouts_executed=0))
            return
        params=json.loads((root/'candidates.json').read_text())
        rows=[json.loads(p.read_text()) for p in sorted((root/'results').glob('*/candidate.json'))]
        done={r['candidate_id'] for r in rows}
        pending=[p for p in params if p['candidate_id'] not in done]
        if args.benchmark_candidates:pending=pending[:args.benchmark_candidates]
        rollout_started=time.monotonic();initial_rows=len(rows);last_progress=rollout_started
        phase_timings=dict(batch_reset_s=0.,planning_s=0.,execution_and_label_s=0.,result_save_s=0.)
        workers=min(config.get('planning_workers',0),len(pending))
        # A single task cannot benefit from multiprocessing startup.
        if workers>1:
            from gpu_planning import PlanningService,export_model
            model=export_model(world.slots[0],checker)
            from datetime import datetime
            folder=root/('planning_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            planning_service=PlanningService(folder,model,[dict(p,goal=config['goal']) for p in pending],workers)
            print('[DATASET PLANNING]',workers,'CPU workers; precomputing',len(pending),'paths',flush=True)
        rollout_step_start=world.step_timings.copy()
        display_start=viewer.timings.copy() if viewer else None
        active_env_steps=0
        def progress_summary():
            elapsed=time.monotonic()-rollout_started
            finished=len(rows)-initial_rows
            result=throughput_estimate(len(params),len(rows),finished,rollout_started-started,
                                       elapsed,bool(config['max_control_steps']))
            result['physics_step_timings']=world.step_timings.copy()
            result['rollout_step_timings']={k:world.step_timings[k]-rollout_step_start[k] for k in world.step_timings}
            result['phase_timings']=phase_timings.copy()
            if viewer:
                result['display_timings']={k:viewer.timings[k]-display_start[k] for k in viewer.timings}
            result['schedule']=config.get('schedule','batch')
            result['planning_workers']=workers if planning_service else 0
            result['planning_timing_basis']='main-thread wait for CPU preplanning' if planning_service else 'synchronous CPU calculation'
            if planning_service and (planning_service.folder/'progress.json').exists():
                result['cpu_planning_progress']=json.loads((planning_service.folder/'progress.json').read_text())
            allocated_steps=result['rollout_step_timings']['steps']*config['num_envs']
            result['active_env_step_fraction']=active_env_steps/allocated_steps if allocated_steps else None
            return result
        from collections import deque
        continuous=config.get('schedule','batch')=='continuous'
        queue=deque(pending)
        def prepare(slot,param,reset_check):
            p=dict(param,goal=config['goal'])
            target_center,target_neck,_=[v[0].cpu().numpy() for v in slot._target_geometry()]
            desired_rotation,desired_waypoints,desired_direction=design_waypoints(target_center,target_neck,p,slot.target_spec['radius'])
            desired_position=desired_waypoints[0][1]
            desired_quaternion=desired_rotation.as_quat().tolist()
            plan_started=time.monotonic()
            # All clones share canonical geometry/joints. Use this freshly
            # reset slot; env_0 may still be executing another candidate.
            if planning_service:
                prepared,wait_s=planning_service.take(p['candidate_id'],slot)
                planned,preflight=prepared['planned'],prepared['preflight']
                compute_s=prepared['compute_wall_s']
            else:
                planned,preflight=plan(slot,slot.pose_search_kin,checker,p)
                compute_s=time.monotonic()-plan_started;wait_s=compute_s
            phase_timings['planning_s']+=time.monotonic()-plan_started
            folder=root/'results'/p['candidate_id'];folder.mkdir(parents=True,exist_ok=True)
            row=dict(scene_id='scene_0001',target_id=config['target'],candidate_id=p['candidate_id'],
                observation_id='observation_0001',observation_path='scene_0001/observation_0001/observation.json',
                parameters=p,env_index=slot.index,env_origin_world=slot.dataset_origin.tolist(),
                robot_joint_positions_at_start=slot.robot.data.joint_pos[0].tolist(),
                preflight=preflight,reset_validation=reset_check,planning_wall_s=time.monotonic()-plan_started,
                planning_compute_s=compute_s,planning_wait_s=wait_s,
                candidate_pose_target_frame=dict(position_xyz=(desired_position-target_center).tolist(),orientation_xyzw=desired_quaternion),
                candidate_position_xyz=desired_position.tolist(),candidate_orientation_quaternion=desired_quaternion,
                quaternion_order='xyzw',candidate_world_frame='canonical env_0; add env_origin_world for executed clone',
                approach_direction=desired_direction.tolist(),
                target_frame_definition='origin at initial fruit centre, axes aligned with world; GT centre for generation only',
                physical_inputs=dict(physics_dt=slot.physics_dt,control_dt=slot.step_dt,
                    target_collision_radius_m=slot.target_spec['radius'],
                    backend=backend,
                    solver_position_iterations=cfg.sim.physx.min_position_iteration_count,
                    approach_speed_m_s=.035,rise_speed_m_s=cfg.dataset_rise_speed,
                    pull_speed_m_s=cfg.dataset_pull_speed,hold_s=DEFAULT_LIMITS['hold_seconds'],limits=DEFAULT_LIMITS),
                label_policy='dataset_v1: gentle target-fruit contact allowed; excessive displacement, break, dangerous non-target contact and non-target hook disqualify')
            if planned is None:
                row.update(classify_result(dict(planning_failure_reason=preflight),DEFAULT_LIMITS))
                if viewer:viewer.set_status(slot.index,p['candidate_id']+' / planning failure');viewer.pump()
                write_json(folder/'candidate.json',row);rows.append(row);return False
            position=planned['prehook'];quat=planned['orientation'].as_quat().tolist()
            row.update(candidate_position_xyz=position.tolist(),candidate_orientation_quaternion=quat,
                quaternion_order='xyzw',approach_direction=planned['direction'].tolist(),
                candidate_pose_target_frame=dict(position_xyz=(position-planned['target_center']).tolist(),orientation_xyzw=quat),
                commanded_waypoints=planned['waypoints'],planned_command_file=str((folder/'planned_commands.npy').relative_to(root)))
            np.save(folder/'planned_commands.npy',planned['commands'])
            slot.contact_diagnostics=[]
            active[slot.index]=execute_steps(slot,planned,identity(slot),app,DEFAULT_LIMITS.copy())
            contexts[slot.index]=(row,folder)
            print('[DATASET READY]',p['candidate_id'],'env',slot.index,'path prepared; awaiting physics step',flush=True)
            if viewer:viewer.set_status(slot.index,p['candidate_id']+' / running');viewer.pump()
            return True

        def fill_slot(index,reset_check=None):
            if not queue:return
            slot=world.slots[index]
            if reset_check is None:
                reset_started=time.monotonic()
                world.reset_slot(index)
                reset_check=state_comparison(references[index],snapshot(slot))
                phase_timings['batch_reset_s']+=time.monotonic()-reset_started
                if not reset_check['passed']:raise RuntimeError('Continuous candidate reset contamination')
            while queue:
                if prepare(slot,queue.popleft(),reset_check):return

        batch_starts=[0] if continuous and pending else range(0,len(pending),config['num_envs'])
        for batch_start in batch_starts:
            reset_started=time.monotonic()
            world.reset_all()
            if viewer:
                for slot in world.slots:viewer.set_status(slot.index,'planning')
                viewer.pump()
            resets=[state_comparison(r,snapshot(s)) for r,s in zip(references,world.slots)]
            phase_timings['batch_reset_s']+=time.monotonic()-reset_started
            if not all(c['passed'] for c in resets):raise RuntimeError('Candidate reset contamination')
            active={};contexts={}
            if continuous:
                for slot in world.slots:fill_slot(slot.index,resets[slot.index])
            else:
                for slot,param in zip(world.slots,pending[batch_start:batch_start+config['num_envs']]):
                    prepare(slot,param,resets[slot.index])
            ticks=0
            print('[DATASET PHYSICS]',len(active),'active environments; beginning shared physics steps',flush=True)
            while active:
                for index in list(active):
                    while index in active:
                        advance_started=time.monotonic()
                        try:next(active[index])
                        except StopIteration as finished:
                            phase_timings['execution_and_label_s']+=time.monotonic()-advance_started
                            save_started=time.monotonic()
                            metrics,trace=finished.value;slot=world.slots[index];row,folder=contexts[index]
                            row.update(metrics);row.update(classify_result(metrics,DEFAULT_LIMITS))
                            first_contact=metrics['first_contact']
                            row.update(first_contact_time_s=(first_contact['step']+1)*slot.step_dt if first_contact else None,
                                target_max_displacement_mm=metrics['target_max_displacement_m']*1000,
                                main_stem_max_displacement_mm=metrics['main_stem_max_displacement_m']*1000,
                                episode_duration_s=metrics.get('executed_physics_steps',len(trace)*cfg.decimation)*slot.physics_dt,
                                hooked_target=metrics['retained_hook'])
                            write_json(folder/'trace.json',trace);write_json(folder/'contacts.json',slot.contact_diagnostics)
                            if 'physical_audit' in metrics:
                                write_json(folder/'physical_audit.json',metrics['physical_audit'])
                                write_json(folder/'tool_contact_trace.json',slot.tool_audit.rows)
                            write_json(folder/'candidate.json',row);rows.append(row);del active[index]
                            print('[DATASET RESULT]',row['candidate_id'],row['result'],metrics['abort_reason'],flush=True)
                            if viewer:viewer.set_status(index,row['candidate_id']+' / '+row['result'])
                            phase_timings['result_save_s']+=time.monotonic()-save_started
                            if continuous:fill_slot(index)
                        else:
                            phase_timings['execution_and_label_s']+=time.monotonic()-advance_started
                            break
                if active:
                    active_env_steps+=len(active)
                    world.step();ticks+=1
                if viewer:viewer.pump(physics_step=True)
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
        summary['dataset_complete']=len(rows)==len(params) and not summary['counts']['incomplete'] and not summary['counts']['invalid_physics']
        summary['num_envs']=config['num_envs'];summary['target']=config['target']
        summary['reset_and_camera_validation_passed']=True
        if planning_service:
            if planning_service.process.wait(timeout=30)!=0:raise RuntimeError('CPU planning service failed during shutdown')
            summary['cpu_planning']=json.loads((planning_service.folder/'summary.json').read_text())
        write_json(root/'summary.json',summary)
        try:
            from trajectory_report import plot_results
            plot_results(root,rows)
        except ImportError:pass
        print('[DATASET COMPLETE]',root,flush=True)
    finally:
        if planning_service:planning_service.close()
        if viewer:
            import sys
            if sys.exc_info()[0] is None:
                viewer.finished=True;viewer.capture();viewer.update(force=True)
                while config.get('keep_open') and app.is_running():
                    viewer.update();time.sleep(.05)
            viewer.close()
        world.close()


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
