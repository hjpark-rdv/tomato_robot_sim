"""Controlled CPU/GPU PhysX comparison; original scene, saved joint commands.

GPU dynamics with CPU tensor readback is deliberate: test the solver before
attempting a fully vectorized CUDA runner. This is not production dataset data.
"""
import argparse
import json
from pathlib import Path
from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--fixture',type=Path,required=True)
parser.add_argument('--mode',choices=['cpu','cpu-no-ccd','gpu'],required=True)
parser.add_argument('--num-envs',type=int,default=1)
parser.add_argument('--physics-hz',type=int,choices=[60,120,240,480,720,960],default=960,help='Timestep experiment; command rate remains 60 Hz')
parser.add_argument('--max-control-steps',type=int,default=0)
parser.add_argument('--external-forces-once',action='store_true',help='Diagnostic only: change TGS external force integration')
parser.add_argument('--stationary-steps',type=int,default=0,help='Diagnostic: hold robot and stop at first break')
parser.add_argument('--preserve-joints',action='store_true',help='Diagnostic: author joints before physics starts, never recreate')
parser.add_argument('--velocity-iterations',type=int,default=4,help='Diagnostic solver setting; production uses 4')
parser.add_argument('--solver',choices=['tgs','pgs'],default='tgs')
parser.add_argument('--zero-load',action='store_true',help='Diagnostic only: disable gravity and preload')
parser.add_argument('--no-contacts',action='store_true',help='Diagnostic only: disable all collision shapes')
parser.add_argument('--cuda-tensors',action='store_true',help='Diagnostic: use CUDA tensor API as well as GPU physics')
parser.add_argument('--fabric',action='store_true',help='Diagnostic: avoid per-step USD physics writeback')
parser.add_argument('--joint-variant',choices=['original','reverse-fixed','locked-d6'],default='original',help='Diagnostic equivalent joint authoring, unchanged break thresholds')
parser.add_argument('--batched-io',action='store_true',help='GPU physics with batched efforts and native collider readback')
parser.add_argument('--legacy-command-uploads',action='store_true',help='A/B check: upload each articulation target separately')
parser.add_argument('--native-replication',action='store_true',help='Native clone/environment IDs for early GPU broadphase filtering')
parser.add_argument('--profile-init',action='store_true',help='Save cProfile of scene initialization and reset')
parser.add_argument('--profile-motion',action='store_true',help='Measure native simulate/fetch wall and CPU times; no extra sync')
parser.add_argument('--python-profile-motion',action='store_true',help='Diagnostic cProfile during motion; adds profiling overhead')
parser.add_argument('--neighbor-reset-interval',type=int,default=0,help='Diagnostic only: reset env_0 every N physics steps while other clones execute unchanged')
parser.add_argument('--legacy-reset',action='store_true',help='Use original per-slot global flushes for controlled comparison')
parser.add_argument('--gpu-partitions',type=int,default=8,choices=[1,2,4,8,16,32])
parser.add_argument('--clone-diagnostic',action='store_true',help='Record unit-aware per-step clone spread and compare batched reads with original native views')
parser.add_argument('--diagnostic-grid-spacing',type=float,default=0.,help='Diagnostic physical grid offset; not used by dataset runner')
parser.add_argument('--contact-view-diagnostic',action='store_true',help='Inspect collider-level CUDA contact view support')
parser.add_argument('--position-iterations',type=int,default=64,help='Diagnostic solver accuracy; clamp both scene limits')
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device='cpu')
args=parser.parse_args()
if not 1<=args.position_iterations<=255 or not 0<=args.velocity_iterations<=255:
    parser.error('Solver iteration count is out of range')
if args.stationary_steps<0 or args.max_control_steps<0 or args.num_envs<1:
    parser.error('Invalid step/environment count')
if args.neighbor_reset_interval and (args.neighbor_reset_interval<1 or args.num_envs<2 or not args.batched_io or args.stationary_steps):
    parser.error('neighbor-reset-interval requires batched motion with at least two environments')
if (args.zero_load or args.no_contacts or args.preserve_joints or args.joint_variant!='original') and not args.stationary_steps:
    parser.error('Physical isolation switches require --stationary-steps')
launcher=AppLauncher(args,limit_cpu_threads=8);app=launcher.app

import hashlib
import time
import traceback
import numpy as np
import torch
import carb
from pxr import UsdPhysics,PhysxSchema,PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface
from assets import build_robot
from harvest_env import HarvestEnvCfg
from dataset_scene import DatasetScene
from dataset_motion import execute_steps
from dataset_design import write_json,classify_result
from pose_candidates import DEFAULT_LIMITS
from pose_worker import state,state_comparison


def identity(slot):
    root=slot.root+'/Robot/link6/tcp/tomato_gripper'
    proximal=[slot.elastic.paths[i]+'/StemCollider' for i in slot.elastic.chains['TRUSS_Pedicel_proximal_'+slot.target_spec['name'][-2:]]]
    distal=slot.target_spec['path']+'/PedicelCollider'
    return dict(target_proximal_pedicels=proximal,target_distal_pedicel=distal,
        accepted_hook_pedicels=[distal,proximal[-1]],
        intended_ring_colliders=[root+f'/RingCollision/segment_{i:02d}' for i in range(8,24)])


def run():
    args.output.mkdir(parents=True,exist_ok=True)
    report=dict(mode=args.mode,num_envs=args.num_envs,complete=False,
        experiment_only=True,fully_vectorized_cuda_pipeline=False,tensor_device='cpu')
    report['arguments']={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
    report['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in Path(__file__).parent.glob('*.py')}
    def checkpoint(phase):
        report['phase']=phase;write_json(args.output/'report.json',report)
        print('[GPU PROBE]',args.mode,phase,flush=True)
    checkpoint('initializing')
    torch.set_num_threads(1)
    carb.settings.get_settings().set_int('/persistent/physics/numThreads',4)
    cfg=HarvestEnvCfg();cfg.sim.device='cuda:0' if args.cuda_tensors else 'cpu';cfg.scene.num_envs=1;cfg.curriculum='approach'
    if args.cuda_tensors and args.mode!='gpu':raise ValueError('CUDA tensors require GPU physics')
    report['tensor_device']=cfg.sim.device
    cfg.sim.use_fabric=args.fabric
    cfg.sim.dt=1/args.physics_hz;cfg.decimation=args.physics_hz//60;cfg.sim.render_interval=cfg.decimation
    cfg.sim.physx.min_position_iteration_count=args.position_iterations
    cfg.sim.physx.max_position_iteration_count=args.position_iterations
    cfg.sim.physx.min_velocity_iteration_count=args.velocity_iterations
    cfg.sim.physx.max_velocity_iteration_count=args.velocity_iterations
    cfg.probe_preserve_joints=args.preserve_joints
    cfg.gpu_legacy_reset=args.legacy_reset
    cfg.gpu_native_replication=args.native_replication
    cfg.gpu_batched_joint_commands=not args.legacy_command_uploads
    cfg.sim.physx.gpu_max_num_partitions=args.gpu_partitions
    cfg.sim.physx.solver_type=1 if args.solver=='tgs' else 0
    if args.zero_load:cfg.sim.gravity=(0.,0.,0.)
    cfg.sim.physx.enable_external_forces_every_iteration=not args.external_forces_once
    cfg.sim.physx.enable_ccd=args.mode=='cpu'
    cfg.probe_gpu_dynamics=args.mode=='gpu'
    cfg.position_jitter=0.;cfg.break_randomization=0.;cfg.plant_model='elastic';cfg.elastic_stiffness_scale=1.
    cfg.target_fruit='Tomato_05';cfg.stem_position=(-.75,.55,.32);cfg.stem_yaw=0.;cfg.stem_scale=.5
    cfg.lift_start_below=.4;cfg.lift_height_reference='mount';cfg.lift_speed=.1
    cfg.override_break_force=False;cfg.override_break_torque=False
    cfg.robot.actuators['lift'].velocity_limit_sim=.25
    cfg.dataset_physics_sync='optimized';cfg.dataset_max_steps=args.max_control_steps
    fixture=json.loads((args.fixture/'candidate.json').read_text())
    if fixture['target_id']!='Tomato_05':raise ValueError('Minimal probe requires a Tomato_05 fixture')
    commands=np.load(args.fixture/'planned_commands.npy')
    phases=[w['phase'] for w in fixture['commanded_waypoints'] for _ in range(w['steps'])]
    if len(commands)!=len(phases):raise RuntimeError('Fixture phase/command count mismatch')
    report['fixture']=dict(candidate_id=fixture['candidate_id'],path=str(args.fixture),
        commands_sha256=hashlib.sha256((args.fixture/'planned_commands.npy').read_bytes()).hexdigest(),
        initial_joints=fixture['robot_joint_positions_at_start'],planned_steps=len(commands))
    cfg.robot.spawn.usd_path=str(build_robot(args.output/'generated/rb5_ring.usda'))
    if args.joint_variant!='original':
        from gpu_joint_variants import install_joint_variant
        install_joint_variant(args.joint_variant)
    import cProfile
    profiler=cProfile.Profile() if args.profile_init else None
    if profiler:
        profiler.enable()
        import faulthandler
        faulthandler.dump_traceback_later(60,repeat=True)
    started=time.perf_counter();world=None;native_profile=None;motion_profiler=None
    try:
        if args.batched_io:
            cfg.gpu_diagnostic_grid_spacing=args.diagnostic_grid_spacing
            if args.mode!='gpu' or args.cuda_tensors:
                raise ValueError('Batched native-readback path requires GPU mode and CPU tensors')
            from gpu_dataset_scene import GpuDatasetScene
            world=GpuDatasetScene(cfg,args.num_envs)
            report['batched_io']=True
        else:
            world=DatasetScene(cfg,args.num_envs)
        context=world.sim.get_physics_context()
        report['backend']=dict(gpu_dynamics=context.is_gpu_dynamics_enabled(),
            command_uploads=('legacy' if args.legacy_command_uploads else 'batched') if args.batched_io else 'per_asset',
            broadphase=context.get_broadphase_type(),ccd=context.is_ccd_enabled(),
            gpu_max_num_partitions=context.get_gpu_max_num_partitions(),
            suppress_readback=world.sim.carb_settings.get('/physics/suppressReadback'),
            physics_dt_s=cfg.sim.dt,solver_position_iterations=args.position_iterations,
            solver=args.solver,solver_velocity_iterations=args.velocity_iterations,fabric=args.fabric,
            external_forces_every_iteration=cfg.sim.physx.enable_external_forces_every_iteration,
            articulation_count=len(world.scene.articulations),rigid_object_count=len(world.scene.rigid_objects))
        if bool(report['backend']['gpu_dynamics'])!=(args.mode=='gpu'):
            raise RuntimeError('Requested physics backend not active')
        for slot in world.slots:
            slot.start_q=torch.tensor([fixture['robot_joint_positions_at_start']],dtype=torch.float32,device=slot.device)
        if args.contact_view_diagnostic:
            from gpu_contacts import inspect_contact_views
            report['contact_views'] = inspect_contact_views(world)
            checkpoint('contact_views_created')
        checkpoint('reset_validation')
        world.reset_all();first=[state(s) for s in world.slots]
        world.reset_all();second=[state(s) for s in world.slots]
        report['repeated_reset']=[state_comparison(a,b) for a,b in zip(first,second)]
        if not all(v['passed'] for v in report['repeated_reset']):raise RuntimeError('Repeated reset mismatch')
        np.savez(args.output/'initial_state.npz',**second[0])
        if profiler:
            faulthandler.cancel_dump_traceback_later()
            profiler.disable();profiler.dump_stats(str(args.output/'initialization.pstats'))
        if args.clone_diagnostic:
            report['clone_properties']={}
            for kind,get_asset in [('robot',lambda s:s.robot),('plant',lambda s:s.elastic.articulation)]:
                for getter in ('get_masses','get_inertias','get_coms','get_dof_stiffnesses','get_dof_dampings','get_dof_armatures'):
                    ref=getattr(get_asset(world.slots[0]).root_physx_view,getter)()
                    report['clone_properties'][kind+':'+getter]=max(float(torch.max(torch.abs(getattr(get_asset(s).root_physx_view,getter)()-ref))) for s in world.slots)
        report['initialization_wall_s']=time.perf_counter()-started
        if args.stationary_steps:
            checkpoint('stationary_diagnostic')
            report['diagnostic']=dict(preserve_joints=args.preserve_joints,
                velocity_iterations=args.velocity_iterations,requested_steps=args.stationary_steps,
                solver=args.solver,zero_load=args.zero_load,no_contacts=args.no_contacts)
            if args.zero_load:
                for s in world.slots:
                    s.elastic.articulation.set_joint_effort_target(torch.zeros_like(s.elastic.preload))
                world.scene.write_data_to_sim()
            if args.no_contacts:
                for prim in world.scene.stage.Traverse():
                    if prim.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                get_physx_simulation_interface().flush_changes()
            initial=state(world.slots[0]);began=time.perf_counter()
            clone_history=[]
            from gpu_validation import check_clone
            for tick in range(args.stationary_steps):
                world.step()
                if args.clone_diagnostic and (tick < 16 or tick % 240 == 239):
                    states=[state(slot) for slot in world.slots]
                    checks=[check_clone(states[0],value,broken=bool(slot._broken[0] or slot._other_broken)) for slot,value in zip(world.slots,states)]
                    metrics={key:max(c.get('metrics',{}).get(key,0.) for c in checks) for key in checks[0].get('metrics',{})}
                    original_read_error={}
                    for slot in world.slots:
                        for name,asset in [('robot',slot.robot),('plant',slot.elastic.articulation)]:
                            view=asset.root_physx_view
                            if hasattr(view,'original'):
                                for getter in sorted(view.cache.GETTERS):
                                    delta=float(torch.max(torch.abs(getattr(view,getter)()-getattr(view.original,getter)())))
                                    key=name+':'+getter
                                    original_read_error[key]=max(original_read_error.get(key,0.),delta)
                    clone_history.append(dict(step=tick+1,passed=all(c['passed'] for c in checks),max_metrics=metrics,
                                              original_read_error=original_read_error))
                    write_json(args.output/'clone_history.json',clone_history)
                    if tick==15:
                        np.savez_compressed(args.output/'clones_step16.npz',**{key:np.stack([s[key] for s in states]) for key in states[0]})
                if tick % 240 == 239:
                    print('[GPU STATIONARY]',tick+1,'wall_s',time.perf_counter()-began,flush=True)
                if any(s._broken[0] or s._other_broken for s in world.slots):break
            if args.clone_diagnostic:
                report['clone_validation']=dict(passed=all(c['passed'] for c in clone_history) and tick+1==args.stationary_steps,
                    sampled_steps=[c['step'] for c in clone_history],requested_steps=args.stationary_steps,
                    scope='identical idle commands, original contact/drive/break settings unless explicitly disabled by diagnostic arguments')
            report['stationary']=dict(steps=tick+1,wall_s=time.perf_counter()-began,
                broken_envs=[s.index for s in world.slots if s._broken[0] or s._other_broken],
                events=world.slots[0].contact_diagnostics,
                state_change=state_comparison(initial,state(world.slots[0])))
            np.savez(args.output/'final_state.npz',**state(world.slots[0]))
            report['complete']=True;checkpoint('complete');return
        checkpoint('executing_same_commands')
        world.step_timings=dict(steps=0,write_s=0.,physics_s=0.,read_s=0.)
        active={slot.index:execute_steps(slot,dict(commands=commands,phases=phases),identity(slot),app,DEFAULT_LIMITS.copy()) for slot in world.slots}
        if args.profile_motion:
            from gpu_step_profile import NativeStepProfile
            native_profile=NativeStepProfile(context);native_profile.start()
        if args.python_profile_motion:
            motion_profiler=cProfile.Profile();motion_profiler.enable()
        rows=[];neighbor_resets=[];began=time.perf_counter();last=began;motion_started_unix=time.time()
        motion_cpu_started=time.process_time()
        while active:
            for index in list(active):
                try:next(active[index])
                except StopIteration as ended:
                    metrics,trace=ended.value
                    row=dict(env_index=index,**metrics,classification=classify_result(metrics,DEFAULT_LIMITS))
                    rows.append(row)
                    write_json(args.output/f'trace_{index}.json',trace)
                    write_json(args.output/f'contacts_{index}.json',world.slots[index].contact_diagnostics)
                    np.savez(args.output/f'final_state_{index}.npz',**state(world.slots[index]))
                    del active[index]
            if active:
                world.step()
                if args.neighbor_reset_interval and world.step_timings['steps'] % args.neighbor_reset_interval == 0:
                    before=[state(s) for s in world.slots[1:]]
                    world.reset_slot(0)
                    checks=[state_comparison(a,state(s)) for a,s in zip(before,world.slots[1:])]
                    restored=state_comparison(second[0],state(world.slots[0]))
                    neighbor_resets.append(dict(step=world.step_timings['steps'],restored=restored,neighbors=checks))
                    if not restored['passed'] or not all(c['passed'] for c in checks):
                        raise RuntimeError('Independent live reset modified physical state')
            if time.perf_counter()-last>15:
                print('[GPU PROBE PROGRESS]',args.mode,world.step_timings,flush=True);last=time.perf_counter()
        elapsed=time.perf_counter()-began
        report['motion_process_cpu_s']=time.process_time()-motion_cpu_started
        if args.neighbor_reset_interval:report['neighbor_resets']=neighbor_resets
        if native_profile:
            native_profile.stop();report['native_step_profile']=native_profile.report()
        if motion_profiler:
            motion_profiler.disable();motion_profiler.dump_stats(str(args.output/'motion.pstats'))
            import pstats
            with (args.output/'motion_profile.txt').open('w') as stream:
                pstats.Stats(motion_profiler,stream=stream).sort_stats('cumulative').print_stats(60)
        steps=world.step_timings['steps']
        report['motion']=dict(wall_s=elapsed,started_unix_s=motion_started_unix,physics_step_timings=world.step_timings.copy(),
            batch_sim_s=steps*cfg.sim.dt,sim_seconds_per_wall_second=steps*cfg.sim.dt/elapsed,
            outcomes=sorted(rows,key=lambda r:r['env_index']))
        # Independent native evidence that GPU buffers/articulations were used.
        import omni.usd
        from omni.physx.bindings._physx import acquire_physx_statistics_interface,PhysicsSceneStats
        stats=PhysicsSceneStats();scene_prim=next(p for p in world.scene.stage.Traverse() if p.IsA(UsdPhysics.Scene))
        ok=acquire_physx_statistics_interface().get_physx_scene_statistics(
            omni.usd.get_context().get_stage_id(),PhysicsSchemaTools.sdfPathToInt(scene_prim.GetPath()),stats)
        report['native_scene_statistics']=dict(available=bool(ok),**{name:getattr(stats,name) for name in
            ('nb_articulations','nb_active_constraints','gpu_mem_heap','gpu_mem_heap_articulation','gpu_mem_rigid_contact_count')})
        checkpoint('reset_after_contact')
        world.reset_all()
        report['reset_after_contact']=[state_comparison(a,state(s)) for a,s in zip(second,world.slots)]
        if not all(v['passed'] for v in report['reset_after_contact']):raise RuntimeError('Reset after contact mismatch')
        # Separate diagnostic, never part of the motion result. Exercise native
        # joint-break events by weakening only the target joint in this stage.
        checkpoint('native_joint_break_probe')
        slot=world.slots[0];joint=UsdPhysics.Joint(world.scene.stage.GetPrimAtPath(slot.joint_paths[0]))
        joint.GetBreakForceAttr().Set(1e-8);joint.GetBreakTorqueAttr().Set(1e-8)
        get_physx_simulation_interface().flush_changes()
        for tick in range(96):
            world.step()
            if slot._broken[0]:break
        report['joint_break_probe']=dict(target_break_event=bool(slot._broken[0]),physics_steps=tick+1,
            diagnostic_break_force=1e-8,diagnostic_break_torque=1e-8,
            events=[c for c in slot.contact_diagnostics if c['event']=='break'])
        world.reset_all()
        report['reset_after_break']=[state_comparison(a,state(s)) for a,s in zip(second,world.slots)]
        report['contact_routing_faults']=world.routing_faults
        report['complete']=True
        report['api_checks_passed']=bool(report['joint_break_probe']['target_break_event'] and
            len(report['joint_break_probe']['events'])==1 and
            all(v['passed'] for v in report['reset_after_break']))
        report['motion_equivalence_requires_cpu_comparison']=True
        checkpoint('complete')
    except BaseException as error:
        report['error']=dict(type=type(error).__name__,message=str(error))
        checkpoint('failed');raise
    finally:
        try:
            if motion_profiler:motion_profiler.disable()
            if native_profile:native_profile.stop()
        finally:
            if world is not None:world.close()


try:run()
except BaseException:
    traceback.print_exc()
finally:app.close()
