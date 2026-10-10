#!/usr/bin/env python3
"""Bounded native Isaac 6.1 actuation/elastic/contact trial, with physical state log."""
import argparse
import json
import time
import traceback
import shutil
import hashlib
import math
import os
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('assembly',type=Path);p.add_argument('--output',type=Path,required=True)
p.add_argument('--mode',choices=['idle','jaw','pulse','belt_control','harvest'],default='harvest')
p.add_argument('--pulse-force-n',type=float,default=.1,help='Diagnostic world +X force, 1..2 seconds, then release')
p.add_argument('--seconds',type=float,default=0.)
p.add_argument('--headless',action='store_true');p.add_argument('--hz',type=int,default=240)
p.add_argument('--physics-only',action='store_true',help='CPU worker: official minimal native bootstrap without RTX renderer; display runs separately')
p.add_argument('--device',choices=['cpu','cuda:0'],default='cpu')
p.add_argument('--broadphase',choices=['auto','MBP','SAP','GPU'],default='auto',help='Independent broadphase diagnostic; auto retains CPU MBP / GPU defaults; solver device remains unchanged')
p.add_argument('--preparation',choices=['direct','fold_then_turn','low_turn_then_lift','tight_fold_then_turn','tight_fold_open_approach'],default='fold_then_turn')
p.add_argument('--isolate-background',action='store_true',help='Component fixture only: disable background plant contacts; never a farm success')
p.add_argument('--performance',choices=['baseline','optimized'],default='optimized',help='Same physical model; control USD/display/callback overhead')
p.add_argument('--threads',type=int,default=16,help='Explicit native PhysX CPU dispatcher threads; 0 keeps installed setting for historical diagnostics')
p.add_argument('--display-fps',type=float,default=30.,help='Wall-clock display cap; physics timestep is unchanged')
p.add_argument('--num-envs',type=int,default=1,help='Identical full dynamic copies, sharing immutable farm obstacles')
p.add_argument('--replicate',action='store_true',help='Use replicated namespace also for the one-world control')
p.add_argument('--replication-source-copy',choices=['flatten','reference'],default='flatten',help='Diagnostic USD composition variant; native equivalence must pass before scaling')
p.add_argument('--replication-update-boundary',action='store_true',help='Flush pending Kit USD notices once after copying roots, before native initialization')
p.add_argument('--contact-backend',choices=['callback','tensor'],default='callback',help='Tensor sensor path needs actor-level/contact equivalence validation')
p.add_argument('--buffer-contact-events',action='store_true',help='Preserve per-step native guard; format raw callback contact evidence after measured loop')
p.add_argument('--record-contact-frames',action='store_true',help='Buffered diagnostic: save native world contact positions/normals for actual collider support inspection')
p.add_argument('--collision-filter',choices=['pairs','groups'],default='pairs',help='Groups compress the identical source exclusion graph; native A/B must pass before scaling')
p.add_argument('--position-iterations',type=int,default=64,help='Explicit solver convergence variant; validate contact/load/recovery before adopting')
p.add_argument('--velocity-iterations',type=int,default=4)
p.add_argument('--gpu-partitions',type=int,default=1,help='Native articulation GPU partitions; recorded explicitly for correctness/scaling checks')
p.add_argument('--solver',choices=['PGS','TGS'],default='PGS',help='Controlled native solver comparison; validate before adopting')
p.add_argument('--armature-setup',choices=['refresh','authored'],default='refresh',help='Authored validates the pre-warmup armature without a redundant runtime setter')
p.add_argument('--gravity-neutral-warmup',action='store_true',help='Initialize at authored rest under zero gravity, restore source gravity after fixed preload and before trial')
p.add_argument('--plant-dynamics',choices=['full','target-truss'],default='full',help='Target-truss freezes far-truss bending; different dynamics, same collision coverage')
p.add_argument('--zero-conveyor-diagnostic',action='store_true',help='Idle/pulse only: remove zero surface-velocity APIs to isolate native GPU fallback; not harvest evidence')
p.add_argument('--native-profile',action='store_true',help='Record native PhysX zones; diagnostic instrumentation may add overhead')
p.add_argument('--compiled-body-audit',action='store_true',help='CPU diagnostic: compare compiled mass/inertia, joint state and material mapping before trial')
p.add_argument('--publish-live',action='store_true',help='Publish latest measured env0 frame for a separate viewer; saved full physics history stays unchanged')
p.add_argument('--live-fps',type=float,default=10.,help='Latest-frame publication cap in wall time')
p.add_argument('--start-gate',type=Path,help='CPU pool diagnostic: publish ready.json and await the controller barrier')
p.add_argument('--gate-timeout',type=float,default=180.)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
if a.physics_only and a.device!='cpu':p.error('--physics-only is restricted to the validated CPU worker scope')
code_dir=a.output/'code';code_dir.mkdir()
code_hashes={}
for source in Path(__file__).parent.glob('*.py'):
    shutil.copy2(source,code_dir/source.name)
    code_hashes[source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
(a.output/'code_sha256.json').write_text(json.dumps(code_hashes,indent=2))
start_wall=time.perf_counter();exit_code=0
live=None;warmup=None
from motion_profile import MotionProfile
profile=MotionProfile(a.output/'profile.json')
from isaacsim import SimulationApp
from motion_app import create_motion_app
app=create_motion_app(SimulationApp,headless=a.headless,width=1280,height=800,
    performance=a.performance,physics_only=a.physics_only)
try:
    import numpy as np
    import torch
    import carb
    import omni.usd
    if not a.headless:import omni.kit.viewport.utility as vu
    extension_manager=app._app.get_extension_manager()
    (a.output/'app_extensions.json').write_text(json.dumps(dict(physics_only=a.physics_only,
        enabled=[x for x in extension_manager.get_extensions() if x.get('enabled')]),indent=2))
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools,Gf
    from scipy.spatial.transform import Rotation as R
    from isaacsim.core.simulation_manager import SimulationManager as sim
    from omni.physx import get_physx_simulation_interface,get_physx_interface
    from omni.physx.bindings._physx import SimulationEvent
    from motion_kinematics import ToolKinematics,plan_motion
    from motion_assets import ROBOT,sha

    manifest=json.loads((a.assembly/'motion_manifest.json').read_text())
    scene=a.assembly/'scene.usda'
    if sha(scene)!=manifest['scene_sha256']:raise ValueError('Scene hash mismatch')
    if not omni.usd.get_context().open_stage(str(scene.resolve())):raise RuntimeError('Stage open failed')
    stage=omni.usd.get_context().get_stage()
    if a.performance=='optimized':
        from motion_visuals import prepare_display_meshes
        (a.output/'display_schema_cleanup.json').write_text(json.dumps(prepare_display_meshes(stage),indent=2))
    belt_fixture=None
    if a.mode=='belt_control':
        from motion_belt_control import build_belt_control
        belt_fixture=build_belt_control(stage,manifest)
        (a.output/'belt_fixture.json').write_text(json.dumps(belt_fixture,indent=2))
    resolution=None
    if a.plant_dynamics=='target-truss':
        from motion_resolution import lock_far_trusses
        resolution=lock_far_trusses(stage,manifest)
        (a.output/'resolution.json').write_text(json.dumps(resolution,indent=2))
    if a.zero_conveyor_diagnostic:
        from motion_gpu_diagnostics import disable_zero_surface_velocity_for_diagnostic
        conveyor_diagnostic=disable_zero_surface_velocity_for_diagnostic(stage,mode=a.mode)
        (a.output/'zero_conveyor_diagnostic.json').write_text(json.dumps(conveyor_diagnostic,indent=2))
    isolated_count=0
    if a.collision_filter=='groups':
        from motion_collision_groups import compress_leaf_collision_policy
        compression=compress_leaf_collision_policy(stage,isolate_background=a.isolate_background)
        (a.output/'collision_filter_compression.json').write_text(json.dumps(compression,indent=2))
        if a.isolate_background:
            disabled=[str(prim.GetPath()) for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Greenhouse/Plants'),Usd.TraverseInstanceProxies())
                      if prim.HasAPI(UsdPhysics.CollisionAPI)]
            isolated_count=len(disabled)
            (a.output/'fixture_disabled_background_colliders.json').write_text(json.dumps(disabled))
    elif a.isolate_background:
        # Opt-in negative control, not the default farm trial. Preserve all
        # geometry and log the exact disabled paths. Originals are never saved.
        plants=stage.GetPrimAtPath('/World/Greenhouse/Plants')
        disabled=[str(prim.GetPath()) for prim in Usd.PrimRange(plants,Usd.TraverseInstanceProxies())
                  if prim.HasAPI(UsdPhysics.CollisionAPI)]
        bg=UsdPhysics.CollisionGroup.Define(stage,'/World/FixtureGroups/Background')
        bg.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Greenhouse/Plants'])
        active=UsdPhysics.CollisionGroup.Define(stage,'/World/FixtureGroups/Active')
        active.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/ElasticTarget','/World/ActuatedTool','/World/HarvestRobot'])
        bg.CreateFilteredGroupsRel().SetTargets([active.GetPath()])
        # The supplied dense decoration also intersects its own truss branches.
        # Diagnostic control excludes only appendage vs plant interactions;
        # arm/tool vs appendages, rods, fruits and house remain physical.
        appendages=[];other_plant=[]
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget')):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
            (appendages if '/Appendage_' in str(prim.GetPath()) else other_plant).append(prim)
        for leaf in appendages:
            rel=UsdPhysics.FilteredPairsAPI.Apply(leaf).CreateFilteredPairsRel()
            for other in other_plant:rel.AddTarget(other.GetPath())
        (a.output/'fixture_internal_leaf_filters.json').write_text(json.dumps(dict(
            leaves=[str(x.GetPath()) for x in appendages],other_colliders=[str(x.GetPath()) for x in other_plant],
            reason='component diagnostic only; authored own-leaf/branch overlap; not farm validity')))
        isolated_count=len(disabled)
        (a.output/'fixture_disabled_background_colliders.json').write_text(json.dumps(disabled))
    for _ in range(5):app.update()
    kin=ToolKinematics(stage)
    phases,planning=plan_motion(kin,np.array(manifest['target']['center']),preparation=a.preparation) if a.mode=='harvest' else ([],{})
    (a.output/'plan.json').write_text(json.dumps(planning,indent=2))
    envs=[dict(index=0,root='/World',robot=ROBOT+'/root_joint',plant='/World/ElasticTarget',
               tool='/World/ActuatedTool/fixed',target=manifest['target']['path'],
               target_joint=manifest['target']['joint'],belts=manifest['tool']['belts'],mapping={})]
    print('[MOTION] model loaded; initialize native PhysX',flush=True)
    # Register the existing composed scene before deprecated global setters.
    # Otherwise a minimal app can silently create a second /PhysicsScene and
    # apply dt/device to that empty scene rather than the farm's actual scene.
    sim.set_default_physics_scene('/World/PhysicsScene')
    sim.set_device(a.device);sim.set_physics_dt(1/a.hz);sim.enable_fabric(True)
    # The minimal worker may not yet have SimulationManager's stage-event cache.
    # set_device() then changes tensor readback but misses the authored GPU scene.
    # Enforce the requested native engine on the actual composed physics scene.
    from motion_app import configure_scene_device
    device_settings=configure_scene_device(stage,a.device,broadphase=a.broadphase)
    # Same native contact-report path as the existing GPU dataset runner.
    # GPU dynamics stays enabled; CPU readback is needed for collider callbacks.
    carb.settings.get_settings().set_bool('/physics/suppressReadback',a.contact_backend=='tensor' and a.device.startswith('cuda'))
    sim.set_backend('torch')
    if a.threads:
        get_physx_interface().set_thread_count(a.threads)
    if a.performance=='optimized':
        from omni.physx.bindings._physx import SETTING_UPDATE_TO_USD,SETTING_UPDATE_VELOCITIES_TO_USD
        carb.settings.get_settings().set_bool(SETTING_UPDATE_TO_USD,False)
        carb.settings.get_settings().set_bool(SETTING_UPDATE_VELOCITIES_TO_USD,False)
        if a.headless:
            # update_fabric=False does not disable the native output buffers.
            # Disable both only when no viewport consumes them; native tensor
            # poses/contact reports and saved state remain available.
            carb.settings.get_settings().set_bool('/physics/fabricUpdateTransformations',False)
            carb.settings.get_settings().set_bool('/physics/fabricUpdateVelocities',False)
    # Allocation only: prevent silent missed broad-phase pairs in this dense farm.
    scene_api=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/World/PhysicsScene'))
    scene_api.CreateGpuFoundLostAggregatePairsCapacityAttr(16384)
    scene_api.CreateGpuTotalAggregatePairsCapacityAttr(16384)
    scene_api.CreateGpuMaxNumPartitionsAttr(a.gpu_partitions)
    scene_api.CreateSolverTypeAttr(a.solver)
    events=[];step_events=[];current_time=0.;phase='idle'
    if a.buffer_contact_events and a.contact_backend!='callback':
        raise ValueError('Buffered contact evidence requires the native callback backend')
    if a.record_contact_frames and not a.buffer_contact_events:
        raise ValueError('Contact frame recording requires --buffer-contact-events')
    path_cache={}
    def path(v):
        if a.performance=='baseline':return str(PhysicsSchemaTools.intToSdfPath(v))
        if v not in path_cache:path_cache[v]=str(PhysicsSchemaTools.intToSdfPath(v))
        return path_cache[v]
    contact_buffer=None
    if a.buffer_contact_events:
        from motion_contact_buffer import BufferedContactRecorder
        contact_buffer=BufferedContactRecorder(path,a.hz,capture_frames=a.record_contact_frames)
    def contact(headers,data):
        contact_started=time.perf_counter()
        if contact_buffer is not None:
            contact_buffer.record(headers,data,t=current_time,phase=phase)
        else:
            for h in headers:
                left,right=path(h.collider0),path(h.collider1)
                for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    c=data[i]
                    e=dict(t=current_time,phase=phase,a=left,b=right,
                           separation_m=float(c.separation),force_n=math.sqrt(sum(float(x)**2 for x in c.impulse))*a.hz,
                           impulse_ns=list(c.impulse))
                    events.append(e);step_events.append(e)
        profile.total['contact_callback']+=time.perf_counter()-contact_started
        profile.count['contact_callback']+=1
    sub=get_physx_simulation_interface().subscribe_contact_report_events(contact) if a.contact_backend=='callback' else None
    breaks=[]
    def on_event(event):
        if event.type==int(SimulationEvent.JOINT_BREAK):
            breaks.append(dict(t=current_time,phase=phase,joint=str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath']))))
    break_sub=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art=PhysxSchema.PhysxArticulationAPI.Apply(prim)
            art.CreateSolverPositionIterationCountAttr(a.position_iterations)
            art.CreateSolverVelocityIterationCountAttr(a.velocity_iterations)
        if prim.IsA(UsdPhysics.Joint) and str(prim.GetPath()).startswith('/World/ElasticTarget/Joints/'):
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr(.0005)
    if a.num_envs>1 or a.replicate:
        from motion_replication import replicate_motion
        replication=replicate_motion(stage,manifest,a.num_envs,device=a.device,source_copy=a.replication_source_copy)
        envs=replication['environments']
        (a.output/'replication.json').write_text(json.dumps(replication,indent=2))
        if a.replication_update_boundary:app.update()
    # Effective settings are recorded separately from the immutable assembly.
    (a.output/'runtime_settings.json').write_text(json.dumps(dict(hz=a.hz,solver=a.solver,armature_setup=a.armature_setup,
        gravity_neutral_warmup=a.gravity_neutral_warmup,
        plant_armature=.0005,aggregate_pair_capacities=16384,initialization='armature authored before warmup',
        source_scene_sha256=sha(scene),device=a.device,isolate_background=a.isolate_background,
        native_scene_device=device_settings,
        disabled_background_colliders=isolated_count,performance=a.performance,threads=a.threads,
        display_fps=a.display_fps,num_envs=a.num_envs,contact_backend=a.contact_backend,collision_filter=a.collision_filter,
        buffer_contact_events=a.buffer_contact_events,
        record_contact_frames=a.record_contact_frames,
        fabric_transform_output=carb.settings.get_settings().get('/physics/fabricUpdateTransformations'),
        fabric_velocity_output=carb.settings.get_settings().get('/physics/fabricUpdateVelocities'),
        plant_dynamics=a.plant_dynamics,zero_conveyor_diagnostic=a.zero_conveyor_diagnostic,
        position_iterations=a.position_iterations,velocity_iterations=a.velocity_iterations,
        gpu_partitions=a.gpu_partitions),indent=2))
    if a.gravity_neutral_warmup:
        from motion_initialization import GravityNeutralWarmup
        warmup=GravityNeutralWarmup(stage,'/World/PhysicsScene');warmup.__enter__()
    load=time.perf_counter();sim.initialize_physics()
    effective_dt=float(sim.get_physics_dt())
    native_scenes=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api=PhysxSchema.PhysxSceneAPI(prim)
            native_scenes.append(dict(path=str(prim.GetPath()),
                enabled_gpu_dynamics=api.GetEnableGPUDynamicsAttr().Get(),
                broadphase_type=api.GetBroadphaseTypeAttr().Get(),
                steps_per_second=api.GetTimeStepsPerSecondAttr().Get(),
                solver=api.GetSolverTypeAttr().Get()))
    (a.output/'native_scene_state.json').write_text(json.dumps(dict(
        requested_device=a.device,requested_broadphase=a.broadphase,requested_dt_s=1/a.hz,
        effective_step_dt_s=effective_dt,scenes=native_scenes),indent=2))
    if not math.isclose(effective_dt,1/a.hz,rel_tol=0.,abs_tol=1e-12):
        raise RuntimeError('Native simulation timestep differs from requested timestep')
    if len(native_scenes)!=1 or native_scenes[0]['path']!='/World/PhysicsScene':
        raise RuntimeError('Unexpected native physics scene inventory: '+str(native_scenes))
    if native_scenes[0]['steps_per_second']!=a.hz:
        raise RuntimeError('Farm native scene Hz differs from requested Hz')
    if any(native_scenes[0][key]!=device_settings[key] for key in ('enabled_gpu_dynamics','broadphase_type')):
        raise RuntimeError('Native initialization changed requested dynamics/broadphase: '+str(native_scenes[0]))
    view=sim.get_physics_sim_view()
    def arr(v):
        if hasattr(v,'detach'):return v.detach().cpu().numpy()
        if hasattr(v,'numpy'):return v.numpy()
        return np.asarray(v)
    def tensor(x,dtype=torch.float32):return torch.as_tensor(x,dtype=dtype,device=sim.get_physics_sim_device())
    robot=view.create_articulation_view([e['robot'] for e in envs])
    plant=view.create_articulation_view([e['plant'] for e in envs])
    print('[MOTION] views',robot.count,plant.count,'backend',type(view),flush=True)
    if robot.count!=a.num_envs or plant.count!=a.num_envs:raise RuntimeError('Articulation identity mismatch')
    print('[MOTION] robot metadata',robot.shared_metatype.dof_names,flush=True)
    print('[MOTION] plant metadata',plant.shared_metatype.dof_names[:12],flush=True)
    print('[MOTION] plant links',plant.shared_metatype.link_names[:10],flush=True)
    print('[MOTION] native spring stiffness',arr(plant.get_dof_stiffnesses())[0,:9].tolist(),flush=True)
    ids=tensor(list(range(a.num_envs)),torch.int32)
    q0=arr(robot.get_dof_positions()).copy()
    names=list(robot.shared_metatype.dof_names)
    rids=[names.index(n) for n in kin.names]
    jawids=[names.index('jaw_L'),names.index('jaw_R')]
    target_tensor=tensor(q0)
    robot.set_dof_position_targets(target_tensor,ids)
    if hasattr(plant,'set_dof_armatures'):
        armatures=plant.get_dof_armatures()
        if a.armature_setup=='refresh':
            plant.set_dof_armatures(torch.full_like(armatures,.0005),ids.cpu())
        elif not np.allclose(arr(armatures),.0005,rtol=0.,atol=1e-10):
            raise RuntimeError('Authored native armature does not match 0.0005')
    # Reuse ElasticPlant's fixed authored-pose gravity preload, not continuous cancellation.
    jac=arr(plant.get_jacobians())[0];mass=arr(plant.get_masses())[0]
    preload=np.zeros(plant.max_dofs)
    for i,m in enumerate(mass):
        if i>0:preload-=jac[i-1,:3].T@np.array([0.,0.,-9.81*m])
    links=list(plant.shared_metatype.link_names)
    for f in manifest['plant']['fruits']:
        i=links.index(f['anchor'].rsplit('/',1)[1]);anchor=next(b for b in manifest['plant']['bodies'] if b['path']==f['anchor'])
        force=np.array([0.,0.,-9.81*f['mass']]);offset=np.array(f['center'])-anchor['rest_position']
        preload-=jac[i-1,:3].T@force+jac[i-1,3:].T@np.cross(offset,force)
    preload_tensor=tensor(np.broadcast_to(preload,(a.num_envs,len(preload))).copy())
    plant.set_dof_actuation_forces(preload_tensor,ids)
    if warmup is not None:
        warmup.restore()
        (a.output/'warmup.json').write_text(json.dumps(warmup.metadata,indent=2))
    allpaths=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):allpaths.append(str(prim.GetPath()))
    bodies=view.create_rigid_body_view(allpaths)
    body_paths=list(bodies.prim_paths)
    initial=arr(bodies.get_transforms()).copy()
    if a.compiled_body_audit:
        from motion_compiled_audit import collect_compiled_body_audit
        reverse_mapping={new:old for old,new in envs[0]['mapping'].items()}
        compiled=collect_compiled_body_audit(stage,body_paths,path_mapping=reverse_mapping,
            articulation_views={'plant':plant,'robot':robot},device=a.device)
        (a.output/'compiled_body_audit.json').write_text(json.dumps(compiled,indent=2))
    toolids=[body_paths.index(e['tool']) for e in envs];targetids=[body_paths.index(e['target']) for e in envs]
    toolid=toolids[0];targetid=targetids[0]
    control_id=body_paths.index(belt_fixture['body_path']) if belt_fixture and a.num_envs==1 and not a.replicate else None
    tensor_contacts=None
    if a.contact_backend=='tensor':
        from motion_contacts import TensorContactReader
        tensor_contacts=TensorContactReader(view,body_paths,max_contact_data_count=max(16384,a.num_envs*16384))
        initial_tensor_events=tensor_contacts.read(t=0.,phase='initialization',dt=1/a.hz)
        events.extend(initial_tensor_events)
    initialization_events=[e for e in (contact_buffer.materialize() if contact_buffer is not None else events) if e['t']==0.]
    initialization_invalid=[e for e in initialization_events if e['separation_m']<-.0005 and
        any(x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/'))]
    if tensor_contacts is not None and tensor_contacts.last_summary.get('raw_minimum_separation_m') is not None:
        if tensor_contacts.last_summary['raw_minimum_separation_m']<-.0005 and not initialization_invalid:
            initialization_invalid=[dict(raw_sensor_summary=tensor_contacts.last_summary)]
    (a.output/'initialization_validity.json').write_text(json.dumps(dict(
        valid=not initialization_invalid,physics_warmup_steps=2,timestamp_scope='native initialize_physics before trial clock',
        reported_contact_points=len(initialization_events),violations=initialization_invalid,
        native_break_events=breaks),indent=2))
    actual=initial[toolid];expected=kin.fk(q0[0,rids])
    fk_error=float(np.linalg.norm(actual[:3]-expected[:3,3]))
    if fk_error>1e-4:raise RuntimeError(f'USD FK/native pose mismatch {fk_error}')
    (a.output/'native_metadata.json').write_text(json.dumps(dict(
        robot_dofs=names,plant_dofs=list(plant.shared_metatype.dof_names),body_paths=body_paths,
        plant_links=links,preload=preload.tolist(),fk_error_m=fk_error,
        model_sha256=sha(scene),physics_initialization_s=time.perf_counter()-load),indent=2))
    total=a.seconds or (sum(x[1] for x in phases)+2 if phases else 7.)
    trace=[initial.copy()];qtrace=[q0[0].copy()];times=[0.];phases_trace=['initial'];command=[q0[0].copy()]
    stop_reason='initialization_penetration_over_0_5mm' if initialization_invalid else None
    batch_qtrace=[q0.copy()]
    belt_commands=[(0.,0.)];force_commands=[(0.,0.,0.)]
    target=envs[0]['target'];last_phase=None
    belts=[PhysxSchema.PhysxSurfaceVelocityAPI(stage.GetPrimAtPath(x)) for e in envs for x in e['belts']]
    belt_attrs=[b.CreateSurfaceVelocityAttr() for b in belts];last_speed=None
    if a.performance=='optimized' and all(attr.Get()==Gf.Vec3f(0) for attr in belt_attrs):
        # Retain the already compiled zero command instead of sending a USD
        # notice for an unchanged surface property on the first trial step.
        last_speed=(0.,0.)
    # Use the existing broad rigid-body view. The native GPU one-body view
    # raised a misaligned-address error for this externally attached fruit.
    # A full persistent buffer preserves the same world force at exactly the
    # selected fruit rows and makes the native mapping unambiguous.
    force_tensor=tensor(np.zeros((len(body_paths),3))) if a.mode=='pulse' else None
    force_ids=tensor(list(range(len(body_paths))),torch.int32) if force_tensor is not None else None
    visuals=None
    if not a.headless:
        from motion_visuals import DeformationVisuals
        visuals=DeformationVisuals(stage,a.assembly,body_paths)
        vu.get_active_viewport().set_active_camera('/World/RobotCamera')
    print('[MOTION] physics ready',len(body_paths),'bodies',total,'sim seconds',flush=True)
    native_profile=None
    if a.native_profile:
        from omni.physx import get_physx_benchmarks_interface
        native_profile=get_physx_benchmarks_interface();native_profile.enable_profile(True)
    if a.publish_live:
        from motion_live import MotionLiveWriter
        live=MotionLiveWriter(a.output,a.assembly,body_paths)
        live.publish(initial,.044+float(q0[0,jawids[1]]-q0[0,jawids[0]]),0.)
    if a.start_gate is not None:
        if a.device!='cpu':raise ValueError('Synchronized process pool is restricted to CPU')
        (a.output/'ready.json').write_text(json.dumps(dict(pid=os.getpid(),model_sha256=sha(scene),
            initial_valid=not initialization_invalid,ready_monotonic_s=time.perf_counter()),indent=2))
        waiting=time.perf_counter()
        while not a.start_gate.exists():
            if time.perf_counter()-waiting>a.gate_timeout:raise TimeoutError('CPU pool start barrier timed out')
            time.sleep(.05)
    started=time.perf_counter()
    last_display=-float('inf')
    last_publish=-float('inf')
    poses=initial.copy();q=q0.copy();current_time=0.;tick=-1
    for tick in range(0 if initialization_invalid else round(total*a.hz)):
        control_started=time.perf_counter()
        t=tick/a.hz;phase='settle';qcommand=q0.copy();gap=.044;speed=(0.,0.)
        pulse_force=np.zeros((a.num_envs,3))
        if a.mode=='pulse':
            phase='load' if 1<=t<2 else 'release' if t>=2 else 'settle'
            if 1<=t<2:pulse_force[:,0]=a.pulse_force_n
            if a.performance=='baseline' or np.any(pulse_force):
                force_tensor.zero_()
                force_tensor[targetids,0]=a.pulse_force_n if 1<=t<2 else 0.
                bodies.apply_forces(force_tensor,force_ids,True)
        elif a.mode=='jaw' and t>1:
            phase='jaw_open_close';gap=.044+.016*np.sin((t-1)*np.pi/2)
            if 5<=t<6:phase='belt_infeed_unloaded';speed=(-.02,-.02)
            elif t>=6:phase='belt_opposed_unloaded';speed=(-.015,.015)
        elif a.mode=='belt_control':
            closed_gap=belt_fixture['command_closed_gap_m']
            if t<1:phase='control_close';gap=.044+float(closed_gap-.044)*t
            elif t<3:phase='control_infeed';gap=closed_gap;speed=(-.02,-.02)
            elif t<4:phase='control_opposed';gap=closed_gap;speed=(-.015,.015)
            else:phase='control_hold';gap=closed_gap
        elif a.mode=='harvest' and t>=2:
            offset=t-2;previous_gap=.044
            for label,duration,qs,nextgap,lvel,rvel in phases:
                if offset<duration:
                    phase=label;u=offset/duration;u=u*u*(3-2*u)
                    f=u*(len(qs)-1);lo=min(int(f),len(qs)-2)
                    qcommand[0,rids]=qs[lo]*(1-(f-lo))+qs[lo+1]*(f-lo)
                    gap=previous_gap+(nextgap-previous_gap)*u;speed=(lvel,rvel);break
                offset-=duration;previous_gap=nextgap
        if a.num_envs>1:qcommand[1:]=qcommand[0]
        qcommand[:,jawids]=[-(gap-.044)/2,(gap-.044)/2]
        target_tensor.copy_(tensor(qcommand));robot.set_dof_position_targets(target_tensor,ids)
        if a.performance=='baseline' or speed!=last_speed:
            with profile.section('belt_usd_write'):
                for i,attr in enumerate(belt_attrs):attr.Set(Gf.Vec3f(0,float(speed[i%2]),0))
            last_speed=speed
        profile.total['control']+=time.perf_counter()-control_started;profile.count['control']+=1
        step_events.clear();current_time=(tick+1)/a.hz
        if contact_buffer is not None:contact_buffer.clear_step()
        if tick<2:profile.checkpoint(tick,'before_simulate')
        with profile.section('simulate'):
            sim.step(update_fabric=a.performance=='baseline')
        with profile.section('fetch'):
            get_physx_simulation_interface().fetch_results()
        if tensor_contacts is not None:
            with profile.section('tensor_contacts'):
                step_events=tensor_contacts.read(t=current_time,phase=phase,dt=1/a.hz);events.extend(step_events)
                if not tensor_contacts.last_summary.get('complete',False):
                    raise RuntimeError('Incomplete tensor contact mapping: '+str(tensor_contacts.last_summary))
        if contact_buffer is not None:step_events=contact_buffer.step_invalid_events()
        with profile.section('readback'):
            poses=arr(bodies.get_transforms()).copy();q=arr(robot.get_dof_positions()).copy()
        trace.append(poses);qtrace.append(q[0]);times.append(current_time);phases_trace.append(phase);command.append(qcommand[0])
        batch_qtrace.append(q.copy())
        belt_commands.append(speed);force_commands.append(tuple(pulse_force[0]))
        if live is not None and time.perf_counter()-last_publish>=1/a.live_fps:
            with profile.section('live_publish'):
                live.publish(poses,.044+float(q[0,jawids[1]]-q[0,jawids[0]]),current_time)
            last_publish=time.perf_counter()
        if not np.isfinite(poses).all():stop_reason='nonfinite_state';break
        if phase!=last_phase:print('[MOTION]',round(t,3),phase,flush=True);last_phase=phase
        # Report all contacts; stop on robot or active-target gross penetration.
        invalid=[e for e in step_events if e['separation_m']<-.0005 and
                 any(x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/'))]
        if invalid:
            stop_reason='penetration_over_0_5mm';print('[MOTION] stop',invalid[:2],flush=True);break
        raw_min=tensor_contacts.last_summary.get('raw_minimum_separation_m') if tensor_contacts is not None else None
        if raw_min is not None and raw_min<-.0005:
            stop_reason='penetration_over_0_5mm';print('[MOTION] stop raw sensor separation',tensor_contacts.last_summary,flush=True);break
        draw=(tick%max(1,round(a.hz/a.display_fps))==0) if a.performance=='baseline' else (time.perf_counter()-last_display>=1/a.display_fps)
        if visuals is not None and draw:
            profile.checkpoint(tick,'before_visual_update')
            with profile.section('visual_update'):
                visuals.update(poses,.044+float(q[0,jawids[1]]-q[0,jawids[0]]))
            if a.performance=='optimized':
                from omni.physxfabric import get_physx_fabric_interface
                with profile.section('fabric_display'):
                    get_physx_fabric_interface().update(current_time,1/a.hz)
            profile.checkpoint(tick,'before_app_update')
            with profile.section('app_update'):app.update()
            last_display=time.perf_counter()
        if tick%(a.hz*2)==0:
            print('[MOTION] progress',round(t,2),'target displacement mm',np.linalg.norm(poses[targetid,:3]-initial[targetid,:3])*1000,flush=True)
            profile.checkpoint(tick,'progress')
    loop_finished=time.perf_counter();loop_wall=loop_finished-started
    if contact_buffer is not None:
        events=contact_buffer.materialize()
        (a.output/'contact_buffer.json').write_text(json.dumps(contact_buffer.metadata(),indent=2))
    if live is not None:
        live.publish(poses,.044+float(q[0,jawids[1]]-q[0,jawids[0]]),current_time)
        live.close(stop_reason)
    if native_profile is not None:
        (a.output/'native_profile.json').write_text(json.dumps(native_profile.get_profile_stats(),indent=2,default=str))
        native_profile.enable_profile(False)
    profile.checkpoint(tick,'physics_complete')
    np.savez_compressed(a.output/'states.npz',time=np.array(times),poses=np.array(trace),
                        robot_q=np.array(qtrace),command=np.array(command),phase=np.array(phases_trace),body_paths=np.array(body_paths),
                        all_environment_robot_q=np.array(batch_qtrace),
                        belt_surface_velocity_local_y_m_s=np.array(belt_commands),external_force_world_n=np.array(force_commands))
    (a.output/'contacts.json').write_text(json.dumps(events))
    (a.output/'break_events.json').write_text(json.dumps(breaks))
    states=np.array(trace);qc=np.array(qtrace)
    target_events=[e for e in events if e['a']==target or e['b']==target or target+'/' in e['a'] or target+'/' in e['b']]
    summary=dict(mode=a.mode,scene=str(scene),model_sha256=sha(scene),training_eligible=False,
                 physics_steps=len(times)-1,simulated_s=times[-1] if times else 0,physics_wall_s=loop_wall,
                 loop_started_monotonic_s=started,loop_finished_monotonic_s=loop_finished,
                 save_wall_s=time.perf_counter()-started-loop_wall,performance=a.performance,threads=a.threads,
                 num_envs=a.num_envs,aggregate_simulated_s=times[-1]*a.num_envs,
                 aggregate_sim_s_per_wall_s=times[-1]*a.num_envs/loop_wall,contact_backend=a.contact_backend,collision_filter=a.collision_filter,
                 contact_evidence_scope='exact collider impulse norm' if tensor_contacts is None else 'actor-level normal-only force; friction excluded; native A/B pending',
                 plant_dynamics=a.plant_dynamics,native_plant_dofs=plant.max_dofs,
                 zero_conveyor_diagnostic=a.zero_conveyor_diagnostic,
                 belt_control=None if control_id is None else dict(
                     transport_m=float(np.dot(states[-1,control_id,:3]-initial[control_id,:3],belt_fixture['transport_axis_world'])),
                     displacement_world_m=(states[-1,control_id,:3]-initial[control_id,:3]).tolist(),
                     fixture_only=True,harvest_success=False),
                 profile={name:dict(total_s=value,count=profile.count[name],max_s=profile.maximum[name]) for name,value in profile.total.items()},
                 completed=stop_reason is None,stop_reason=stop_reason,harvest_success=None,
                 initialization_valid=not initialization_invalid,
                 native_target_detached=any(e['joint']==envs[0]['target_joint'] for e in breaks),
                 fixture_only=a.isolate_background,disabled_background_colliders=isolated_count,
                 native_break_events=breaks,physics_settings=dict(hz=a.hz,solver=a.solver,armature=.0005,device=a.device,
                     position_iterations=a.position_iterations,velocity_iterations=a.velocity_iterations,
                     gpu_partitions=a.gpu_partitions,
                     gpu_found_lost_aggregate_capacity=16384,gpu_total_aggregate_capacity=16384),
                 jaw_actual_gap_mm=(44+1000*(qc[:,jawids[1]]-qc[:,jawids[0]])).tolist(),
                 target_displacement_max_m=float(np.linalg.norm(states[:,targetid,:3]-initial[targetid,:3],axis=1).max()),
                 target_force_max_n=max((e['force_n'] for e in target_events),default=0),
                 force_metric='maximum norm of one contact point impulse / physics dt; not summed force or damage limit',
                 target_displacement_final_m=float(np.linalg.norm(states[-1,targetid,:3]-initial[targetid,:3])),
                 per_environment=[dict(index=i,target=e['target'],
                     target_displacement_max_m=float(np.linalg.norm(states[:,targetids[i],:3]-initial[targetids[i],:3],axis=1).max()),
                     target_displacement_final_m=float(np.linalg.norm(states[-1,targetids[i],:3]-initial[targetids[i],:3]))) for i,e in enumerate(envs)],
                 robot_tracking_max_m_or_rad=np.max(np.abs(qc-np.array(command)),axis=0).tolist(),
                 contact_events=len(events),min_contact_separation_m=min((e['separation_m'] for e in events),default=None),
                 limitations=['Not a validated grasp/detach detector','No real fruit damage calibration',
                              'Motion diagnostic; dynamic belt-loop deformation not modeled'])
    (a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print('[MOTION] saved',a.output,stop_reason,flush=True)
except Exception:
    exit_code=1;error=traceback.format_exc();print(error,flush=True)
    (a.output/'runtime_error.json').write_text(json.dumps(dict(error=error,elapsed_s=time.perf_counter()-start_wall),indent=2))
finally:
    if warmup is not None:warmup.restore()
    if live is not None and not live.closed:live.close('runtime_error')
    app.close(exit_code=exit_code)
