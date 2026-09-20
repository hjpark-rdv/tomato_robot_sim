"""One fixed-scene candidate per fresh Isaac/PhysX process."""
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from hook_motion import RobotKinematics
from geometry import RING_RADIUS,WIRE_RADIUS
from pose_candidates import candidate_waypoints,classify,DEFAULT_LIMITS,rear_capsule_geometry


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def state(env):
    return dict(robot_q=env.robot.data.joint_pos.cpu().numpy().copy(),
        robot_dq=env.robot.data.joint_vel.cpu().numpy().copy(),
        robot_root=env.robot.data.root_state_w.cpu().numpy().copy(),
        elastic_q=env.elastic.articulation.data.joint_pos.cpu().numpy().copy(),
        elastic_dq=env.elastic.articulation.data.joint_vel.cpu().numpy().copy(),
        elastic_bodies=env.elastic.articulation.data.body_state_w.cpu().numpy().copy(),
        fruits=np.stack([f.data.root_state_w.cpu().numpy().copy() for f in env.fruits]),
        preload=env.elastic.preload.cpu().numpy().copy(),
        stiffness=env.elastic.articulation.data.joint_stiffness.cpu().numpy().copy(),
        damping=env.elastic.articulation.data.joint_damping.cpu().numpy().copy())


def state_comparison(a,b):
    differences={k:float(np.max(np.abs(a[k]-b[k]))) for k in a}
    return dict(passed=all(v<=1e-6 for v in differences.values()),max_abs_by_field=differences,tolerance=1e-6)


def rgbd_snapshot(env,directory,center):
    import omni.replicator.core as rep
    from omni.physx import get_physx_interface
    from pxr import UsdGeom,Gf
    import imageio.v2 as imageio
    center=np.asarray(center,dtype=float)
    eye=center+np.array([.28,-.28,.16]);target=center.copy()
    camera=UsdGeom.Camera.Define(env.stage,'/World/PoseDatasetCamera')
    focal,aperture=24.,20.955
    camera.CreateFocalLengthAttr(focal);camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(aperture*.75);camera.CreateClippingRangeAttr(Gf.Vec2f(.01,10.))
    view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1))
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(view.GetInverse())
    product=rep.create.render_product(str(camera.GetPath()),(960,720))
    rgb=rep.AnnotatorRegistry.get_annotator('rgb');depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
    rgb.attach(product);depth.attach(product)
    env.elastic.sync_visuals();env.sim.physics_sim_view.update_articulations_kinematic()
    get_physx_interface().update_transformations(False,True,True)
    try:
        for _ in range(10): env.sim.render()
        pixels=rgb.get_data()[:,:,:3].copy();z=depth.get_data().copy().squeeze().astype(np.float32)
        if pixels.shape!=(720,960,3) or z.shape!=(720,960): raise RuntimeError('Invalid RGB-D dimensions')
        valid=np.isfinite(z)&(z>0)&(z<10.)
        if valid.mean()<.05: raise RuntimeError('Empty depth capture')
        imageio.imwrite(directory/'rgb.png',pixels);np.save(directory/'depth.npy',z)
        imageio.imwrite(directory/'depth_valid.png',valid.astype(np.uint8)*255)
    finally:
        rgb.detach(product);depth.detach(product);product.destroy()
    usd_pose=np.asarray(view.GetInverse()).T;optical_pose=usd_pose.copy();optical_pose[:3,1:3]*=-1
    return dict(rgb='rgb.png',depth='depth.npy',depth_valid='depth_valid.png',depth_units='metres',
        depth_definition='distance_to_image_plane: optical +Z, not range',width=960,height=720,
        K=[[960*focal/aperture,0,480],[0,960*focal/aperture,360],[0,0,1]],
        world_from_camera_optical=optical_pose.tolist(),eye_world=eye.tolist(),look_at_world=target.tolist(),
        valid_fraction=float(valid.mean()),rgb_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
        depth_sha256=hashlib.sha256(z.tobytes()).hexdigest())


def structure(env,directory):
    from pxr import UsdGeom,UsdPhysics
    spec=env.target_spec;root='/World/envs/env_0/Robot/link6/tcp/tomato_gripper'
    distal=spec['path']+'/PedicelCollider'
    proximal=[env.elastic.paths[i]+'/StemCollider' for i in env.elastic.chains['TRUSS_Pedicel_proximal_05']]
    ring=[root+f'/RingCollision/segment_{i:02d}' for i in range(32)]
    for path in [distal,*proximal,*ring]:
        prim=env.stage.GetPrimAtPath(path)
        if not prim or not prim.IsA(UsdGeom.Capsule) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            raise ValueError('Required live physics capsule missing: '+path)
    joint=UsdPhysics.FixedJoint(env.stage.GetPrimAtPath(env.joint_paths[0]))
    report=dict(scene=env.provenance,target_prim=spec['path'],target_fruit_collider=spec['path']+'/FruitCollider',
        target_distal_pedicel=distal,target_proximal_pedicels=proximal,
        accepted_hook_pedicels=[distal,proximal[-1]],
        hook_prim=root,ring_colliders=ring,intended_ring_colliders=ring[8:24],
        intended_region='rear 135..225 degree arc of the CAD half ring; segment 08..23',
        joint_path=env.joint_paths[0],joint_body0=[str(p) for p in joint.GetBody0Rel().GetTargets()],
        joint_body1=[str(p) for p in joint.GetBody1Rel().GetTargets()],
        break_force_N=float(joint.GetBreakForceAttr().Get()),break_torque_Nm=float(joint.GetBreakTorqueAttr().Get()),
        ring_radius_m=RING_RADIUS,wire_radius_m=WIRE_RADIUS,
        success_definition='prior aperture insertion + rear-arc native target contact + 1 s retained geometry and >=3 hold-contact ticks; stable last half-second; no break, dangerous/non-target-first contact or excess displacement',
        success_scope='operational stable hook in current collision proxy model; not a proof of real harvesting')
    write_json(directory/'structure.json',report)
    return report


def plan(env,kin,checker,params):
    center,neck,_=[v[0].cpu().numpy() for v in env._target_geometry()]
    rotation,waypoints,direction=candidate_waypoints(center,neck,params)
    start=env.robot.data.joint_pos[0].cpu().numpy().copy();q=start.copy();position,orientation=kin.fk(q)
    commands=[];phases=[];diagnostics=[]
    for name,goal in waypoints:
        endpoint,pe,re=kin.ik(goal,rotation,q)
        if pe>.002 or re>.03: return None,dict(reason='endpoint_ik',phase=name,position_error_m=pe,rotation_error_rad=re)
        if name=='preapproach':
            seconds=max(1.5*np.max(np.abs(endpoint[1:]-q[1:]))/.35,1.5*abs(endpoint[0]-q[0])/.08,1.)
            steps=int(np.ceil(seconds/env.step_dt));u=np.linspace(0,1,steps+1)[1:];u=u*u*(3-2*u)
            segment=q[None]+u[:,None]*(endpoint-q)[None]
        else:
            distance=np.linalg.norm(goal-position);angle=(rotation*orientation.inv()).magnitude()
            n=max(2,int(np.ceil(distance/.003)),int(np.ceil(angle/.025)))
            knots=[q.copy()]
            for u in np.linspace(0,1,n+1)[1:]:
                r=Rotation.from_rotvec((rotation*orientation.inv()).as_rotvec()*u)*orientation
                sol,pe,re=kin.ik(position+(goal-position)*u,r,knots[-1])
                if pe>.002 or re>.03: return None,dict(reason='path_ik',phase=name,position_error_m=pe,rotation_error_rad=re)
                knots.append(sol)
            speed=.002 if name in ('rise','pull') else .035
            seconds=max(1.5*distance/speed,angle/.15,.2)
            steps=int(np.ceil(seconds/env.step_dt));u=np.linspace(0,1,steps+1)[1:];u=u*u*(3-2*u)
            segment=np.stack([np.interp(u,np.linspace(0,1,len(knots)),np.array(knots)[:,i]) for i in range(len(q))],axis=1)
        segment=np.concatenate([segment,np.repeat(segment[-1:],20,axis=0)])
        commands.extend(segment);phases.extend([name]*len(segment));q=segment[-1]
        position,orientation=kin.fk(q)
        diagnostics.append(dict(phase=name,position_xyz=goal.tolist(),orientation_xyzw=rotation.as_quat().tolist(),steps=len(segment)))
    commands=np.asarray(commands)
    if ((commands<kin.bounds[0]-1e-7)|(commands>kin.bounds[1]+1e-7)).any(): return None,dict(reason='joint_limits')
    velocities=np.abs(np.diff(np.concatenate([start[None],commands]),axis=0))/env.step_dt
    max_speed=np.ones(len(start));max_speed[env.lift_id]=.25
    if (velocities>max_speed+1e-6).any(): return None,dict(reason='joint_velocity_limit',maximum_velocity=velocities.max(axis=0).tolist())
    # Inspect every command and further subdivide any larger joint-space jump.
    previous=start;checked=0
    for step,q in enumerate(commands):
        subdivisions=max(1,int(np.ceil(np.max(np.abs(q[1:]-previous[1:]))/.015)),int(np.ceil(abs(q[0]-previous[0])/.001)))
        for u in np.linspace(0,1,subdivisions+1)[1:]:
            collision=checker.check(previous+(q-previous)*u);checked+=1
            if collision: return None,dict(reason='self_collision',step=step,phase=phases[step],pair=collision)
        previous=q
    return dict(commands=commands,phases=phases,waypoints=diagnostics,direction=direction,
                orientation=rotation,prehook=waypoints[0][1],target_center=center,target_neck=neck),dict(passed=True,samples=checked,max_arm_sample_rad=.015,max_lift_sample_m=.001)


def retained_geometry(env):
    """Distance/plane seating of distal and terminal proximal target capsules."""
    center=env._target_geometry()[0][0].cpu().numpy();anchor=env.fruit.data.root_pos_w[0].cpu().numpy()
    axis=anchor-center;axis/=np.linalg.norm(axis)
    caps=[(center+axis*env.target_spec['radius'],anchor,.0015)]
    idx=env.elastic.chains['TRUSS_Pedicel_proximal_05'][-1]
    body=env.elastic.poses()[idx];rot=Rotation.from_quat(body[[4,5,6,3]])
    # Body rest orientation is identity; the capsule child carries its own
    # rotation. Use the exact endpoints used by ElasticPlant to author it.
    local_a,local_b,radius=env.elastic.rod_shapes[idx]
    caps.append((body[:3]+rot.apply(local_a),body[:3]+rot.apply(local_b),radius))
    rp,rq=env.ring_pose();p=rp[0].cpu().numpy();r=Rotation.from_quat(rq[0].cpu().numpy()[[1,2,3,0]])
    paths=[env.target_spec['path']+'/PedicelCollider',env.elastic.paths[idx]+'/StemCollider']
    evaluated=[rear_capsule_geometry([cap],p,r) for cap in caps]
    seated_paths=[path for path,(_,seated) in zip(paths,evaluated) if seated]
    return min(gap for gap,_ in evaluated),bool(seated_paths),r.inv().apply(anchor-p),seated_paths


def execute(env,planned,identity,app,limits):
    start_centers=env.elastic.fruit_centers().copy();start_rods=env.elastic.poses()[:,:3].copy()
    main_ids=env.elastic.chains['STEM_MainStem'];trace=[];env.contact_diagnostics=[]
    accepted=set(identity['accepted_hook_pedicels']);rear=set(identity['intended_ring_colliders'])
    intended_family=set(identity['target_proximal_pedicels'])|{identity['target_distal_pedicel']}
    first_contact=None;first_non_target=False;max_non=0.;inserted=False;guarded=False;hold=0;hold_ticks=int(round(limits['hold_seconds']/env.step_dt))
    stiffness=torch.full_like(env.targets,800.);stiffness[:,env.lift_id]=50000.
    damping=torch.full_like(env.targets,40.);damping[:,env.lift_id]=2500.
    env.robot.write_joint_stiffness_to_sim(stiffness);env.robot.write_joint_damping_to_sim(damping)
    hold_command=None;abort=None;index=0
    while index<len(planned['commands']) or (guarded and hold<hold_ticks):
        if not app.is_running(): raise RuntimeError('Simulation closed before candidate finished')
        phase='hold' if guarded else planned['phases'][index]
        if phase=='pull' and (not trace or trace[-1]['phase']!='pull'):
            stiffness[:]=30.;stiffness[:,env.lift_id]=5000.
            damping[:]=5.;damping[:,env.lift_id]=500.
            env.robot.write_joint_stiffness_to_sim(stiffness);env.robot.write_joint_damping_to_sim(damping)
        command=hold_command if guarded else planned['commands'][index]
        env.targets[:]=torch.as_tensor(command,device=env.device,dtype=torch.float32)
        env.robot.set_joint_position_target(env.targets);step=len(trace);env.contact_diagnostic_step=step
        cursor=len(env.contact_diagnostics)
        for _ in range(env.cfg.decimation):
            subcursor=len(env.contact_diagnostics)
            env.scene.write_data_to_sim();env.sim.step(render=False);env.scene.update(env.physics_dt)
            if not guarded and phase in ('insert','rise','pull'):
                hit=any(c['event']=='contact' and
                    ((c['a'] in rear and c['b'] in accepted) or (c['b'] in rear and c['a'] in accepted))
                    for c in env.contact_diagnostics[subcursor:])
                seated_paths=retained_geometry(env)[3] if hit else []
                inner_hit=any(c['event']=='contact' and
                    ((c['a'] in rear and c['b'] in seated_paths) or (c['b'] in rear and c['a'] in seated_paths))
                    for c in env.contact_diagnostics[subcursor:]) if hit else False
                if hit: inserted |= bool(env._geometry()[3][0])
                if inner_hit and inserted:
                    guarded=True;hold_command=env.robot.data.joint_pos[0].cpu().numpy().copy()
                    env.targets[:]=torch.as_tensor(hold_command,device=env.device,dtype=torch.float32)
                    env.robot.set_joint_position_target(env.targets)
                    stiffness[:]=30.;stiffness[:,env.lift_id]=5000.
                    damping[:]=5.;damping[:,env.lift_id]=500.
                    env.robot.write_joint_stiffness_to_sim(stiffness);env.robot.write_joint_damping_to_sim(damping)
        contacts=[c for c in env.contact_diagnostics[cursor:] if c['event']=='contact']
        exact=[]
        for c in contacts:
            robot,other=(c['a'],c['b']) if '/Robot/' in c['a'] else (c['b'],c['a'])
            target_pedicel=other in intended_family
            if first_contact is None:
                first_contact=dict(c,object=other,robot_collider=robot);first_non_target=not target_pedicel
            if not target_pedicel: max_non=max(max_non,c['force'])
            if robot in rear and other in accepted: exact.append(c)
        inserted |= bool(env._geometry()[3][0])
        gap,seated,relative,seated_paths=retained_geometry(env)
        exact=[c for c in exact if c['a'] in seated_paths or c['b'] in seated_paths]
        centers=env.elastic.fruit_centers();rods=env.elastic.poses()[:,:3]
        displacement=float(np.linalg.norm(centers[env.target_index]-start_centers[env.target_index]))
        main_displacement=float(np.linalg.norm(rods[main_ids]-start_rods[main_ids],axis=1).max())
        tracking_error=float(np.linalg.norm(kin_fk_position(env,command)-env.ring_pose()[0][0].cpu().numpy()))
        trace.append(dict(step=step,phase=phase,target_displacement_m=displacement,
            main_stem_displacement_m=main_displacement,cluster_centroid_displacement_m=float(np.linalg.norm(centers.mean(axis=0)-start_centers.mean(axis=0))),
            intended_contact=bool(exact),gap_m=gap,seated=seated,seated_pedicels=seated_paths,relative_anchor=relative.tolist(),
            inserted_ever=inserted,tracking_error_m=tracking_error,joints=env.robot.data.joint_pos[0].tolist(),command=np.asarray(command).tolist()))
        if env._broken[0] or env._other_broken: abort='joint_break';break
        if displacement>limits['target_displacement_m'] or main_displacement>limits['main_displacement_m']: abort='excessive_displacement';break
        if max_non>limits['dangerous_non_target_force_N']: abort='dangerous_non_target_contact';break
        if phase=='hold': hold+=1
        elif not guarded: index+=1
        if len(trace)%240==0: print('[POSE PROGRESS]',len(trace),phase,flush=True)
        if guarded and hold>=hold_ticks: break
    held=[r for r in trace if r['phase']=='hold'];tail=held[-max(1,hold_ticks//2):]
    stable=float(np.max(np.linalg.norm(np.asarray([r['relative_anchor'] for r in tail])-np.asarray(tail[-1]['relative_anchor']),axis=1))) if tail else None
    retained=(len(held)>=hold_ticks and all(r['seated'] for r in held) and
              sum(r['intended_contact'] for r in held)>=3 and stable<=limits['stable_relative_motion_m'])
    metrics=dict(retained_hook=bool(retained),inserted=inserted,guarded_target_contact=guarded,
        first_contact_object=first_contact['object'] if first_contact else None,first_contact=first_contact,
        first_contact_non_target=first_non_target,max_non_target_force_N=max_non,
        target_max_displacement_m=max((r['target_displacement_m'] for r in trace),default=0.),
        main_stem_max_displacement_m=max((r['main_stem_displacement_m'] for r in trace),default=0.),
        target_broken=bool(env._broken[0]),other_broken=bool(env._other_broken),
        min_target_rear_gap_m=min((r['gap_m'] for r in trace),default=None),
        intended_contact_ticks=sum(r['intended_contact'] for r in trace),
        max_tracking_error_m=max((r['tracking_error_m'] for r in trace),default=0.),
        hold_ticks=len(held),hold_contact_ticks=sum(r['intended_contact'] for r in held),
        stable_relative_motion_m=stable,abort_reason=abort,executed_steps=len(trace))
    return metrics,trace


def kin_fk_position(env,command):
    return env.pose_search_kin.fk(command)[0]


def run_pose(env,args,app):
    from pose_collision import SelfCollisionCheck
    directory=args.run_dir;directory.mkdir(parents=True,exist_ok=True)
    env.reset();first=state(env);env.reset();initial=state(env)
    reset=state_comparison(first,initial)
    if not reset['passed']: raise RuntimeError('Repeated reset differs: '+json.dumps(reset))
    if args.pose_reference_state:
        reference=dict(np.load(args.pose_reference_state,allow_pickle=False));reset['cross_process']=state_comparison(reference,initial)
        if not reset['cross_process']['passed']: raise RuntimeError('Candidate did not start from reference state')
    np.savez_compressed(directory/'initial_state.npz',**initial);write_json(directory/'reset_check.json',reset)
    kin=RobotKinematics(env);env.pose_search_kin=kin
    identity=structure(env,directory);checker=SelfCollisionCheck(env,kin)
    write_json(directory/'self_collision_model.json',checker.manifest)
    start_collision=checker.check(env.robot.data.joint_pos[0].cpu().numpy())
    if start_collision: raise RuntimeError('Initial PICK_READY self collision: '+str(start_collision))
    center=env._target_geometry()[0][0].cpu().numpy()
    observation=(dict(rgb=None,depth=None,available=False,reason='explicit physics-only validation; RGB-D dataset incomplete')
                 if args.pose_physics_only else dict(rgbd_snapshot(env,directory,center),available=True))
    write_json(directory/'observation.json',observation)
    reset['after_observation']=state_comparison(initial,state(env))
    if not reset['after_observation']['passed']: raise RuntimeError('Observation capture changed physics state')
    write_json(directory/'reset_check.json',reset)
    if args.pose_candidate is None:
        print('[POSE INSPECT] '+json.dumps(dict(target=identity['target_prim'],reset=reset,collision_shapes=len(checker.shapes))),flush=True)
        return
    params=json.loads(args.pose_candidate.read_text());limits=DEFAULT_LIMITS.copy()
    planned,screen=plan(env,kin,checker,params)
    result=dict(scene_id='scene_0001',target_id='Tomato_05',candidate_id=params['candidate_id'],parameters=params,
        rgb=observation['rgb'],depth=observation['depth'],observation=observation,
        robot_joint_names=env.robot.joint_names,robot_joint_positions_at_start=initial['robot_q'][0].tolist(),
        reset_check=reset,preflight=screen,thresholds=limits,
        coordinate_convention='world metres; quaternion xyzw; pose origin = CAD ring center',
        domain_randomization=False,dataset_complete=observation['available'],plant_model=env.elastic.model,physics_dt=env.physics_dt,control_dt=env.step_dt)
    orientation,waypoints,direction=candidate_waypoints(center,env._target_geometry()[1][0].cpu().numpy(),params)
    result.update(candidate_position_xyz=waypoints[0][1].tolist(),candidate_orientation_quaternion=orientation.as_quat().tolist(),
                  approach_direction=direction.tolist(),pre_hook_distance=params['pre_hook_distance_m'],
                  waypoint_positions={name:p.tolist() for name,p in waypoints})
    if planned is None:
        metrics=dict(planning_failure=True,planning_failure_reason=screen,retained_hook=False,
            first_contact_object=None,target_max_displacement_m=None,main_stem_max_displacement_m=None)
        labels=dict(result='ik_or_planning_failure',events=['ik_or_planning_failure'],hook_success=False)
    else:
        np.save(directory/'planned_commands.npy',planned['commands']);write_json(directory/'planned_phases.json',planned['phases'])
        metrics,trace=execute(env,planned,identity,app,limits)
        labels=classify(metrics,limits);write_json(directory/'trace.json',trace)
        write_json(directory/'contacts.json',env.contact_diagnostics)
    result.update(metrics);result.update(labels);write_json(directory/'candidate.json',result)
    print('[POSE RESULT] '+json.dumps({k:result[k] for k in ('candidate_id','result','events','hook_success','first_contact_object','target_max_displacement_m','main_stem_max_displacement_m')}),flush=True)
