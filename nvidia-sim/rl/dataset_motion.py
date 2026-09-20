"""Candidate planner and cooperative physics executor derived from pose_worker.

Each generator yields once per PhysX substep; a single shared simulation step
then advances all environments. No scene writes, resets or stepping in a slot.
"""
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from dataset_design import waypoints as make_waypoints
from pose_candidates import rear_capsule_geometry


def non_target_seated(env, path):
    """Require geometry as well as native rear-arc contact for a wrong hook."""
    cap = None
    if '/ElasticPlant/' in path and path.endswith('/StemCollider'):
        body_path = path.rsplit('/', 1)[0]
        if body_path in env.elastic.paths:
            index = env.elastic.paths.index(body_path)
            body = env.elastic.poses()[index]
            rotation = Rotation.from_quat(body[[4,5,6,3]])
            a,b,radius = env.elastic.rod_shapes[index]
            cap = (body[:3]+rotation.apply(a), body[:3]+rotation.apply(b), radius)
    else:
        for spec,fruit in zip(env.fruit_specs,env.fruits):
            if path == spec['path']+'/PedicelCollider':
                anchor=fruit.data.root_pos_w[0].cpu().numpy()
                rotation=Rotation.from_quat(fruit.data.root_quat_w[0].cpu().numpy()[[1,2,3,0]])
                center=anchor+rotation.apply(spec['center'])
                axis=anchor-center;axis/=np.linalg.norm(axis)
                cap=(center+axis*spec['radius'],anchor,.0015)
                break
    if cap is None:return False
    position,quaternion=env.ring_pose()
    return rear_capsule_geometry([cap],position[0].cpu().numpy(),
        Rotation.from_quat(quaternion[0].cpu().numpy()[[1,2,3,0]]))[1]

def plan(env,kin,checker,params):
    center,neck,_=[v[0].cpu().numpy() for v in env._target_geometry()]
    rotation,waypoints,direction=make_waypoints(center,neck,params)
    if params.get('goal', 'rise') == 'rise':
        waypoints = [w for w in waypoints if w[0] != 'pull']
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
            speed=(getattr(env.cfg,'dataset_rise_speed',.002) if name=='rise' else
                   getattr(env.cfg,'dataset_pull_speed',.004) if name=='pull' else .035)
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
    idx=env.elastic.chains['TRUSS_Pedicel_proximal_'+env.target_spec['name'][-2:]][-1]
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


def execute_steps(env,planned,identity,app,limits):
    start_centers=env.elastic.fruit_centers().copy();start_rods=env.elastic.poses()[:,:3].copy()
    main_ids=env.elastic.chains['STEM_MainStem'];trace=[];env.contact_diagnostics=[]
    accepted=set(identity['accepted_hook_pedicels']);rear=set(identity['intended_ring_colliders'])
    intended_family=set(identity['target_proximal_pedicels'])|{identity['target_distal_pedicel']}
    first_contact=None;first_non_target=False;max_non=0.;inserted=False;guarded=False;hold=0;hold_ticks=int(round(limits['hold_seconds']/env.step_dt))
    stiffness=torch.full_like(env.targets,800.);stiffness[:,env.lift_id]=50000.
    damping=torch.full_like(env.targets,40.);damping[:,env.lift_id]=2500.
    env.robot.write_joint_stiffness_to_sim(stiffness);env.robot.write_joint_damping_to_sim(damping)
    hold_command=None;abort=None;index=0; forced_hold=False; non_target_count=0; hooked_non_target=False
    while index<len(planned['commands']) or (guarded and hold<hold_ticks):
        if not app.is_running(): raise RuntimeError('Simulation closed before candidate finished')
        if not guarded and index >= len(planned['commands']):
            break
        phase='hold' if guarded else planned['phases'][index]
        if phase=='pull' and (not trace or trace[-1]['phase']!='pull'):
            stiffness[:]=30.;stiffness[:,env.lift_id]=5000.
            damping[:]=5.;damping[:,env.lift_id]=500.
            env.robot.write_joint_stiffness_to_sim(stiffness);env.robot.write_joint_damping_to_sim(damping)
        command=hold_command if guarded else planned['commands'][index]
        env.targets[:]=torch.as_tensor(command,device=env.device,dtype=torch.float32)
        env.robot.set_joint_position_target(env.targets);step=len(trace);env.contact_diagnostic_step=step
        env.command_dirty=True
        cursor=len(env.contact_diagnostics)
        for _ in range(env.cfg.decimation):
            subcursor=len(env.contact_diagnostics)
            yield None  # The shared runner advances ALL slots exactly once.
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
                    env.command_dirty=True
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
            if not target_pedicel and other != env.target_spec['path']+'/FruitCollider':
                non_target_count += 1
                if robot in rear and 'Pedicel' in other:
                    hooked_non_target |= non_target_seated(env,other)
            if robot in rear and other in accepted: exact.append(c)
        inserted |= bool(env._geometry()[3][0])
        gap,seated,relative,seated_paths=retained_geometry(env)
        exact=[c for c in exact if c['a'] in seated_paths or c['b'] in seated_paths]
        centers=env.elastic.fruit_centers();rods=env.elastic.poses()[:,:3]
        displacement=float(np.linalg.norm(centers[env.target_index]-start_centers[env.target_index]))
        main_displacement=float(np.linalg.norm(rods[main_ids]-start_rods[main_ids],axis=1).max())
        tracking_error=float(np.linalg.norm(env.pose_search_kin.fk(command)[0]-env.ring_pose()[0][0].cpu().numpy()))
        trace.append(dict(step=step,phase=phase,target_displacement_m=displacement,
            main_stem_displacement_m=main_displacement,cluster_centroid_displacement_m=float(np.linalg.norm(centers.mean(axis=0)-start_centers.mean(axis=0))),
            intended_contact=bool(exact),gap_m=gap,seated=seated,seated_pedicels=seated_paths,relative_anchor=relative.tolist(),
            inserted_ever=inserted,tracking_error_m=tracking_error,joints=env.robot.data.joint_pos[0].tolist(),command=np.asarray(command).tolist()))
        if env.cfg.dataset_max_steps and len(trace) >= env.cfg.dataset_max_steps:
            abort='debug_step_limit';break
        if env._broken[0] or env._other_broken: abort='joint_break';break
        if displacement>limits['target_displacement_m'] or main_displacement>limits['main_displacement_m']: abort='excessive_displacement';break
        if max_non>limits['dangerous_non_target_force_N']: abort='dangerous_non_target_contact';break
        if phase=='hold': hold+=1
        elif not guarded:
            index+=1
            if index == len(planned['commands']):
                guarded=True;forced_hold=True
                hold_command=env.robot.data.joint_pos[0].cpu().numpy().copy()
        if len(trace)%240==0: print('[POSE PROGRESS]',len(trace),phase,flush=True)
        if guarded and hold>=hold_ticks: break
    held=[r for r in trace if r['phase']=='hold'];tail=held[-max(1,hold_ticks//2):]
    stable=float(np.max(np.linalg.norm(np.asarray([r['relative_anchor'] for r in tail])-np.asarray(tail[-1]['relative_anchor']),axis=1))) if tail else None
    retained=(len(held)>=hold_ticks and all(r['seated'] for r in held) and
              sum(r['intended_contact'] for r in held)>=3 and stable<=limits['stable_relative_motion_m'])
    metrics=dict(non_target_contact_count=non_target_count,hooked_non_target=hooked_non_target,retained_hook=bool(retained),inserted=inserted,guarded_target_contact=guarded and not forced_hold,
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
