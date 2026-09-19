"""Measure elastic loading, recovery and native failure in the original cluster."""
import json
import numpy as np
import torch
from hook_motion import MotionRecorder, MotionTrial, RobotKinematics, close_camera_view
from scipy.spatial.transform import Rotation
from geometry import RING_RADIUS


def rendered_seam(env):
    from omni.physx import get_physx_interface
    env.elastic.sync_visuals()
    get_physx_interface().update_transformations(False,True,True)
    result = env.elastic.rendered_attachment_metrics()
    print('[ELASTIC RENDER]',json.dumps(result),flush=True)
    return result


def run_elastic(env, args, app):
    if args.elastic_action == 'main-push':
        return run_main_push(env,args,app)
    if args.elastic_action == 'push':
        return run_push(env,args,app)
    env.contact_diagnostics = []
    trace = []
    rendered = {}
    center = env._target_geometry()[0][0].cpu().numpy()
    if env.sim.has_gui():
        env.sim.set_camera_view(eye=center+[.28,-.30,.12], target=center+[0,0,.025])
    recorder = MotionRecorder(env,args.run_dir/'elastic_video') if args.record else None
    last_force = None
    def step(phase, force):
        nonlocal last_force
        if not app.is_running():
            raise KeyboardInterrupt('Application closed')
        f = torch.tensor([[force]],dtype=torch.float32,device=env.device)
        if tuple(force) != last_force:
            env.fruit.set_external_force_and_torque(f,torch.zeros_like(f),is_global=True)
            last_force = tuple(force)
        env.contact_diagnostic_step = len(trace)
        for _ in range(env.cfg.decimation):
            env.scene.write_data_to_sim()
            env.sim.step(render=False)
            env.scene.update(env.physics_dt)
        row=dict(step=len(trace),phase=phase,force_N=list(force),
                 fruit_center=env._target_geometry()[0][0].tolist(),
                 broken=bool(env._broken[0]),other_broken=bool(env._other_broken),
                 **env.elastic.metrics())
        trace.append(row)
        if recorder and len(trace)%4==0:
            recorder.frame()
        elif recorder is None and env.sim.has_gui() and len(trace)%2==0:
            env.elastic.sync_visuals()
            env.sim.render()
        return row
    try:
        for phase,seconds,force in [('settle',3.,[0,0,0]),('load',2.,[.3,0,0]),('recover',12.,[0,0,0])]:
            for _ in range(round(seconds/env.step_dt)):
                row=step(phase,force)
                if phase=='settle' and (row['broken'] or row['other_broken']):
                    raise RuntimeError('Plant detached during initial settling; inspect elastic_contacts.json')
            print('[ELASTIC PHASE]',json.dumps(row),flush=True)
            if phase=='settle' and row['main_stem_max_displacement_m']>.01:
                raise RuntimeError('Main stem rest pose drift exceeds 1 cm')
            if recorder:
                recorder.snapshot(phase)
            rendered[phase] = rendered_seam(env)
        settled=np.array([r['fruit_center'] for r in trace if r['phase']=='settle'][-1])
        loaded=np.array([r['fruit_center'] for r in trace if r['phase']=='load'][-1])
        recovered=np.array(trace[-1]['fruit_center'])
        result=dict(target=env.cfg.target_fruit,model=env.elastic.model,
                    loaded_displacement_m=float(np.linalg.norm(loaded-settled)),
                    recovery_error_m=float(np.linalg.norm(recovered-settled)),
                    broken=bool(env._broken[0]),other_broken=bool(env._other_broken),
                    finite=bool(np.isfinite(np.array([r['fruit_center'] for r in trace])).all()))
        result['max_joint_gap_m']=max(r['max_joint_gap_m'] for r in trace)
        result['max_attachment_gap_m']=max(r['max_attachment_gap_m'] for r in trace)
        result['max_visual_attachment_error_m']=max(r['max_visual_attachment_error_m'] for r in trace)
        result['rendered_seams']=rendered
        result['elasticity_passed'] = (result['finite'] and not result['broken'] and not result['other_broken']
            and .0005 < result['loaded_displacement_m'] < .10
            and result['recovery_error_m'] < .001 and result['max_joint_gap_m'] < .0005
            and result['max_attachment_gap_m'] < .0005
            and result['max_visual_attachment_error_m'] < .00005
            and max(r['max_rendered_seam_surface_gap_m'] for r in rendered.values()) < .00005
            and max(r['max_usd_fruit_pose_error_m'] for r in rendered.values()) < .00005)
        # Overload is a separate mechanics test, never counted as a harvest.
        if result['elasticity_passed']:
            for _ in range(120):
                step('overload',[6.,0,0])
                if env._broken[0]:
                    break
        result['native_overload_break'] = (result['elasticity_passed'] and bool(env._broken[0]) and
            any(e['event']=='break' and e['path']==env.joint_paths[0] for e in env.contact_diagnostics))
        result['other_break_during_overload'] = bool(env._other_broken)
        env.reset()
        last_force = None
        for _ in range(180):
            step('reset_settle',[0,0,0])
        result['reset_reattachment_stable'] = not bool(env._broken[0]) and not bool(env._other_broken)
        result['reset_position_error_m'] = float(np.linalg.norm(np.array(trace[-1]['fruit_center'])-settled))
        result['passed'] = (result['elasticity_passed'] and result['native_overload_break']
                            and not result['other_break_during_overload']
                            and result['reset_reattachment_stable'] and result['reset_position_error_m']<.001)
        (args.run_dir/'elastic_trace.json').write_text(json.dumps(trace)+'\n')
        (args.run_dir/'elastic_result.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.run_dir/'elastic_contacts.json').write_text(json.dumps(env.elastic.contact_records)+'\n')
        (args.run_dir/'elastic_events.json').write_text(json.dumps(env.contact_diagnostics)+'\n')
        print('[ELASTIC RESULT]',json.dumps(result),flush=True)
        if not result['passed']:
            raise RuntimeError('Elastic mechanics validation failed; see elastic_result.json')
    finally:
        (args.run_dir/'elastic_trace.json').write_text(json.dumps(trace)+'\n')
        (args.run_dir/'elastic_contacts.json').write_text(json.dumps(env.elastic.contact_records)+'\n')
        (args.run_dir/'elastic_events.json').write_text(json.dumps(env.contact_diagnostics)+'\n')
        env.fruit.set_external_force_and_torque(torch.zeros(1,1,3,device=env.device),torch.zeros(1,1,3,device=env.device))
        if recorder:
            recorder.close()


class ElasticPushTrial(MotionTrial):
    def step(self,target):
        proceed=super().step(target)
        self.trace[-1].update(self.env.elastic.metrics())
        self.trace[-1]['rod_positions']=self.env.elastic.poses()[:,:3].tolist()
        self.trace[-1]['fruit_centers']=self.env.elastic.fruit_centers().tolist()
        self.trace[-1]['main_stem_collision']=bool(self.env._main_stem_hit)
        return proceed


def run_push(env,args,app):
    """A guarded ring push/retract experiment, not a claimed harvesting policy."""
    env.contact_diagnostics=[]
    env.elastic.contact_records=[]
    kin=RobotKinematics(env)
    recorder=MotionRecorder(env,args.run_dir/'elastic_video') if args.record else None
    trial=ElasticPushTrial(env,kin,recorder,force_limit=3.,goal='push',app=app)
    try:
        trial.phase='settle'
        for _ in range(180):
            trial.step(env.targets[0].cpu().numpy().copy())
        baseline=env.elastic.poses()[:,:3].copy()
        center,neck,_=[v[0].cpu().numpy().copy() for v in env._target_geometry()]
        rotation=Rotation.from_matrix(np.array([[1,0,0],[0,0,1],[0,-1,0]]))
        # Bring the rear arc to the actual target pedicel, then press at most
        # 4 mm farther. Plant deformation is produced solely by ring contact.
        contact=neck+np.array([RING_RADIUS,0,0])
        approach=contact+np.array([.10,0,0])
        if not trial.move('preapproach',approach,rotation,.035):
            raise RuntimeError('Elastic push approach failed: '+str(trial.abort_reason))
        if env.sim.has_gui():
            eye,focus=close_camera_view(center)
            env.sim.set_camera_view(eye=eye,target=focus)
        trial.stop_force=.5
        k=torch.full_like(env.targets,80.); k[:,env.lift_id]=10000.
        d=torch.full_like(env.targets,10.); d[:,env.lift_id]=500.
        env.robot.write_joint_stiffness_to_sim(k)
        env.robot.write_joint_damping_to_sim(d)
        trial.move('near_contact',contact+np.array([.005,0,0]),rotation,.015)
        if not trial.contact_stopped and not trial.aborted:
            trial.move('elastic_push',contact-np.array([.004,0,0]),rotation,.002)
        loaded=env.elastic.poses()[:,:3].copy()
        loaded_fruit_center=env._target_geometry()[0][0].cpu().numpy().copy()
        contact_records=list(env.contact_diagnostics)
        before_retract=len(trial.trace)
        if recorder:
            recorder.snapshot('loaded')
        # Check actual rendered mesh boundaries, not just tensor frame origins.
        loaded_rendered_seam=rendered_seam(env)
        was_broken=bool(env._broken[0]) or bool(env._other_broken)
        trial.stop_force=None
        trial.contact_stopped=False
        env.robot.write_joint_stiffness_to_sim(k)
        if not was_broken and not trial.aborted:
            current=env.ring_pose()[0][0].cpu().numpy()
            trial.move('retract',current+np.array([.06,0,0]),rotation,.01)
        trial.phase='recover'
        for _ in range(240):
            trial.step(env.targets[0].cpu().numpy().copy())
        recovered=env.elastic.poses()[:,:3]
        target_name=env.cfg.target_fruit[-2:]
        target_contact=[c for c in contact_records if c['event']=='contact' and
                        ('/RingCollision/' in c['a']+' '+c['b']) and
                        (env.target_spec['path']+'/PedicelCollider' in c['a']+' '+c['b'] or
                         'TRUSS_Pedicel_proximal_'+target_name in c['a']+' '+c['b'])]
        result=dict(target=env.cfg.target_fruit,action='guarded ring push and retract',
                    physical_target_contact=bool(target_contact),
                    max_target_contact_N=max((c['force'] for c in target_contact),default=0.),
                    loaded_max_rod_displacement_m=float(np.linalg.norm(loaded-baseline,axis=1).max()),
                    recovery_max_rod_error_m=float(np.linalg.norm(recovered-baseline,axis=1).max()),
                    loaded_fruit_displacement_m=float(np.linalg.norm(loaded_fruit_center-center)),
                    recovery_fruit_error_m=float(np.linalg.norm(env._target_geometry()[0][0].cpu().numpy()-center)),
                    max_fruit_contact_N=max((c['force'] for c in env.contact_diagnostics if c['event']=='contact'
                                             and '/FruitCollider' in c['a']+' '+c['b']),default=0.),
                    broken=bool(env._broken[0]),other_broken=bool(env._other_broken),
                    aborted=trial.aborted,abort_reason=trial.abort_reason,model=env.elastic.model,
                    steps_before_retract=before_retract)
        result['max_attachment_gap_m']=max(r['max_attachment_gap_m'] for r in trial.trace)
        result['max_visual_attachment_error_m']=max(r['max_visual_attachment_error_m'] for r in trial.trace)
        result['loaded_rendered_seam']=loaded_rendered_seam
        result['passed']=(result['physical_target_contact'] and not result['broken'] and not result['other_broken']
                          and not result['aborted'] and result['loaded_max_rod_displacement_m']>.0002
                          and result['recovery_max_rod_error_m']<.002 and result['max_attachment_gap_m']<.0005
                          and result['max_visual_attachment_error_m']<.00005
                          and loaded_rendered_seam['max_rendered_seam_center_gap_m']<.00005
                          and loaded_rendered_seam['max_rendered_seam_surface_gap_m']<.00005
                          and loaded_rendered_seam['max_usd_fruit_pose_error_m']<.00005)
        (args.run_dir/'elastic_push_result.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.run_dir/'elastic_push_trace.json').write_text(json.dumps(trial.trace)+'\n')
        (args.run_dir/'elastic_push_contacts.json').write_text(json.dumps(env.contact_diagnostics)+'\n')
        print('[ELASTIC PUSH RESULT]',json.dumps(result),flush=True)
        if recorder:
            recorder.snapshot('recovered')
        if not result['passed']:
            raise RuntimeError('Guarded ring push did not meet validation criteria')
    finally:
        if recorder:
            recorder.close()


def run_main_push(env,args,app):
    """Reproduce wrong ring contact with the main stem using robot drives only."""
    env.contact_diagnostics=[]
    env.elastic.contact_records=[]
    kin=RobotKinematics(env)
    recorder=MotionRecorder(env,args.run_dir/'elastic_video',main_stem=True) if args.record else None
    trial=ElasticPushTrial(env,kin,recorder,force_limit=3.,goal='push',app=app)
    result={}
    try:
        trial.phase='settle'
        for _ in range(180):
            trial.step(env.targets[0].cpu().numpy().copy())
            if env._broken[0] or env._other_broken:
                raise RuntimeError('Plant detached before robot approach')
        baseline=env.elastic.poses()[:,:3].copy()
        fruit_baseline=env.elastic.fruit_centers().copy()
        main=env.elastic.chains['STEM_MainStem']
        # Use the exposed main stem below the lowest fruit. Above the truss,
        # foliage contacts the ring first and pushes the trunk out of its path.
        height=float(fruit_baseline[:,2].min()-.15)
        rod=min(main,key=lambda i:abs(baseline[i,2]-height))
        # The rod midpoint coincides with a pruned-branch stub in this asset.
        # Contact the upper interior of the same capsule, clear of that stub.
        a,b,_=env.elastic.rod_shapes[rod]
        pose=env.elastic.poses()[rod]
        point=baseline[rod]+Rotation.from_quat(pose[[4,5,6,3]]).apply(.30*(b-a))
        rotation=Rotation.from_matrix(np.array([[1,0,0],[0,0,1],[0,-1,0]]))
        contact=point+np.array([RING_RADIUS+env.elastic.rod_shapes[rod][2],0,0])
        if env.sim.has_gui():
            focus=(point+fruit_baseline.mean(0))/2
            env.sim.set_camera_view(eye=focus+[.65,-.85,.3],target=focus)
        if not trial.move('preapproach',contact+np.array([.10,0,0]),rotation,.05):
            raise RuntimeError('Main stem approach failed: '+str(trial.abort_reason))
        trial.stop_force=.8
        k=torch.full_like(env.targets,80.); k[:,env.lift_id]=10000.
        d=torch.full_like(env.targets,10.); d[:,env.lift_id]=500.
        env.robot.write_joint_stiffness_to_sim(k)
        env.robot.write_joint_damping_to_sim(d)
        trial.move('main_near_contact',contact+np.array([.008,0,0]),rotation,.015)
        if not trial.contact_stopped and not trial.aborted:
            trial.move('main_push',contact-np.array([.020,0,0]),rotation,.003)
        loaded=env.elastic.poses()[:,:3].copy()
        fruits_loaded=env.elastic.fruit_centers().copy()
        loaded_seam=rendered_seam(env)
        if recorder:
            recorder.snapshot('main_loaded')
        contact_end=len(trial.trace)
        trial.stop_force=None; trial.contact_stopped=False
        env.robot.write_joint_stiffness_to_sim(k)
        if not env._broken[0] and not env._other_broken and not trial.aborted:
            current=env.ring_pose()[0][0].cpu().numpy()
            trial.move('main_retract',current+np.array([.08,0,0]),rotation,.02)
        trial.phase='recover'
        for _ in range(480):
            trial.step(env.targets[0].cpu().numpy().copy())
        recovered=env.elastic.poses()[:,:3]
        contacts=[c for c in env.contact_diagnostics if c['event']=='contact' and
                  '/RingCollision/' in c['a']+' '+c['b'] and any(
                      '/ElasticPlant/STEM_MainStem_' in p and p.endswith('/StemCollider')
                      for p in (c['a'],c['b']))]
        displacements=np.linalg.norm(fruits_loaded-fruit_baseline,axis=1)
        result=dict(action='deliberate wrong ring approach to main stem',model=env.elastic.model,
                    contact_rod=env.elastic.paths[rod],physical_ring_main_contact=bool(contacts),
                    max_main_contact_N=max((c['force'] for c in contacts),default=0.),
                    loaded_main_displacement_m=float(np.linalg.norm(loaded[main]-baseline[main],axis=1).max()),
                    loaded_cluster_centroid_displacement_m=float(np.linalg.norm(fruits_loaded.mean(0)-fruit_baseline.mean(0))),
                    loaded_each_fruit_displacement_m=displacements.tolist(),
                    recovery_main_error_m=float(np.linalg.norm(recovered[main]-baseline[main],axis=1).max()),
                    recovery_cluster_error_m=float(np.linalg.norm(env.elastic.fruit_centers().mean(0)-fruit_baseline.mean(0))),
                    root_displacement_m=float(np.linalg.norm(loaded[main[0]]-baseline[main[0]])),
                    main_stem_collision_failure=bool(env._main_stem_hit),harvest_success=bool(env.success[0]),
                    broken=bool(env._broken[0]),other_broken=bool(env._other_broken),
                    aborted=trial.aborted,abort_reason=trial.abort_reason,loaded_rendered_seam=loaded_seam,
                    steps_before_retract=contact_end,
                    max_joint_gap_m=max(r['max_joint_gap_m'] for r in trial.trace),
                    max_attachment_gap_m=max(r['max_attachment_gap_m'] for r in trial.trace))
        # This diagnostic tests direct contact and coupled target motion.
        # Report the pre-existing precision-recovery threshold independently;
        # the full validate/push suites retain their original recovery gates.
        result['precision_recovery_passed']=result['recovery_main_error_m']<.002
        result['passed']=(result['physical_ring_main_contact'] and result['main_stem_collision_failure']
            and not result['harvest_success'] and not result['broken'] and not result['other_broken']
            and not result['aborted'] and result['loaded_main_displacement_m']>.001
            and result['loaded_cluster_centroid_displacement_m']>.001 and min(displacements)>.0005
            and result['root_displacement_m']<.00001
            and result['max_joint_gap_m']<.0005 and result['max_attachment_gap_m']<.0005
            and loaded_seam['max_rendered_seam_surface_gap_m']<.00005)
        print('[MAIN STEM PUSH RESULT]',json.dumps(result),flush=True)
        if recorder:
            recorder.snapshot('main_recovered')
        if not result['passed']:
            raise RuntimeError('Main stem collision reproduction did not meet validation criteria')
    finally:
        (args.run_dir/'main_stem_push_result.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.run_dir/'main_stem_push_trace.json').write_text(json.dumps(trial.trace)+'\n')
        (args.run_dir/'main_stem_push_contacts.json').write_text(json.dumps(env.contact_diagnostics)+'\n')
        (args.run_dir/'main_stem_plant_contacts.json').write_text(json.dumps(env.elastic.contact_records)+'\n')
        if recorder:
            recorder.close()
