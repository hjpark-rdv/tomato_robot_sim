"""Object-relative, physics-validated contact motion in the original greenhouse."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import torch

SIM_DIR = Path(__file__).resolve().parents[1]
from geometry import RING_CENTER
from contact_planner import motion_waypoints, propose_candidates, validate_parameters, pedicel_gap


def close_camera_view(center):
    target=np.asarray(center)+np.array([.005,0,.012])
    return target+np.array([.20/np.sqrt(2),-.20/np.sqrt(2),.045]),target


class MotionTrial:
    def __init__(self, env, kin, recorder=None, force_limit=30., goal='engage', app=None):
        self.env, self.kin = env, kin
        self.trace = []
        self.phase = 'ready'
        self.contacts = []
        self.max_force = 0.
        self.aborted = False
        self.recorder=recorder
        self.force_limit=force_limit
        self.abort_reason=None
        self.goal=goal
        self.contact_stopped=False
        self.app=app
        self.stop_force = None
        self.initial_target_center = env._target_geometry()[0][0].cpu().numpy().copy()
        if hasattr(env, 'elastic'):
            self.initial_rods = env.elastic.poses()[:, :3].copy()

    def step(self, target):
        if self.app is not None and not self.app.is_running():
            raise KeyboardInterrupt('Simulation application closed')
        env = self.env
        terminal_before=bool(env._broken[0])
        diagnostic_start=len(env.contact_diagnostics)
        env._stem_contact.fill(False)
        env._bad_contact.fill(False)
        env._peak_force.fill(0)
        env.contact_diagnostic_step=len(self.trace)
        env.targets[:] = torch.as_tensor(target, device=env.device, dtype=torch.float32)
        env.robot.set_joint_position_target(env.targets)
        for _ in range(env.cfg.decimation):
            env.scene.write_data_to_sim()
            env.sim.step(render=False)
            env.scene.update(env.physics_dt)
            guarded = ((self.goal=='engage' and env._stem_contact[0]) or
                       (self.stop_force is not None and env._peak_force[0] >= self.stop_force))
            if guarded and not env._broken[0]:
                # Guard inside the physics loop, not after a 60 Hz action:
                # the next substep can already exceed the native break threshold.
                self.contact_stopped=True
                env.targets[:]=env.robot.data.joint_pos
                env.robot.set_joint_position_target(env.targets)
                env.robot.write_joint_stiffness_to_sim(torch.zeros_like(env.targets))
        env.episode_length_buf += 1
        if not terminal_before:
            env._get_dones()
            env._get_rewards()
        elif env._other_broken:
            env.success[:]=False
        p, quat = env.ring_pose()
        row = dict(step=len(self.trace),phase=self.phase,ring=p[0].tolist(),quat=quat[0].tolist(),
                   joints=env.robot.data.joint_pos[0].tolist(),
                   command=np.asarray(target).tolist(),
                   inserted=bool(env.inserted[0]),hooked=bool(env.hooked[0]),
                   contact=bool(env._stem_contact[0]),force=float(env._peak_force[0]),
                   broken=bool(env._broken[0]),other_broken=env._other_broken,native_detachment_success=bool(env.success[0]))
        contacts=[c for c in env.contact_diagnostics[diagnostic_start:] if c['event']=='contact']
        intended=lambda c: '/RingCollision/' in c['a']+' '+c['b'] and env.target_spec['path']+'/PedicelCollider' in c['a']+' '+c['b']
        row['non_pedicel_force']=max((c['force'] for c in contacts if not intended(c)),default=0.)
        row['pedicel_force']=max((c['force'] for c in contacts if intended(c)),default=0.)
        row['fruit_force']=max((c['force'] for c in contacts if '/FruitCollider' in c['a']+' '+c['b']),default=0.)
        row['target_center']=env._target_geometry()[0][0].tolist()
        row['target_displacement_m']=float(np.linalg.norm(np.asarray(row['target_center'])-self.initial_target_center))
        row['main_stem_collision']=bool(getattr(env, '_main_stem_hit', False))
        row['main_stem_force']=max((c['force'] for c in contacts if any(
            '/ElasticPlant/STEM_MainStem_' in c[k] and c[k].endswith('/StemCollider')
            for k in ('a','b'))),default=0.)
        if hasattr(env, 'elastic'):
            ids=env.elastic.chains['STEM_MainStem']
            row['main_stem_displacement_m']=float(np.linalg.norm(
                env.elastic.poses()[ids,:3]-self.initial_rods[ids],axis=1).max())
        self.max_force = max(self.max_force, row['force'])
        self.trace.append(row)
        if self.recorder and len(self.trace)%4==0:
            self.recorder.frame()
        if row['contact']:
            self.contacts.append(row)
        if row['non_pedicel_force']>self.force_limit:
            self.aborted=True
            self.abort_reason='non_pedicel_force_limit'
        if env.sim.has_gui() and len(self.trace)%2 == 0 and self.recorder is None:
            if hasattr(env, 'elastic'):
                env.elastic.sync_visuals()
            env.sim.render()
        return not (row['broken'] or row['other_broken'] or self.aborted or self.contact_stopped)

    def move(self, name, position, rotation, speed=.035):
        self.phase = name
        env = self.env
        if name=='below' and env.sim.has_gui():
            eye,target=close_camera_view(env._target_geometry()[0][0].cpu().numpy())
            env.sim.set_camera_view(eye=eye,target=target)
        if name=='pull' and self.goal=='engage':
            stiffness=torch.full_like(env.targets,30.)
            stiffness[:,env.lift_id]=5000.
            damping=torch.full_like(env.targets,5.)
            damping[:,env.lift_id]=500.
            env.robot.write_joint_stiffness_to_sim(stiffness)
            env.robot.write_joint_damping_to_sim(damping)
        start_q = env.robot.data.joint_pos[0].cpu().numpy().copy()
        start_p, start_r = self.kin.fk(start_q)
        q, pe, re = self.kin.ik(position, rotation, start_q)
        print('[HOOK IK] '+json.dumps(dict(phase=name,position=position.tolist(),position_error=pe,rotation_error=re,joints=q.tolist())),flush=True)
        if pe>.002 or re>.03:
            self.aborted = True
            self.abort_reason='unreachable_pose'
            return False
        if name == 'preapproach':
            # First stay away from the cluster while moving to the approach pose.
            seconds=max(np.max(np.abs(q[1:]-start_q[1:]))/.35,abs(q[0]-start_q[0])/.08,1.)
            steps=int(np.ceil(seconds/env.step_dt))
            for i in range(1,steps+1):
                u=i/steps; u=u*u*(3-2*u)
                if not self.step(start_q+(q-start_q)*u):
                    return False
        else:
            steps=max(10,int(np.ceil(np.linalg.norm(position-start_p)/speed/env.step_dt)))
            seed=start_q
            for i in range(1,steps+1):
                u=i/steps; u=u*u*(3-2*u)
                p=start_p+(position-start_p)*u
                r=Rotation.from_rotvec((rotation*start_r.inv()).as_rotvec()*u)*start_r
                seed, pe, re = self.kin.ik(p,r,seed)
                if pe>.003 or re>.04:
                    self.aborted=True
                    self.abort_reason='unreachable_path'
                    return False
                if not self.step(seed):
                    return False
            # The redundant lift allows many IK solutions. Keep the continuous
            # branch reached along the path, not a separately solved endpoint.
            q=seed
        for _ in range(20):
            if not self.step(q):
                return False
        print('[HOOK PHASE] '+json.dumps({k:v for k,v in self.trace[-1].items() if k!='rod_positions'}),flush=True)
        if np.linalg.norm(np.asarray(self.trace[-1]['ring'])-position)>.006:
            self.aborted=True
            self.abort_reason='tracking_error'
            return False
        if self.recorder:
            self.recorder.snapshot(name)
        return True


class MotionRecorder:
    def __init__(self,env,directory,main_stem=False):
        import omni.replicator.core as rep
        import imageio.v2 as imageio
        from pxr import UsdGeom,Gf
        self.env,self.directory=env,directory
        directory.mkdir(parents=True,exist_ok=True)
        center=env._target_geometry()[0][0].cpu().numpy()
        self.center=center
        self.views=[]
        close_eye,close_target=close_camera_view(center)
        views=[('close',close_eye,close_target,55.),
               ('overview',center+np.array([.9,-1.2,.65]),center+np.array([.30,0,.02]),24.)]
        if hasattr(env,'elastic'):
            attachment=np.asarray(env.target_spec['pose'][:3])
            views.append(('attachment',attachment+np.array([.055,-.060,.025]),attachment,65.))
        if main_stem:
            focus=center+np.array([-.04,0,-.08])
            views=[('main_stem',focus+np.array([.65,-.85,.25]),focus,40.),views[1]]
        for name,eye,target,focal in views:
            cam=UsdGeom.Camera.Define(env.stage,'/World/HookCamera_'+name)
            if name=='close':
                self.close_camera=cam
            cam.CreateFocalLengthAttr(focal)
            cam.CreateClippingRangeAttr(Gf.Vec2f(.005,100.))
            view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1))
            UsdGeom.Xformable(cam).MakeMatrixXform().Set(view.GetInverse())
            product=rep.create.render_product(str(cam.GetPath()),(960,720))
            rgb=rep.AnnotatorRegistry.get_annotator('rgb'); rgb.attach(product)
            writer=imageio.get_writer(str(directory/(name+'.mp4')),fps=15,codec='libx264',quality=8)
            self.views.append((name,product,rgb,writer))
        for _ in range(10):
            env.sim.render()

    def frame(self):
        from omni.physx import get_physx_interface
        if hasattr(self.env, 'elastic'):
            self.env.elastic.sync_visuals()
        self.env.sim.physics_sim_view.update_articulations_kinematic()
        get_physx_interface().update_transformations(False,True,True)
        self.env.sim.render()
        for name,product,rgb,writer in self.views:
            pixels=rgb.get_data()
            if getattr(pixels,'size',0):
                writer.append_data(pixels[:,:,:3])

    def snapshot(self,name):
        from PIL import Image
        self.frame()
        for view,product,rgb,writer in self.views:
            pixels=rgb.get_data()
            if getattr(pixels,'size',0):
                Image.fromarray(pixels).save(self.directory/(name+'_'+view+'.png'))

    def close(self):
        for _,product,rgb,writer in self.views:
            writer.close(); rgb.detach(product)
        for _,product,_,_ in self.views:
            product.destroy()

    def orbit_snapshots(self):
        """Move only the camera around the final physical state; hide no geometry."""
        from pxr import UsdGeom,Gf
        from PIL import Image
        target=self.center+np.array([.005,0,.012])
        for yaw in (0,45,90,135,180,225,270,315):
            a=np.deg2rad(yaw)
            eye=target+np.array([.20*np.cos(a),.20*np.sin(a),.045])
            view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1))
            UsdGeom.Xformable(self.close_camera).MakeMatrixXform().Set(view.GetInverse())
            for _ in range(3):
                self.env.sim.render()
            Image.fromarray(self.views[0][2].get_data()).save(self.directory/f'contact_view_{yaw:03d}.png')


class RobotKinematics:
    """URDF FK/IK used for planning only; execution always uses physical drives."""
    def __init__(self, env):
        self.env = env
        root = ET.parse(SIM_DIR / 'robot_usd/rb5_farmily.urdf').getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], 'tomato_gripper'
        while link in by_child:
            joint = by_child[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        self.chain = []
        for j in reversed(chain):
            o = j.find('origin')
            tf = np.eye(4)
            if o is not None:
                tf[:3, 3] = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
                tf[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            a = j.find('axis')
            axis = np.fromstring(a.get('xyz'), sep=' ') if a is not None else np.zeros(3)
            name = j.get('name')
            index = env.robot.joint_names.index(name) if name in env.robot.joint_names else None
            self.chain.append((tf, axis, j.get('type'), index))
        self.world = np.eye(4)
        self.world[:3, 3] = env.cfg.robot.init_state.pos
        q = np.asarray(env.cfg.robot.init_state.rot)
        self.world[:3, :3] = Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        self.bounds = env.robot.data.soft_joint_pos_limits[0].cpu().numpy().T

    def fk(self, q):
        t = self.world.copy()
        for origin, axis, kind, index in self.chain:
            t = t @ origin
            if index is not None:
                if kind == 'prismatic':
                    t[:3, 3] += t[:3, :3] @ (axis*q[index])
                elif kind in ('revolute', 'continuous'):
                    t[:3, :3] = t[:3, :3] @ Rotation.from_rotvec(axis*q[index]).as_matrix()
        return t[:3, 3] + t[:3, :3] @ np.asarray(RING_CENTER), Rotation.from_matrix(t[:3, :3])

    def ik(self, position, rotation, seed):
        def error(q):
            p, r = self.fk(q)
            return np.r_[(p-position)*5, (rotation*r.inv()).as_rotvec(), (q-seed)*.002]
        result = least_squares(error, np.clip(seed, *self.bounds), bounds=self.bounds, max_nfev=100)
        p, r = self.fk(result.x)
        return result.x, float(np.linalg.norm(p-position)), float((rotation*r.inv()).magnitude())


def run_hook(env, args, app):
    kin = RobotKinematics(env)
    q = env.robot.data.joint_pos[0].cpu().numpy()
    pos, rot = kin.fk(q)
    center, neck, axis = [v[0].cpu().numpy() for v in env._target_geometry()]
    report = dict(ring=env.ring_pose()[0][0].tolist(), fk=pos.tolist(), ring_rotation=rot.as_matrix().tolist(),
                  target=center.tolist(), neck=neck.tolist(), axis=axis.tolist(), radius=env.target_spec['radius'],
                  fruits=[dict(name=s['name'],radius=s['radius'],center=c) for s,c in zip(env.fruit_specs,env.provenance['fruit_centers_world'])],
                  q=q.tolist(), joint_names=env.robot.joint_names)
    print('[HOOK PROBE] '+json.dumps(report), flush=True)
    (args.run_dir/'hook_probe.json').write_text(json.dumps(report, indent=2)+'\n')
    assert np.linalg.norm(pos-report['ring']) < .0001, 'URDF/PhysX FK mismatch'
    candidates=json.loads(args.hook_config.read_text()) if args.hook_config else [{}]
    if not isinstance(candidates,list):
        candidates=[candidates]
    if not candidates:
        raise ValueError('Empty hook candidate list')
    if args.hook_search:
        seeds=candidates if args.hook_config else []
        candidates=seeds+propose_candidates(center,neck,np.asarray(env.provenance['fruit_centers_world']),
            np.asarray([s['radius'] for s in env.fruit_specs]),seed=args.seed,count=args.hook_trials)
        (args.run_dir/'search_candidates.json').write_text(json.dumps(candidates,indent=2)+'\n')
    for params in candidates:
        validate_parameters(params)
    trials=[]
    scenes=[]
    best_motion_score=-np.inf
    for index in range(args.hook_trials):
        params=candidates[index%len(candidates)]
        target=args.target_fruit_override or params.get('target_fruit',args.target_fruit)
        if target not in [s['name'] for s in env.fruit_specs]:
            raise ValueError(f'Unknown target fruit: {target}')
        env.target_index=next(i for i,s in enumerate(env.fruit_specs) if s['name']==target)
        env.target_spec=env.fruit_specs[env.target_index]
        env.fruit=env.fruits[env.target_index]
        env.cfg.target_fruit=target
        env.provenance.update(target=target,break_force=env.target_spec['force'],break_torque=env.target_spec['torque'])
        scenes.append(dict(env.provenance))
        env.reset()
        stiffness=torch.full_like(env.targets,800.)
        stiffness[:,env.lift_id]=50000.
        env.robot.write_joint_stiffness_to_sim(stiffness)
        damping=torch.full_like(env.targets,40.)
        damping[:,env.lift_id]=2500.
        env.robot.write_joint_damping_to_sim(damping)
        env.contact_diagnostics=[]
        center, neck, axis=[v[0].cpu().numpy() for v in env._target_geometry()]
        rotation,poses=motion_waypoints(center,neck,params)
        recorder=MotionRecorder(env,args.run_dir/f'video_{index:03d}') if args.record else None
        trial=MotionTrial(env,kin,recorder,params.get('force_limit_N',30.),args.hook_goal,app)
        for name,p in poses:
            speed=params.get('contact_speed_m_s',.002 if args.hook_goal=='engage' else .008) if name in ('rise','pull') else params.get('speed_m_s',.035)
            if not trial.move(name,p,rotation,speed):
                break
        break_step=len(trial.trace)-1 if env._broken[0] else None
        valid_detachment_at_break=bool(env.success[0])
        hold_steps=int(np.ceil((args.hook_hold_seconds if args.hook_goal=='engage' else 1.)/env.step_dt))
        if (env.success[0] or trial.contact_stopped) and not trial.aborted:
            if recorder:
                recorder.snapshot('hooked')
            trial.phase='contact_hold' if args.hook_goal=='engage' else 'release_observation'
            for _ in range(hold_steps):
                trial.step(env.targets[0].cpu().numpy().copy())
                if env._other_broken or trial.aborted or (args.hook_goal=='engage' and env._broken[0]):
                    break
        ring_p,ring_q=env.ring_pose()
        ring_r=Rotation.from_quat(ring_q[0].cpu().numpy()[[1,2,3,0]])
        gap=pedicel_gap(ring_p[0].cpu().numpy(),ring_r,env._target_geometry()[0][0].cpu().numpy(),
                        env.fruit.data.root_pos_w[0].cpu().numpy(),env.target_spec['radius'])
        task_success=(bool(env.inserted[0]) and bool(env.hooked[0]) and trial.contact_stopped and not env._broken[0] and not env._other_broken) if args.hook_goal=='engage' else bool(env.success[0])
        if args.hook_goal=='engage':
            task_success=task_success and gap<.0015 and sum(r['phase']=='contact_hold' for r in trial.trace)>=hold_steps
        engagement_conditions_met=bool(task_success and not trial.aborted)
        main_stem_collision=bool(getattr(env, '_main_stem_hit', False))
        task_success=task_success and not main_stem_collision
        result=dict(trial=index,target=target,goal=args.hook_goal,params=params,phase=trial.phase,success=task_success and not trial.aborted,
                    engagement_conditions_met=engagement_conditions_met,main_stem_collision=main_stem_collision,
                    target_center=center.tolist(),target_neck=neck.tolist(),target_axis=axis.tolist(),
                    inserted=bool(env.inserted[0]),hooked=bool(env.hooked[0]),
                    broken=bool(env._broken[0]),other_broken=env._other_broken,
                    aborted=trial.aborted,abort_reason=trial.abort_reason,max_force=trial.max_force,contact_steps=len(trial.contacts),steps=len(trial.trace))
        result.update(break_step=break_step,max_fruit_contact_N=max((r['fruit_force'] for r in trial.trace),default=0.),
                      max_main_stem_contact_N=max((r['main_stem_force'] for r in trial.trace),default=0.),
                      max_main_stem_displacement_m=max((r.get('main_stem_displacement_m',0.) for r in trial.trace),default=0.),
                      max_target_displacement_m=max((r['target_displacement_m'] for r in trial.trace),default=0.),
                      valid_detachment_at_break=valid_detachment_at_break,
                      max_target_pedicel_contact_N=max((r['pedicel_force'] for r in trial.trace),default=0.),
                      final_pedicel_gap_m=gap,contact_hold_s=sum(r['phase']=='contact_hold' for r in trial.trace)*env.step_dt,
                      max_non_pedicel_contact_N=max((r['non_pedicel_force'] for r in trial.trace),default=0.),
                      post_break_observation_s=((len(trial.trace)-1-break_step)*env.step_dt if break_step is not None else 0.))
        trials.append(result)
        result['score']=(1000*int(result['success'])+20*int(result['inserted'])+50*int(result['hooked'])
                         -200*int(result['other_broken'])-100*int(result['broken'] and not result['success'])
                         -20*int(result['aborted'])-.1*min(result['max_force'],1000))
        print('[HOOK RESULT] '+json.dumps(result),flush=True)
        (args.run_dir/f'hook_trace_{index:03d}.json').write_text(json.dumps(trial.trace)+'\n')
        (args.run_dir/f'hook_contacts_{index:03d}.json').write_text(json.dumps(env.contact_diagnostics)+'\n')
        (args.run_dir/f'hook_scene_{index:03d}.json').write_text(json.dumps(scenes[-1],indent=2)+'\n')
        (args.run_dir/'hook_results.json').write_text(json.dumps(trials,indent=2)+'\n')
        if result['success'] and result['score']>best_motion_score:
            best_motion_score=result['score']
            (args.run_dir/'successful_motion.json').write_text(json.dumps(dict(
                schema='farmily-contact-motion-v1',target=env.cfg.target_fruit,
                scene=env.provenance,params=params,result=result,
                joint_names=env.robot.joint_names,control_dt=env.step_dt,
                trajectory=trial.trace),indent=2)+'\n')
        if recorder:
            recorder.snapshot('final')
            recorder.orbit_snapshots()
            recorder.close()
    best=max(trials,key=lambda t:t['score'])
    summary=dict(method='CEM geometry screening and PhysX shooting' if args.hook_search else 'parameterized contact-motion evaluation',
                 attempts=len(trials),successes=sum(t['success'] for t in trials),
                 wrong_fruit_breaks=sum(t['other_broken'] for t in trials),best=best,
                 scene=scenes[best['trial']])
    (args.run_dir/'hook_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if best['success']:
        (args.run_dir/'best_hook.json').write_text(json.dumps(dict(best['params'],target_fruit=best['target']),indent=2)+'\n')
    (args.run_dir/'scene_provenance.json').write_text(json.dumps(summary['scene'],indent=2)+'\n')
    print('[HOOK SUMMARY] '+json.dumps({k:v for k,v in summary.items() if k not in ('scene','best')}),flush=True)
