"""Persistent native S300 rail/lift/arm/jaw execution and measured group harvest.

Reuses the existing S300 group selector and IK planner. Only actuator targets
and native belt surface velocities drive the robot; stored poses are evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import shutil
import sys
import time

import mujoco as mj
import numpy as np
import psutil
from scipy.spatial.transform import Rotation as R

from run_smoke import Attachments, belt_velocity_from_physx
from plan_adapter import DOF_NAMES, sample_plan

REPO=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(REPO/'nvidia-sim/harvest_s300'))
from harvest_group_policy import HarvestGroupSelector
from truss_sequence import horizontal_truss_direction
from rail_motion import rail_arrival_check


def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


FREE_PHASES=frozenset({'settle','idle','rail_move','rail_settle','ready_hold',
    'prepare_open','prepare_open_hold','prepare_lift_branch','prepare_joint_branch',
    'prepare_extend','return_joint_branch','return_lift_branch','ready_close','hold'})


def phase_frequency(phase,contact_hz,free_hz=None):
    """Lower the frequency only for named, non-contact transfer phases.

    Every belt cycle phase (including both open holds and the final opening)
    and unknown future phases keep the contact frequency. A list of selected
    contact names silently dropped the second opening to 60 Hz.
    """
    return free_hz if free_hz is not None and phase in FREE_PHASES else contact_hz


class Session:
    def __init__(self,model_dir,hz=120,gravity_compensation=True,diagnostic_continue=False,released_gutter_diagnostic=False,threads=0,threads_in_contact_only=False):
        start=time.perf_counter();self.directory=Path(model_dir);self.meta=json.loads((self.directory/'export.json').read_text())
        if sha(self.directory/'scene.xml')!=self.meta['model_sha256']:raise ValueError('Model hash mismatch')
        if hz<=0:raise ValueError('Positive physics frequency required')
        if threads<0:raise ValueError('Nonnegative native worker count required')
        cache=self.directory/'compiled_scene.mjb';certificate=self.directory/'compiled_scene.json'
        cached=json.loads(certificate.read_text()) if certificate.exists() else {}
        valid=(cache.exists() and cached.get('model_xml_sha256')==self.meta['model_sha256'] and
               cached.get('mujoco_version')==mj.__version__ and cached.get('mjb_sha256')==sha(cache))
        self.diagnostic_continue=diagnostic_continue
        self.released_gutter_diagnostic=released_gutter_diagnostic
        self.model=m=(mj.MjModel.from_binary_path(str(cache)) if valid else
                      mj.MjModel.from_xml_path(str(self.directory/'scene.xml')))
        if not valid:
            mj.mj_saveModel(m,str(cache))
            write(certificate,dict(model_xml_sha256=self.meta['model_sha256'],mjb_sha256=sha(cache),mujoco_version=mj.__version__))
        self.loaded_cached_model=valid;m.opt.timestep=1/hz
        m.flg_surfacevel=True;self.data=d=mj.MjData(m);self.hz=hz;self.gravity_compensation=gravity_compensation
        self.threads=threads;self.threads_in_contact_only=threads_in_contact_only
        self.active_threads=0 if threads_in_contact_only else threads
        if self.active_threads:mj.mju_threadpool(d,self.active_threads)
        self.worker_events=[dict(t=0.,phase='initial',workers=self.active_threads)]
        for name,q in self.meta['robot']['initial_q'].items():d.qpos[m.joint(name).qposadr]=q
        for name,v in self.meta['robot']['initial_qvel'].items():d.qvel[m.joint(name).dofadr]=v
        self.qadr=np.array([int(m.joint(n).qposadr[0]) for n in DOF_NAMES]);self.vadr=np.array([int(m.joint(n).dofadr[0]) for n in DOF_NAMES])
        self.aids=np.array([m.actuator('drive_'+n).id for n in DOF_NAMES])
        self.kp=np.array([m.actuator_gainprm[a,0] for a in self.aids])
        self.at=Attachments(m,self.meta['attachments']);self.breaks=[];self.rows=[];self.contacts=[];self.phase_events=[]
        self.toolids=set(m.body(n).id for n in self.meta['tools']);self.robotids=set(m.body(n).id for n in self.meta['robot']['body_names'].values())
        self.belt_geoms=[np.flatnonzero((m.geom_bodyid==m.body(n).id)&((m.geom_contype!=0)|(m.geom_conaffinity!=0))) for n in self.meta['robot']['belt_body_names']]
        self.base_mask=(m.geom_contype.copy(),m.geom_conaffinity.copy())
        self.windows=self.meta.get('windows',[dict(id='target',prefix='',bodies=self.meta['rods']+self.meta['fruits'],fruit_specs=self.meta['fruit_specs'],
                         source_assembly=self.meta['source_assembly'],source_trial=self.meta['source_trial'],source_plan=str(self.directory/'source_plan.json'))])
        self.wids=[{m.body(n).id for n in w['bodies']} for w in self.windows]
        self.geom_class={g['name']:g for g in self.meta['geometry']};self.load_s=time.perf_counter()-start
        self.reference_plant_preload=np.zeros(m.nv)
        mj.mj_forward(m,d)
        # Source Isaac uses a fixed authored-pose plant gravity preload. Copy
        # its semantics through current native Jacobians, including fruit mass
        # at the parent branch. This is computed once, not contact cancellation.
        plant_dofs=np.array([i for i in range(m.nv) if i not in self.vadr and
                            m.jnt_type[m.dof_jntid[i]]!=mj.mjtJoint.mjJNT_FREE])
        self.reference_plant_preload[plant_dofs]=d.qfrc_bias[plant_dofs]
        for bid,parent in zip(self.at.ids,self.at.parents):
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mj.mj_jac(m,d,jp,jr,d.xpos[bid],int(parent))
            self.reference_plant_preload[plant_dofs]+=(jp.T@(-m.body_mass[bid]*m.opt.gravity))[plant_dofs]
        self.first_violation=None;self.stop=None;self.max_active_pen=0.;self.max_released_pen=0.;self.max_force=0.;self.physics_s=0.
        self.peak_actuator=np.zeros(10);self.tracking=np.zeros(10);self.last_phase=None;self.window_index=0
        self.save_interval=max(1,round(hz/30));self.tick=0
        self.next_save_s=1/30;self.next_log_s=5.;self.frequency_events=[];self.steps_by_hz={}
        self.cpu_started=time.process_time();self.initial_fruit_positions=d.xpos[self.at.ids].copy()
        self.set_window(0)
        self.record('initial',d.qpos[self.qadr].copy(),d.ctrl[self.aids].copy())

    def released_contact_allowed(self,bodies,geom_ids,attached_at_solver_start=None):
        """Only the user's dropped-fruit exceptions bypass active validity."""
        active=self.data.eq_active[self.at.eqs] if attached_at_solver_start is None else attached_at_solver_start
        released={int(b) for b,state in zip(self.at.ids,active) if not state}
        dropped=[int(b) for b in bodies if b in released]
        if not dropped:return False
        for fruit in dropped:
            other=int(bodies[1] if bodies[0]==fruit else bodies[0])
            name=self.model.body(other).name
            if name in self.meta['tools'] or name in ('Robot_world','Robot_S300Carrier'):return True
            if other==0:
                # Original floor, not gutters or arbitrary structure.
                structural={r['name']:r['classification'] for r in self.meta.get('farm',{}).get('structural_inventory',[])}
                return any('ground' in (self.model.geom(int(g)).name or '').lower() or
                           (self.model.geom(int(g)).name or '')=='CatchFloor' or
                           self.released_gutter_diagnostic and structural.get(self.model.geom(int(g)).name)=='gutter'
                           for g in geom_ids)
            fruit_name=self.model.body(fruit).name
            truss=fruit_name.split('_Fruit_',1)[0]
            if name.startswith(truss+'_') and '_Truss_' in '_'+name:return True
        return False

    def set_window(self,index):
        self.window_index=index;m=self.model;released=set(self.at.ids[~self.data.eq_active[self.at.eqs].astype(bool)])
        for wi,ids in enumerate(self.wids):
            gids=np.flatnonzero(np.isin(m.geom_bodyid,list(ids)))
            m.geom_contype[gids]=self.base_mask[0][gids] if wi==index else 0
            m.geom_conaffinity[gids]=self.base_mask[1][gids] if wi==index else 0
            detached=gids[np.isin(m.geom_bodyid[gids],list(released))]
            m.geom_contype[detached]=self.base_mask[0][detached];m.geom_conaffinity[detached]=self.base_mask[1][detached]

    def poses(self,window):
        names=window['bodies'];ids=[self.model.body(n).id for n in names]
        return np.c_[self.data.xpos[ids],self.data.xquat[ids][:,[1,2,3,0]]]

    def record(self,phase,command,drive):
        d=self.data;m=self.model;mj.mj_kinematics(m,d)
        self.rows.append(dict(time=float(d.time),qpos=d.qpos.copy(),qvel=d.qvel.copy(),eq_active=d.eq_active.copy(),
            phase=phase,window_index=self.window_index,command=command.copy(),drive=drive.copy(),
            robot_q=d.qpos[self.qadr].copy(),tool_pose=np.r_[d.xpos[m.body('Tool_fixed').id],d.xquat[m.body('Tool_fixed').id]],
            fruit_positions=d.xpos[self.at.ids].copy()))

    def step(self,command,belt,phase):
        m=self.model;d=self.data;attached=d.eq_active[self.at.eqs].copy();solver_time=float(d.time)
        start=time.perf_counter();mj.mj_step1(m,d);self.physics_s+=time.perf_counter()-start
        d.xfrc_applied[:]=0;d.qfrc_applied[:]=self.reference_plant_preload
        torque,angles=self.at.apply(d)
        # Finite actuators provide gravity feedforward. Their original force
        # caps remain enforced by MuJoCo, including the 10N jaw caps.
        drive=command.copy()
        if self.gravity_compensation:drive+=d.qfrc_bias[self.vadr]/self.kp
        d.ctrl[self.aids]=drive
        m.geom_surfacevel[:]=0
        for side,gids in enumerate(self.belt_geoms):
            rot=R.from_quat(m.geom_quat[gids][:,[1,2,3,0]])
            m.geom_surfacevel[gids,:3]=rot.inv().apply(np.tile([0,belt[side],0],(len(gids),1)))
        start=time.perf_counter();mj.mj_step2(m,d);self.physics_s+=time.perf_counter()-start
        forces,events=self.at.measure_breaks(d,torque,phase)
        for event in events:event.update(solver_time_s=solver_time,release_effective_time_s=float(d.time),window=self.windows[self.window_index]['id'])
        self.breaks.extend(events)
        for ci,c in enumerate(d.contact):
            if c.efc_address<0:continue
            bodies=m.geom_bodyid[[c.geom1,c.geom2]];names=[m.body(int(b)).name for b in bodies]
            released=any(b in self.at.ids and not attached[np.flatnonzero(self.at.ids==b)[0]] for b in bodies)
            released_exception=released and self.released_contact_allowed(bodies,[c.geom1,c.geom2],attached)
            force=np.zeros(6);mj.mj_contactForce(m,d,ci,force);norm=float(np.linalg.norm(force[:3]));pen=max(0.,-float(c.dist))
            robot=any(b in self.robotids for b in bodies)
            if released:self.max_released_pen=max(self.max_released_pen,pen)
            if not released_exception:
                self.max_active_pen=max(self.max_active_pen,pen)
                if pen>.001 and self.first_violation is None:self.first_violation=dict(t=solver_time,phase=phase,bodies=names,
                        geoms=[m.geom(int(c.geom1)).name,m.geom(int(c.geom2)).name],penetration_m=pen)
                if pen>.001 and not self.diagnostic_continue:self.stop='active_contact_penetration_1mm'
            self.max_force=max(self.max_force,norm)
            if robot or pen>.001 or norm>1.:self.contacts.append([solver_time,phase,*names,int(c.geom1),int(c.geom2),float(c.dist),norm,released,released_exception])
            if not released_exception and pen>.010:self.stop='gross_active_overlap_10mm'
        self.peak_actuator=np.maximum(self.peak_actuator,np.abs(d.actuator_force[self.aids]))
        self.tracking=np.maximum(self.tracking,np.abs(d.qpos[self.qadr]-command))
        if not np.isfinite(d.qpos).all() or any(w.number for w in d.warning):self.stop='solver_instability'
        if phase!=self.last_phase:
            self.phase_events.append(dict(t=solver_time,phase=phase,window=self.windows[self.window_index]['id']))
            self.last_phase=phase
        self.tick+=1
        actual_hz=round(1/m.opt.timestep);self.steps_by_hz[actual_hz]=self.steps_by_hz.get(actual_hz,0)+1
        if d.time+1e-9>=self.next_save_s or self.stop:
            self.record(phase,command,drive);self.next_save_s=(np.floor((d.time+1e-9)*30)+1)/30
        if d.time+1e-9>=self.next_log_s:
            print(f'sim={d.time:.1f}s plant={self.windows[self.window_index]["id"]} {phase} detached={len(self.breaks)} active_overlap={self.max_active_pen*1000:.3f}mm',flush=True)
            self.next_log_s+=5.

    def execute(self,plan,output,free_hz=None):
        started=float(self.data.time);guarded=set();guardlog=[];phase='settle';rail_checked=False
        end=plan['settle_seconds']+sum(r['duration_s'] for r in plan['phases'])
        while self.data.time-started<end-1e-9 and self.stop is None:
            t=float(self.data.time-started);sample=sample_plan(plan,t);phase=sample['phase']
            frequency=phase_frequency(phase,self.hz,free_hz)
            workers=0 if self.threads_in_contact_only and phase in FREE_PHASES else self.threads
            if workers!=self.active_threads:
                mj.mju_threadpool(self.data,workers);self.active_threads=workers
                self.worker_events.append(dict(t=float(self.data.time),phase=phase,workers=workers))
            if abs(self.model.opt.timestep-1/frequency)>1e-12:
                self.model.opt.timestep=1/frequency
                self.frequency_events.append(dict(t=float(self.data.time),phase=phase,hz=frequency))
            if phase not in ('settle','preopen','rail_move','rail_settle','idle') and not rail_checked:
                check=rail_arrival_check(float(self.data.qpos[self.qadr[0]]),float(sample['command'][0]))
                guardlog.append(dict(t=float(self.data.time),phase=phase,rail_arrival=check))
                rail_checked=True
                if not check['reached']:self.stop='measured_rail_arrival_not_reached';break
            opening=plan.get('preparation_opening') or {}
            must_open=phase in opening.get('feedback_before_phases',[]) or phase in ('cycle_reclose','withdraw')
            if must_open and phase not in guarded:
                gap=.044+self.data.qpos[self.qadr[9]]-self.data.qpos[self.qadr[8]]
                okay=abs(gap-.096)<=.0005
                guardlog.append(dict(t=float(self.data.time),phase=phase,actual_gap_m=float(gap),reached=bool(okay)))
                guarded.add(phase)
                if not okay:self.stop='measured_opening_not_reached';break
            self.step(sample['command'],belt_velocity_from_physx(sample['authored_belt_speed']),phase)
        self.record(phase,sample['command'],self.data.ctrl[self.aids])
        write(output/'opening_feedback.json',guardlog)
        return dict(start_s=started,end_s=float(self.data.time),completed=self.stop is None,stop=self.stop)

    def save(self,output,elapsed,cycles):
        m=self.model;d=self.data
        mj.mj_saveModel(m,str(output/'model.mjb'))
        rows=self.rows;keys=('time','qpos','qvel','eq_active','phase','window_index','command','drive','robot_q','tool_pose','fruit_positions')
        np.savez_compressed(output/'states.npz',**{key:np.asarray([r[key] for r in rows]) for key in keys},
                            mocap_pos=np.empty((len(rows),0,3)),mocap_quat=np.empty((len(rows),0,4)))
        write(output/'contacts.json',dict(columns=['solver_time_s','phase','body1','body2','geom1','geom2','signed_distance_m','force_norm_n','released_fruit','released_contact_exception'],rows=self.contacts))
        write(output/'phase_events.json',self.phase_events);write(output/'break_events.json',self.breaks)
        write(output/'frequency_events.json',self.frequency_events)
        write(output/'worker_events.json',self.worker_events)
        summary=dict(mujoco_version=mj.__version__,mode='dynamic_robot',hz=self.hz,model_dir=str(self.directory),
            native_worker_threads=self.threads,
            native_workers_in_contact_only=self.threads_in_contact_only,
            load_s=self.load_s,simulated_s=float(d.time),loop_wall_s=elapsed,realtime_factor=float(d.time)/elapsed,
            physics_wall_s=self.physics_s,rss_mib=psutil.Process().memory_info().rss/2**20,
            stop=self.stop,first_1mm_violation=self.first_violation,
            active_contact_criterion_passed=self.first_violation is None and self.stop is None,
            active_contact_penetration_m=self.max_active_pen,released_penetration_m=self.max_released_pen,
            max_contact_force_n=self.max_force,breaks=self.breaks,cycles=cycles,nbody=m.nbody,nv=m.nv,ngeom=m.ngeom,
            actuator_peak_force_n_or_nm=dict(zip(DOF_NAMES,self.peak_actuator.tolist())),
            max_tracking_m_or_rad=dict(zip(DOF_NAMES,self.tracking.tolist())),
            actual_station_y_m=self.meta['rail_origin_station_m']+float(d.qpos[self.qadr[0]]),
            plant_preload='Fixed authored-pose native Jacobian preload, matching source initialization semantics',
            robot_bias_feedforward=self.gravity_compensation,force_caps_unchanged=True,
            diagnostic_continues_after_1mm=self.diagnostic_continue,
            released_gutter_impacts_diagnostic=self.released_gutter_diagnostic,
            prescribed_physical_pose=False,physics_advanced=True,training_eligible=False,harvest_success=None,
            full_physics_validated=False,states_sha256=sha(output/'states.npz'))
        summary.update(cpu_process_s=time.process_time()-self.cpu_started,
                       loaded_cached_model=self.loaded_cached_model,
                       source_model_xml_sha256=self.meta['model_sha256'],
                       model_mjb_sha256=sha(output/'model.mjb'),
                       maximum_fruit_displacement_m=float(np.linalg.norm(d.xpos[self.at.ids]-self.initial_fruit_positions,axis=1).max()),
                       end_to_end_collection_verified=False)
        summary['physics_steps_by_hz']=self.steps_by_hz
        write(output/'summary.json',summary);return summary


def run(model_dir,output,hz=120,windows=2,max_cycles=2,source_first=True,gravity_compensation=True,seconds=None,depth=None,diagnostic_continue=False,free_hz=None,released_gutter_diagnostic=False,threads=0,threads_in_contact_only=False):
    if free_hz is not None and not 0<free_hz<=hz:raise ValueError('Free-motion frequency must be positive and no higher than contact frequency')
    if threads<0:raise ValueError('Nonnegative native worker count required')
    output=Path(output);output.mkdir(parents=True,exist_ok=False);s=Session(model_dir,hz,gravity_compensation,diagnostic_continue,released_gutter_diagnostic,threads,threads_in_contact_only)
    code=output/'code';code.mkdir()
    for path in Path(__file__).parent.glob('*.py'):shutil.copy2(path,code/path.name)
    write(output/'invocation.json',dict(argv=sys.argv,python=sys.executable,mujoco_version=mj.__version__,
          code_sha256={p.name:sha(p) for p in code.glob('*.py')},model_xml_sha256=s.meta['model_sha256']))
    if 'rail_origin_station_m' not in s.meta:
        man=json.loads(Path(s.meta['source_manifest']).read_text());s.meta['rail_origin_station_m']=man['rail']['initial_station_y_m']
    results=[];began=time.perf_counter();planning_s=0.;loop_elapsed=0.
    for wi,window in enumerate(s.windows[:windows]):
        if s.stop:break
        s.set_window(wi);mj.mj_forward(s.model,s.data)
        if window.get('source_plan_sha256') and sha(window['source_plan'])!=window['source_plan_sha256']:
            raise ValueError('Window source plan hash mismatch: '+window['source_plan'])
        source=json.loads(Path(window['source_plan']).read_text())
        policy_path=Path(window['source_trial'])/'harvest_group_policy.json'
        source_manifest=json.loads((Path(window['source_assembly'])/'motion_manifest.json').read_text())
        source_fruits={f['name']:f for f in source_manifest['plant']['fruits']}
        from harvest_group_policy import load_policy
        policy=load_policy(policy_path,window['source_assembly'],
                           [source_fruits[f['name']] for f in window['fruit_specs']])
        # The reused selector requires canonical Truss_XX_Fruit_YY identities.
        # Only native body/equality paths acquire the persistent MJB prefix.
        fruits=[{**f,'path':window['prefix']+f['name'],
                 'native_name':window['prefix']+f['name']} for f in window['fruit_specs']]
        direction=np.asarray(source['approach_direction_world'],float)
        grasp_depth=source['grasp_depth_m'] if depth is None else float(depth)
        if not .0<grasp_depth<=.158:raise ValueError('Grasp depth must lie inside the existing belt span')
        effective_height=(source_manifest.get('grasp_reference') or {}).get('height_offset_m',0.)+source['grasp_height_offset_m']
        selector=HarvestGroupSelector(policy,lambda *args:direction,depth_m=grasp_depth,height_offset_m=effective_height,output=output/f'groups_plant{window["id"]}.json')
        recorded_sequence=Path(window['source_trial'])/'truss_sequence.json'
        recorded_groups=json.loads(recorded_sequence.read_text()).get('cycle_plans',[]) if recorded_sequence.exists() else []
        attempts={};paths=window['bodies']
        for ci in range(max_cycles):
            if s.stop:break
            local_breaks=[dict(joint=f['joint'],name=f['name']) for f in fruits
                          if not s.data.eq_active[s.model.equality(f['native_name']+'_attachment').id]]
            group=selector(fruits,local_breaks,paths,s.poses(window),attempts,max_attempts=2)
            if group is None:results.append(dict(window=window['id'],termination=selector.last_decision['reason'],cycle=ci));break
            directory=output/f'plant{window["id"]}_cycle{ci:02d}';directory.mkdir()
            request=dict(measured_joint_q=s.data.qpos[s.qadr[1:8]].tolist(),
                measured_jaw_q=s.data.qpos[s.qadr[8:]].tolist(),
                measured_station_y_m=s.meta['rail_origin_station_m']+float(s.data.qpos[s.qadr[0]]),
                rail_origin_station_m=s.meta['rail_origin_station_m'],
                station_y_m=source.get('rail_destination',{}).get('station_y_m',s.meta['rail_origin_station_m']),
                target_center=group['aim_world_m'],direction=direction.tolist(),
                depth_m=grasp_depth,height_offset_m=effective_height)
            write(directory/'selected_group.json',group)
            write(directory/'request.json',request);before=time.perf_counter()
            source_group=recorded_groups[0].get('fruits',[]) if recorded_groups else []
            source_goal=np.asarray(source['goal_pose'],float)
            source_belt_aim=source_goal[:3,3]+source_goal[:3,:3]@np.array([0.,.158-source['grasp_depth_m'],.037])
            source_matches=(set(source_group)==set(group['fruits']) and
                            abs(grasp_depth-source['grasp_depth_m'])<=1e-9 and
                            np.linalg.norm(source_belt_aim-np.asarray(group['group_selection']['belt_aim_world_m']))<=1e-5)
            if source_first and wi==0 and ci==0 and source_matches:
                plan={**source,'phases':[dict(row) for row in source['phases']],
                      'settle_seconds':2.,'initial_command':s.data.qpos[s.qadr].tolist(),
                      'initial_gap_m':.044,'belt_ramp_seconds':.5}
                rail_rebase=source_manifest['rail']['initial_station_y_m']-s.meta['rail_origin_station_m']
                for row in plan['phases']:
                    row['rail_waypoint_positions_m']=(np.asarray(row['rail_waypoint_positions_m'])+rail_rebase).tolist()
                plan['source_first_exact_plan_reused']=True
            else:plan=None
            if plan is None:
                try:
                    with (directory/'planning.log').open('w') as stream:
                        subprocess.run([str(REPO/'nvidia-sim/.asset-venv/bin/python'),str(Path(__file__).with_name('plan_adapter.py')),
                           window['source_assembly'],str(directory/'request.json'),str(directory/'plan.json'),'--source-plan',window['source_plan']],check=True,stdout=stream,stderr=subprocess.STDOUT)
                except subprocess.CalledProcessError as error:
                    result=dict(window=window['id'],cycle=ci,completed=False,executed=False,
                                termination='planning_process_failed',returncode=error.returncode,selected_group=group['fruits'])
                    results.append(result);write(directory/'planning_failure.json',result)
                    planning_s+=time.perf_counter()-before
                    break
                plan=json.loads((directory/'plan.json').read_text())
            write(directory/'plan.json',plan);planning_s+=time.perf_counter()-before
            if seconds is not None:
                # Bounded idle/first-prefix diagnostic, never successful harvest.
                plan={**plan,'phases':plan['phases']};end=min(seconds,plan['settle_seconds']+sum(r['duration_s'] for r in plan['phases']))
                rows=[];remaining=max(0.,end-plan['settle_seconds'])
                for row in plan['phases']:
                    if remaining<=0:break
                    if remaining<row['duration_s']:
                        # Do not compress the original curve to fit a short test.
                        break
                    rows.append(row);remaining-=row['duration_s']
                if not rows:rows=[dict(name='idle',duration_s=seconds,waypoint_q=[plan['start'],plan['start']],end_gap_m=.044,belt_local_y_m_s=[0.,0.],rail_waypoint_positions_m=[float(s.data.qpos[s.qadr[0]])]*2)]
                plan={**plan,'phases':rows,'belt_cycle':None,'settle_seconds':0.}
            before=time.perf_counter();result=s.execute(plan,directory,free_hz);loop_elapsed+=time.perf_counter()-before
            for name in group['fruits']:attempts[name]=attempts.get(name,0)+1
            result.update(window=window['id'],cycle=ci,selected_group=group['fruits']);results.append(result)
            if seconds is not None:break
    summary=s.save(output,max(loop_elapsed,1e-9),results);summary['planning_wall_s']=planning_s;summary['total_wall_s']=time.perf_counter()-began
    write(output/'summary.json',summary)
    from make_report import write_trial_report
    write_trial_report(output)
    print(json.dumps(summary,indent=2));return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--hz',type=int,default=120);p.add_argument('--windows',type=int,default=2);p.add_argument('--max-cycles',type=int,default=2)
    p.add_argument('--replan-first',action='store_true');p.add_argument('--no-gravity-feedforward',action='store_true');p.add_argument('--seconds',type=float)
    p.add_argument('--depth-m',type=float,help='Optional shallower belt engagement, using existing IK and group footprint')
    p.add_argument('--diagnostic-continue',action='store_true',help='Explicitly continue after 1mm invalidity, never counted valid; gross overlap still stops')
    p.add_argument('--free-hz',type=int,help='Opt-in lower free-motion frequency; contact phases retain --hz; separately validate')
    p.add_argument('--released-gutter-diagnostic',action='store_true',help='Keep physical dropped-fruit/gutter impacts, report separately without aborting harvesting; robot/gutter and attached-fruit rules unchanged')
    p.add_argument('--threads',type=int,default=0,help='Native worker threads per environment; 0 uses the single-thread pipeline')
    p.add_argument('--threads-in-contact-only',action='store_true',help='Use native workers only outside declared free-motion phases; avoids thread overhead during rail/preparation')
    a=p.parse_args();run(a.model,a.output,a.hz,a.windows,a.max_cycles,not a.replan_first,not a.no_gravity_feedforward,a.seconds,a.depth_m,a.diagnostic_continue,a.free_hz,a.released_gutter_diagnostic,a.threads,a.threads_in_contact_only)
