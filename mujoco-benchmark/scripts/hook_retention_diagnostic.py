"""Legacy Isaac seating geometry reused as a separate MuJoCo diagnostic.

This does not change center_entry or hook_success. It measures a candidate proxy;
actual target contact permissions and harvesting/detachment remain unvalidated.
"""
import sys
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
from pose_candidates import rear_capsule_geometry, DEFAULT_LIMITS
from trajectory_search import center_region
from geometry import RING_RADIUS, WIRE_RADIUS
from suite import RING
from diagnose_contact_timing import force_rows


def capsule_endpoints(model,data,gid):
    if int(model.geom_type[gid]) != int(mj.mjtGeom.mjGEOM_CAPSULE):
        raise ValueError('Retention requires actual capsule geometry')
    axis=data.geom_xmat[gid].reshape(3,3)[:,2]*model.geom_size[gid,1]
    return data.geom_xpos[gid]-axis,data.geom_xpos[gid]+axis,float(model.geom_size[gid,0])


class HookProbe:
    def __init__(self,engine,hold_start,sample_hz=60):
        self.engine=engine;self.hold_start=hold_start;self.sample_hz=sample_hz
        m=engine.model;self.private=mj.MjData(m);self.rows=[];self.count=0
        self.stride=round(1/m.opt.timestep/sample_hz)
        from target_truss_identity import target_names
        self.target_names=list(target_names(engine.target,engine.target_spec['anchor'])['pedicels'])
        self.target_ids=[m.geom(n).id for n in self.target_names]
        self.rear_ids=[]
        for g in engine.hookgeoms:
            p=m.geom_pos[g]-RING
            if int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_CAPSULE) and abs(m.geom_size[g,0]-WIRE_RADIUS)<1e-6:
                angle=abs(np.arctan2(p[2],p[0]))
                if angle>=3*np.pi/4-1e-6:self.rear_ids.append(int(g))
        if len(self.rear_ids)!=16:raise ValueError(f'Expected authored 16 rear arc capsules, found {len(self.rear_ids)}')
        self.rear_names={m.geom(g).name for g in self.rear_ids}
        self.other_pedicels=[g for g in range(m.ngeom) if ('glb_col_' in (m.geom(g).name or '')) and
                            ('Attachment_' in m.geom(g).name or 'Pedicel_proximal_' in m.geom(g).name) and g not in self.target_ids]
        bodies=set()
        for i in range(1,m.nbody):
            if (m.body(i).name or '').startswith('Robot_') or m.body(i).name=='Hook' or int(m.body_parentid[i]) in bodies:bodies.add(i)
        self.robot_geoms={g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in bodies}
        self.fruit=engine.fruit
        local=self.fruit_local(engine.data)
        self.initial_outside=not center_region(local,engine.target_spec['radius'])

    def frame(self,data):
        e=self.engine;rot=Rotation.from_matrix(data.xmat[e.hook].reshape(3,3))
        return data.xpos[e.hook]+rot.apply(RING),rot

    def fruit_local(self,data):
        p,r=self.frame(data);e=self.engine
        return r.inv().apply(data.xpos[e.fruit]+data.xmat[e.fruit].reshape(3,3)@np.array(e.target_spec['center'])-p)

    def __call__(self,m,data):
        self.count+=1
        if self.count%self.stride:return
        d=self.private;mj.mj_copyData(d,m,data);mj.mj_forward(m,d)
        p,r=self.frame(d);seated_names=[];gaps=[]
        for g,name in zip(self.target_ids,self.target_names):
            gap,seated=rear_capsule_geometry([capsule_endpoints(m,d,g)],p,r);gaps.append(gap)
            if seated:seated_names.append(name)
        contacts=force_rows(m,d,robot_ids=self.robot_geoms)
        active=[c for c in contacts if c['normal_force_N']>=DEFAULT_LIMITS['contact_force_floor_N']]
        intended=[];forbidden=[];wrong=[]
        for c in active:
            names=set(c['geoms'])
            if names & self.rear_names and names & set(self.target_names):intended.append(c)
            else:forbidden.append(c)
            if names & self.rear_names:
                for g in self.other_pedicels:
                    if m.geom(g).name in names and rear_capsule_geometry([capsule_endpoints(m,d,g)],p,r)[1]:wrong.append(c);break
        anchor=d.xpos[m.geom_bodyid[self.target_ids[0]]]
        self.rows.append(dict(time_s=float(data.time),phase='hold' if data.time>self.hold_start+1e-8 else 'path',
                              center_entry=bool(self.initial_outside and center_region(self.fruit_local(d),self.engine.target_spec['radius'])),
                              seated=bool(seated_names),seated_geoms=seated_names,min_rear_gap_m=float(min(gaps)),
                              intended_contact=bool(intended),seated_intended_contact=any(set(c['geoms'])&set(seated_names) for c in intended),
                              forbidden_contacts=forbidden,wrong_pedicel_contacts=wrong,
                              relative_anchor=r.inv().apply(anchor-p).tolist(),
                              force_timing='private forward recomputation at poststep time'))

    def summary(self,physics):
        held=[r for r in self.rows if r['phase']=='hold'];tail=held[-max(1,round(self.sample_hz*.5)):]
        stable=float(np.max(np.linalg.norm(np.array([r['relative_anchor'] for r in tail])-tail[-1]['relative_anchor'],axis=1))) if tail else None
        retained=(len(held)>=round(DEFAULT_LIMITS['hold_seconds']*self.sample_hz) and all(r['seated'] for r in held)
                  and sum(r['seated_intended_contact'] for r in held)>=3 and stable<=DEFAULT_LIMITS['stable_relative_motion_m'])
        return dict(target=self.engine.target,target_capsules=self.target_names,rear_capsules=sorted(self.rear_names),
                    legacy_limits=DEFAULT_LIMITS,center_entry_sampled=any(r['center_entry'] for r in self.rows),
                    seated_ticks=sum(r['seated'] for r in self.rows),intended_contact_ticks=sum(r['intended_contact'] for r in self.rows),
                    held_ticks=len(held),held_seated_ticks=sum(r['seated'] for r in held),held_contact_ticks=sum(r['seated_intended_contact'] for r in held),
                    stable_relative_motion_m=stable,legacy_retention_proxy=bool(retained),
                    forbidden_force_ticks=sum(bool(r['forbidden_contacts']) for r in self.rows),
                    wrong_pedicel_ticks=sum(bool(r['wrong_pedicel_contacts']) for r in self.rows),
                    physics_valid=physics.get('glb_physics_valid'),hook_success=None,
                    scope='60Hz legacy geometry/contact/1s retention proxy, not validated harvesting success; no collision permissions changed')
