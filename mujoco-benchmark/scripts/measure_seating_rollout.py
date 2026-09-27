"""240Hz offline seating evidence on a private MjData; live rollout unchanged."""
import json,gzip
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation
from hook_retention_diagnostic import HookProbe,capsule_endpoints,rear_capsule_geometry
from hook_seating_geometry import Capsule,seating_geometry
from seating_evaluation import contact_identity,timeline_evidence
from diagnose_contact_timing import force_rows
from geometry import RING_RADIUS

class SeatingProbe:
    def __init__(self,engine,trace):
        self.engine=engine;self.trace=trace;self.mapping=HookProbe(engine,0);self.private=mj.MjData(engine.model);self.rows=[];self.events=[]
        if engine.model.nplugin or any(getattr(mj,n)() is not None for n in dir(mj) if n.startswith('get_mjcb_')):raise ValueError('Private forward requires no callbacks/plugins')
        p,r=self.mapping.frame(engine.data)
        self.wires=[Capsule(engine.model.geom(g).name,r.inv().apply(a-p),r.inv().apply(b-p),radius) for g in self.mapping.rear_ids for a,b,radius in [capsule_endpoints(engine.model,engine.data,g)]]
        self.initial=self.geometry(engine.data)[0]
    def geometry(self,d):
        p,r=self.mapping.frame(d);metrics=[];legacy=False
        for gid in self.mapping.target_ids:
            a,b,radius=capsule_endpoints(self.engine.model,d,gid)
            if np.linalg.norm((a+b)/2-p)>.1:continue
            capsule=Capsule(self.engine.model.geom(gid).name,a,b,radius)
            metric=seating_geometry(capsule,p,r,self.wires,ring_radius=RING_RADIUS,max_surface_gap_m=.0015);metric['target_geom']=capsule.name;metrics.append(metric)
            legacy|=rear_capsule_geometry([(a,b,radius)],p,r)[1]
        return any(x['geometric_seating_candidate'] for x in metrics),bool(legacy),metrics
    def __call__(self,m,data):
        mj.mj_copyData(self.private,m,data);mj.mj_forward(m,self.private);d=self.private
        phase=self.trace[min(round(data.time*60),len(self.trace)-1)]['phase'];seated,legacy,metrics=self.geometry(d)
        live=force_rows(m,data,robot_ids=self.mapping.robot_geoms);fresh=force_rows(m,d,robot_ids=self.mapping.robot_geoms)
        goodnames=[x['target_geom'] for x in metrics if x['geometric_seating_candidate']]
        intended=[c for c in fresh if contact_identity(c,goodnames,self.mapping.rear_names,phase) and c['normal_force_N']>=.01]
        forbidden=[c for c in fresh if not contact_identity(c,self.mapping.target_names,self.mapping.rear_names,phase)]
        min_dist=min([float(c.dist) for c in d.contact]+[0.]);valid=min_dist>=-.0005 and np.isfinite(d.qpos).all() and not np.any(d.warning.number)
        desired=self.engine.command(data.time);qerror=data.qpos[self.engine.qids]-desired
        self.rows.append(dict(time_s=float(data.time),phase=phase,seated=seated,legacy_seated=legacy,target_contact=bool(intended),
            non_target_force_N=max([c['normal_force_N'] for c in forbidden]+[0.]),physics_valid=bool(valid),max_penetration_m=-min_dist,
            prismatic_error_m=float(abs(qerror[0])),revolute_error_rad=float(max(abs(qerror[1:]))),geometry=metrics,
            max_intended_force_N=max([c['normal_force_N'] for c in intended]+[0.])))
        # Contact rows preserve positive distances, inactive contacts and exact force timing.
        if live or fresh:self.events.append(dict(poststep_time_s=float(data.time),live_solve_time_s=float(data.time-m.opt.timestep),private_forward_time_s=float(data.time),phase=phase,live_previous_solve=live,private_forward=fresh))
    def save(self,folder):
        folder=Path(folder);summary=timeline_evidence(self.rows,initially_seated=self.initial,dt=self.engine.model.opt.timestep)
        summary.update(sample_hz=1/self.engine.model.opt.timestep,seated_samples=sum(r['seated'] for r in self.rows),legacy_seated_samples=sum(r['legacy_seated'] for r in self.rows),intended_contact_samples=sum(r['target_contact'] for r in self.rows),max_penetration_m=max(r['max_penetration_m'] for r in self.rows),max_prismatic_error_m=max(r['prismatic_error_m'] for r in self.rows),max_revolute_error_rad=max(r['revolute_error_rad'] for r in self.rows),contact_timing='live previous solve vs private forward at poststep timestamp; geometry at poststep')
        (folder/'seating_summary.json').write_text(json.dumps(summary,indent=2));(folder/'seating_samples.json.gz').write_bytes(gzip.compress(json.dumps(self.rows).encode(),mtime=0));(folder/'contact_events_240hz.json.gz').write_bytes(gzip.compress(json.dumps(self.events).encode(),mtime=0));return summary
