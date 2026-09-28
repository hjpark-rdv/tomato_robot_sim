"""Server geometry evidence for a covered scene; never executes a robot path.

Controls use an actual ring-wire collider as a geometric probe. They test the
native distance + production screen/classification, not IK reachability.
"""
import argparse,json,tempfile,time,hashlib
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation
from add_neighbor_truss_obstacles import _build_variant_template,_canonical_variant,coverage_from_model
from environment_preflight import Policy,screen
from search_model_cache import sha

def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')

def validate(source,covered,layout,out,controls_only=False):
    out.mkdir(parents=True,exist_ok=True)
    started=time.monotonic()
    if controls_only:
        report=json.loads((out/'geometry.json').read_text())
        if report['hashes'][str(covered/'model.mjb')]!=sha(covered/'model.mjb'):
            raise ValueError('Saved geometry report model hash mismatch')
        m=mj.MjModel.from_binary_path(str(covered/'model.mjb'));d=mj.MjData(m);mj.mj_forward(m,d)
    else:
        old=mj.MjModel.from_binary_path(str(source/'model.mjb'))
        m=mj.MjModel.from_binary_path(str(covered/'model.mjb'));d=mj.MjData(m);mj.mj_forward(m,d)
        od=mj.MjData(old);mj.mj_forward(old,od)
        meta=json.loads((layout/'scene.json').read_text())
        report=dict(schema='covered_scene_geometry_v1',training_eligible=False,hook_success=None,
                    source=str(source),covered=str(covered),layout=str(layout),mujoco=mj.__version__)
        report['coverage']=coverage_from_model(meta,m,True,True)
        dynamics={}
        for key in ('nq','nv','nbody','nu'):
            dynamics[key]=getattr(old,key)==getattr(m,key)
        for key in ('body_mass','body_inertia','dof_damping','dof_armature','jnt_stiffness','actuator_gainprm','actuator_biasprm','qpos0'):
            dynamics[key]=bool(np.array_equal(getattr(old,key),getattr(m,key)))
        geomerr=0.;original_unchanged=True
        for i in range(old.ngeom):
            name=old.geom(i).name;j=m.geom(name).id
            for key in ('geom_type','geom_contype','geom_conaffinity','geom_friction','geom_margin','geom_gap','geom_solref','geom_solimp','geom_size'):
                original_unchanged &= bool(np.array_equal(getattr(old,key)[i],getattr(m,key)[j]))
            geomerr=max(geomerr,float(np.max(abs(od.geom_xpos[i]-d.geom_xpos[j]))),float(np.max(abs(od.geom_xmat[i]-d.geom_xmat[j]))))
        report['original_model_preservation']=dict(dynamics=dynamics,geometry_settings_equal=original_unchanged,max_world_transform_difference=geomerr)
        report['hashes']={str(p):sha(p) for p in [source/'model.mjb',covered/'model.mjb',covered/'model.xml',layout/'scene.json']}
        templates={};errors=[];counts={};missing=[]
        with tempfile.TemporaryDirectory(prefix='covered_transform_') as tmp:
            for plant in meta['placements']:
                for ti,truss in enumerate(plant['trusses']):
                    key,_=_canonical_variant(truss)
                    if key not in templates:templates[key]=_build_variant_template(truss,tmp)[2]
                    rot=Rotation.from_euler('z',plant['yaw_deg']+truss['y_deg'],degrees=True)
                    anchor=np.asarray(truss['attachment_world_m'])
                    for k,row in enumerate(templates[key]):
                        name=f"neighbor_truss_collision_{row['kind']}_p{plant['id']:02d}_t{ti:02d}_g{k:03d}"
                        try:g=m.geom(name).id
                        except KeyError:missing.append(name);continue
                        if row['shape']=='capsule':
                            expected=rot.apply(row['endpoints_m'])+anchor
                            axis=d.geom_xmat[g].reshape(3,3)[:,2]*m.geom_size[g,1]
                            actual=np.array([d.geom_xpos[g]-axis,d.geom_xpos[g]+axis])
                            error=min(np.max(abs(expected-actual)),np.max(abs(expected-actual[::-1])))
                            error=max(error,abs(m.geom_size[g,0]-row['radius_m']))
                        else:
                            from scipy.spatial import cKDTree
                            expected=rot.apply(row['vertices_m'])+anchor
                            mid=m.geom_dataid[g];a=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid]
                            actual=m.mesh_vert[a:a+n]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
                            error=max(cKDTree(actual).query(expected)[0].max(),cKDTree(expected).query(actual)[0].max())
                        counts[row['kind']]=counts.get(row['kind'],0)+1
                        errors.append(dict(name=name,error_m=float(error)))
        errors.sort(key=lambda r:-r['error_m'])
        report['source_transform_reproduction']=dict(passed=not missing and max([r['error_m'] for r in errors],default=0)<1e-6,numerical_tolerance_m=1e-6,counts=counts,missing=missing,worst=errors[:20],scope='Same source collider and placement; not mesh-vs-biological accuracy')
        write(out/'geometry.json',report)
    # Preselect one actual collider of each class, before screening the controls.
    from types import SimpleNamespace
    from hook_retention_diagnostic import HookProbe
    from target_fruit_contact_trial import scope_from_engine
    reference=json.loads((covered/'reference.json').read_text())
    hook=m.body('Hook').id
    engine=SimpleNamespace(model=m,data=d,hook=hook,hookgeoms=np.flatnonzero(m.geom_bodyid==hook),
        target='Tomato_02',fruit=m.body('Tomato_02').id,
        target_spec=next(s for s in reference['fruit_specs'] if s['name']=='Tomato_02'))
    scope=scope_from_engine(engine,HookProbe(engine,0.),search_policy=True)
    wire=m.geom(scope.wires[0]).id
    gids=[next(i for i in range(m.ngeom) if (m.geom(i).name or '').startswith('neighbor_truss_collision_'+kind+'_')) for kind in ('fruit','rachis','pedicel','peduncle')]
    class Probe:
        version='MuJoCo '+mj.__version__;joint_steps=np.full(3,.001)
        robot_names=[m.geom(wire).name];robot_radii=np.array([m.geom_rbound[wire]])
        def __init__(self,gid):
            self.gid=gid;self.environment_names=[m.geom(gid).name];self.environment_positions=d.geom_xpos[[gid]].copy();self.environment_radii=np.array([m.geom_rbound[gid]])
            self.inventory=dict(control='geometric wire probe; no IK, no physical robot motion',source_model_sha256=report['hashes'][str(covered/'model.mjb')]);self.robot_positions=d.geom_xpos[[wire]].copy()
        def set_robot(self,q):d.geom_xpos[wire]=q;self.robot_positions=np.array([q])
        def distance(self,i,j,limit):
            seg=np.zeros(6);v=mj.mj_geomDistance(m,d,wire,self.gid,limit,seg);return float(v),seg
    controls=[]
    for gid in gids:
        backend=Probe(gid);center=d.geom_xpos[gid].copy()
        positive=[center+np.array([.0001,0,0]),center+np.array([.0002,0,0])]
        offset=m.geom_rbound[gid]+m.geom_rbound[wire]+.005
        negative=[center+np.array([offset,0,0]),center+np.array([offset+.002,0,0])]
        for kind,points in [('intersection',positive),('nonintersection',negative)]:
            distances=[]
            for q in np.linspace(points[0],points[1],5):backend.set_robot(q);distances.append(backend.distance(0,0,10)[0])
            result=screen(backend,points,['ready','ready'],1/60,Policy(clearance_m=0),True)
            controls.append(dict(kind=kind,obstacle=m.geom(gid).name,probe=backend.robot_names[0],points=np.asarray(points).tolist(),signed_distances_m=distances,result=result))
    write(out/'controls.json',controls)
    report['controls_passed']=all((r['result']['complete'] and ((not r['result']['passed'] and max(r['signed_distances_m'])<0) if r['kind']=='intersection' else (r['result']['passed'] and min(r['signed_distances_m'])>0))) for r in controls)
    report['wall_s']=time.monotonic()-started
    write(out/'geometry.json',report)
    print(json.dumps(dict(coverage=report['coverage']['passed'],transform=report['source_transform_reproduction']['passed'],controls=report['controls_passed'],wall_s=report['wall_s']),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('covered',type=Path);p.add_argument('layout',type=Path);p.add_argument('output',type=Path);p.add_argument('--controls-only',action='store_true');a=p.parse_args();validate(a.source,a.covered,a.layout,a.output,a.controls_only)
