"""Independent finite-capsule controls with actual authored rear-wire shapes.

Kinematic fixtures, including the positive, are never normal-start robot success.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation
from hook_seating_geometry import Capsule,seating_geometry
from hook_retention_diagnostic import rear_capsule_geometry
from seating_evaluation import contact_identity,timeline_evidence
from diagnose_contact_timing import force_rows
from trajectory_search import center_region


def controls(goal_file,output):
    src=json.loads(Path(goal_file).read_text());wires=[Capsule(w['name'],w['a'],w['b'],w['radius_m']) for w in src['rear_wire_capsules_ring']]
    radius=src['search']['ring_radius_m'];targetname=src['target_capsules_world'][0]['name'];wr=max(w.radius for w in wires);r=.001
    # Independently specified coordinates, not seating_goal output.
    locations={'positive':-radius+wr+r+.0004,'fruit_only':0.,'front':radius-wr-r-.0004,'outside':-radius-wr-r-.0004,'penetration':-radius+wr,
               'wrong_pedicel':-radius+wr+r+.0004,'rachis':-radius+wr+r+.0004}
    output=Path(output);output.mkdir(parents=True,exist_ok=False);results=[]
    for name,x in locations.items():
        identity='other_pedicel' if name=='wrong_pedicel' else 'Rachis' if name=='rachis' else targetname
        target=Capsule(identity,[x,-.01,0],[x,.01,0],r)
        xml='<mujoco><option gravity="0 0 0" timestep="0.004166666666666667"/><worldbody>'
        for w in wires:
            s=' '.join(map(str,[*w.a,*w.b]));xml+=f'<geom name="{w.name}" type="capsule" fromto="{s}" size="{w.radius}" margin=".0005" gap=".0005" rgba="0 .8 1 1"/>'
        if name=='fruit_only':
            xml+='<geom name="fruit_visual_only" type="sphere" pos="-.012 0 0" size=".0116" contype="0" conaffinity="0" rgba="1 0 0 .5"/>'
        xml+=f'<body><freejoint/><geom name="{identity}" type="capsule" fromto="{x} -.01 0 {x} .01 0" size="{r}" mass=".01" margin=".0005" gap=".0005" rgba="1 .3 .1 1"/></body></worldbody></mujoco>'
        model=mj.MjModel.from_xml_string(xml);data=mj.MjData(model);mj.mj_forward(model,data)
        geometry=seating_geometry(target,[0,0,0],Rotation.identity(),wires,ring_radius=radius,max_surface_gap_m=.0015)
        native=min(mj.mj_geomDistance(model,data,model.geom(identity).id,model.geom(w.name).id,.1,np.zeros(6)) for w in wires)
        assert abs(native-geometry['minimum_rear_surface_gap_m'])<2e-8
        legacy=rear_capsule_geometry([(target.a.copy(),target.b.copy(),target.radius)],np.zeros(3),Rotation.identity())[1]
        contacts=force_rows(model,data);correct=identity==targetname
        result=dict(name=name,expected_target_seating=name=='positive',geometry=geometry,legacy_seated=bool(legacy),identity_matches=correct,
                    accepted_target_geometry=bool(correct and geometry['geometric_seating_candidate']),native_gap_m=native,
                    contacts=contacts,intended_force=any(contact_identity(c,[targetname],[w.name for w in wires],'hold') and c['normal_force_N']>.01 for c in contacts),
                    automatic_entry_success=False,hook_success=None,training_eligible=False,scope='Prescribed independent static geometry/contact fixture, no robot rollout')
        result['maximum_normal_force_N']=max([c['normal_force_N'] for c in contacts]+[0.])
        result['fruit_center_occupancy_only']=bool(name=='fruit_only' and center_region([-.012,0,0],.0116))
        if name=='fruit_only':assert result['fruit_center_occupancy_only'] and not result['accepted_target_geometry']
        assert result['accepted_target_geometry']==result['expected_target_seating'],name
        (output/f'{name}.xml').write_text(xml);results.append(result)
    def row(t,phase,seated=True):return dict(time_s=t,phase=phase,seated=seated,target_contact=seated,non_target_force_N=0.,physics_valid=True)
    ts=np.arange(0,1.6+1e-8,1/240)
    base=[row(float(t),'seat' if t<.2 else 'hold' if t<1.3 else 'verify') for t in ts]
    timeline={}
    for name,initial,rows in [('prescribed_entry_evidence',False,base),('preseated_stationary',True,base),('transient_exit',False,[dict(r,seated=r['time_s']<.5,target_contact=r['time_s']<.5) for r in base]),('missing_samples',False,[r for r in base if not .5<r['time_s']<.8])]:
        timeline[name]=timeline_evidence(rows,initially_seated=initial,dt=1/240)
    assert timeline['prescribed_entry_evidence']['contact_retention_evidence']
    assert not any(timeline[n]['contact_retention_evidence'] for n in ['preseated_stationary','transient_exit','missing_samples'])
    report=dict(fixtures=results,timeline_controls=timeline,timeline_scope='Synthetic timestamps for evaluator controls, not real physics or robot entry',goal_source=str(goal_file),diagnostic_only=True,training_eligible=False)
    (output/'controls.json').write_text(json.dumps(report,indent=2));return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('goals',type=Path);p.add_argument('output',type=Path);a=p.parse_args();controls(a.goals,a.output)
