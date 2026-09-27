from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from seating_evaluation import timeline_evidence,contact_identity

def series():
    return [dict(time_s=float(t),phase='seat' if t<.2 else 'hold' if t<1.3 else 'verify',seated=True,target_contact=True,non_target_force_N=0.,physics_valid=True) for t in np.arange(0,1.6,1/240)]

def test_preseated_is_not_automatic_entry():
    assert timeline_evidence(series(),initially_seated=False,dt=1/240)['contact_retention_evidence']
    assert not timeline_evidence(series(),initially_seated=True,dt=1/240)['contact_retention_evidence']

@pytest.mark.parametrize('failure',['gap','exit','wrong_contact','penetration','short_hold','non_target'])
def test_retention_requires_continuity_identity_and_physics(failure):
    rows=series()
    if failure=='gap':rows=[r for r in rows if not .5<r['time_s']<.8]
    if failure=='exit':rows=[dict(r,seated=r['time_s']<.8) for r in rows]
    if failure=='wrong_contact':rows=[dict(r,target_contact=False) for r in rows]
    if failure=='penetration':rows=[dict(r,physics_valid=False) for r in rows]
    if failure=='short_hold':rows=[dict(r,time_s=r['time_s']*.5) for r in rows]
    if failure=='non_target':rows=[dict(r,non_target_force_N=.1) for r in rows]
    assert not timeline_evidence(rows,initially_seated=False,dt=1/240)['contact_retention_evidence']

def test_exact_target_wire_and_phase_required():
    for phase in ['seat','hold','verify']:assert contact_identity({'geoms':['wire','target']},['target'],['wire'],phase)
    for names,phase in [(['wire','Rachis'],'seat'),(['wire','other_pedicel'],'hold'),(['mount','target'],'seat'),(['wire','target'],'insert')]:assert not contact_identity({'geoms':names},['target'],['wire'],phase)
