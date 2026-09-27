"""Search breadth and result semantics, not greenhouse harvesting success."""
import copy
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import motion_family_search as mf


def geometry():
    from hook_seating_geometry import Capsule
    # Independent finite geometry. +Y is the plane normal, rear is -X.
    rear=[]
    angles=np.linspace(3*np.pi/4,5*np.pi/4,17)
    for i in range(16):
        a=.0275*np.array([np.cos(angles[i]),0,np.sin(angles[i])])
        b=.0275*np.array([np.cos(angles[i+1]),0,np.sin(angles[i+1])])
        rear.append(Capsule('rear_'+str(i),a,b,.001))
    return dict(center=np.array([0.,0.,.5]),radius=.014,heading=np.array([1.,0.,0.]),
                targets=[Capsule('pedicel',np.array([0.,0.,.515]),np.array([.01,.002,.535]),.001)],
                rear_wires=rear,ring_radius=.0275)


def config():
    p={k:(lo+hi)/2 for k,(lo,hi) in mf.RANGES.items()}
    p.update(azimuth_deg=20.,roll_deg=12.,pitch_deg=-8.,entry_twist_deg=30.,target_selector=0.,wire_selector=.5)
    return p


@pytest.mark.parametrize('family',mf.FAMILIES)
def test_whole_paths_have_explicit_pose_schema(family):
    c=mf.make_candidate(family,config(),**geometry(),candidate_id='test')
    rows=c['pose_waypoints']
    assert 2<=len(rows)<=16 and rows[0]['phase']=='preapproach'
    assert rows[-1]['phase']=='verify'
    assert any(r['phase']=='insert' for r in rows)
    assert all(0<=r['minimum_seconds']<=10 for r in rows)
    assert all(np.isclose(np.linalg.norm(r['orientation_xyzw']),1.) for r in rows)
    assert c['training_eligible'] is False and c['hook_success'] is None


def test_families_are_not_five_names_for_one_line():
    paths={f:mf.make_candidate(f,config(),**geometry(),candidate_id=f)['pose_waypoints'] for f in mf.FAMILIES}
    assert len({mf.digest(p) for p in paths.values()})==5
    for f in ('flank_left','flank_right'):
        xyz=np.array([r['ring_position_xyz'] for r in paths[f] if r['phase']=='insert'])
        assert np.linalg.matrix_rank(xyz[1:]-xyz[0],tol=1e-8)>=2
    side=paths['side_mouth']
    assert not np.allclose(side[0]['ring_position_xyz'],paths['under_center'][0]['ring_position_xyz'])


def test_pivot_moves_position_with_orientation():
    rows=mf.make_candidate('pivot_sweep',config(),**geometry(),candidate_id='pivot')['pose_waypoints']
    rows=[r for r in rows if r['phase']=='insert']
    center=geometry()['center']
    # The first insert is the pivot start; subsequent pivot poses keep fruit-relative coordinates.
    values=[Rotation.from_quat(r['orientation_xyzw']).inv().apply(center-np.array(r['ring_position_xyz'])) for r in rows]
    assert np.max(np.linalg.norm(np.array(values)-values[0],axis=1))<1e-9


def test_translation_equivariance_and_inputs_preserved():
    g=geometry();p=config();p0=copy.deepcopy(p)
    a=mf.make_candidate('flank_left',p,**g,candidate_id='a')
    from hook_seating_geometry import Capsule
    shift=np.array([.2,-.4,1.])
    bgeom=dict(g,center=g['center']+shift,
               targets=[Capsule(x.name,x.a+shift,x.b+shift,x.radius) for x in g['targets']])
    b=mf.make_candidate('flank_left',p,**bgeom,candidate_id='b')
    for x,y in zip(a['pose_waypoints'],b['pose_waypoints']):
        assert np.allclose(np.array(x['ring_position_xyz'])+shift,y['ring_position_xyz'])
        assert np.allclose(x['orientation_xyzw'],y['orientation_xyzw'])
    assert p==p0


def test_sobol_reproducible_prefix_and_bounded():
    assert mf.parameters(4,6)==mf.parameters(8,6)[:4]
    assert mf.parameters(4,6)!=mf.parameters(4,7)
    for p in mf.parameters(4,6):
        assert all(lo<=p[k]<=hi for k,(lo,hi) in mf.RANGES.items())


@pytest.mark.parametrize('bad',[0,3,65,True,-1])
def test_parameter_budget_validation(bad):
    with pytest.raises(ValueError):mf.parameters(bad,2)


def test_round_robin_generation_and_bounded_refinement():
    cs,rej=mf.generate_candidates(geometry(),4,22)
    assert len(cs)+len(rej)==20
    assert {c['family'] for c in cs}==set(mf.FAMILIES)
    assert len({c['candidate_id'] for c in cs})==len(cs)
    again,_=mf.generate_candidates(geometry(),4,22,parents=[cs[0]])
    refined=[c for c in again if c['parent_id']]
    assert 1<=len(refined)<=2
    assert all(c['search_parameters']['azimuth_deg']!=cs[0]['search_parameters']['azimuth_deg'] for c in refined)


@pytest.mark.parametrize('outcome',['ik_not_found','path_blocked','audit_inconclusive','experimental_limit_stop','motion_completed_no_capture_evidence'])
def test_failed_search_never_becomes_impossible(outcome):
    r=mf.summarize_target([dict(outcome=outcome,family='under_center',physics_executed=False)])
    assert r['impossible'] is None and r['hook_success'] is None
    assert r['status']=='unresolved_within_budget'


def test_geometric_candidate_not_failed_or_certified_capture():
    result=dict(physics_executed=True,completed=True,authorized_contact_evidence=dict(contact_retention_evidence=False))
    assert mf.trial_outcome(result,[dict(seated=True,target_contact=False)])=='geometric_candidate'
    result['stop_reasons']=['forbidden_force']
    assert mf.trial_outcome(result,[dict(seated=True)])=='other_object_contact'


def test_complete_motion_is_not_capture():
    assert mf.trial_outcome(dict(physics_executed=True,completed=True),[])=='motion_completed_no_capture_evidence'
    assert mf.trial_outcome(dict(physics_executed=False,audit_status='sampled_clear'),[])=='audit_only_clear'
