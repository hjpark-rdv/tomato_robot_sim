import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_planner import motion_waypoints
from pose_candidates import (base_candidate,coarse_candidates,candidate_waypoints,tolerance_candidates,
    classify,rear_capsule_geometry,NOMINAL)
from pose_search import summarize
from geometry import RING_RADIUS


def test_nominal_geometry_matches_existing_approach():
    center=np.array([-.7,1.1,.96]);neck=center+[.005,.003,.015]
    a,waypoints=motion_waypoints(center,neck,NOMINAL)
    b,generated,direction=candidate_waypoints(center,neck,base_candidate())
    assert np.allclose(a.as_matrix(),b.as_matrix())
    assert all(n==m and np.allclose(p,q) for (n,p),(m,q) in zip(waypoints,generated))
    assert np.allclose(direction,[-1,0,0])


def test_coarse_search_is_unique_deterministic_and_finite():
    candidates=coarse_candidates()
    assert len(candidates)==31
    assert json.dumps(candidates)==json.dumps(coarse_candidates())
    assert len({p['candidate_id'] for p in candidates})==31
    assert len({json.dumps({k:v for k,v in p.items() if k!='candidate_id'},sort_keys=True) for p in candidates})==31
    for p in candidates:
        r,poses,d=candidate_waypoints(np.zeros(3),np.array([0,0,.015]),p)
        assert np.isfinite(np.array([v for _,v in poses])).all()
        assert np.linalg.det(r.as_matrix())==pytest.approx(1.)
        assert np.linalg.norm(d)==pytest.approx(1.)


def test_tolerance_isolated_position_and_orientation():
    parent=coarse_candidates()[0];children=tolerance_candidates(parent)
    assert len(children)==36
    center=np.zeros(3);neck=np.array([0,0,.015])
    r,base,_=candidate_waypoints(center,neck,parent)
    base=np.array([v for _,v in base])
    for p in children:
        rr,poses,_=candidate_waypoints(center,neck,p);positions=np.array([v for _,v in poses])
        if p['perturbation_kind']=='position':
            assert np.allclose(rr.as_matrix(),r.as_matrix())
            assert np.allclose(np.linalg.norm(positions-base,axis=1),p['perturbation_magnitude'])
        else:
            assert np.allclose(positions,base)
            assert np.rad2deg((rr*r.inv()).magnitude())==pytest.approx(p['perturbation_magnitude'])
    assert parent==coarse_candidates()[0]


def test_rear_hook_rejects_open_side_and_out_of_plane():
    for x,expected in [(-RING_RADIUS+.0025,True),(RING_RADIUS,False),(-RING_RADIUS-.0025,False)]:
        gap,seated=rear_capsule_geometry([(np.array([x,-.01,0]),np.array([x,.01,0]),.0015)],np.zeros(3),Rotation.identity())
        assert seated is expected
    _,seated=rear_capsule_geometry([(np.array([-RING_RADIUS,.02,0]),np.array([-RING_RADIUS,.04,0]),.0015)],np.zeros(3),Rotation.identity())
    assert not seated


@pytest.mark.parametrize('bad',[
    dict(first_contact_non_target=True),dict(max_non_target_force_N=3.1),
    dict(target_max_displacement_m=.021),dict(main_stem_max_displacement_m=.031),
    dict(target_broken=True),dict(other_broken=True),dict(planning_failure=True)])
def test_retained_contact_cannot_override_failure(bad):
    assert not classify(dict(retained_hook=True,**bad))['hook_success']


def test_multiple_events_preserved_and_no_success_range_invented():
    result=classify(dict(first_contact_non_target=True,target_max_displacement_m=.025))
    assert result['result']=='excessive_displacement'
    assert set(result['events'])=={'miss','non_target_contact','excessive_displacement'}
    row=dict(result,parameters=coarse_candidates()[0])
    summary=summarize([row])
    assert summary['success_count']==0 and summary['successful_pose_ranges'] is None
    assert summary['tolerance_status']=='not_run_no_successful_coarse_candidate'


def test_tolerance_rate_includes_planning_rejections_in_denominator():
    rows=[]
    for i,p in enumerate(tolerance_candidates(coarse_candidates()[0])[:6]):
        label='success_target_hook' if i<4 else 'miss' if i==4 else 'ik_or_planning_failure'
        rows.append(dict(candidate_id=p['candidate_id'],parameters=p,result=label,events=[label],hook_success=i<4,
            target_max_displacement_m=.001,main_stem_max_displacement_m=.001,candidate_position_xyz=[0.,0.,0.]))
    summary=summarize(rows);group=summary['tolerance_test_results'][0]
    assert group['completed'] and group['attempted']==6 and group['successes']==4
    assert group['success_rate']==pytest.approx(4/6)
    assert summary['valid_candidates']==5
