from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from access_route import (Budget,SearchBudget,checked_edge,checked_path,timed_polyline,ik_options)
from prepare_access_ablation import configs


def test_intermediate_obstacle_is_checked():
    visited=[]
    def valid(q):visited.append(q.copy());return not (.45<=q[0]<=.55)
    assert not checked_edge([0.],[1.],[.01],valid)
    assert len(visited)>2


def test_invalid_start_not_skipped():
    assert not checked_edge([0.],[1.],[.1],lambda q:q[0]>0)


def test_polyline_retiming_preserves_detour_and_endpoint():
    path=np.array([[0.,0.],[0.,1.],[1.,1.],[1.,0.]])
    commands=timed_polyline(path,[.1,.2])
    assert np.array_equal(commands[0],path[0]) and np.array_equal(commands[-1],path[-1])
    assert checked_path(commands,[.01,.01],lambda q:not(.1<q[0]<.9 and q[1]<.8))
    assert (np.abs(np.diff(commands,axis=0))*60<=np.array([.1,.2])+1e-8).all()
    for v in path:assert np.min(np.linalg.norm(commands-v,axis=1))<1e-12


@pytest.mark.parametrize('bad',[0.,-1.,float('nan'),float('inf')])
def test_timing_rejects_invalid_speed(bad):
    with pytest.raises(ValueError):timed_polyline([[0.],[1.]],[bad])


def test_state_budget_is_hard():
    b=Budget(1.,2);b.tick();b.tick()
    with pytest.raises(SearchBudget):b.tick()


def test_alternate_ik_branch_is_retained_after_first_collision():
    class Kin:
        bounds=np.array([[-1.,-1.],[1.,1.]])
        def ik(self,p,r,q):return np.array([p[0], -.5 if q[1]<0 else .5]),999.,999.
        def fk(self,q):return np.array([q[0],0,0]),Rotation.identity()
    qs,records=ik_options(Kin(),[.3,0,0],Rotation.identity(),[0.,-.5],
        lambda q:q[1]>.1,[.01,.01],seeds=4,max_solutions=2)
    assert len(qs)==1 and qs[0][1]>.1
    assert records[0]['status']=='goal_collision'
    assert any(r['status']=='collision_valid_goal' for r in records)


def test_fk_is_authoritative_not_ik_reported_zero_error():
    class Kin:
        bounds=np.array([[-1.],[1.]])
        def ik(self,p,r,q):return np.array([.5]),0.,0.
        def fk(self,q):return np.array([.5,0,0]),Rotation.identity()
    qs,_=ik_options(Kin(),[0.,0.,0.],Rotation.identity(),[0.],lambda q:True,[.01],seeds=1,max_solutions=1)
    assert not qs


def test_low_tilt_baseline_not_accidentally_large_sobol_tilt():
    rows=configs()
    assert [p['azimuth_deg'] for p in rows]==[-45.,0.,45.]
    for r in rows:assert r['roll_deg']==r['pitch_deg']==r['elevation_deg']==r['entry_twist_deg']==0


def test_one_factor_ablation_changes_exactly_one_key():
    a=configs();b=configs(tilt_axis='roll',tilt_deg=15.)
    for x,y in zip(a,b):assert [k for k in x if x[k]!=y[k]]==['roll_deg']


@pytest.mark.parametrize('kwargs',[{'azimuths':[0.,0.]},{'tilt_deg':10.},{'tilt_axis':'pitch','tilt_deg':30.}])
def test_ablation_limits(kwargs):
    with pytest.raises(ValueError):configs(**kwargs)
