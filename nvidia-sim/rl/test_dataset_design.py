import numpy as np
import pytest
from dataset_design import candidates,crop_rgbd,classify_result,waypoints,BOUNDS
from pose_candidates import DEFAULT_LIMITS,candidate_waypoints
from dataset_design import throughput_estimate


def test_sobol_reproducible_prefix_and_ranges():
    a,b=candidates(100,42),candidates(1000,42)
    assert a==b[:100]
    assert a!=candidates(100,43)
    assert len({p['candidate_id'] for p in b})==1000
    for p in b[1:]:
        values=dict(p,**{f'offset_{k}_m':v for k,v in zip('xyz',p['offset_xyz_m'])})
        assert all(lo<=values[k]<=hi for k,(lo,hi) in BOUNDS.items())


def test_nominal_waypoints_preserved():
    p=candidates(1)[0];center=np.array([0.,0.,1.]);neck=center+[.005,0,.02]
    a,ap,ad=candidate_waypoints(center,neck,p);b,bp,bd=waypoints(center,neck,p)
    assert np.allclose(a.as_matrix(),b.as_matrix())
    assert np.allclose(ad,bd)
    assert all(x==y and np.allclose(xx,yy) for (x,xx),(y,yy) in zip(ap,bp))


def test_insertion_and_lift_change_actual_trajectory():
    p=candidates(1)[0];c=np.array([0.,0.,1.]);n=c+[0,0,.02]
    _,before,_=waypoints(c,n,p)
    p['insertion_distance_m']+=.005;p['lift_offset_m']+=.004
    _,after,_=waypoints(c,n,p)
    before,after=dict(before),dict(after)
    assert np.allclose(after['insert']-before['insert'],[.005,0,0])
    assert np.allclose(after['rise']-before['rise'],[.005,0,.004])


def test_crop_preserves_projection_and_missing_depth():
    rgb=np.zeros((80,100,3),np.uint8);depth=np.ones((80,100),np.float32)
    depth[38:42,48:52]=np.nan;K=np.array([[100.,0,50],[0,100,40],[0,0,1]])
    local,z,valid,meta=crop_rgbd(rgb,depth,K,np.eye(4),[0,0,1],.2)
    assert local.shape==(20,20,3) and z.dtype==np.float32
    assert np.isnan(z[8:12,8:12]).all() and not valid[38:42,48:52].any()
    pixel=np.asarray(meta['K_local'])@np.array([0,0,1.])
    assert np.allclose(pixel[:2],[10,10])
    assert not meta['truncated']


def test_crop_boundary_and_hidden_target_not_discarded():
    rgb=np.zeros((80,100,3),np.uint8);depth=np.full((80,100),np.nan,np.float32)
    K=np.array([[100.,0,50],[0,100,40],[0,0,1]])
    _,z,_,meta=crop_rgbd(rgb,depth,K,np.eye(4),[-.48,0,1.],.2)
    assert meta['truncated'] and meta['local_depth_valid_fraction']==0 and np.isnan(z).all()
    with pytest.raises(ValueError):crop_rgbd(rgb,depth,K,np.eye(4),[0,0,-1.],.2)


def test_labels_do_not_turn_gentle_fruit_contact_into_failure():
    m=dict(retained_hook=True,inserted=True,first_contact_non_target=True,max_non_target_force_N=.6,
           target_max_displacement_m=.002,main_stem_max_displacement_m=.003)
    assert classify_result(m,DEFAULT_LIMITS)['hook_success']
    for bad in [dict(max_non_target_force_N=3.1),dict(target_broken=True),dict(hooked_non_target=True),
                dict(target_max_displacement_m=.021),dict(inserted=False)]:
        assert not classify_result(dict(m,**bad),DEFAULT_LIMITS)['hook_success']
    assert classify_result(dict(m,abort_reason='debug_step_limit'),DEFAULT_LIMITS)['result']=='incomplete'
    assert classify_result(dict(planning_failure_reason={'reason':'endpoint_ik'}),DEFAULT_LIMITS)['result']=='ik_failure'
    assert classify_result(dict(planning_failure_reason={'reason':'self_collision'}),DEFAULT_LIMITS)['result']=='planning_failure'


def test_throughput_uses_only_new_completed_candidates_on_resume():
    report=throughput_estimate(1000,204,4,30,120)
    assert report['pending_candidates']==796
    assert report['estimated_remaining_minutes']==398
    assert report['estimated_total_minutes']==500.5
    assert throughput_estimate(1000,200,0,30,10)['estimated_remaining_minutes'] is None


def test_truncated_debug_trials_never_predict_full_dataset_duration():
    report=throughput_estimate(1000,8,8,30,5,truncated=True)
    assert report['estimated_remaining_minutes'] is None
    assert report['estimated_total_minutes'] is None


def test_pull_speed_changes_only_pull_duration_not_geometry():
    from types import SimpleNamespace
    import torch
    from scipy.spatial.transform import Rotation
    from dataset_motion import plan

    class Kinematics:
        bounds=np.array([[-5.]*7,[5.]*7])
        def fk(self,q):return q[1:4].copy(),Rotation.identity()
        def ik(self,position,rotation,q):
            q=q.copy();q[1:4]=position;return q,0.,0.

    env=SimpleNamespace(
        _target_geometry=lambda:[torch.tensor([[0.,0.,1.]]),torch.tensor([[0.,0.,1.03]]),torch.tensor([[0.,0.,1.]])],
        robot=SimpleNamespace(data=SimpleNamespace(joint_pos=torch.zeros(1,7))),
        step_dt=1/60,lift_id=0,cfg=SimpleNamespace(dataset_rise_speed=.002,dataset_pull_speed=.002))
    checker=SimpleNamespace(check=lambda q:None)
    params=dict(candidates(1)[0],goal='pull')
    slow,_=plan(env,Kinematics(),checker,params)
    env.cfg.dataset_pull_speed=.004
    fast,_=plan(env,Kinematics(),checker,params)
    assert slow is not None and fast is not None
    for a,b in zip(slow['waypoints'],fast['waypoints']):
        assert a['position_xyz']==b['position_xyz']
        assert a['orientation_xyzw']==b['orientation_xyzw']
        if a['phase']=='pull':assert b['steps']<a['steps']
        else:assert a['steps']==b['steps']
