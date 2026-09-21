import numpy as np
from gpu_validation import check_clone


def state():
    body=np.zeros((1,2,13));body[...,3]=1
    return dict(robot_q=np.zeros((1,7)),robot_dq=np.zeros((1,7)),robot_root=body[:,:1].copy(),
        elastic_q=np.zeros((1,6)),elastic_dq=np.zeros((1,6)),elastic_bodies=body.copy(),
        fruits=body.copy(),preload=np.zeros((1,6)),stiffness=np.ones((1,6)),damping=np.ones((1,6)))


def test_quaternion_sign_and_small_gpu_rounding_are_not_pose_divergence():
    ref=state();other=state();other['elastic_bodies'][...,3:7]*=-1
    other['elastic_bodies'][...,0]=8e-6
    other['elastic_bodies'][...,10]=.0125
    other['elastic_dq'][:]=.003
    assert check_clone(ref,other)['passed']


def test_position_velocity_break_and_nonfinite_fail_closed():
    for key,index,value in [('fruits',(...,0),.0002),('fruits',(...,10),.03),('elastic_dq',(...,),.1),('preload',(...,),.1)]:
        other=state();other[key][index]=value
        assert not check_clone(state(),other)['passed']
    assert not check_clone(state(),state(),broken=True)['passed']
    other=state();other['fruits'][...,0]=np.nan
    assert not check_clone(state(),other)['passed']


def test_zero_quaternion_is_invalid_even_when_all_numbers_are_finite():
    other=state();other['fruits'][...,3:7]=0
    assert not check_clone(state(),other)['passed']
