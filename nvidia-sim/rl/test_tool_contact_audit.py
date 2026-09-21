import numpy as np
from tool_contact_audit import ToolContactAudit


def model():
    return dict(shapes=[dict(path='/Robot/RingCollision/segment_00',body=0,local_endpoints=[[-.01,0,0],[.01,0,0]],radius_m=.001),
        dict(path='/ElasticPlant/StemCollider',body=1,local_endpoints=[[0,0,-.01],[0,0,.01]],radius_m=.001)])


def test_crossing_capsules_are_rejected_but_separated_capsules_are_not():
    audit=ToolContactAudit(model())
    poses=np.array([[0,0,0,1,0,0,0],[0,.005,0,1,0,0,0]],float)
    assert not audit.sample(poses,0)['invalid_penetration']
    poses[1,1]=.0001
    assert audit.sample(poses,1)['invalid_penetration']
    assert np.isclose(audit.report()['worst']['gap_m'],-.0019)
    assert not audit.report()['passed']


def test_bounded_small_solver_overlap_is_reported_without_being_hidden():
    audit=ToolContactAudit(model())
    poses=np.array([[0,0,0,1,0,0,0],[0,.0019,0,1,0,0,0]],float)
    assert not audit.sample(poses,0)['invalid_penetration']
    assert np.isclose(audit.report()['worst']['gap_m'],-.0001)


def test_sphere_axis_and_rotated_tool_are_supported():
    m=model();m['shapes'][1]['local_endpoints']=[[0,0,0]]*2
    audit=ToolContactAudit(m)
    poses=np.array([[0,0,0,2**-.5,0,0,2**-.5],[0,.005,0,1,0,0,0]],float)
    assert audit.sample(poses,0)['invalid_penetration']


def test_invalid_trial_cannot_become_success_or_partial_training_example():
    from dataset_design import classify_result
    from dataset_videos import select
    metrics=dict(physics_valid=False,retained_hook=True,inserted=True)
    result=classify_result(metrics,{})
    assert result['result']=='invalid_physics'
    assert not result['hook_success']
    assert result['exclude_from_valid_trajectory_analysis']
    assert select([dict(candidate_id='candidate_00049',center_entry_safe=True,**result)])==[]


def test_next_candidate_does_not_inherit_previous_penetration():
    audit=ToolContactAudit(model())
    poses=np.array([[0,0,0,1,0,0,0],[0,0,0,1,0,0,0]],float)
    audit.sample(poses,0)
    assert not audit.report()['passed']
    audit.reset();poses[1,1]=.005
    audit.sample(poses,0)
    assert audit.report()['passed'] and audit.report()['sample_count']==1


def test_nonfinite_native_pose_is_not_reported_as_collision_free():
    import pytest
    audit=ToolContactAudit(model())
    poses=np.array([[0,0,0,1,0,0,0],[float('nan'),0,0,1,0,0,0]],float)
    with pytest.raises(ValueError,match='Nonfinite'):
        audit.sample(poses,0)
