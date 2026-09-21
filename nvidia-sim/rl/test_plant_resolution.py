import numpy as np
import pytest
from plant_resolution import joint_layout, appendage_policy


def nodes(count=16):
    return np.column_stack([np.linspace(0,1,count+1),np.zeros((count+1,2))])


def test_full_preserves_every_hinge_and_spring():
    result=joint_layout('full','STEM_MainStem',nodes(),[10,0,0],'Tomato_05',True)
    assert not result['free'][0]
    assert result['free'][1:].all()
    assert np.array_equal(result['spring_scale'],np.ones(16))


def test_light_preserves_target_pedicel_even_if_center_is_far():
    result=joint_layout('light','TRUSS_Pedicel_proximal_05',nodes(3),[10,0,0],'Tomato_05')
    assert result['free'].all() and result['protected'].all()
    assert np.array_equal(result['spring_scale'],np.ones(3))


@pytest.mark.parametrize('world_root',[False,True])
def test_remote_hinge_reduction_preserves_serial_small_angle_compliance(world_root):
    result=joint_layout('light','STEM_MainStem',nodes(),[10,0,0],'Tomato_05',world_root)
    assert 0<result['free'].sum()<16-int(world_root)
    assert np.isclose(np.sum(1/result['spring_scale'][result['free']]),16-int(world_root))
    if world_root:assert not result['free'][0]


def test_near_target_segment_keeps_original_spring_and_collision_nodes_unchanged():
    original=nodes();copy=original.copy()
    result=joint_layout('light','TRUSS_Rachis',original,[.5,0,0],'Tomato_05')
    assert result['protected'].any()
    assert result['free'][result['protected']].all()
    assert (result['spring_scale'][result['protected']]==1).all()
    assert np.array_equal(original,copy)
    assert np.isclose(np.sum(1/result['spring_scale'][result['free']]),16)


def test_unknown_resolution_is_rejected():
    with pytest.raises(ValueError):joint_layout('typo','stem',nodes(),[0,0,0],'Tomato_05')


def test_appendage_policy_keeps_original_and_allows_old_light_comparison():
    assert appendage_policy('full')=='keep'
    assert appendage_policy('light')=='ignore'
    assert appendage_policy('light','keep')=='keep'
    with pytest.raises(ValueError):appendage_policy('full','ignore')
    with pytest.raises(ValueError):appendage_policy('typo')
