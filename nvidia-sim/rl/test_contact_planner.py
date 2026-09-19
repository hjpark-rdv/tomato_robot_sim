"""Geometry and configuration regressions; no simulator or ROS process needed."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_planner import motion_waypoints,pedicel_gap,validate_parameters
from geometry import RING_RADIUS,WIRE_RADIUS


def test_aligned_pull_keeps_pedicel_in_ring_plane():
    center=np.array([-.7,1.1,.9])
    axis=np.array([.5,.1,.86]);axis/=np.linalg.norm(axis)
    neck=center+axis*.016
    rotation,poses=motion_waypoints(center,neck,dict(align_stem=True,yaw_deg=25))
    normal=rotation.as_matrix()[:,1]
    assert np.dot(normal,axis)==pytest.approx(-1.)
    for _,p in poses[-2:]:
        assert np.dot(p-neck,normal)==pytest.approx(0.,abs=1e-12)
    assert poses[1][1][2]<center[2]


def test_gui_pose_has_downward_normal():
    rotation,poses=motion_waypoints(np.zeros(3),np.array([0.,0.,.015]),dict(align_stem=False))
    assert np.allclose(rotation.apply([0,1,0]),[0,0,-1])
    assert [p[0] for p in poses]==['preapproach','below','insert','rise','pull']


def test_gap_detects_rear_arc_contact_but_preserves_open_side():
    radius=.01
    # Pedicel is perpendicular to the ring (along Y), crossing its plane.
    center=np.array([-RING_RADIUS,-.02,0.])
    attachment=center+np.array([0,.04,0])
    assert pedicel_gap(np.zeros(3),Rotation.identity(),center,attachment,radius)==pytest.approx(-WIRE_RADIUS-.0015)
    center[0]=RING_RADIUS;attachment[0]=RING_RADIUS
    assert pedicel_gap(np.zeros(3),Rotation.identity(),center,attachment,radius)>.025


@pytest.mark.parametrize('params',[{'speed_m_s':float('nan')},{'yaw_deg':float('inf')},{'contact_speed_m_s':0.},
                                  {'align_stem':'false'},{'unknown_option':1}])
def test_invalid_motion_fails_before_simulation(params):
    with pytest.raises(ValueError):
        validate_parameters(params)


def test_coincident_fruit_and_neck_rejected():
    with pytest.raises(ValueError):
        motion_waypoints(np.zeros(3),np.zeros(3),{})
