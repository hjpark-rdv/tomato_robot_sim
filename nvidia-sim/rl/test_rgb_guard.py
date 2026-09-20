import cv2
import numpy as np

from rgb_camera import optical_look_at, project
from rgb_guard import RGBMotionGuard


K = np.array([[500.,0,320],[0,500.,240],[0,0,1.]])


def scene(camera_x, object_x=0):
    """Textured red plane, rendered by homography from known test geometry."""
    rng = np.random.default_rng(4)
    texture = np.zeros((100,100,3), np.uint8)
    texture[:] = [190,35,25]
    for x,y in rng.integers(3,94,(70,2)):
        cv2.rectangle(texture,(int(x),int(y)),(int(x+3),int(y+3)),(int(rng.integers(90,250)),25,20),-1)
    pose = np.eye(4); pose[0,3] = camera_x
    corners = np.array([[-.025,-.025,.25],[.025,-.025,.25],[.025,.025,.25],[-.025,.025,.25]])
    corners[:,0] += object_x
    uv,_=project(corners,pose,K)
    matrix=cv2.getPerspectiveTransform(np.float32([[0,0],[99,0],[99,99],[0,99]]),uv.astype(np.float32))
    image=cv2.warpPerspective(texture,matrix,(640,480),borderValue=(210,210,210))
    return image,pose


def initialized():
    guard=RGBMotionGuard(K)
    image,pose=scene(0)
    guard.initialize(image,[265,185,110,110],pose)
    assert len(guard.points)>=8
    return guard


def test_camera_motion_alone_does_not_trigger():
    guard=initialized()
    for i in range(1,25):
        image,pose=scene(i*.0015)
        result=guard.observe(image,pose)
        assert not result['stop'],result
    assert guard.model is not None
    assert result['residual_px']<1.


def test_moving_target_triggers_after_calibration():
    guard=initialized()
    for i in range(1,24):
        image,pose=scene(i*.0015)
        assert not guard.observe(image,pose)['stop']
    for i in range(24,36):
        image,pose=scene(i*.0015,(i-23)*.001)
        result=guard.observe(image,pose)
        if result['stop']:
            break
    assert result['stop'] and result['reason']=='motion_or_model_mismatch'
    assert result['persistence']>=3


def test_occlusion_is_uncertainty_not_hook_success():
    guard=initialized()
    for i in range(1,24):
        image,pose=scene(i*.0015)
        guard.observe(image,pose)
    result=guard.observe(np.full_like(image,210),pose)
    assert result['stop'] and result['reason']=='tracking_lost'


def test_no_parallax_rejects_unobservable_model():
    guard=initialized()
    image,pose=scene(0)
    for _ in range(60):
        result=guard.observe(image,pose)
    assert result['stop'] and result['reason']=='unobservable_motion_model'


def test_blank_roi_rejects_tracking():
    guard=RGBMotionGuard(K)
    image=np.full((480,640,3),210,np.uint8)
    guard.initialize(image,[200,200,50,50],np.eye(4))
    assert guard.observe(image,np.eye(4))['reason']=='insufficient_visible_features'


def test_optical_frame_projects_look_at_to_center():
    eye=np.array([.1,-.065,.11]); target=np.array([-.05,0,0])
    pose=optical_look_at(eye,target)
    uv,z=project(target,pose,K)
    assert np.allclose(uv[0],[320,240]) and z[0]>0
    assert np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3))
    assert np.linalg.det(pose[:3,:3])>0


def test_visual_diagnostics_reproduce_online_decision():
    guard=initialized()
    window=[]
    measured=0
    for i in range(1,36):
        image,pose=scene(i*.0015,max(0,i-23)*.001)
        result=guard.observe(image,pose)
        if 'predicted_pixels' in guard.visual:
            actual=np.asarray(guard.visual['observed_pixels'])
            predicted=np.asarray(guard.visual['predicted_pixels'])
            error=actual-predicted
            assert np.allclose(error,guard.visual['innovation_vectors_px'])
            median=np.median(error,axis=0)
            assert np.allclose(median,result['flow_innovation_px'])
            window=(window+[median])[-6:]
            assert np.isclose(np.linalg.norm(np.sum(window,axis=0)),result['residual_px'])
            measured+=1
        if result['stop']:
            break
    assert measured>3 and result['stop']
