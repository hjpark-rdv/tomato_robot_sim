import numpy as np
from build_glb_physics import junction_neighbors

def test_root_straddling_segment_boundary():
 points=np.array([[0.,0,0],[1,0,0],[2,0,0],[3,0,0]])
 assert junction_neighbors(points,np.full(4,.1),np.array([.95,0,0]),.05,0)==[1]

def test_no_distant_or_nonadjacent_suppression():
 points=np.array([[0.,0,0],[1,0,0],[2,0,0],[3,0,0]])
 assert junction_neighbors(points,np.full(4,.1),np.array([.5,0,0]),.05,0)==[]
 assert junction_neighbors(points,np.full(4,.1),np.array([2.9,0,0]),.05,0)==[]


def test_hook_main_stem_penetration_invalidates_glb_rollout():
 from robot_engine import valid_glb_physics
 # Regression: scene_0001/Tomato_05/candidate_00005.
 assert not valid_glb_physics(.0003470519,.0038601738,False)
 assert valid_glb_physics(.0003470519,.0004,False)
 assert not valid_glb_physics(.0006,0.,False)
 assert not valid_glb_physics(0.,0.,True)


def test_gutter_arm_penetration_invalidates_rollout():
 from robot_engine import valid_glb_physics
 assert not valid_glb_physics(0.,0.,False,.0006)
 assert valid_glb_physics(0.,0.,False,.0004)


def test_background_stem_arm_contact_invalidates_rollout():
 from robot_engine import valid_glb_physics
 assert not valid_glb_physics(0.,0.,False,0.,.0006)
 assert valid_glb_physics(0.,0.,False,0.,.0004)
