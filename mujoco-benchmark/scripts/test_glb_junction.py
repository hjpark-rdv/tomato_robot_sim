import numpy as np
from build_glb_physics import junction_neighbors

def test_root_straddling_segment_boundary():
 points=np.array([[0.,0,0],[1,0,0],[2,0,0],[3,0,0]])
 assert junction_neighbors(points,np.full(4,.1),np.array([.95,0,0]),.05,0)==[1]

def test_no_distant_or_nonadjacent_suppression():
 points=np.array([[0.,0,0],[1,0,0],[2,0,0],[3,0,0]])
 assert junction_neighbors(points,np.full(4,.1),np.array([.5,0,0]),.05,0)==[]
 assert junction_neighbors(points,np.full(4,.1),np.array([2.9,0,0]),.05,0)==[]
