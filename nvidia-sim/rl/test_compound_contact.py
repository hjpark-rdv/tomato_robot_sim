"""A non-target hook on a secondary compound capsule must not be ignored."""
from types import SimpleNamespace
import numpy as np
import torch
import dataset_motion


def test_secondary_capsule_uses_its_actual_geometry(monkeypatch):
    path='/World/envs/env_0/ElasticPlant/TRUSS_Pedicel_proximal_07_00/StemCollider_01'
    a=np.array([0.,.01,0.]);b=np.array([0.,.02,0.])
    plant=SimpleNamespace(collider_shapes={path:(0,a,b,.001)},
        poses=lambda:np.array([[1.,2.,3.,1.,0.,0.,0.]]))
    env=SimpleNamespace(elastic=plant,ring_pose=lambda:(torch.zeros((1,3)),torch.tensor([[1.,0.,0.,0.]])))
    seen=[]
    def geometry(caps,*args):
        seen.extend(caps)
        return 0.,True
    monkeypatch.setattr(dataset_motion,'rear_capsule_geometry',geometry)
    assert dataset_motion.non_target_seated(env,path)
    assert np.allclose(seen[0][0],[1.,2.01,3.])
    assert np.allclose(seen[0][1],[1.,2.02,3.])
    assert seen[0][2]==.001
