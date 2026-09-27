"""Surface attachment and authored-axis rotation regression."""
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from build_glb_physics import attachment_frame


def test_y_rotation_and_surface_attachment():
    model=SimpleNamespace(geom_size=np.array([[.006,.04,0]]))
    data=SimpleNamespace(geom_xmat=np.eye(3).reshape(1,9),geom_xpos=np.array([[1.,2.,3.]]))
    ped=np.array([[0,0,0],[.01,0,0],[.02,0,0],[.03,.01,0]])
    for angle in (0,20,90,180):
        rot,p=attachment_frame(model,data,0,ped,.3,angle)
        canonical=Rotation.from_euler('x',90,degrees=True).as_matrix()
        np.testing.assert_allclose(Rotation.from_euler('z',angle,degrees=True).as_matrix()@canonical,rot,atol=1e-15)
        center=data.geom_xpos[0]+np.array([0,0,(2*.3-1)*.04])
        np.testing.assert_allclose(np.linalg.norm(p-center),.006,atol=1e-15)
        assert abs(p[2]-center[2])<1e-15
