import unittest
import numpy as np
from assemble_plant_scene import project,along,transport,surface_attachment,tipward_rotation,require_tipward

class AttachmentGeometry(unittest.TestCase):
    def test_project_and_along(self):
        points=np.array([[0.,0,0],[0,0,1],[1,0,2]])
        s,p,t=project(points,np.array([.5,.2,1.5]))
        q,u=along(points,s)
        np.testing.assert_allclose(q,p);np.testing.assert_allclose(t,u)
    def test_transport_preserves_shape(self):
        a=np.array([0.,0,1]);b=np.array([1.,0,1])/np.sqrt(2);r=transport(a,b)
        np.testing.assert_allclose(r@a,b,atol=1e-12)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r),1)
    def test_bounds(self):
        for s in [-.01,1.01]:
            with self.assertRaises(ValueError):along(np.array([[0.,0,0],[0,0,1]]),s)
    def test_surface_attachment_touches_without_overlap(self):
        for direction in ([1,0,2],[1,0,-2],[0,-1,0]):
            root,surface,normal=surface_attachment(np.zeros(3),[0,0,1],.006,.002,direction)
            self.assertAlmostEqual(np.linalg.norm(root[:2])-.006-.002,0)
            self.assertAlmostEqual(np.linalg.norm(surface[:2]),.006)
            self.assertGreater(np.dot(normal,direction),0)
            for t in np.linspace(0,.02,20):
                point=root+t*np.asarray(direction)
                self.assertGreaterEqual(np.linalg.norm(point[:2])-.006-.002,-1e-12)
    def test_parallel_attachment_rejected(self):
        with self.assertRaises(ValueError):surface_attachment(np.zeros(3),[0,0,1],.006,.002,[0,0,1])
    def test_rootward_rotated_to_tipward_without_reflection(self):
        a=np.array([1.,0,-2]);t=np.array([0.,0,1.]);r=tipward_rotation(a,t)
        self.assertGreater(require_tipward(r@a,t),0)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r),1)
        with self.assertRaises(ValueError):require_tipward(a,t)
    def test_tipward_and_horizontal_preserved(self):
        for a in ([1.,0,2],[1.,0,0]):
            np.testing.assert_allclose(tipward_rotation(a,[0,0,1]),np.eye(3),atol=1e-12)
            self.assertGreaterEqual(require_tipward(a,[0,0,1]),0)
    def test_identity(self):
        np.testing.assert_array_equal(transport(np.array([0.,0,1]),np.array([0.,0,1])),np.eye(3))
if __name__=='__main__':unittest.main()
