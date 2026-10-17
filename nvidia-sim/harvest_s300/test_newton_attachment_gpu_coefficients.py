"""Actual native GPU K/B controls; run under the shared GPU benchmark lock."""
import os
from pathlib import Path
import unittest
import numpy as np
import warp as wp
from mujoco_warp._src.constraint import _contact_kbimp
from mujoco_warp._src.types import vec5
from newton_attachment_variant import direct_reference_solref


@wp.kernel(enable_backward=False)
def native_coefficient_probe(dt:wp.array[float],refs:wp.array[wp.vec2],
                             imps:wp.array[vec5],discrete:bool,
                             result:wp.array2d[float]):
    i=wp.tid()
    kbi=_contact_kbimp(0,dt[i],refs[i],imps[i],.0001,discrete)
    for j in range(3):result[i,j]=kbi[j]


@unittest.skipUnless(os.getenv('S300_ATTACHMENT_CUDA_TEST')=='1','Actual CUDA opt-in required')
class NativeAttachmentCoefficientTests(unittest.TestCase):
    def probe(self,refs,frequencies,discrete=True,imps=None):
        with wp.ScopedDevice('cuda:0'):
            n=len(refs)
            out=wp.zeros((n,3),dtype=float)
            wp.launch(native_coefficient_probe,n,inputs=[
                wp.array(1/np.array(frequencies,dtype=np.float32),dtype=float),
                wp.array(np.asarray(refs,dtype=np.float32),dtype=wp.vec2),
                wp.array(np.tile([.999,.9999,.001,.5,2.],(n,1)) if imps is None else imps,dtype=vec5),
                discrete,out])
            return out.numpy()

    def test_discrete_native_coefficients_no_connect_timeconstant_clamp(self):
        positive=self.probe([[.002,1.]]*3,[60,120,400])
        for row in positive[1:]:np.testing.assert_array_equal(row,positive[0])
        self.assertGreater(positive[0,0],249000.)
        self.assertGreater(positive[0,1],999.)
        direct=direct_reference_solref([[.002,1.]],discrete=True)[0]
        negative=self.probe([direct]*3,[60,120,400])
        np.testing.assert_allclose(negative,positive,rtol=2e-7,atol=1e-5)
        doubled=self.probe([[-250000.,-2000.]]*3,[60,120,400])
        np.testing.assert_allclose(doubled[:,0],positive[:,0],rtol=2e-7)
        np.testing.assert_allclose(doubled[:,1],2*positive[:,1],rtol=2e-7)

    def test_continuous_control_does_clamp_and_old_discrete_assumption_is_wrong(self):
        continuous=self.probe([[.002,1.]]*3,[60,120,400],False)
        self.assertLess(continuous[0,0],1000.)
        self.assertLess(continuous[-1,0],41000.)
        wrong_direct=self.probe([[-40000.,-400.]]*3,[60,120,400])
        actual_discrete=self.probe([[.002,1.]]*3,[60,120,400])
        self.assertGreater(float(np.min(actual_discrete[:,0]/wrong_direct[:,0])),6.)

    def test_actual_server_mjb_equality_coefficients(self):
        import mujoco as mj
        source=Path(os.getenv('S300_ATTACHMENT_SOURCE_MJB',
            '/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated/input_model.mjb'))
        if not source.is_file():self.skipTest('Server asset not present')
        model=mj.MjModel.from_binary_path(str(source))
        self.assertEqual(model.opt.integrator,mj.mjtIntegrator.mjINT_DISCRETE)
        self.assertTrue(np.all(model.eq_type==mj.mjtEq.mjEQ_CONNECT))
        rates=np.repeat([60,120,400],model.neq)
        refs=np.tile(model.eq_solref,(3,1));imps=np.tile(model.eq_solimp,(3,1))
        positive=self.probe(refs,rates,imps=imps)
        direct=direct_reference_solref(refs,discrete=True)
        negative=self.probe(direct,rates,imps=imps)
        np.testing.assert_allclose(negative,positive,rtol=2e-7,atol=1e-5)
        self.assertGreater(float(positive[:,0].min()),249000.)
        from newton_inputs import sha256
        print(dict(actual_source_mjb=str(source),mjb_sha256=sha256(source),
                   integrator='DISCRETE',positive_native_kbi=positive[0].tolist(),
                   direct_native_kbi=negative[0].tolist(),
                   maximum_relative_kb_difference=float(np.max(np.abs(
                       (negative[:,:2]-positive[:,:2])/positive[:,:2])))))


if __name__=='__main__':unittest.main()
