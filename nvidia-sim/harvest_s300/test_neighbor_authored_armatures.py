import unittest
from pxr import Usd, UsdPhysics, Sdf
import numpy as np
from neighbor_plant_runtime import authored_armatures


class AuthoredArmatureTests(unittest.TestCase):
    def test_thin_leaf_and_unset_native_default_stay_distinct(self):
        stage=Usd.Stage.CreateInMemory()
        UsdPhysics.Joint.Define(stage,'/Plant/Joints/Stem')
        leaf=UsdPhysics.Joint.Define(stage,'/Plant/Joints/Leaf')
        leaf.GetPrim().CreateAttribute('physxJoint:armature',Sdf.ValueTypeNames.Float).Set(.000005)
        before=stage.GetRootLayer().ExportToString()
        values=authored_armatures(stage,'/Plant',['Leaf:0','Stem:1','Leaf:2'])
        np.testing.assert_allclose(values,[.000005,0.,.000005],rtol=1e-7,atol=0.)
        self.assertEqual(before,stage.GetRootLayer().ExportToString())
        with self.assertRaises(ValueError):authored_armatures(stage,'/Plant',['Missing:0'])
