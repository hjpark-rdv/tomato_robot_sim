import unittest
import mujoco as mj
import numpy as np

from newton_plant_variant import selected_bodies, joint_mapping, fit_compliance, compliance


def model(leaf_collision=False, second_joint=True):
    return mj.MjModel.from_xml_string('''<mujoco><compiler inertiafromgeom="false"/><worldbody>
      <body name="Stem_MainStem_0"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
       <body name="Stem_MainStem_1" pos="0 0 .1"><joint name="a" axis="1 0 0" stiffness="10"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <body name="Stem_MainStem_2" pos="0 0 .1">''' +
        ('<joint name="b" axis="1 0 0" stiffness="10"/>' if second_joint else '')+'''
         <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
         <body name="tip" pos="0 0 .2"><inertial pos="0 0 0" mass=".1" diaginertia=".1 .1 .1"/></body>
         <body name="Leaf_test"><joint name="leaf" axis="0 1 0" stiffness=".01"/>
          <geom type="sphere" size=".01" contype="'''+('1' if leaf_collision else '0')+'''" conaffinity="0"/>
          <inertial pos="0 0 0" mass=".01" diaginertia=".01 .01 .01"/>
         </body>
        </body>
       </body>
      </body></worldbody></mujoco>''')


class PlantVariantTests(unittest.TestCase):
    def test_colliding_leaf_cannot_be_silently_locked(self):
        with self.assertRaisesRegex(ValueError, 'noncolliding'):
            selected_bodies(model(True), 'leaves_fixed')

    def test_removal_is_limited_to_selected_whole_bodies(self):
        self.assertEqual(selected_bodies(model(), 'leaves_fixed'), {'Leaf_test'})
        self.assertEqual(selected_bodies(model(), 'coarse2'), {'Leaf_test','Stem_MainStem_2'})

    def test_coordinate_mapping_retains_named_joints(self):
        old,new=model(),model(second_joint=False)
        qp,qv=joint_mapping(old,new)
        np.testing.assert_array_equal(qp,[0,2]);np.testing.assert_array_equal(qv,[0,2])

    def test_positive_compliance_fit_preserves_end_response(self):
        old,new=model(),model(second_joint=False)
        od,nd=mj.MjData(old),mj.MjData(new)
        mj.mj_forward(old,od);mj.mj_forward(new,nd)
        result=fit_compliance(old,od,new,nd,['tip'])
        self.assertTrue(result['success'])
        self.assertLess(result['weighted_relative_error_after'],1e-8)
        self.assertTrue(all(x['compliance_factor']>0 for x in result['factors']))
        self.assertGreater(new.jnt_stiffness[0],0)
        np.testing.assert_allclose(compliance(old,od,['tip'])[0],compliance(new,nd,['tip'])[0],atol=1e-10)


if __name__=='__main__':unittest.main()
