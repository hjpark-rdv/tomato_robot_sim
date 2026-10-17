import unittest
import mujoco as mj
import numpy as np

from newton_plant_variant import selected_bodies, joint_mapping, fit_compliance, compliance, sentinel_bodies


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

    def test_each_prefixed_truss_has_its_own_coarsening_chain_and_sentinels(self):
        names = []
        for prefix, truss in (("", "Truss_00"), ("P01_", "Truss_02")):
            names.extend(prefix + "Stem_MainStem_" + str(i) for i in range(4))
            names.extend(prefix + truss + "_Truss_01_Peduncle_" + str(i) for i in range(3))
            names.extend(prefix + truss + "_Truss_01_Rachis_" + str(i) for i in range(3))
            names.extend(prefix + truss + "_Truss_01_Pedicel_01_" + str(i) for i in range(3))
            names.append(prefix + "Leaf_test")
        text = '<mujoco><worldbody>' + ''.join(
            f'<body name="{name}"><joint name="j{i}"/><geom type="sphere" size=".01" '
            'contype="0" conaffinity="0"/></body>' for i, name in enumerate(names))
        full = mj.MjModel.from_xml_string(text + '</worldbody></mujoco>')
        selected = selected_bodies(full, 'coarse2')
        for prefix, truss in (("", "Truss_00"), ("P01_", "Truss_02")):
            self.assertIn(prefix + "Stem_MainStem_1", selected)
            self.assertIn(prefix + "Stem_MainStem_3", selected)
            self.assertNotIn(prefix + "Stem_MainStem_2", selected)
            for chain in ("Peduncle", "Rachis", "Pedicel_01"):
                self.assertIn(prefix + truss + "_Truss_01_" + chain + "_1", selected)
                self.assertNotIn(prefix + truss + "_Truss_01_" + chain + "_2", selected)
            self.assertIn(prefix + "Leaf_test", selected)
        meta = dict(attachments=[dict(anchor="Truss_00_Truss_01_Pedicel_01_2"),
            dict(anchor="P01_Truss_02_Truss_01_Pedicel_01_2")])
        self.assertEqual(sentinel_bodies(full, meta), ["Stem_MainStem_3", "P01_Stem_MainStem_3",
            "Truss_00_Truss_01_Peduncle_2", "P01_Truss_02_Truss_01_Peduncle_2",
            "Truss_00_Truss_01_Pedicel_01_2", "P01_Truss_02_Truss_01_Pedicel_01_2"])

    def test_single_window_coarse_selection_matches_original_rule_exactly(self):
        import re
        names = ["Stem_MainStem_" + str(i) for i in range(16)]
        names.extend("Truss_00_Truss_01_Peduncle_" + str(i) for i in range(5))
        names.extend("Truss_00_Truss_01_Rachis_" + str(i) for i in range(10))
        for fruit in range(1, 12):
            names.extend(f"Truss_00_Truss_01_Pedicel_{fruit:02d}_{i}" for i in range(3))
        names.extend("Leaf_" + str(i) for i in range(27))
        xml = '<mujoco><worldbody>' + ''.join(f'<body name="{name}"><joint name="j{i}"/>'
            '<geom type="sphere" size=".01" contype="0" conaffinity="0"/></body>'
            for i, name in enumerate(names)) + '</worldbody></mujoco>'
        full = mj.MjModel.from_xml_string(xml)
        chains = {}
        for name in names:
            if name.startswith("Leaf_"):
                continue
            key = ("stem" if name.startswith("Stem_") else
                "pedicel_" + re.search(r'_(\d+)_\d+$', name).group(1) if "Pedicel_" in name else
                "peduncle" if "Peduncle_" in name else "rachis")
            chains.setdefault(key, []).append(name)
        for stride in (2, 4):
            expected = {name for name in names if name.startswith("Leaf_")}
            expected.update(name for chain in chains.values()
                            for index, name in enumerate(chain) if index % stride)
            self.assertEqual(selected_bodies(full, "coarse" + str(stride)), expected)
        meta = dict(attachments=[dict(anchor="Truss_00_Truss_01_Pedicel_01_2")])
        self.assertEqual(sentinel_bodies(full, meta), ["Stem_MainStem_15",
            "Truss_00_Truss_01_Peduncle_4", "Truss_00_Truss_01_Pedicel_01_2"])

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
