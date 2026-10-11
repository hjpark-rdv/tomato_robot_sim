"""Contact activation regression for the source PhysX rest/contact offsets.

The source plant capsules author contactOffset=.0005m and restOffset=0m.
MuJoCo >=3.9 changed margin/gap semantics: margin sets force activation and
gap extends detection. The official migration is margin_new=margin_old-gap_old,
gap_new=gap_old; see https://mujoco.readthedocs.io/en/stable/changelog.html#version-3-9-0-may-27-2026 .
These tests separate measured surface intersection from buffered detection.
"""
import unittest

import mujoco as mj
import numpy as np


def sphere_contact(distance_m, margin_m, gap_m):
    """Two small source-mass spheres with prescribed initial surface distance."""
    model = mj.MjModel.from_xml_string(f'''<mujoco>
      <option gravity="0 0 0" integrator="discrete"/>
      <default><geom margin="{margin_m}" gap="{gap_m}" solref=".002 1"/></default>
      <worldbody>
        <geom name="fixed" type="sphere" size=".01"/>
        <body pos="{.02+distance_m} 0 0">
          <freejoint/>
          <inertial pos="0 0 0" mass=".005" diaginertia=".0000002 .0000002 .0000002"/>
          <geom name="moving" type="sphere" size=".01"/>
        </body>
      </worldbody>
    </mujoco>''')
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    result = []
    for index, contact in enumerate(data.contact):
        force = np.zeros(6)
        mj.mj_contactForce(model, data, index, force)
        result.append(dict(distance_m=float(contact.dist),
                           activation_margin_m=float(contact.includemargin),
                           constraint_address=int(contact.efc_address),
                           force_norm_n=float(np.linalg.norm(force[:3]))))
    return result


class ContactSemanticsTests(unittest.TestCase):
    def test_legacy_attributes_generate_force_before_surface_contact(self):
        rows = sphere_contact(.0002, .0005, .0005)
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]['distance_m'], .0002, places=12)
        self.assertAlmostEqual(rows[0]['activation_margin_m'], .001, places=12)
        self.assertGreaterEqual(rows[0]['constraint_address'], 0)
        self.assertGreater(rows[0]['force_norm_n'], 0.)

    def test_source_rest_offset_mapping_keeps_positive_clearance_inactive(self):
        for distance in (.0002, .0008):
            with self.subTest(distance=distance):
                rows = sphere_contact(distance, 0., .0005)
                self.assertEqual(len(rows), 1)
                self.assertAlmostEqual(rows[0]['distance_m'], distance, places=12)
                self.assertEqual(rows[0]['constraint_address'], -1)
                self.assertEqual(rows[0]['force_norm_n'], 0.)
        self.assertEqual(sphere_contact(.0012, 0., .0005), [])

    def test_actual_surface_penetration_still_generates_contact_force(self):
        rows = sphere_contact(-.0002, 0., .0005)
        self.assertEqual(len(rows), 1)
        self.assertLess(rows[0]['distance_m'], 0.)
        self.assertEqual(rows[0]['activation_margin_m'], 0.)
        self.assertGreaterEqual(rows[0]['constraint_address'], 0)
        self.assertGreater(rows[0]['force_norm_n'], 0.)


if __name__ == '__main__':
    unittest.main()
