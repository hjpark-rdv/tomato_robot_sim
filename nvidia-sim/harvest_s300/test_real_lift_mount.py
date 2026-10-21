"""Asset integration checks. Set S300_LIFT_SOURCE and S300_LIFT_CORRECTED."""
import json
import os
from pathlib import Path
import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from real_lift_mount import ROBOT, PLATFORM, LIFT_MESH, apply, matrix, components
from motion_kinematics import ToolKinematics


@unittest.skipUnless(os.environ.get('S300_LIFT_SOURCE') and os.environ.get('S300_LIFT_CORRECTED'), 'Requires original/corrected lift assets')
class LiftMountIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=Usd.Stage.Open(os.environ['S300_LIFT_SOURCE']+'/composed.usdc')
        cls.corrected=Usd.Stage.Open(os.environ['S300_LIFT_CORRECTED']+'/composed.usdc')
        cls.report=json.loads((Path(os.environ['S300_LIFT_CORRECTED'])/'real_lift_mount.json').read_text())

    def test_original_cad_surfaces_stay_at_zero_pose(self):
        _,v,f,cc=components(self.source)
        for row in self.report['components']:
            p=self.corrected.GetPrimAtPath(row['path']);m=UsdGeom.Mesh(p);t=matrix(self.corrected,row['path'])
            verts=np.asarray(m.GetPointsAttr().Get())@t[:3,:3].T+t[:3,3]
            faces=np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
            np.testing.assert_allclose(verts[faces],v[f[cc[row['component']]]],atol=5e-8,rtol=0)
        self.assertEqual(self.report['source_triangles'],self.report['output_triangles'])

    def test_virtual_platform_is_gone(self):
        for p in (LIFT_MESH,PLATFORM+'/visuals/mesh_0/box',PLATFORM+'/collisions/mesh_0/box'):
            prim=self.corrected.GetPrimAtPath(p)
            self.assertFalse(prim and prim.IsActive())
        self.assertTrue(self.corrected.GetPrimAtPath(PLATFORM+'/collisions/real_mounting_plate').HasAPI(UsdPhysics.CollisionAPI))

    def test_existing_lift_drive_and_mass_are_preserved(self):
        p=ROBOT+'/joints/farmily_lift_height_joint'
        for attr in ('physics:lowerLimit','physics:upperLimit','physics:axis','drive:linear:physics:stiffness','drive:linear:physics:damping','drive:linear:physics:maxForce'):
            self.assertEqual(self.source.GetPrimAtPath(p).GetAttribute(attr).Get(),self.corrected.GetPrimAtPath(p).GetAttribute(attr).Get())
        for path in (PLATFORM,ROBOT+'/world'):
            for attr in ('physics:mass','physics:diagonalInertia','physics:centerOfMass'):
                self.assertEqual(self.source.GetPrimAtPath(path).GetAttribute(attr).Get(),self.corrected.GetPrimAtPath(path).GetAttribute(attr).Get())

    def test_full_stroke_fk_changes_only_height(self):
        a=ToolKinematics(self.source);b=ToolKinematics(self.corrected)
        for lift in (0,.375,.75):
            q=np.array([lift,1.34,.218,2.49,.428,2.98,3.14]);old=a.fk(q);new=b.fk(q)
            expected=old.copy();expected[2,3]+=self.report['arm_mount_offset_z_m']
            np.testing.assert_allclose(new,expected,atol=1e-8,rtol=0)
        np.testing.assert_allclose(matrix(self.source,ROBOT+'/world'),matrix(self.corrected,ROBOT+'/world'),atol=0,rtol=0)

    def test_rb5_sits_on_plate(self):
        gap=self.report['rb5_base_gap_range_m']
        self.assertEqual(gap[0],0)
        self.assertLess(gap[1],.0002) # Import's original 1.57 rad CAD tilt.
        self.assertLess(abs(self.report['arm_mount_offset_z_m']),.01)

    def test_double_correction_rejected(self):
        with self.assertRaisesRegex(ValueError,'already present'):
            apply(self.corrected)


if __name__=='__main__':unittest.main()
