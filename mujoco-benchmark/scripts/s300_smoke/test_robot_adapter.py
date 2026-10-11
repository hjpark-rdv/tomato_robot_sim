"""Synthetic USD / native MuJoCo integration checks for the S300 adapter.

Run with ``nvidia-sim/.asset-venv/bin/python -m unittest discover -s
mujoco-benchmark/scripts/s300_smoke -p test_robot_adapter.py``. USD export and
native execution use their existing separate interpreters. Set
S300_NATIVE_PYTHON to select another installed native MuJoCo interpreter.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from robot_adapter import ROBOT, TOOL, add_robot


NATIVE_PYTHON = Path(os.environ.get('S300_NATIVE_PYTHON',
    str(Path(__file__).resolve().parents[2]/'.mujoco315-venv/bin/python')))


def rotation(q):
    x, y, z, w = R.from_matrix(q).as_quat()
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def rigid(stage, path, transform):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(transform.T.tolist()))
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(1.)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.01, .01, .01))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1., Gf.Vec3f(0, 0, 0)))
    return prim


def joint(stage, path, parent, child, local, kind='fixed', axis='X'):
    schema = dict(fixed=UsdPhysics.FixedJoint, slide=UsdPhysics.PrismaticJoint,
                  hinge=UsdPhysics.RevoluteJoint)[kind]
    obj = schema.Define(stage, path)
    obj.CreateBody0Rel().SetTargets([parent])
    obj.CreateBody1Rel().SetTargets([child])
    obj.CreateLocalPos0Attr(Gf.Vec3f(*local[:3, 3]))
    obj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    obj.CreateLocalRot0Attr(rotation(local[:3, :3]))
    obj.CreateLocalRot1Attr(Gf.Quatf(1., Gf.Vec3f(0, 0, 0)))
    obj.CreateCollisionEnabledAttr(False)
    if kind != 'fixed':
        obj.CreateAxisAttr(axis)
        obj.CreateLowerLimitAttr(-.026 if kind == 'slide' else -180.)
        obj.CreateUpperLimitAttr(.026 if kind == 'slide' else 180.)
        drive = UsdPhysics.DriveAPI.Apply(obj.GetPrim(), 'linear' if kind == 'slide' else 'angular')
        drive.CreateStiffnessAttr(1500. if kind == 'slide' else 800.)
        drive.CreateDampingAttr(30. if kind == 'slide' else 40.)
        drive.CreateMaxForceAttr(10. if kind == 'slide' else 250.)
        drive.CreateTargetPositionAttr(0.)
        drive.CreateTargetVelocityAttr(0.)
    return obj


def synthetic(overlap=False):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    world_transform = np.eye(4)
    world_transform[:3, 3] = [1.2, -.4, .7]
    rigid(stage, ROBOT+'/world', world_transform)
    arm_local = np.eye(4)
    arm_local[:3, :3] = R.from_euler('z', .23).as_matrix()
    arm_local[:3, 3] = [.2, .1, .3]
    arm_transform = world_transform@arm_local
    rigid(stage, ROBOT+'/link6', arm_transform)
    arm = joint(stage, ROBOT+'/joints/wrist3', ROBOT+'/world',
                ROBOT+'/link6', arm_local, 'hinge', axis='Y')
    arm.GetPrim().CreateAttribute('state:angular:physics:position',
                                Sdf.ValueTypeNames.Float).Set(31.)
    tool_local = np.eye(4)
    tool_local[:3, 3] = [0., .12, .015]
    tool_transform = arm_transform@tool_local
    rigid(stage, TOOL+'/fixed', tool_transform)
    joint(stage, TOOL+'/Joints/jaw_fixed', ROBOT+'/link6', TOOL+'/fixed', tool_local)
    for side, sign in [('L', -1), ('R', 1)]:
        offset = np.eye(4)
        offset[:3, 3] = [0. if overlap else sign*.06, 0., 0.]
        rigid(stage, TOOL+'/'+side, tool_transform@offset)
        joint(stage, TOOL+'/Joints/jaw_'+side, TOOL+'/fixed', TOOL+'/'+side, offset, 'slide')
        rigid(stage, TOOL+'/'+side+'_belt', tool_transform@offset)
        joint(stage, TOOL+'/Joints/belt_'+side, TOOL+'/'+side,
              TOOL+'/'+side+'_belt', np.eye(4))
    # Same-body and source-adjacent contacts are filtered; siblings intentionally
    # overlap in the contact-control fixture and must still produce constraints.
    for path in [ROBOT+'/world', ROBOT+'/link6', TOOL+'/fixed',
                 TOOL+'/L', TOOL+'/R', TOOL+'/L_belt', TOOL+'/R_belt']:
        cube = UsdGeom.Cube.Define(stage, path+'/Collider')
        cube.CreateSizeAttr(.03)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    manifest = dict(initial_robot_q=[0, 0, 0, 0, 0, 0, .42],
                    initial_jaw_q=dict(jaw_L=0., jaw_R=0.),
                    tool=dict(bodies=dict(fixed=TOOL+'/fixed', L=TOOL+'/L', R=TOOL+'/R'),
                              belts=[TOOL+'/L_belt', TOOL+'/R_belt']))
    return stage, manifest, world_transform, arm_local, tool_local


class RobotAdapterIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='s300_robot_adapter_test_')
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def export(self, stage, manifest):
        root = ET.Element('mujoco')
        ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false', meshdir='meshes')
        opt = ET.SubElement(root, 'option', gravity='0 0 0', integrator='implicitfast')
        ET.SubElement(opt, 'flag', filterparent='disable')
        world = ET.SubElement(root, 'worldbody')
        assets = ET.SubElement(root, 'asset')
        contact = ET.SubElement(root, 'contact')
        result = add_robot(stage, manifest, root=root, world=world, assets=assets,
                           contact=contact, meshdir=self.directory/'meshes')
        ET.ElementTree(root).write(self.directory/'scene.xml')
        (self.directory/'metadata.json').write_text(json.dumps(result['metadata']))
        return result

    def native(self, source):
        if not NATIVE_PYTHON.exists():
            self.fail('Native MuJoCo interpreter missing: '+str(NATIVE_PYTHON))
        program = '''import json,sys,mujoco as mj,numpy as np
from pathlib import Path
p=Path(sys.argv[1]);m=mj.MjModel.from_xml_path(str(p/'scene.xml'));d=mj.MjData(m)
meta=json.loads((p/'metadata.json').read_text())
for name,value in meta['initial_q'].items():d.qpos[m.joint(name).qposadr]=value
mj.mj_forward(m,d)
'''+source
        completed = subprocess.run([str(NATIVE_PYTHON), '-c', program, str(self.directory)],
                                   text=True, capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stdout+'\n'+completed.stderr)
        return json.loads(completed.stdout)

    def test_nonzero_state_keeps_joint_zero_geometry_and_matches_fk(self):
        stage, manifest, world, arm, tool = synthetic()
        result = self.export(stage, manifest)
        turn = np.eye(4)
        turn[:3, :3] = R.from_euler('y', .42).as_matrix()
        expected = world@arm@turn@tool
        native = self.native('''bid=m.body('Tool_fixed').id
print(json.dumps(dict(position=d.xpos[bid].tolist(),rotation=d.xmat[bid].reshape(3,3).tolist(),
                     state=float(d.qpos[m.joint('wrist3').qposadr][0]),mocap=m.nmocap)))
''')
        np.testing.assert_allclose(native['position'], expected[:3, 3], atol=2e-7, rtol=0)
        np.testing.assert_allclose(native['rotation'], expected[:3, :3], atol=2e-7, rtol=0)
        self.assertEqual(native['state'], .42)
        self.assertEqual(native['mocap'], 0)
        self.assertEqual(len(result['geometry']), 7)

    def test_source_force_cap_limits_actual_jaw_effort(self):
        self.export(*synthetic()[:2])
        actual = self.native('''
forces={}
for name in ['jaw_L','jaw_R']:
    aid=m.actuator('drive_'+name).id;jid=m.joint(name).id
    d.qpos[m.jnt_qposadr[jid]]=m.jnt_range[jid,0]
    d.ctrl[aid]=m.jnt_range[jid,1]
mj.mj_forward(m,d)
for name in ['jaw_L','jaw_R']:forces[name]=float(d.actuator_force[m.actuator('drive_'+name).id])
print(json.dumps(forces))
''')
        self.assertEqual(actual, dict(jaw_L=10., jaw_R=10.))

    def test_only_adjacent_pairs_excluded_and_siblings_collide(self):
        stage, manifest, *_ = synthetic(overlap=True)
        manifest['initial_robot_q'][-1] = 0.
        result = self.export(stage, manifest)
        measured = self.native('''
pairs=sorted(set(tuple(sorted((m.body(int(m.geom_bodyid[c.geom1])).name,
                              m.body(int(m.geom_bodyid[c.geom2])).name))) for c in d.contact[:d.ncon]))
print(json.dumps(dict(pairs=pairs,active_colliders=int(np.count_nonzero(m.geom_contype)))))
''')
        pairs = [tuple(row) for row in measured['pairs']]
        self.assertTrue(any(a.startswith('Tool_L') and b.startswith('Tool_R') for a,b in pairs), pairs)
        self.assertNotIn(('Tool_L', 'Tool_fixed'), pairs)
        self.assertNotIn(('Tool_R', 'Tool_fixed'), pairs)
        self.assertEqual(measured['active_colliders'], 7)
        self.assertNotIn(['Tool_L', 'Tool_R'], result['metadata']['exclusions'])

    def test_inconsistent_source_frame_is_rejected_before_export(self):
        stage, manifest, *_ = synthetic()
        prim = stage.GetPrimAtPath(ROBOT+'/link6')
        matrix = np.array(UsdGeom.Xformable(prim).GetLocalTransformation()).T
        matrix[0, 3] += .05
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(matrix.T.tolist()))
        with self.assertRaisesRegex(ValueError, 'transform vs zero joint frame mismatch'):
            self.export(stage, manifest)


if __name__ == '__main__':
    unittest.main()
