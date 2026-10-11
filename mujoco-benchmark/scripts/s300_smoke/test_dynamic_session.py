"""Small native fixtures for harvesting state, guards and contact exceptions.

No server USD or prior result files are required. Run with the native
MuJoCo 3.15 interpreter using unittest discovery.
"""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from plan_adapter import DOF_NAMES
from run_harvest import Session


def fixture(directory):
    root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
    option = ET.SubElement(root, 'option', gravity='0 0 0', integrator='discrete', timestep=str(1/120))
    ET.SubElement(option, 'flag', filterparent='disable')
    world = ET.SubElement(root, 'worldbody')
    actuators = ET.SubElement(root, 'actuator')
    equality = ET.SubElement(root, 'equality')
    names, geometry = {}, []

    def body(parent, name, pos='0 0 0', mass='.1'):
        node = ET.SubElement(parent, 'body', name=name, pos=pos)
        ET.SubElement(node, 'inertial', pos='0 0 0', mass=mass, diaginertia='.001 .001 .001')
        return node

    def geom(node, name, pos='0 0 0', kind='sphere', size='.005'):
        ET.SubElement(node, 'geom', name=name, type=kind, size=size, pos=pos,
                      contype='1', conaffinity='3')
        geometry.append(dict(name=name, collision=True))

    parent = world
    for index, name in enumerate(DOF_NAMES[:8]):
        body_name = 'Robot_world' if index == 0 else 'Robot_link'+str(index)
        parent = body(parent, body_name)
        names['/source/'+body_name] = body_name
        slider = index < 2
        ET.SubElement(parent, 'joint', name=name, type='slide' if slider else 'hinge',
                      axis='0 1 0' if index == 0 else '0 0 1',
                      limited='true', range='-3 3')
        ET.SubElement(actuators, 'position', name='drive_'+name, joint=name,
                      kp='500' if slider else '800', kv='10',
                      forcelimited='true', forcerange='-250 250')
    fixed = body(parent, 'Tool_fixed')
    names['/source/Tool_fixed'] = 'Tool_fixed'
    geom(fixed, 'fixed_collector', pos='3 0 0')
    for side, x in [('L', -.06), ('R', .06)]:
        jaw = body(fixed, 'Tool_'+side)
        names['/source/Tool_'+side] = 'Tool_'+side
        ET.SubElement(jaw, 'joint', name='jaw_'+side, type='slide', axis='1 0 0',
                      limited='true', range='-.03 .03')
        ET.SubElement(actuators, 'position', name='drive_jaw_'+side,
                      joint='jaw_'+side, kp='1500', kv='30',
                      forcelimited='true', forcerange='-10 10')
        belt = body(jaw, 'Tool_'+side+'_belt')
        names['/source/Tool_'+side+'_belt'] = 'Tool_'+side+'_belt'
        geom(belt, 'belt_'+side, pos=str(x)+' 0 0')

    rods = []
    for name, pos in [('Truss_00_Rachis_0', '1 0 1'),
                      ('Truss_00_Pedicel_09_0', '1.1 0 1'),
                      ('Truss_01_Rachis_0', '2.5 0 1'),
                      ('Stem_MainStem_0', '2 0 1')]:
        node = body(world, name, pos)
        if name == 'Truss_00_Rachis_0':
            ET.SubElement(node, 'joint', name='branch_hinge', type='hinge',
                          axis='0 1 0', stiffness='10', damping='.1')
        geom(node, name+'_geom')
        rods.append(name)
        if '_Rachis_' in name:
            ET.SubElement(node, 'site', name=name+'_anchor', pos='0 .03 0', size='.001')
    attachments = []
    fruits = []
    for truss, pos in [('00', '1 .03 1'), ('01', '2.5 .03 1')]:
        name = 'Truss_'+truss+'_Fruit_03'
        node = body(world, name, pos, mass='.002')
        ET.SubElement(node, 'freejoint', name=name+'_free')
        geom(node, name+'_geom', size='.01')
        ET.SubElement(node, 'site', name=name+'_neck', size='.001')
        ET.SubElement(equality, 'connect', name=name+'_attachment',
                      site1='Truss_'+truss+'_Rachis_0_anchor', site2=name+'_neck')
        attachments.append(dict(name=name, anchor='Truss_'+truss+'_Rachis_0',
                                local_rot0=[0, 0, 0, 1], local_rot1=[0, 0, 0, 1],
                                stiffness=[.01, .03, .03], damping=.00015,
                                deadband_deg=1., force_n=100., torque_nm=100.))
        fruits.append(name)
    geom(world, 'GroundCover_fixture', pos='0 0 -1', kind='box', size='1 1 .1')
    geom(world, 'Gutter_fixture', pos='0 0 -2', kind='box', size='1 1 .1')
    ET.SubElement(world.find("body[@name='Robot_world']"), 'geom', name='arm_fixture',
                  type='sphere', pos='4 0 0', size='.005', contype='1', conaffinity='3')
    # Name the arm separately so the exception classifier must distinguish it
    # from the chassis/base allowance.
    arm = body(world, 'Robot_test_arm', pos='4 0 0')
    geom(arm, 'Robot_test_arm_geom')
    names['/source/Robot_test_arm'] = 'Robot_test_arm'
    xml = directory/'scene.xml'
    ET.ElementTree(root).write(xml)
    meta = dict(model_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
                robot=dict(initial_q={name:0. for name in DOF_NAMES},
                           initial_qvel={name:0. for name in DOF_NAMES},
                           body_names=names, belt_body_names=['Tool_L_belt', 'Tool_R_belt']),
                attachments=attachments, tools=['Tool_fixed','Tool_L','Tool_R','Tool_L_belt','Tool_R_belt'],
                rods=rods, fruits=fruits, fruit_specs=[], source_assembly=str(directory),
                source_trial=str(directory), geometry=geometry, rail_origin_station_m=0.,
                windows=[dict(id='fixture', prefix='', bodies=rods+fruits,
                              fruit_specs=[], source_assembly=str(directory),
                              source_trial=str(directory), source_plan=str(directory/'source_plan.json'))])
    (directory/'export.json').write_text(json.dumps(meta))


def plan(session, phase, *, rail=0., gap=.096):
    q = session.data.qpos[session.qadr].copy()
    return dict(settle_seconds=0., initial_command=q.tolist(), initial_gap_m=gap,
                preparation_opening=dict(feedback_before_phases=['approach']),
                phases=[dict(name=phase, duration_s=1/120, waypoint_q=[q[1:8].tolist()]*2,
                             rail_waypoint_positions_m=[rail,rail], end_gap_m=gap,
                             belt_local_y_m_s=[0.,0.])])


class HarvestSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='s300_harvest_test_')
        self.directory = Path(self.temp.name)
        fixture(self.directory)
        self.session = Session(self.directory, gravity_compensation=False)

    def tearDown(self):
        self.temp.cleanup()

    def allow_pair(self, other, *, fruit='Truss_00_Fruit_03', geom=None):
        s = self.session
        body_ids = [s.model.body(fruit).id, s.model.body(other).id if other else 0]
        geom_ids = [s.model.geom(fruit+'_geom').id,
                    s.model.geom(geom or other+'_geom').id]
        return s.released_contact_allowed(body_ids, geom_ids)

    def release(self, name='Truss_00_Fruit_03'):
        s = self.session
        s.data.eq_active[s.model.equality(name+'_attachment').id] = False

    def test_exception_requires_released_fruit_and_same_truss(self):
        self.assertFalse(self.allow_pair('Truss_00_Pedicel_09_0'))
        self.release()
        self.assertTrue(self.allow_pair('Truss_00_Pedicel_09_0'))
        self.assertTrue(self.allow_pair('Truss_00_Rachis_0'))
        self.assertFalse(self.allow_pair('Truss_01_Rachis_0'))
        self.assertFalse(self.allow_pair('Truss_01_Fruit_03'))
        self.assertFalse(self.allow_pair('Stem_MainStem_0'))
        self.assertFalse(self.allow_pair('Robot_test_arm'))
        self.assertTrue(self.allow_pair('', geom='GroundCover_fixture'))
        self.assertFalse(self.allow_pair('', geom='Gutter_fixture'))

    def test_rail_guard_uses_actual_arrival_before_actuation(self):
        s = self.session
        result = s.execute(plan(s, 'approach', rail=.2), self.directory)
        self.assertEqual(result['stop'], 'measured_rail_arrival_not_reached')
        self.assertEqual(s.data.time, 0.)
        feedback = json.loads((self.directory/'opening_feedback.json').read_text())
        self.assertEqual(feedback[0]['rail_arrival']['absolute_error_m'], [.2])

    def test_break_during_solver_does_not_reclassify_that_contact_as_dropped(self):
        s = self.session
        bid = s.model.body('Truss_00_Fruit_03').id
        joint = int(s.model.body_jntadr[bid])
        address = int(s.model.jnt_qposadr[joint])
        # An explicit positive-control initial intersection. It must remain
        # invalid even when its solved force triggers a release this timestep.
        s.data.qpos[address:address+3] = [1., .007, 1.]
        s.at.force_limit[:] = 0.
        mj.mj_forward(s.model,s.data)
        s.step(np.zeros(10),np.zeros(2),'release_transition_control')
        self.assertFalse(s.data.eq_active[s.model.equality('Truss_00_Fruit_03_attachment').id])
        self.assertIsNotNone(s.first_violation)
        pair = {'Truss_00_Fruit_03','Truss_00_Rachis_0'}
        matched = [row for row in s.contacts if set(row[2:4])==pair]
        self.assertTrue(matched)
        self.assertTrue(all(not row[-2] and not row[-1] for row in matched))

    def test_opening_guard_does_not_substitute_requested_gap(self):
        s = self.session
        result = s.execute(plan(s, 'approach', gap=.096), self.directory)
        self.assertEqual(result['stop'], 'measured_opening_not_reached')
        self.assertEqual(s.data.time, 0.)
        feedback = json.loads((self.directory/'opening_feedback.json').read_text())
        self.assertEqual(feedback[-1]['actual_gap_m'], .044)

    def test_measured_arrival_and_opening_allow_physical_step(self):
        s = self.session
        s.data.qpos[s.qadr[0]] = .2
        s.data.qpos[s.qadr[8]] = -.026
        s.data.qpos[s.qadr[9]] = .026
        mj.mj_forward(s.model,s.data)
        result = s.execute(plan(s, 'approach', rail=.2), self.directory)
        self.assertTrue(result['completed'])
        self.assertGreater(s.data.time,0.)
        self.assertEqual(s.model.nmocap,0)

    def test_commands_produce_finite_effort_without_pose_assignment(self):
        s = self.session
        command = s.data.qpos[s.qadr].copy()
        command[2] = .3
        command[8:10] = [-.02, .02]
        s.step(command,np.array([-.05,-.05]),'native_drive')
        self.assertGreater(s.data.time,0.)
        self.assertGreater(np.linalg.norm(s.data.qpos[s.qadr]),0.)
        self.assertGreater(abs(s.data.qpos[s.qadr[2]]-command[2]),.1)
        self.assertTrue(np.all(np.abs(s.data.actuator_force[s.aids[8:]])<=10.00001))
        self.assertEqual(s.model.nmocap,0)
        for ids in s.belt_geoms:
            np.testing.assert_allclose(s.model.geom_surfacevel[ids,:3],[[0,-.05,0]],atol=1e-12)

    def test_released_no_contact_fruit_is_not_pulled_by_belt(self):
        s = self.session
        self.release()
        bid = s.model.body('Truss_00_Fruit_03').id
        mj.mj_kinematics(s.model,s.data)
        initial_position = s.data.xpos[bid].copy()
        for _ in range(4):
            s.step(np.zeros(10),np.array([-.05,-.05]),'no_contact_belt')
        mj.mj_kinematics(s.model,s.data)
        np.testing.assert_allclose(s.data.xpos[bid],initial_position,atol=1e-12)
        np.testing.assert_allclose(s.data.xfrc_applied[bid],np.zeros(6),atol=1e-12)

    def test_model_hash_mismatch_is_rejected(self):
        with (self.directory/'scene.xml').open('a') as stream:
            stream.write('\n<!-- changed -->\n')
        with self.assertRaisesRegex(ValueError,'Model hash mismatch'):
            Session(self.directory)


if __name__=='__main__':
    unittest.main()
