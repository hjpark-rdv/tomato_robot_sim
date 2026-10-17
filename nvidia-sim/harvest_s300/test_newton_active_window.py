"""CPU tests for fixed neighboring colliders and canonical dynamic state maps."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from newton_active_window import prepare, lift_state, canonical_state
import newton_active_window as active_window
from newton_inputs import sha256


def fixture(directory, *, robot_after_plants=False, empty_visual_leaf=False):
    directory.mkdir()
    bodies,eqs,windows,attachments=[],[],[],[]
    for index,plant in enumerate(('22','27','29')):
        p='' if index==0 else 'P'+plant+'_'
        stem,branch,fruit=p+'Stem_root',p+'Stem_branch',p+'Truss_Fruit_01'
        bodies.append(f'''<body name="{stem}" pos="{index*2} 0 1">
         <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/><geom name="{p}stem_g" type="sphere" size=".03"/>
         <body name="{branch}" pos="0 0 .3"><joint name="{p}bend" type="hinge" axis="0 1 0" stiffness="2" damping=".2"/>
          <inertial pos="0 0 0" mass=".1" diaginertia=".01 .01 .01"/><geom name="{p}branch_g" type="capsule" size=".01 .05"/>
          <site name="{p}anchor" pos=".2 0 0"/></body></body>
         <body name="{fruit}" pos="{index*2+.2} 0 1.3"><freejoint name="{p}free"/>
          <inertial pos="0 0 0" mass=".01" diaginertia=".00001 .00001 .00001"/>
          <geom name="{p}fruit_g" type="sphere" size=".02" friction=".6 .01 .001"/><site name="{p}fruit_site"/></body>''')
        eqs.append(f'<connect name="{fruit}_attachment" site1="{p}anchor" site2="{p}fruit_site"/>')
        windows.append(dict(id=plant,prefix=p,bodies=[stem,branch,fruit]))
        attachments.append(dict(name=fruit,anchor=branch))
    xml='''<mujoco><compiler angle="radian" inertiafromgeom="false" fusestatic="false" discardvisual="true"/>
      <option gravity="0 0 -9.81" solver="Newton"/><worldbody><geom name="floor" type="plane" size="10 10 .1"/>
      <body name="Robot" pos="0 -3 1"><joint name="rail" type="slide" axis="0 1 0" damping=".3"/>
      <inertial pos="0 0 0" mass="2" diaginertia=".1 .1 .1"/><geom name="robot_g" type="sphere" size=".1"/></body>
      '''+''.join(bodies)+'''</worldbody><equality>'''+''.join(eqs)+'''</equality>
      <actuator><position name="rail_motor" joint="rail" kp="100"/></actuator></mujoco>'''
    if robot_after_plants:
        root=ET.fromstring(xml);world=root.find('worldbody')
        robot=next(body for body in world.findall('body') if body.get('name')=='Robot')
        world.remove(robot);world.append(robot);xml=ET.tostring(root,encoding='unicode')
    if empty_visual_leaf:
        tree=ET.fromstring(xml);world=tree.find('worldbody')
        body=ET.SubElement(world,'body',name='cached_leaf')
        ET.SubElement(body,'geom',name='discarded_leaf_visual',type='sphere',size='.1',contype='0',conaffinity='0',margin='.0007')
        xml=ET.tostring(tree,encoding='unicode')
    (directory/'input_scene.xml').write_text(xml)
    m=mj.MjModel.from_xml_path(str(directory/'input_scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d)
    initial=dict(qpos=d.qpos.copy(),qvel=d.qvel.copy(),eq_active=d.eq_active.copy(),ctrl=d.ctrl.copy(),
      robot_qpos_addresses=np.array([m.joint('rail').qposadr[0]],np.int32),robot_dof_addresses=np.array([m.joint('rail').dofadr[0]],np.int32),actuator_ids=np.array([0]),
      plant_dofs=np.array([m.joint(p+'bend').dofadr[0] for p in ('','P27_','P29_')]),
      plant_preload=np.arange(m.nv,dtype=float)*.001,runtime_dof_damping=m.dof_damping.copy(),runtime_dof_length=m.dof_length.copy(),
      body_positions=d.xpos.copy(),body_quaternions_wxyz=d.xquat.copy(),body_names=np.array([m.body(i).name for i in range(m.nbody)]),actuator_kp=np.array([100.]))
    np.savez_compressed(directory/'initial_state.npz',**initial)
    np.savez_compressed(directory/'commands.npz',time=np.array([0.]),command=np.zeros((1,1)))
    (directory/'source_export.json').write_text(json.dumps(dict(windows=windows,attachments=attachments,robot={},training_eligible=False)))
    mj.mj_saveModel(m,str(directory/'input_model.mjb'))
    filenames=['input_scene.xml','input_model.mjb','source_export.json','initial_state.npz','commands.npz']
    (directory/'inputs.json').write_text(json.dumps(dict(input_files={n:dict(sha256=sha256(directory/n)) for n in filenames})))
    return m,initial


class ActiveWindowTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.source=self.root/'full'
        self.model,self.initial=fixture(self.source)
    def tearDown(self):self.temp.cleanup()

    def test_frozen_equalities_attached_and_zero_solver_rows(self):
        p=prepare(self.source,self.root/'small','22')
        self.assertEqual(p['full_counts']['nv'],22);self.assertEqual(p['reduced_counts']['nv'],8)
        self.assertEqual(p['reduced_counts']['neq'],3);self.assertEqual(p['frozen_eq_solver_rows'],0)
        self.assertEqual(len(p['frozen_attachment_names']),2);self.assertTrue(p['protected_physics']['passed'])
        m=mj.MjModel.from_binary_path(str(self.root/'small/input_model.mjb'))
        self.assertEqual(m.body('P27_Truss_Fruit_01').jntnum[0],0)
        with np.load(self.root/'small/initial_state.npz') as s:self.assertTrue(s['eq_active'].all())
        for field in ('geom_size','geom_pos','geom_quat','geom_friction','geom_contype','geom_conaffinity'):
            np.testing.assert_array_equal(getattr(m,field),getattr(self.model,field))

    def test_measured_snapshot_detached_free_fruit_and_lift(self):
        b={k:v.copy() for k,v in self.initial.items()}
        b['qpos'][self.model.joint('P27_bend').qposadr[0]]=.19
        b['qpos'][self.model.joint('P29_bend').qposadr[0]]=-.23
        qa,va=self.model.joint('P29_free').qposadr[0],self.model.joint('P29_free').dofadr[0]
        b['qpos'][qa:qa+3]=[4.7,.1,.8];b['qvel'][va:va+6]=[.1,.2,-.3,.4,.5,.6]
        b['eq_active'][2]=False;b['ctrl'][0]=.27;b['time']=np.array([4.25])
        p=prepare(self.source,self.root/'small','27',bank_state=b)
        self.assertEqual(p['reduced_counts']['nv'],14)
        self.assertEqual(p['detached_dynamic_attachment_names'],['P29_Truss_Fruit_01'])
        self.assertEqual(p['initial_clock_s'],4.25);self.assertTrue(all(p['dynamic_state_preservation'].values()))
        self.assertLess(p['body_position_max_abs_difference_m'],1e-12)
        cached=dict(np.load(self.root/'small/canonical_full_state.npz'))
        full_data=mj.MjData(self.model);full_data.qpos[:]=b['qpos'];mj.mj_kinematics(self.model,full_data)
        np.testing.assert_array_equal(cached['body_positions'],full_data.xpos)
        m=mj.MjModel.from_binary_path(str(self.root/'small/input_model.mjb'))
        self.assertEqual(m.jnt_type[m.joint('P29_free').id],mj.mjtJoint.mjJNT_FREE)
        s=dict(np.load(self.root/'small/initial_state.npz'));s['time']=np.array([5.])
        s['qpos'][m.joint('P29_free').qposadr[0]]+=.12;s['qvel'][m.joint('P29_free').dofadr[0]]+=.17
        out,ev=lift_state(self.source,self.root/'small',s)
        self.assertAlmostEqual(out['qpos'][qa],b['qpos'][qa]+.12)
        adr=self.model.joint('bend').qposadr[0];self.assertEqual(out['qpos'][adr],b['qpos'][adr])
        self.assertEqual(out['time'][0],5.);self.assertTrue(ev['retained_qpos_exact'])
        self.assertFalse(out['eq_active'][2]);self.assertTrue(out['eq_active'][0])

    def test_reject_static_false_release_and_attachment_reactivation(self):
        prepare(self.source,self.root/'small','22');s=dict(np.load(self.root/'small/initial_state.npz'))
        s['time']=np.array([1.]);s['eq_active'][1]=False
        with self.assertRaisesRegex(ValueError,'Frozen attachment'):lift_state(self.source,self.root/'small',s)
        b={k:v.copy() for k,v in self.initial.items()};b['eq_active'][2]=False
        prepare(self.source,self.root/'loose','22',bank_state=b)
        s=dict(np.load(self.root/'loose/initial_state.npz'));s['eq_active'][2]=True;s['time']=np.array([1.])
        with self.assertRaisesRegex(ValueError,'reactivated'):lift_state(self.source,self.root/'loose',s)

    def test_robot_after_frozen_plants_preserves_named_transmission(self):
        source=self.root/'plant_before_robot';full,bank=fixture(source,robot_after_plants=True)
        bank['qpos'][bank['robot_qpos_addresses'][0]]=.23;bank['ctrl'][0]=.31
        output=self.root/'robot_after_reduced';p=prepare(source,output,'27',bank_state=bank)
        reduced=mj.MjModel.from_binary_path(str(output/'input_model.mjb'))
        old,new=int(full.actuator_trnid[0,0]),int(reduced.actuator_trnid[0,0])
        self.assertNotEqual(old,new)
        self.assertEqual(full.joint(old).name,'rail');self.assertEqual(reduced.joint(new).name,'rail')
        self.assertTrue(p['protected_physics']['checks']['actuator_transmission:rail_motor:joint_name'])
        for field in ('actuator_trntype','actuator_gainprm','actuator_biasprm','actuator_gear','actuator_ctrlrange','actuator_acc0'):
            np.testing.assert_array_equal(getattr(full,field),getattr(reduced,field))
        initial=dict(np.load(output/'initial_state.npz'))
        self.assertEqual(initial['qpos'][initial['robot_qpos_addresses'][0]],.23)
        self.assertEqual(initial['ctrl'][0],.31)

    def test_source_unchanged_and_bad_inputs_rejected(self):
        old={p.name:sha256(p) for p in self.source.iterdir()}
        with self.assertRaisesRegex(ValueError,'Unknown'):prepare(self.source,self.root/'bad','41')
        b=copy.deepcopy(self.initial);b['qpos'][0]=np.nan
        with self.assertRaisesRegex(ValueError,'nonfinite'):prepare(self.source,self.root/'nan','22',bank_state=b)
        prepare(self.source,self.root/'small','22')
        self.assertEqual(old,{p.name:sha256(p) for p in self.source.iterdir()})
        with self.assertRaises(FileExistsError):prepare(self.source,self.root/'small','22')
        (self.root/'small/input_model.mjb').write_bytes(b'changed')
        s=dict(np.load(self.root/'small/initial_state.npz'));s['time']=np.array([1.])
        with self.assertRaisesRegex(ValueError,'Reduced model SHA'):lift_state(self.source,self.root/'small',s)

    def test_prepare_scope_reuses_source_hashes_but_not_between_calls(self):
        prepare(self.source,self.root/'first','22')
        report=json.loads((self.root/'first/inputs.json').read_text())
        scope=report['verification_scope']
        self.assertEqual(scope['unique_file_sha_reads'],6)  # five inputs + manifest
        self.assertGreaterEqual(scope['stable_sha_reuses'],6)
        self.assertEqual(scope['unique_read_only_model_loads'],1)
        self.assertFalse(scope['global_or_cross_call_cache'])
        prepare(self.source,self.root/'second','22')
        other=json.loads((self.root/'second/inputs.json').read_text())['verification_scope']
        self.assertEqual(other['unique_file_sha_reads'],6)
        np.testing.assert_array_equal(
            np.load(self.root/'first/initial_state.npz')['qpos'],
            np.load(self.root/'second/initial_state.npz')['qpos'])
        self.assertEqual(sha256(self.root/'first/input_model.mjb'),sha256(self.root/'second/input_model.mjb'))

    def test_prepare_scope_rejects_source_change_after_first_verification(self):
        original=active_window._freeze_xml
        def mutate_after_read(*args):
            original(*args)
            # The model is already loaded; an otherwise unused byte change
            # must still invalidate the source at the final same-call check.
            with (self.source/'input_model.mjb').open('ab') as stream:
                stream.write(b'changed-after-first-sha')
        with mock.patch.object(active_window,'_freeze_xml',side_effect=mutate_after_read):
            with self.assertRaisesRegex(ValueError,'modified|changed'):
                prepare(self.source,self.root/'mutated','22')

    def test_compiled_margin_cache_is_preserved_without_changing_geom_margin(self):
        source=self.root/'margin_source';full,initial=fixture(source,empty_visual_leaf=True)
        leaf=full.body('cached_leaf').id;self.assertEqual(full.body_geomnum[leaf],0)
        full.body_margin[leaf]=.0009
        mj.mj_saveModel(full,str(source/'input_model.mjb'))
        manifest=json.loads((source/'inputs.json').read_text())
        manifest['input_files']['input_model.mjb']['sha256']=sha256(source/'input_model.mjb')
        (source/'inputs.json').write_text(json.dumps(manifest))
        prepare(source,self.root/'cached_margin','22')
        reduced=mj.MjModel.from_binary_path(str(self.root/'cached_margin/input_model.mjb'))
        np.testing.assert_array_equal(full.body_margin,reduced.body_margin)
        np.testing.assert_array_equal(full.geom_margin,reduced.geom_margin)

    def test_initial_clock_survives_without_an_explicit_bank(self):
        initial={k:v.copy() for k,v in self.initial.items()};initial['time']=np.array([12.5])
        self.assertEqual(canonical_state(self.model,initial)['time'][0],12.5)
        self.assertEqual(canonical_state(self.model,self.initial)['time'][0],0.)
        self.assertEqual(canonical_state(self.model,initial,dict(time=np.array([14.])))['time'][0],14.)


if __name__=='__main__':unittest.main()
