import tempfile
import unittest
from pathlib import Path

import mujoco as mj
import numpy as np

from newton_geometric_harvest import collision_variant, geometric_plan, capture_receipt, sensor_poses, volume_bounds
from plan_adapter import sample_plan
from newton_contact_policy import identify_geometries


class GeometricHarvestTests(unittest.TestCase):
    def test_tall_box_counts_new_upper_and_lower_centres_and_retains_declared_legacy_box(self):
        model=self.fixture();plan=geometric_plan(self.plan())
        fruits=['Truss_00_Fruit_01','p18_Truss_00_Fruit_01']
        low,high=volume_bounds()
        np.testing.assert_allclose(high-low,[.096,.158,.084])
        self.assertAlmostEqual((low[2]+high[2])/2,.037)
        with tempfile.TemporaryDirectory() as root:
            for z0,z1,expected in ((-.004,.078,2),(-.006,.080,0)):
                model.body_pos[model.body(fruits[0]).id,2]=z0
                model.body_pos[model.body(fruits[1]).id,2]=z1
                path=self.states(root,model)
                result=capture_receipt(model,dict(robot_qpos_addresses=np.arange(10)),plan,path,fruits)
                self.assertEqual(len(result['harvested']),expected)
                old=capture_receipt(model,dict(robot_qpos_addresses=np.arange(10)),plan,path,fruits,
                    volume=dict(tool_box_min_m=[-.048,0.,.013],tool_box_max_m=[.048,.158,.061]))
                self.assertFalse(old['harvested'])
        for policy in (dict(tool_box_min_m=[0,0,0],tool_box_max_m=[0,1,1]),
                       dict(tool_box_min_m=[0,0,float('nan')])):
            with self.assertRaises(ValueError):volume_bounds(policy)

    def fixture(self):
        dummy=''.join(f'<body pos="0 0 {i+2}"><joint type="slide"/><geom size=".001"/></body>' for i in range(9))
        return mj.MjModel.from_xml_string(f'''<mujoco><option gravity="0 0 0"/><worldbody>
          <body name="Tool_fixed"><joint type="slide" axis="1 0 0"/><geom name="belt" size=".02"/></body>
          {dummy}
          <body name="Stem_MainStem_0"><geom name="stem" size=".03"/></body>
          <body name="p18_Stem_MainStem_0"><geom name="other_stem" size=".03"/></body>
          <body name="Truss_00_Rachis"><geom name="branch" size=".03"/></body>
          <body name="Truss_00_Fruit_01" pos="-.02 .1 .03"><geom name="fruit" size=".01"/></body>
          <body name="p18_Truss_00_Fruit_01" pos=".02 .1 .03"><geom name="other_fruit" size=".01"/></body>
          <body name="Truss_00_Fruit_02" pos=".1 .1 .03"><geom name="outside_fruit" size=".01"/></body>
          <body name="gutter"><geom name="gutter" size=".03"/></body>
        </worldbody></mujoco>''')

    def metadata(self):
        names=['Stem_MainStem_0','p18_Stem_MainStem_0','Truss_00_Rachis',
               'Truss_00_Fruit_01','p18_Truss_00_Fruit_01','Truss_00_Fruit_02']
        return dict(windows=[dict(bodies=names)],attachments=[dict(name=n) for n in names if 'Fruit_' in n])

    def plan(self):
        names=['ready_hold','prepare_open','approach','close','cycle_open','withdraw','return_ready']
        return dict(initial_command=[0.]*10,initial_gap_m=.044,settle_seconds=.05,
            total_duration_s=7.05,phases=[dict(name=n,duration_s=1.,waypoint_q=[[0.]*7]*3,
                end_gap_m=.096,rail_native_command_positions_m=[0.]*3,belt_local_y_m_s=[.2,.2]) for n in names])

    def states(self,root,model,*,offset=0.,phase='geometric_capture_hold'):
        path=Path(root)/'states.npz';q=np.zeros(model.nq);q[0]=offset
        np.savez_compressed(path,phase=np.array(['initial',phase]),qpos=np.array([q,q]),time=[0.,3.3])
        return path

    def test_actual_contacts_disabled_for_every_fruit_and_branch_but_stems_and_gutter_remain(self):
        source=self.fixture();model,report=collision_variant(source,self.metadata())
        d=mj.MjData(model);mj.mj_forward(model,d)
        contacts={model.geom(int(g)).name for c in d.contact for g in c.geom}
        self.assertIn('stem',contacts);self.assertIn('other_stem',contacts);self.assertIn('gutter',contacts)
        self.assertNotIn('branch',contacts)
        for row in report['disabled_geoms']:
            self.assertEqual(model.geom_contype[row['geom_id']],0)
            self.assertEqual(model.geom_conaffinity[row['geom_id']],0)
        for name in ('geom_friction','geom_margin','geom_solref','body_mass','body_inertia','jnt_stiffness'):
            np.testing.assert_array_equal(getattr(model,name),getattr(source,name))
        self.assertNotEqual(source.geom_contype[source.geom('fruit').id],0)

    def test_ideal_hold_removes_belt_cycle_and_preserves_existing_approach_and_return_commands(self):
        old=self.plan();new=geometric_plan(old)
        self.assertEqual([p['name'] for p in new['phases']],
            ['ready_hold','prepare_open','approach','geometric_capture_hold','withdraw','return_ready'])
        for t in np.linspace(0,new['total_duration_s'],80):
            np.testing.assert_array_equal(sample_plan(new,t)['authored_belt_speed'],[0.,0.])
        self.assertEqual(old['phases'][3]['name'],'close')
        self.assertEqual(new['phases'][2]['waypoint_q'],old['phases'][2]['waypoint_q'])
        self.assertEqual(new['phases'][-1]['waypoint_q'],old['phases'][-1]['waypoint_q'])

    def test_only_measured_reached_volume_counts_all_prepared_plants_and_never_duplicates(self):
        model=self.fixture();plan=geometric_plan(self.plan())
        fruits=[s['name'] for s in self.metadata()['attachments']]
        with tempfile.TemporaryDirectory() as root:
            receipt=capture_receipt(model,dict(robot_qpos_addresses=np.arange(10)),plan,self.states(root,model),fruits)
            self.assertTrue(receipt['reached'])
            self.assertEqual({r['name'] for r in receipt['harvested']},set(fruits)-{'Truss_00_Fruit_02'})
            duplicate=capture_receipt(model,dict(robot_qpos_addresses=np.arange(10)),plan,
                self.states(root,model),fruits,[r['name'] for r in receipt['harvested']])
            self.assertEqual(duplicate['harvested'],[])
            for kwargs in (dict(offset=.02),dict(phase='approach')):
                rejected=capture_receipt(model,dict(robot_qpos_addresses=np.arange(10)),plan,self.states(root,model,**kwargs),fruits)
                self.assertFalse(rejected['reached']);self.assertEqual(rejected['harvested'],[])

    def test_hidden_fruit_changes_render_pose_only(self):
        poses={'fruit':np.array([0.,0.,0.,0.,0.,0.,1.]),'stem':np.ones(7)}
        hidden=sensor_poses(poses,{'fruit'})
        self.assertEqual(hidden['fruit'][2],-1000.)
        self.assertEqual(poses['fruit'][2],0.)
        np.testing.assert_array_equal(hidden['stem'],poses['stem'])

    def test_common_contact_audit_accepts_zero_fruit_roles_only_in_explicit_ideal_mode(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="Tool_L_belt"><geom name="belt" size=".01"/></body>
          <body name="Truss_00_Fruit_01"><geom name="fruit" size=".01"/></body>
          <body name="Truss_00_Pedicel_01"><geom name="pedicel" size=".01"/></body>
        </worldbody></mujoco>''')
        meta=dict(robot=dict(belt_body_names=['Tool_L_belt'],geometry=[
            dict(name='belt',collision=True,owner='Tool_L_belt',source='/Tool/L_belt/Test')]),
            attachments=[dict(name='Truss_00_Fruit_01')])
        _,roles,_=identify_geometries(model,meta)
        self.assertTrue(np.any(roles==1));self.assertTrue(np.any(roles==2))
        for name in ('fruit','pedicel'):
            i=model.geom(name).id;model.geom_contype[i]=0;model.geom_conaffinity[i]=0
        with self.assertRaises(ValueError):identify_geometries(model,meta)
        meta['geometric_harvest']=dict(mode='main_stem_only_geometric_harvest')
        _,roles,belts=identify_geometries(model,meta)
        self.assertFalse(np.any(roles));self.assertEqual(len(belts),1)
        model.geom_contype[model.geom('fruit').id]=1
        with self.assertRaisesRegex(ValueError,'still has active'):identify_geometries(model,meta)


if __name__=='__main__':unittest.main()
