import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from compact_motion_assets import file_sha256
from native_route_start import prepare_route_start,apply_route_start
from test_resume_native_state import Articulation,Bodies


class NewPlant:
    def __init__(self):
        self.count=1;self.max_dofs=2
        self.shared_metatype=SimpleNamespace(dof_names=['new_leaf:0','new_leaf:1'])
        self.q=np.array([[.012,.023]]);self.qvel=np.array([[.034,.045]])
    def get_dof_positions(self):return self.q
    def get_dof_velocities(self):return self.qvel
    def set_dof_positions(self,*args):raise AssertionError('New active plant must not be reset')
    def set_dof_velocities(self,*args):raise AssertionError('New active plant must not be reset')
    def set_dof_actuation_forces(self,*args):raise AssertionError('New active plant preload must not be overwritten')


class NativeRouteStartTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.assembly=self.root/'assembly';self.assembly.mkdir()
        self.stage=Usd.Stage.CreateNew(str(self.assembly/'scene.usda'))
        self.base='/World/HarvestRobot/Robot/world';self.carrier='/World/HarvestRobot/Robot/S300Carrier'
        self.tool='/World/ActuatedTool/R';self.old_fruit='/World/ElasticTarget/Truss_02_Fruit_01'
        self.frozen='/World/RouteFrozen/Plant_22/Truss_02_Fruit_01'
        for path in (self.base,self.carrier,self.tool,self.old_fruit,self.frozen):
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage,path).GetPrim())
        self.stage.GetRootLayer().Save();sha=file_sha256(self.assembly/'scene.usda')
        self.names=['lift','jaw_L','jaw_R','root_joint']
        self.source_paths=[self.base,self.carrier,self.tool,self.old_fruit]
        self.poses=np.column_stack([np.arange(12).reshape(4,3)/10.,np.tile([0,0,0,1.],(4,1))])
        self.velocities=np.arange(24).reshape(4,6)/100.
        self.arrays=dict(body_paths=np.array(self.source_paths),body_poses_world=self.poses,
            body_velocities_world=self.velocities,robot_dof_names=np.array(self.names),
            robot_q=np.array([.31,.02,.03,.75]),robot_qvel=np.array([.11,.12,.13,.14]))
        self.metadata=dict(scene_sha256='a'*64,metadata_sha256='b'*64,time_s=40.,
            source_plant='/World/Greenhouse/Plants/Plant_22',
            manifest=dict(plant=dict(root='/World/ElasticTarget'),rail=dict(carrier_body=self.carrier)),
            sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
                cycle_plans=[dict(status='motion_completed',end_s=40.,stop_reason=None)]))
        self.metadata['controller_state']=dict(schema='native_robot_drive_targets_v1',robot_dof_names=self.names,
            position_targets=[.3184,.02,.03,.75],velocity_targets=[0.,0.,0.,0.],
            position_source='native_get_dof_position_targets',velocity_source='native_get_dof_velocity_targets')
        outgoing=dict(source_plant=self.metadata['source_plant'],checkpoint_metadata_sha256='b'*64,
            checkpoint_time_s=40.,frozen_root='/World/RouteFrozen/Plant_22',detached_fruits=[dict(path=self.frozen)])
        self.manifest=dict(scene_sha256=sha,plant=dict(source_plant='/World/Greenhouse/Plants/Plant_19',root='/World/ElasticTarget'),
            rail=dict(dof_name='root_joint',carrier_body=self.carrier,initial_displacement_m=0.,
                checkpoint_rebase=dict(world_station_preserved=True,previous_native_q_m=.75)),
            route_checkpoint=dict(schema='s300-active-window-checkpoint-v1',checkpoint=str(self.root/'checkpoint'),
                source_scene_sha256='a'*64,checkpoint_time_s=40.,outgoing=outgoing,outgoing_history=[outgoing]))

    def tearDown(self):self.tmp.cleanup()

    def prepare(self):
        with patch('native_route_start.load_checkpoint',return_value=(self.metadata,self.arrays)):
            return prepare_route_start(self.stage,self.assembly,self.manifest)

    def test_only_robot_and_remapped_released_fruit_restore_new_plant_unchanged(self):
        context=self.prepare();robot=Articulation(self.names);plant=NewPlant()
        paths=[self.base,self.tool,self.old_fruit,self.carrier,self.frozen]
        poses=np.vstack([self.poses[0],self.poses[2],[9.,8.,7.,0,0,0,1],self.poses[1],np.array([1.,1.,1.,0,0,0,1])])
        velocities=np.vstack([self.velocities[0],self.velocities[2],[1.,2.,3.,4.,5.,6.],self.velocities[1],np.zeros(6)])
        bodies=Bodies(paths,poses,velocities);plant_q=plant.q.copy();plant_qvel=plant.qvel.copy();refreshed=[]
        report=apply_route_start(context,robot,plant,bodies,refresh_articulations=lambda:refreshed.append(True))
        self.assertEqual(refreshed,[True]);np.testing.assert_array_equal(robot.q[0],[.31,.02,.03,0.])
        np.testing.assert_array_equal(robot.qvel[0],self.arrays['robot_qvel'])
        np.testing.assert_array_equal(plant.q,plant_q);np.testing.assert_array_equal(plant.qvel,plant_qvel)
        np.testing.assert_array_equal(bodies.poses[paths.index(self.old_fruit)],poses[paths.index(self.old_fruit)])
        np.testing.assert_array_equal(bodies.poses[paths.index(self.frozen)],self.poses[-1])
        np.testing.assert_array_equal(bodies.velocities[paths.index(self.frozen)],self.velocities[-1])
        self.assertEqual({paths[i] for i in bodies.set_rows[0]},{self.carrier,self.frozen})
        self.assertEqual(report['max_position_error_m'],0.)
        self.assertFalse(report['new_active_plant_native_state_set']);self.assertFalse(report['new_active_plant_preload_set'])
        self.assertEqual(report['new_active_rest_diagnostics']['bodies'][0]['measured_initial_velocity_world'],[1.,2.,3.,4.,5.,6.])

    def test_rail_rebase_is_exact_documented_coordinate_change(self):
        self.manifest['rail']['checkpoint_rebase']['previous_native_q_m']=.8
        with self.assertRaisesRegex(ValueError,'measured source q'):self.prepare()
        self.manifest['rail']['checkpoint_rebase']['previous_native_q_m']=.75
        self.manifest['rail']['checkpoint_rebase']['world_station_preserved']=False
        with self.assertRaisesRegex(ValueError,'Documented'):self.prepare()

    def test_standalone_background_leaf_measured_pose_velocity_is_restored(self):
        path='/World/BackgroundCompliantFoliage/Plant_23/Leaf_CompoundLeaf_03'
        jid='/World/BackgroundCompliantFoliage/Plant_23/Joints/Leaf_CompoundLeaf_03'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage,path).GetPrim())
        j=UsdPhysics.Joint.Define(self.stage,jid);j.CreateBody1Rel().SetTargets([path]);j.CreateExcludeFromArticulationAttr(True)
        row=dict(path=path,joint=jid,source_stem='/World/Greenhouse/Plants/Plant_23/Stem',
            source_group='CompoundLeaf_03',meshes=[path+'/Appendage_mesh_1'],mass=.002,
            inertia=[1e-6,2e-6,3e-6],stiffness=.005,damping=.0008)
        self.manifest['background_compliant_foliage']=dict(bodies=[copy.deepcopy(row)])
        self.metadata['manifest']['background_compliant_foliage']=dict(bodies=[copy.deepcopy(row)])
        self.stage.GetRootLayer().Save();self.manifest['scene_sha256']=file_sha256(self.assembly/'scene.usda')
        pose=np.array([.2,.3,.4,0.,0.,0.,1.]);velocity=np.array([.01,.02,.03,.04,.05,.06])
        self.arrays['body_paths']=np.r_[self.arrays['body_paths'],[path]]
        self.arrays['body_poses_world']=np.vstack([self.arrays['body_poses_world'],pose])
        self.arrays['body_velocities_world']=np.vstack([self.arrays['body_velocities_world'],velocity])
        context=self.prepare();paths=[self.base,self.carrier,self.tool,self.frozen,self.old_fruit,path]
        poses=np.vstack([self.poses[0],self.poses[1],self.poses[2],self.poses[3],
            np.array([2.,3.,4.,0,0,0,1.]),np.array([9.,8.,7.,0,0,0,1.])])
        velocities=np.vstack([self.velocities,np.ones(6),np.zeros(6)])
        bodies=Bodies(paths,poses,velocities);result=apply_route_start(context,Articulation(self.names),NewPlant(),bodies)
        np.testing.assert_array_equal(bodies.poses[-1],pose);np.testing.assert_array_equal(bodies.velocities[-1],velocity)
        self.assertEqual(result['restored_passive_background_leaf_paths'],[path])
        self.assertEqual(result['new_passive_background_leaf_paths'],[])

    def test_controller_targets_rebased_but_new_plant_and_measured_velocities_not_commands(self):
        target=np.array([.3184,.021,.029,.751]);velocity=np.array([0.,0.,0.,0.])
        self.metadata['controller_state']=dict(schema='native_robot_drive_targets_v1',robot_dof_names=self.names,
            position_targets=target.tolist(),velocity_targets=velocity.tolist(),
            position_source='native_get_dof_position_targets',velocity_source='native_get_dof_velocity_targets')
        context=self.prepare();robot=Articulation(self.names);plant=NewPlant()
        bodies=Bodies([self.base,self.carrier,self.tool,self.old_fruit,self.frozen],
            np.vstack([self.poses,self.poses[-1]]),np.vstack([self.velocities,self.velocities[-1]]))
        report=apply_route_start(context,robot,plant,bodies)
        np.testing.assert_allclose(robot.targets[0],[.3184,.021,.029,.001],atol=1e-16)
        np.testing.assert_array_equal(robot.velocity_targets[0],velocity)
        np.testing.assert_array_equal(robot.qvel[0],self.arrays['robot_qvel'])
        self.assertEqual(report['restored_controller_state']['position_targets_max_abs_error'],0.)
        self.assertFalse(report['new_active_plant_native_state_set'])

    def test_float32_quaternion_norm_roundoff_is_not_orientation_error(self):
        self.arrays['body_poses_world'][0,3:]=[0.,0.,0.,.99999994]
        context=self.prepare();robot=Articulation(self.names)
        paths=[self.base,self.carrier,self.tool,self.old_fruit,self.frozen]
        bodies=Bodies(paths,np.vstack([self.poses,self.poses[-1]]),np.vstack([self.velocities,self.velocities[-1]]))
        report=apply_route_start(context,robot,NewPlant(),bodies)
        self.assertEqual(report['max_orientation_error_rad'],0.)

    def test_solver_cached_articulated_velocity_is_separate_from_standalone_replay(self):
        context=self.prepare();robot=Articulation(self.names)
        paths=[self.base,self.carrier,self.tool,self.old_fruit,self.frozen]
        velocities=np.vstack([self.velocities,self.velocities[-1]])
        velocities[0,:3]+=.0002;velocities[2,3:]+=.0003
        bodies=Bodies(paths,np.vstack([self.poses,self.poses[-1]]),velocities)
        report=apply_route_start(context,robot,NewPlant(),bodies)
        self.assertEqual(report['standalone_max_linear_velocity_error_m_s'],0.)
        self.assertEqual(report['standalone_max_angular_velocity_error_rad_s'],0.)
        self.assertGreater(report['articulated_max_linear_velocity_error_m_s'],.0001)
        self.assertGreater(report['articulated_max_angular_velocity_error_rad_s'],.0001)
        self.assertEqual({paths[i] for i in bodies.set_rows[0]},{self.carrier,self.frozen})
        np.testing.assert_array_equal(robot.qvel[0],self.arrays['robot_qvel'])

    def test_current_active_fruit_alias_is_rejected(self):
        self.manifest['route_checkpoint']['outgoing']['detached_fruits'][0]['path']=self.old_fruit
        with self.assertRaisesRegex(ValueError,'aliases'):self.prepare()

    def test_parent_proof_invalid_capture_or_missing_measured_body_rejected(self):
        self.metadata['sequence']['checkpoint_physics_valid']=False
        with self.assertRaises(ValueError):self.prepare()
        self.metadata['sequence']['checkpoint_physics_valid']=True
        self.manifest['route_checkpoint']['outgoing']['checkpoint_metadata_sha256']='c'*64
        with self.assertRaisesRegex(ValueError,'parent state proof'):self.prepare()

    def test_actual_native_dof_order_is_not_guessed_or_sorted(self):
        context=self.prepare();robot=Articulation(list(reversed(self.names)))
        bodies=Bodies([self.base,self.carrier,self.tool,self.old_fruit,self.frozen],
            np.vstack([self.poses,self.poses[-1]]),np.vstack([self.velocities,self.velocities[-1]]))
        with self.assertRaisesRegex(ValueError,'DOF order'):apply_route_start(context,robot,NewPlant(),bodies)

    def test_prior_window_dynamic_fruit_uses_current_parent_snapshot_not_active_alias(self):
        prior='/World/RouteFrozen/Plant_18/Truss_00_Fruit_01'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage,prior).GetPrim())
        self.stage.GetRootLayer().Save();self.manifest['scene_sha256']=file_sha256(self.assembly/'scene.usda')
        self.arrays['body_paths']=np.r_[self.arrays['body_paths'],[prior]]
        self.arrays['body_poses_world']=np.vstack([self.arrays['body_poses_world'],[.6,.7,.01,0,0,0,1]])
        self.arrays['body_velocities_world']=np.vstack([self.arrays['body_velocities_world'],[.2,.3,.4,.5,.6,.7]])
        self.manifest['route_checkpoint']['outgoing_history'].insert(0,dict(source_plant='/World/Greenhouse/Plants/Plant_18',
            frozen_root='/World/RouteFrozen/Plant_18',detached_fruits=[dict(path=prior)]))
        context=self.prepare()
        self.assertEqual(context['body_mapping'][prior],prior)
        self.assertEqual(context['body_mapping'][self.frozen],self.old_fruit)
        self.assertNotIn(self.old_fruit,context['body_mapping'])

    def test_missing_measured_robot_link_is_not_filled_with_authored_pose(self):
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage,'/World/HarvestRobot/Robot/ExtraLink').GetPrim())
        self.stage.GetRootLayer().Save();self.manifest['scene_sha256']=file_sha256(self.assembly/'scene.usda')
        with self.assertRaisesRegex(ValueError,'complete measured parent'):self.prepare()

    def test_route_continuation_missing_controller_proof_is_not_measured_hold(self):
        self.metadata.pop('controller_state')
        with self.assertRaisesRegex(ValueError,'proven original controller'):self.prepare()


if __name__=='__main__':unittest.main()
