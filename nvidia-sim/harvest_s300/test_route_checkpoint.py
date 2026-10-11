import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

import route_checkpoint as rc
from motion_assets import ROBOT
from rail_assets import BASE_BODY, ROOT_JOINT, sha


ARM_NAMES=['farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3']


def body(stage,path,position):
    p=UsdGeom.Xform.Define(stage,path).GetPrim()
    UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(*position))
    UsdPhysics.RigidBodyAPI.Apply(p)
    return p


def fixture(assembly):
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    base=body(stage,BASE_BODY,(0,1,0));anchor_path=ROBOT+'/rail_anchor';body(stage,anchor_path,(0,1,0))
    body(stage,ROBOT+'/link6',(0,1,1))
    for name in ('fixed','L','R','L_belt','R_belt'):body(stage,'/World/ActuatedTool/'+name,(0,1,1))
    for name in ('crank','linkL','linkR'):UsdGeom.Xform.Define(stage,'/World/ActuatedTool/fixed/Transmission/'+name)
    joint=UsdPhysics.PrismaticJoint.Define(stage,ROOT_JOINT);joint.CreateBody0Rel().SetTargets([anchor_path]);joint.CreateBody1Rel().SetTargets([BASE_BODY]);joint.CreateAxisAttr('Y')
    for name in ARM_NAMES:
        cls=UsdPhysics.PrismaticJoint if name==ARM_NAMES[0] else UsdPhysics.RevoluteJoint
        cls.Define(stage,ROBOT+'/joints/'+name)
    for side in ('L','R'):UsdPhysics.PrismaticJoint.Define(stage,'/World/ActuatedTool/Joints/jaw_'+side)
    root='/World/ElasticTarget';UsdPhysics.ArticulationRootAPI.Apply(UsdGeom.Xform.Define(stage,root).GetPrim())
    rod=body(stage,root+'/rod',(1,0,0));leaf=body(stage,root+'/leaf',(1,.1,0))
    fruits=[]
    for suffix in ('01','02'):
        name='Truss_00_Fruit_'+suffix;path=root+'/'+name;body(stage,path,(1,0,float(suffix)/100))
        j=UsdPhysics.FixedJoint.Define(stage,root+'/HarvestJoints/'+name);j.CreateBody0Rel().SetTargets([root+'/rod']);j.CreateBody1Rel().SetTargets([path])
        fruits.append(dict(name=name,path=path,joint=str(j.GetPath()),anchor=root+'/rod',center=[1,0,float(suffix)/100],mass=.001))
    cap=UsdGeom.Sphere.Define(stage,fruits[0]['path']+'/Collider');cap.CreateRadiusAttr(.01);UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    skin_path=root+'/Skin/mesh_0';m=UsdGeom.Mesh.Define(stage,skin_path)
    vertices=np.array([[1.1,0,0],[1.1,.1,0],[1.1,0,.1]])
    m.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices]);m.CreateFaceVertexCountsAttr([3]);m.CreateFaceVertexIndicesAttr([0,1,2])
    UsdGeom.Xform.Define(stage,'/World/Greenhouse/Plants/Plant_19')
    source='/World/Greenhouse/Plants/Plant_22';UsdGeom.Xform.Define(stage,source).GetPrim().SetActive(False)
    paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    poses=[]
    for path in paths:
        t=rc._world(stage,path);poses.append([*t[:3,3],0,0,0,1])
    poses=np.asarray(poses,float);vel=np.ones((len(paths),6))*.002
    for i,path in enumerate(paths):
        if path.startswith(ROBOT) or path.startswith('/World/ActuatedTool'):poses[i,1]+=1.
        if path==anchor_path:poses[i,1]-=1.
        if path.startswith(root):poses[i,1]+=.2
    # Measured arm orientation differs from authored zero pose: q must be
    # replayed by native JointState instead of baking this rotation into FK.
    poses[paths.index(ROBOT+'/link6'),3:]=[0,0,np.sin(.2),np.cos(.2)]
    names=ARM_NAMES+['jaw_L','jaw_R','root_joint'];q=np.arange(len(names),dtype=float)*.01;q[-1]=1.
    arrays=dict(body_paths=np.asarray(paths),body_poses_world=poses,body_velocities_world=vel,
        robot_dof_names=np.asarray(names),robot_q=q,robot_qvel=q*.01,
        plant_dof_names=np.asarray(['rod:rotX']),plant_q=np.array([.1]),plant_qvel=np.array([.002]))
    manifest=dict(scene_sha256='',plant=dict(root=root,source_plant=source,
        bodies=[dict(path=root+'/rod'),dict(path=root+'/leaf')],fruits=fruits,skin_paths=[skin_path]),target=fruits[0],
        rail=dict(initial_station_y_m=1.,limits_world_y_m=[0.,5.],limits_relative_m=[-1.,4.],dof_name='root_joint',anchor_body=anchor_path),
        grasp_reference=dict(center=[1,0,.1],height_offset_m=.009),initial_robot_q=[0]*7,
        tool=dict(marker='keep exact tool'),mount_revision=dict(robot_translation_y_m=-.2))
    metadata=dict(scene_sha256='',source_plant=source,time_s=10.,break_events=[dict(joint=fruits[0]['joint'],t=9.)],
        sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
            cycle_plans=[dict(status='motion_completed',end_s=10.,stop_reason=None)]))
    assembly.mkdir();stage.GetRootLayer().Export(str(assembly/'scene.usda'));manifest['scene_sha256']=sha(assembly/'scene.usda');metadata['scene_sha256']=manifest['scene_sha256']
    (assembly/'motion_manifest.json').write_text(json.dumps(manifest))
    metadata.update(source_assembly=str(assembly.resolve()),manifest=copy.deepcopy(manifest))
    metadata.update(metadata_sha256='b'*64,controller_state=dict(schema='native_robot_drive_targets_v1',
        robot_dof_names=names,position_targets=(q+.0084).tolist(),velocity_targets=[0.]*len(names),
        position_source='native_get_dof_position_targets',velocity_source='native_get_dof_velocity_targets'))
    np.savez_compressed(assembly/'skin.npz',**{'0_vertices':vertices,'0_ids':np.zeros((3,2),int),
        '0_weights':np.array([[1.,0]]*3),'0_rest':np.array([[1.,0,0]]),'0_body_paths':np.array([root+'/rod'])})
    return stage,manifest,metadata,arrays


class Kinematics:
    def __init__(self,stage):self.names=ARM_NAMES


class RouteCheckpointTests(unittest.TestCase):
    def test_target_attachment_scope_keeps_excluded_obstacle_joints_exact(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget'
        body(stage,root+'/neck',(0,0,0));fruits=[]
        for index in (0,1,2):
            name=f'Truss_{index:02d}_Fruit_01';path=root+'/'+name
            body(stage,path,(1,0,0))
            joint=UsdPhysics.FixedJoint.Define(stage,root+'/HarvestJoints/'+name)
            joint.CreateBody0Rel().SetTargets([root+'/neck']);joint.CreateBody1Rel().SetTargets([path])
            joint.CreateLocalPos0Attr(Gf.Vec3f(1,0,0));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
            joint.CreateBreakForceAttr(3.);joint.CreateBreakTorqueAttr(.08)
            fruits.append(dict(name=name,path=path,joint=str(joint.GetPath()),anchor=root+'/neck',
                break_force_n=3.,break_torque_nm=.08))
        manifest=dict(plant=dict(fruits=fruits))
        from fruit_attachment import configure_joint
        config=configure_joint(stage,fruits[2]['joint'],kind='elastic',force_n=3.,torque_nm=.008)
        template=dict(joints=[config],scope='Only current-truss fruit-neck joints')
        before=rc._joint_snapshot(stage.GetPrimAtPath(fruits[0]['joint']))
        report=rc._apply_attachment_template(stage,manifest,template,
            selected_names=[fruits[1]['name'],fruits[2]['name']],selection_provenance={'sha256':'a'*64})
        self.assertEqual(rc._joint_snapshot(stage.GetPrimAtPath(fruits[0]['joint'])),before)
        self.assertEqual(fruits[0]['break_torque_nm'],.08)
        self.assertEqual(fruits[1]['break_torque_nm'],.008)
        self.assertEqual(report['selection'],'exact_eligible_near_trusses')
        self.assertEqual(report['excluded_fruit_names'],[fruits[0]['name']])
        self.assertTrue(report['excluded_joint_properties_unchanged'])
        self.assertEqual(stage.GetPrimAtPath(fruits[0]['joint']).GetTypeName(),'PhysicsFixedJoint')
        self.assertEqual(stage.GetPrimAtPath(fruits[1]['joint']).GetTypeName(),'PhysicsJoint')
        with self.assertRaises(ValueError):rc._apply_attachment_template(stage,manifest,template,selected_names=['Fruit_01'])
        mismatched=dict(joints=[config,dict(config,torque_nm=.08)])
        with self.assertRaisesRegex(ValueError,'heterogeneous'):rc._apply_attachment_template(stage,manifest,mismatched)

    def test_joint_state_velocity_does_not_become_drive_velocity_target(self):
        with tempfile.TemporaryDirectory() as directory:
            stage,m,metadata,arrays=fixture(Path(directory)/'assembly')
            joint=stage.GetPrimAtPath(ROBOT+'/joints/'+ARM_NAMES[0])
            drive=UsdPhysics.DriveAPI.Apply(joint,'linear');drive.CreateTargetVelocityAttr(.123)
            with patch.object(rc,'ToolKinematics',Kinematics):rc.author_robot_state(stage,m,arrays)
            self.assertAlmostEqual(drive.GetTargetVelocityAttr().Get(),.123,places=6)
            self.assertNotEqual(drive.GetTargetVelocityAttr().Get(),arrays['robot_qvel'][0])

    def test_proven_controller_commands_are_authored_separately_from_state(self):
        with tempfile.TemporaryDirectory() as directory:
            stage,m,metadata,arrays=fixture(Path(directory)/'assembly')
            joint=stage.GetPrimAtPath(ROBOT+'/joints/'+ARM_NAMES[0])
            drive=UsdPhysics.DriveAPI.Apply(joint,'linear')
            targets=arrays['robot_q']+.0084;velocity=np.zeros(len(targets))
            with patch.object(rc,'ToolKinematics',Kinematics):
                rc.author_robot_state(stage,m,arrays,dict(position_targets=targets,velocity_targets=velocity,provenance={'verified':True}))
            self.assertAlmostEqual(drive.GetTargetPositionAttr().Get(),targets[0],places=6)
            self.assertEqual(drive.GetTargetVelocityAttr().Get(),0.)
            self.assertEqual(joint.GetAttribute('state:linear:physics:position').Get(),arrays['robot_q'][0])

    def test_unsigned_zero_identity_canonicalization_keeps_exact_numeric_pose(self):
        from motion_kinematics import transmission_transforms
        from compact_motion_assets import property_inventory
        stage=Usd.Stage.CreateInMemory()
        original={}
        for name,matrix in transmission_transforms(.044).items():
            path='/World/ActuatedTool/fixed/Transmission/'+name
            op=UsdGeom.Xform.Define(stage,path).MakeMatrixXform()
            op.Set(Gf.Matrix4d(matrix.T.tolist()));original[path]=np.asarray(op.Get()).copy()
        report=rc._canonicalize_transmission_zero_signs(stage)
        self.assertEqual(len(report),3)
        for path,before in original.items():
            after=np.asarray(stage.GetPrimAtPath(path).GetAttribute('xformOp:transform').Get())
            np.testing.assert_array_equal(after,before)
            self.assertFalse(np.any(np.signbit(after[after==0.])))
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'signed_zero.usdc';stage.Flatten().Export(str(output))
            self.assertEqual(property_inventory(stage),property_inventory(Usd.Stage.Open(str(output))))

    def test_robot_zero_rig_joint_state_and_fixed_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            stage,m,metadata,arrays=fixture(Path(directory)/'assembly')
            anchor=rc._world(stage,m['rail']['anchor_body']).copy()
            with patch.object(rc,'ToolKinematics',Kinematics):result=rc.author_robot_state(stage,m,arrays)
            np.testing.assert_array_equal(rc._world(stage,m['rail']['anchor_body']),anchor)
            np.testing.assert_allclose(rc._world(stage,BASE_BODY)[:3,3],[0,2,0])
            np.testing.assert_allclose(rc._world(stage,ROBOT+'/link6')[:3,:3],np.eye(3))
            self.assertEqual(result['station_world_y_m'],2.)
            self.assertEqual(m['initial_robot_q'],arrays['robot_q'][:7].tolist())
            self.assertEqual(m['initial_jaw_q']['jaw_R'],arrays['robot_q'][8])
            j=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(ROOT_JOINT))
            np.testing.assert_allclose(j.GetLocalPos0Attr().Get(),[0,1,0])
            self.assertEqual(j.GetPrim().GetAttribute('state:linear:physics:position').Get(),0.)
            self.assertEqual(m['rail']['limits_relative_m'],[-2.,3.])

    def test_freeze_measured_shape_and_keep_detached_fruit_dynamic(self):
        with tempfile.TemporaryDirectory() as directory:
            assembly=Path(directory)/'assembly';stage,m,metadata,arrays=fixture(assembly)
            result=rc.freeze_outgoing(stage,assembly,m,metadata,arrays)
            root=result['frozen_root']
            self.assertFalse(stage.GetPrimAtPath('/World/ElasticTarget'))
            self.assertFalse(stage.GetPrimAtPath(root+'/rod').HasAPI(UsdPhysics.RigidBodyAPI))
            f=stage.GetPrimAtPath(root+'/Truss_00_Fruit_01');self.assertTrue(f.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertFalse(stage.GetPrimAtPath(root+'/Truss_00_Fruit_02').HasAPI(UsdPhysics.RigidBodyAPI))
            disabled=stage.GetPrimAtPath(root+'/HarvestJoints/Truss_00_Fruit_01')
            self.assertTrue(disabled.IsActive())
            self.assertIs(UsdPhysics.Joint(disabled).GetJointEnabledAttr().Get(),False)
            path=Path(directory)/'frozen.usdc';stage.Flatten().Export(str(path))
            reopened=Usd.Stage.Open(str(path));disabled=reopened.GetPrimAtPath(root+'/HarvestJoints/Truss_00_Fruit_01')
            self.assertTrue(disabled and disabled.IsA(UsdPhysics.Joint))
            self.assertIs(UsdPhysics.Joint(disabled).GetJointEnabledAttr().Get(),False)
            np.testing.assert_allclose(UsdGeom.Mesh(stage.GetPrimAtPath(root+'/Skin/mesh_0')).GetPointsAttr().Get(),
                [[1.1,.2,0],[1.1,.3,0],[1.1,.2,.1]],atol=1e-7)
            self.assertEqual(result['detached_fruits'][0]['original_joint'],m['plant']['fruits'][0]['joint'])
            np.testing.assert_allclose(UsdPhysics.RigidBodyAPI(f).GetVelocityAttr().Get(),[.002]*3)
            np.testing.assert_allclose(UsdPhysics.RigidBodyAPI(f).GetAngularVelocityAttr().Get(),np.rad2deg([.002]*3))
            self.assertTrue(stage.GetPrimAtPath(m['plant']['source_plant']) and not stage.GetPrimAtPath(m['plant']['source_plant']).IsActive())

    def test_reject_nonfinite_missing_and_wrong_model(self):
        with tempfile.TemporaryDirectory() as directory:
            _,m,metadata,arrays=fixture(Path(directory)/'assembly')
            rc.validate_snapshot(metadata,arrays,m)
            for field in ('body_velocities_world','robot_qvel','plant_qvel'):
                invalid=copy.deepcopy(arrays);invalid[field].flat[0]=np.nan
                with self.assertRaises(ValueError):rc.validate_snapshot(metadata,invalid,m)
            wrong=copy.deepcopy(metadata);wrong['scene_sha256']='bad'
            with self.assertRaises(ValueError):rc.validate_snapshot(wrong,arrays,m)
            wrong=copy.deepcopy(arrays);wrong['body_paths'][0]=wrong['body_paths'][1]
            with self.assertRaises(ValueError):rc.validate_snapshot(metadata,wrong,m)

    def test_previously_fallen_fruit_uses_latest_state_and_original_release_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            stage,m,metadata,arrays=fixture(Path(directory)/'assembly')
            root='/World/RouteFrozen/Plant_19';path=root+'/Truss_00_Fruit_01'
            body(stage,path,(0,0,.8))
            joint=UsdPhysics.FixedJoint.Define(stage,root+'/HarvestJoints/Truss_00_Fruit_01')
            joint.GetPrim().SetActive(False)
            record=dict(path=path,joint=str(joint.GetPath()),native_break_events=[dict(t=3.,joint='/World/ElasticTarget/HarvestJoints/Truss_00_Fruit_01')],
                        source_scene_sha256='a'*64,checkpoint_metadata_sha256='b'*64,pose_world=[0,0,.8,0,0,0,1])
            m['route_checkpoint']=dict(outgoing_history=[dict(frozen_root=root,detached_fruits=[record])])
            arrays['body_paths']=np.r_[arrays['body_paths'],[path]]
            arrays['body_poses_world']=np.vstack([arrays['body_poses_world'],[.2,.3,.01,0,0,0,1]])
            arrays['body_velocities_world']=np.vstack([arrays['body_velocities_world'],[.1,.2,.3,.4,.5,.6]])
            native_proof=copy.deepcopy(record['native_break_events'])
            report=rc.restore_prior_detached(stage,m,metadata,arrays)
            np.testing.assert_allclose(rc._world(stage,path)[:3,3],[.2,.3,.01])
            np.testing.assert_allclose(UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetVelocityAttr().Get(),[.1,.2,.3])
            self.assertEqual(record['native_break_events'],native_proof)
            self.assertEqual(record['checkpoint_metadata_sha256'],'b'*64)
            self.assertEqual(record['pose_world'][:3],[.2,.3,.01])
            self.assertEqual(report['updated_dynamic_fruit_count'],1)

    def test_immutable_switch_keeps_tool_mount_and_previous_fruit(self):
        with tempfile.TemporaryDirectory() as directory:
            assembly=Path(directory)/'assembly';stage,m,metadata,arrays=fixture(assembly);source_sha=sha(assembly/'scene.usda')
            from compact_motion_assets import compact_assembly,verify_compact_dependencies
            compact=Path(directory)/'compact';compact_assembly(assembly,compact)
            assembly=compact;source_sha=sha(assembly/'scene.usda');metadata['scene_sha256']=source_sha
            metadata.update(source_assembly=str(assembly.resolve()),manifest=json.loads((assembly/'motion_manifest.json').read_text()),
                metadata_sha256='b'*64)
            def new_plant(stage,path,report,output):
                root='/World/ElasticTarget';UsdGeom.Xform.Define(stage,root);body(stage,root+'/rod',(1,1,0));fruit_path=root+'/Truss_00_Fruit_01';body(stage,fruit_path,(1,1,.2))
                f=dict(name='Truss_00_Fruit_01',path=fruit_path,joint=root+'/HarvestJoints/new',anchor=root+'/rod',center=[1,1,.2],mass=.001)
                report['plant']=dict(root=root,source_plant=path,fruits=[f],bodies=[],skin_paths=[])
                stage.GetPrimAtPath(path).SetActive(False);np.savez_compressed(output/'skin.npz')
            with patch('native_route_checkpoint.load_checkpoint',return_value=(metadata,arrays)),patch.object(rc,'ToolKinematics',Kinematics),patch.object(rc,'build_plant',new_plant):
                output=Path(directory)/'next';rc.switch_window(assembly,Path(directory)/'checkpoint',output,19,'Truss_00_Fruit_01')
                new=json.loads((output/'motion_manifest.json').read_text())
                self.assertEqual(new['tool'],m['tool']);self.assertEqual(new['mount_revision'],m['mount_revision'])
                self.assertEqual(new['plant']['source_plant'],'/World/Greenhouse/Plants/Plant_19')
                self.assertEqual(new['route_checkpoint']['outgoing']['detached_fruits'][0]['route_identity'],'Plant_22/Truss_00_Fruit_01')
                self.assertEqual(sha(assembly/'scene.usda'),source_sha)
                self.assertTrue((output/'composed.usdc').is_file());verify_compact_dependencies(output,new)
                self.assertEqual(new['compact_assets']['materialized_dependency']['sha256'],sha(output/'composed.usdc'))
                with self.assertRaises(FileExistsError):rc.switch_window(assembly,Path(directory)/'checkpoint',output,19,'Truss_00_Fruit_01')
                with self.assertRaises(ValueError):rc.switch_window(assembly,Path(directory)/'checkpoint',Path(directory)/'reset',22,'Truss_00_Fruit_01')

    def test_same_wrapper_sha_does_not_authorize_other_parent_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            assembly=Path(directory)/'assembly';stage,m,metadata,arrays=fixture(assembly)
            metadata['manifest']['tool']['marker']='different materialized parent'
            with patch('native_route_checkpoint.load_checkpoint',return_value=(metadata,arrays)):
                output=Path(directory)/'next'
                with self.assertRaisesRegex(ValueError,'exact checkpoint parent'):rc.switch_window(assembly,Path(directory)/'checkpoint',output,19,'Truss_00_Fruit_01')
                self.assertFalse(output.exists())

    def test_compact_dependency_tamper_is_rejected(self):
        from compact_motion_assets import compact_assembly
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'assembly';_,m,metadata,arrays=fixture(source)
            compact=Path(directory)/'compact';compact_assembly(source,compact)
            data=compact/'composed.usdc';data.write_bytes(data.read_bytes()+b'tampered')
            with self.assertRaises(ValueError):rc.switch_window(compact,Path(directory)/'checkpoint',Path(directory)/'next',19,'Truss_00_Fruit_01')
            self.assertFalse((Path(directory)/'next').exists())


if __name__=='__main__':unittest.main()
