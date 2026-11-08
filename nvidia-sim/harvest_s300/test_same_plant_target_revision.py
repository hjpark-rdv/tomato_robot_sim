import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics

from compact_motion_assets import file_sha256
from native_route_checkpoint import save_checkpoint,load_checkpoint
from plant_interaction_variants import disable_other_plant_contacts,verified_disabled_colliders
from resume_native_state import prepare_resume,apply_resume
from same_plant_target_revision import (FIELD,SCHEMA,revise,validate_resume_revision,
    verified_active_release_inventory,_recipe,_audit)
import test_harvest_contact_scope as contact_scope_tests
from test_resume_native_state import Articulation,Bodies

ROOT='/World/ElasticTarget'
NAMES=['root_joint','farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']


def fixture(folder,*,with_instances=False,implicit_old_flags=False):
    folder.mkdir();foreign=folder/'foreign';foreign.mkdir()
    old,stage,frozen,_=contact_scope_tests.HarvestContactScopeTests().route_fixture(foreign)
    frozenjoint=stage.GetPrimAtPath('/World/RouteFrozen/Plant_22/HarvestJoints/Truss_02_Fruit_09')
    frozenjoint.SetActive(True);UsdPhysics.Joint(frozenjoint).CreateJointEnabledAttr(False)
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    stage.SetTimeCodesPerSecond(24.);stage.SetFramesPerSecond(24.)
    UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    def body(path,position):
        p=UsdGeom.Xform.Define(stage,path).GetPrim();UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.01)
        UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(*position));return p
    for path in ['/World/HarvestRobot/Robot/world','/World/HarvestRobot/Robot/S300Carrier',
            '/World/ActuatedTool/L','/World/ActuatedTool/R']:
        body(path,[0.,1.,.6])
    UsdPhysics.ArticulationRootAPI.Apply(UsdGeom.Xform.Define(stage,ROOT).GetPrim())
    body(ROOT+'/Stem_MainStem_00',[0.,0.,0.])
    cap=UsdGeom.Capsule.Define(stage,ROOT+'/Stem_MainStem_00/Collider');cap.CreateRadiusAttr(.01);cap.CreateHeightAttr(.2)
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim()).CreateCollisionEnabledAttr(True)
    bodies=[dict(name='Stem_MainStem_00',path=ROOT+'/Stem_MainStem_00')];fruits=[]
    for t in (0,1,2):
        rod=f'{ROOT}/Truss_{t:02d}_Rachis_00';body(rod,[.1*t,0.,1.])
        cap=UsdGeom.Capsule.Define(stage,rod+'/Collider');cap.CreateRadiusAttr(.002);cap.CreateHeightAttr(.05)
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim()).CreateCollisionEnabledAttr(True)
        bodies.append(dict(name=rod.rsplit('/',1)[-1],path=rod))
        for i in (1,2):
            name=f'Truss_{t:02d}_Fruit_{i:02d}';path=ROOT+'/'+name;body(path,[.1*t,0.,1.1+i*.02])
            for suffix in ['/Collider','/Visual_01']:
                mesh=UsdGeom.Mesh.Define(stage,path+suffix);mesh.CreatePointsAttr([(0.,0.,0.),(.01,0.,0.),(0.,.01,0.)])
                mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
                UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(suffix=='/Collider')
            j=UsdPhysics.FixedJoint.Define(stage,ROOT+'/HarvestJoints/'+name)
            j.CreateBody0Rel().SetTargets([rod]);j.CreateBody1Rel().SetTargets([path]);j.CreateBreakForceAttr(3.)
            j.CreateBreakTorqueAttr(.008);j.CreateExcludeFromArticulationAttr(True)
            fruits.append(dict(name=name,path=path,joint=str(j.GetPath()),anchor=rod,center=[.1*t,0.,1.1+i*.02]))
    # Leaf and skin CollisionAPIs must remain dormant even when physically named.
    for path in [ROOT+'/Leaf_01/Collider',ROOT+'/Skin/mesh_0']:
        p=UsdGeom.Mesh.Define(stage,path);UsdPhysics.CollisionAPI.Apply(p.GetPrim()).CreateCollisionEnabledAttr(False)
    m=dict(plant=dict(root=ROOT,source_plant='/World/Greenhouse/Plants/Plant_20',bodies=bodies,fruits=fruits),
        target=copy.deepcopy(fruits[-1]),route_checkpoint=old['route_checkpoint'],
        rail=dict(base_body='/World/HarvestRobot/Robot/world',carrier_body='/World/HarvestRobot/Robot/S300Carrier',
            dof_name='root_joint',initial_station_y_m=1.,source_base_position_world_m=[0.,1.,.6]),
        grasp_reference=dict(height_offset_m=.009),training_eligible=False,harvest_success=None)
    if with_instances:
        stage.GetRootLayer().documentation='Exact authored namespace and instance template must survive retarget'
        template=UsdGeom.Xform.Define(stage,'/AuthoredPlantTemplate').GetPrim()
        mesh=UsdGeom.Mesh.Define(stage,'/AuthoredPlantTemplate/Mesh')
        mesh.CreatePointsAttr([(0.,0.,0.),(.03,0.,0.),(0.,.04,0.)]);mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
        for i in range(3):
            instance=UsdGeom.Xform.Define(stage,f'/World/FullFarm/Instance_{i}').GetPrim()
            instance.GetReferences().AddInternalReference('/AuthoredPlantTemplate');instance.SetInstanceable(True)
    if implicit_old_flags:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
            path=str(prim.GetPath())
            if path.startswith(ROOT+'/Truss_02_') and path.endswith('/Collider'):
                prim.RemoveProperty('physics:collisionEnabled')
    m['plant_interaction_variant']=disable_other_plant_contacts(stage,m,include_already_disabled=True)
    stage.GetRootLayer().Export(str(folder/'scene.usda'));m['scene_sha256']=file_sha256(folder/'scene.usda')
    (folder/'motion_manifest.json').write_text(json.dumps(m));np.savez(folder/'skin.npz',vertices=np.arange(12))
    paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    cache=UsdGeom.XformCache();poses=[]
    for p in paths:
        matrix=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p)))
        poses.append([*matrix[3,:3],0.,0.,0.,1.])
    velocities=np.arange(len(paths)*6).reshape(-1,6)*.00001
    parent=folder.parent/'previous_trial';parent.mkdir();cp=parent/'cycle_checkpoints/002'
    metadata=save_checkpoint(cp,assembly=folder,scene_sha256=m['scene_sha256'],manifest=m,time_s=10.,
        body_paths=paths,body_poses_world=poses,body_velocities_world=velocities,
        robot_dof_names=NAMES,robot_q=np.arange(10)*.01,robot_qvel=np.arange(10)*.0001,
        robot_position_targets=np.arange(10)*.01+.00084,robot_velocity_targets=np.zeros(10),
        plant_dof_names=['rod:0','rod:1','rod:2'],plant_q=[.1,.2,.3],plant_qvel=[.01,.02,.03],preload=[.4,.5,.6],
        breaks=[dict(t=8.,phase='close',joint=fruits[4]['joint'])],
        sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
            cycle_plans=[dict(status='motion_completed',end_s=10.,stop_reason=None)]))
    np.savez(parent/'states.npz',time=np.array([10.]),poses=np.array([poses]),robot_q=np.array([np.arange(10)*.01]))
    (parent/'summary.json').write_text(json.dumps(dict(completed=True,stop_reason=None,initialization_valid=True,
        actual_self_collision_audit_passed=True,simulated_s=10.,model_sha256=m['scene_sha256'])))
    (parent/'self_collision_actual.json').write_text(json.dumps(dict(passed=True,complete=True,
        scene_sha256=m['scene_sha256'],input_sha256=file_sha256(parent/'states.npz'),physics_rerun=False)))
    return stage,m,cp,frozen,metadata


class SamePlantTargetRevisionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.source=self.root/'source';self.stage,self.old,self.cp,self.frozen,self.metadata=fixture(self.source)
    def tearDown(self):self.tmp.cleanup()
    def derive(self):
        output=self.root/'derived';report=revise(self.source,self.cp,output,'Truss_00_Fruit_01')
        m=json.loads((output/'motion_manifest.json').read_text());stage=Usd.Stage.Open(str(output/'scene.usda'))
        return output,m,stage,report

    def test_authored_instance_namespace_documentation_and_internal_references_preserved(self):
        special=self.root/'instance_source'
        # A separate fixture has its own immutable parent trial directory.
        directory=self.root/'instanced';directory.mkdir();special=directory/'source'
        source,old,cp,_,meta=fixture(special,with_instances=True)
        output=directory/'derived';report=revise(special,cp,output,'Truss_00_Fruit_01')
        revised=json.loads((output/'motion_manifest.json').read_text())
        actual=Sdf.Layer.OpenAsAnonymous(str(output/'composed.usdc'))
        original=Sdf.Layer.OpenAsAnonymous(str(special/'scene.usda'))
        paths=lambda layer:self.spec_paths(layer)
        self.assertEqual(paths(original),paths(actual))
        self.assertFalse(any(path.startswith('/Flattened_Prototype_')for path in paths(actual)))
        self.assertEqual(original.documentation,actual.documentation)
        for i in range(3):
            path=f'/World/FullFarm/Instance_{i}'
            self.assertEqual(original.GetPrimAtPath(path).GetInfo('references'),actual.GetPrimAtPath(path).GetInfo('references'))
            self.assertTrue(actual.GetPrimAtPath(path).GetInfo('instanceable'))
        self.assertTrue(report['property_proof']['source_authored_namespace_preserved'])
        self.assertEqual(validate_resume_revision(output,revised,meta)['target_truss'],'Truss_00')
        storage=json.loads((output/'checkpoint_materialization_identity.json').read_text())
        self.assertEqual(storage['source_composition_mode'],'exact_self_contained_authored_layer_copy')
        self.assertTrue(storage['canonical_equal'])

    @staticmethod
    def spec_paths(layer):
        paths=[];layer.Traverse(Sdf.Path.absoluteRootPath,lambda p:paths.append(str(p)));return set(paths)

    def test_schema_fallback_true_is_restored_as_no_authored_property(self):
        directory=self.root/'implicit';directory.mkdir();special=directory/'source'
        source,old,cp,_,meta=fixture(special,implicit_old_flags=True)
        oldpath=ROOT+'/Truss_02_Rachis_00/Collider.physics:collisionEnabled'
        self.assertIsNone(source.GetRootLayer().GetAttributeAtPath(oldpath))
        output=directory/'derived';revise(special,cp,output,'Truss_00_Fruit_01')
        revised=json.loads((output/'motion_manifest.json').read_text())
        self.assertEqual(validate_resume_revision(output,revised,meta)['target_truss'],'Truss_00')
        actual=Sdf.Layer.OpenAsAnonymous(str(output/'composed.usdc'))
        self.assertFalse(actual.GetAttributeAtPath(oldpath).default)
        # Native release stays physically enabled even though its true value
        # is a schema fallback, not a newly invented authored opinion.
        released=ROOT+'/Truss_02_Fruit_01/Collider.physics:collisionEnabled'
        self.assertIsNone(actual.GetAttributeAtPath(released))

    def test_changed_flag_metadata_or_time_samples_cannot_be_hidden_by_normalization(self):
        output,m,_,_=self.derive();source=Usd.Stage.Open(str(self.source/'scene.usda'))
        recipe=_recipe(source,self.old,self.metadata,self.cp,'Truss_00_Fruit_01')
        for mutate in [lambda a:a.SetMetadata('displayName','concealed extra metadata'),lambda a:a.Set(False,1.)]:
            from same_plant_target_revision import _open_asset
            wrong=_open_asset(output,m)
            mutate(wrong.GetPrimAtPath(ROOT+'/Truss_02_Fruit_02/Collider').GetAttribute('physics:collisionEnabled'))
            with self.assertRaisesRegex(ValueError,'nondefault authored metadata'):_audit(source,wrong,recipe)

    def test_exact_primary_flags_and_all_released_preserved_without_plant_rebuild(self):
        before={p.name:p.read_bytes() for p in self.source.iterdir() if p.is_file()}
        output,m,stage,report=self.derive()
        self.assertEqual(report['schema'],SCHEMA);self.assertFalse(report['plant_rebuilt_or_frozen'])
        self.assertEqual(len(report['exact_new_target_primary_colliders']),3)
        self.assertEqual(sum(r['after']is True for r in report['property_proof']['collision_changes']),3)
        self.assertEqual(sum(r['after']is False for r in report['property_proof']['collision_changes']),2)
        self.assertEqual(m['plant'],self.old['plant']);self.assertEqual(m['route_checkpoint'],self.old['route_checkpoint'])
        self.assertEqual((output/'skin.npz').read_bytes(),(self.source/'skin.npz').read_bytes())
        for p,value in before.items():self.assertEqual((self.source/p).read_bytes(),value)
        released=ROOT+'/Truss_02_Fruit_01/Collider'
        self.assertTrue(stage.GetPrimAtPath(released).GetAttribute('physics:collisionEnabled').Get())
        self.assertTrue(stage.GetPrimAtPath(self.frozen+'/Collider').GetAttribute('physics:collisionEnabled').Get())
        disabled=verified_disabled_colliders(stage,m)
        self.assertNotIn(released,disabled);self.assertNotIn(self.frozen+'/Collider',disabled)
        for path in [ROOT+'/Truss_00_Fruit_01/Visual_01',ROOT+'/Leaf_01/Collider',ROOT+'/Skin/mesh_0']:
            self.assertFalse(stage.GetPrimAtPath(path).GetAttribute('physics:collisionEnabled').Get())
        # Derivation leaves even the proven broken source joint authored EXACTLY as before;
        # existing resume disables it in session before compilation, never reattaches it.
        self.assertIsNot(UsdPhysics.Joint(stage.GetPrimAtPath(released.rsplit('/',1)[0].replace('/Truss_','/HarvestJoints/Truss_',1))).GetJointEnabledAttr().Get(),False)
        with self.assertRaises(FileExistsError):revise(self.source,self.cp,output,'Truss_00_Fruit_01')

    def test_explicit_resume_only_restores_full_state_controller_and_break_markers(self):
        output,m,stage,_=self.derive()
        with self.assertRaisesRegex(ValueError,'exact same source assembly'):
            prepare_resume(stage,output,m,self.cp)
        context=prepare_resume(stage,output,m,self.cp,allow_target_truss_revision=True)
        _,arrays=load_checkpoint(self.cp)
        robot=Articulation(NAMES);plant=Articulation(arrays['plant_dof_names'])
        bodies=Bodies(arrays['body_paths'],arrays['body_poses_world'].copy(),arrays['body_velocities_world'].copy())
        for path in context['report']['standalone_body_paths']:
            i=bodies.prim_paths.index(path);bodies.poses[i,0]+=.2;bodies.velocities[i]*=0.
        result=apply_resume(context,robot,plant,bodies)
        for kind,view in [('robot',robot),('plant',plant)]:
            np.testing.assert_array_equal(view.q[0],arrays[kind+'_q']);np.testing.assert_array_equal(view.qvel[0],arrays[kind+'_qvel'])
        np.testing.assert_array_equal(plant.preload[0],arrays['preload'])
        np.testing.assert_array_equal(robot.targets[0],context['metadata']['controller_state']['position_targets'])
        self.assertEqual(result['max_position_error_m'],0.)
        self.assertEqual(context['report']['initial_native_break_count'],1)
        self.assertFalse(context['report']['imported_native_prior'][0]['native_new_event'])
        self.assertEqual(context['report']['imported_native_prior'][0]['original_t'],8.)
        self.assertFalse(UsdPhysics.Joint(stage.GetPrimAtPath(self.old['plant']['fruits'][4]['joint'])).GetJointEnabledAttr().Get())
        self.assertFalse(result['bitwise_engine_continuity'])
        self.assertEqual(context['report']['explicit_proxy_revision']['active_native_prior_break_count'],1)

    def test_independent_full_authored_audit_rejects_shape_material_mass_joint_or_robot_changes(self):
        output,m,_,_=self.derive();source=Usd.Stage.Open(str(self.source/'scene.usda'))
        recipe=_recipe(source,self.old,self.metadata,self.cp,'Truss_00_Fruit_01')
        for mutate in [
            lambda s:UsdGeom.Capsule(s.GetPrimAtPath(ROOT+'/Truss_00_Rachis_00/Collider')).GetRadiusAttr().Set(.009),
            lambda s:UsdPhysics.MassAPI(s.GetPrimAtPath(ROOT+'/Truss_00_Fruit_01')).GetMassAttr().Set(.1),
            lambda s:UsdPhysics.Joint(s.GetPrimAtPath(self.old['plant']['fruits'][0]['joint'])).GetBreakForceAttr().Set(9.),
            lambda s:s.GetPrimAtPath(ROOT+'/Truss_00_Fruit_01/Collider').CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.02),
            lambda s:UsdGeom.Xformable(s.GetPrimAtPath('/World/HarvestRobot/Robot/world')).GetOrderedXformOps()[0].Set(Gf.Vec3d(1.,1.,.6)),
            lambda s:s.DefinePrim('/World/ExtraCollider','Cube')]:
            from pxr import Sdf
            wrong=Usd.Stage.Open(Usd.Stage.Open(str(output/'scene.usda')).Flatten(addSourceFileComment=False));mutate(wrong)
            with self.assertRaisesRegex(ValueError,'noncollision authored properties or topology'):_audit(source,wrong,recipe)

    def test_no_visual_enabling_active_release_ghosting_or_arbitrary_collider_exemption(self):
        output,m,_,_=self.derive();source=Usd.Stage.Open(str(self.source/'scene.usda'))
        recipe=_recipe(source,self.old,self.metadata,self.cp,'Truss_00_Fruit_01')
        for path,value in [(ROOT+'/Truss_00_Fruit_01/Visual_01',True),
                (ROOT+'/Truss_02_Fruit_01/Collider',False),
                (ROOT+'/Stem_MainStem_00/Collider',False),
                (ROOT+'/Truss_01_Fruit_01/Collider',True)]:
            wrong=Usd.Stage.Open(Usd.Stage.Open(str(output/'scene.usda')).Flatten(addSourceFileComment=False))
            wrong.GetPrimAtPath(path).GetAttribute('physics:collisionEnabled').Set(value)
            with self.assertRaisesRegex(ValueError,'exact target/release whitelist'):_audit(source,wrong,recipe)
        bad=copy.deepcopy(m);bad['plant_interaction_variant']['preserved_native_released_fruit_colliders'].pop(ROOT+'/Truss_02_Fruit_01/Collider')
        with self.assertRaisesRegex(ValueError,'preservation declaration'):verified_disabled_colliders(Usd.Stage.Open(str(output/'scene.usda')),bad)
        bad=copy.deepcopy(m);bad[FIELD]['preserved_active_native_released_fruit_colliders'][ROOT+'/Truss_02_Fruit_02/Collider']={}
        with self.assertRaisesRegex(ValueError,'exact checkpoint proof'):verified_active_release_inventory(Usd.Stage.Open(str(output/'scene.usda')),bad)

    def test_same_truss_released_target_missing_target_and_stopped_or_old_checkpoint_rejected(self):
        for name in ['Truss_02_Fruit_02','Truss_02_Fruit_01','Truss_44_Fruit_01']:
            with self.assertRaises(ValueError):revise(self.source,self.cp,self.root/name,name)
        parent=self.cp.parent.parent
        summary=json.loads((parent/'summary.json').read_text())
        for changes in [dict(completed=False,stop_reason='invalid'),dict(simulated_s=11.)]:
            (parent/'summary.json').write_text(json.dumps(dict(summary,**changes)))
            with self.assertRaisesRegex(ValueError,'roll back|cannot roll back'):
                revise(self.source,self.cp,self.root/'stopped','Truss_00_Fruit_01')
        self.assertFalse((self.root/'stopped').exists())

    def test_checkpoint_source_manifest_skin_and_contact_policy_tamper_rejected(self):
        output,m,_,_=self.derive()
        for mutate in [lambda x:x[FIELD].update(checkpoint_metadata_sha256='f'*64),
                lambda x:x[FIELD].update(checkpoint_npz_sha256='e'*64),
                lambda x:x[FIELD].update(source_plant='/World/Greenhouse/Plants/Plant_99'),
                lambda x:x['plant'].update(root='/World/FakePlant'),
                lambda x:x.update(contact_limit_mm=5.),
                lambda x:x[FIELD].update(native_breaks_reattached=True)]:
            bad=copy.deepcopy(m);mutate(bad)
            with self.assertRaises(ValueError):validate_resume_revision(output,bad,self.metadata)
        (output/'skin.npz').write_bytes(b'changed skin')
        with self.assertRaisesRegex(ValueError,'skin changed'):validate_resume_revision(output,m,self.metadata)

    def test_target_revision_authorization_cannot_expand_other_resume_permissions(self):
        for flag in ['allow_leaf_proxy_revision','allow_accuracy_proxy_revision','allow_attachment_transfer',
                'allow_conditioned_start','allow_terminal_recovery','allow_original_cycle_suffix']:
            with self.assertRaisesRegex(ValueError,'cannot be combined'):
                prepare_resume(self.stage,self.source,self.old,self.cp,allow_target_truss_revision=True,**{flag:True})
        with self.assertRaisesRegex(ValueError,'Explicit boolean'):
            prepare_resume(self.stage,self.source,self.old,self.cp,allow_target_truss_revision=1)


if __name__=='__main__':unittest.main()
