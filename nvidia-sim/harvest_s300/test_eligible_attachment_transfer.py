import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf,Usd,UsdGeom,UsdPhysics

from compact_motion_assets import file_sha256
from eligible_attachment_transfer import (apply_transfer,_audit,revise,revise_fresh,
                                         validate_resume_revision,FRESH_SCHEMA)
from fruit_attachment import configure_joint
from native_route_checkpoint import save_checkpoint,load_checkpoint
from route_checkpoint import _joint_snapshot


def fixture(folder):
    folder.mkdir();stage=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    paths=[]
    def body(path,pos):
        prim=UsdGeom.Xform.Define(stage,path).GetPrim();UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(prim);UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.01)
        sphere=UsdGeom.Sphere.Define(stage,path+'/Collider');sphere.CreateRadiusAttr(.01)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim());paths.append(path);return prim
    root='/World/ElasticTarget';UsdGeom.Xform.Define(stage,root)
    body('/World/Robot/base',[0.,1.26,.66]);fruits=[];bodies=[];template_rows=[]
    for truss in (0,1,2):
        x=-.9 if truss==0 else -.5;anchor=f'{root}/neck_{truss}'
        body(anchor,[x,0.,1.]);bodies.append(dict(path=anchor))
        for i in (1,2):
            name=f'Truss_{truss:02d}_Fruit_{i:02d}';path=f'{root}/{name}';body(path,[x,0.,1.1+i*.02])
            j=UsdPhysics.FixedJoint.Define(stage,root+'/HarvestJoints/'+name)
            j.CreateBody0Rel().SetTargets([anchor]);j.CreateBody1Rel().SetTargets([path])
            j.CreateLocalPos0Attr(Gf.Vec3f(0.,0.,.1+i*.02));j.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
            j.CreateExcludeFromArticulationAttr(True);j.CreateBreakForceAttr(3.);j.CreateBreakTorqueAttr(.08)
            f=dict(name=name,path=path,joint=str(j.GetPath()),anchor=anchor,center=[x,0.,1.1+i*.02],
                   break_force_n=3.,break_torque_nm=.08);fruits.append(f)
            if truss==2:
                config=configure_joint(stage,str(j.GetPath()),kind='elastic',force_n=3.,torque_nm=.008)
                f['break_torque_nm']=.008
                template_rows.append(dict(name=name,joint=str(j.GetPath()),anchor=anchor,fruit=path,**config))
    manifest=dict(plant=dict(root=root,source_plant='/World/Greenhouse/Plants/Plant_22',bodies=bodies,fruits=fruits),
        target=copy.deepcopy(fruits[-1]),rail=dict(initial_station_y_m=1.6,base_body='/World/Robot/base',
            source_base_position_world_m=[0.,1.26,.66]),
        initial_robot_q=[.18,0.,0.,0.,0.,0.,0.],
        fruit_attachment=dict(kind='elastic',joints=template_rows,scope='Only current Truss02',calibrated=False,training_eligible=False))
    stage.GetRootLayer().Export(str(folder/'scene.usda'));manifest['scene_sha256']=file_sha256(folder/'scene.usda')
    (folder/'motion_manifest.json').write_text(json.dumps(manifest));np.savez(folder/'skin.npz',dummy=np.zeros(1))
    inventory=dict(trusses=[dict(plant_id=22,truss_index=t,root_position_world_m=[-.55,1.6,.3],
        gutter_center_x=-.775,rail_near=True,fruit_count=2) for t in (0,1,2)])
    return stage,manifest,inventory,paths


def metadata(manifest):
    return dict(break_events=[dict(t=9.,joint=manifest['plant']['fruits'][-1]['joint'])])


class EligibleAttachmentTransferTests(unittest.TestCase):
    def test_fresh_independent_asset_preserves_normal_state_and_imports_zero_breaks(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);assembly=root/'assembly';stage,old,inventory,_=fixture(assembly)
            old['initial_robot_q']=[.18,0.,0.,0.,0.,0.,0.]
            old['grasp_reference']=dict(height_offset_m=.009)
            (assembly/'motion_manifest.json').write_text(json.dumps(old))
            inv=root/'inventory.json';inv.write_text(json.dumps(inventory))
            report=revise_fresh(assembly,root/'fresh',inv)
            fresh=json.loads((root/'fresh/motion_manifest.json').read_text())
            actual=Usd.Stage.Open(str(root/'fresh/scene.usda'))
            self.assertEqual(report['schema'],FRESH_SCHEMA)
            self.assertEqual(report['inherited_native_breaks'],[])
            self.assertFalse(report['accepted_previous_route_history'])
            self.assertEqual(fresh['initial_robot_q'],old['initial_robot_q'])
            self.assertEqual(fresh['target']['name'],'Truss_01_Fruit_01')
            self.assertEqual(set(fresh['grasp_reference']['fruits']),{'Truss_01_Fruit_01','Truss_01_Fruit_02'})
            self.assertEqual(len(report['eligible_fruit_names']),4)
            self.assertEqual(report['attachment_hypothesis']['preserved_native_broken_joints'],[])
            self.assertNotIn('eligible_attachment_transfer',fresh)
            self.assertNotIn('route_checkpoint',fresh)
            self.assertTrue(report['nonselected_properties_unchanged'])
            for fruit in old['plant']['fruits']:
                self.assertIsNot(UsdPhysics.Joint(actual.GetPrimAtPath(fruit['joint'])).GetJointEnabledAttr().Get(),False)
            for key in ('target','grasp_reference','fruit_attachment','scene_sha256','compact_assets','fresh_campaign'):
                fresh.pop(key,None);old.pop(key,None)
            for fruit in old['plant']['fruits']:
                if fruit['name'].startswith('Truss_01'):fruit['break_torque_nm']=.008
            self.assertEqual(fresh,dict(old,training_eligible=False,harvest_success=None))
            self.assertEqual(file_sha256(assembly/'scene.usda'),report['source_scene_sha256'])
            with self.assertRaises(FileExistsError):revise_fresh(assembly,root/'fresh',inv)

    def test_fresh_rejects_measured_or_previously_visited_source_and_disabled_neck(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);assembly=root/'assembly';stage,old,inventory,_=fixture(assembly)
            inv=root/'inventory.json';inv.write_text(json.dumps(inventory))
            for key in ('route_checkpoint','route_window_switch','eligible_attachment_transfer','fresh_campaign'):
                (assembly/'motion_manifest.json').write_text(json.dumps(dict(old,**{key:{}})))
                with self.assertRaisesRegex(ValueError,'unvisited authored'):
                    revise_fresh(assembly,root/('wrong_'+key),inv)
                self.assertFalse((root/('wrong_'+key)).exists())
            for q in (None,[0.]*6,[True]+[0.]*6,[float('nan')]+[0.]*6):
                (assembly/'motion_manifest.json').write_text(json.dumps(dict(old,initial_robot_q=q)))
                with self.assertRaisesRegex(ValueError,'finite authored normal robot q'):
                    revise_fresh(assembly,root/'invalid_q',inv)
                self.assertFalse((root/'invalid_q').exists())
            joint=UsdPhysics.Joint(stage.GetPrimAtPath(old['plant']['fruits'][-1]['joint']))
            joint.CreateJointEnabledAttr(False);stage.GetRootLayer().Export(str(assembly/'scene.usda'))
            old['scene_sha256']=file_sha256(assembly/'scene.usda')
            (assembly/'motion_manifest.json').write_text(json.dumps(old))
            with self.assertRaisesRegex(ValueError,'native-disabled'):
                revise_fresh(assembly,root/'disabled',inv)
            self.assertFalse((root/'disabled').exists())

    def test_fresh_exact_initial_target_cannot_expand_near_scope_or_resume_authority(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);assembly=root/'assembly';_,old,inventory,_=fixture(assembly)
            inv=root/'inventory.json';inv.write_text(json.dumps(inventory))
            for name in ('Truss_00_Fruit_01','Truss_02_Fruit_01','Truss_01_Fruit_99'):
                with self.assertRaisesRegex(ValueError,'exactly one eligible Truss01'):
                    revise_fresh(assembly,root/name,inv,target_name=name)
            revise_fresh(assembly,root/'fresh',inv)
            fresh=json.loads((root/'fresh/motion_manifest.json').read_text())
            with self.assertRaisesRegex(ValueError,'lacks exact checkpoint'):
                validate_resume_revision(root/'fresh',fresh,dict(source_assembly=str(assembly),
                    scene_sha256=old['scene_sha256'],metadata_sha256='not a native checkpoint'))

    def test_only_unbroken_eligible01_transfers_and02_native_joint_disabled(self):
        with tempfile.TemporaryDirectory() as temp:
            stage,manifest,inventory,_=fixture(Path(temp)/'assembly');old=Usd.Stage.Open(stage.Flatten())
            before={f['joint']:_joint_snapshot(stage.GetPrimAtPath(f['joint'])) for f in manifest['plant']['fruits']}
            report=apply_transfer(stage,manifest,metadata(manifest),inventory)
            self.assertEqual(report['selected_fruit_names'],['Truss_01_Fruit_01','Truss_01_Fruit_02'])
            self.assertEqual(report['approved_source_parameters']['force_n'],3.)
            self.assertEqual(report['approved_source_parameters']['torque_nm'],.008)
            for fruit in manifest['plant']['fruits']:
                prim=stage.GetPrimAtPath(fruit['joint'])
                if fruit['name'].startswith('Truss_01'):
                    self.assertEqual(prim.GetTypeName(),'PhysicsJoint');self.assertEqual(fruit['break_torque_nm'],.008)
                elif fruit['joint'] in report['preserved_native_broken_joints']:
                    self.assertIs(UsdPhysics.Joint(prim).GetJointEnabledAttr().Get(),False)
                else:self.assertEqual(_joint_snapshot(prim),before[fruit['joint']])
            self.assertEqual([r['name'] for r in manifest['fruit_attachment']['joints']],
                ['Truss_02_Fruit_01','Truss_02_Fruit_02','Truss_01_Fruit_01','Truss_01_Fruit_02'])

    def test_no_sourceplant_or_behindgutter_scope_authorization_expansion(self):
        with tempfile.TemporaryDirectory() as temp:
            stage,manifest,inventory,_=fixture(Path(temp)/'assembly')
            for wrong in (0,2):
                with self.assertRaisesRegex(ValueError,'Only explicitly approved'):
                    apply_transfer(stage,copy.deepcopy(manifest),metadata(manifest),inventory,truss_index=wrong)
            other=copy.deepcopy(manifest);other['plant']['source_plant']='/World/Greenhouse/Plants/Plant_19'
            with self.assertRaisesRegex(ValueError,'Only explicitly approved'):apply_transfer(stage,other,metadata(manifest),inventory)
            excluded=copy.deepcopy(manifest)
            for f in excluded['plant']['fruits']:
                if f['name'].startswith('Truss_01'):f['center'][0]=-.9
            with self.assertRaisesRegex(ValueError,'No unbroken eligible'):apply_transfer(stage,excluded,metadata(manifest),inventory)

    def test_broken01_fruit_is_not_reconfigured_or_resurrected(self):
        with tempfile.TemporaryDirectory() as temp:
            stage,manifest,inventory,_=fixture(Path(temp)/'assembly');m=metadata(manifest)
            m['break_events'].append(dict(t=9.5,joint=manifest['plant']['fruits'][2]['joint']))
            oldtype=stage.GetPrimAtPath(manifest['plant']['fruits'][2]['joint']).GetTypeName()
            report=apply_transfer(stage,manifest,m,inventory)
            self.assertEqual(report['selected_fruit_names'],['Truss_01_Fruit_02'])
            broken=stage.GetPrimAtPath(manifest['plant']['fruits'][2]['joint'])
            self.assertEqual(broken.GetTypeName(),oldtype)
            self.assertFalse(UsdPhysics.Joint(broken).GetJointEnabledAttr().Get())

    def test_independent_property_audit_rejects_mass_collider_or_selected_torque_tamper(self):
        with tempfile.TemporaryDirectory() as temp:
            source,old,inventory,_=fixture(Path(temp)/'assembly');m=metadata(old)
            derived=Usd.Stage.Open(source.Flatten());revised=copy.deepcopy(old)
            report=apply_transfer(derived,revised,m,inventory);proof=_audit(source,derived,old,m,inventory,report)
            self.assertTrue(proof['nonselected_properties_unchanged'])
            for mutate in (
                lambda stage:UsdPhysics.MassAPI(stage.GetPrimAtPath('/World/ElasticTarget/neck_1')).GetMassAttr().Set(.02),
                lambda stage:UsdGeom.Sphere(stage.GetPrimAtPath('/World/ElasticTarget/Truss_01_Fruit_01/Collider')).GetRadiusAttr().Set(.015),
                lambda stage:UsdPhysics.Joint(stage.GetPrimAtPath(report['selected_joint_paths'][0])).GetBreakTorqueAttr().Set(.016)):
                wrong=Usd.Stage.Open(derived.Flatten());mutate(wrong)
                with self.assertRaisesRegex(ValueError,'Actual derived properties'):_audit(source,wrong,old,m,inventory,report)

    def test_immutable_transfer_and_resume_proof_bind_exact_terminal_checkpoint(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);assembly=root/'assembly';stage,manifest,inventory,paths=fixture(assembly)
            cache=UsdGeom.XformCache();poses=[]
            for path in paths:
                matrix=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
                poses.append([*matrix[:3,3],0.,0.,0.,1.])
            cp=root/'previous_trial/cycle_checkpoints/000';cp.parent.mkdir(parents=True)
            save_checkpoint(cp,assembly=assembly,scene_sha256=manifest['scene_sha256'],time_s=10.,body_paths=paths,
                body_poses_world=poses,body_velocities_world=np.zeros((len(paths),6)),
                robot_dof_names=['root_joint','jaw_L','jaw_R'],robot_q=[0.,0.,0.],robot_qvel=[0.,0.,0.],
                plant_dof_names=['neck_1:rotX'],plant_q=[0.],plant_qvel=[0.],preload=[0.],
                breaks=metadata(manifest)['break_events'],manifest=manifest,
                sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
                    cycle_plans=[dict(status='motion_completed',end_s=10.,stop_reason=None)]))
            summary=dict(completed=True,stop_reason=None,initialization_valid=True,actual_self_collision_audit_passed=True,simulated_s=10.,model_sha256=manifest['scene_sha256'])
            (cp.parent.parent/'summary.json').write_text(json.dumps(summary))
            (cp.parent.parent/'states.npz').write_bytes(b'exact final native trace fixture')
            (cp.parent.parent/'self_collision_actual.json').write_text(json.dumps(dict(passed=True,complete=True,
                scene_sha256=manifest['scene_sha256'],input_sha256=file_sha256(cp.parent.parent/'states.npz'),physics_rerun=False)))
            inv=root/'inventory.json';inv.write_text(json.dumps(inventory));output=root/'revised'
            report=revise(assembly,cp,output,inv);revised=json.loads((output/'motion_manifest.json').read_text())
            native_metadata,_=load_checkpoint(cp)
            proof=validate_resume_revision(output,revised,native_metadata)
            self.assertTrue(proof['all_nonselected_properties_unchanged'])
            self.assertEqual(proof['exact_selected_joint_paths'],report['selected_joint_paths'])
            self.assertEqual(file_sha256(assembly/'scene.usda'),manifest['scene_sha256'])
            with self.assertRaises(FileExistsError):revise(assembly,cp,output,inv)
            (cp.parent.parent/'summary.json').write_text(json.dumps(dict(summary,completed=False,stop_reason='later_physics_invalid')))
            with self.assertRaisesRegex(ValueError,'parent stopped'):validate_resume_revision(output,revised,native_metadata)


if __name__=='__main__':unittest.main()
