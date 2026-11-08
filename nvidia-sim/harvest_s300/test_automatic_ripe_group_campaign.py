"""The opt-in row campaign reuses exact per-window GT color policies."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pxr import Usd,UsdGeom,Gf,Vt
from harvest_group_policy import load_policy
from run_row_harvest import (automatic_ripe_group_arguments,
    validate_automatic_ripe_group_arguments,campaign)
from test_run_row_harvest import RunRowHarvestTests


class AutomaticRipeGroupCampaignTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.arguments=['--whole-truss','--truss-order','highest-first',
            '--approach-yaw-mode','truss-axis','--single-fruit-when-remaining','0',
            '--physics-only','--initial-truss-first']

    def tearDown(self):self.tmp.cleanup()

    def assembly(self,label,source):
        folder=self.root/label;folder.mkdir();stage=Usd.Stage.CreateNew(str(folder/'composed.usdc'))
        fruits=[]
        for i,color in [(1,(.8,.02,.01)),(2,(.02,.7,.01))]:
            name='Truss_02_Fruit_'+f'{i:02d}';path='/World/ElasticTarget/'+name
            UsdGeom.Xform.Define(stage,path)
            for sub in ['Collider','Visual_0']:
                mesh=UsdGeom.Mesh.Define(stage,path+'/'+sub)
                mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
            fruits.append(dict(name=name,path=path,joint='/World/ElasticTarget/HarvestJoints/'+name))
        stage.GetRootLayer().Save();manifest=dict(plant=dict(source_plant=source,root='/World/ElasticTarget',fruits=fruits))
        (folder/'motion_manifest.json').write_text(json.dumps(manifest));return folder,fruits

    def test_real_extractor_and_loader_bind_each_actual_window_and_remove_legacy_priority(self):
        before=self.arguments.copy();first,fruits=self.assembly('first','/World/Greenhouse/Plants/Plant_20')
        second,_=self.assembly('second','/World/Greenhouse/Plants/Plant_22')
        first_path=self.root/'policies/first.json';second_path=self.root/'policies/second.json'
        args,proof=automatic_ripe_group_arguments(first,first_path,self.arguments)
        args2,proof2=automatic_ripe_group_arguments(second,second_path,self.arguments)
        self.assertEqual(self.arguments,before);self.assertNotIn('--initial-truss-first',args)
        self.assertTrue(proof['initial_truss_priority_removed']);self.assertEqual(proof['minimum_fruits'],2)
        self.assertEqual(proof['eligible_names'],['Truss_02_Fruit_01'])
        self.assertNotEqual(proof['source_sha256']['motion_manifest.json'],proof2['source_sha256']['motion_manifest.json'])
        self.assertIn(str(first_path),args);self.assertIn(str(second_path),args2)
        self.assertFalse(any(t.startswith('--capture')or t=='--camera-harvest' for t in args))
        self.assertFalse(proof['training_eligible'])
        with self.assertRaisesRegex(ValueError,'hash mismatch'):load_policy(first_path,second,fruits)
        with self.assertRaises(FileExistsError):automatic_ripe_group_arguments(first,first_path,self.arguments)

    def test_explicit_camera_policy_and_legacy_individual_modes_fail_before_extraction(self):
        for extra in [['--camera-harvest'],['--harvest-group-policy','/old/policy.json'],
                      ['--harvest-group-policy=/old/policy.json'],['--single-fruit-when-remaining','1'],
                      ['--split-unproductive-pairs']]:
            with patch('prepare_harvest_group_policy.prepare')as prepare:
                with self.assertRaises(ValueError):automatic_ripe_group_arguments('/no/model','/no/policy',self.arguments+extra)
                prepare.assert_not_called()

    def test_campaign_default_never_extracts_policy_or_changes_default_report(self):
        fixture=RunRowHarvestTests();fixture.setUp()
        try:
            with patch('run_row_harvest.verify_compact_dependencies',return_value={}), \
                    patch('run_row_harvest.automatic_ripe_group_arguments')as auto:
                report=campaign(fixture.assembly,fixture.inventory_path,fixture.screen_path,
                    fixture.root/'default_unchanged',execute=False)
                auto.assert_not_called();self.assertNotIn('automatic_ripe_group_policy',report)
        finally:fixture.tearDown()

    def test_campaign_optin_records_teacher_semantics_without_native_or_camera(self):
        fixture=RunRowHarvestTests();fixture.setUp()
        try:
            args=self.arguments+['--headless','--native-materialized-stage','--save-route-checkpoint',
                '--checkpoint-cycle-states','--mode','harvest','--plant-dynamics','full']
            with patch('run_row_harvest.verify_compact_dependencies',return_value={}), \
                    patch('run_row_harvest.automatic_ripe_group_arguments')as auto:
                report=campaign(fixture.assembly,fixture.inventory_path,fixture.screen_path,
                    fixture.root/'optin_plan_only',execute=False,base_arguments=args,auto_ripe_group_policy=True)
                auto.assert_not_called();self.assertTrue(report['automatic_ripe_group_policy']['enabled'])
                self.assertFalse(report['automatic_ripe_group_policy']['camera_selected'])
                self.assertFalse(report['camera_capture']['enabled'])
        finally:fixture.tearDown()


if __name__=='__main__':unittest.main()
