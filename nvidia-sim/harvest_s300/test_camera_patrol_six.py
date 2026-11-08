"""UNIT FIXTURE ONLY: six-truss controller flow, never native physics proof.

No simulator, renderer, GPU or robot is started. Native boundary adapters are
injected; camera queue, dispatch, startup classification/cache and exact
selected-cycle evidence counting use the production implementations.
"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import camera_patrol_runner as runner


BASE=['--headless','--camera-harvest','--capture-gt','--whole-truss',
    '--native-materialized-stage','--verify-full-environment-preflight',
    '--verify-harvest-contact-after-step','--save-route-checkpoint','--checkpoint-cycle-states',
    '--mode','harvest','--plant-dynamics','full','--camera-scan-view','row-front']


class CameraPatrolSixTrussControlFlowTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.output=self.root/'unit_fixture_campaign'
        self.detached={22:[],23:[],24:[]};self.windows=[];self.native_calls=[];self.probes=[]
        self.checkpoint_snapshots={};self.last_checkpoint=None;self.accepted_history=[]
        self.original=self.make_assembly(self.root/'initial',22)
        self.inventory=self.root/'inventory.json'
        self.write(self.inventory,dict(unit_fixture=True,trusses=[dict(plant_id=plant,truss_index=truss,
            gutter_center_x=-.775,root_position_world_m=[-.68,77.,.3],
            fruit_centroid_world_m=[-.65,99.,66.],rail_near=True)
            for plant in (22,23,24) for truss in (range(3) if plant!=24 else range(1))]))
        for target,side_effect in [('completed_final_checkpoint',self.completed),('_measured_state',self.measured)]:
            context=patch.object(runner,target,side_effect=side_effect);context.start();self.addCleanup(context.stop)

    @staticmethod
    def write(path,value):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value))

    @staticmethod
    def source(plant):return f'/World/Greenhouse/Plants/Plant_{plant:02d}'

    def make_assembly(self,path,plant,history=()):
        path=Path(path);path.mkdir(parents=True)
        (path/'scene.usda').write_text('#UNIT FIXTURE ONLY: no USD/native execution\n')
        (path/'composed.usdc').write_bytes(('UNIT FIXTURE ONLY plant'+str(plant)).encode())
        fruits=[dict(name=f'Truss_{truss:02d}_Fruit_{fruit:02d}',
            joint=f'/World/ElasticTarget/HarvestJoints/Truss_{truss:02d}_Fruit_{fruit:02d}')
            for truss in (range(3) if plant!=24 else range(1)) for fruit in (1,2)]
        manifest=dict(unit_fixture=True,scene_sha256=hashlib.sha256((path/'scene.usda').read_bytes()).hexdigest(),
            compact_assets=dict(materialized_dependency=dict(relative_path='composed.usdc',
                sha256=hashlib.sha256((path/'composed.usdc').read_bytes()).hexdigest())),
            mounted_rgbd=dict(unit_fixture=True),rail=dict(initial_station_y_m=0.),
            plant=dict(source_plant=self.source(plant),fruits=fruits),
            route_checkpoint=dict(outgoing_history=list(history)))
        self.write(path/'motion_manifest.json',manifest);return path

    def window_switch(self,assembly,checkpoint,output,next_plant_id,target_name,**kwargs):
        # A failed unused proposal must not replace the accepted parent.
        self.assertEqual(Path(checkpoint),self.last_checkpoint)
        previous=json.loads((Path(assembly)/'motion_manifest.json').read_text())
        self.assertEqual(previous['plant']['source_plant'],self.source(22))
        self.assertTrue(kwargs['allow_frozen_reactivation']);self.assertTrue(kwargs['fast_materialization'])
        self.assertEqual(Path(kwargs['near_trusses_metadata']),self.inventory)
        snapshot=json.loads((Path(checkpoint)/'checkpoint.json').read_text())
        self.assertEqual(len(snapshot['all_plants_native_detached'][str(22)]),6)
        self.windows.append(dict(source=previous['plant']['source_plant'],next_plant=next_plant_id,
            checkpoint=str(checkpoint),target_name=target_name,source_assembly=str(assembly)))
        history=[dict(source_plant=self.source(22),unharvested_reactivation_eligible=False,
            detached_fruits=copy.deepcopy(self.detached[22]))]
        self.make_assembly(output,next_plant_id,history)
        return dict(unit_fixture=True,checkpoint=str(checkpoint),new_source_plant=self.source(next_plant_id))

    def completed(self,trial,summary):
        self.assertTrue(summary['completed']);self.assertIsNone(summary['stop_reason'])
        checkpoint=Path(trial)/'route_checkpoint';checkpoint.mkdir()
        assembly=Path(summary['fixture_assembly']);manifest=json.loads((assembly/'motion_manifest.json').read_text())
        plant=int(manifest['plant']['source_plant'].rsplit('_',1)[1])
        metadata=dict(unit_fixture=True,break_events=copy.deepcopy(self.detached[plant]),
            sequence=copy.deepcopy(summary.get('truss_sequence',{})))
        self.write(checkpoint/'checkpoint.json',dict(unit_fixture=True,source_assembly=str(assembly),
            sequence=copy.deepcopy(metadata['sequence']),break_events=copy.deepcopy(metadata['break_events']),
            all_plants_native_detached={str(key):value for key,value in self.detached.items()}))
        self.checkpoint_snapshots[str(checkpoint)]=(assembly,copy.deepcopy(metadata))
        self.last_checkpoint=checkpoint;self.accepted_history.append(checkpoint)
        return checkpoint,metadata

    def measured(self,assembly,manifest,checkpoint):
        expected,metadata=self.checkpoint_snapshots[str(checkpoint)]
        self.assertEqual(Path(assembly),expected)
        return copy.deepcopy(metadata),dict(unit_fixture=True,verified_final_physical_state=True,
            measured_initial_station_y_m=0.),0.,.2

    def observed_objects(self):
        objects=[]
        for plant in (22,23,24):
            broken={event['joint'] for event in self.detached[plant]}
            for truss in (range(3) if plant!=24 else range(1)):
                for fruit in (1,2):
                    name=f'Truss_{truss:02d}_Fruit_{fruit:02d}'
                    joint='/World/ElasticTarget/HarvestJoints/'+name
                    if joint in broken:continue
                    objects.append(dict(key=f'{self.source(plant)}/Truss_{truss:02d}/Fruit_{fruit:02d}',
                        instance_id=plant*1000+truss*10+fruit,class_name='fruit',red_proxy=True,
                        visible_pixels=40,valid_depth_pixels=35,
                        visible_surface_median_world_m=[-.6,(fruit-1.5)*.02,
                            1.9 if plant==24 else 1.0+(plant-22)*.3+truss*.1]))
        return objects

    def benchmark(self,command,check):
        self.assertTrue(check)
        assembly=Path(command[3]);cases=Path(command[4]);output=Path(command[command.index('--output')+1])
        case=json.loads(cases.read_text())[0];name=case['name'];args=case['arguments'];trial=output/name
        trial.mkdir(parents=True);manifest=json.loads((assembly/'motion_manifest.json').read_text())
        plant=int(manifest['plant']['source_plant'].rsplit('_',1)[1])
        idle=args[args.index('--mode')+1]=='idle'
        summary=dict(unit_fixture=True,mode='idle' if idle else 'harvest',completed=True,stop_reason=None,
            initialization_valid=True,rail_destination=None,simulated_s=3. if idle else 5.,
            native_new_break_count=0,fixture_assembly=str(assembly),selected_cycle_evidence=dict(cycles=[]))
        if idle:
            self.assertIn('--physics-only',args);self.assertNotIn('--capture-gt',args)
            self.probes.append(dict(plant=plant,trial=trial,parent_checkpoint=str(self.last_checkpoint)))
            if plant==24:
                summary.update(completed=False,initialization_valid=False,stop_reason='penetration_over_contact_policy')
                self.write(trial/'initialization_failure.json',dict(unit_fixture=True,reason='initialization_penetration'))
            self.write(trial/'initialization_validity.json',dict(unit_fixture=True,valid=summary['completed']))
        else:
            summary['truss_sequence']=dict(cycle_plans=[dict(index=0,purpose='camera_scan_prefix',
                harvest_attempt=False,status='motion_completed',fruits=[],start_s=0.,end_s=5.,
                planned_end_s=5.,measured_frame_index=0,end_frame_index=1)],
                termination='camera_scan_complete_patrol_decision_required')
            summary['selected_cycle_evidence']['scans']=[dict(index=0,status='motion_completed',
                harvest_attempt=False,actual_recorded_phases=['settle'])]
            before=json.loads((self.output/'camera_patrol.json').read_text())
            self.native_calls.append(dict(plant=plant,arguments=args,prior_credited=before['achieved_distinct_trusses'],
                parent_checkpoint=str(self.last_checkpoint),trial=trial))
            self.assertIn('--capture-gt',args);self.assertNotIn('--physics-only',args)
            if '--resume-native-checkpoint'in args:
                self.assertEqual(args[args.index('--resume-native-checkpoint')+1],str(self.last_checkpoint))
            if '--camera-target-truss'in args:
                truss=args[args.index('--camera-target-truss')+1]
                self.assertIn(plant,(22,23));events=[];selected=[]
                for fruit in (1,2):
                    name=f'{truss}_Fruit_{fruit:02d}';joint='/World/ElasticTarget/HarvestJoints/'+name
                    self.assertNotIn(joint,{event['joint'] for event in self.detached[plant]})
                    event=dict(joint=joint,t=2.,phase='close',native_new_event=True)
                    events.append(event);selected.append(dict(name=name,native_joint=joint,
                        native_detached_at_cycle_start=False,native_detached_at_cycle_end=True,
                        new_native_break_events_during_cycle=[event]))
                self.detached[plant].extend(events);summary['native_new_break_count']=2
                summary['truss_sequence']['cycle_plans'].append(dict(index=1,status='motion_completed',
                    fruits=[fruit['name'] for fruit in selected],truss_key=truss,start_s=0.,end_s=5.,
                    planned_end_s=5.,measured_frame_index=0,end_frame_index=1))
                summary['truss_sequence']['termination']='camera_physical_cycle_budget_reached_remaining_fruit_not_completion'
                summary['selected_cycle_evidence']['cycles']=[dict(index=1,status='motion_completed',
                    recorded_physical_cycle=True,actual_trace_measurements_available=True,
                    actual_recorded_phases=['close','cycle_open','cycle_reclose','withdraw'],selected_fruits=selected)]
            np.savez(trial/'states.npz',unit_fixture=True,time=np.array([0.,5.]),phase=np.array(['initial','close']))
            frame=trial/'rgbd/frame_00000';frame.mkdir(parents=True)
            self.write(frame/'objects.json',dict(unit_fixture=True,objects=self.observed_objects()))
            row=dict(timestamp_s=5.,physics_step=300,rgb='frame_00000/rgb.png',
                gt=dict(objects='objects.json'),render_did_not_advance_physics=True,
                world_from_camera_optical=[[0,0,-1,0],[1,0,0,0],[0,-1,0,1.],[0,0,0,1]])
            (trial/'rgbd/frames.jsonl').write_text(json.dumps(row)+'\n')
        self.write(trial/'summary.json',summary)
        native_command=['fixture-python','run_motion.py',str(assembly),'--output',str(trial)]+args
        self.write(output/(cases.stem+'_results.json'),[dict(timeout=False,returncode=0,summary=summary,command=native_command)])

    def test_six_distinct_trusses_across_two_accepted_windows_with_failed_unused_window(self):
        report=runner.camera_patrol_campaign(self.original,self.inventory,self.output,BASE,
            execute=True,goal_trusses=6,max_decisions=30,camera_foreaft_m=0.,camera_to_grasp_height_m=0.,
            benchmark_runner=self.benchmark,window_switch=self.window_switch)
        self.assertEqual(report['status'],'measured_distinct_truss_goal_reached',report.get('stop_reason'))
        self.assertTrue(report['goal_reached']);self.assertEqual(report['achieved_distinct_trusses'],6)
        expected={f'{self.source(plant)}/Truss_{truss:02d}' for plant in (22,23) for truss in range(3)}
        self.assertEqual(set(report['measured_harvested_trusses']),expected)
        self.assertEqual(sum(len(value) for value in self.detached.values()),12)
        self.assertEqual(len(self.detached[24]),0)
        harvest=[row for row in report['segments'] if row['decision']['action']=='harvest' and row['status']=='completed_physics']
        self.assertEqual(len(harvest),6)
        self.assertTrue(all(len(row['measured_harvested_trusses'])==1 for row in harvest))
        self.assertTrue(all(row['benchmark_result']['summary']['native_new_break_count']==2 for row in harvest))
        self.assertEqual({json.loads((Path(row['assembly'])/'motion_manifest.json').read_text())['plant']['source_plant']
            for row in harvest},{self.source(22),self.source(23)})
        deferred=[row for row in report['segments'] if row['status']=='unused_window_startup_deferred']
        self.assertEqual(len(deferred),1);self.assertFalse(deferred[0]['startup_preflight']['passed'])
        self.assertTrue(deferred[0]['accepted_parent_unchanged'])
        self.assertEqual([row['next_plant'] for row in self.windows],[24,23])
        self.assertEqual(self.windows[0]['checkpoint'],self.windows[1]['checkpoint'])
        self.assertEqual([probe['plant'] for probe in self.probes],[24,23])
        self.assertTrue(all(len(row['measured_harvested_trusses'])==0 for row in report['segments'] if row['decision']['action']=='scan'))
        self.assertTrue(any(call['prior_credited']==1 for call in self.native_calls))  # two fruit breaks counted as one truss
        ledger=json.loads((self.output/'camera_patrol_queue.json').read_text())
        failed_target=ledger['queue'][self.source(24)+'/Truss_00']
        self.assertEqual(failed_target['status'],'not_attempted_shared_invalid_initial_model')
        self.assertTrue(failed_target['blockers'])  # exact tested target evidence is preserved
        self.assertIn(self.source(24),ledger['plant_quarantines'])
        self.assertFalse(ledger['plant_quarantines'][self.source(24)]['impossible'])
        self.assertEqual(len(ledger['native_detached_keys']),12)
        self.assertFalse(report['training_eligible']);self.assertIsNone(report['harvest_success'])
        self.assertTrue(all(row['unit_fixture'] for row in [json.loads((Path(record['trial'])/'summary.json').read_text())
            for record in report['segments'] if record.get('trial')]))


if __name__=='__main__':unittest.main(verbosity=2)
