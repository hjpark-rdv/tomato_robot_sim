import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from camera_prefix_connection import (connection_blocks, splice_camera_connection,
    lift_then_arm_path, validate_checkpoint_provenance, connect_camera_prefix,
    connect_external_camera_prefix, verify_rail_holds, proposal_steps, fine_path_check,
    reuse_verified_camera_audits, required_contact_policy, _sha)


def fixture(lift=True):
    a=np.array([.02,.1,.2,.3,.4,.5,.6]); raised=a.copy();raised[0]=.17
    b=raised+np.array([0.,.2,.1,-.1,.2,-.2,.1])
    def phase(name,waypoints,duration=1.,gap=.096,belt=None):
        q=np.asarray(waypoints)
        return dict(name=name,waypoint_q=q.tolist(),duration_s=duration,end_gap_m=gap,
            belt_local_y_m_s=[0.,0.] if belt is None else belt,
            rail_waypoint_positions_m=[-1.4569]*len(q),
            rail_native_command_positions_m=[-1.4571]*len(q))
    forward=[phase('prepare_joint_branch',[raised if lift else a,b],6.)]
    backward=[phase('return_joint_branch',[b,raised if lift else a],6.)]
    if lift:
        forward.insert(0,phase('prepare_lift_branch',[a,(a+raised)/2,raised],4.))
        backward.append(phase('return_lift_branch',[raised,(a+raised)/2,a],4.))
    prefix=[phase('ready_hold',[a,a],.5,.044),phase('prepare_open',[a,a],1.),
        phase('prepare_open_hold',[a,a],.5)]
    middle=[phase('approach',[b,b+.001],4.),phase('belt_start_open',[b+.001,b+.001],1.,belt=[.1,-.1]),
        phase('close',[b+.001,b+.001],2.,.028,[.1,-.1]),
        phase('cycle_open',[b+.001,b+.001],1.,.096,[.1,-.1]),phase('withdraw',[b+.001,b],4.)]
    phases=prefix+forward+middle+backward+[phase('ready_close',[a,a],1.,.044)]
    clock=sum(p['duration_s']for p in phases[:next(i for i,p in enumerate(phases)if p['name']=='belt_start_open')])
    return dict(start=a.tolist(),phases=phases,ready_lift_branch={'legacy':'metadata'},
        joint_ready_connector={'legacy':'metadata'},target_center_world_m=[1.,2.,3.],
        preparation_opening=dict(location='ready',required_gap_m=.096,tracking_tolerance_m=.0005,
            requested_transfer_gap_m=.096,effective_transfer_gap_m=.096,
            feedback_before_phases=(['prepare_lift_branch']if lift else [])+['prepare_joint_branch','approach']),
        belt_cycle=dict(phases=['belt_start_open','close','cycle_open'],start_after_settle_s=clock,duration_s=4.))


def certificate_fixture(root):
    root=Path(root);model=root/'model';model.mkdir();cp=root/'checkpoint';cp.mkdir()
    for name in ('scene.usda','composed.usdc','motion_manifest.json'):(model/name).write_text(name)
    (cp/'checkpoint.json').write_text('{"metadata_sha256":"canonical"}')
    (cp/'checkpoint.npz').write_bytes(b'exact measured state')
    cache=root/'native_cache.json';cache.write_text('exact32sourceboundpieces')
    original=root/'source_plan.json';source=fixture();original.write_text(json.dumps(source))
    b=connection_blocks(source);plan=splice_camera_connection(source,lift_then_arm_path(b['start'],b['goal']))
    folder=root/'connection';folder.mkdir();connected=folder/'connected_plan.json';connected.write_text(json.dumps(plan))
    code=folder/'code';code.mkdir();(code/'camera_prefix_connection.py').write_text('verified helper snapshot')
    names=[p['name']for p in plan['phases']];phases=[dict(phase=name,clear=True)for name in names]
    self_report=dict(passed=True,complete=True,hz=60,samples=777,phases=phases,
        scene_sha256=_sha(model/'scene.usda'),input_sha256=_sha(connected),physics_rerun=False,
        geometry=dict(clearance_m=.002))
    environment=dict(schema='s300-rail-environment-preflight-v1',passed=True,complete=True,
        eligible_for_native_execution=True,native_hz=60,requested_hz=60,strict_native_period_sampling=True,
        early_terminated=False,hard_intersection_sample_pairs=0,selected_phases=names,samples=777,
        allowed_truss_key='Truss_00',geometry=dict(wheel_floor_overlap_limit_m=.001,
            released_fruit_collection_diagnostic=dict(collection_enabled=True,
                gripper_support_enabled=True,same_truss_enabled=True)),
        sampling=dict(passed=True,complete=True,hz=60,samples=777,phases=phases),
        checkpoint_proof=dict(checkpoint=str(cp),metadata_sha256='canonical',
            checkpoint_json_sha256=_sha(cp/'checkpoint.json'),npz_sha256=_sha(cp/'checkpoint.npz')),
        native_tool_cache=dict(path=str(cache),sha256=_sha(cache),source_model_bound=True),
        source_sha256=dict(plan=_sha(connected),scene=_sha(model/'scene.usda'),
            manifest=_sha(model/'motion_manifest.json'),materialized_dependency=_sha(model/'composed.usdc')))
    for name,value in [('full_self60.json',self_report),('full_environment60.json',environment)]:
        (folder/name).write_text(json.dumps(value))
    report=dict(schema='s300-resumed-camera-prefix-connection-v1',status='full_audits_passed',
        eligible_for_native_execution=True,source_files_unchanged=True,full_hz=60,
        verified_plan=str(connected),verified_plan_sha256=_sha(connected),
        source_checkpoint=str(cp),source_assembly=str(model),source_plan=str(original),
        native_tool_cache=dict(path=str(cache),sha256=_sha(cache)),
        source_file_sha256={str(p):_sha(p)for p in [model/'scene.usda',model/'composed.usdc',
            model/'motion_manifest.json',cp/'checkpoint.json',cp/'checkpoint.npz',cache,original]},
        code_sha256=_sha(code/'camera_prefix_connection.py'),
        audit_file_sha256={name:_sha(folder/name)for name in ['full_self60.json','full_environment60.json']},
        full_self=dict(passed=True,complete=True,samples=777),full_environment=dict(passed=True,complete=True),
        checkpoint_provenance=dict(checkpoint_metadata_sha256='canonical'))
    report['contact_policy']=required_contact_policy('Truss_00')
    (folder/'report.json').write_text(json.dumps(report))
    return connected,cp,cache


class CameraPrefixConnectionTests(unittest.TestCase):
    def test_optional_lift_and_exact_reverse_are_replaced_preserving_full_manipulation(self):
        for lift in (False,True):
            with self.subTest(lift=lift):
                source=fixture(lift);before=copy.deepcopy(source);block=connection_blocks(source)
                path=lift_then_arm_path(block['start'],block['goal'])
                out=splice_camera_connection(source,path)
                self.assertEqual(source,before)
                original=[p for p in source['phases']if p['name']not in
                    ('prepare_lift_branch','prepare_joint_branch','return_joint_branch','return_lift_branch')]
                unchanged=[p for p in out['phases']if not p['name'].startswith(('prepare_camera_ompl_','return_camera_ompl_'))]
                self.assertEqual(original,unchanged)
                self.assertEqual(out['target_center_world_m'],source['target_center_world_m'])
                fw=[p for p in out['phases']if p['name'].startswith('prepare_camera_ompl_')]
                rw=[p for p in out['phases']if p['name'].startswith('return_camera_ompl_')]
                for a,b in zip(fw,rw[::-1]):
                    self.assertEqual(a['waypoint_q'],b['waypoint_q'][::-1])
                    self.assertEqual(a['duration_s'],b['duration_s'])
                    self.assertEqual(a['belt_local_y_m_s'],[0.,0.])
                    self.assertEqual(a['rail_waypoint_positions_m'],[-1.4569,-1.4569])
                    self.assertEqual(a['rail_native_command_positions_m'],[-1.4571,-1.4571])
                first=next(i for i,p in enumerate(out['phases'])if p['name']=='belt_start_open')
                self.assertAlmostEqual(out['belt_cycle']['start_after_settle_s'],sum(p['duration_s']for p in out['phases'][:first]))
                self.assertEqual(out['belt_cycle']['duration_s'],4.)

    def test_carriage_first_holds_all_six_actual_arm_angles_and_endpoints(self):
        source=fixture();block=connection_blocks(source);a=block['start'].copy();b=block['goal'].copy()
        path=lift_then_arm_path(a,b)
        np.testing.assert_array_equal(path[0],a);np.testing.assert_array_equal(path[-1],b)
        np.testing.assert_array_equal(path[1,1:],a[1:]);self.assertEqual(path[1,0],b[0])
        out=splice_camera_connection(source,path)
        caps=np.array([.08]+[.35]*6)
        for p in out['phases']:
            if p['name'].startswith(('prepare_camera_ompl_','return_camera_ompl_')):
                q=np.asarray(p['waypoint_q']);self.assertTrue(np.all(1.5*np.abs(q[1]-q[0])/p['duration_s']<=caps+1e-10))

    def test_new_first_and_each_connector_leg_requires_original_measured_open_guard(self):
        source=fixture();source['preparation_opening']['feedback_before_phases'].append('prepare_open_hold')
        original=copy.deepcopy(source['preparation_opening']);block=connection_blocks(source)
        revised=splice_camera_connection(source,lift_then_arm_path(block['start'],block['goal']))
        opening=revised['preparation_opening'];feedback=opening['feedback_before_phases']
        names=[p['name']for p in revised['phases']if p['name'].startswith('prepare_camera_ompl_')]
        self.assertTrue(names);self.assertTrue(set(names)<=set(feedback))
        first_move=next(p for p in revised['phases']if not np.array_equal(p['waypoint_q'][0],p['waypoint_q'][-1]))
        self.assertEqual(first_move['name'],names[0]);self.assertIn(first_move['name'],feedback)
        self.assertIn('approach',feedback);self.assertIn('prepare_open_hold',feedback)
        self.assertNotIn('prepare_lift_branch',feedback);self.assertNotIn('prepare_joint_branch',feedback)
        self.assertEqual({k:v for k,v in opening.items()if k!='feedback_before_phases'},
            {k:v for k,v in original.items()if k!='feedback_before_phases'})
        self.assertEqual(source['preparation_opening'],original)
        for edit in (lambda p:p.pop('preparation_opening'),
                     lambda p:p['preparation_opening'].update(required_gap_m=.044)):
            bad=copy.deepcopy(source);edit(bad)
            with self.assertRaisesRegex(ValueError,'measured96mm opening policy'):
                splice_camera_connection(bad,lift_then_arm_path(block['start'],block['goal']))

    def test_rejects_invalid_rail_reverse_belts_missing_lift_and_changed_endpoints(self):
        for edit in (
            lambda p:p['phases'][3]['rail_waypoint_positions_m'].__setitem__(1,-1.4),
            lambda p:p['phases'][3]['rail_native_command_positions_m'].__setitem__(1,-1.4),
            lambda p:p['phases'][3].update(belt_local_y_m_s=[.1,0.]),
            lambda p:p['phases'][3]['waypoint_q'][1].__setitem__(2,float('nan')),
            lambda p:p['phases'][-2]['waypoint_q'][-1].__setitem__(1,.5),
            lambda p:p['phases'][-2].update(name='not_return_lift')):
            plan=fixture();edit(plan)
            with self.assertRaises(ValueError):connection_blocks(plan)
        plan=fixture();block=connection_blocks(plan)
        for index in (0,-1):
            path=lift_then_arm_path(block['start'],block['goal']);path[index,0]+=.001
            with self.assertRaisesRegex(ValueError,'exact measured source and staging'):
                splice_camera_connection(plan,path)
        with self.assertRaises(ValueError):splice_camera_connection(plan,[block['start'],block['goal']],hz=120)

    def test_retimed_belt_clock_rejects_bad_source_without_fixing_it(self):
        plan=fixture();b=connection_blocks(plan);plan['belt_cycle']['start_after_settle_s']+=.1
        with self.assertRaisesRegex(ValueError,'Retimed belt clock'):
            splice_camera_connection(plan,[b['start'],b['goal']])

    def test_rail_measured_coordinate_and_native_target_are_not_interchangeable(self):
        plan=fixture();origin=dict(displacement_m=-1.4569,native_position_target_m=-1.4571,
            checkpoint_metadata_sha256='actual')
        proof=verify_rail_holds(plan,origin)
        self.assertFalse(proof['requested_new_rail_travel'])
        self.assertEqual(proof['checkpoint_metadata_sha256'],'actual')
        for field in ('displacement_m','native_position_target_m'):
            bad=copy.deepcopy(origin);bad[field]+=.0001
            with self.assertRaisesRegex(ValueError,'measured geometry and retained native command'):
                verify_rail_holds(plan,bad)

    def test_exact_completed_same_model_checkpoint_and_measured_not_drive_start_required(self):
        plan=fixture();names=['lift','base','shoulder','elbow','wrist1','wrist2','wrist3']
        manifest={'scene_sha256':'scene','plant':{'source_plant':'Plant24'},'compact_assets':{'materialized_dependency':{'sha256':'usdc'}}}
        metadata=dict(source_assembly='/tmp/model',scene_sha256='scene',manifest=copy.deepcopy(manifest),
            source_plant='Plant24',metadata_sha256='canonical',checkpoint_npz_sha256='npz',time_s=8.5,
            sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
                cycle_plans=[dict(status='motion_completed',stop_reason=None,end_s=8.5)]))
        arrays=dict(robot_dof_names=np.array(names+['root_joint']),robot_q=np.r_[plan['start'],-1.4569])
        result=validate_checkpoint_provenance('/tmp/model',manifest,metadata,arrays,plan,names)
        self.assertEqual(result['checkpoint_metadata_sha256'],'canonical')
        for edit in (
            lambda m:m.update(source_assembly='/tmp/oldmodel'),
            lambda m:m['manifest']['compact_assets']['materialized_dependency'].update(sha256='other'),
            lambda m:m['sequence'].update(checkpoint_physics_valid=False),
            lambda m:m['sequence'].update(checkpoint_capture_reason='guard_stop')):
            bad=copy.deepcopy(metadata);edit(bad)
            with self.assertRaises(ValueError):validate_checkpoint_provenance('/tmp/model',manifest,bad,arrays,plan,names)
        changed=copy.deepcopy(plan);changed['start'][0]+=.008315
        with self.assertRaisesRegex(ValueError,'exact measured checkpoint q'):
            validate_checkpoint_provenance('/tmp/model',manifest,metadata,arrays,changed,names)

    def test_invalid_budgets_cannot_start_queries_or_create_outputs(self):
        with tempfile.TemporaryDirectory()as tmp:
            for options in ({'solve_seconds':31},{'solve_seconds':float('nan')},{'hz':120},
                            {'seed':2029},{'proposal_resolution':'unsafe'},
                            {'strategy':'teleport'},{'strategy':'lift-then-arm','joint_path':'x'}):
                out=Path(tmp)/'output'
                with self.assertRaises(ValueError):
                    connect_camera_prefix('missing','missing','missing',out,
                        allowed_truss_key='Truss_00',native_tool_cache='missing',**options)
                self.assertFalse(out.exists())

    def test_coarse_proposal_is_not_a_fine_certificate_and_uses_separate_finite_budget(self):
        from connect_s300_prefix import JointSpace,check_polyline
        bounds=np.array([[0.]+[-1.]*6,[.75]+[1.]*6]);a=np.zeros(7);b=a.copy();b[0]=.006
        path=np.array([a,b]);blocked=lambda q:not np.isclose(q[0],.002,rtol=0.,atol=1e-12)
        # Deliberately thin collision between coarse endpoints is skipped by
        # proposal samples and MUST be rejected by the independent fine pass.
        coarse=check_polyline(path,JointSpace(bounds,proposal_steps('coarse')),blocked)
        self.assertTrue(coarse['passed'])
        fine=fine_path_check(path,bounds,blocked)
        self.assertFalse(fine['passed']);self.assertFalse(fine['complete'])
        self.assertEqual(fine['q'][0],.002)
        self.assertEqual(fine['budget_seconds'],120.);self.assertEqual(fine['max_state_checks'],10000)
        self.assertEqual(fine['state_checks'],2)
        np.testing.assert_array_equal(fine['steps_m_or_rad'],proposal_steps('fine'))
        self.assertTrue(fine['proposal_resolution_is_not_certificate'])
        clear=fine_path_check(path,bounds,lambda q:True)
        self.assertTrue(clear['passed']);self.assertTrue(clear['complete']);self.assertEqual(clear['state_checks'],4)
        import inspect
        # Public defaults retain fine proposal resolution and original seed.
        signature=inspect.signature(connect_camera_prefix)
        self.assertEqual(signature.parameters['proposal_resolution'].default,'fine')
        self.assertEqual(signature.parameters['seed'].default,2026)

    def test_native_wrapper_uses_asset_python_retains_log_and_requires_full_bound_plan(self):
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'connected'
            def process(command,**kwargs):
                self.assertTrue(command[0].endswith('/nvidia-sim/.asset-venv/bin/python'))
                self.assertIn('--strategy',command);self.assertIn('lift-then-arm',command)
                self.assertNotIn('PYTHONPATH',kwargs['env'])
                out.mkdir();plan=out/'connected_plan.json';plan.write_text('{}')
                (out/'report.json').write_text(json.dumps(dict(status='full_audits_passed',
                    source_files_unchanged=True,eligible_for_native_execution=True,
                    verified_plan=str(plan),verified_plan_sha256=_sha(plan))))
                return subprocess.CompletedProcess(command,0,'actual asset output\n','stderr retained\n')
            with patch('camera_prefix_connection.subprocess.run',side_effect=process):
                result=connect_external_camera_prefix('assembly','plan','checkpoint',out,
                    allowed_truss_key='Truss_00',native_tool_cache='cache',strategy='lift-then-arm')
            self.assertEqual(result,out/'connected_plan.json')
            self.assertIn('stderr retained',out.with_suffix('.log').read_text())
            with self.assertRaises(FileExistsError):
                connect_external_camera_prefix('a','b','c',out,allowed_truss_key='T',native_tool_cache='c')

    def test_native_wrapper_never_accepts_incomplete_failure_or_sha_tampering(self):
        for report in (
            dict(status='finite_search_no_path_not_impossible'),
            dict(status='full_audits_passed',eligible_for_native_execution=False,source_files_unchanged=True),
            dict(status='full_audits_passed',eligible_for_native_execution=True,source_files_unchanged=False),
            dict(status='full_audits_passed',eligible_for_native_execution=True,source_files_unchanged=True,verified_plan_sha256='wrong')):
            with tempfile.TemporaryDirectory()as tmp:
                out=Path(tmp)/'connected'
                def process(command,**kwargs):
                    out.mkdir();plan=out/'connected_plan.json';plan.write_text('{}')
                    (out/'report.json').write_text(json.dumps(dict(report,verified_plan=str(plan))))
                    return subprocess.CompletedProcess(command,0,'','')
                with patch('camera_prefix_connection.subprocess.run',side_effect=process):
                    with self.assertRaises(ValueError):connect_external_camera_prefix('a','b','c',out,
                        allowed_truss_key='Truss_00',native_tool_cache='cache')

    def test_complete_bound_audit_reuse_copies_exact_original_bytes_without_new_queries(self):
        with tempfile.TemporaryDirectory()as tmp:
            connected,cp,cache=certificate_fixture(tmp);folder=connected.parent
            destself=Path(tmp)/'run/self.json';destenv=Path(tmp)/'run/env.json'
            proof=reuse_verified_camera_audits(connected,destself,destenv,checkpoint=cp,native_tool_cache=cache)
            self.assertEqual(destself.read_bytes(),(folder/'full_self60.json').read_bytes())
            self.assertEqual(destenv.read_bytes(),(folder/'full_environment60.json').read_bytes())
            self.assertTrue(proof['audit_bytes_identical']);self.assertEqual(proof['new_FCL_queries'],0)
            self.assertEqual(proof['new_physics_steps'],0)
            old=destself.read_bytes()
            with self.assertRaises(ValueError):reuse_verified_camera_audits(connected,destself,destenv,
                checkpoint=cp,native_tool_cache=cache)
            self.assertEqual(old,destself.read_bytes())

    def test_reuse_rejects_stale_source_audit_sha_or_different_checkpoint_or_cache(self):
        for kind in ('source','audit','checkpoint','cache','missing'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory()as tmp:
                connected,cp,cache=certificate_fixture(tmp);folder=connected.parent
                if kind=='source':(cp/'checkpoint.npz').write_bytes(b'changed state')
                if kind=='audit':(folder/'full_self60.json').write_text('{}')
                if kind=='checkpoint':cp=Path(tmp)/'different_checkpoint'
                if kind=='cache':cache=Path(tmp)/'different_cache'
                if kind=='missing':(folder/'full_environment60.json').unlink()
                with self.assertRaises(ValueError):reuse_verified_camera_audits(connected,
                    Path(tmp)/'out/self',Path(tmp)/'out/env',checkpoint=cp,native_tool_cache=cache)
                self.assertFalse((Path(tmp)/'out/self').exists())

    def test_reuse_rejects_incomplete_or_non60_certificate_even_when_file_hash_matches(self):
        for edit in (
            lambda x:x.update(complete=False),lambda x:x.update(eligible_for_native_execution=False),
            lambda x:x.update(native_hz=120),lambda x:x.update(strict_native_period_sampling=False),
            lambda x:x.update(selected_phases=['approach']),lambda x:x.update(hard_intersection_sample_pairs=1),
            lambda x:x['geometry'].update(wheel_floor_overlap_limit_m=0.),
            lambda x:x['geometry']['released_fruit_collection_diagnostic'].update(same_truss_enabled=False)):
            with tempfile.TemporaryDirectory()as tmp:
                connected,cp,cache=certificate_fixture(tmp);folder=connected.parent
                ef=folder/'full_environment60.json';environment=json.loads(ef.read_text());edit(environment)
                ef.write_text(json.dumps(environment));rf=folder/'report.json';report=json.loads(rf.read_text())
                report['audit_file_sha256']['full_environment60.json']=_sha(ef);rf.write_text(json.dumps(report))
                with self.assertRaises(ValueError):reuse_verified_camera_audits(connected,
                    Path(tmp)/'out/self',Path(tmp)/'out/env',checkpoint=cp,native_tool_cache=cache)

    def test_reuse_rejects_missing_forward_open_guard_even_if_plan_and_audit_hashes_match(self):
        with tempfile.TemporaryDirectory()as tmp:
            connected,cp,cache=certificate_fixture(tmp);folder=connected.parent
            plan=json.loads(connected.read_text());plan['preparation_opening']['feedback_before_phases']=['approach']
            connected.write_text(json.dumps(plan));rf=folder/'report.json';r=json.loads(rf.read_text())
            r['verified_plan_sha256']=_sha(connected);rf.write_text(json.dumps(r))
            with self.assertRaisesRegex(ValueError,'full-open guard'):reuse_verified_camera_audits(connected,
                Path(tmp)/'out/self',Path(tmp)/'out/env',checkpoint=cp,native_tool_cache=cache)


if __name__=='__main__':unittest.main()
