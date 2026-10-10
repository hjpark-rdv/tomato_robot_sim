"""A short failed trial must not appear as a fast completed simulation."""
import json
import tempfile
import unittest
from pathlib import Path

from report_performance import generate


class ReportTests(unittest.TestCase):
    def fixture(self, root, name, completed, separation=.0001, envs=1):
        path = root/name
        path.mkdir()
        summary = dict(completed=completed, stop_reason=None if completed else 'penetration_over_0_5mm',
                       physics_wall_s=2., save_wall_s=.5, simulated_s=.01 if not completed else 1.,
                       num_envs=envs, fixture_only=True, min_contact_separation_m=separation,
                       training_eligible=False, harvest_success=None, model_sha256='abcd')
        (path/'summary.json').write_text(json.dumps(summary))
        (path/'runtime_settings.json').write_text(json.dumps(dict(device='cpu',plant_dynamics='full',position_iterations=64,
             stage_metrics=dict(meters_per_unit=1.,up_axis='Z',kilograms_per_unit=1.))))
        (path/'native_metadata.json').write_text(json.dumps(dict(plant_dofs=['a','b'],robot_dofs=['c'],body_paths=['/body'])))
        return path

    def test_complete_and_invalid_attempts_preserved_with_separate_timing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root, 'complete', True)
            self.fixture(root, 'failed', False, -.01)
            (root/'cases_results.json').write_text(json.dumps([
                dict(name='complete',total_wall_s=9.,returncode=0,timeout=False),
                dict(name='failed',total_wall_s=9.,returncode=0,timeout=False)]))
            report = generate(root)
            rows = {row['name']:row for row in report['trials']}
            self.assertEqual(rows['complete']['accepted_aggregate_sim_s_per_wall_s'], .5)
            self.assertEqual(rows['complete']['startup_and_other_s'], 6.5)
            self.assertIsNone(rows['failed']['accepted_aggregate_sim_s_per_wall_s'])
            self.assertIn('recorded_penetration_over_0_5mm',rows['failed']['timing_exclusion_reasons'])
            self.assertTrue(rows['complete']['fixture_only'])
            self.assertIn('failed', (root/'index.html').read_text())
            self.assertEqual(len(generate(root)['trials']),2)

    def test_real_pulse_null_belt_summary_and_loaded_ball_transport(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            pulse=self.fixture(root,'pulse',True)
            loaded=self.fixture(root,'loaded',True)
            for folder,mode,control in ((pulse,'pulse',None),
                                        (loaded,'belt_control',dict(transport_m=.04056))):
                summary=json.loads((folder/'summary.json').read_text())
                summary.update(mode=mode,belt_control=control)
                (folder/'summary.json').write_text(json.dumps(summary))
            rows={row['name']:row for row in generate(root)['trials']}
            self.assertIsNone(rows['pulse']['synthetic_sphere_transport_mm'])
            self.assertFalse(rows['pulse']['synthetic_belt_control'])
            self.assertAlmostEqual(rows['loaded']['synthetic_sphere_transport_mm'],40.56)
            self.assertTrue(rows['loaded']['synthetic_belt_control'])

    def test_cross_environment_and_group_warnings_exclude_clone_timing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = self.fixture(root,'clones',True,envs=2)
            (folder/'contacts.json').write_text(json.dumps([
                dict(a='/World/MotionEnvs/env_0/body',b='/World/MotionEnvs/env_1/body'),
                dict(a='/World/MotionEnvs/env_0/body',b='/World/gutter')]))
            (root/'clones.log').write_text('[Warning] [omni.physx.plugin] Collisions are supported currently only in one collision group.\n')
            row = generate(root)['trials'][0]
            self.assertEqual(row['cross_environment_evidence']['cross_environment_point_records'],1)
            self.assertEqual(row['warnings']['collision_group_warnings'],1)
            self.assertIsNone(row['accepted_aggregate_sim_s_per_wall_s'])

    def test_does_not_overwrite_existing_unrelated_report(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'index.html').write_text('existing report')
            with self.assertRaises(FileExistsError):
                generate(root)
            self.assertEqual((root/'index.html').read_text(),'existing report')

    def test_native_crash_before_physics_is_finished_failure_without_speed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'gpu_bootstrap',False)
            (folder/'summary.json').unlink()
            (folder/'runtime_settings.json').unlink()
            (folder/'native_metadata.json').unlink()
            (folder/'native_crash_python_stack.txt').write_text('app.update before physics initialization')
            (root/'cases_results.json').write_text(json.dumps([dict(name='gpu_bootstrap',returncode=-11)]))
            row=generate(root)['trials'][0]
            self.assertEqual(row['status'],'runtime_nonzero_exit')
            self.assertIsNone(row['accepted_aggregate_sim_s_per_wall_s'])
            self.assertIn('nonzero_process_exit',row['timing_exclusion_reasons'])
            self.assertIn('native_crash_python_stack.txt',row['files'])

    def test_completed_gpu_capacity_error_is_not_valid_speed_or_collision_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'gpu_capacity',True,envs=2)
            original=(folder/'summary.json').read_bytes()
            (root/'gpu_capacity.log').write_text('[Error] [omni.physx.plugin] foundLostAggregatePairsCapacity needs 66371 vs 16384; simulation will miss interactions\n')
            row=generate(root)['trials'][0]
            self.assertTrue(row['completed'])
            self.assertTrue(row['warnings']['missed_interactions_error'])
            self.assertEqual(row['status'],'completed_but_native_interactions_lost')
            self.assertFalse(row['timing_comparable'])
            self.assertIn('native_interaction_capacity_exceeded',row['timing_exclusion_reasons'])
            self.assertIsNone(row['accepted_aggregate_sim_s_per_wall_s'])
            self.assertEqual((folder/'summary.json').read_bytes(),original)

    def test_missing_known_reference_contacts_excludes_speed_without_zero_contact_universal_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'readback_lost',True)
            untouched=self.fixture(root,'legitimate_no_contact',True)
            original=(folder/'summary.json').read_bytes()
            (root/'readback_comparison.json').write_text(json.dumps(dict(candidate=str(folder),passed=False,
                environment_results=[dict(environment=0,contacts=dict(reference_points=5792,actual_points=0))])))
            rows={row['name']:row for row in generate(root)['trials']}
            self.assertFalse(rows['readback_lost']['timing_comparable'])
            self.assertIsNone(rows['readback_lost']['accepted_aggregate_sim_s_per_wall_s'])
            self.assertEqual(rows['readback_lost']['status'],'completed_but_native_contact_evidence_missing')
            self.assertTrue(rows['legitimate_no_contact']['timing_comparable'])
            self.assertEqual((folder/'summary.json').read_bytes(),original)

    def test_legacy_warmup_penetration_invalidates_completed_trial_without_rewriting_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = self.fixture(root,'old_loaded',True)
            original = (folder/'summary.json').read_bytes()
            (folder/'contacts.json').write_text(json.dumps([
                dict(t=0.,phase='idle',a='/World/ActuatedTool/L_belt',b='/World/fixture',separation_m=-.0017,force_n=65.),
                dict(t=.1,phase='belt',a='/World/ActuatedTool/L_belt',b='/World/fixture',separation_m=.0001,force_n=.2)]))
            row = generate(root)['trials'][0]
            self.assertFalse(row['initialization_evidence']['valid'])
            self.assertEqual(row['initialization_evidence']['violation_points'],1)
            self.assertEqual(row['initialization_evidence']['maximum_active_point_force_n'],65.)
            self.assertIsNone(row['accepted_aggregate_sim_s_per_wall_s'])
            self.assertEqual((folder/'summary.json').read_bytes(),original)

    def test_common_time_diagnostic_is_not_displayed_as_physics_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.fixture(root,'different_hz',True)
            (root/'hz_comparison.json').write_text(json.dumps(dict(diagnostic_only=True,passed=None,
                                                                  interpretation='Different physical timestep')))
            (root/'hz_displacement.png').write_bytes(b'plot fixture')
            report=generate(root);page=(root/'index.html').read_text()
            self.assertTrue(report['comparisons'][0]['diagnostic_only'])
            self.assertIn('동등성 판정 없음',page);self.assertIn('src="hz_displacement.png"',page)

    def test_comparison_with_null_description_does_not_break_refresh(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.fixture(root,'completed',True)
            (root/'native_equivalence.json').write_text(json.dumps(dict(passed=True,interpretation=None)))
            report=generate(root)
            self.assertTrue(report['comparisons'][0]['passed'])
            self.assertIn('native_equivalence.json',(root/'index.html').read_text())

    def test_worker_report_cannot_claim_pass_when_comparison_row_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.fixture(root,'completed',True)
            entry=dict(completed=True,native_break_events=[],comparison=dict(passed=True,
                environment_results=[dict(states=dict(poses_exact=True,joint_coordinates=dict(exact=True),
                    commands_exact=True),contacts=dict(exact_order_independent=True))]))
            (root/'workers_equivalence.json').write_text(json.dumps(dict(
                passed=True,workers=2,scope='Matched individual SI references',rows=[entry])))
            result=generate(root)['comparisons'][0]
            self.assertFalse(result['passed'])
            self.assertEqual(result['strict_pass_count'],1)
            self.assertEqual(result['bit_exact_workers'],1)

    def test_minimal_cpu_worker_requires_actual_native_scene_device_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'worker',True)
            (folder/'app_extensions.json').write_text(json.dumps(dict(physics_only=True,enabled=[])))
            row=generate(root)['trials'][0]
            self.assertFalse(row['timing_comparable']);self.assertEqual(row['status'],'completed_native_device_unverified')
            runtime=json.loads((folder/'runtime_settings.json').read_text())
            runtime['native_scene_device']=dict(enabled_gpu_dynamics=False,broadphase_type='MBP')
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            row=generate(root)['trials'][0]
            self.assertTrue(row['timing_comparable']);self.assertTrue(row['native_device_matches_requested'])
            runtime['native_scene_device']['enabled_gpu_dynamics']=True
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            row=generate(root)['trials'][0]
            self.assertFalse(row['timing_comparable']);self.assertEqual(row['status'],'completed_but_native_device_mismatch')

    def parallel_fixture(self, root):
        pool=root/'pool2_results';pool.mkdir()
        campaign=[]
        for name,start,end in [('worker_00',100.,102.),('worker_01',100.25,103.)]:
            folder=self.fixture(pool,name,True)
            summary=json.loads((folder/'summary.json').read_text())
            summary.update(initialization_valid=True,loop_started_monotonic_s=start,
                           loop_finished_monotonic_s=end,physics_wall_s=end-start)
            (folder/'summary.json').write_text(json.dumps(summary))
            runtime=json.loads((folder/'runtime_settings.json').read_text())
            runtime['native_scene_device']=dict(enabled_gpu_dynamics=False,broadphase_type='MBP')
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            (pool/(name+'.log')).write_text('[Warning] ordinary display warning\n')
            (pool/(name+'_resources.json')).write_text(json.dumps([dict(rss_bytes=1024**3,cpu_percent=100.)]))
            campaign.append(dict(name=name,returncode=0,total_wall_s=10.))
        (pool/'cases_results.json').write_text(json.dumps(campaign))
        group=dict(index=0,jobs=2,names=['worker_00','worker_01'],throughput_eligible=True,
                   exclusion_reasons=[],measured_concurrent_physics_window_s=3.,
                   sum_aggregate_simulated_s=2.,group_startup_wall_s=5.,post_loop_wall_s=1.,group_total_wall_s=9.)
        (pool/'cases_parallel_groups.json').write_text(json.dumps([group]))
        (pool/'parallel_group_0000_resources.json').write_text(json.dumps([
            dict(monotonic_s=99.,children=[dict(rss_bytes=3*1024**3,cpu_percent=10.)],gpu_memory_mib=600),
            dict(monotonic_s=101.,children=[dict(rss_bytes=1024**3,cpu_percent=100.),
                                          dict(rss_bytes=2*1024**3,cpu_percent=150.)],gpu_memory_mib=800)]))
        return pool

    def test_nested_parallel_group_uses_concurrent_window_and_simultaneous_resources(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.parallel_fixture(root)
            report=generate(root)
            self.assertEqual(len(report['trials']),2)
            child=report['trials'][0]
            self.assertEqual(child['resources']['rss_gib_max'],1.)
            self.assertEqual(child['warnings']['warnings'],1)
            self.assertEqual(child['process_total_wall_s'],10.)
            self.assertIn('pool2_results/worker_00.log',child['files'].values())
            group=report['parallel_groups'][0]
            self.assertTrue(group['throughput_eligible'])
            self.assertAlmostEqual(group['accepted_aggregate_sim_s_per_wall_s'],2/3)
            self.assertAlmostEqual(group['accepted_end_to_end_aggregate_sim_s_per_wall_s'],2/9)
            self.assertEqual(group['loop_start_spread_s'],.25)
            self.assertEqual(group['resources']['whole_group_lifetime']['rss_gib_max'],3.)
            self.assertEqual(group['resources']['concurrent_physics_window']['cpu_percent_mean'],250.)
            self.assertIn('동시 physics', (root/'index.html').read_text())

    def test_parallel_campaign_flag_cannot_override_invalid_or_missing_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);pool=self.parallel_fixture(root)
            raw=(pool/'cases_parallel_groups.json').read_bytes()
            summary_path=pool/'worker_01/summary.json'
            summary=json.loads(summary_path.read_text());summary['initialization_valid']=False
            summary_path.write_text(json.dumps(summary))
            group=generate(root)['parallel_groups'][0]
            self.assertFalse(group['throughput_eligible'])
            self.assertIsNone(group['accepted_aggregate_sim_s_per_wall_s'])
            self.assertTrue(any('initialization_invalid' in reason for reason in group['exclusion_reasons']))
            self.assertEqual((pool/'cases_parallel_groups.json').read_bytes(),raw)
            for file in (pool/'worker_01').iterdir():file.unlink()
            (pool/'worker_01').rmdir()
            group=generate(root)['parallel_groups'][0]
            self.assertFalse(group['throughput_eligible'])
            self.assertIn('worker_01:missing_trial',group['exclusion_reasons'])

    def test_parallel_raw_timing_mismatch_is_not_silently_repaired(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);pool=self.parallel_fixture(root)
            path=pool/'cases_parallel_groups.json';groups=json.loads(path.read_text())
            groups[0]['measured_concurrent_physics_window_s']=1.
            path.write_text(json.dumps(groups))
            group=generate(root)['parallel_groups'][0]
            self.assertFalse(group['throughput_eligible'])
            self.assertIn('campaign_native_window_mismatch',group['exclusion_reasons'])
            self.assertIsNone(group['accepted_aggregate_sim_s_per_wall_s'])

    def test_worker_trace_equivalence_is_separate_from_parallel_runtime_speed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.parallel_fixture(root)
            (root/'pool_trace_equivalence.json').write_text(json.dumps(dict(
                total_workers=54,strict_pass_count=54,strict_fail_count=0,
                interpretation='Finite trace comparison only')))
            (root/'single_trace_equivalence.json').write_text(json.dumps(dict(
                comparisons_total=6,comparisons_passed=6,interpretation='Single controls')))
            report=generate(root)
            self.assertTrue(report['comparisons'][0]['passed'])
            self.assertTrue(report['comparisons'][1]['passed'])
            self.assertTrue(report['parallel_groups'][0]['throughput_eligible'])
            self.assertIn('54/54 worker strict trace PASS',(root/'index.html').read_text())
            self.assertIn('6/6 strict trace PASS',(root/'index.html').read_text())

    def test_cpu_solver_with_explicit_sap_or_gpu_broadphase_is_not_gpu_dynamics(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'broadphase_trial',True)
            runtime=json.loads((folder/'runtime_settings.json').read_text())
            for broadphase in ('SAP','GPU'):
                runtime['native_scene_device']=dict(enabled_gpu_dynamics=False,
                    broadphase_type=broadphase,requested_broadphase=broadphase,
                    dynamics_backend='cpu',broadphase_backend='gpu' if broadphase=='GPU' else 'cpu')
                (folder/'runtime_settings.json').write_text(json.dumps(runtime))
                row=generate(root)['trials'][0]
                self.assertTrue(row['native_device_matches_requested'])
                self.assertTrue(row['timing_comparable'])
                self.assertEqual(row['device'],'cpu')
                self.assertEqual(row['native_broadphase'],broadphase)
            runtime['native_scene_device']['enabled_gpu_dynamics']=True
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            row=generate(root)['trials'][0]
            self.assertFalse(row['native_device_matches_requested'])
            self.assertFalse(row['timing_comparable'])

    def test_legacy_stage_unit_mismatch_keeps_raw_timing_without_SI_certification(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'legacy',True)
            runtime=json.loads((folder/'runtime_settings.json').read_text())
            runtime['stage_metrics']=dict(meters_per_unit=.01,up_axis='Y',kilograms_per_unit=1.)
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            original=(folder/'summary.json').read_bytes()
            row=generate(root)['trials'][0]
            self.assertTrue(row['completed']);self.assertFalse(row['timing_comparable'])
            self.assertIsNone(row['accepted_aggregate_sim_s_per_wall_s'])
            self.assertEqual(row['historical_numerical_aggregate_sim_s_per_wall_s'],.5)
            self.assertIn('legacy_stage_metrics_not_SI',row['timing_exclusion_reasons'])
            self.assertEqual((folder/'summary.json').read_bytes(),original)

    def test_source_stage_metrics_audit_requires_matching_model_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'source_audited',True)
            runtime=json.loads((folder/'runtime_settings.json').read_text());runtime.pop('stage_metrics')
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            audit=dict(models=[dict(model_sha256='different',meters_per_unit=1.,up_axis='Z',kilograms_per_unit=1.)])
            (root/'stage_metrics_source_audit.json').write_text(json.dumps(audit))
            self.assertFalse(generate(root)['trials'][0]['timing_comparable'])
            audit['models'][0]['model_sha256']='abcd'
            (root/'stage_metrics_source_audit.json').write_text(json.dumps(audit))
            self.assertTrue(generate(root)['trials'][0]['timing_comparable'])

    def test_front_summary_cannot_promote_named_legacy_baseline_to_SI_result(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=self.fixture(root,'legal_metre_z_rebuilt_pulse120',True)
            generate(root)
            self.assertIn('현재 metre/Z/kg를 확인한 결과',(root/'index.html').read_text())
            runtime=json.loads((folder/'runtime_settings.json').read_text())
            runtime['stage_metrics']['meters_per_unit']=.01
            (folder/'runtime_settings.json').write_text(json.dumps(runtime))
            generate(root)
            self.assertIn('정상 완료한 성능 결과가 아직 없습니다',(root/'index.html').read_text())


if __name__ == '__main__':
    unittest.main()
