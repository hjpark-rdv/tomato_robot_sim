"""Per-window exact-source cache selection; no native physics in these tests."""
import copy,json,shutil,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import test_native_tool_hulls as native_fixture
import test_run_row_harvest as row_fixture
from run_row_harvest import (verified_window_native_tool_cache,capture_window_native_tool_cache,
                            campaign,DEFAULT_ARGUMENTS)


class AutomaticNativeToolCacheTests(unittest.TestCase):
    def test_existing_real_loader_binds_exact_assembly_and_rejects_foreign_copy(self):
        f=native_fixture.NativeToolHullTests();f.setUp()
        try:
            proof=verified_window_native_tool_cache(f.assembly,f.cache)
            self.assertTrue(proof['source_hashes_verified']);self.assertEqual(proof['loader']['native_piece_count'],2)
            self.assertFalse(proof['diagnostic_native_state_accepted'])
            foreign=f.root/'other_window';shutil.copytree(f.assembly,foreign)
            with self.assertRaisesRegex(ValueError,'source scene/USDC SHA mismatch'):
                verified_window_native_tool_cache(foreign,f.cache)
        finally:f.tearDown()

    def test_capture_reuses_only_existing_idle_benchmark_and_retains_result(self):
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);commands=[]
            def runner(command,check):
                commands.append(command);cases=Path(command[4]);case=json.loads(cases.read_text())[0]
                args=case['arguments'];self.assertEqual(args[args.index('--mode')+1],'idle')
                self.assertEqual(args[args.index('--seconds')+1],'3');self.assertIn('--physics-only',args)
                self.assertNotIn('--rail-to-y',args);self.assertNotIn('--camera-harvest',args)
                self.assertEqual(args[args.index('--capture-tool-colliders')+1:],
                    ['/World/ActuatedTool/L/part_209','/World/ActuatedTool/R/part_331'])
                folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
                summary=dict(mode='idle',completed=True,stop_reason=None,initialization_valid=True,
                    simulated_s=3.,rail_destination=None,native_new_break_count=0)
                (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary=summary)]))
            expected=dict(cache='/exact/current/native_cache.json',source_hashes_verified=True,diagnostic_native_state_accepted=False)
            with patch('run_row_harvest.verified_window_native_tool_cache',return_value=expected.copy())as verify:
                result=capture_window_native_tool_cache('/exact/current',DEFAULT_ARGUMENTS,root/'probe','capture',
                    '/benchmark/python','/native/python',120,runner)
                self.assertEqual(len(commands),1);self.assertEqual(result['cache'],expected['cache'])
                self.assertTrue((root/'probe/probe_result.json').is_file())
                self.assertEqual(Path(verify.call_args.args[0]),Path('/exact/current'))

    def test_initial_cache_is_taken_out_of_shared_profile_before_actual_native_command(self):
        f=row_fixture.RunRowHarvestTests();f.setUp()
        try:
            args=DEFAULT_ARGUMENTS+['--native-tool-collision-cache','/exact/initial/cache.json'];original=args.copy();calls=[]
            def runner(command,check):
                cases=Path(command[4]);case=json.loads(cases.read_text())[0];calls.append(case['arguments'])
                folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
                (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=1,timeout=False,summary=None)]))
            with patch('run_row_harvest.verify_compact_dependencies',return_value={}), \
                 patch('run_row_harvest.verified_window_native_tool_cache',return_value=dict(cache='/verified/current/cache.json'))as verify:
                report=campaign(f.assembly,f.inventory_path,f.screen_path,f.root/'auto_cache',execute=True,
                    max_plants=1,base_arguments=args,preflight_new_windows=True,auto_native_tool_cache=True,
                    benchmark_runner=runner)
            self.assertEqual(args,original);self.assertEqual(len(calls),1)
            self.assertNotIn('/exact/initial/cache.json',calls[0]);self.assertIn('/verified/current/cache.json',calls[0])
            self.assertEqual(verify.call_args.args,(f.assembly.resolve(),Path('/exact/initial/cache.json')))
            self.assertTrue(report['automatic_native_tool_cache']['initial_cache_never_forwarded_to_other_windows'])
        finally:f.tearDown()

    def test_capture_failure_is_saved_and_never_imports_cache(self):
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp)
            def runner(command,check):
                cases=Path(command[4]);folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
                (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=1,timeout=False,summary=None)]))
            with patch('run_row_harvest.verified_window_native_tool_cache')as verify:
                with self.assertRaisesRegex(ValueError,'idle did not pass'):
                    capture_window_native_tool_cache('/exact/current',DEFAULT_ARGUMENTS,root/'probe','capture',
                        '/benchmark/python','/native/python',120,runner)
                verify.assert_not_called();self.assertTrue((root/'probe/probe_result.json').is_file())


if __name__=='__main__':unittest.main()
