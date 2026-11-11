"""Protect opt-in model selection and the original GPU execution schedule."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class LauncherTests(unittest.TestCase):
    def test_standalone_uses_local_inputs_and_optional_history(self):
        wrapper=Path(__file__).resolve().parents[1]/'run_s300_newton.sh'
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);inputs=root/'assets/current/inputs'
            inputs.mkdir(parents=True);(inputs/'inputs.json').write_text('{}')
            capture=root/'argv.json';python=root/'python'
            python.write_text('#!/usr/bin/env python3\nimport json,os,sys\n'
                'open(os.environ["S300_TEST_CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n')
            python.chmod(0o755)
            env=dict(os.environ,S300_NEWTON_PYTHON=str(python),S300_TEST_CAPTURE=str(capture),
                     S300_NEWTON_RUNTIME=str(root))
            for variable in ('S300_NEWTON_INPUTS','S300_NEWTON_OUTPUT'):
                env.pop(variable,None)
            for headless in (False,True):
                subprocess.run([str(wrapper),'standalone']+(['--headless'] if headless else []),
                               env=env,check=True,capture_output=True,text=True)
                argv=json.loads(capture.read_text())
                self.assertEqual(argv[2],str(inputs))
                self.assertEqual(argv[argv.index('--recording')+1],'summary')
                self.assertEqual(argv[argv.index('--viewer-backend')+1],'viser')
                self.assertEqual('--gui' in argv,not headless)
                self.assertEqual('--session' in argv,not headless)
                self.assertEqual('--interactive-viewer' in argv,not headless)
                self.assertNotIn('--diagnostic-continue',argv)

    def test_row_profile_reuses_existing_orchestrator_and_preserves_overrides(self):
        wrapper = Path(__file__).resolve().parents[1] / 'run_s300_newton.sh'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture, python = root / 'argv.json', root / 'python'
            python.write_text('#!/usr/bin/env python3\nimport json,os,sys\n'
                'open(os.environ["S300_TEST_CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n')
            python.chmod(0o755)
            env = dict(os.environ, S300_NEWTON_PYTHON=str(python),
                S300_TEST_CAPTURE=str(capture), S300_NEWTON_ROUTE=str(root / 'default_route'),
                S300_NEWTON_MODEL_CACHE=str(root / 'cache'))
            for route in (None, root / 'route with spaces'):
                args = [str(wrapper), 'row-realtime'] + ([] if route is None else [str(route)])
                subprocess.run(args + ['--campaign', 'my_trial', '--validate-only'],
                    env=env, check=True, capture_output=True, text=True)
                argv = json.loads(capture.read_text())
                self.assertTrue(argv[1].endswith('/run_newton_row_harvest.py'))
                self.assertEqual(argv[argv.index('--prepared-root') + 1],
                    str(root / 'default_route') if route is None else str(route))
                self.assertIn('--active-window', argv)
                self.assertIn('--released-gutter-diagnostic', argv)
                self.assertNotIn('--diagnostic-continue', argv)
                self.assertEqual(argv[argv.index('--solver-tolerance') + 1], '1e-8')
                self.assertEqual(argv[argv.index('--dense-hessian-overflow-capacity') + 1], '256')
                self.assertEqual(argv[-3:], ['--campaign', 'my_trial', '--validate-only'])

    def test_gpu_and_fast_keep_distinct_inputs_with_same_physics_options(self):
        wrapper = Path(__file__).resolve().parents[1] / 'run_s300_newton.sh'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            original, fast = root / 'original', root / 'variant'
            for folder in (original, fast):
                folder.mkdir()
                (folder / 'inputs.json').write_text('{}')
            capture = root / 'argv.json'
            python = root / 'python'
            python.write_text('#!/usr/bin/env python3\nimport json,os,sys\n'
                              'open(os.environ["S300_TEST_CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n')
            python.chmod(0o755)
            env = dict(os.environ, S300_NEWTON_PYTHON=str(python),
                       S300_NEWTON_INPUTS=str(original), S300_NEWTON_FAST_INPUTS=str(fast),
                       S300_NEWTON_OUTPUT=str(root / 'output'),
                       S300_TEST_CAPTURE=str(capture))
            commands = []
            for mode in ('gpu', 'gpu-fast'):
                subprocess.run([str(wrapper), mode, '--gui'], env=env,
                               check=True, capture_output=True, text=True)
                commands.append(json.loads(capture.read_text()))
            old, new = commands
            self.assertEqual(old[2], str(original))
            self.assertEqual(new[2], str(fast))
            # Only prepared-input selection changes, not dt, force checks or control flags.
            self.assertEqual(old[3:], new[3:])
            self.assertEqual(new[new.index('--hz') + 1], '400')
            self.assertEqual(new[new.index('--free-hz') + 1], '60')
            self.assertIn('--native-control-callback', new)
            self.assertIn('--canonical-constraints', new)
            self.assertEqual(new[-1], '--gui')

            # The new approximation is a named opt-in; legacy GPU presets above
            # retain their original model and schedule without this policy.
            env['S300_NEWTON_REALTIME_INPUTS'] = str(fast)
            subprocess.run([str(wrapper), 'gpu-realtime', '--gui'], env=env,
                           check=True, capture_output=True, text=True)
            realtime = json.loads(capture.read_text())
            self.assertEqual(realtime[2], str(fast))
            self.assertEqual(realtime[max(i for i,v in enumerate(realtime) if v=='--hz')+1], '220')
            self.assertEqual(realtime[realtime.index('--free-hz')+1], '60')
            policy = json.loads(Path(realtime[realtime.index('--contact-policy')+1]).read_text())
            self.assertEqual(policy['default_m'], .001)
            self.assertEqual(policy['belt_pedicel_m'], .002)
            self.assertIn('cycle_open_final', policy['relaxed_phases'])
            self.assertNotIn('cycle_open_final_hold', policy['relaxed_phases'])
            self.assertNotIn('--diagnostic-continue', realtime)
            self.assertIn('--interactive-viewer', realtime)
            self.assertNotIn('--interactive-viewer', old)
            subprocess.run([str(wrapper), 'gpu-realtime', '--gui', '--no-interactive-viewer'],
                           env=env, check=True, capture_output=True, text=True)
            automatic = json.loads(capture.read_text())
            self.assertGreater(automatic.index('--no-interactive-viewer'),
                               automatic.index('--interactive-viewer'))
            self.assertNotIn('--contact-policy', old)
            self.assertNotIn('--contact-policy', new)


if __name__ == '__main__':
    unittest.main()
