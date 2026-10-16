"""Protect opt-in model selection and the original GPU execution schedule."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class LauncherTests(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
