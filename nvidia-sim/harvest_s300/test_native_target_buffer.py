"""Real CPU Torch ownership regression for the three native initialization paths."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
try:
    import torch
except ImportError:
    torch=None


@unittest.skipIf(torch is None,'Run separately with the existing planning/native Torch Python')
class NativeTargetBufferTests(unittest.TestCase):
    def assignments(self):
        tree=ast.parse(Path(__file__).with_name('run_motion.py').read_text())
        result=[node for node in ast.walk(tree) if isinstance(node,ast.Assign) and
                len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and
                node.targets[0].id=='target_tensor']
        self.assertEqual(len(result),3)
        return result

    def test_opt_in_preserves_measured_start_through_repeated_native_target_writes(self):
        for assignment in self.assignments():
            measured=np.array([[.0192492958,.3,.4]],np.float32)
            original=measured.copy()
            context=dict(q0=measured,a=SimpleNamespace(retain_lift_drive_reference=True),
                tensor=lambda value:torch.as_tensor(value,dtype=torch.float32,device='cpu'))
            exec(compile(ast.fix_missing_locations(ast.Module(body=[assignment],type_ignores=[])),
                         'actual_native_target_constructor','exec'),context)
            target=context['target_tensor']
            for _ in range(100):
                command=measured.copy();command[0,0]+=.0083152987
                target.copy_(torch.as_tensor(command))
                np.testing.assert_array_equal(measured,original)
                self.assertAlmostEqual(float(target[0,0]),.0275645945,places=8)
            self.assertFalse(np.shares_memory(target.numpy(),measured))

    def test_default_retains_legacy_target_buffer_semantics(self):
        for assignment in self.assignments():
            measured=np.array([[.0192492958,.3,.4]],np.float32)
            context=dict(q0=measured,a=SimpleNamespace(retain_lift_drive_reference=False),
                tensor=lambda value:torch.as_tensor(value,dtype=torch.float32,device='cpu'))
            exec(compile(ast.fix_missing_locations(ast.Module(body=[assignment],type_ignores=[])),
                         'actual_native_target_constructor','exec'),context)
            self.assertTrue(np.shares_memory(context['target_tensor'].numpy(),measured))


if __name__=='__main__':unittest.main()
