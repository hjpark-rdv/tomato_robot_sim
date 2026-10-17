"""Explicit solver tolerance must survive upload; default is untouched."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from newton_gpu_options import solver_tolerance_value, apply_explicit_solver_tolerance


class SolverToleranceTests(unittest.TestCase):
    def test_opt_in_overwrites_upload_floor_but_none_never_writes(self):
        class DeviceArray:
            def __init__(self): self.value = 1e-6; self.calls = []
            def fill_(self, value): self.value = value; self.calls.append(value)
        tolerance = DeviceArray()
        options = SimpleNamespace(tolerance=tolerance)
        self.assertIsNone(apply_explicit_solver_tolerance(options, None))
        self.assertEqual(tolerance.calls, [])
        self.assertEqual(tolerance.value, 1e-6)
        self.assertEqual(apply_explicit_solver_tolerance(options, 1e-8), 1e-8)
        self.assertEqual(tolerance.calls, [1e-8])
        self.assertEqual(tolerance.value, 1e-8)

    def test_nonfinite_or_nonpositive_values_cannot_reach_device(self):
        for value in (float('nan'), float('inf'), -float('inf'), 0, -1, True):
            with self.assertRaises(ValueError): solver_tolerance_value(value)
        self.assertEqual(solver_tolerance_value(1e-8), 1e-8)

    def test_runner_applies_after_engine_upload_before_evidence_and_capture(self):
        source = Path(__file__).with_name('run_newton_harvest.py').read_text()
        tree = ast.parse(source)
        run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run')
        overrides = [node for node in ast.walk(run) if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name) and node.func.id == 'apply_explicit_solver_tolerance']
        self.assertEqual(len(overrides), 1)
        call = overrides[0]
        self.assertEqual(ast.unparse(call.args[0]), 'solver.mjw_model.opt')
        self.assertEqual(ast.unparse(call.args[1]), 'args.solver_tolerance')
        assignments = {ast.unparse(node.targets[0]): node.lineno for node in ast.walk(run)
            if isinstance(node, ast.Assign) and len(node.targets) == 1}
        self.assertLess(assignments['(solver, d)'], call.lineno)
        self.assertLess(call.lineno, assignments['effective_gpu_options'])
        capture_lines = [node.lineno for node in ast.walk(run) if isinstance(node, ast.Attribute) and node.attr == 'ScopedCapture']
        self.assertTrue(capture_lines)
        self.assertLess(call.lineno, min(capture_lines))


if __name__ == '__main__':
    unittest.main()
