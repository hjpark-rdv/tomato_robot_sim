"""Run production pre-initialize farm-sampling wiring with an exact mock checker."""
import ast
from contextlib import redirect_stdout,redirect_stderr
import io
from pathlib import Path
import sys
from types import SimpleNamespace,ModuleType
import unittest
from unittest.mock import patch
from test_run_motion_conditioning import actual_cli,code,MAIN

WIRE=next(node for node in ast.walk(MAIN) if isinstance(node,ast.If) and
          ast.unparse(node.test)=='a.verify_preparation_environment_preflight')

class PreparationEnvironmentWiringTests(unittest.TestCase):
    def test_explicit_mode_only_and_actual120hz_single_environment(self):
        self.assertFalse(actual_cli([]).verify_preparation_environment_preflight)
        self.assertTrue(actual_cli(['--hz','120','--verify-preparation-environment-preflight']).verify_preparation_environment_preflight)
        for args in (['--hz','60'],['--hz','240'],['--hz','120','--num-envs','2'],['--hz','120','--replicate']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                actual_cli(args+['--verify-preparation-environment-preflight'])

    def run_wire(self,enabled,error=None,*,native_cache=None):
        calls=[];module=ModuleType('rail_environment_preflight')
        def audit(*args,**kwargs):
            calls.append((args,kwargs))
            if error:raise error
            return dict(passed=True,complete=True)
        module.audit_external_rail=audit
        context=dict(a=SimpleNamespace(verify_preparation_environment_preflight=enabled,
            assembly=Path('/fresh_model'),output=Path('/trial'),resume_native_checkpoint=None,
            hz=120,released_fruit_ground_diagnostic=True,allow_resume_attachment_transfer=False,
            native_tool_collision_cache=native_cache),
            initial_sequence_group=dict(truss_key='Truss_01'))
        with patch.dict(sys.modules,rail_environment_preflight=module),redirect_stdout(io.StringIO()):
            exec(code([WIRE]),context)
        return calls

    def test_fresh_source_exact_group_and_full_preparation_scope(self):
        self.assertEqual(self.run_wire(False),[])
        calls=self.run_wire(True);self.assertEqual(len(calls),1)
        args,kwargs=calls[0]
        self.assertEqual(args,(Path('/fresh_model'),Path('/trial/preparation_environment_preflight.json')))
        self.assertEqual(kwargs['scope'],'preparation');self.assertEqual(kwargs['hz'],120)
        self.assertEqual(kwargs['allowed_truss_key'],'Truss_01');self.assertIsNone(kwargs['checkpoint'])
        self.assertFalse(kwargs['allow_attachment_transfer'])
        self.assertEqual(kwargs['max_distance_queries'],2000000)

    def test_preparation_checker_receives_same_runtime_native_tool_pieces(self):
        path=Path('/measured-native-tool.json')
        self.assertEqual(self.run_wire(True,native_cache=path)[0][1]['native_tool_cache'],path)
        self.assertIsNone(self.run_wire(True)[0][1]['native_tool_cache'])

    def test_blocked_or_incomplete_checker_cannot_reach_native_initialize(self):
        for reason in ('static foreign leaf blocks preparation','inconclusive query budget'):
            with self.assertRaisesRegex(RuntimeError,reason):self.run_wire(True,RuntimeError(reason))
        text=ast.unparse(MAIN)
        self.assertLess(text.index('a.verify_preparation_environment_preflight'),text.index('sim.initialize_physics('))

if __name__=='__main__':unittest.main()
