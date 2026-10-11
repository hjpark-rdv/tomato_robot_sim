"""Execute the opt-in production rail-screen wiring before native simulation."""
import ast
from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import sys
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import patch

from test_run_motion_conditioning import actual_cli, code, MAIN

WIRE = next(node for node in ast.walk(MAIN) if isinstance(node, ast.If) and
            'a.verify_rail_environment_preflight' in ast.unparse(node.test))


class RunMotionRailEnvironmentTests(unittest.TestCase):
    def test_default_is_unchanged_and_explicit_rail_sampling_matches_physics(self):
        self.assertFalse(actual_cli([]).verify_rail_environment_preflight)
        self.assertTrue(actual_cli(['--hz','120','--verify-rail-environment-preflight']).verify_rail_environment_preflight)
        for arguments in (['--hz','240'], ['--hz','60'], ['--hz','120','--num-envs','2'],
                          ['--hz','120','--replicate']):
            with self.subTest(arguments=arguments), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                actual_cli(arguments+['--verify-rail-environment-preflight'])

    def run_wiring(self, enabled, phases, *, group=None, released=False, error=None, native_cache=None, attachment_transfer=False):
        calls=[]
        module=ModuleType('rail_environment_preflight')
        def audit(*args, **kwargs):
            calls.append((args,kwargs))
            if error: raise error
            return dict(status='sampled_clear', complete=True, passed=True)
        module.audit_external_rail=audit
        namespace=dict(a=SimpleNamespace(verify_rail_environment_preflight=enabled,
            assembly=Path('/model'),output=Path('/result'),resume_native_checkpoint=Path('/measured-final'),
            hz=120,released_fruit_ground_diagnostic=released,native_tool_collision_cache=native_cache,
            allow_resume_attachment_transfer=attachment_transfer),
            planning=dict(phases=[dict(name=name) for name in phases]),initial_sequence_group=group)
        with patch.dict(sys.modules,rail_environment_preflight=module),redirect_stdout(io.StringIO()):
            exec(code([WIRE]),namespace)
        return calls

    def test_default_and_same_station_never_invoke_a_farm_scan(self):
        self.assertEqual(self.run_wiring(False,['rail_move']),[])
        self.assertEqual(self.run_wiring(True,['approach','close']),[])
        self.assertEqual(self.run_wiring(True,[]),[])

    def test_initial_rail_uses_exact_parent_and_existing_contact_permission(self):
        calls=self.run_wiring(True,['rail_move','rail_settle','approach'],
                              group=dict(truss_key='Truss_01'),released=True)
        self.assertEqual(len(calls),1)
        args,kw=calls[0]
        self.assertEqual(args,(Path('/model'),Path('/result/rail_environment_preflight.json')))
        self.assertEqual(kw['plan'],Path('/result/plan.json'))
        self.assertEqual(kw['checkpoint'],Path('/measured-final'))
        self.assertEqual(kw['allowed_truss_key'],'Truss_01')
        self.assertTrue(kw['released_fruit_collection_diagnostic'])
        self.assertEqual(kw['hz'],120)

    def test_rail_checker_receives_same_runtime_native_tool_pieces(self):
        path=Path('/measured-native-tool.json')
        calls=self.run_wiring(True,['rail_move'],native_cache=path)
        self.assertEqual(calls[0][1]['native_tool_cache'],path)
        self.assertIsNone(self.run_wiring(True,['rail_move'])[0][1]['native_tool_cache'])

    def test_exact_attachment_transfer_permission_reaches_rail_checker(self):
        self.assertTrue(self.run_wiring(True,['rail_move'],attachment_transfer=True)[0][1]['allow_attachment_transfer'])
        self.assertFalse(self.run_wiring(True,['rail_move'])[0][1]['allow_attachment_transfer'])

    def test_blocked_or_inconclusive_screen_stops_before_native_initialization(self):
        with self.assertRaisesRegex(RuntimeError,'inconclusive'):
            self.run_wiring(True,['rail_move'],error=RuntimeError('inconclusive source identity'))
        # The production call is earlier in the same outer try than model
        # initialization and the first simulation loop, not a post-run report.
        order=ast.unparse(MAIN)
        self.assertLess(order.index('audit_external_rail('),order.index('sim.initialize_physics('))


if __name__=='__main__':unittest.main()
