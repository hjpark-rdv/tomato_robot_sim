"""CLI scope and production cache/plan forwarding for the opt-in joint gauge."""
import ast
from contextlib import redirect_stderr
import io
import unittest
from test_run_motion_conditioning import actual_cli,MAIN

class RunMotionJointTurnTests(unittest.TestCase):
    def test_enabled_requires_existing_horizontal_joint_preparation(self):
        self.assertFalse(actual_cli([]).horizontal_ready_normalize_turns)
        args=['--mode','harvest','--preparation','horizontal_ready','--horizontal-ready-connector','joint',
              '--horizontal-ready-normalize-turns']
        self.assertTrue(actual_cli(args).horizontal_ready_normalize_turns)
        for bad in (['--horizontal-ready-normalize-turns'],['--mode','harvest','--horizontal-ready-normalize-turns']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(bad)

    def test_every_replanned_cycle_receives_flag_and_cache_key_cannot_alias_default(self):
        calls=[n for n in ast.walk(MAIN) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='plan_motion']
        self.assertEqual(len(calls),1)
        keyword=next(n for n in calls[0].keywords if n.arg=='horizontal_ready_normalize_turns')
        self.assertEqual(ast.unparse(keyword.value),'a.horizontal_ready_normalize_turns')
        # The same production make_plan wrapper is used by both the initial
        # plan and later measured-state cycles; failed-query dedup must retain
        # this opt-in distinction.
        text=ast.unparse(MAIN)
        self.assertIn("'horizontal_ready_normalize_turns'",text)
        self.assertIn('make_plan(center - reference_center, next_axis',text)

if __name__=='__main__':unittest.main()
