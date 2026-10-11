import ast
from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

from truss_sequence import select_group,validate_max_fruit_attempts
from row_harvest_targets import select_scoped_group


class MaxFruitAttemptsTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'Truss_02_Fruit_{i:02}',path=f'/fruit/{i}',joint=f'/joint/{i}') for i in (1,2)]
        self.paths=[f['path']for f in self.fruits]
        self.poses=np.array([[0,0,1,0,0,0,1],[.02,0,1.01,0,0,0,1]])

    def test_default_two_is_identical_and_six_retains_fruits_until_six_attempts(self):
        for selector in (select_group,select_scoped_group):
            for number in (0,1,2):
                attempts={f['name']:number for f in self.fruits}
                self.assertEqual(selector(self.fruits,[],self.paths,self.poses,attempts),
                    selector(self.fruits,[],self.paths,self.poses,attempts,max_attempts=2))
            for number in range(6):
                attempts={f['name']:number for f in self.fruits}
                self.assertIsNotNone(selector(self.fruits,[],self.paths,self.poses,attempts,max_attempts=6))
            attempts={f['name']:6 for f in self.fruits}
            self.assertIsNone(selector(self.fruits,[],self.paths,self.poses,attempts,max_attempts=6))
            self.assertIsNone(selector(self.fruits,[dict(joint=f['joint'])for f in self.fruits],
                self.paths,self.poses,{},max_attempts=6))

    def test_retry_does_not_starve_an_untried_fruit_or_cross_a_truss(self):
        extra=dict(name='Truss_01_Fruit_01',path='/fruit/other',joint='/joint/other')
        fruits=self.fruits+[extra];paths=self.paths+[extra['path']]
        poses=np.vstack([self.poses,[.001,0,1.001,0,0,0,1]])
        attempts={f['name']:5 for f in self.fruits}
        group=select_scoped_group(fruits,[],paths,poses,attempts,max_attempts=6)
        self.assertEqual(group['fruits'],[extra['name']])
        attempts[extra['name']]=6
        group=select_scoped_group(fruits,[],paths,poses,attempts,max_attempts=6)
        self.assertEqual(group['fruits'],[f['name']for f in self.fruits])

    def test_invalid_budget_rejected_even_without_any_eligible_fruit(self):
        for value in (0,9,-1,True,2.5,float('nan')):
            with self.subTest(value=value),self.assertRaises(ValueError):validate_max_fruit_attempts(value)
            with self.assertRaises(ValueError):select_group([],[],[],np.zeros((0,7)),{},max_attempts=value)
            with self.assertRaises(ValueError):select_scoped_group([],[],[],np.zeros((0,7)),{},max_attempts=value)

    def test_actual_runner_cli_defaults_validation_and_both_selector_sites(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text())
        # Execute the real parser and validation block, stopping before any
        # output mutation, SDK import, stage opening, or physical execution.
        prefix=[]
        for node in tree.body:
            if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='mkdir':break
            prefix.append(node)
        code=compile(ast.Module(body=prefix,type_ignores=[]),str(source),'exec')
        for args,expected in (([],2),(['--max-fruit-attempts','6'],6),(['--max-fruit-attempts','8'],8),(['--max-fruit-attempts','1'],1)):
            env={}
            with patch.object(sys,'argv',[str(source),'/unused-assembly','--output','/unused-output',*args]):exec(code,env)
            self.assertEqual(env['a'].max_fruit_attempts,expected)
        for bad in ('0','9','2.5'):
            with patch.object(sys,'argv',[str(source),'/unused-assembly','--output','/unused-output','--max-fruit-attempts',bad]),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):exec(code,{})
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='sequence_selector']
        self.assertEqual(len(calls),2)
        for call in calls:
            value=next(k.value for k in call.keywords if k.arg=='max_attempts')
            self.assertEqual(ast.unparse(value),'a.max_fruit_attempts')

    def test_explicit_extension_keeps_six_measured_attempts_and_remains_bounded(self):
        attempts={f['name']:6 for f in self.fruits};before=dict(attempts)
        self.assertIsNone(select_group(self.fruits,[],self.paths,self.poses,attempts,max_attempts=6))
        self.assertIsNotNone(select_group(self.fruits,[],self.paths,self.poses,attempts,max_attempts=8))
        self.assertEqual(attempts,before)
        self.assertIsNone(select_group(self.fruits,[],self.paths,self.poses,
            {f['name']:8 for f in self.fruits},max_attempts=8))


if __name__=='__main__':unittest.main()
