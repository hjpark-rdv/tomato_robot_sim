import ast
from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

from truss_sequence import select_group,validate_single_fruit_when_remaining
from row_harvest_targets import select_scoped_group


class SingleFruitAimTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'Truss_02_Fruit_{i:02}',path=f'/fruit/{i}',joint=f'/joint/{i}') for i in (1,2,3)]
        self.paths=[f['path']for f in self.fruits]
        self.poses=np.array([[0,0,1,0,0,0,1],[.025,0,1.01,0,0,0,1],[0,0,.9,0,0,0,1]])

    def test_default_is_exact_and_sparse_pair_selects_actual_individual_center(self):
        breaks=[dict(joint=self.fruits[2]['joint'])]
        for selector in (select_group,select_scoped_group):
            old=selector(self.fruits,breaks,self.paths,self.poses,{})
            explicit=selector(self.fruits,breaks,self.paths,self.poses,{},single_fruit_when_remaining=0)
            self.assertEqual(old,explicit);self.assertEqual(old['fruits'],[f['name']for f in self.fruits[:2]])
            single=selector(self.fruits,breaks,self.paths,self.poses,{},single_fruit_when_remaining=2)
            self.assertEqual(single['fruits'],[self.fruits[0]['name']])
            np.testing.assert_array_equal(single['measured_centroid_world_m'],self.poses[0,:3])
            self.assertTrue(single['individual_sparse_aim']['selected_individually'])
            self.assertFalse(single['individual_sparse_aim']['height_offset_changed'])
            high=selector(self.fruits,breaks,self.paths,self.poses,{},single_fruit_when_remaining=2,order='highest-first')
            self.assertEqual(high['fruits'],[self.fruits[1]['name']])
            np.testing.assert_array_equal(high['measured_centroid_world_m'],self.poses[1,:3])

    def test_scoped_threshold_counts_own_truss_and_preserves_other_truss_inventory(self):
        other=[dict(name=f'Truss_01_Fruit_{i:02}',path=f'/other/{i}',joint=f'/otherjoint/{i}')for i in (1,2,3,4)]
        fruits=self.fruits+other;paths=self.paths+[f['path']for f in other]
        poses=np.vstack([self.poses,[[0,0,2,0,0,0,1]]*4])
        breaks=[dict(joint=self.fruits[2]['joint'])]
        group=select_scoped_group(fruits,breaks,paths,poses,{},single_fruit_when_remaining=2)
        self.assertEqual(group['fruits'],[self.fruits[0]['name']])
        self.assertEqual(group['individual_sparse_aim']['attached_fruit_count'],2)
        # Exhausting attempts excludes candidates, not attached physical fruit;
        # it must not manufacture a sparse remainder in the four-fruit truss.
        attempts={f['name']:2 for f in self.fruits}
        group=select_scoped_group(fruits,breaks,paths,poses,attempts,single_fruit_when_remaining=2)
        self.assertEqual(len(group['fruits']),2)
        self.assertFalse(group['individual_sparse_aim']['selected_individually'])
        self.assertEqual(group['individual_sparse_aim']['attached_fruit_count'],4)

    def test_invalid_threshold_rejected_even_for_empty_scope(self):
        for invalid in (-1,4,True,1.5,float('nan')):
            with self.assertRaises(ValueError):validate_single_fruit_when_remaining(invalid)
            for selector in (select_group,select_scoped_group):
                with self.assertRaises(ValueError):
                    selector([],[],[],np.zeros((0,7)),{},single_fruit_when_remaining=invalid)

    def test_actual_runner_cli_and_both_selector_sites(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text());prefix=[]
        for node in tree.body:
            if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='mkdir':break
            prefix.append(node)
        code=compile(ast.Module(body=prefix,type_ignores=[]),str(source),'exec')
        for args,expected in (([],0),(['--single-fruit-when-remaining','2'],2),(['--single-fruit-when-remaining','3'],3)):
            env={}
            with patch.object(sys,'argv',[str(source),'/unused-assembly','--output','/unused-output',*args]):exec(code,env)
            self.assertEqual(env['a'].single_fruit_when_remaining,expected)
        for invalid in ('-1','4','1.5'):
            with patch.object(sys,'argv',[str(source),'/unused-assembly','--output','/unused-output','--single-fruit-when-remaining',invalid]),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):exec(code,{})
        calls=[n for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)and n.func.id=='sequence_selector']
        self.assertEqual(len(calls),2)
        for call in calls:
            value=next(k.value for k in call.keywords if k.arg=='single_fruit_when_remaining')
            self.assertEqual(ast.unparse(value),'a.single_fruit_when_remaining')


if __name__=='__main__':unittest.main()
