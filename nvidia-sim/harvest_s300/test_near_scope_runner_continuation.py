"""Exercise the actual runner's opt-in initial selection without starting Kit."""
import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np


class NearScopeRunnerContinuationTests(unittest.TestCase):
    def test_completed_original_truss_selects_own_measured_next_axis_and_first_depth(self):
        self._assert_next_truss(initial_exhausted=False)

    def test_attached_original_fruit_at_six_attempts_advances_without_reset_or_detachment(self):
        self._assert_next_truss(initial_exhausted=True)

    def _assert_next_truss(self,initial_exhausted):
        source=Path(__file__).with_name('run_motion.py')
        tree=ast.parse(source.read_text())
        main=next(node for node in tree.body if isinstance(node,ast.Try))
        nodes=main.body
        first=next(i for i,node in enumerate(nodes) if isinstance(node,ast.Assign) and
            any(isinstance(t,ast.Name) and t.id=='initial_sequence_group' for t in node.targets))
        end=next(i for i in range(first,len(nodes)) if isinstance(nodes[i],ast.Assign) and
            any(isinstance(t,ast.Tuple) and any(isinstance(v,ast.Name) and v.id=='phases' for v in t.elts)
                for t in nodes[i].targets))
        fruits=[dict(name=f'Truss_{truss:02d}_Fruit_{j+1:02d}',path=f'/fruit/{truss}/{j}',
                     joint=f'/joint/{truss}/{j}',center=[10.+j,20.,30.])
                for truss in (1,2) for j in range(3)]
        # The current measured new truss goes along Y. The old truss and its
        # saved entry direction go along X; authored centers are far away.
        positions=np.array([[1.,0.,1.1],[1.,.1,1.2],[1.,.2,1.3],
                            [2.,0.,1.],[2.1,0.,1.],[2.2,0.,1.]])
        prior_axis=dict(direction_world=[1.,0.,0.],fruit_names=[f['name'] for f in fruits[3:]])
        prior_cycle=dict(status='motion_completed',fruits=[fruits[3]['name']],
            start_s=2.,end_s=54.,planned_end_s=54.,measured_frame_index=0,end_frame_index=6480)
        metadata=dict(sequence=dict(initial_axis=prior_axis,cycle_plans=[prior_cycle]))
        with tempfile.TemporaryDirectory() as temporary:
            options=SimpleNamespace(whole_truss=True,truss_order='highest-first',
                near_trusses_metadata=Path('/explicit-inventory'),initial_truss_first=True,
                max_fruit_attempts=6,single_fruit_when_remaining=2,output=Path(temporary),
                truss_axis_min_fruits=3,grasp_depth_mm=65.,later_grasp_depth_mm=50.,
                grasp_height_offset_mm=-1.5,last_fruit_height_offset_mm=None,last_fruit_depth_mm=None)
            from truss_sequence import truss_fruits
            from row_harvest_targets import select_scoped_group
            native_breaks=[dict(joint=f['joint']) for f in fruits[3:]]
            attempts={f['name']:1 for f in fruits[3:]}
            if initial_exhausted:
                native_breaks=native_breaks[:-1]
                attempts[fruits[-1]['name']]=6
            env=dict(a=options,np=np,json=json,sequence_fruits=fruits,
                manifest=dict(target=dict(name=fruits[3]['name']),plant=dict(fruits=fruits),
                              grasp_reference=dict(height_offset_m=0.)),
                resume_context=dict(metadata=metadata),truss_fruits=truss_fruits,
                sequence_selector=select_scoped_group,
                initial_inventory_breaks=native_breaks,
                initial_body_paths=[f['path'] for f in fruits],initial_body_poses=positions,
                initial_attempt_counts=attempts.copy(),
                physical_unproductive_pairs=set(),selection_evidence=lambda value:value,
                initial_axis=prior_axis,reference_center=np.array([.4,1.,1.]),
                planning_kin=SimpleNamespace(world=np.eye(4)),
                initial_grasp_depth_mm=50.,initial_depth_evidence=None)
            exec(compile(ast.Module(body=nodes[first:end],type_ignores=[]),str(source),'exec'),env)
            group=env['initial_sequence_group'];axis=env['initial_axis']
            self.assertEqual(group['truss_key'],'Truss_01')
            self.assertEqual(group['fruits'],[fruits[2]['name']])
            self.assertEqual(group['measured_centroid_world_m'],positions[2].tolist())
            np.testing.assert_allclose(axis['direction_world'],[0.,1.,0.],atol=1e-12)
            self.assertFalse(axis['near_scope_axis']['other_truss_retained_axis_used'])
            self.assertEqual(env['initial_grasp_depth_mm'],65.)
            self.assertEqual(env['initial_height_mm'],-1.5)
            self.assertEqual(env['initial_depth_evidence']['completed_native_cycles'],0)
            evidence=json.loads((Path(temporary)/'initial_near_scope_selection.json').read_text())
            self.assertTrue(evidence['switched_truss'])
            self.assertEqual(evidence['reason'],
                'initial_truss_finite_attempt_or_rejection_exhaustion' if initial_exhausted
                else 'initial_truss_all_detached')
            self.assertEqual(env['initial_attempt_counts'],attempts)
            self.assertEqual(env['initial_inventory_breaks'],native_breaks)
            self.assertFalse(evidence['physical_state_reset'])
            if initial_exhausted:
                self.assertEqual(evidence['initial_truss_remaining'],[fruits[-1]['name']])
                self.assertEqual(evidence['initial_truss_attempt_exhausted'],[fruits[-1]['name']])
                self.assertFalse(any(row['joint']==fruits[-1]['joint'] for row in native_breaks))
                np.testing.assert_array_equal(env['initial_body_poses'],positions)


if __name__=='__main__':unittest.main()
