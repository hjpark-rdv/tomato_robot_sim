"""Execute both actual startup restore branches without starting Kit or PhysX."""
import ast
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


SOURCE = Path(__file__).with_name('run_motion.py')


def assigned(node, name):
    return isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == name for target in node.targets)


def actual_startup_nodes():
    tree = ast.parse(SOURCE.read_text())
    main = next(node for node in tree.body if isinstance(node, ast.Try)).body
    first = next(i for i, node in enumerate(main) if assigned(node, 'resume_context'))
    # Include the actual common import, if present. A resume-conditional import
    # must not be injected by the test: that is the production bug being caught.
    while first and isinstance(main[first-1], ast.ImportFrom):
        first -= 1
    last = next(i for i in range(first, len(main)) if isinstance(main[i], ast.ImportFrom)
                and main[i].module == 'motion_stage_metrics')
    selection = next(i for i, node in enumerate(main) if assigned(node, 'initial_robot_q'))
    end = next(i for i in range(selection, len(main)) if isinstance(main[i], ast.ImportFrom)
               and main[i].module == 'truss_sequence')
    return main[first:last] + main[selection:end]


class RouteStartInventoryImportTests(unittest.TestCase):
    def context(self, breaks=()):
        return dict(metadata=dict(scene_sha256='exact-source', time_s=11.,
            break_events=list(breaks), sequence=dict(cycle_plans=[], attempt_counts={})),
            arrays=dict(robot_dof_names=np.array(['lift', 'arm']), robot_q=np.array([.03, .2]),
                body_paths=np.array(['/active/Fruit_01']),
                body_poses_world=np.array([[1., 2., 3., 0., 0., 0., 1.]])))

    def run_startup(self, *, resume=None, route=None):
        with tempfile.TemporaryDirectory() as output:
            args = SimpleNamespace(resume_native_checkpoint=Path('/exact/checkpoint') if resume else None,
                assembly=Path('/exact/assembly'), output=Path(output), num_envs=1, replicate=False,
                mode='idle', allow_resume_leaf_proxy_revision=False,
                allow_resume_accuracy_proxy_revision=False, allow_resume_attachment_transfer=False,
                allow_resume_conditioned_start=False, resume_original_cycle_suffix=False,
                allow_resume_target_truss_revision=False)
            manifest = dict(initial_robot_q=[.03, .2])
            if route is not None:
                manifest['route_checkpoint'] = dict(schema='checked-route')
            env = dict(np=np, json=json, a=args, stage=object(), manifest=manifest,
                kin=SimpleNamespace(names=['lift', 'arm']))
            with patch('native_route_start.prepare_route_start', return_value=route) as prepare_route, \
                    patch('resume_native_state.prepare_resume', return_value=resume) as prepare_resume:
                exec(compile(ast.fix_missing_locations(ast.Module(body=actual_startup_nodes(),
                    type_ignores=[])), str(SOURCE), 'exec'), env)
            return env, prepare_route.call_count, prepare_resume.call_count

    def test_actual_new_window_reactivation_imports_inventory_without_same_asset_resume(self):
        restored = self.context()
        route = dict(unharvested_reactivation=restored, report=dict(validated=True),
            controller_state=dict(position_targets=np.array([.03, .2])))
        before = copy.deepcopy(restored)
        env, route_calls, resume_calls = self.run_startup(route=route)
        self.assertEqual((route_calls, resume_calls), (1, 0))
        self.assertIsNone(env['resume_context'])
        self.assertEqual(env['initial_inventory_breaks'], [])
        self.assertEqual(env['initial_body_paths'], ['/active/Fruit_01'])
        self.assertEqual(env['initial_attempt_counts'], {})
        np.testing.assert_array_equal(env['initial_body_poses'], before['arrays']['body_poses_world'])
        np.testing.assert_array_equal(restored['arrays']['body_poses_world'], before['arrays']['body_poses_world'])

    def test_same_asset_resume_still_retains_exact_prior_breaks_q_and_attempt_counts(self):
        event = dict(t=2.5, joint='/active/joint01', type='native_joint_break')
        context = self.context([event])
        context['metadata']['sequence']['attempt_counts'] = {'Fruit_01': 2}
        before = copy.deepcopy(context)
        env, route_calls, resume_calls = self.run_startup(resume=context)
        self.assertEqual((route_calls, resume_calls), (0, 1))
        marker = env['initial_inventory_breaks'][0]
        self.assertEqual(marker['original_event'], event)
        self.assertFalse(marker['native_new_event'])
        self.assertEqual(marker['t'], 0.)
        self.assertEqual(env['initial_attempt_counts'], {'Fruit_01': 2})
        np.testing.assert_array_equal(env['initial_robot_q'], before['arrays']['robot_q'])
        self.assertEqual(context['metadata'], before['metadata'])

    def test_new_never_archived_window_does_not_invent_inventory(self):
        route = dict(unharvested_reactivation=None, report=dict(validated=True),
            controller_state=dict(position_targets=np.array([.03, .2])))
        env, route_calls, resume_calls = self.run_startup(route=route)
        self.assertEqual((route_calls, resume_calls), (1, 0))
        self.assertEqual(env['initial_inventory_breaks'], [])
        self.assertIsNone(env['initial_body_paths'])
        self.assertIsNone(env['initial_body_poses'])

    def test_fresh_start_without_route_or_resume_preserves_initial_state(self):
        env, route_calls, resume_calls = self.run_startup()
        self.assertEqual((route_calls, resume_calls), (0, 0))
        self.assertEqual(env['initial_inventory_breaks'], [])
        np.testing.assert_array_equal(env['initial_robot_q'], [.03, .2])


if __name__ == '__main__':
    unittest.main()
