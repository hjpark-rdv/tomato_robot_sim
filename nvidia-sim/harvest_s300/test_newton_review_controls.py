"""CPU control-flow checks from independent comparison review.

The mocked CUDA-shaped owner tests only no-op ownership: it is not GPU algebra
or farm physics evidence. Schedule qualification uses real saved-file parsing.
"""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from mujoco_warp._src import smooth

import newton_gpu_inertia_cusolver as inertia
from benchmark_solver_compare import summarize_run


class IndependentReviewControls(unittest.TestCase):
    def test_dense_only_owner_does_not_allocate_cusolver_or_touch_hooks(self):
        device = SimpleNamespace(is_cuda=True)
        solver = SimpleNamespace(use_mujoco_cpu=False, _step=0,
            mj_model=SimpleNamespace(nv=6),
            mjw_model=SimpleNamespace(nv_pad=8, qLD_block_total=36),
            mjw_data=SimpleNamespace(nworld=1, qpos=SimpleNamespace(device=device),
                                    qLD=SimpleNamespace(shape=(1, 36))))
        original = smooth._factor_i_sparse, smooth._solve_LD_sparse, smooth.solve_LD
        with patch.object(inertia.wp, 'get_stream', return_value=SimpleNamespace(is_capturing=False)), \
             patch.object(inertia, 'CuSolverAPI', side_effect=AssertionError('No cuSOLVER allocation permitted')):
            owner = inertia.CuSolverInertiaFactor(solver)
            self.assertTrue(owner.native_dense_only)
            self.assertFalse(owner.evidence['enabled'])
            owner.check_health()
            owner.close()
            owner.close()
        self.assertIsNone(inertia._owner)
        self.assertEqual(original, (smooth._factor_i_sparse, smooth._solve_LD_sparse, smooth.solve_LD))

    def _schedule(self, path, *, final_phase='retreat', samples=4, profile=False):
        summary = dict(status='completed', stop=None, worlds=1, actual_steps=3,
            requested_steps=3, profile_instrumented=profile,
            active_contact_criterion_passed=True, finite_recorded_state=True,
            solver_capacity_valid=True, qacc_finite=True,
            engine=dict(engine='newton.solvers.SolverMuJoCo.step', world_count=1,
                source_fields_preserved=True, physics_meshes_and_colliders_preserved=True,
                source_model_array_sha256='model'),
            input_sha256='input', commands_sha256='command', executed_commands_sha256='executed',
            loop_wall_s=2., scheduled_simulated_s=3., final_time_per_world_s=[3.],
            max_active_penetration_m=.0009)
        (path/'summary.json').write_text(json.dumps(summary))
        np.savez(path/'executed_commands.npz', time_edges=np.arange(4.),
                 phase=np.array(['approach', 'belt', 'retreat']))
        np.savez(path/'states.npz', phase=np.array(['initial'] +
            ['belt'] * max(0, samples-2) + [final_phase]))

    def test_full_sequence_requires_expected_last_phase_and_every_state(self):
        with TemporaryDirectory() as temporary:
            path = Path(temporary)
            self._schedule(path)
            self.assertTrue(summarize_run(path, worlds=1)['clean_harvest_throughput_eligible'])
            self._schedule(path, final_phase='belt')
            row = summarize_run(path, worlds=1)
            self.assertTrue(row['throughput_eligible'])
            self.assertFalse(row['full_existing_sequence_completed'])
            self.assertFalse(row['clean_harvest_throughput_eligible'])
            self._schedule(path, samples=3)
            self.assertFalse(summarize_run(path, worlds=1)['full_existing_sequence_completed'])

    def test_profiled_complete_sequence_never_gets_clean_speed_credit(self):
        with TemporaryDirectory() as temporary:
            path = Path(temporary)
            self._schedule(path, profile=True)
            row = summarize_run(path, worlds=1)
            self.assertTrue(row['full_existing_sequence_completed'])
            self.assertFalse(row['clean_harvest_throughput_eligible'])


if __name__ == '__main__':
    unittest.main()
