"""Measured source-command and persistent-rail regression checks.

The numerical checks use saved source commands, not a second copy of the
sampling implementation. Server asset checks skip on installations without
the preserved Isaac trials; the NumPy-only import check is always available.
"""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from plan_adapter import DOF_NAMES, sample_plan


REPO = Path(__file__).resolve().parents[3]
DATA = Path('/root/docker_share/mujoko_debugging_data')
D_ASSEMBLY = DATA/'20261011_194707_plant_interaction_comparison/D_target_only'
D_TRIAL = DATA/'20261011_194707_plant_interaction_comparison/trials/D_target_only_connected'
NEXT_ASSEMBLY = DATA/'20261011_205000_target_only_row_harvest/assembly_step01_plant41'
NEXT_TRIAL = DATA/'20261011_205000_target_only_row_harvest/trials/step01_plant41_shallow_yaw_minus45_runtimefix'


def source_plan(trial):
    plan = json.loads((trial/'plan.json').read_text())
    with np.load(trial/'states.npz') as states:
        plan['initial_command'] = states['command'][0].tolist()
    plan.update(settle_seconds=2., initial_gap_m=.044)
    return plan


class PlanAdapterTests(unittest.TestCase):
    def require(self, *paths):
        if not all(Path(path).exists() for path in paths):
            self.skipTest('Preserved server source assets unavailable')

    def test_native_environment_import_does_not_require_usd(self):
        directory = str(Path(__file__).parent)
        subprocess.run([sys.executable, '-c',
            "import sys;sys.path.insert(0,sys.argv[1]);import plan_adapter;"
            "assert not any(k=='pxr' or k.startswith('pxr.') for k in sys.modules)",
            directory], check=True, capture_output=True, text=True)

    def test_saved_full_source_curve_belt_ramp_and_phase_are_preserved(self):
        self.require(D_TRIAL/'states.npz', D_TRIAL/'plan.json')
        plan = source_plan(D_TRIAL)
        with np.load(D_TRIAL/'states.npz') as states:
            measured_time = states['time']
            # Source logs the issued command alongside the resulting next-step
            # state. Evaluate at the command's pre-step time, not that state time.
            indices = np.flatnonzero(measured_time >= 2. + 1./60.)
            for index in indices:
                sample = sample_plan(plan, measured_time[index] - 1./60.)
                np.testing.assert_allclose(sample['command'], states['command'][index], atol=3e-7, rtol=0)
                np.testing.assert_allclose(sample['authored_belt_speed'],
                    states['belt_surface_velocity_local_y_m_s'][index], atol=3e-14, rtol=0)
                self.assertEqual(sample['phase'], str(states['phase'][index]))
        self.assertEqual(plan['belt_cycle']['close_open_repetitions'], 2)
        self.assertTrue(sample_plan(plan, 100.)['completed'])

    def test_second_station_replan_uses_measured_start_and_shared_rail_origin(self):
        asset_python = REPO/'nvidia-sim/.asset-venv/bin/python'
        self.require(asset_python, D_ASSEMBLY/'motion_manifest.json',
                     NEXT_ASSEMBLY/'motion_manifest.json', NEXT_TRIAL/'plan.json')
        source = json.loads((NEXT_TRIAL/'plan.json').read_text())
        manifest = json.loads((NEXT_ASSEMBLY/'motion_manifest.json').read_text())
        origin = json.loads((D_ASSEMBLY/'motion_manifest.json').read_text())['rail']['initial_station_y_m']
        # A new runtime may arrive a little away from the recorded station. The
        # planner must preserve this measurement; no source-origin reset/teleport.
        measured_station = manifest['rail']['initial_station_y_m'] + .003
        request = dict(measured_joint_q=source['start'], measured_station_y_m=measured_station,
            rail_origin_station_m=origin, station_y_m=source['rail_destination']['station_y_m'],
            target_center=source['target_center_world_m'], direction=source['approach_direction_world'],
            grasp_translation=source['grasp_translation_world_m'], measured_jaw_q=[-.026, .026])
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory);request_file=directory/'request.json';output=directory/'plan.json'
            request_file.write_text(json.dumps(request))
            process=subprocess.run([str(asset_python), str(Path(__file__).with_name('plan_adapter.py')),
                str(NEXT_ASSEMBLY),str(request_file),str(output),'--source-plan',str(NEXT_TRIAL/'plan.json')],
                check=True, capture_output=True, text=True)
            plan=json.loads(output.read_text())
        self.assertFalse(plan['collision_certified'])
        self.assertEqual(plan['dof_names'], DOF_NAMES)
        self.assertEqual(plan['measured_initial_joint_q'], request['measured_joint_q'])
        first=sample_plan(plan,0.)
        rail_start=sample_plan(plan,plan['settle_seconds'])
        final=sample_plan(plan,plan['total_duration_s'])
        self.assertAlmostEqual(first['command'][0], measured_station-origin, places=12)
        self.assertAlmostEqual(rail_start['command'][0], first['command'][0], places=12)
        np.testing.assert_array_equal(rail_start['command'][1:8], request['measured_joint_q'])
        self.assertAlmostEqual(final['command'][0], request['station_y_m']-origin, places=12)
        self.assertGreater(final['command'][0], 1.7)  # Not the old window-local 0.071m.
        self.assertEqual(plan['belt_cycle']['close_open_repetitions'], 2)

    def test_invalid_clock_is_rejected_before_command(self):
        for time_s in (-1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                sample_plan({}, time_s)


if __name__ == '__main__':
    unittest.main()
