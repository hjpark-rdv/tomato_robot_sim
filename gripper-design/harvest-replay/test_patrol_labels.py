import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from patrol_labels import segment_label


class ActualPatrolLabels(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.trial=Path(self.temp.name)
        self.row=dict(index=4,campaign='actual',phase='harvest')
        self.proof=dict(trial=str(self.trial),summary_sha256='actualsummary',states_sha256='actualstates')
        self.cycle=dict(status='planning_rejected',recorded_physical_cycle=False,planned_manipulation_started=False)

    def tearDown(self):self.temp.cleanup()
    def save(self,phases=('initial','camera_pan'),cycles=None):
        (self.trial/'summary.json').write_text(json.dumps(dict(completed=True,
            selected_cycle_evidence={'cycles':[self.cycle] if cycles is None else cycles})))
        np.savez(self.trial/'states.npz',time=np.array([0.,5.]),phase=np.array(phases))

    def test_rejected_plans_with_only_camera_frames_are_not_actual_harvest_motion(self):
        self.save();label,description,evidence=segment_label(self.row,self.proof)
        self.assertIn('접근계획 차단·관측5초, 실제수확0회',label)
        self.assertIn('실제 수확 동작은 0회',description)
        self.assertEqual(evidence['unexecuted_planning_rejections'],1)
        self.assertFalse(evidence['physical_harvest_success_claimed'])
        self.assertEqual(evidence['states_sha256'],'actualstates')

    def test_any_actual_approach_or_manipulation_cannot_receive_observation_only_label(self):
        for phase in ('approach','close','belt_spin','cycle_reclose'):
            self.save(phases=('initial',phase))
            label,description,evidence=segment_label(self.row,self.proof)
            self.assertNotIn('실제수확0회',label);self.assertIsNone(evidence);self.assertIsNone(description)

    def test_missing_or_partially_executed_cycle_evidence_does_not_invent_zero_attempts(self):
        for cycles in ([],[dict(self.cycle,recorded_physical_cycle=True)],
                [dict(self.cycle,planned_manipulation_started=True)],[{'status':'planning_rejected'}]):
            self.save(cycles=cycles);label,_,evidence=segment_label(self.row,self.proof)
            self.assertNotIn('실제수확0회',label);self.assertIsNone(evidence)

    def test_partial_diagnostic_never_opens_or_invents_normal_summary(self):
        self.proof.update(record_kind='partial_diagnostic',physical_stop_reason='incomplete')
        label,description,evidence=segment_label(self.row,self.proof)
        self.assertIn('중단 incomplete',label);self.assertIsNone(evidence)
        self.assertFalse((self.trial/'summary.json').exists())


if __name__=='__main__':unittest.main()
