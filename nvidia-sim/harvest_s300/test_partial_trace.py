import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from partial_trace import preserve_partial_trace


class PartialTraceTests(unittest.TestCase):
    def runtime(self):
        return dict(times=[0.,.1],trace=np.array([[[0.,0.,1.,0,0,0,1]],[[.01,0.,1.,0,0,0,1]]]),
            qtrace=np.array([[0.,.2],[.1,.3]]),body_paths=['/World/Robot/link1'],
            phases_trace=['idle','camera_pan'],cycle_trace=[0,0],command=[[0.,.2],[.1,.3]],
            breaks=[dict(t=.1,joint='/World/Plant/joint')])

    def test_aligned_actual_samples_are_saved_as_incomplete_only(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'trial';source=Path(folder)/'assembly';source.mkdir()
            (source/'scene.usda').write_text('#usda 1.0')
            (source/'motion_manifest.json').write_text('{}')
            runtime=self.runtime();runtime['a']=SimpleNamespace(assembly=source)
            runtime['manifest']=dict(scene_sha256='declared_source_wrapper_sha')
            before=copy.deepcopy(runtime)
            report=preserve_partial_trace(output,runtime)
            self.assertFalse(report['complete']);self.assertFalse(report['completed'])
            self.assertFalse(report['training_eligible']);self.assertIsNone(report['harvest_success'])
            self.assertFalse(report['checkpoint_eligible']);self.assertFalse(report['finalized_native_audit_claimed'])
            self.assertEqual(report['last_timestamp_s'],.1)
            self.assertEqual(report['source_assembly'],str(source.resolve()))
            with np.load(output/'states_partial.npz',allow_pickle=False) as states:
                np.testing.assert_array_equal(states['poses'],runtime['trace'])
                np.testing.assert_array_equal(states['robot_q'],runtime['qtrace'])
                np.testing.assert_array_equal(states['time'],runtime['times'])
                np.testing.assert_array_equal(states['phase'],runtime['phases_trace'])
            np.testing.assert_array_equal(runtime['trace'],before['trace'])
            self.assertEqual(json.loads((output/'break_events_partial.json').read_text()),runtime['breaks'])
            self.assertFalse((output/'states.npz').exists());self.assertFalse((output/'summary.json').exists())

    def test_before_trace_initialization_creates_no_output(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'absent'
            self.assertIsNone(preserve_partial_trace(output,{}))
            self.assertIsNone(preserve_partial_trace(output,dict(times=[])))
            self.assertFalse(output.exists())

    def test_nonaligned_nonfinite_or_nonincreasing_history_is_rejected(self):
        changes=[dict(times=[0.,0.]),dict(times=[.1,0.]),dict(times=[0.,np.nan]),
                 dict(trace=np.zeros((1,1,7))),dict(qtrace=[[0.,.2],[np.inf,.3]]),
                 dict(phases_trace=['idle']),dict(cycle_trace=[0,.5]),
                 dict(body_paths=['/World/Robot/link1','/World/Robot/link1'])]
        with tempfile.TemporaryDirectory() as folder:
            for i,change in enumerate(changes):
                with self.subTest(change=change):
                    runtime=self.runtime();runtime.update(change);output=Path(folder)/str(i)
                    with self.assertRaises(ValueError):preserve_partial_trace(output,runtime)
                    self.assertFalse(output.exists())

    def test_existing_normal_and_partial_results_are_never_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)
            (output/'states.npz').write_bytes(b'original normal states')
            (output/'summary.json').write_text('{"completed":false}')
            report=preserve_partial_trace(output,self.runtime())
            self.assertTrue(report['normal_states_preserved']);self.assertTrue(report['normal_summary_preserved'])
            self.assertEqual((output/'states.npz').read_bytes(),b'original normal states')
            self.assertEqual((output/'summary.json').read_text(),'{"completed":false}')
            original=(output/'states_partial.npz').read_bytes()
            with self.assertRaises(FileExistsError):preserve_partial_trace(output,self.runtime())
            self.assertEqual((output/'states_partial.npz').read_bytes(),original)

    def test_unaligned_optional_commands_are_explicitly_omitted_without_losing_actual_state(self):
        with tempfile.TemporaryDirectory() as folder:
            runtime=self.runtime();runtime['command']=[[0.,.2]]
            report=preserve_partial_trace(folder,runtime)
            self.assertIn('command',report['omitted_optional_arrays'])
            with np.load(Path(folder)/'states_partial.npz',allow_pickle=False) as states:
                self.assertNotIn('command',states);self.assertIn('robot_q',states)


if __name__=='__main__':unittest.main()
