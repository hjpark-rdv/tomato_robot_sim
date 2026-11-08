import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from partial_recording import partial_recording, published_partial_source
from register_run import register


class PartialRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.assembly = self.root/'assembly'; self.assembly.mkdir()
        self.trial = self.root/'trial'; self.trial.mkdir()
        self.write(self.assembly/'scene.usda', b'actual source wrapper')
        self.write(self.assembly/'composed.usdc', b'actual materialized source')
        self.json(self.assembly/'motion_manifest.json', dict(scene_sha256=self.sha(self.assembly/'scene.usda')))
        native = dict(model_sha256=self.sha(self.assembly/'scene.usda'),
                      body_paths=['/World/Robot', '/World/Fruit'], robot_dofs=['lift', 'jaw_L', 'jaw_R'])
        self.json(self.trial/'native_metadata.json', native)
        self.json(self.trial/'runtime_settings.json', dict(source_scene_sha256=self.sha(self.assembly/'scene.usda'),
                  native_scene_sha256=self.sha(self.assembly/'composed.usdc'), device='cpu', hz=60, solver='TGS'))
        poses = np.zeros((3, 2, 7)); poses[:, :, 6] = 1; poses[1:, 0, 2] = [.01, .02]
        arrays = dict(time=np.array([0., .5, 1.45]), poses=poses, robot_q=np.zeros((3, 3)),
                      body_paths=np.array(native['body_paths']), phase=np.array(['initial', 'scan', 'scan']))
        np.savez_compressed(self.trial/'states_partial.npz', **arrays)
        self.json(self.trial/'break_events_partial.json', [])
        self.json(self.trial/'runtime_error.json', dict(error='Actual recorded software exception'))
        self.descriptor = dict(schema='s300-interrupted-actual-trace-v1', complete=False, completed=False,
            checkpoint_eligible=False, training_eligible=False, finalized_native_audit_claimed=False,
            source_assembly=str(self.assembly), actual_source_scene_sha256=self.sha(self.assembly/'scene.usda'),
            declared_wrapper_scene_sha256=self.sha(self.assembly/'scene.usda'),
            actual_manifest_sha256=self.sha(self.assembly/'motion_manifest.json'),
            states='states_partial.npz', states_sha256=self.sha(self.trial/'states_partial.npz'),
            break_events='break_events_partial.json', samples=3, first_timestamp_s=0., last_timestamp_s=1.45,
            array_shapes={key:list(value.shape) for key,value in arrays.items()})
        self.json(self.trial/'partial_trace.json', self.descriptor)

    def sha(self, path): return hashlib.sha256(path.read_bytes()).hexdigest()
    def write(self, path, data): path.write_bytes(data)
    def json(self, path, data): path.write_text(json.dumps(data))

    def test_exact_partial_binding_has_no_normal_summary_or_harvest_credit(self):
        before = {p.name:self.sha(p) for p in self.trial.iterdir()}
        record = partial_recording(self.assembly, self.trial)
        self.assertEqual(record['result']['recorded_samples'], 3)
        self.assertEqual(record['result']['simulated_s'], 1.45)
        self.assertFalse(record['result']['completed']); self.assertFalse(record['result']['checkpoint_eligible'])
        self.assertEqual(record['result']['harvest_credit'], 0)
        self.assertFalse(record['result']['training_eligible'])
        source = published_partial_source(record, self.trial)
        self.assertEqual(source['states_file'], 'states_partial.npz'); self.assertIsNone(source['summary_sha256'])
        self.assertFalse((self.trial/'summary.json').exists())
        self.assertEqual(before, {p.name:self.sha(p) for p in self.trial.iterdir()})

    def test_partial_model_state_and_mapping_mismatches_are_rejected(self):
        for field,value,message in [('states_sha256','0'*64,'states SHA'),
                ('actual_manifest_sha256','0'*64,'source SHA'),
                ('last_timestamp_s',2.,'time bounds'),('complete',True,'incomplete'),
                ('states','../states_partial.npz','filenames')]:
            with self.subTest(field=field):
                changed=dict(self.descriptor); changed[field]=value
                self.json(self.trial/'partial_trace.json', changed)
                with self.assertRaisesRegex(ValueError,message): partial_recording(self.assembly,self.trial)
        self.json(self.trial/'partial_trace.json',self.descriptor)
        path=self.trial/'native_metadata.json';native=json.loads(path.read_text());native['body_paths'].reverse();self.json(path,native)
        with self.assertRaisesRegex(ValueError,'mapping mismatch'): partial_recording(self.assembly,self.trial)

    def test_partial_cannot_replace_an_existing_normal_record(self):
        self.json(self.trial/'summary.json', dict(completed=True))
        with self.assertRaisesRegex(ValueError,'canonical'): partial_recording(self.assembly,self.trial)

    def test_explicit_existing_registration_never_promotes_partial_to_normal(self):
        root=self.root/'web';root.mkdir();(root/'catalog.json').write_text(json.dumps(dict(schema='farmily-harvest-replay-catalog-v1',runs=[])))
        folder=root/'assets/fault';folder.mkdir(parents=True)
        partial=partial_recording(self.assembly,self.trial)
        replay=dict(source=published_partial_source(partial,self.trial),result_summary=partial['result'],duration_s=1.45,frames=3)
        self.json(folder/'replay.json',replay)
        with self.assertRaisesRegex(ValueError,'explicit opt-in'):
            register(self.assembly,self.trial,'fault','Partial software fault',root=root,existing=True)
        register(self.assembly,self.trial,'fault','Partial software fault',root=root,existing=True,allow_partial_diagnostic=True)
        row=json.loads((root/'catalog.json').read_text())['runs'][0]
        self.assertTrue(row['diagnostic_only']);self.assertEqual(row['harvest_credit'],0)
        self.assertFalse(row['checkpoint_eligible'])


if __name__ == '__main__': unittest.main()
