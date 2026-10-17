"""CPU controls for runner verification and exact-byte active-window assets."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

import newton_asset_cache as cache
import newton_prepared_assets as assets
import run_newton_harvest as runner
from newton_active_window import prepare
from newton_inputs import sha256
from newton_window_continuation import _prepared
from test_newton_active_window import fixture


class PreparedAssetIntegrationControls(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.inputs = self.root / 'inputs'
        self.inputs.mkdir()
        (self.inputs / 'input_model.mjb').write_bytes(b'compiled model')
        (self.inputs / 'meshes').mkdir()
        (self.inputs / 'meshes/a.obj').write_bytes(b'mesh a')
        self.proof = dict(input_files={name: dict(path=str(self.inputs / name),
            sha256=sha256(self.inputs / name)) for name in ('input_model.mjb', 'meshes/a.obj')})

    def test_uncached_runner_hashes_every_declared_source_once(self):
        with mock.patch.object(assets, '_hash_file', wraps=assets._hash_file) as hashed:
            model, evidence, digests = runner.verify_prepared_inputs(self.inputs, self.proof)
        self.assertEqual(model, self.inputs / 'input_model.mjb')
        self.assertIsNone(evidence)
        self.assertCountEqual([call.args[0] for call in hashed.call_args_list],
                              [self.inputs / name for name in self.proof['input_files']])
        self.assertEqual(digests, {name: value['sha256'] for name, value in self.proof['input_files'].items()})

    def test_cached_runner_hashes_model_only_in_staging_and_preserves_origin(self):
        for expected_hit in (False, True):
            with mock.patch.object(assets, '_hash_file', wraps=assets._hash_file) as hashed, \
                 mock.patch.object(cache, '_hash_stream', wraps=cache._hash_stream) as streams:
                model, evidence, digests = runner.verify_prepared_inputs(
                    self.inputs, self.proof, self.root / 'cache')
            self.assertEqual([call.args[0] for call in hashed.call_args_list], [self.inputs / 'meshes/a.obj'])
            self.assertEqual(streams.call_count, 2)  # One original read, one independent cache read.
            self.assertEqual(evidence['cache_hit'], expected_hit)
            self.assertEqual(evidence['origin_requested'], str(self.inputs / 'input_model.mjb'))
            self.assertEqual(model.read_bytes(), (self.inputs / 'input_model.mjb').read_bytes())
            self.assertEqual(digests['input_model.mjb'], self.proof['input_files']['input_model.mjb']['sha256'])

    def test_cache_does_not_allow_model_or_non_model_source_tampering(self):
        runner.verify_prepared_inputs(self.inputs, self.proof, self.root / 'cache')
        (self.inputs / 'input_model.mjb').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'Source model SHA mismatch'):
            runner.verify_prepared_inputs(self.inputs, self.proof, self.root / 'cache')
        (self.inputs / 'input_model.mjb').write_bytes(b'compiled model')
        (self.inputs / 'meshes/a.obj').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
            runner.verify_prepared_inputs(self.inputs, self.proof, self.root / 'cache')

    def test_missing_model_digest_and_unsafe_manifest_paths_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'physical MJB'):
            runner.verify_prepared_inputs(self.inputs, dict(input_files={}))
        proof = dict(input_files=dict(self.proof['input_files']))
        proof['input_files']['../escaped'] = dict(sha256='0' * 64)
        with self.assertRaisesRegex(ValueError, 'relative path'):
            runner.verify_prepared_inputs(self.inputs, proof, self.root / 'cache')
        self.assertFalse((self.root / 'cache').exists())

    def test_transition_verification_keeps_manifest_and_rejects_linked_asset_tampering(self):
        linked = self.root / 'linked'
        assets.verify_manifest_files(self.inputs, self.proof['input_files'])
        assets.link_manifest_assets(self.inputs, linked, self.proof['input_files'])
        (linked / 'inputs.json').write_text(json.dumps(self.proof))
        self.assertEqual(_prepared(linked), self.proof)
        (self.inputs / 'meshes/a.obj').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'Transition prepared input SHA'):
            _prepared(linked)

    def test_active_window_compiles_original_mesh_bytes_through_shared_directory(self):
        source, output = self.root / 'full', self.root / 'reduced'
        fixture(source)
        mesh = source / 'native_tool' / 'nested' / 'tetra.obj'
        mesh.parent.mkdir(parents=True)
        mesh.write_text('v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\n'
                        'f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
        tree = ET.parse(source / 'input_scene.xml')
        asset = ET.SubElement(tree.getroot(), 'asset')
        ET.SubElement(asset, 'mesh', name='tool_mesh', file='native_tool/nested/tetra.obj')
        geom = tree.getroot().find('.//geom[@name="robot_g"]')
        del geom.attrib['size']
        geom.set('type', 'mesh'); geom.set('mesh', 'tool_mesh')
        tree.write(source / 'input_scene.xml', encoding='unicode')
        original = mj.MjModel.from_xml_path(str(source / 'input_scene.xml'))
        mj.mj_saveModel(original, str(source / 'input_model.mjb'))
        manifest = json.loads((source / 'inputs.json').read_text())
        manifest['input_files']['native_tool/nested/tetra.obj'] = {}
        for name in manifest['input_files']:
            manifest['input_files'][name] = dict(sha256=sha256(source / name))
        (source / 'inputs.json').write_text(json.dumps(manifest))
        before = {name: (source / name).read_bytes() for name in manifest['input_files']}
        proof = prepare(source, output, '22')
        self.assertTrue((output / 'native_tool').is_symlink())
        self.assertEqual((output / 'native_tool').resolve(), source / 'native_tool')
        self.assertTrue(proof['protected_physics']['passed'])
        reduced = mj.MjModel.from_binary_path(str(output / 'input_model.mjb'))
        for name in ('mesh_vert', 'mesh_face', 'mesh_pos', 'mesh_quat', 'geom_pos', 'geom_quat'):
            np.testing.assert_array_equal(getattr(original, name), getattr(reduced, name))
        for name in ('input_scene.xml', 'input_model.mjb', 'initial_state.npz', 'source_export.json',
                     'canonical_full_state.npz', 'active_window_proof.json'):
            self.assertFalse((output / name).is_symlink())
        for name, content in before.items():
            self.assertEqual((source / name).read_bytes(), content)
        result = json.loads((output / 'inputs.json').read_text())
        self.assertEqual(result['canonical_prepared_files_sha256'],
                         {name: entry['sha256'] for name, entry in manifest['input_files'].items()})
        assets.verify_manifest_files(output, result['input_files'])


if __name__ == '__main__':
    unittest.main()
