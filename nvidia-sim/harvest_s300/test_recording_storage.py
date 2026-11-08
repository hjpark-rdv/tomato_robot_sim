import base64
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from recording_storage import (save_array, save_arrays, load_array, original_bytes, compact_array,
    restore_array, save_shared_labels, share_file)
from compact_camera_storage import compact_camera, compact_web, inventory


class RecordingStorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_exact_float_bits_legacy_gzip_and_restore(self):
        # Include signed zero and distinct NaN payloads, not just allclose.
        array = np.array([0x80000000, 0x7fc00001, 0x7fc00202, 0x3f800001], dtype=np.uint32).view(np.float32)
        path = self.root/'depth.npy'
        np.save(path, array)
        raw = path.read_bytes()
        self.assertEqual(load_array(path).tobytes(), array.tobytes())
        compact_array(path)
        self.assertFalse(path.exists())
        self.assertEqual(original_bytes(path), raw)
        self.assertEqual(load_array(path).tobytes(), array.tobytes())
        self.assertEqual(load_array(path.with_name('depth.npy.gz')).tobytes(), array.tobytes())
        self.assertEqual(restore_array(path).read_bytes(), raw)

    def test_new_big_endian_fortran_arrays_and_uint_ids(self):
        for name, array in [('depth.npy', np.asfortranarray(np.ones((5,3), dtype='>f4'))),
                            ('instance.npy', np.array([[0, 4294967295]], np.uint32))]:
            actual = save_array(self.root/name, array)
            loaded = load_array(actual)
            self.assertEqual(loaded.dtype, array.dtype)
            self.assertEqual(loaded.shape, array.shape)
            self.assertEqual(loaded.tobytes(), array.tobytes())
            with self.assertRaises(FileExistsError): save_array(self.root/name, array)

    def test_grouped_write_joins_all_owned_arrays_and_exact_labels(self):
        arrays={'depth.npy':np.array([[np.nan,1.234]],np.float32),
                'instance.npy':np.array([[0,0xffffffff]],np.uint32)}
        verified=set()
        save_arrays(self.root,arrays,labels={'7':'fruit'},label_pool=self.root/'shared',verified=verified)
        for name,array in arrays.items():
            self.assertEqual(load_array(self.root/name).tobytes(),array.tobytes())
        self.assertEqual(json.loads((self.root/'renderer_labels.json').read_text()),{'7':'fruit'})
        self.assertEqual(len(verified),1)
        with self.assertRaises(FileExistsError):save_arrays(self.root,arrays)

    def test_conflicting_archive_never_removes_original(self):
        path = self.root/'depth.npy'
        np.save(path, np.ones((5,5)))
        raw = path.read_bytes()
        path.with_name('depth.npy.gz').write_bytes(gzip.compress(b'wrong bytes'))
        with self.assertRaises(ValueError): compact_array(path)
        self.assertEqual(path.read_bytes(), raw)

    def test_corrupt_or_object_array_is_rejected_without_delete(self):
        path = self.root/'depth.npy'
        np.save(path, np.array([{'bad':'pickle'}], dtype=object))
        with self.assertRaises(ValueError): compact_array(path)
        self.assertTrue(path.is_file())

    def test_labels_share_only_exact_content_and_survive_dataset_move(self):
        root = self.root/'rgbd'; root.mkdir()
        for i, labels in enumerate([{'7':'fruit'}, {'7':'fruit'}, {'7':'stem'}]):
            frame=root/f'frame_{i:05d}';frame.mkdir()
            save_shared_labels(frame/'renderer_labels.json',labels,root/'shared_labels')
        self.assertEqual(len(list((root/'shared_labels').glob('*.json'))),2)
        root.rename(self.root/'moved')
        for i, value in enumerate(['fruit','fruit','stem']):
            self.assertEqual(json.loads((self.root/'moved'/f'frame_{i:05d}'/'renderer_labels.json').read_text()),{'7':value})

    def test_export_sharing_never_links_source_capture(self):
        source=self.root/'source.png';source.write_bytes(b'original pixels')
        a=self.root/'a.png';b=self.root/'b.png'
        a.write_bytes(source.read_bytes());b.write_bytes(source.read_bytes())
        share_file(a,self.root/'pool');share_file(b,self.root/'pool')
        self.assertEqual(a.stat().st_ino,b.stat().st_ino)
        self.assertNotEqual(a.stat().st_ino,source.stat().st_ino)

    def report(self):
        return dict(arrays=[],labels=[],models=[],unmodified_noncanonical_labels=[],
                    metadata_backups={},shared_files=0,report_path=str(self.root/'audit.json'))

    def test_migration_dry_run_then_preserves_original_manifest_hash(self):
        capture=self.root/'rgbd';capture.mkdir()
        frame=capture/'frame_00000';frame.mkdir()
        for name in ('depth.npy','instance.npy','semantic.npy','renderer_instance.npy'):
            np.save(frame/name,np.zeros((20,20),np.float32))
        (frame/'renderer_labels.json').write_text(json.dumps({'7':'fruit'},indent=2)+'\n')
        log=capture/'frames.jsonl';log.write_text('{"depth":"frame_00000/depth.npy"}\n')
        before=hashlib.sha256(log.read_bytes()).hexdigest()
        compact_camera(self.root,self.report(),False)
        self.assertTrue((frame/'depth.npy').exists())
        compact_camera(self.root,self.report(),True)
        self.assertFalse((frame/'depth.npy').exists())
        self.assertEqual(hashlib.sha256(log.read_bytes()).hexdigest(),before)
        report=self.report();compact_camera(self.root,report,True)
        self.assertEqual(report['arrays'],[])

    def test_interrupted_frame_resumes_after_depth_was_compacted(self):
        frame=self.root/'rgbd/frame_00000';frame.mkdir(parents=True)
        for name in ('depth.npy','instance.npy'):
            np.save(frame/name,np.zeros((10,10),np.float32))
        compact_array(frame/'depth.npy')
        report=self.report();compact_camera(self.root,report,True)
        self.assertEqual(len(report['arrays']),1)
        self.assertTrue((frame/'instance.npy.gz').is_file())

    def test_web_lossless_scene_survives_without_plain_glb_and_bad_hash_blocks(self):
        for i in range(2):
            d=self.root/f'run{i}';d.mkdir()
            raw=b'geometry, topology, and material bytes'*100
            packed=gzip.compress(raw,mtime=0)
            (d/'scene.glb').write_bytes(raw);(d/'scene.glb.gz').write_bytes(packed)
            (d/'replay.json').write_text(json.dumps(dict(scene_url='scene.glb',scene_info_url='replay_scene.json')))
            (d/'replay_scene.json').write_text(json.dumps(dict(sha256=hashlib.sha256(raw).hexdigest(),compressed_sha256=hashlib.sha256(packed).hexdigest())))
        before=inventory(self.root)['allocated_bytes']
        report=self.report();compact_web(self.root,report,True)
        for i in range(2):
            d=self.root/f'run{i}'
            self.assertFalse((d/'scene.glb').exists())
            record=json.loads((d/'replay.json').read_text())
            self.assertEqual(gzip.decompress((d/record['scene_url']).read_bytes()),raw)
            old=json.loads(gzip.decompress(base64.b64decode(report['metadata_backups'][str(d/'replay.json')]['data'])))
            self.assertEqual(old['scene_url'],'scene.glb')
        self.assertEqual((self.root/'run0/scene.glb.gz').stat().st_ino,(self.root/'run1/scene.glb.gz').stat().st_ino)
        d=self.root/'bad';d.mkdir()
        (d/'scene.glb').write_bytes(raw);(d/'scene.glb.gz').write_bytes(gzip.compress(b'corrupt'))
        (d/'replay.json').write_bytes((self.root/'run0/replay.json').read_bytes())
        (d/'replay_scene.json').write_bytes((self.root/'run0/replay_scene.json').read_bytes())
        with self.assertRaises(ValueError):compact_web(self.root,self.report(),True)
        self.assertTrue((d/'scene.glb').is_file())


if __name__ == '__main__':unittest.main()
