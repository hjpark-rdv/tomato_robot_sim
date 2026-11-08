import gzip
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from scene_reuse import reference_identity,verify_reference_proof,sha


class ExactSceneReuseTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.assembly=self.root/'assembly';self.trial=self.root/'actual';self.bundle=self.root/'bundle'
        for folder in (self.assembly,self.trial,self.bundle):folder.mkdir()
        (self.assembly/'composed.usdc').write_bytes(b'exact physical source')
        (self.assembly/'motion_manifest.json').write_text('{}')
        poses=np.zeros((2,2,7));poses[:,:,6]=1;poses[1,0,1]=.15
        np.savez(self.trial/'states.npz',body_paths=np.array(['rail','fruit']),poses=poses)
        (self.bundle/'scene.glb.gz').write_bytes(gzip.compress(b'identity fixture; full GLB is checked by independent validator'))
        self.scene=sha(self.assembly/'composed.usdc');self.manifest=sha(self.assembly/'motion_manifest.json')
        self.info=dict(source_scene_sha256=self.scene,fixture=None,compressed_sha256=sha(self.bundle/'scene.glb.gz'),sha256='full-glb-sha')
        (self.bundle/'replay_scene.json').write_text(json.dumps(self.info))
        self.replay=dict(schema='farmily-harvest-replay-v1',source_geometry_modified=False,
            source=dict(scene_sha256=self.scene,manifest_sha256=self.manifest,trial=str(self.trial),
                assembly=str(self.assembly),states_sha256=sha(self.trial/'states.npz'),
                all_frames_exported=True,physics_rerun=False),scene_info_url='replay_scene.json',scene_url='scene.glb.gz')
        self.save()

    def tearDown(self):self.temp.cleanup()
    def save(self):(self.bundle/'replay.json').write_text(json.dumps(self.replay))
    def identify(self,**kwargs):
        return reference_identity(self.bundle,scene_sha256=kwargs.get('scene',self.scene),
            manifest_sha256=kwargs.get('manifest',self.manifest),body_paths=kwargs.get('paths',['rail','fruit']))

    def test_exact_source_and_complete_body_order_bind_the_reference(self):
        proof,info,glb,poses=self.identify()
        self.assertEqual(info,self.info);self.assertEqual(poses.shape,(2,7))
        self.assertEqual(glb,self.bundle/'scene.glb.gz')
        verify_reference_proof(proof,scene_sha256=self.scene,manifest_sha256=self.manifest,body_paths=['rail','fruit'])

    def test_different_native_scene_manifest_or_mapping_cannot_reuse_geometry(self):
        for arguments in ({'scene':'other'},{'manifest':'other'},{'paths':['fruit','rail']},{'paths':['rail']}):
            with self.assertRaises(ValueError):self.identify(**arguments)

    def test_cache_chains_and_partial_or_missing_frames_are_rejected(self):
        for change in ('chain','partial','frames'):
            if change=='chain':self.replay['geometry_reuse']={'untrusted':True}
            if change=='partial':self.replay['source']['record_kind']='partial_diagnostic'
            if change=='frames':self.replay['source']['all_frames_exported']=False
            self.save()
            with self.assertRaises(ValueError):self.identify()
            self.replay.pop('geometry_reuse',None);self.replay['source'].pop('record_kind',None)
            self.replay['source']['all_frames_exported']=True

    def test_changed_reference_bytes_and_fixtures_are_rejected(self):
        proof,*_=self.identify();(self.bundle/'scene.glb.gz').write_bytes(b'changed')
        with self.assertRaises(ValueError):self.identify()
        with self.assertRaises(ValueError):verify_reference_proof(proof,scene_sha256=self.scene,
            manifest_sha256=self.manifest,body_paths=['rail','fruit'])

    def test_same_geometry_name_with_changed_physical_source_is_rejected(self):
        (self.assembly/'composed.usdc').write_bytes(b'changed collision model')
        with self.assertRaises(ValueError):self.identify()


if __name__=='__main__':unittest.main()
