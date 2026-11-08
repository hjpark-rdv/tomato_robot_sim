"""Camera replay checks use a tiny recorded fixture, not a simulator."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

from sensor_replay import export_camera, sha, source_frames, validate_camera


def write(path, value):
    path.write_text(json.dumps(value)+'\n')


class CameraReplay(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.assembly, self.trial, self.output = [self.root/name for name in ('assembly','trial','web')]
        for path in (self.assembly,self.trial,self.output):
            path.mkdir()
        (self.assembly/'composed.usdc').write_bytes(b'physical scene identity')
        write(self.trial/'runtime_settings.json',{'native_scene_sha256':sha(self.assembly/'composed.usdc')})
        poses=np.array([[[1.,2.,3.,0.,0.,0.,1.]],[[1.,2.,4.,0.,0.,0.,1.]]],dtype=np.float32)
        q=np.array([[.1,.2],[.2,.3]],dtype=np.float32)
        np.savez(self.trial/'states.npz',time=np.array([0.,.5]),poses=poses,robot_q=q,body_paths=['/body'])
        self.sensor = self.trial/'rgbd'
        self.sensor.mkdir()
        write(self.sensor/'camera.json',dict(schema='s300-mounted-rgbd-v1',frames=2,
              body_path='/body',body_from_camera_optical=np.eye(4).tolist(),resolution_wh=[4,6]))
        self.rows=[]
        for index in range(2):
            folder=self.sensor/f'frame_{index:05d}';folder.mkdir()
            for name in ('rgb.png','recognition.png','depth_overlay.png'):
                Image.new('RGB',(4,6),(100,120,index)).save(folder/name)
            write(folder/'camera.json',{'example':True});write(folder/'objects.json',[])
            tf=np.eye(4);tf[:3,3]=poses[index,0,:3]
            self.rows.append(dict(timestamp_s=index*.5,phase='move',physics_step=index*60,
              frame_id=index,measured_mount_body_pose=poses[index,0].tolist(),robot_q=q[index].tolist(),
              world_from_camera_optical=tf.tolist(),rgb=f'{folder.name}/rgb.png',
              gt=dict(recognition='recognition.png',depth_overlay='depth_overlay.png',objects='objects.json')))
        self.write_rows()

    def write_rows(self):
        (self.sensor/'frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.rows))

    def tearDown(self):
        self.temp.cleanup()

    def test_exact_copied_pixels_and_timestamps_and_tamper_rejected(self):
        descriptor=export_camera(self.assembly,self.trial,self.output)
        result=validate_camera(self.output/'replay.json',descriptor,self.assembly,self.trial)
        self.assertEqual(result['frames'],2);self.assertEqual(result['images'],6)
        pixels=self.output/'camera/frame_00000/rgb.png'
        self.assertEqual(pixels.read_bytes(),(self.sensor/'frame_00000/rgb.png').read_bytes())
        pixels.write_bytes(b'tampered camera pixels')
        with self.assertRaisesRegex(ValueError,'SHA mismatch'):
            validate_camera(self.output/'replay.json',descriptor,self.assembly,self.trial)

    def test_unrelated_camera_pose_and_timestamp_rejected(self):
        self.rows[1]['measured_mount_body_pose'][0]+=1.;self.write_rows()
        with self.assertRaisesRegex(ValueError,'pose differs'):
            source_frames(self.assembly,self.trial)
        self.rows[1]['measured_mount_body_pose'][0]-=1.
        self.rows[1]['timestamp_s']=.25;self.write_rows()
        with self.assertRaisesRegex(ValueError,'no exact recorded state'):
            source_frames(self.assembly,self.trial)

    def test_selected_region_keeps_visible_overlay_and_original_gt(self):
        folder=self.sensor/'frame_00000'
        Image.new('RGB',(4,6),(255,40,215)).save(folder/'harvest_region.png')
        write(folder/'region.json',{'visible_selected_count':1,'hidden_masks_fabricated':False})
        self.rows[0]['gt'].update(recognition='harvest_region.png',
                                 original_recognition='recognition.png',region='region.json')
        self.write_rows()
        descriptor=export_camera(self.assembly,self.trial,self.output)
        result=validate_camera(self.output/'replay.json',descriptor,self.assembly,self.trial)
        self.assertEqual(result['images'],7)
        frames=json.loads((self.output/'camera/frames.json').read_text())['frames']
        self.assertEqual(frames[0]['recognition'],'frame_00000/harvest_region.png')
        self.assertEqual(frames[0]['original_recognition'],'frame_00000/recognition.png')
        self.assertEqual((self.output/'camera/frame_00000/region.json').read_bytes(),(folder/'region.json').read_bytes())

    def test_external_camera_requires_state_and_scene_identity(self):
        external=self.root/'rendered';self.sensor.rename(external);self.sensor=external
        with self.assertRaisesRegex(ValueError,'exact source states SHA'):
            source_frames(self.assembly,self.trial,external)
        write(external/'provenance.json',dict(mode='measured_state_render',
             states_sha256=sha(self.trial/'states.npz'),scene_sha256=sha(self.assembly/'composed.usdc')))
        self.assertEqual(source_frames(self.assembly,self.trial,external)['mode'],'measured_state_render')
        data=json.loads((external/'provenance.json').read_text());data['scene_sha256']='wrong'
        write(external/'provenance.json',data)
        with self.assertRaisesRegex(ValueError,'exact source scene SHA'):
            source_frames(self.assembly,self.trial,external)

    def test_unsafe_pixel_path_and_old_recording(self):
        self.rows[0]['rgb']='../secret.png';self.write_rows()
        with self.assertRaisesRegex(ValueError,'Unsafe sensor file'):
            source_frames(self.assembly,self.trial)
        self.sensor.rename(self.root/'unused_sensor')
        self.assertIsNone(export_camera(self.assembly,self.trial,self.output))


if __name__=='__main__':
    unittest.main()
