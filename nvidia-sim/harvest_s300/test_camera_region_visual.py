import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

from camera_region_visual import annotate_region, REGION_COLOR
from recording_storage import compact_array


class CameraRegionVisualTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.instance = np.zeros((128, 96), np.uint32)
        self.instance[40:80, 20:65] = 101
        self.instance[55:65, 35:45] = 900  # Occluding leaf inside fruit extent.
        self.instance[90:110, 70:90] = 102  # Other, unselected fruit.
        np.save(self.directory / 'instance.npy', self.instance)
        self.base = np.full((128, 96, 3), (25, 70, 55), np.uint8)
        Image.fromarray(self.base).save(self.directory / 'recognition.png')
        Image.new('RGB', (96, 128), (40, 50, 60)).save(self.directory / 'rgb.png')
        self.analysis = np.full((158, 288, 3), (30, 40, 50), np.uint8)
        self.analysis[30:, 96:192] = self.base
        Image.fromarray(self.analysis).save(self.directory / 'analysis.png')
        self.source = '/World/Greenhouse/Plants/Plant_22/Truss_02/'
        rows = [dict(instance_id=101, key=self.source + 'Fruit_01', class_name='fruit', truss='Truss_02'),
                dict(instance_id=102, key=self.source + 'Fruit_02', class_name='fruit', truss='Truss_02'),
                dict(instance_id=900, key=self.source + 'Leaf', class_name='leaf')]
        (self.directory / 'objects.json').write_text(json.dumps(dict(timestamp_s=4., objects=rows)))
        self.group = dict(fruits=['Truss_02_Fruit_01', 'Truss_02_Fruit_03'], truss_key='Truss_02',
                          aim_world_m=[.1, .2, 1.2], camera_region=dict(
            observation=dict(observation_path='/record/scan/frame_00008', timestamp_s=4.),
            accepted_observations=[dict(name='Truss_02_Fruit_01', key=self.source + 'Fruit_01', instance_id=101),
                                   dict(name='Truss_02_Fruit_03', key=self.source + 'Fruit_03', instance_id=103)]))

    def test_inner_contours_do_not_paint_occluder_unselected_or_hidden_pixels(self):
        result = annotate_region(self.directory, self.group)
        image = np.asarray(Image.open(self.directory / result['recognition']))
        # Exclude the small legend strip while checking the actual annotation.
        changed = np.any(image[25:] != self.base[25:], axis=-1)
        self.assertTrue(changed.any())
        self.assertTrue(np.all(self.instance[25:][changed] == 101))
        self.assertTrue(np.all(image[55:65, 35:45] == self.base[55:65, 35:45]))
        self.assertTrue(np.all(image[90:110, 70:90] == self.base[90:110, 70:90]))
        self.assertTrue(np.all(image[40, 20] == REGION_COLOR))
        self.assertEqual(result['visible_selected_instance_ids'], [101])
        self.assertEqual(result['selected_instance_ids'], [101, 103])
        metadata = json.loads((self.directory / 'region.json').read_text())
        self.assertFalse(metadata['hidden_masks_fabricated'])
        self.assertEqual(metadata['currently_absent_selected_fruits'], ['Truss_02_Fruit_03'])
        self.assertEqual(metadata['selection_observation_path'], '/record/scan/frame_00008')
        self.assertEqual(metadata['target_aim_world_m'], [.1, .2, 1.2])

    def test_compressed_mask_keeps_exact_selection_pixels_and_original_hash(self):
        raw_hash=hashlib.sha256((self.directory/'instance.npy').read_bytes()).hexdigest()
        compact_array(self.directory/'instance.npy')
        result=annotate_region(self.directory,self.group)
        pixels=np.asarray(Image.open(self.directory/result['recognition']))
        changed=np.any(pixels[25:]!=self.base[25:],axis=-1)
        self.assertTrue(changed.any())
        self.assertTrue(np.all(self.instance[25:][changed]==101))
        metadata=json.loads((self.directory/'region.json').read_text())
        self.assertEqual(metadata['source_sha256']['instance.npy'],raw_hash)

    def test_original_bytes_and_other_analysis_panels_are_unchanged(self):
        files = ['rgb.png', 'recognition.png', 'analysis.png', 'instance.npy', 'objects.json']
        before = {name: hashlib.sha256((self.directory / name).read_bytes()).hexdigest() for name in files}
        group = copy.deepcopy(self.group)
        result = annotate_region(self.directory, self.group)
        self.assertEqual(self.group, group)
        self.assertEqual(before, {name: hashlib.sha256((self.directory / name).read_bytes()).hexdigest() for name in files})
        analysis = np.asarray(Image.open(self.directory / result['analysis']))
        np.testing.assert_array_equal(analysis[:30], self.analysis[:30])
        np.testing.assert_array_equal(analysis[:, :96], self.analysis[:, :96])
        np.testing.assert_array_equal(analysis[:, 192:], self.analysis[:, 192:])
        np.testing.assert_array_equal(analysis[30:, 96:192], np.asarray(Image.open(self.directory / result['recognition'])))

    def test_wrong_identity_and_panel_dimensions_are_rejected_before_writing(self):
        bad = copy.deepcopy(self.group)
        bad['camera_region']['accepted_observations'][0]['key'] = self.source + 'Fruit_02'
        with self.assertRaisesRegex(ValueError, 'name and accepted object key disagree'):
            annotate_region(self.directory, bad)
        self.assertFalse((self.directory / 'harvest_region.png').exists())
        bad = copy.deepcopy(self.group)
        bad['camera_region']['accepted_observations'][0]['instance_id'] = 102
        with self.assertRaisesRegex(ValueError, 'identity disagrees'):
            annotate_region(self.directory, bad)
        Image.new('RGB', (288, 128)).save(self.directory / 'analysis.png')
        with self.assertRaisesRegex(ValueError, '30-pixel header'):
            annotate_region(self.directory, self.group)
        self.assertFalse((self.directory / 'region.json').exists())


if __name__ == '__main__':
    unittest.main()
