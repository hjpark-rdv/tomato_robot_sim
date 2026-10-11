"""Native cache regression for merging an already used source model.

Small generated assets replace the server farm. The second native compile and
cache write must never change the base model's cache or certificate.
"""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco as mj

from merge_windows import merge


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_source(directory, plant, x):
    directory.mkdir()
    (directory/'meshes').mkdir()
    root = ET.Element('mujoco')
    ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    body = ET.SubElement(world, 'body', name='Stem_MainStem_0', pos=f'{x} 0 0')
    ET.SubElement(body, 'geom', name='stem_geom', type='sphere', size='.02')
    ET.SubElement(root, 'contact')
    ET.SubElement(root, 'equality')
    ET.ElementTree(root).write(directory/'scene.xml', encoding='unicode')
    manifest = directory/'source_manifest.json'
    manifest.write_text(json.dumps(dict(plant=dict(source_plant=f'/World/Plants/Plant_{plant}'))))
    (directory/'source_plan.json').write_text(json.dumps(dict(plant=plant, input='saved_plan'))+'\n')
    (directory/'plan.json').write_text(json.dumps(dict(plant=plant, input='trial_plan'))+'\n')
    meta = dict(source_manifest=str(manifest), source_assembly=str(directory),
                source_trial=str(directory), rods=['Stem_MainStem_0'], fruits=[],
                fruit_specs=[], attachments=[], model_sha256=digest(directory/'scene.xml'),
                source_plan_sha256=digest(directory/'source_plan.json'),
                geometry=[dict(name='stem_geom', owner='Stem_MainStem_0', collision=True)])
    (directory/'export.json').write_text(json.dumps(meta))
    return directory


class MergeCacheTests(unittest.TestCase):
    def test_recompiled_merged_cache_cannot_mutate_used_base(self):
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            base = make_source(temporary/'base', '29', 0.)
            extra = make_source(temporary/'extra', '41', 1.)
            # A plant-only export has no source_plan copy yet, so its live
            # trial plan must become an independent bound snapshot too.
            (extra/'source_plan.json').unlink()
            extra_meta=json.loads((extra/'export.json').read_text())
            del extra_meta['source_plan_sha256']
            (extra/'export.json').write_text(json.dumps(extra_meta))
            old = mj.MjModel.from_xml_path(str(base/'scene.xml'))
            for name in ('compiled_scene.mjb', 'model.mjb'):
                mj.mj_saveModel(old, str(base/name))
            certificate = dict(model_xml_sha256=digest(base/'scene.xml'),
                               mjb_sha256=digest(base/'compiled_scene.mjb'),
                               mujoco_version=mj.__version__)
            (base/'compiled_scene.json').write_text(json.dumps(certificate))
            protected = ('scene.xml', 'export.json', 'compiled_scene.mjb',
                         'compiled_scene.json', 'model.mjb')
            snapshots = {name: (digest(base/name), (base/name).stat().st_ino,
                                (base/name).stat().st_nlink) for name in protected}
            combined = temporary/'combined'
            merge(base, [('P01_', extra)], combined)
            for name in ('compiled_scene.mjb', 'compiled_scene.json', 'model.mjb'):
                self.assertFalse((combined/name).exists(), name)
            newer = mj.MjModel.from_xml_path(str(combined/'scene.xml'))
            self.assertEqual(newer.nbody, old.nbody+1)
            self.assertGreater(newer.body('P01_Stem_MainStem_0').id, 0)
            # This is the actual native cache-writing operation, rather than a
            # mock of Session's invalid-cache branch.
            mj.mj_saveModel(newer, str(combined/'compiled_scene.mjb'))
            (combined/'compiled_scene.json').write_text(json.dumps(dict(
                model_xml_sha256=digest(combined/'scene.xml'),
                mjb_sha256=digest(combined/'compiled_scene.mjb'),
                mujoco_version=mj.__version__)))
            for name, snapshot in snapshots.items():
                self.assertEqual(digest(base/name), snapshot[0], name)
                self.assertEqual((base/name).stat().st_ino, snapshot[1], name)
                self.assertEqual((base/name).stat().st_nlink, snapshot[2], name)
            for name in ('compiled_scene.mjb', 'compiled_scene.json'):
                self.assertNotEqual((combined/name).stat().st_ino,
                                    (base/name).stat().st_ino)
            metadata = json.loads((combined/'export.json').read_text())
            self.assertEqual(metadata['model_sha256'], digest(combined/'scene.xml'))
            self.assertFalse((combined/'source_plan.json').exists())
            for index, source in enumerate((base, extra)):
                bound = metadata['windows'][index]
                saved = Path(bound['source_plan'])
                original_plan=source/('source_plan.json' if index==0 else 'plan.json')
                self.assertEqual(saved.parent, combined.resolve())
                self.assertEqual(bound['source_plan_sha256'], digest(saved))
                self.assertNotEqual(saved.stat().st_ino,
                                    original_plan.stat().st_ino)
                original = saved.read_bytes()
                # In-place edits to the live trial/export cannot alter a bound
                # plan in the newly prepared model.
                original_plan.write_text('{"changed_after_merge":true}\n')
                self.assertEqual(saved.read_bytes(), original)
                self.assertEqual(bound['source_plan_sha256'], digest(saved))

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            base = make_source(temporary/'base', '29', 0.)
            combined = temporary/'combined'
            combined.mkdir()
            marker = combined/'previous_result.json'
            marker.write_text('{"preserve":true}\n')
            with self.assertRaises(FileExistsError):
                merge(base, [], combined)
            self.assertEqual(marker.read_text(), '{"preserve":true}\n')


if __name__ == '__main__':
    unittest.main()
