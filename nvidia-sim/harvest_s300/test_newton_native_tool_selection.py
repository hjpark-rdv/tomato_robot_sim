"""Explicit second tool correction preserves the previously corrected part."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from newton_inputs import sha256
from newton_native_tool_pieces import prepare
from newton_fruit_proxy import geom_body_vertices
from test_newton_native_tool_pieces import fixture


def right_fixture(root):
    source, pieces = fixture(root)
    previous = mj.MjModel.from_binary_path(str(source / 'input_model.mjb'))
    tree = ET.parse(source / 'input_scene.xml')
    body = next(b for b in tree.iter('body') if b.get('name') == 'Tool_L')
    body.set('name', 'Tool_R')
    body.find('geom').set('name', 's300_g0697')
    # Stand in for already-preserved native part161 pieces on the other jaw.
    left = ET.SubElement(tree.getroot().find('worldbody'), 'body', name='Tool_L', pos='-1 0 0')
    ET.SubElement(left, 'inertial', mass='1', pos='0 0 0', diaginertia='.01 .02 .03')
    for index, name in enumerate(('s300_g0620', 's300_g0620__native001')):
        ET.SubElement(left, 'geom', name=name, mesh='other', type='mesh', pos=f'0 {index * .02} 0')
    tree.write(source / 'input_scene.xml', encoding='unicode')
    model = mj.MjModel.from_xml_path(str(source / 'input_scene.xml'))
    model.dof_damping[:] = previous.dof_damping
    model.dof_length[:] = previous.dof_length
    mj.mj_saveModel(model, str(source / 'input_model.mjb'))
    metadata = json.loads((source / 'source_export.json').read_text())
    right = metadata['robot']['geometry'][0]
    right.update(name='s300_g0697', owner='Tool_R', source='/World/Tool/R/part_239/mesh')
    left_rows = [dict(name=name, owner='Tool_L', collision=True,
        source='/World/Tool/L/part_161/mesh', native_piece_index=index)
        for index, name in enumerate(('s300_g0620', 's300_g0620__native001'))]
    metadata['geometry'] = [copy.deepcopy(right), metadata['geometry'][1], *left_rows]
    metadata['robot']['geometry'] += copy.deepcopy(left_rows)
    (source / 'source_export.json').write_text(json.dumps(metadata))
    proof = json.loads((source / 'inputs.json').read_text())
    for name in proof['input_files']:
        proof['input_files'][name]['sha256'] = sha256(source / name)
    (source / 'inputs.json').write_text(json.dumps(proof))
    value = json.loads(pieces.read_text())
    value.update(target_geom='s300_g0697', target_body='Tool_R',
        source_input_model_sha256=sha256(source / 'input_model.mjb'))
    value['source'].update(source_collider_path=right['source'], owner_path='/World/Tool/R')
    pieces.write_text(json.dumps(value))
    return source, pieces


class NativeToolSelectionControls(unittest.TestCase):
    def test_second_explicit_collider_correction_preserves_original_left_pieces(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source, pieces = right_fixture(root)
            before = {str(p.relative_to(source)): sha256(p) for p in source.rglob('*') if p.is_file()}
            # The old default cannot silently consume another part's evidence.
            with self.assertRaisesRegex(ValueError, 'Exact native tool-piece schema'):
                prepare(source, root / 'implicit_wrong_selection', pieces)
            self.assertFalse((root / 'implicit_wrong_selection').exists())
            output = root / 'explicit_right'
            result = prepare(source, output, pieces, target_geom='s300_g0697', target_body='Tool_R')
            self.assertEqual(result['selected_geom'], 's300_g0697')
            self.assertEqual(result['selected_body'], 'Tool_R')
            self.assertTrue(result['invariant_proof']['passed'])
            old = mj.MjModel.from_binary_path(str(source / 'input_model.mjb'))
            new = mj.MjModel.from_binary_path(str(output / 'input_model.mjb'))
            for name in ('s300_g0620', 's300_g0620__native001'):
                np.testing.assert_array_equal(geom_body_vertices(old, old.geom(name).id),
                                              geom_body_vertices(new, new.geom(name).id))
                self.assertEqual(new.body(int(new.geom_bodyid[new.geom(name).id])).name, 'Tool_L')
            self.assertEqual([r['geom'] for r in result['pieces']],
                ['s300_g0697', 's300_g0697__native001', 's300_g0697__native002'])
            self.assertEqual(before, {str(p.relative_to(source)): sha256(p)
                for p in source.rglob('*') if p.is_file()})
            self.assertEqual(sha256(output / 'commands.npz'), sha256(source / 'commands.npz'))
            self.assertEqual(sha256(output / 'initial_state.npz'), sha256(source / 'initial_state.npz'))

    def test_explicit_geometry_and_body_must_both_match_hash_bound_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source, pieces = right_fixture(root)
            for index, names in enumerate((('s300_g0697', 'Tool_L'), ('s300_g0620', 'Tool_R'),
                                           ('', 'Tool_R'), ('s300_g0697', ''))):
                output = root / f'rejected_{index}'
                with self.subTest(names=names), self.assertRaises(ValueError):
                    prepare(source, output, pieces, target_geom=names[0], target_body=names[1])
                self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
