"""CPU geometry controls for exact native piece placement and protected fields."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import mujoco as mj
import numpy as np

from newton_native_tool_pieces import prepare, verify_invariants
from newton_fruit_proxy import _write_obj
from newton_inputs import sha256
from scipy.spatial import ConvexHull


def box(center, half):
    return np.asarray(center) + np.asarray([[a, b, c] for a in (-half[0], half[0])
        for b in (-half[1], half[1]) for c in (-half[2], half[2])])


def fixture(root):
    source = root / 'source';source.mkdir();meshes = source / 'meshes';meshes.mkdir()
    parts = [box([-.02, 0, 0], [.003, .03, .01]), box([.02, 0, 0], [.003, .03, .01]),
             box([0, -.027, 0], [.017, .003, .01])]
    original = np.concatenate(parts)
    _write_obj(meshes / 'original.obj', original, ConvexHull(original).simplices)
    cube = box([0, 0, 0], [.003, .004, .005])
    _write_obj(meshes / 'other.obj', cube, ConvexHull(cube).simplices)
    angle = .47
    quaternion = [np.cos(angle / 2), 0, 0, np.sin(angle / 2)]
    transform = np.asarray([[np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    offset = np.asarray([.1, -.03, .02])
    sphere = np.asarray([.3, .1, .2]) + offset + transform @ np.asarray([.002, .003, .001])
    xml = f'''<mujoco><compiler angle="radian" inertiafromgeom="false" discardvisual="true" meshdir="{meshes}"/>
      <default><geom friction=".6 .02 .003" margin="0" gap=".0005" solref=".002 1"/></default>
      <asset><mesh name="original" file="original.obj"/><mesh name="other" file="other.obj"/></asset>
      <worldbody><body name="Tool_L" pos=".3 .1 .2"><freejoint name="tool_free"/>
      <inertial mass="2" pos="0 0 0" diaginertia=".01 .02 .03"/>
      <geom name="s300_g0620" type="mesh" mesh="original" pos=".1 -.03 .02" quat="{' '.join(map(str, quaternion))}" contype="2" conaffinity="1" density="0"/>
      </body><body name="other"><inertial mass="1" pos="0 0 0" diaginertia=".01 .01 .01"/>
      <geom name="protected" mesh="other" type="mesh" pos="-.4 0 0"/></body>
      <geom name="hole_probe" type="sphere" size=".002" pos="{' '.join(map(str, sphere))}" contype="1" conaffinity="2"/>
      <geom name="invisible_to_physics" type="sphere" size=".1" pos="2 0 0" contype="0" conaffinity="0"/>
      </worldbody></mujoco>'''
    (source / 'input_scene.xml').write_text(xml)
    model = mj.MjModel.from_xml_path(str(source / 'input_scene.xml'))
    model.dof_length[:] = np.arange(model.nv) * .001 + .02
    model.dof_damping[:] = np.arange(model.nv) * .002
    mj.mj_saveModel(model, str(source / 'input_model.mjb'))
    data = mj.MjData(model)
    np.savez_compressed(source / 'initial_state.npz', qpos=data.qpos, qvel=data.qvel,
                        ctrl=data.ctrl, eq_active=data.eq_active)
    np.savez_compressed(source / 'commands.npz', time=[0.], command=np.zeros((1, 0)))
    usd = root / 'source.usdc';usd.write_bytes(b'original source USD fixture evidence')
    row = dict(name='s300_g0620', owner='Tool_L', collision=True,
               source='/World/Tool/L/part_161/mesh')
    metadata = dict(geometry=[row, dict(name='protected', owner='other', collision=True)],
                    robot=dict(geometry=[copy.deepcopy(row)]))
    (source / 'source_export.json').write_text(json.dumps(metadata))
    names = ['input_scene.xml', 'input_model.mjb', 'initial_state.npz', 'commands.npz',
             'source_export.json', 'meshes/original.obj', 'meshes/other.obj']
    (source / 'inputs.json').write_text(json.dumps(dict(input_files={name:
        dict(sha256=sha256(source / name)) for name in names})))
    pieces = dict(schema='s300-native-convex-pieces-v1', coordinate_frame='body_local',
        source_input_model_sha256=sha256(source / 'input_model.mjb'),
        target_geom='s300_g0620', target_body='Tool_L',
        source=dict(usd_path=str(usd), source_scene_sha256=sha256(usd),
                    source_collider_path=row['source'], owner_path='/World/Tool/L'),
        pieces=[dict(vertices_body_local_m=(points @ transform.T + offset).tolist()) for points in parts])
    path = root / 'pieces.json';path.write_text(json.dumps(pieces))
    return source, path


class NativeToolPiecesTests(unittest.TestCase):
    def test_hollow_part_native_pieces_remove_hull_bridge_without_double_transform(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);source, path = fixture(root)
            before = {str(p.relative_to(source)): sha256(p) for p in source.rglob('*') if p.is_file()}
            result = prepare(source, root / 'derived', path)
            self.assertEqual(before, {str(p.relative_to(source)): sha256(p)
                for p in source.rglob('*') if p.is_file()})
            old = mj.MjModel.from_binary_path(str(source / 'input_model.mjb'))
            new = mj.MjModel.from_binary_path(str(root / 'derived/input_model.mjb'))
            self.assertEqual(new.ngeom - old.ngeom, 2)
            self.assertTrue(result['invariant_proof']['passed'])
            self.assertTrue(result['compiler_policy_unchanged'])
            self.assertTrue(result['no_parent_resume_sha_bypass'])
            self.assertEqual(sha256(source / 'meshes/original.obj'),
                             sha256(root / 'derived/original_selected_tool_mesh.obj'))
            names = [row['geom'] for row in result['pieces']]
            self.assertEqual(names, ['s300_g0620', 's300_g0620__native001', 's300_g0620__native002'])
            for model in (old, new):
                self.assertNotIn('invisible_to_physics', [model.geom(g).name for g in range(model.ngeom)])
            od, nd = mj.MjData(old), mj.MjData(new)
            mj.mj_kinematics(old, od);mj.mj_kinematics(new, nd)
            original_distance = mj.mj_geomDistance(old, od, old.geom('s300_g0620').id,
                old.geom('hole_probe').id, .2, None)
            native_distances = [mj.mj_geomDistance(new, nd, new.geom(name).id,
                new.geom('hole_probe').id, .2, None) for name in names]
            self.assertLess(original_distance, 0.)
            self.assertGreater(min(native_distances), .01)
            self.assertEqual(sha256(source / 'initial_state.npz'), sha256(root / 'derived/initial_state.npz'))
            self.assertEqual(sha256(source / 'commands.npz'), sha256(root / 'derived/commands.npz'))
            self.assertNotEqual(sha256(source / 'input_model.mjb'), sha256(root / 'derived/input_model.mjb'))
            for name in ('body_mass', 'body_inertia', 'dof_damping', 'dof_length'):
                np.testing.assert_array_equal(getattr(old, name), getattr(new, name))
            with self.assertRaises(FileExistsError):
                prepare(source, root / 'derived', path)

    def test_invariants_reject_contact_mass_and_other_geometry_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);source, path = fixture(root)
            result = prepare(source, root / 'derived', path)
            old = mj.MjModel.from_binary_path(str(source / 'input_model.mjb'))
            filename = root / 'derived/input_model.mjb';names = [row['geom'] for row in result['pieces']]
            for field, index in [('geom_friction', (old.geom('s300_g0620').id, 0)),
                                 ('body_mass', old.body('Tool_L').id),
                                 ('geom_pos', (old.geom('protected').id, 0))]:
                new = mj.MjModel.from_binary_path(str(filename))
                if field.startswith('geom_'):
                    name = old.geom(index[0]).name;target = (new.geom(name).id, index[1])
                else:
                    target = index
                getattr(new, field)[target] += .001
                with self.assertRaisesRegex(ValueError, 'Protected physical'):
                    verify_invariants(old, new, names)
            new = mj.MjModel.from_binary_path(str(filename))
            new.mesh_vert[int(new.mesh_vertadr[new.mesh('other').id]), 0] += .001
            with self.assertRaisesRegex(ValueError, 'Protected physical'):
                verify_invariants(old, new, names)

    def test_source_hash_coordinate_frame_and_nonfinite_pieces_fail_before_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);source, path = fixture(root);original = json.loads(path.read_text())
            changes = [dict(source_input_model_sha256='0' * 64), dict(coordinate_frame='world'),
                       dict(pieces=[]), dict(pieces=[dict(vertices_body_local_m=[[float('nan'), 0, 0]] * 4)])]
            for index, update in enumerate(changes):
                value = dict(original, **update);path.write_text(json.dumps(value))
                out = root / f'bad{index}'
                with self.assertRaises(ValueError):
                    prepare(source, out, path)
                self.assertFalse(out.exists())
            path.write_text(json.dumps(original))
            (source / 'commands.npz').write_bytes(b'changed commands')
            with self.assertRaisesRegex(ValueError, 'SHA mismatch'):
                prepare(source, root / 'bad_hash', path)


if __name__ == '__main__':
    unittest.main()
