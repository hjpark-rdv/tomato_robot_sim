from pathlib import Path
import tempfile
import unittest
import json

import mujoco as mj
import numpy as np

from newton_gripper_variant import containment_audit, prepare, prepare_contact_proxy, sha256


def fixture(root):
    source = root / "source"
    source.mkdir()
    vertices = " ".join(str(x) for x in np.array([[a,b,c] for a in [-.01,.01] for b in [-.02,.02] for c in [-.03,.03]]).ravel())
    xml = '<mujoco><compiler inertiafromgeom="false"/><option timestep=".0025"/>' + \
        '<asset><mesh name="box" vertex="'+vertices+'"/></asset><worldbody>' + \
        '<body name="Tool_L"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia=".01 .02 .03"/>' + \
        '<geom name="outer" type="mesh" mesh="box"/>' + \
        '<geom name="deep" type="mesh" mesh="box" pos="0 0 0" size="1"/>' + \
        '<geom name="different_material" type="mesh" mesh="box" friction=".9 .01 .01"/>' + \
        '</body><body name="Tool_L_belt" pos=".1 0 0"><geom name="belt" type="mesh" mesh="box"/></body>' + \
        '</worldbody></mujoco>'
    (source / 'input_scene.xml').write_text(xml)
    # Separate scaled asset so containment is real, not equal-shape rounding.
    import xml.etree.ElementTree as ET
    tree = ET.parse(source / 'input_scene.xml')
    tree.getroot().find('asset').append(ET.Element('mesh', name='small', vertex=' '.join(str(float(x)*.2) for x in np.array([[a,b,c] for a in [-.01,.01] for b in [-.02,.02] for c in [-.03,.03]]).ravel())))
    for g in tree.getroot().iter('geom'):
        if g.get('name') in ('deep','different_material'):g.set('mesh','small')
    tree.write(source / 'input_scene.xml', encoding='unicode')
    model = mj.MjModel.from_xml_path(str(source / 'input_scene.xml'))
    model.dof_length[:] = np.arange(model.nv)*.001+.02
    mj.mj_saveModel(model,str(source/'input_model.mjb'))
    data=mj.MjData(model)
    np.savez_compressed(source/'initial_state.npz',qpos=data.qpos,qvel=data.qvel,ctrl=data.ctrl,eq_active=data.eq_active)
    np.savez_compressed(source/'commands.npz',time=np.array([0.]),time_edges=np.array([0.,.0025]))
    (source/'source_export.json').write_text(json.dumps(dict(geometry=[dict(name='deep',collision=True)])))
    files=('input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json')
    (source/'inputs.json').write_text(json.dumps(dict(input_files={f:dict(sha256=sha256(source/f)) for f in files})))
    return source


class GripperVariantTests(unittest.TestCase):
    def test_strict_containment_preserves_belts_and_contact_material(self):
        with tempfile.TemporaryDirectory() as temp:
            source=fixture(Path(temp));model=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
            audit=containment_audit(model,minimum_clearance_m=.001)
            self.assertEqual([r['geom'] for r in audit['removed']],['deep'])
            self.assertGreaterEqual(audit['removed'][0]['certified_minimum_clearance_m'],.001)
            self.assertTrue(audit['exterior_union_unchanged'])

    def test_clearance_levels_and_invalid_values(self):
        with tempfile.TemporaryDirectory() as temp:
            source=fixture(Path(temp));model=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
            self.assertEqual(containment_audit(model,minimum_clearance_m=.009)['removed_colliders'],0)
            for value in (0,-1,float('nan')):
                with self.assertRaises(ValueError):containment_audit(model,minimum_clearance_m=value)
            with self.assertRaisesRegex(ValueError,'Belt'):containment_audit(model,tool_bodies=('Tool_L_belt',))

    def test_rotated_translated_body_frame_and_outside_piece(self):
        with tempfile.TemporaryDirectory() as temp:
            source=fixture(Path(temp));model=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
            q=np.array([np.cos(.39),0,0,np.sin(.39)]);rotation=np.zeros(9)
            mj.mju_quat2Mat(rotation,q)
            for g in range(model.ngeom):
                if model.body(int(model.geom_bodyid[g])).name!='Tool_L':continue
                model.geom_pos[g]=rotation.reshape(3,3)@model.geom_pos[g]+[.04,-.03,.02]
                composed=np.zeros(4);mj.mju_mulQuat(composed,q,model.geom_quat[g]);model.geom_quat[g]=composed
            self.assertEqual([r['geom'] for r in containment_audit(model)['removed']],['deep'])
            model.geom_pos[model.geom('deep').id]+=[.1,0,0]
            self.assertEqual(containment_audit(model)['removed_colliders'],0)

    def test_prepare_preserves_body_and_retained_geometry(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=fixture(root)
            before={p.name:sha256(p) for p in source.iterdir()}
            result=prepare(source,root/'variant')
            self.assertEqual(before,{p.name:sha256(p) for p in source.iterdir()})
            self.assertTrue(result['invariant_proof']['passed'])
            self.assertEqual(result['source_ngeom']-result['variant_ngeom'],1)
            self.assertEqual(result['original_nv'],result['variant_nv'])
            self.assertEqual(sha256(source/'commands.npz'),sha256(root/'variant/commands.npz'))
            self.assertEqual(sha256(source/'initial_state.npz'),sha256(root/'variant/initial_state.npz'))
            with self.assertRaises(FileExistsError):prepare(source,root/'variant')

    def test_id_preserving_mask_mode_changes_only_buried_contact_masks(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=fixture(root)
            result=prepare(source,root/'variant',keep_disabled_geom_ids=True)
            old=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
            new=mj.MjModel.from_binary_path(str(root/'variant/input_model.mjb'))
            self.assertTrue(result['invariant_proof']['passed'])
            self.assertTrue(result['native_disabled_geom_storage_preserved'])
            self.assertEqual(old.ngeom,new.ngeom)
            self.assertEqual(old.nmesh,new.nmesh)
            for g in range(old.ngeom):self.assertEqual(old.geom(g).name,new.geom(g).name)
            self.assertEqual(int(new.geom_contype[new.geom('deep').id]),0)
            self.assertEqual(int(new.geom_conaffinity[new.geom('deep').id]),0)
            self.assertEqual(result['source_active_geoms']-result['variant_active_geoms'],1)

    def test_outer_proxy_rejects_belt_before_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=fixture(root)
            with self.assertRaisesRegex(ValueError,'belts remain protected'):
                prepare_contact_proxy(source,root/'proxy',geom_names=('belt',))
            self.assertFalse((root/'proxy').exists())

if __name__=='__main__':unittest.main()
