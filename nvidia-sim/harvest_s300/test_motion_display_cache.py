"""Display caching preserves composed geometry across reset and target changes."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Tf, Usd, UsdGeom

from motion_pose_display import MeasuredPoseDisplay
from motion_visuals import DeformationVisuals
from test_motion_visuals import mesh, active_collision_state


def poses(centers):
    return np.column_stack((np.asarray(centers, dtype=np.float64),
                            np.tile([0., 0., 0., 1.], (len(centers), 1))))


def skin_fixture(root):
    stage = Usd.Stage.CreateInMemory()
    paths = ['/World/plant22/body', '/World/plant27/body', '/World/detached_fruit']
    for path in paths:
        UsdGeom.Xform.Define(stage, path)
    skin_paths = ['/World/ElasticTarget/Skin/mesh_0', '/World/ElasticTarget/Skin/mesh_1']
    arrays = {}
    for index, path in enumerate(skin_paths):
        mesh(stage, path, False)
        vertices = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])
        for key, value in dict(vertices=vertices, rest=np.zeros((3, 3)),
                ids=np.full((3, 1), index), weights=np.ones((3, 1)),
                body_paths=np.array(paths)).items():
            arrays[f'{index}_{key}'] = value
    mesh(stage, '/World/real_collider', True)
    for name in ('crank', 'linkL', 'linkR'):
        UsdGeom.Xform.Define(stage, '/World/ActuatedTool/fixed/Transmission/' + name)
    (root/'motion_manifest.json').write_text(json.dumps({'plant': {'skin_paths': skin_paths}}))
    np.savez(root/'skin.npz', **arrays)
    return stage, paths, skin_paths


class DisplayCacheControls(unittest.TestCase):
    def test_pose_cache_tracks_parent_changes_and_detached_fruit(self):
        stage = Usd.Stage.CreateInMemory()
        paths = ['/World/parent', '/World/parent/bridge/child', '/World/detached_fruit']
        for path in paths:
            UsdGeom.Xform.Define(stage, path)
        UsdGeom.Xformable(stage.GetPrimAtPath('/World/parent/bridge')).AddTranslateOp().Set(Gf.Vec3d(.2, 0., 0.))
        source = stage.GetRootLayer().ExportToString()
        display = MeasuredPoseDisplay(stage, paths)
        sample = poses([[0., 0., 0.], [.5, .3, .1], [2., 0., 0.]])
        self.assertEqual(display.update(sample)['bodies_updated'], 3)
        notices = []
        token = Tf.Notice.Register(Usd.Notice.ObjectsChanged, lambda event, sender: notices.append(event), stage)
        self.assertEqual(display.update(sample)['bodies_updated'], 0)
        self.assertEqual(notices, [])
        # Child's measured world pose is unchanged while its parent moves.
        # Its local matrix must nevertheless change to preserve the world pose.
        sample[0, 0] = .1
        self.assertEqual(display.update(sample)['bodies_updated'], 2)
        actual = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(paths[1])))
        np.testing.assert_allclose(actual[3, :3], sample[1, :3], atol=1e-15)
        sample[2, 2] = -.3
        self.assertEqual(display.update(sample)['bodies_updated'], 1)
        token.Revoke()
        self.assertEqual(source, stage.GetRootLayer().ExportToString())

    def test_exact_input_bits_reset_and_new_viewer_force_first_update(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, '/World/body')
        display = MeasuredPoseDisplay(stage, ['/World/body'])
        sample = poses([[0., 0., 0.]])
        self.assertEqual(display.update(sample)['bodies_updated'], 1)
        self.assertEqual(display.update(sample.copy())['bodies_updated'], 0)
        sample[0, 0] = -0.  # Numerically equal, different input bytes.
        self.assertEqual(display.update(sample)['bodies_updated'], 1)
        sample[0, 0] = np.nextafter(0., 1.)
        self.assertEqual(display.update(sample)['bodies_updated'], 1)
        display.invalidate()
        self.assertEqual(display.update(sample)['bodies_updated'], 1)
        self.assertEqual(MeasuredPoseDisplay(stage, ['/World/body']).update(sample)['bodies_updated'], 1)

    def test_skin_dependency_cache_target_switch_gap_and_reset(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stage, paths, skin_paths = skin_fixture(root)
            source = stage.GetRootLayer().ExportToString()
            collision = active_collision_state(stage)
            visual = DeformationVisuals(stage, root, paths)
            sample = poses([[0., 0., 0.], [3., 0., 0.], [0., 4., 0.]])
            self.assertEqual(visual.update(sample, .044)['meshes_updated'], 2)
            notices = []
            token = Tf.Notice.Register(Usd.Notice.ObjectsChanged, lambda event, sender: notices.append(event), stage)
            same = visual.update(sample, .044)
            self.assertEqual((same['meshes_updated'], same['meshes_skipped']), (0, 2))
            self.assertEqual(notices, [])
            sample[2, 2] = -.2  # Released fruit moves; neither rod depends on it.
            self.assertEqual(visual.update(sample, .044)['meshes_updated'], 0)
            sample[0, 0] = .125
            self.assertEqual(visual.update(sample, .044)['meshes_updated'], 1)
            sample[1, 2] = .25  # Next target becomes active.
            self.assertEqual(visual.update(sample, .044)['meshes_updated'], 1)
            current = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(skin_paths[1])).GetPointsAttr().Get())
            np.testing.assert_array_equal(current, [[3., 0., .25], [4., 0., .25], [3., 1., .25]])
            before_gap = [np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Get()).copy()
                          for path in skin_paths]
            self.assertEqual(visual.update(sample, np.nextafter(.044, 1.))['meshes_updated'], 0)
            gap_only = visual.update(sample, .04)
            self.assertEqual((gap_only['meshes_updated'],gap_only['transmission_updates']), (0,3))
            for path,expected in zip(skin_paths,before_gap):
                np.testing.assert_array_equal(np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Get()),expected)
            visual.invalidate()
            self.assertEqual(visual.update(sample, .044)['meshes_updated'], 2)
            self.assertEqual(DeformationVisuals(stage, root, paths).update(sample, .044)['meshes_updated'], 2)
            token.Revoke()
            self.assertEqual(collision, active_collision_state(stage))
            self.assertEqual(source, stage.GetRootLayer().ExportToString())

    def test_cached_frames_match_forced_full_saved_renderer_updates(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cached, paths, skin_paths = skin_fixture(root)
            reference = Usd.Stage.CreateInMemory()
            reference.GetRootLayer().ImportFromString(cached.GetRootLayer().ExportToString())
            left_pose, right_pose = MeasuredPoseDisplay(cached, paths), MeasuredPoseDisplay(reference, paths)
            left_skin, right_skin = DeformationVisuals(cached, root, paths), DeformationVisuals(reference, root, paths)
            sample = poses([[0., 0., 0.], [3., 0., 0.], [0., 4., 0.]])
            sequence = [(sample.copy(), .044), (sample.copy(), .044)]
            sample[0, 1] = .2; sequence.append((sample.copy(), .044))
            sample[2, 2] = -.4; sequence.append((sample.copy(), .04))
            sample[1, 0] += .3; sequence.append((sample.copy(), .04))
            sequence.append((poses([[0., 0., 0.], [3., 0., 0.], [0., 4., 0.]]), .044))
            for state, gap in sequence:
                right_pose.invalidate(); right_skin.invalidate()
                left_pose.update(state); left_skin.update(state, gap)
                right_pose.update(state); right_skin.update(state, gap)
                for x, y in zip(left_pose.ops, right_pose.ops):
                    np.testing.assert_array_equal(np.asarray(x.Get()), np.asarray(y.Get()))
                for path in skin_paths:
                    x, y = UsdGeom.Mesh(cached.GetPrimAtPath(path)), UsdGeom.Mesh(reference.GetPrimAtPath(path))
                    np.testing.assert_array_equal(np.asarray(x.GetPointsAttr().Get()), np.asarray(y.GetPointsAttr().Get()))
                    np.testing.assert_array_equal(np.asarray(x.GetExtentAttr().Get()), np.asarray(y.GetExtentAttr().Get()))
                for name in ('crank', 'linkL', 'linkR'):
                    np.testing.assert_array_equal(np.asarray(left_skin.transmission[name].Get()),
                                                  np.asarray(right_skin.transmission[name].Get()))


if __name__ == '__main__':
    unittest.main()
