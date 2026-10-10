"""Latest-frame transport coherence and original replay transform regression."""
import json
import tempfile
import threading
import unittest
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from motion_live import MotionLiveWriter, MotionLiveReader
from motion_pose_display import MeasuredPoseDisplay, restore_recorded_display_fixture


class LiveTransportTests(unittest.TestCase):
    def setup_stream(self, directory, paths=('/World/body',)):
        root = Path(directory)
        assembly = root / 'assembly'
        assembly.mkdir()
        (assembly / 'scene.usda').write_text('#usda 1.0\n')
        writer = MotionLiveWriter(root / 'trial', assembly, paths)
        return writer, MotionLiveReader(root / 'trial')

    def test_fixed_size_and_exact_measured_frame_with_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            writer, reader = self.setup_stream(directory)
            self.assertIsNone(reader.read())
            size = (writer.output / 'live.bin').stat().st_size
            poses = np.array([[.1, .2, .3, 0., 0., 0., 1.]])
            for index in range(20):
                writer.publish(poses, .044, index / 240)
            snapshot = reader.read()
            np.testing.assert_array_equal(snapshot['poses'], poses)
            self.assertEqual(snapshot['time'], 19/240)
            self.assertEqual(snapshot['gap'], .044)
            self.assertEqual(snapshot['status'], 'running')
            writer.close()
            self.assertEqual(reader.read()['status'], 'completed')
            self.assertEqual((writer.output / 'live.bin').stat().st_size, size)
            self.assertEqual(sorted(path.name for path in writer.output.iterdir()), ['live.bin', 'live.json'])
            self.assertEqual(json.loads((writer.output/'live.json').read_text())['publications'], 20)
            self.assertEqual(reader.verify_source(), writer.assembly/'scene.usda')
            with self.assertRaises(FileExistsError):
                MotionLiveWriter(writer.output, writer.assembly, ['/World/body'])

    def test_partial_frames_are_rejected_and_stopped_reason_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            writer, reader = self.setup_stream(directory)
            poses = np.array([[0., 0., 0., 0., 0., 0., 1.]])
            writer.publish(poses, .04, .01)
            writer.buffer[0] = writer.sequence + 1
            writer.poses[0, 0] = 999.
            self.assertIsNone(reader.read())
            writer.poses[:] = poses
            writer.buffer[0] = writer.sequence
            writer.close('penetration_limit')
            self.assertEqual(reader.read()['status'], 'stopped')
            self.assertEqual(json.loads((writer.output/'live.json').read_text())['reason'], 'penetration_limit')
            with self.assertRaises(RuntimeError):
                writer.publish(poses, .04, .02)
            (writer.assembly/'scene.usda').write_text('changed scene')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                reader.verify_source()

    def test_replicated_stream_selects_env0_without_mixing_worlds(self):
        paths = ['/World/MotionEnvs/env_1/ElasticTarget/A',
                 '/World/MotionEnvs/env_0/ElasticTarget/A', '/World/shared_body',
                 '/World/MotionEnvs/env_0/ActuatedTool/fixed']
        with tempfile.TemporaryDirectory() as directory:
            writer, reader = self.setup_stream(directory, paths)
            poses = np.column_stack((np.arange(12).reshape(4,3), np.tile([0.,0.,0.,1.], (4,1))))
            writer.publish(poses, .04, .1)
            self.assertEqual(reader.body_paths, ['/World/ElasticTarget/A', '/World/shared_body', '/World/ActuatedTool/fixed'])
            np.testing.assert_array_equal(reader.read()['poses'], poses[[1,2,3]])
            writer.close()

    def test_concurrent_reader_never_accepts_a_mixed_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = [f'/World/body_{i}' for i in range(248)]
            writer, reader = self.setup_stream(directory, paths)
            errors = []
            done = threading.Event()
            def produce():
                try:
                    for index in range(1, 2001):
                        poses = np.zeros((248, 7))
                        poses[:, :3] = index
                        poses[:, 6] = 1.
                        writer.publish(poses, index * 1e-6, float(index))
                    writer.close()
                except Exception as error:
                    errors.append(error)
                finally:
                    done.set()
            thread = threading.Thread(target=produce)
            thread.start()
            accepted = 0
            while not done.is_set():
                frame = reader.read()
                if frame is not None:
                    self.assertTrue(np.all(frame['poses'][:, :3] == frame['time']))
                    self.assertEqual(frame['gap'], frame['time'] * 1e-6)
                    self.assertTrue(np.all(frame['poses'][:, 6] == 1.))
                    accepted += 1
            thread.join()
            self.assertEqual(errors, [])
            self.assertGreater(accepted, 0)
            self.assertEqual(reader.read()['time'], 2000.)


def original_pose_setup(stage, paths):
    """Frozen formula from render_motion.py before helper extraction."""
    index = {path:i for i,path in enumerate(paths)}
    cache = UsdGeom.XformCache()
    rest_world = {path: np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T for path in paths}
    references, ops = [], []
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        parent_world = np.asarray(cache.GetLocalToWorldTransform(prim.GetParent())).T
        ancestor = prim.GetParent()
        while ancestor and str(ancestor.GetPath()) not in index:
            ancestor = ancestor.GetParent()
        ancestor_path = str(ancestor.GetPath()) if ancestor else None
        relative = np.linalg.inv(rest_world[ancestor_path]) @ parent_world if ancestor_path else parent_world
        references.append((index[ancestor_path] if ancestor_path else None, relative))
        ops.append(UsdGeom.Xformable(prim).MakeMatrixXform())
    return references, ops


class MeasuredPoseTests(unittest.TestCase):
    def test_recorded_fixture_restores_only_visual_in_session_layer(self):
        with tempfile.TemporaryDirectory() as directory:
            trial = Path(directory)
            stage = Usd.Stage.CreateInMemory()
            UsdGeom.Xform.Define(stage, '/World/ActuatedTool').AddTranslateOp().Set(Gf.Vec3d(.4,.2,.1))
            before = stage.GetRootLayer().ExportToString()
            fixture = dict(body_path='/World/ActuatedTool/Diagnostics/BeltControl/Sphere',diameter_m=.04,
                           start_center_world=[.5,.6,.7],tool_frame_world=np.eye(3).tolist())
            (trial/'belt_fixture.json').write_text(json.dumps(fixture))
            report = restore_recorded_display_fixture(stage,trial)
            prim = stage.GetPrimAtPath(fixture['body_path'])
            self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))
            self.assertEqual(UsdGeom.Sphere(prim).GetRadiusAttr().Get(),.02)
            self.assertEqual(before,stage.GetRootLayer().ExportToString())
            np.testing.assert_allclose(np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim))[3,:3],[.5,.6,.7])
            self.assertTrue(report['fixture_only'])

    def test_session_fixture_pose_updates_work_with_default_root_edit_target(self):
        with tempfile.TemporaryDirectory() as directory:
            trial=Path(directory);stage=Usd.Stage.CreateInMemory()
            UsdGeom.Xform.Define(stage,'/World/ActuatedTool').AddTranslateOp().Set(Gf.Vec3d(.4,.2,.1))
            path='/World/ActuatedTool/Diagnostics/BeltControl/Sphere'
            fixture=dict(body_path=path,diameter_m=.03,start_center_world=[.5,.6,.7],
                         tool_frame_world=np.eye(3).tolist())
            (trial/'belt_fixture.json').write_text(json.dumps(fixture))
            before=stage.GetRootLayer().ExportToString()
            restore_recorded_display_fixture(stage,trial)
            self.assertEqual(stage.GetEditTarget().GetLayer(),stage.GetRootLayer())
            display=MeasuredPoseDisplay(stage,[path])
            for center in ([.5,.6,.7],[.5,.56,.7],[.51,.53,.7]):
                display.update(np.array([[*center,0.,0.,0.,1.]]))
                np.testing.assert_allclose(np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(
                    stage.GetPrimAtPath(path)))[3,:3],center,atol=1e-15)
            self.assertEqual(stage.GetEditTarget().GetLayer(),stage.GetRootLayer())
            self.assertEqual(stage.GetRootLayer().ExportToString(),before)

    def test_exact_existing_formula_for_nested_static_and_measured_parents(self):
        stage = Usd.Stage.CreateInMemory()
        paths = ['/World/static/bodyA', '/World/static/bodyA/bridge/bodyB', '/World/other/bodyC']
        transforms = {'/World': [0,0,0], '/World/static': [.1,.2,.3], paths[0]: [.4,.5,.6],
                      '/World/static/bodyA/bridge': [.03,-.02,.01], paths[1]: [-.1,-.2,-.3],
                      '/World/other': [-.4,.2,-.1], paths[2]: [.2,-.3,.5]}
        rng = np.random.default_rng(923)
        for path, position in transforms.items():
            matrix = np.eye(4)
            matrix[:3,:3] = Rotation.random(random_state=rng).as_matrix()
            matrix[:3,3] = position
            UsdGeom.Xform.Define(stage, path).MakeMatrixXform().Set(Gf.Matrix4d(matrix.T.tolist()))
        copy = Usd.Stage.CreateInMemory()
        copy.GetRootLayer().ImportFromString(stage.GetRootLayer().ExportToString())
        references, ops = original_pose_setup(stage, paths)
        display = MeasuredPoseDisplay(copy, paths)
        for _ in range(10):
            poses = np.column_stack((rng.normal(size=(3,3)), Rotation.random(3,random_state=rng).as_quat()))
            worlds = np.repeat(np.eye(4)[None],len(paths),axis=0)
            worlds[:,:3,:3] = Rotation.from_quat(poses[:,3:7]).as_matrix()
            worlds[:,:3,3] = poses[:,:3]
            for i,op in enumerate(ops):
                ancestor,relative = references[i]
                parent = relative if ancestor is None else worlds[ancestor] @ relative
                op.Set(Gf.Matrix4d((np.linalg.inv(parent) @ worlds[i]).T.tolist()))
            display.update(poses)
            for original, extracted in zip(ops, display.ops):
                np.testing.assert_array_equal(np.asarray(original.Get()), np.asarray(extracted.Get()))
            cache = UsdGeom.XformCache()
            for i,path in enumerate(paths):
                np.testing.assert_allclose(np.asarray(cache.GetLocalToWorldTransform(copy.GetPrimAtPath(path))).T,
                                           worlds[i], atol=1e-15)

    def test_missing_body_is_an_error(self):
        with self.assertRaisesRegex(ValueError, 'missing'):
            MeasuredPoseDisplay(Usd.Stage.CreateInMemory(), ['/World/missing'])


if __name__ == '__main__':
    unittest.main()
