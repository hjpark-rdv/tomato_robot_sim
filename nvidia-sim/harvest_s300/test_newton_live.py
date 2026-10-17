"""Newton display mapping/transport/lifecycle without Kit or a GPU."""
import json
from pathlib import Path
import sys
import tempfile
import textwrap
import unittest

import numpy as np

from motion_live import MotionLiveReader
from newton_live import (NewtonLivePublisher, NewtonPoseMapping, NewtonViewerProcess,
                         display_publication_metrics, required_skin_body_paths,
                         write_display_ack, process_is_descendant, WallClockPacer,
                         write_interactive_start)


class NewtonLiveTests(unittest.TestCase):
    def test_pacing_preserves_sim_clock_and_does_not_catch_up_after_stall(self):
        now=[0.]
        waits=[]
        def sleep(dt):
            waits.append(dt);now[0]+=dt
        pacer=WallClockPacer(clock=lambda:now[0],sleep=sleep)
        now[0]=.02
        self.assertAlmostEqual(pacer.wait(.1),.08)
        now[0]+=.3
        self.assertEqual(pacer.wait(.2),0.)
        self.assertAlmostEqual(pacer.wait(.3),.1)
        self.assertAlmostEqual(now[0],.5)
        self.assertAlmostEqual(pacer.last_sim,.3)
        with self.assertRaises(ValueError):pacer.wait(.2)
        with self.assertRaises(ValueError):WallClockPacer(0)

    def test_interactive_start_is_bound_to_viewer_and_is_one_shot(self):
        with tempfile.TemporaryDirectory() as root:
            assembly,poses,mapping=self.fixture(root)
            publisher=NewtonLivePublisher(Path(root)/'trial',assembly,['stem','tool'],mapping)
            publisher.publish(poses,.04,0.)
            with NewtonViewerProcess(publisher.output,isaac_python=sys.executable,
                    viewer_script=self.fake_viewer(root)) as viewer:
                ready=viewer.wait_ready(5.)
                write_interactive_start(publisher.output,viewer.display_source_sha256,
                    render_updates=20,pid=ready['pid'])
                write_interactive_start(publisher.output,'wrong',render_updates=1,pid=-1)
                viewer.wait_user_start()
                self.assertEqual(viewer.report['interactive_start']['pid'],ready['pid'])
                self.assertEqual(MotionLiveReader(publisher.output).read()['time'],0.)
                request=publisher.output/'interactive_start.json'
                request.unlink()
                write_interactive_start(publisher.output,viewer.display_source_sha256,
                    render_updates=20,pid=-1)
                with self.assertRaisesRegex(RuntimeError,'does not match'):
                    viewer.wait_user_start()
            publisher.close()

    def fixture(self, root):
        root = Path(root)
        assembly = root / 'assembly'
        assembly.mkdir()
        (assembly / 'scene.usda').write_text('#usda 1.0\n')
        (assembly / 'motion_manifest.json').write_text(json.dumps(
            dict(plant=dict(skin_paths=['/World/ElasticTarget/Skin/rod']))))
        np.savez(assembly / 'skin.npz', **{'0_body_paths': ['/World/ElasticTarget/rod']})
        poses = np.array([[1., 2., 3., 0., 0., 0., 1.],
                          [4., 5., 6., 0., 0., 0., 1.]])
        mapping = {'/World/ElasticTarget/rod': 'stem', '/World/ActuatedTool/fixed': 'tool'}
        return assembly, poses, mapping

    def test_exact_mapping_transport_and_source_provenance(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            before = poses.copy()
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly,
                ['tool', 'stem'], mapping, physical_model_sha256='physical-model')
            self.assertTrue(publisher.publish(poses, .096, 0.))
            reader = MotionLiveReader(Path(root) / 'trial')
            np.testing.assert_array_equal(reader.read()['poses'], poses[[1, 0]])
            np.testing.assert_array_equal(before, poses)
            self.assertEqual(reader.metadata['physics_engine'], 'newton')
            self.assertEqual(reader.metadata['physical_model_sha256'], 'physical-model')
            self.assertNotEqual(reader.metadata['model_sha256'], 'physical-model')
            self.assertEqual(reader.verify_source(), assembly / 'scene.usda')
            publisher.close()
            self.assertEqual(reader.read()['status'], 'completed')

    def test_throttle_only_skips_display_and_force_keeps_final_frame(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            now = [0.]
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly,
                ['stem', 'tool'], mapping, fps=10., clock=lambda: now[0])
            self.assertTrue(publisher.due())
            self.assertTrue(publisher.publish(poses, .04, 0.))
            now[0] = .01
            self.assertFalse(publisher.due())
            self.assertFalse(publisher.publish(poses, .04, .01))
            poses[0, 0] = 7.
            self.assertTrue(publisher.publish(poses, .04, .02, force=True))
            publisher.close('penetration_limit')
            data = json.loads(publisher.report_path.read_text())
            self.assertEqual(data['published_measured_frames'], 2)
            self.assertEqual(data['skipped_by_publication_throttle'], 1)
            self.assertEqual(data['last_published_time_s'], .02)
            self.assertEqual(MotionLiveReader(publisher.output).read()['status'], 'stopped')

    def test_missing_skin_mapping_fails_before_stream_creation(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, _, _ = self.fixture(root)
            self.assertEqual(required_skin_body_paths(assembly), ['/World/ElasticTarget/rod'])
            with self.assertRaisesRegex(ValueError, 'required skin bodies'):
                NewtonLivePublisher(Path(root) / 'trial', assembly, ['tool'],
                    {'/World/ActuatedTool/fixed': 'tool'})
            self.assertFalse((Path(root) / 'trial' / 'live.bin').exists())

    def test_explicit_quaternion_conversion_does_not_mutate_input(self):
        mapping = NewtonPoseMapping(['body'], {'/World/body': 0}, quaternion_order='wxyz')
        poses = np.array([[1., 2., 3., 1., 0., 0., 0.]])
        original = poses.copy()
        np.testing.assert_array_equal(mapping.convert(poses), [[1., 2., 3., 0., 0., 0., 1.]])
        np.testing.assert_array_equal(poses, original)

    def test_native_to_usd_rigid_frame_correction(self):
        offset = np.eye(4)
        offset[:3, 3] = [1., 0., 0.]
        mapping = NewtonPoseMapping(['body'], {'/World/body': 'body'},
                                    native_from_usd={'/World/body': offset})
        # A +90 degree world yaw rotates the source-frame x offset into world y.
        poses = np.array([[2., 3., 4., 0., 0., 2**-.5, 2**-.5]])
        np.testing.assert_allclose(mapping.convert(poses)[0, :3], [2., 4., 4.], atol=1e-14)
        bad = offset.copy()
        bad[0, 0] = 2.
        with self.assertRaisesRegex(ValueError, 'rigid'):
            NewtonPoseMapping(['body'], {'/World/body': 0}, native_from_usd={'/World/body': bad})

    def test_mapping_rejects_unknown_body_zero_quaternion_and_bad_shape(self):
        with self.assertRaisesRegex(ValueError, 'Unknown native body'):
            NewtonPoseMapping(['body'], {'/World/body': 'other'})
        with self.assertRaisesRegex(ValueError, 'out of range'):
            NewtonPoseMapping(['body'], {'/World/body': 2})
        mapping = NewtonPoseMapping(['body'], {'/World/body': 0})
        with self.assertRaisesRegex(ValueError, 'shape'):
            mapping.convert(np.zeros((2, 7)))
        with self.assertRaisesRegex(ValueError, 'Zero measured'):
            mapping.convert(np.zeros((1, 7)))

    def test_publisher_rejects_backwards_time_even_during_throttle(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly,
                ['stem', 'tool'], mapping)
            publisher.publish(poses, .04, 1.)
            with self.assertRaisesRegex(ValueError, 'moved backwards'):
                publisher.publish(poses, .04, .9)
            publisher.close()
            with self.assertRaises(FileExistsError):
                NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)

    def test_display_drop_count_excludes_final_status_and_counts_startup_loss(self):
        observed = [dict(sequence=6, status='running'),
                    dict(sequence=10, status='running'),
                    dict(sequence=12, status='completed')]
        result = display_publication_metrics(observed, dict(status='completed', publications=5))
        self.assertEqual(result['displayed_unique_measured_publications'], 2)
        self.assertEqual(result['undisplayed_published_frames'], 3)
        self.assertTrue(result['undisplayed_count_complete'])
        self.assertFalse(result['final_status_update_counted_as_frame'])

    def test_display_drop_count_running_is_explicit_lower_bound(self):
        result = display_publication_metrics([dict(sequence=4, status='running')], dict(status='waiting'))
        self.assertFalse(result['undisplayed_count_complete'])
        self.assertEqual(result['undisplayed_published_frames'], 1)

    def fake_viewer(self, root, *, fails=False, physics_steps=0, ack_time=None):
        path = Path(root) / 'fake_viewer.py'
        path.write_text(textwrap.dedent(f'''\
            import argparse, json, pathlib, time, os, signal
            p=argparse.ArgumentParser()
            p.add_argument('trial');p.add_argument('--ready-file')
            a,_=p.parse_known_args()
            if {fails!r}: raise SystemExit(7)
            signal.signal(signal.SIGINT, signal.default_int_handler)
            metadata=json.loads((pathlib.Path(a.trial)/'live.json').read_text())
            pathlib.Path(a.ready_file).write_text(json.dumps(dict(pid=os.getpid(), physics_steps={physics_steps},
                model_sha256=metadata['model_sha256'], measured_time_s=0.)))
            if {ack_time!r} is not None:
                (pathlib.Path(a.trial)/'live_display_ack.json').write_text(json.dumps(dict(
                    pid=os.getpid(), sequence=2, last_sim_time_s={ack_time!r},
                    displayed_count=1, physics_steps=0, timeline_time_s=0.,
                    render_update_completed=True)))
            try:
                while True: time.sleep(.05)
            except KeyboardInterrupt: pass
            '''))
        return path

    def test_launch_readiness_and_cleanup_touch_only_viewer_process(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            with NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                viewer_script=self.fake_viewer(root)) as viewer:
                self.assertEqual(viewer.wait_ready(5.)['physics_steps'], 0)
                self.assertIsNone(viewer.process.poll())
            self.assertIsNotNone(viewer.process.poll())
            self.assertEqual(viewer.report['returncode'], 0)
            self.assertFalse(publisher.closed)
            publisher.close()

    def test_viewer_start_failure_is_reported_without_changing_physics_stream(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            viewer = NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                viewer_script=self.fake_viewer(root, fails=True))
            with self.assertRaisesRegex(RuntimeError, 'before readiness'):
                viewer.wait_ready(5.)
            viewer.close()
            self.assertEqual(viewer.report['returncode'], 7)
            self.assertEqual(MotionLiveReader(publisher.output).read()['status'], 'running')
            publisher.close()

    def test_viewer_readiness_rejects_claimed_physics_steps(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            with NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                viewer_script=self.fake_viewer(root, physics_steps=1)) as viewer:
                with self.assertRaisesRegex(RuntimeError, 'physics stepping'):
                    viewer.wait_ready(5.)
            publisher.close()

    def test_display_ack_hashes_measured_pose_after_completed_render(self):
        with tempfile.TemporaryDirectory() as root:
            _, poses, _ = self.fixture(root)
            snapshot = dict(sequence=8, time=1.25, status='running', poses=poses)
            ack = write_display_ack(root, snapshot, displayed_count=2,
                                    render_updates=5, timeline_time_s=0., pid=42)
            self.assertEqual(ack['last_sim_time_s'], 1.25)
            self.assertEqual(ack['sequence'], 8)
            self.assertEqual(ack['displayed_count'], 2)
            self.assertEqual(ack['physics_steps'], 0)
            self.assertEqual(len(ack['pose_sha256']), 64)
            self.assertEqual(json.loads((Path(root)/'live_display_ack.json').read_text()), ack)
            with self.assertRaisesRegex(RuntimeError, 'zero physics steps'):
                write_display_ack(root, snapshot, displayed_count=2,
                                  render_updates=5, timeline_time_s=.1)
            self.assertEqual(json.loads((Path(root)/'live_display_ack.json').read_text()), ack)

    def test_wait_displayed_accepts_final_render_ack_and_reports_liveness(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            with NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                viewer_script=self.fake_viewer(root, ack_time=1.5)) as viewer:
                viewer.wait_ready(5.)
                self.assertTrue(viewer.is_alive())
                self.assertEqual(viewer.wait_displayed(1.5, timeout=5.)['last_sim_time_s'], 1.5)
                self.assertEqual(viewer.report['final_display_ack']['sequence'], 2)
            self.assertFalse(viewer.is_alive())
            publisher.close()

    def test_wait_displayed_timeout_preserves_independent_producer(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            with NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                viewer_script=self.fake_viewer(root, ack_time=.5)) as viewer:
                viewer.wait_ready(5.)
                with self.assertRaisesRegex(TimeoutError, 'requested measured frame'):
                    viewer.wait_displayed(1., timeout=.05)
                self.assertEqual(viewer.report['display_error'], 'requested_frame_timeout')
                self.assertFalse(publisher.closed)
                self.assertTrue(viewer.is_alive())
            publisher.close()

    def test_wait_displayed_rejects_stale_other_viewer_ack(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            with NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                viewer_script=self.fake_viewer(root)) as viewer:
                viewer.wait_ready(5.)
                (publisher.output/'live_display_ack.json').write_text(json.dumps(dict(
                    pid=-1, sequence=100, last_sim_time_s=99., physics_steps=0,
                    timeline_time_s=0., render_update_completed=True)))
                with self.assertRaises(TimeoutError):
                    viewer.wait_displayed(1., timeout=.05)
            publisher.close()

    def test_shell_wrapper_and_child_viewer_have_distinct_verified_pids(self):
        with tempfile.TemporaryDirectory() as root:
            assembly, poses, mapping = self.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            script = self.fake_viewer(root, ack_time=1.5)
            wrapper = Path(root) / 'isaac_python.sh'
            wrapper.write_text('#!/bin/sh\n[ "$1" = "--no-ros-env" ] && shift\n'
                               + sys.executable + ' "$@" &\nchild=$!\nwait "$child"\n')
            wrapper.chmod(0o755)
            with NewtonViewerProcess(publisher.output, isaac_python=wrapper,
                                     viewer_script=script) as viewer:
                ready = viewer.wait_ready(5.)
                self.assertNotEqual(ready['pid'], viewer.process.pid)
                self.assertTrue(process_is_descendant(ready['pid'], viewer.process.pid))
                self.assertEqual(viewer.report['viewer_python_pid'], ready['pid'])
                self.assertEqual(viewer.wait_displayed(1.5, timeout=5.)['pid'], ready['pid'])
            publisher.close()


if __name__ == '__main__':
    unittest.main()
