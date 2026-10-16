"""Camera-motion evidence controls without Kit, a GPU or physics stepping."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

from newton_live import (CameraMotionEvidence, NewtonLivePublisher,
                         NewtonViewerProcess, camera_test_position, write_display_ack)
import test_newton_live as fixtures


class CameraMotionTests(unittest.TestCase):
    def test_motion_is_bounded_and_does_not_modify_reference_vectors(self):
        eye = np.array([1., 2., 3.])
        right = np.array([2., 0., 0.])
        up = np.array([0., 0., 3.])
        before = [v.copy() for v in (eye, right, up)]
        points = np.asarray([camera_test_position(eye, right, up, t) for t in np.linspace(0., 12., 121)])
        np.testing.assert_array_equal(points[0], eye)
        self.assertGreater(np.linalg.norm(points - eye, axis=1).max(), .1)
        self.assertLessEqual(np.linalg.norm(points - eye, axis=1).max(), np.hypot(.15, .08) + 1e-12)
        for actual, original in zip((eye, right, up), before):
            np.testing.assert_array_equal(actual, original)
        with self.assertRaises(ValueError):
            camera_test_position(eye, right, up, float('nan'))

    def test_only_actual_post_render_samples_count_as_movement(self):
        evidence = CameraMotionEvidence('/World/LiveEntryCamera', [1., 2., 3.])
        evidence.record([1., 2., 3.], render_update=1, elapsed_wall_s=0.,
                        measured_time_s=0., timeline_time_s=0.)
        self.assertEqual(evidence.summary()['movement_count'], 0)
        evidence.record([1.1, 2., 3.], render_update=2, elapsed_wall_s=.1,
                        measured_time_s=1., timeline_time_s=0.)
        self.assertEqual(evidence.summary()['sample_count'], 2)
        self.assertEqual(evidence.summary()['movement_count'], 1)
        self.assertAlmostEqual(evidence.summary()['maximum_displacement_m'], .1)
        with self.assertRaisesRegex(RuntimeError, 'zero display physics'):
            evidence.record([1., 2., 3.], render_update=3, elapsed_wall_s=.2,
                            measured_time_s=1., timeline_time_s=.01)
        with self.assertRaises(ValueError):
            evidence.record([1., 2., 3.], render_update=2, elapsed_wall_s=.2,
                            measured_time_s=1., timeline_time_s=0.)

    def test_ack_keeps_measured_pose_hash_and_adds_camera_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            poses = np.array([[1., 2., 3., 0., 0., 0., 1.]])
            snapshot = dict(sequence=4, time=1.5, status='running', poses=poses)
            camera = CameraMotionEvidence('/World/Camera', [0., 0., 0.])
            camera.record([0., 0., 0.], render_update=1, elapsed_wall_s=0.,
                          measured_time_s=0., timeline_time_s=0.)
            camera.record([.1, 0., 0.], render_update=2, elapsed_wall_s=.1,
                          measured_time_s=1.5, timeline_time_s=0.)
            ack = write_display_ack(directory, snapshot, displayed_count=1,
                render_updates=2, timeline_time_s=0., camera_motion=camera.summary())
            self.assertEqual(ack['pose_sha256'], hashlib.sha256(poses.astype('<f8').tobytes()).hexdigest())
            self.assertEqual(ack['camera_test_motion']['movement_count'], 1)
            self.assertEqual(ack['physics_steps'], 0)
            bad = camera.summary() | {'physics_steps': 1}
            with self.assertRaises(RuntimeError):
                write_display_ack(directory, snapshot, displayed_count=1,
                    render_updates=2, timeline_time_s=0., camera_motion=bad)

    def test_process_dispatch_and_camera_flag_alone_do_not_prove_motion(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = fixtures.NewtonLiveTests()
            assembly, poses, mapping = fixture.fixture(root)
            publisher = NewtonLivePublisher(Path(root) / 'trial', assembly, ['stem', 'tool'], mapping)
            publisher.publish(poses, .096, 0.)
            with NewtonViewerProcess(publisher.output, isaac_python=sys.executable,
                    viewer_script=fixture.fake_viewer(root, ack_time=1.5),
                    camera_test_motion=True) as viewer:
                viewer.wait_ready(5.)
                self.assertIn('--camera-test-motion', viewer.report['command'])
                with self.assertRaisesRegex(RuntimeError, 'camera evidence'):
                    viewer.wait_displayed(1.5, timeout=1.)
                camera = CameraMotionEvidence('/World/Camera', [0., 0., 0.])
                camera.record([0., 0., 0.], render_update=1, elapsed_wall_s=0.,
                              measured_time_s=1.5, timeline_time_s=0.)
                snapshot = dict(sequence=2, time=1.5, status='running', poses=poses)
                write_display_ack(publisher.output, snapshot, displayed_count=1,
                    render_updates=1, timeline_time_s=0., pid=viewer.report['viewer_python_pid'],
                    camera_motion=camera.summary())
                with self.assertRaises(TimeoutError):
                    viewer.wait_displayed(1.5, timeout=.05)
                camera.record([.1, 0., 0.], render_update=2, elapsed_wall_s=.1,
                              measured_time_s=1.5, timeline_time_s=0.)
                write_display_ack(publisher.output, snapshot, displayed_count=1,
                    render_updates=2, timeline_time_s=0., pid=viewer.report['viewer_python_pid'],
                    camera_motion=camera.summary())
                ack = viewer.wait_displayed(1.5, timeout=1.)
                self.assertEqual(ack['camera_test_motion']['movement_count'], 1)
                self.assertFalse(publisher.closed)
            publisher.close()


if __name__ == '__main__':
    unittest.main()
