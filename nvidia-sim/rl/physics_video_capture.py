"""Read actual GPU probe poses; render later without advancing physics.

Clips explicitly distinguish robot trials, applied-force tests and a preinserted
kinematic ring fixture. A camera never changes the simulated scene.
"""
import numpy as np
from pose_video_capture import body_poses, export_scene
from dataset_design import write_json


class PhysicsVideoCapture:
    def __init__(self, slot, directory):
        self.slot = slot
        self.directory = directory
        self.active = False

    def begin(self, name, title, subtitle, extra_body=None):
        if self.active:
            raise RuntimeError('Finish the previous clip before beginning another')
        self.output = self.directory / name
        self.output.mkdir(parents=True, exist_ok=True)
        self.extra_body = extra_body
        extra = [(extra_body[0], self.extra_pose())] if extra_body else []
        export_scene(self.slot, self.output / 'scene.npz', extra_bodies=extra)
        self.poses = [self.read()]
        self.times = [0.]
        self.ticks = 0
        self.trace = []
        self.title, self.subtitle = title, subtitle
        self.active = True

    def extra_pose(self):
        p = self.extra_body[1].get_transforms()[0].cpu().numpy()
        return p[[0, 1, 2, 6, 3, 4, 5]].copy()  # native xyzw -> wxyz

    def read(self):
        p = body_poses(self.slot)
        return np.concatenate((p, self.extra_pose()[None])) if self.extra_body else p

    def sample(self, row=None):
        if not self.active:
            return
        self.ticks += 1
        if row is not None:
            self.trace.append(dict(row, main_stem_displacement_m=row['main_displacement_m']))
        # Sample actual states; 15 simulation fps -> 30 output fps at 2x.
        if self.ticks % max(1, round(1 / (15 * self.slot.physics_dt))) == 0:
            self.times.append(self.ticks * self.slot.physics_dt)
            self.poses.append(self.read())

    def finish(self, result, trace=None, retained_hook=False, first_contact=None, extra_metadata=None):
        if not self.active:
            return
        duration = self.ticks * self.slot.physics_dt
        if self.times[-1] < duration - 1e-9:
            self.times.append(duration)
            self.poses.append(self.read())
        poses = np.asarray(self.poses)
        if not np.isfinite(poses).all() or duration <= 0:
            raise RuntimeError('Invalid physics recording')
        np.savez_compressed(self.output / 'motion.npz', poses=poses, times_s=np.asarray(self.times))
        rows = self.trace if trace is None else trace
        control_dt=self.slot.physics_dt if trace is None else self.slot.step_dt
        expected=int(np.ceil(self.ticks/(control_dt/self.slot.physics_dt)))
        if len(rows) != expected:
            raise RuntimeError(f'Trace/physics mismatch: {len(rows)} vs {self.ticks}')
        write_json(self.output / 'trace.json', rows)
        write_json(self.output / 'recording.json', dict(
            candidate_id=self.output.name, title=self.title, subtitle=self.subtitle,
            result=result, retained_hook=retained_hook,
            first_contact=first_contact,
            first_contact_object=(first_contact or {}).get('object'),
            control_dt=control_dt, physics_dt=self.slot.physics_dt, simulated_duration_s=duration,
            capture_fps=15, output_fps=30, playback_speed=2,
            validation=dict(recorded_physics=dict(passed=True, finite=True,
                physics_steps=self.ticks, pose_samples=len(poses),
                scope='Actual native poses; no animation interpolation or additional dynamics')),
            extra_body=self.extra_body[0] if self.extra_body else None, **(extra_metadata or {})))
        self.active = False
        print('[PHYSICS VIDEO CAPTURE]', self.output, duration, flush=True)
