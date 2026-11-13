"""Regression checks for moved mount collision bounds and continuous playback."""
import json
from pathlib import Path
import tempfile
import unittest

import mujoco as mj
import numpy as np

from preview_workspace_motion import collision_model, refit_body_bvh, WorkspaceMotion, edge_steps, duration
from newton_patrol_motion import VELOCITY


class WorkspacePreviewTests(unittest.TestCase):
    def test_side_mount_removes_the_plate_from_native_contacts_only_in_its_copy(self):
        source=mj.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="Robot_farmily_lift_platform"><joint type="slide" axis="0 0 1"/>
            <geom name="base" type="box" size=".05 .05 .05" pos="2 0 0"/>
            <geom name="plate" type="box" size=".2 .2 .02"/>
            <body name="Robot_link1" pos="3 0 0">
              <joint axis="0 1 0"/><geom type="sphere" size=".03"/>
            </body>
          </body>
          <body name="Robot_probe" pos="0 0 .03"><joint type="slide" axis="0 0 1"/>
            <geom type="sphere" size=".03"/>
          </body>
          <body name="Robot_world" pos="0 3 0">
            <geom name="s300_g0045" type="box" size=".1 .1 .1"/>
            <geom name="s300_g0050" type="box" size=".1 .1 .1"/>
            <geom name="s300_g0052" type="box" size=".1 .1 .1"/>
          </body>
        </worldbody></mujoco>''')
        data=mj.MjData(source);mj.mj_kinematics(source,data)
        platform=source.body('Robot_farmily_lift_platform').id;root=source.body('Robot_link1').id
        poses=np.tile(np.eye(4),(source.nbody+1,1,1))
        poses[:source.nbody,:3,3]=data.xpos
        poses[:source.nbody,:3,:3]=data.xmat.reshape(-1,3,3)
        poses[-1]=poses[platform]
        with tempfile.TemporaryDirectory()as folder:
            folder=Path(folder);(folder/'model').mkdir()
            (folder/'model/export.json').write_text(json.dumps(dict(geometry=[
                dict(name='base',source='/farmily_lift_platform/collisions/link0/base'),
                dict(name='plate',source='/collisions/real_mounting_plate')])))
            meta=dict(assets=str(folder),platform_id=platform,plate_id=source.nbody,
                      arm_bodies=[root],model_sha256='fixture')
            def plate_contacts(model):
                d=mj.MjData(model);mj.mj_kinematics(model,d);mj.mj_collision(model,d)
                gid=model.geom('plate').id
                return [c for c in d.contact if gid in (int(c.geom1),int(c.geom2))]
            self.assertTrue(plate_contacts(source))
            model,report=collision_model(source,source.qpos0.copy(),meta,poses,folder,layout_key='side')
            self.assertEqual(plate_contacts(model),[])
            self.assertEqual(report['removed_native_colliders'],['plate','s300_g0045','s300_g0050','s300_g0052'])
            for name in report['replacement_chassis_colliders']:
                self.assertEqual(model.geom_contype[model.geom(name).id],0)
                self.assertNotEqual(source.geom_contype[source.geom(name).id],0)
            self.assertTrue(plate_contacts(source))
            legacy,_=collision_model(source,source.qpos0.copy(),meta,poses,folder)
            self.assertTrue(plate_contacts(legacy))

    def test_moved_plate_collision_is_detected_after_bvh_refit(self):
        model = mj.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="platform"><joint type="slide" axis="0 0 1"/>
            <geom name="plate" type="box" size=".05 .05 .05" pos="0 0 .5"/>
            <geom type="box" size=".05 .05 .05" pos="-1 0 .5"/>
          </body>
          <body pos=".4 0 .5"><joint type="slide" axis="0 1 0"/>
            <geom name="probe" type="sphere" size=".03"/>
          </body>
        </worldbody></mujoco>''')
        data = mj.MjData(model)
        mj.mj_kinematics(model, data)
        mj.mj_collision(model, data)
        self.assertEqual(data.ncon, 0)
        model.geom_pos[model.geom('plate').id, 0] = .4
        mj.mj_setConst(model, data)
        mj.mj_kinematics(model, data)
        refit_body_bvh(model, model.body('platform').id)
        mj.mj_collision(model, data)
        self.assertGreater(data.ncon, 0)
        self.assertLess(min(c.dist for c in data.contact), -.001)

    def test_forward_reverse_loop_has_no_joint_reset_jump(self):
        a, b, c = np.zeros(6), np.full(6, .4), np.full(6, -.2)
        segments = [dict(points=np.array([a,b]),duration_s=2.,start_capture_m=[0]*3,target_capture_m=[1]*3),
                    dict(points=np.array([b,c]),duration_s=3.,start_capture_m=[1]*3,target_capture_m=[2]*3)]
        motion = WorkspaceMotion(segments, {})
        for t in [2., 5., 8., 14.]:
            left, _ = motion.sample(t-1e-6)
            right, _ = motion.sample(t+1e-6)
            np.testing.assert_allclose(left, right, atol=1e-9)
        np.testing.assert_allclose(motion.sample(5.)[0], c)
        np.testing.assert_allclose(motion.sample(8.)[0], b)

    def test_mixed_lift_and_arm_edges_respect_separate_units(self):
        a = np.zeros(7)
        b = np.r_[.011, np.full(6, .01)]
        steps = edge_steps(a, b)
        self.assertLessEqual(abs(b[0]-a[0])/steps, .0005)
        self.assertLessEqual(np.linalg.norm(b[1:]-a[1:])/steps, .002)

    def test_coordinated_playback_moves_lift_and_obeys_preview_speeds(self):
        a = np.r_[.75, np.zeros(6)]
        b = np.r_[.35, np.full(6, .2)]
        points = np.array([a,b])
        span = duration(points)
        motion = WorkspaceMotion([
            dict(points=np.array([a,a]),duration_s=1.,start_capture_m=[0]*3,target_capture_m=[0]*3),
            dict(points=points,duration_s=span,start_capture_m=[0]*3,target_capture_m=[1]*3)], {})
        times = np.linspace(1., 1.+span, 1001)
        states = np.array([motion.sample(t)[0] for t in times])
        velocity = np.diff(states,axis=0)/np.diff(times)[:,None]
        self.assertTrue(np.all(np.max(abs(velocity),axis=0) <= VELOCITY[1:8]))
        self.assertAlmostEqual(motion.sample(1.)[1]['lift_height_m'], .75)
        self.assertAlmostEqual(motion.sample(1.+span)[1]['lift_height_m'], .35)


if __name__ == '__main__':
    unittest.main()
