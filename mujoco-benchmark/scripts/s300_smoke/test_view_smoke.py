"""Native rendering regression: captions must not mask an empty scene."""
import sys
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import render_backend  # Configure EGL before importing MuJoCo.
import mujoco as mj
import numpy as np
from view_smoke import (restore_frame, scene_pixels, resolve_model, make_camera,
                        caption_lines, replay_times, load_replay_states,
                        stream_sha256, verify_replay_integrity, main)


class ReplayIntegrity(unittest.TestCase):
    def fixture(self,directory):
        folder=Path(directory)
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody><body>
          <freejoint/><geom type="sphere" size=".02"/></body></worldbody></mujoco>''')
        mj.mj_saveModel(model,str(folder/'model.mjb'))
        data=mj.MjData(model)
        np.savez_compressed(folder/'states.npz',time=np.array([0.]),qpos=data.qpos[None,:])
        summary=dict(model_mjb_sha256=stream_sha256(folder/'model.mjb'),
                     states_sha256=stream_sha256(folder/'states.npz'))
        (folder/'summary.json').write_text(json.dumps(summary))
        return folder,summary

    def assert_rejected_before_native_load(self,folder,label):
        reader=mock.Mock(side_effect=AssertionError('Native model opened before integrity validation'))
        native=SimpleNamespace(MjModel=SimpleNamespace(from_binary_path=reader))
        with mock.patch.object(sys,'argv',['view_smoke.py',str(folder),'--gui']), \
             mock.patch.dict(sys.modules,{'mujoco':native}):
            with self.assertRaisesRegex(ValueError,'Replay '+label+' SHA256 mismatch'):
                main()
        reader.assert_not_called()

    def test_matching_native_snapshot_and_states_are_verified(self):
        with tempfile.TemporaryDirectory() as directory:
            folder,summary=self.fixture(directory)
            self.assertEqual(verify_replay_integrity(folder,summary),
                             dict(model_sha256_verified=True,states_sha256_verified=True))

    def test_old_render_python_is_rejected_without_recursive_reexec(self):
        with tempfile.TemporaryDirectory() as directory:
            folder,_=self.fixture(directory)
            render_python=Path(__file__).resolve().parents[2]/'.mujoco315-render-venv/bin/python'
            with mock.patch.object(sys,'argv',['view_smoke.py',str(folder),'--quality','ultra']), \
                 mock.patch.object(sys,'version_info',(3,11,0)), \
                 mock.patch.object(sys,'executable',str(render_python)), \
                 mock.patch.object(Path,'is_file',return_value=True), \
                 mock.patch('view_smoke.os.execv') as restart:
                with self.assertRaisesRegex(RuntimeError,'requires Python >=3.12'):
                    main()
                restart.assert_not_called()

    def test_changed_model_rejected_before_native_load(self):
        with tempfile.TemporaryDirectory() as directory:
            folder,_=self.fixture(directory)
            with (folder/'model.mjb').open('ab') as stream:stream.write(b'changed native bytes')
            self.assert_rejected_before_native_load(folder,'model')

    def test_changed_states_rejected_before_model_or_archive_load(self):
        with tempfile.TemporaryDirectory() as directory:
            folder,_=self.fixture(directory)
            (folder/'states.npz').write_bytes(b'not the saved archive')
            self.assert_rejected_before_native_load(folder,'states')

    def test_old_smoke_without_hashes_keeps_parent_model_and_archive_loading(self):
        with tempfile.TemporaryDirectory() as directory:
            folder,_=self.fixture(directory)
            trial=folder/'old_smoke';trial.mkdir()
            (folder/'states.npz').rename(trial/'states.npz')
            self.assertEqual(resolve_model(trial,{}),folder/'model.mjb')
            self.assertEqual(verify_replay_integrity(trial,{}),
                             dict(model_sha256_verified=False,states_sha256_verified=False))
            model=mj.MjModel.from_binary_path(str(resolve_model(trial,{})))
            states=load_replay_states(trial/'states.npz')
            self.assertEqual(states['qpos'].shape,(1,model.nq))

    def test_large_hash_uses_bounded_read_chunks(self):
        payload=b'x'*(2*1024*1024+37)
        class TrackingStream(io.BytesIO):
            def __init__(self,data):super().__init__(data);self.read_sizes=[]
            def read(self,size=-1):self.read_sizes.append(size);return super().read(size)
        stream=TrackingStream(payload)
        with mock.patch.object(Path,'open',return_value=stream):
            self.assertEqual(stream_sha256('/unused/large_model.mjb'),
                             hashlib.sha256(payload).hexdigest())
        self.assertGreaterEqual(len(stream.read_sizes),3)
        self.assertTrue(all(0<size<=1024*1024 for size in stream.read_sizes))


class PlaybackRendering(unittest.TestCase):
    def test_exact_run_model_precedes_old_smoke_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            parent=Path(directory);trial=parent/'trial';trial.mkdir()
            (parent/'model.mjb').write_bytes(b'old model')
            self.assertEqual(resolve_model(trial,{}),parent/'model.mjb')
            (trial/'model.mjb').write_bytes(b'exact full run model')
            self.assertEqual(resolve_model(trial,{}),trial/'model.mjb')

    def test_dynamic_camera_and_caption_do_not_claim_prescribed_tool(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
           <body name="Tool_fixed" pos=".2 1.4 1.2"><geom type="box" size=".1 .1 .1"/></body>
           </worldbody></mujoco>''')
        data=mj.MjData(model);mj.mj_kinematics(model,data)
        summary=dict(mode='dynamic_robot',mujoco_version='3.15.0',hz=120,first_1mm_violation=None)
        camera,name=make_camera(mj,model,data,'auto',summary)
        self.assertEqual(name,'aisle');self.assertEqual(camera.type,mj.mjtCamera.mjCAMERA_FREE)
        self.assertAlmostEqual(camera.lookat[1],1.55)
        states=dict(time=np.array([2.]),phase=np.array(['approach']))
        caption=' '.join(caption_lines(summary,states,0,np.array([],dtype=int)))
        self.assertIn('driven rail / lift / arm / jaws',caption)
        self.assertNotIn('prescribed tool',caption)
        self.assertIn('approach',caption)

    def test_belt_camera_uses_measured_tool_local_frame(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
           <body name="Tool_fixed" pos=".2 1.4 1.2" quat=".707106781186548 0 0 .707106781186548">
           <geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>''')
        data=mj.MjData(model);mj.mj_kinematics(model,data);mj.mj_comPos(model,data);mj.mj_camlight(model,data)
        tool=model.body('Tool_fixed').id;rotation=data.xmat[tool].reshape(3,3)
        for requested,local_eye,local_target in (
            ('belt',[0.,.09,.55],[0.,.10,.04]),
            ('fruit',[-.12,.10,.30],[0.,.18,.04]),
            ('harvest',[0.,.22,.08],[0.,.12,.03]),
        ):
            with self.subTest(camera=requested):
                camera,name=make_camera(mj,model,data,requested,dict(mode='dynamic_robot'))
                target=data.xpos[tool]+rotation@np.array(local_target)
                eye=data.xpos[tool]+rotation@np.array(local_eye)
                np.testing.assert_allclose(camera.lookat,target,atol=1e-12)
                scene=mj.MjvScene(model,maxgeom=100)
                mj.mjv_updateScene(model,data,mj.MjvOption(),None,camera,mj.mjtCatBit.mjCAT_ALL,scene)
                actual_eye=(scene.camera[0].pos+scene.camera[1].pos)/2
                np.testing.assert_allclose(actual_eye,eye,atol=1e-6)
                self.assertEqual(name,requested)

    def test_diagnostic_headlight_leaves_physics_and_geometry_intact(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
           <body name="Tool_fixed"><geom type="box" size=".1 .1 .1"/></body>
           </worldbody></mujoco>''')
        data=mj.MjData(model);mj.mj_kinematics(model,data)
        arrays=('geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity','geom_friction','geom_solref')
        before={key:getattr(model,key).copy() for key in arrays}
        timestep=model.opt.timestep
        make_camera(mj,model,data,'harvest',dict(mode='dynamic_robot'))
        np.testing.assert_allclose(model.vis.headlight.ambient,[.6,.6,.6])
        np.testing.assert_allclose(model.vis.headlight.diffuse,[.8,.8,.8])
        self.assertEqual(model.vis.headlight.active,1)
        for key,values in before.items():np.testing.assert_array_equal(getattr(model,key),values)
        self.assertEqual(model.opt.timestep,timestep)

    def test_velocity_optional_arrays_and_no_physics_advance(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody><body>
          <freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>''')
        data=mj.MjData(model)
        qpos=data.qpos.copy();qpos[:3]=[1.,2.,3.]
        states=dict(time=np.array([8.]),qpos=np.array([qpos]),qvel=np.array([[1.,2.,3.,.1,.2,.3]]))
        restore_frame(mj,model,data,states,0)
        np.testing.assert_array_equal(data.qpos,qpos)
        np.testing.assert_array_equal(data.qvel,states['qvel'][0])
        self.assertEqual(data.time,8.)
        np.testing.assert_array_equal(data.xpos[1],[1.,2.,3.])

    def test_detachment_counts_only_fruit_attachment_constraints(self):
        summary=dict(mode='dynamic_robot',first_1mm_violation=dict(t=3.))
        states=dict(time=np.array([2.,4.]),eq_active=np.array([[0,1,1],[0,0,1]]))
        ids=np.array([1,2])
        first=' '.join(caption_lines(summary,states,0,ids))
        second=' '.join(caption_lines(summary,states,1,ids))
        self.assertIn('detached=0',first);self.assertIn('detached=1',second)
        self.assertNotIn('criterion exceeded',first);self.assertIn('criterion exceeded',second)

    def test_last_stopping_state_is_not_lost_between_video_samples(self):
        times=replay_times(45.,55.19583333335029,30)
        self.assertAlmostEqual(times[-1],55.19583333335029)
        self.assertTrue(np.all(np.diff(times)>0))
        self.assertLess(times[-1]-times[-2],1/30)
        np.testing.assert_allclose(replay_times(0.,1.,30),np.linspace(0.,1.,31))

    def test_replay_arrays_remain_available_after_archive_is_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'states.npz'
            qpos=np.array([[1.,2.,3.],[4.,5.,6.]])
            np.savez_compressed(path,time=np.array([0.,1.]),qpos=qpos,
                                phase=np.array(['prepare','close']),unused_large_feature=np.ones((20,20)))
            states=load_replay_states(path)
            path.unlink()
            np.testing.assert_array_equal(states['qpos'],qpos)
            self.assertEqual(states['phase'][1],'close')
            self.assertNotIn('unused_large_feature',states)

    def test_black_frame_rejected_before_caption(self):
        with self.assertRaises(RuntimeError):
            scene_pixels(np.zeros((120,160,3),dtype=np.uint8))

    def test_visible_stored_pose_without_physics_step(self):
        model=mj.MjModel.from_xml_string('''<mujoco>
          <worldbody>
            <camera name="top" pos="0 0 2"/>
            <light pos="0 0 2" dir="0 0 -1"/>
            <body name="fruit"><freejoint/>
              <geom type="sphere" size=".2" rgba=".8 .1 .1 1"/>
            </body>
          </worldbody></mujoco>''')
        data=mj.MjData(model)
        states={k:np.array([getattr(data,k).copy()]) for k in
                ('qpos','eq_active','mocap_pos','mocap_quat')}
        states['time']=np.array([3.25])
        # The old replay left these zero and rendered black.
        self.assertFalse(data.cam_xpos.any())
        restore_frame(mj,model,data,states,0)
        np.testing.assert_allclose(data.cam_xpos,[[0,0,2]])
        np.testing.assert_allclose(data.light_xpos,[[0,0,2]])
        np.testing.assert_array_equal(data.qpos,states['qpos'][0])
        self.assertEqual(data.time,3.25)
        self.assertFalse(data.qvel.any())
        with mj.Renderer(model,height=120,width=160) as renderer:
            renderer.update_scene(data,camera='top')
            rgb=renderer.render()
            scene_pixels(rgb)
            self.assertGreater(int(rgb[60,80,0]),int(rgb[60,80,1])+20)
            self.assertGreater(np.count_nonzero(rgb[:,:,0]>rgb[:,:,1]+20),100)


if __name__=='__main__':unittest.main()
