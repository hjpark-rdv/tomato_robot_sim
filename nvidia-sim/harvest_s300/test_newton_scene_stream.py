import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from concurrent.futures import Future

import mujoco as mj
import numpy as np

from newton_scene_chunks import PackedScene,pack_scene,geom_y_bounds,ray_radius,stream_manifest,prepare
from newton_scene_stream import StreamingCamera
from newton_viser import geom_color,geom_mesh


class ScenePackingTests(unittest.TestCase):
    def test_subset_preserves_mesh_material_transform_and_body_identity(self):
        m=mj.MjModel.from_xml_string('''<mujoco><asset>
          <material name="red" rgba=".9 .1 .2 1"/>
          <mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
          </asset><worldbody><geom name="excluded" type="box" size="1 1 1" pos="0 40 0"/>
          <geom name="mesh" type="mesh" mesh="tetra" pos="0 2 1" euler="0 0 45" material="red"/>
          <body name="robot"><geom name="tool" type="sphere" size=".1"/></body></worldbody></mujoco>''')
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'pack.npz';pack_scene(m,[1,2],p);packed=PackedScene(p)
            self.assertEqual([packed.body(i).name for i in range(packed.nbody)],['world','robot'])
            for old,new in [(1,0),(2,1)]:
                np.testing.assert_array_equal(geom_color(m,old),geom_color(packed,new))
                np.testing.assert_array_equal(geom_mesh(m,old).vertices,geom_mesh(packed,new).vertices)
                np.testing.assert_array_equal(geom_mesh(m,old).faces,geom_mesh(packed,new).faces)
            with self.assertRaisesRegex(ValueError,'SHA'):PackedScene(p,'invalid')

    def test_bounds_cover_surface_crossing_edge_and_optical_corner(self):
        m=mj.MjModel.from_xml_string('<mujoco><worldbody><geom type="box" size=".1 3 .1" pos="0 5 0"/></worldbody></mujoco>')
        np.testing.assert_allclose(geom_y_bounds(m),[[2,8]])
        radius=ray_radius(dict(K=[[100,0,100],[0,100,100],[0,0,1]],resolution_wh=[200,200],depth_range_m=[.1,5]))
        self.assertAlmostEqual(radius,5*np.sqrt(3))

    def test_auto_never_starts_offline_packing_and_explicit_stream_requires_it(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(stream_manifest(folder))
            with self.assertRaisesRegex(ValueError,'Prepare'):stream_manifest(folder,'stream')

    def test_distant_farm_growth_does_not_increase_local_payload(self):
        with tempfile.TemporaryDirectory() as folder:
            counts=[]
            for n in (10,100):
                source=Path(folder)/str(n);(source/'model').mkdir(parents=True);(source/'inputs').mkdir()
                geoms=''.join(f'<geom name="plant{i}" type="box" size=".05 .2 .1" pos="0 {i} 0"/>' for i in range(n))
                (source/'model/scene.xml').write_text('<mujoco><worldbody>'+geoms+'</worldbody></mujoco>')
                (source/'inputs/input_model.mjb').write_bytes(b'physics identity only; packer never edits this')
                mount=dict(K=[[100,0,100],[0,100,100],[0,0,1]],resolution_wh=[200,200],depth_range_m=[.1,.5])
                (source/'patrol_assets.json').write_text(json.dumps(dict(mounted_rgbd=mount,start_station_m=0,end_station_m=4)))
                catalog=dict(objects={str(i+1):dict(plant=f'plant{i}',class_id=2) for i in range(n)},
                             geom_to_instance={f'plant{i}':i+1 for i in range(n)})
                (source/'sensor_catalog.json').write_text(json.dumps(catalog))
                result=prepare(source,source/'scene_stream',stride=2,overlap=.2,camera_excursion=.2)
                counts.append(result['chunks'][0]['ngeom'])
                self.assertIsNotNone(stream_manifest(source))
                (source/'model/scene.xml').write_text('changed source')
                with self.assertRaisesRegex(ValueError,'source changed'):stream_manifest(source)
            self.assertEqual(counts,[4,4])


class Worker:
    def __init__(self,ready=False):self.ready=ready;self.ready_wall_s=1.;self.closed=False
    def poll_ready(self):return self.ready
    def close(self):self.closed=True


class ImmediatePool:
    def submit(self,function,*args):
        future=Future();future.set_result(function(*args));return future


class HandoffTests(unittest.TestCase):
    def test_startup_waits_only_for_current_scene_not_next_prefetch(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'manifest.json'
            path.write_text(json.dumps(dict(mount={},chunks=[{},{}])))
            ready=Future();ready.set_result('current')
            pending=Future()
            active=dict(index=0,worker=Worker(True),visual=ready,logged=True)
            upcoming=dict(index=1,worker=Worker(False),visual=pending,logged=False)
            with patch.object(StreamingCamera,'_launch',side_effect=[active,upcoming]):
                camera=StreamingCamera(folder,path,{},'camera',output=Path(folder)/'logs')
            self.assertIs(camera.pending,upcoming)
            self.assertFalse(pending.done())
            camera.close()

    def camera(self):
        camera=StreamingCamera.__new__(StreamingCamera)
        camera.sections=[dict(core_station_m=[0,.3],valid_station_m=[-2,2.3],camera_y_m=[-4,4.3]),
                         dict(core_station_m=[.3,.6],valid_station_m=[-1.7,2.6],camera_y_m=[-3.7,4.6])]
        camera.active_index=0;camera.events=[];camera.retired=[];camera.pool=ImmediatePool();camera.viewer=None
        camera.mount={'body_from_camera_optical':np.eye(4).tolist()}
        visual=Future();visual.set_result('first')
        camera.active=dict(index=0,worker=Worker(True),visual=visual,logged=True)
        next_visual=Future()
        camera.pending=dict(index=1,worker=Worker(False),visual=next_visual,logged=False)
        return camera

    def test_late_preload_keeps_motion_in_overlap_without_waiting(self):
        camera=self.camera();poses={'camera':np.array([0,.5,0,0,0,0,1])}
        start=time.perf_counter()
        for _ in range(200):self.assertEqual(camera.advance(.5,poses,'camera'),0)
        self.assertLess(time.perf_counter()-start,1.)
        self.assertFalse(camera.pending['visual'].done())
        self.assertEqual(camera.events,[])

    def test_gpu_ready_but_file_not_ready_does_not_call_result(self):
        camera=self.camera();camera.pending['worker'].ready=True
        self.assertEqual(camera.advance(.4,{'camera':[0,0,0,0,0,0,1]},'camera'),0)

    def test_ready_switch_keeps_pose_and_records_zero_load_wait(self):
        camera=self.camera();camera.pending['worker'].ready=True;camera.pending['visual'].set_result('second')
        old=camera.active['worker'];pose=np.array([0,.5,0,0,0,0,1]);saved=pose.copy()
        self.assertEqual(camera.advance(.5,{'camera':pose},'camera'),1)
        np.testing.assert_array_equal(pose,saved)
        self.assertTrue(old.closed)
        self.assertEqual(camera.events[-1]['event'],'switch')
        self.assertEqual(camera.events[-1]['load_wait_s'],0.)

    def test_no_stale_sensor_beyond_coverage(self):
        camera=self.camera()
        with self.assertRaisesRegex(RuntimeError,'deadline missed'):
            camera.advance(2.4,{'camera':[0,2.4,0,0,0,0,1]},'camera')
        self.assertEqual(camera.events[-1]['event'],'coverage_exhausted')

    def test_actual_camera_excursion_guard(self):
        camera=self.camera()
        with self.assertRaisesRegex(RuntimeError,'optical coverage'):
            camera.advance(.1,{'camera':[0,99,0,0,0,0,1]},'camera')

    def test_identical_payload_reuses_gpu_without_reloading_or_closing(self):
        camera=self.camera();camera.pending=None
        for section in camera.sections:section.update(geometry_sha256='same',catalog_sha256='same')
        original=camera.active['worker']
        camera._request_next({},'camera')
        self.assertIs(camera.pending['worker'],original)
        self.assertEqual(camera.advance(.5,{'camera':[0,.5,0,0,0,0,1]},'camera'),1)
        self.assertFalse(original.closed)
        self.assertTrue(camera.events[-1]['reused_gpu_scene'])

    def test_subtree_cleanup_and_worker_join_are_deferred(self):
        camera=self.camera();camera.pending['worker'].ready=True;camera.pending['visual'].set_result('second')
        tasks=[];removed=[]
        class Pool:
            def submit(self,fn):tasks.append(fn);return Future()
        class OldFarm:
            def remove(self):removed.append(True)
        class Viewer:
            def activate_farm(self,index):return OldFarm()
        camera.pool=Pool();camera.viewer=Viewer();old=camera.active['worker']
        camera.advance(.5,{'camera':[0,.5,0,0,0,0,1]},'camera')
        self.assertFalse(removed);self.assertFalse(old.closed)
        tasks[0]();self.assertTrue(removed);self.assertTrue(old.closed)


if __name__=='__main__':unittest.main()
