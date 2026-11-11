"""Bounded render prefetch; no load wait, CUDA initialization or joins at handoff.

Only exposure RPC waits for the ACTIVE sensor. A spawned next worker loads,
uploads and warms the next sensor while the existing physics process runs.
Measured poses and global harvest identities are never reset on a scene switch.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
import multiprocessing as mp
from pathlib import Path
import time
import traceback

import numpy as np

from newton_scene_chunks import PackedScene


def _camera_worker(connection,assets,manifest_path,index,scale,poses,body,log):
    import contextlib
    with open(log,'w') as output,contextlib.redirect_stdout(output),contextlib.redirect_stderr(output):
        try:
            started=time.perf_counter()
            from newton_mounted_camera import MountedCamera
            manifest=json.loads(Path(manifest_path).read_text());section=manifest['chunks'][index]
            camera=MountedCamera(assets,scale=scale,packed=(Path(manifest_path).parent,section,manifest))
            # Trigger ray tracer kernels, BVH and image allocation before READY.
            camera.capture(poses,body,0.)
            connection.send(dict(kind='ready',setup_wall_s=camera.setup_wall_s,ready_wall_s=time.perf_counter()-started))
            while True:
                message=connection.recv()
                if message['kind']=='close':break
                observation,images=camera.capture(message['poses'],message['body'],message['timestamp'],
                                                  output=message['output'],include_raw=message.get('include_raw',False))
                connection.send(dict(kind='frame',observation=observation,images=images))
        except (EOFError,BrokenPipeError):pass
        except BaseException:
            try:connection.send(dict(kind='error',error=traceback.format_exc()))
            except (EOFError,BrokenPipeError):pass
        finally:connection.close()


class CameraWorker:
    def __init__(self,assets,manifest,index,scale,poses,body,log):
        ctx=mp.get_context('spawn');self.pipe,child=ctx.Pipe()
        self.process=ctx.Process(target=_camera_worker,args=(child,str(assets),str(manifest),index,scale,poses,body,str(log)),daemon=True)
        self.started=time.perf_counter();self.ready=False;self.ready_wall_s=None
        self.process.start();child.close()

    def poll_ready(self):
        if self.ready:return True
        if self.pipe.poll():
            try:message=self.pipe.recv()
            except EOFError:raise RuntimeError('Camera prefetch worker exited before READY')
            if message['kind']!='ready':raise RuntimeError('Camera prefetch failed: '+str(message))
            self.ready=True;self.ready_wall_s=message['ready_wall_s']
        elif not self.process.is_alive():raise RuntimeError('Camera prefetch worker died')
        return self.ready

    def capture(self,poses,body,timestamp,output,include_raw=False):
        if not self.ready:raise RuntimeError('Attempted exposure on an unprepared camera')
        self.pipe.send(dict(kind='capture',poses=poses,body=body,timestamp=timestamp,
                            output=None if output is None else str(output),include_raw=include_raw))
        if not self.pipe.poll(30.):raise RuntimeError('Active camera exposure timed out; see worker log')
        message=self.pipe.recv()
        if message['kind']!='frame':raise RuntimeError('Active camera failed: '+str(message))
        return message['observation'],message['images']

    def close(self):
        try:self.pipe.send(dict(kind='close'))
        except (OSError,EOFError,BrokenPipeError):pass
        self.process.join(timeout=2.)
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=2.)
        self.pipe.close()


class StreamingCamera:
    def __init__(self,assets,manifest_path,poses,body,*,scale=.5,output,viewer=None,worker_factory=CameraWorker,reuse_identical=True):
        started=time.perf_counter();self.assets=Path(assets);self.path=Path(manifest_path)
        self.manifest=json.loads(self.path.read_text());self.sections=self.manifest['chunks']
        if not self.sections:raise ValueError('Empty scene stream')
        self.mount=self.manifest['mount'];self.scale=scale;self.viewer=viewer
        self.reuse_identical=reuse_identical
        self.output=Path(output);self.output.mkdir(exist_ok=True)
        self.factory=worker_factory;self.pool=ThreadPoolExecutor(max_workers=2,thread_name_prefix='scene-prefetch')
        self.events=[];self.active_index=0;self.pending=None;self.active=None;self.closed=False
        self.retired=[];self.max_resident_workers=0
        try:
            self.active=self._launch(0,poses,body)
            # Startup is the only preparation barrier. No route is executing.
            self._startup_ready(self.active)
            self.visual=self.active['visual'].result()
            self._request_next(poses,body)
            # The NEXT section is not a startup barrier either. Its overlap
            # supplies time to finish while the current section runs.
        except BaseException:
            self.close();raise
        self.setup_wall_s=time.perf_counter()-started

    def _launch(self,index,poses,body):
        section=self.sections[index]
        worker=self.factory(self.assets,self.path,index,self.scale,poses,body,self.output/f'camera_{index:04d}.log')
        def load_visual():
            visual=PackedScene(self.path.parent/section['geometry'],section['geometry_sha256'])
            if self.viewer and index!=self.active_index:self.viewer.stage_farm(index,visual)
            return visual
        future=self.pool.submit(load_visual)
        self.events.append(dict(event='prefetch_start',index=index,wall_s=time.perf_counter()))
        resident=1+int(self.active is not None)+sum(not f.done() for f in self.retired)
        self.max_resident_workers=max(self.max_resident_workers,resident)
        return dict(index=index,worker=worker,visual=future,logged=False)

    def _ready(self,item):
        worker_ready=item['worker'].poll_ready()
        if not worker_ready or not item['visual'].done():return False
        item['visual'].result() # done() checked: this cannot wait for loading
        if not item['logged']:
            self.events.append(dict(event='prefetch_ready',index=item['index'],wall_s=time.perf_counter(),
                                    load_wall_s=item['worker'].ready_wall_s))
            item['logged']=True
        return True

    def _startup_ready(self,item):
        deadline=time.perf_counter()+120.
        while not self._ready(item):
            if time.perf_counter()>deadline:raise RuntimeError('Initial camera preparation timed out')
            time.sleep(.01)

    def _request_next(self,poses,body):
        index=self.active_index+1
        # Do not accumulate retiring GPU contexts if a user teleports forward.
        self.retired=[f for f in self.retired if not f.done()]
        if index<len(self.sections) and self.pending is None and not self.retired:
            current=self.sections[self.active_index];following=self.sections[index]
            identical=getattr(self,'reuse_identical',True) and all(current.get(k) and current[k]==following.get(k)
                          for k in ('geometry_sha256','catalog_sha256'))
            if identical:
                # A short farm can fit entirely in both windows. Never reload
                # the identical GPU scene just because a station index changed.
                self.pending=dict(self.active,index=index,reuse=True,logged=True)
                self.events.append(dict(event='reuse_ready_scene',index=index,wall_s=time.perf_counter()))
            else:self.pending=self._launch(index,poses,body)

    def advance(self,station,poses,body):
        started=time.perf_counter()
        if not np.isfinite(station):raise ValueError('Finite measured station required')
        self._request_next(poses,body)
        section=self.sections[self.active_index]
        ready=self.pending is not None and self._ready(self.pending)
        if station>=section['core_station_m'][1] and ready:
            previous=self.active;old=self.active_index
            self.active=self.pending;self.pending=None;self.active_index=self.active['index']
            reused=self.active.get('reuse',False)
            old_farm=self.viewer.activate_farm(self.active_index) if self.viewer and not reused else None
            self.visual=self.active['visual'].result() # already ready above
            def retire():
                try:
                    if old_farm is not None:old_farm.remove()
                finally:previous['worker'].close()
            if not reused:self.retired.append(self.pool.submit(retire))
            self.events.append(dict(event='switch',previous=old,index=self.active_index,station_m=station,
                wall_s=time.perf_counter(),switch_wall_s=time.perf_counter()-started,load_wait_s=0.,reused_gpu_scene=reused))
            section=self.sections[self.active_index]
        if not section['valid_station_m'][0]<=station<=section['valid_station_m'][1]:
            self.events.append(dict(event='coverage_exhausted',station_m=station,index=self.active_index))
            raise RuntimeError('Scene prefetch deadline missed: no prepared camera covers measured station')
        # Guard actual optical center; no reliance on nominal ready posture.
        from newton_mounted_camera import camera_world
        y=float(camera_world(poses[body],self.mount['body_from_camera_optical'])[1,3])
        if not section['camera_y_m'][0]<=y<=section['camera_y_m'][1]:
            raise RuntimeError('Measured camera is outside the prepared optical coverage')
        return self.active_index

    def capture(self,poses,body,timestamp,*,output=None,station=None,include_raw=False):
        if station is None:raise ValueError('Scene stream requires measured rail station')
        index=self.advance(float(station),poses,body)
        started=time.perf_counter()
        observation,images=self.active['worker'].capture(poses,body,timestamp,output,include_raw=include_raw)
        observation['stream_section']=index
        observation['camera_rpc_wall_s']=time.perf_counter()-started
        if output is not None:
            (Path(output)/'observation.json').write_text(json.dumps(observation,indent=2,allow_nan=False)+'\n')
        return observation,images

    def evidence(self):
        from newton_inputs import sha256
        return dict(schema='s300-visual-stream-evidence-v1',events=self.events,
            manifest=str(self.path),manifest_sha256=sha256(self.path),source_hashes=self.manifest['source_hashes'],
            switches=sum(e['event']=='switch' for e in self.events),
            runtime_load_wait_s=sum(e.get('load_wait_s',0.) for e in self.events),
            deadline_misses=sum(e['event']=='coverage_exhausted' for e in self.events),
            max_resident_workers=self.max_resident_workers,
            scope=self.manifest['scope'],initial_preparation_wall_s=self.setup_wall_s)

    def close(self):
        if self.closed:return
        self.closed=True
        seen=set()
        for item in (self.active,self.pending):
            if item and id(item['worker']) not in seen:
                item['worker'].close();seen.add(id(item['worker']))
        self.pool.shutdown(wait=True,cancel_futures=True)
