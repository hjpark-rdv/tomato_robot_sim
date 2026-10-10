#!/usr/bin/env python3
"""Open the assembled USD in Isaac Sim, optionally run a short rigid-body check."""
import argparse
import json
import time
import traceback
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('scene',type=Path)
p.add_argument('--headless',action='store_true')
p.add_argument('--seconds',type=float,default=0,help='0: interactive inspection; >0: bounded physics smoke')
p.add_argument('--output',type=Path,required=True)
p.add_argument('--camera',default='/World/RobotCamera')
p.add_argument('--viewer-fps',type=int,default=30,
               help='idle GUI frame cap, 0 keeps original pacing; ignored for headless/physics smoke')
a=p.parse_args()
if not 0<=a.viewer_fps<=240:p.error('--viewer-fps must be between 0 and 240')
if not a.scene.is_file():raise FileNotFoundError(a.scene)
a.output.mkdir(parents=True,exist_ok=True)
if (a.output/'runtime.json').exists():raise FileExistsError(a.output/'runtime.json')
started=time.perf_counter()
exit_code=0
print('[HARVEST] starting Isaac Sim',flush=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':a.headless,'width':1280,'height':800,
                   'renderer':'RaytracedLighting','anti_aliasing':2})
try:
    import omni.usd
    import omni.kit.viewport.utility as vu
    import numpy as np
    from pxr import UsdGeom, UsdPhysics, PhysxSchema
    import carb
    carb.settings.get_settings().set_bool('/persistent/app/usd/muteUsdDiagnostics', False)
    print('[HARVEST] opening full scene:',a.scene,flush=True)
    if not omni.usd.get_context().open_stage(str(a.scene.resolve())):
        raise RuntimeError('USD stage did not open')
    stage=omni.usd.get_context().get_stage()
    for _ in range(60):app.update()
    viewport=vu.get_active_viewport()
    viewport.set_active_camera(a.camera)
    for _ in range(30):app.update()
    report={'scene':str(a.scene),'headless':a.headless,'startup_s':time.perf_counter()-started,
            'physics_requested_s':a.seconds,'scope':'assembly/rigid smoke, not harvest/bending validation'}
    # Renderer capture is the actual Isaac viewport, never an inferred illustration.
    for name in ('RobotCamera','FarmCamera','ToolCamera'):
        viewport.set_active_camera('/World/'+name)
        for _ in range(15):app.update()
        vu.capture_viewport_to_file(viewport,str(a.output/(name+'.png')))
        for _ in range(10):app.update()
    viewport.set_active_camera(a.camera)
    if a.seconds>0:
        from isaacsim.core.simulation_manager import SimulationManager as sim
        from omni.physx import get_physx_simulation_interface
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        events=[]
        def contacts(headers, data):
            for h in headers:
                for j in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    c=data[j]
                    events.append(dict(time=float(sim.get_simulation_time()),
                        collider0=str(h.collider0),collider1=str(h.collider1),
                        separation_m=float(c.separation),impulse_Ns=list(c.impulse)))
        contact_subscription=get_physx_simulation_interface().subscribe_contact_report_events(contacts)
        sim.set_device('cuda:0')
        sim.set_physics_dt(1/120)
        print('[HARVEST] loading full collision geometry into GPU PhysX',flush=True)
        physics_load_started=time.perf_counter()
        sim.initialize_physics()
        report['physics_initialization_s']=time.perf_counter()-physics_load_started
        poses=[];t0=time.perf_counter()
        path='/World/HarvestRobot/Robot/link6'
        view=sim.get_physics_sim_view().create_rigid_body_view(path)
        if view.count!=1:raise RuntimeError(f'Expected one physical link6, got {view.count}')
        for i in range(round(a.seconds*120)):
            sim.step(update_fabric=True)
            raw=view.get_transforms()
            if hasattr(raw,'detach'): raw=raw.detach().cpu().numpy()
            elif hasattr(raw,'numpy'): raw=raw.numpy()
            pose=np.asarray(raw)[0,:3].tolist()
            poses.append([float(sim.get_simulation_time()),*pose])
            if not np.isfinite(pose).all():raise RuntimeError('Nonfinite link6 state')
            if i%120==0:print(f'[HARVEST] physics {i/120:.1f}/{a.seconds:.1f}s',flush=True)
        report['physics_wall_s']=time.perf_counter()-t0
        report['link6_max_displacement_m']=float(np.max(np.linalg.norm(np.array(poses)[:,1:]-np.array(poses)[0,1:],axis=1)))
        np.save(a.output/'link6_pose.npy',poses)
        report['contact_points']=len(events)
        report['minimum_contact_separation_m']=min((e['separation_m'] for e in events),default=None)
        (a.output/'contacts.json').write_text(json.dumps(events)+'\n')
        report['physics_steps']=len(poses)
        for _ in range(10):app.update()
        vu.capture_viewport_to_file(viewport,str(a.output/'after_physics.png'))
        for _ in range(10):app.update()
    report['elapsed_s']=time.perf_counter()-started
    (a.output/'runtime.json').write_text(json.dumps(report,indent=2)+'\n')
    print('[HARVEST]',json.dumps(report),flush=True)
    if not a.headless:
        from viewer_pacing import ViewerPacing
        viewer_pacing=ViewerPacing(a.viewer_fps)
        (a.output/'viewer_pacing.json').write_text(json.dumps({
            'idle_fps':a.viewer_fps,'original_loop_settings':viewer_pacing.original,
            'physics_playback':'original run-loop settings restored on Play',
            'render_quality_and_geometry':'unchanged'},indent=2)+'\n')
        print(f'[HARVEST] idle viewer cap {a.viewer_fps} FPS; original pacing during Play',flush=True)
        print('[HARVEST] scene ready; close the window to exit',flush=True)
        while app.is_running():app.update()
except Exception:
    exit_code=1
    message=traceback.format_exc()
    print(message,flush=True)
    (a.output/'runtime_error.json').write_text(json.dumps({'error':message,'elapsed_s':time.perf_counter()-started},indent=2)+'\n')
finally:
    # 6.1 closes the process via os._exit; never let that swallow a Python failure.
    app.close(exit_code=exit_code)
