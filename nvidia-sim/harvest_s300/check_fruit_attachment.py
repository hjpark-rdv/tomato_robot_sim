"""Native unit fixtures for passive neck rotation, recovery and native release.

Synthetic isolated bodies, no robot/harvest-success claim. Uses the production
joint authoring helper and the existing physics-only Isaac bootstrap.
"""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--hz',type=int,default=120)
p.add_argument('--solver',choices=['PGS','TGS'],default='TGS')
p.add_argument('--uncapped-fixture',action='store_true')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
from motion_app import create_motion_app
app=create_motion_app(SimulationApp,headless=True,width=640,height=480,performance='optimized',physics_only=True)
exit_code=0
try:
    import numpy as np
    import torch
    import omni.usd
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools
    from omni.physx import get_physx_interface,get_physx_simulation_interface
    from omni.physx.bindings._physx import SimulationEvent
    from isaacsim.core.simulation_manager import SimulationManager as sim
    from motion_app import configure_scene_device
    from fruit_attachment import configure_joint,relative_angles
    from scipy.spatial.transform import Rotation as R
    omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
    sim.switch_physics_engine('physx')
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene');scene.CreateGravityMagnitudeAttr(0.)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateSolverTypeAttr(a.solver)
    cases=[('idle','elastic',.008,0.,0),('recover','elastic',.008,.003,0),
           ('twist_break','elastic',.008,.012,0),('bend_break','elastic',.008,.012,1),
           ('fixed_strong','fixed',.08,.012,0),('fixed_weak','fixed',.008,.012,0)]
    for i,(name,kind,threshold,torque,axis) in enumerate(cases):
        anchor=UsdGeom.Xform.Define(stage,'/World/Anchor_'+name);anchor.AddTranslateOp().Set(Gf.Vec3d(0,i*.1,0))
        UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
        am=UsdPhysics.MassAPI.Apply(anchor.GetPrim());am.CreateMassAttr(.1);am.CreateDiagonalInertiaAttr(Gf.Vec3f(.001))
        weld=UsdPhysics.FixedJoint.Define(stage,'/World/Anchors/'+name);weld.CreateBody1Rel().SetTargets([anchor.GetPath()]);weld.CreateLocalPos0Attr(Gf.Vec3f(0,i*.1,0))
        body=UsdGeom.Xform.Define(stage,'/World/'+name);body.AddTranslateOp().Set(Gf.Vec3d(0,i*.1,0))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        m=UsdPhysics.MassAPI.Apply(body.GetPrim());m.CreateMassAttr(.005);m.CreateDiagonalInertiaAttr(Gf.Vec3f(2.42e-7))
        b=PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim());b.CreateSleepThresholdAttr(0.);b.CreateSolverPositionIterationCountAttr(64);b.CreateSolverVelocityIterationCountAttr(4)
        if a.uncapped_fixture:b.CreateMaxAngularVelocityAttr(1000000.)
        j=UsdPhysics.FixedJoint.Define(stage,'/World/Joints/'+name);j.CreateBody0Rel().SetTargets([anchor.GetPath()]);j.CreateBody1Rel().SetTargets([body.GetPath()])
        configure_joint(stage,str(j.GetPath()),kind=kind,force_n=3.,torque_nm=threshold)
    stage.GetRootLayer().Export(str(a.output/'fixture.usda'))
    omni.usd.get_context().open_stage(str((a.output/'fixture.usda').resolve()))
    for _ in range(3):app.update()
    stage=omni.usd.get_context().get_stage()
    sim.switch_physics_engine('physx')
    sim.set_default_physics_scene('/World/PhysicsScene');sim.set_device('cpu');sim.set_backend('torch');sim.set_physics_dt(1/a.hz)
    configure_scene_device(stage,'cpu');get_physx_interface().set_thread_count(2)
    import carb
    from omni.physx.bindings._physx import SETTING_UPDATE_TO_USD,SETTING_UPDATE_VELOCITIES_TO_USD
    carb.settings.get_settings().set_bool(SETTING_UPDATE_TO_USD,False)
    carb.settings.get_settings().set_bool(SETTING_UPDATE_VELOCITIES_TO_USD,False)
    sim.enable_fabric(True)
    events=[];current_t=0.
    def event(e):
        if e.type==int(SimulationEvent.JOINT_BREAK):events.append(dict(t=current_t,joint=str(PhysicsSchemaTools.decodeSdfPath(*e.payload['jointPath']))))
    subscription=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(event)
    sim.initialize_physics()
    assert sim.get_active_physics_engine()=='physx'
    print('ENGINE',sim.get_active_physics_engine(),'DT',sim.get_physics_dt(),flush=True)
    view=sim.get_physics_sim_view().create_rigid_body_view(['/World/'+c[0] for c in cases])
    print('MASSES',view.get_masses(),'INERTIAS',view.get_inertias(),flush=True)
    paths=list(view.prim_paths);ids=torch.arange(view.count,dtype=torch.int32);torques=torch.zeros((view.count,3));trace=[];times=[]
    for tick in range(4*a.hz):
        t=tick/a.hz;torques.zero_()
        for name,kind,threshold,torque,axis in cases:
            if 1<=t<2:torques[paths.index('/World/'+name),axis]=torque*min((t-1)/.25,1.)
        view.apply_forces_and_torques_at_position(None,torques,None,ids,True)
        current_t=(tick+1)/a.hz;sim.step(update_fabric=False);get_physx_simulation_interface().fetch_results()
        trace.append(view.get_transforms().numpy().copy());times.append(current_t)
    values=np.array(trace);times=np.array(times);rows=[]
    for name,kind,threshold,torque,axis in cases:
        q=values[:,paths.index('/World/'+name),3:];angles=R.from_quat(q).magnitude();broken=[e for e in events if e['joint'].endswith('/'+name)]
        before=times<=broken[0]['t'] if broken else np.ones(len(times),bool)
        row=dict(case=name,kind=kind,break_torque_nm=threshold,applied_torque_nm=torque,axis=axis,
            max_rotation_deg=float(np.rad2deg(angles[before]).max()),final_rotation_deg=float(np.rad2deg(angles[-1])),breaks=broken)
        rows.append(row)
    by={r['case']:r for r in rows}
    checks=dict(finite=bool(np.isfinite(values).all()),idle_stable=by['idle']['max_rotation_deg']<.01 and not by['idle']['breaks'],
        elastic_rotation=10<by['recover']['max_rotation_deg']<30 and not by['recover']['breaks'],
        recovery=by['recover']['final_rotation_deg']<2.,
        twist_native_break=bool(by['twist_break']['breaks']),bend_native_break=bool(by['bend_break']['breaks']),
        fixed_strong_retained=not by['fixed_strong']['breaks'],fixed_weak_break=bool(by['fixed_weak']['breaks']))
    np.savez_compressed(a.output/'states.npz',time=times,poses=values,body_paths=paths)
    report=dict(hz=a.hz,solver=a.solver,uncapped_synthetic_fixture=a.uncapped_fixture,checks=checks,passed=all(checks.values()),rows=rows,fixture_only=True,harvest_success=None)
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:exit_code=1
except Exception:
    import traceback
    traceback.print_exc();exit_code=1
finally:
    app.close()
raise SystemExit(exit_code)
