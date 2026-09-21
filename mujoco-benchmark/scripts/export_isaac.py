"""Read the unchanged full Isaac plant into a portable, audited reference bundle."""
import argparse
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nvidia-sim/rl'))
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser()
p.add_argument('--output',type=Path,default=ROOT/'mujoco-benchmark/assets/reference')
p.add_argument('--source-run',type=Path,default=ROOT/'nvidia-sim/rl/runs/20260921_213000_single_view_batched')
AppLauncher.add_app_launcher_args(p);p.set_defaults(device='cpu',headless=True)
args=p.parse_args();app=AppLauncher(args,limit_cpu_threads=8).app
import json, hashlib
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from assets import build_robot
from harvest_env import HarvestEnvCfg
from gpu_dataset_scene import GpuDatasetScene
from visual_colors import fallback_color

def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,Path):return str(x)
    raise TypeError(type(x))

def main():
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    cfg=HarvestEnvCfg();cfg.sim.device='cpu';cfg.sim.dt=1/120;cfg.decimation=2
    cfg.sim.physx.solver_type=0;cfg.sim.physx.enable_ccd=True
    cfg.plant_model='elastic';cfg.plant_resolution='full';cfg.main_appendage_collisions='keep'
    cfg.elastic_joint_armature=.0005;cfg.elastic_stiffness_scale=1.
    cfg.position_jitter=0.;cfg.break_randomization=0.;cfg.target_fruit='Tomato_05'
    cfg.stem_position=(-.75,.55,.32);cfg.stem_yaw=0.;cfg.stem_scale=.5
    cfg.lift_start_below=.4;cfg.lift_height_reference='mount';cfg.lift_speed=.1
    cfg.override_break_force=False;cfg.override_break_torque=False
    cfg.dataset_physics_sync='optimized'
    cfg.robot.spawn.usd_path=str(build_robot(out/'robot_export_only.usda'))
    import faulthandler
    faulthandler.dump_traceback_later(60,repeat=True)
    cfg.dataset_cpu_single=True
    world=GpuDatasetScene(cfg,1);slot=world.slots[0];el=slot.elastic;stage=slot.stage
    print('[원본 추출] 초기화 완료',flush=True)
    cache=UsdGeom.XformCache()
    def pose(path):
        tf=Gf.Transform(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)))
        q=tf.GetRotation().GetQuat().GetNormalized()
        return np.r_[tf.GetTranslation(),q.GetReal(),q.GetImaginary()]
    paths=el.paths+[s['anchor'] for s in slot.fruit_specs]+[s['path'] for s in slot.fruit_specs]
    art=el.articulation.root_physx_view
    names=el.articulation.body_names
    masses=art.get_masses()[0].numpy();inertias=art.get_inertias()[0].numpy();coms=art.get_coms()[0].numpy()
    bodies=[]
    for path in paths:
        if path in [s['path'] for s in slot.fruit_specs]:
            fruit=slot.fruits[[s['path'] for s in slot.fruit_specs].index(path)].root_physx_view
            mass=float(fruit.get_masses().reshape(-1)[0]);inertia=fruit.get_inertias().reshape(-1,9)[0].numpy();com=fruit.get_coms().reshape(-1,7)[0].numpy()
        else:
            k=names.index(path.rsplit('/',1)[-1]);mass=float(masses[k]);inertia=inertias[k];com=coms[k]
        bodies.append(dict(path=path,name=path.rsplit('/',1)[-1],pose=pose(path),mass=mass,inertia=inertia,com_pose_xyzw=com))
    by_path={b['path']:b for b in bodies}
    joints=[]
    for path in el.joints+slot.cluster_joint_paths:
        j=UsdPhysics.Joint(stage.GetPrimAtPath(path));child=str(j.GetBody1Rel().GetTargets()[0]);parents=j.GetBody0Rel().GetTargets()
        angles=[]
        for axis in ('rotX','rotY','rotZ'):
            d=UsdPhysics.DriveAPI(j.GetPrim(),axis);lim=UsdPhysics.LimitAPI(j.GetPrim(),axis)
            if d:
                angles.append(dict(axis=axis,k=d.GetStiffnessAttr().Get()*180/np.pi,d=d.GetDampingAttr().Get()*180/np.pi,
                    limit_deg=[lim.GetLowAttr().Get(),lim.GetHighAttr().Get()],max_force=d.GetMaxForceAttr().Get()))
        q=j.GetLocalRot1Attr().Get()
        joints.append(dict(path=path,parent=str(parents[0]) if parents else None,child=child,
            pos1=list(j.GetLocalPos1Attr().Get()),rot1=[q.GetReal(),*q.GetImaginary()],axes=angles,
            break_force=j.GetBreakForceAttr().Get(),break_torque=j.GetBreakTorqueAttr().Get(),
            exclude_articulation=j.GetExcludeFromArticulationAttr().Get()))
    tool=slot.root+'/Robot/link6/tcp/tomato_gripper'
    toolpose=pose(tool);by_path[tool]=dict(path=tool,pose=toolpose,name='Hook')
    shapes=[];visuals=[];filters=[]
    selected_prims=[prim for path in by_path for prim in Usd.PrimRange(stage.GetPrimAtPath(path))]
    print('[원본 추출] 대상 prim',len(selected_prims),flush=True)
    for prim in selected_prims:
        path=str(prim.GetPath())
        owner=next((b for b in by_path if path.startswith(b+'/')),None)
        if owner is None:continue
        enabled=prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        isvisual=prim.IsA(UsdGeom.Mesh) and not enabled and owner in [s['path'] for s in slot.fruit_specs] and 'Trichomes' not in path
        if not enabled and not isvisual:continue
        bp=by_path[owner]['pose'];rot=R.from_quat(bp[[4,5,6,3]])
        tf=cache.GetLocalToWorldTransform(prim);m=np.asarray(tf)
        local=lambda pts:rot.inv().apply(np.asarray(pts)@m[:3,:3]+m[3,:3]-bp[:3])
        rec=dict(path=path,body=owner,name='g'+str(len(shapes)) if enabled else 'v'+str(len(visuals)))
        if prim.IsA(UsdGeom.Capsule):
            cap=UsdGeom.Capsule(prim);v=np.zeros(3);v['XYZ'.index(cap.GetAxisAttr().Get())]=cap.GetHeightAttr().Get()/2
            rec.update(type='capsule',ends=local([-v,v]),radius=cap.GetRadiusAttr().Get()*np.linalg.norm(m[0,:3]))
        elif prim.IsA(UsdGeom.Sphere):
            rec.update(type='sphere',center=local([[0,0,0]])[0],radius=UsdGeom.Sphere(prim).GetRadiusAttr().Get()*np.linalg.norm(m[0,:3]))
        elif prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim);verts=local(mesh.GetPointsAttr().Get());counts=mesh.GetFaceVertexCountsAttr().Get();ids=mesh.GetFaceVertexIndicesAttr().Get();faces=[];offset=0
            for n in counts:
                faces.extend([[ids[offset],ids[offset+k],ids[offset+k+1]] for k in range(1,n-1)]);offset+=n
            rec.update(type='mesh',vertices=verts,faces=faces)
        else:raise ValueError('Unsupported collider '+path)
        color=UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        rec['color']=list(color[0]) if color else fallback_color(path)
        if enabled:
            api=PhysxSchema.PhysxCollisionAPI(prim)
            rec.update(contact_offset=api.GetContactOffsetAttr().Get() if api else None,rest_offset=api.GetRestOffsetAttr().Get() if api else None)
            shapes.append(rec)
        else:visuals.append(rec)
    print('[원본 추출] 형상 완료',len(shapes),len(visuals),flush=True)
    for prim in selected_prims:
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            filters.extend([[str(prim.GetPath()),str(t)] for t in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()])
    data=dict(schema=1,model='full',source_commit=__import__('subprocess').check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        bodies=bodies,joints=joints,shapes=shapes,visuals=visuals,filters=filters,tool_path=tool,tool_pose=toolpose,
        plant_model=el.model,fruit_specs=[{k:v for k,v in s.items() if k!='local_matrix'} for s in slot.fruit_specs],
        gravity=[0,0,-9.81],armature=cfg.elastic_joint_armature,friction=dict(static=.5,dynamic=.4,restitution=0.),
        native_joint_names=el.articulation.joint_names,preload_torque=(el.preload*el.articulation.data.joint_stiffness)[0].numpy(),
        robot_fk=dict(world=slot.pose_search_kin.world,chain=slot.pose_search_kin.chain,joint_names=slot.robot.joint_names),
        source_sha256={f:hashlib.sha256((ROOT/'nvidia-sim/rl'/f).read_bytes()).hexdigest() for f in ['assets.py','elastic_plant.py','greenhouse_env.py','geometry.py']})
    (out/'reference.json').write_text(json.dumps(data,default=plain,indent=2,allow_nan=True))
    # A standalone PhysX reference: remove robot, greenhouse and decorative static
    # visuals, retain every plant collision shape and native joint. No source edits.
    print('[원본 추출] JSON 완료',flush=True)
    flat=stage.Flatten();ref=Usd.Stage.Open(flat)
    keep=paths+el.joints+slot.cluster_joint_paths
    keep += [str(x.GetPath()) for x in stage.Traverse() if x.IsA(UsdPhysics.Scene) or x.HasAPI(UsdPhysics.MaterialAPI)]
    for prim in list(ref.Traverse()):
        path=str(prim.GetPath())
        if not any(path==k or path.startswith(k+'/') or k.startswith(path+'/') for k in keep):
            ref.RemovePrim(path)
    for path in slot.cluster_joint_paths:
        j=UsdPhysics.Joint.Get(ref,path);j.GetBreakForceAttr().Set(float('inf'));j.GetBreakTorqueAttr().Set(float('inf'))
    ref.Export(str(out/'plant_reference.usda'))
    # FK of *recorded actual joints*, not newly optimized poses. Never truncate to
    # pretend an aborted recording was a complete approach.
    trajectories=[]
    for folder in sorted((args.source_run/'results').glob('candidate_*')):
        if not (folder/'trace.json').exists():continue
        trace=json.loads((folder/'trace.json').read_text())
        if len(trace)<2:continue
        candidate=json.loads((folder/'candidate.json').read_text())
        dt=candidate['physical_inputs']['control_dt'];poses=[]
        for row in trace:
            pos,rot=slot.pose_search_kin.fk(row['joints']);poses.append([*pos,*rot.as_quat()])
        trajectories.append(dict(id=folder.name,source=str(folder),sample_dt=dt,frame='ring_center, world XYZ + xyzw',
            poses=poses,phases=[r['phase'] for r in trace],duration_s=(len(trace)-1)*dt,
            source_result=candidate.get('result'),scope='recorded prefix only; no simulated robot in either benchmark'))
    (out/'trajectories.json').write_text(json.dumps(trajectories,default=plain))
    print('[원본 추출 완료]',len(bodies),'강체',len(shapes),'충돌체',len(trajectories),'기록 경로',flush=True)

try:main()
except BaseException:
    import traceback,os
    traceback.print_exc();sys.stdout.flush();sys.stderr.flush();os._exit(1)
else:
    # This short export process owns the entire app; avoid Kit's stop callback
    # deadlock after large USD extraction. Files have already been closed.
    import os
    sys.stdout.flush();os._exit(0)
