"""Standalone original plant + prescribed hook, same suite as MuJoCo, no arm."""
import argparse,json,sys,time,os,hashlib
from pathlib import Path
HOME=Path(__file__).resolve().parents[1];ROOT=HOME.parent
sys.path.insert(0,str(ROOT/'nvidia-sim/rl'))
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--suite',type=Path,default=HOME/'assets/reference/suite.json');p.add_argument('--hz',type=int,default=120);p.add_argument('--count',type=int,default=1);p.add_argument('--start',type=int,default=0);p.add_argument('--output',type=Path,required=True);p.add_argument('--solver',choices=['pgs','tgs'],default='pgs');p.add_argument('--recovery',type=float,default=2.);p.add_argument('--rgbd',action='store_true')
p.add_argument('--gui',action='store_true',help='식물과 고리를 화면에 표시하고 완료 후 창 유지')
p.add_argument('--trajectory',help='suite의 경로 ID 한 개 선택')
p.add_argument('--view-fps',type=float,default=30.)
p.add_argument('--exit-on-finish',action='store_true',help='GUI 시험 완료 후 자동 종료')
p.add_argument('--robot-trace',type=Path)
p.add_argument('--robot-control-dt',type=float,default=1/60)
p.add_argument('--robot-visuals',choices=['hidden','visible'],default='visible')
p.add_argument('--camera-scale',type=float,default=1.)
AppLauncher.add_app_launcher_args(p);p.set_defaults(device='cpu',headless=True);a=p.parse_args()
if a.view_fps<=0:p.error('--view-fps must be positive')
if a.gui:a.headless=False
a.enable_cameras=a.enable_cameras or a.rgbd or a.gui;app=AppLauncher(a,limit_cpu_threads=8).app
import numpy as np
import torch,psutil
from scipy.spatial.transform import Rotation as R
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,PhysicsSchemaTools
import omni.usd
import omni.physics.tensors as tensors
from omni.physx import get_physx_interface,get_physx_simulation_interface
from isaaclab.sim import SimulationContext,SimulationCfg,PhysxCfg
from assets import capsule,collision
sys.path.insert(0,str(HOME/'scripts'))
from suite import resample,RING
from metrics import audit
from visual_colors import shape_color

def main():
    a.output.mkdir(parents=True,exist_ok=False);ref=json.loads(a.reference.read_text());suite=json.loads(a.suite.read_text())
    trials=[t for t in suite if t['id']==a.trajectory] if a.trajectory else suite[a.start:a.start+a.count]
    if a.trajectory:a.count=1
    if len(trials)!=a.count:raise ValueError('Not enough distinct trajectories')
    (a.output/'inputs.json').write_text(json.dumps(dict(suite_sha256=hashlib.sha256(a.suite.read_bytes()).hexdigest(),reference_sha256=hashlib.sha256(a.reference.read_bytes()).hexdigest(),trials=[t['id'] for t in trials]),indent=2))
    stage_path=(a.reference.parent/'plant_reference.usda').resolve()
    if a.gui or a.rgbd:
        from pxr import UsdLux
        # The viewport inspects lighting at the stage-open event. Author the
        # light before opening, rather than adding it after sim.reset().
        view_path=(a.output/'display_stage.usda').resolve()
        display_stage=Usd.Stage.CreateNew(str(view_path))
        display_stage.GetRootLayer().subLayerPaths=[str(stage_path)]
        UsdLux.DomeLight.Define(display_stage,'/World/BenchmarkLight').CreateIntensityAttr(1000.)
        display_stage.GetRootLayer().Save()
        stage_path=view_path
        print('[Isaac 조명] 장면을 열기 전에 관찰용 조명을 준비했습니다.',flush=True)
    omni.usd.get_context().open_stage(str(stage_path))
    stage=omni.usd.get_context().get_stage();scene_path=next(str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    scene=UsdPhysics.Scene.Get(stage,scene_path);scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    pc=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());pc.CreateSolverTypeAttr('PGS' if a.solver=='pgs' else 'TGS')
    pc.CreateEnableGPUDynamicsAttr(False);pc.CreateBroadphaseTypeAttr('MBP');pc.CreateEnableCCDAttr(True)
    robot=None
    if a.robot_trace:
        from robot_replay import RobotReplay
        robot=RobotReplay(stage,ref,a.reference.parent/'robot_export_only.usda',a.robot_trace,a.robot_control_dt,a.robot_visuals=='visible')
    hookpath='/World/BenchmarkHook';hook=UsdGeom.Xform.Define(stage,hookpath);hook.AddTranslateOp().Set(Gf.Vec3d(0,0,10));hook.AddOrientOp().Set(Gf.Quatf(1))
    api=UsdPhysics.RigidBodyAPI.Apply(hook.GetPrim());api.CreateKinematicEnabledAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(hook.GetPrim()).CreateThresholdAttr(0.)
    mapping={s['path']:s['name'] for s in ref['shapes'] if s['body']!=ref['tool_path']}
    for s in ref['shapes']:
        if s['body']!=ref['tool_path']:continue
        path=hookpath+'/'+s['name'];mapping[path]=s['name']
        if s['type']=='capsule':capsule(stage,path,*s['ends'],s['radius'])
        elif s['type']=='mesh':
            mesh=UsdGeom.Mesh.Define(stage,path);mesh.CreatePointsAttr(s['vertices']);mesh.CreateFaceVertexCountsAttr([3]*len(s['faces']));mesh.CreateFaceVertexIndicesAttr(np.array(s['faces']).reshape(-1).tolist());collision(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        if robot:
            UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr(False)
            mapping[s['path']]=s['name']
    if robot:
        hook.GetPrim().RemoveAPI(UsdPhysics.RigidBodyAPI)
        hook.GetPrim().RemoveAPI(PhysxSchema.PhysxContactReportAPI)
    contact_prefix=robot.tool_path if robot else hookpath
    # Explicit native iterations and no per-step USD writeback. Physics geometry,
    # drives, filter relationships and standalone attachment constraints retained.
    cfg=SimulationCfg(dt=1/a.hz,device='cpu',physics_prim_path=scene_path,use_fabric=False,
        physx=PhysxCfg(solver_type=0 if a.solver=='pgs' else 1,enable_ccd=True,min_position_iteration_count=64,max_position_iteration_count=64,min_velocity_iteration_count=4,max_velocity_iteration_count=4))
    began=time.perf_counter();sim=SimulationContext(cfg)
    sim.carb_settings.set_bool('/physics/disableContactProcessing',False)
    sim.reset()
    sim.carb_settings.set_bool('/physics/updateToUsd',False);sim.carb_settings.set_bool('/physics/updateVelocitiesToUsd',False)
    view=tensors.create_simulation_view('numpy');view.set_subspace_roots('/')
    if robot:robot.bind(view)
    art=view.create_articulation_view('/World/envs/env_0/ElasticPlant');meta=art.get_metatype(0)
    expected=ref['native_joint_names'];actual=list(meta.dof_names)
    preload=np.array([[ref['preload_torque'][expected.index(n)] for n in actual]],np.float32)
    idx=np.array([0],np.int32);art.set_dof_armatures(np.full_like(preload,ref['armature']),idx)
    plant_paths=[b['path'] for b in ref['bodies']];art_indices=[list(meta.link_names).index(p.rsplit('/',1)[-1]) for p in plant_paths[:79]]
    fruits=view.create_rigid_body_view([s['path'] for s in ref['fruit_specs']]);fruit_order={p:i for i,p in enumerate(fruits.prim_paths)}
    fruit_indices=[fruit_order[s['path']] for s in ref['fruit_specs']];fids=np.arange(11,dtype=np.int32)
    body_lookup={b['path']:b for b in ref['bodies']}
    fposes=np.array([np.asarray(body_lookup[path]['pose'])[[0,1,2,4,5,6,3]] for path in fruits.prim_paths],np.float32)
    root_name=list(meta.link_names)[0]
    root_rest=next(b['pose'] for b in ref['bodies'] if b['name']==root_name)
    rootpose=np.asarray(root_rest,dtype=np.float32)[[0,1,2,4,5,6,3]][None]
    hv=robot if robot else view.create_rigid_body_view(hookpath);pairs=[]
    def oncontact(headers,data):
        for h in headers:
            x,y=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            if (x.startswith(contact_prefix+'/') or y.startswith(contact_prefix+'/')) and h.num_contact_data and x in mapping and y in mapping:pairs.append([mapping[x],mapping[y]])
    sub=get_physx_simulation_interface().subscribe_contact_report_events(oncontact)
    def reset():
        art.set_root_transforms(rootpose,idx);art.set_root_velocities(np.zeros((1,6),np.float32),idx)
        art.set_dof_positions(np.zeros_like(preload),idx);art.set_dof_velocities(np.zeros_like(preload),idx)
        art.set_dof_position_targets(np.zeros_like(preload),idx);art.set_dof_velocity_targets(np.zeros_like(preload),idx)
        fruits.set_transforms(fposes,fids);fruits.set_velocities(np.zeros((11,6),np.float32),fids)
        if robot:robot.reset()
        view.update_articulations_kinematic()
    def sethook(pose,initial=False):
        if robot:return
        rot=R.from_quat(pose[3:]);tf=np.array([[*(pose[:3]-rot.apply(RING)),*pose[3:]]],np.float32)
        (hv.set_transforms if initial else hv.set_kinematic_targets)(tf,idx)
    def poses():return np.concatenate([art.get_link_transforms()[0,art_indices],fruits.get_transforms()[fruit_indices],hv.get_transforms()]).copy()
    rgb=depth=product=None
    if a.rgbd or a.gui:
        from pxr import UsdLux,UsdShade
        from PIL import Image
        target=next(s for s in ref['fruit_specs'] if s['name']=='Tomato_05');tp=np.array(target['pose']);centre=tp[:3]+R.from_quat(tp[[4,5,6,3]]).apply(target['center']);eye=centre+np.array([.13,.38,.16])*a.camera_scale
        camera=UsdGeom.Camera.Define(stage,'/World/BenchmarkCamera');camera.CreateFocalLengthAttr(20.);va=40*np.tan(np.deg2rad(24));camera.CreateVerticalApertureAttr(float(va));camera.CreateHorizontalApertureAttr(float(va*640/480));camera.CreateClippingRangeAttr(Gf.Vec2f(.005,5.))
        UsdGeom.Xformable(camera).MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*centre),Gf.Vec3d(0,0,1)).GetInverse())
        for prim in stage.Traverse():
            if robot and str(prim.GetPath()).startswith(robot.root+'/'):continue
            if prim.HasAPI(UsdPhysics.CollisionAPI):UsdGeom.Imageable(prim).MakeVisible()
        from pxr import Sdf
        materials={}
        for shape in ref['shapes']+ref['visuals']:
            path=hookpath+'/'+shape['name'] if shape['body']==ref['tool_path'] else shape['path'];prim=stage.GetPrimAtPath(path)
            if not prim:continue
            color=tuple([.85,.85,.85] if shape['body']==ref['tool_path'] else shape_color(shape))
            if color not in materials:
                material=UsdShade.Material.Define(stage,f'/World/BenchmarkMaterials/m{len(materials)}');shader=UsdShade.Shader.Define(stage,str(material.GetPath())+'/Shader');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');materials[color]=material
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[color])
        # A tensor-driven kinematic target is not always written back to USD.
        # Mirror only the hook for rendering; never author physics transforms.
        display_path='/World/BenchmarkHookDisplay'
        Sdf.CopySpec(stage.GetRootLayer(),hookpath,stage.GetRootLayer(),display_path)
        display=stage.GetPrimAtPath(display_path)
        for prim in Usd.PrimRange(display):
            prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit([]))
            for prop in list(prim.GetProperties()):
                if prop.GetName().startswith(('physics:','physx')):prim.RemoveProperty(prop.GetName())
        display_op=UsdGeom.Xformable(display).MakeMatrixXform()
        UsdGeom.Imageable(hook.GetPrim()).MakeInvisible()
        def sync_render():
            get_physx_interface().update_transformations(False,True,False)
            transform=hv.get_transforms()[0];mat=Gf.Matrix4d().SetRotate(Gf.Quatd(float(transform[6]),Gf.Vec3d(*map(float,transform[3:6]))));mat.SetTranslateOnly(Gf.Vec3d(*map(float,transform[:3])));display_op.Set(mat)
        if a.rgbd:
            import omni.replicator.core as rep
            (a.output/'rgbd_camera.json').write_text(json.dumps(dict(resolution=[640,480],rgbd_hz=15,K=[[240/np.tan(np.deg2rad(24)),0,320],[0,240/np.tan(np.deg2rad(24)),240],[0,0,1]],clipping_m=[.005,5.],depth_definition='metres along camera optical axis',hook_display='native pose visual mirror; no physics schemas'),indent=2))
            product=rep.create.render_product(str(camera.GetPath()),(640,480));rgb=rep.AnnotatorRegistry.get_annotator('rgb');depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');rgb.attach(product);depth.attach(product)
        if a.gui:
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            if viewport is None:raise RuntimeError('GUI viewport unavailable')
            viewport.camera_path=str(camera.GetPath());viewport.set_texture_resolution((960,720))
            print('[Isaac 화면] 토마토 관찰 카메라 설정 완료. 동작 종료 후 창을 닫으면 종료됩니다.',flush=True)
        for _ in range(8):sim.render()
    # Apply pending USD notices before native reset/targets in *all* modes.
    # Otherwise a kinematic hook can snap back to its spawn pose on step one.
    get_physx_simulation_interface().flush_changes()
    initialization=time.perf_counter()-began;began=time.perf_counter()
    for _ in range(100):reset()
    reset_s=time.perf_counter()-began;results=[];batch_start=time.perf_counter()
    for trial in trials:
        times,hooks=resample(trial,a.hz,a.recovery);reset();sethook(hooks[0],True);view.update_articulations_kinematic();states=[poses()];robot_q=[robot.joint_positions()] if robot else []
        error=float(np.max(np.abs(states[0][:-1,:3]-np.array([b['pose'][:3] for b in ref['bodies']]))))
        if error>1e-6:raise RuntimeError(f'Initial state differs from reference by {error} m')
        if a.rgbd or a.gui:
            before=poses();sync_render()
            for _ in range(3):sim.render()
            if np.max(abs(poses()-before))>1e-6:raise RuntimeError('Render warmup advanced physics')
            if a.rgbd:
                Image.fromarray(rgb.get_data()[:,:,:3]).save(a.output/'rgb.png');np.save(a.output/'depth_m.npy',depth.get_data())
        print('[Isaac 실행 시작]',trial['id'],a.hz,'Hz',flush=True)
        events=[];physics=0.;cpu=time.process_time();began=time.perf_counter();unstable=False;render_s=0.;frames=0
        for k,pose in enumerate(hooks[1:]):
            if a.gui and not app.is_running():return
            pairs.clear()
            if robot:robot.command(times[k+1])
            else:sethook(pose)
            art.set_dof_actuation_forces(preload,idx)
            t=time.perf_counter();sim.step(render=False);physics+=time.perf_counter()-t
            states.append(poses());events.append(pairs.copy())
            if robot:robot_q.append(robot.joint_positions())
            render_due=(a.gui and k%max(1,round(a.hz/a.view_fps))==0) or (a.rgbd and k%max(1,round(a.hz/15))==0)
            if render_due:
                t=time.perf_counter();sync_render();sim.render()
                if a.gui and not app.is_running():return
                if a.rgbd and k%max(1,round(a.hz/15))==0:image=rgb.get_data();z=depth.get_data()
                render_s+=time.perf_counter()-t;frames+=1
            if not np.isfinite(states[-1]).all() or np.max(abs(art.get_dof_velocities()))>1e4:unstable=True;break
        elapsed=time.perf_counter()-began;cpu=time.process_time()-cpu;times=times[:len(states)];states=np.array(states);sim_s=float(times[-1]);start=time.perf_counter();metrics=audit(ref,states,times,events);audit_s=time.perf_counter()-start
        row=dict(engine='Isaac Sim',version='5.1',mode='physics_plus_gui_rgbd' if a.gui and a.rgbd else 'physics_plus_viewer' if a.gui else 'physics_plus_rgbd' if a.rgbd else 'physics_only',rgbd_frames=sum(k%max(1,round(a.hz/15))==0 for k in range(len(states)-1)) if a.rgbd else 0,rgbd_render_wall_s=render_s if a.rgbd else 0.,render_frames=frames,render_wall_s=render_s,trajectory_id=trial['id'],category=trial.get('category','recorded'),hz=a.hz,solver=a.solver,iterations=64,
            requested_sim_s=float(resample(trial,a.hz,a.recovery)[0][-1]),simulated_s=sim_s,wall_s=elapsed,physics_only_wall_s=physics,
            rtf=sim_s/elapsed,physics_only_rtf=sim_s/physics,steps_per_second=(len(states)-1)/physics,process_cpu_percent=cpu/elapsed*100,
            rss_mib=psutil.Process().memory_info().rss/2**20,audit_wall_s=audit_s,unstable=unstable,break_enabled=False,robot_present=robot is not None,robot_visuals=a.robot_visuals if robot else None,camera_scale=a.camera_scale,**metrics)
        if unstable:row['result']='unstable'
        actual_hook=np.column_stack([states[:,-1,:3]+R.from_quat(states[:,-1,3:]).apply(RING),states[:,-1,3:]])
        folder=a.output/trial['id'];folder.mkdir();np.savez_compressed(folder/'states.npz',times_s=times,poses=states,hook=hooks[:len(states)],actual_hook=actual_hook,robot_joints=robot_q);(folder/'result.json').write_text(json.dumps(row,indent=2));(folder/'contacts.json').write_text(json.dumps(events));(folder/'trajectory.json').write_text(json.dumps(trial));results.append(row)
        (a.output/'summary.json').write_text(json.dumps(dict(engine='Isaac Sim',count=len(results),requested_count=a.count,hz=a.hz,initialization_s=initialization,reset_100_s=reset_s,reset_mean_s=reset_s/100,total_wall_s=time.perf_counter()-batch_start,results=results),indent=2))
        print('[Isaac 비교 결과]',len(results),'/',a.count,trial['id'],row['result'],'RTF',round(row['rtf'],2),flush=True)
        if trial['id']=='fixture_fruit_collision' and row['contact_steps']==0:
            raise RuntimeError('Fruit contact fixture produced no native reports; do not trust subsequent contact labels')
    if a.gui:
        from omni.kit.viewport.utility import capture_viewport_to_file
        import asyncio
        before=poses()
        sync_render()
        for _ in range(3):sim.render()
        capture=capture_viewport_to_file(viewport,str(a.output/'gui_final.png'))
        pending=asyncio.ensure_future(capture.wait_for_result());deadline=time.perf_counter()+15.
        while app.is_running() and not pending.done() and time.perf_counter()<deadline:sim.render()
        if not app.is_running():return
        if not pending.done():raise RuntimeError('GUI screenshot timed out')
        pending.result()
        if np.max(abs(poses()-before))>1e-6:raise RuntimeError('Final display advanced physics')
        print('[Isaac 화면 저장]',a.output/'gui_final.png',flush=True)
        if not a.exit_on_finish:
            print('[Isaac 화면] 실행 및 저장 완료. 마지막 상태를 유지합니다. 창을 닫으면 종료됩니다.',flush=True)
            while app.is_running():
                sim.render();time.sleep(1/a.view_fps)

try:main()
except BaseException:
    import traceback
    traceback.print_exc();sys.stdout.flush();sys.stderr.flush();os._exit(1)
else:sys.stdout.flush();os._exit(0)
