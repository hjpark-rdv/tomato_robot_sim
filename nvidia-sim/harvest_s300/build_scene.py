#!/usr/bin/env python3
"""Assemble the full greenhouse, reviewed plants, S300 V10, RB5 and V5.1 tool.

Offline OpenUSD builder. Original USD/GLB triangles and saved placement metadata.
No MuJoCo collision proxy import, mesh decimation or source asset edits.
The farm is a full static triangle environment at this assembly stage.
"""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

REPO = Path(__file__).resolve().parents[2]
SIM = REPO / 'nvidia-sim'
FARM = SIM / 'env_usd/tomato_greenhouse_upgraded_with_stems_and_clusters_v2_isaac.usd'
ROBOT = SIM / 'robot_usd/rb5_farmily.usd'
TOOL = REPO / 'gripper-design/inaho_gripper_v5/models/inaho_gripper_v5_assembly.glb'


def sha(path):
    return hashlib.file_digest(Path(path).open('rb'), 'sha256').hexdigest()


def new_stage(path, root):
    s = Usd.Stage.CreateNew(str(path))
    s.SetDefaultPrim(UsdGeom.Xform.Define(s, root).GetPrim())
    UsdGeom.SetStageUpAxis(s, 'Z')
    UsdGeom.SetStageMetersPerUnit(s, 1.)
    return s


def array(spec):
    types = {'Float32Array':'<f4','Float64Array':'<f8','Uint32Array':'<u4','Uint16Array':'<u2','Uint8Array':'u1'}
    return np.frombuffer(base64.b64decode(spec['data']), dtype=types[spec['type']])


def material(s, name, rgb, metallic=0., roughness=.55):
    mat = UsdShade.Material.Define(s, str(s.GetDefaultPrim().GetPath())+'/Looks/'+name)
    shader = UsdShade.Shader.Define(s, mat.GetPath().AppendChild('Surface'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float,rgb)))
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def mesh(s, path, points, faces, rgb, normals=None, colors=None, approximation='none', roughness=.55, metallic=0.):
    p = UsdGeom.Mesh.Define(s, path)
    points = np.asarray(points, dtype=np.float32)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite geometry: '+path)
    p.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    p.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces),3,np.int32)))
    p.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(faces,dtype=np.int32).ravel()))
    p.CreateSubdivisionSchemeAttr('none')
    p.CreateDoubleSidedAttr(True)
    p.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.array([points.min(0),points.max(0)],np.float32)))
    p.CreateDisplayColorAttr([Gf.Vec3f(*map(float,rgb))])
    if normals is not None:
        p.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals,np.float32)))
        p.SetNormalsInterpolation('vertex')
    if colors is not None:
        p.CreateDisplayColorPrimvar('vertex').Set(Vt.Vec3fArray.FromNumpy(np.asarray(colors,np.float32)*rgb))
    else:
        UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(material(s,p.GetPrim().GetName(),rgb,metallic,roughness))
    UsdPhysics.CollisionAPI.Apply(p.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(p.GetPrim()).CreateApproximationAttr(approximation)
    return p


def box(s, path, center, size, color):
    p = UsdGeom.Cube.Define(s,path)
    p.CreateSizeAttr(1.)
    p.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
    p.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
    p.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(p.GetPrim())
    return p


def s300_asset(snapshot, output):
    source = json.load(gzip.open(snapshot,'rt'))
    s = new_stage(output,'/S300')
    # Viewer Y-up -> Isaac Z-up. S300 long axis becomes world Y.
    rot = np.array([[0,0,-1],[-1,0,0],[0,1,0]],float)
    kept, removed = [], []
    for i,item in enumerate(source['meshes']):
        points = array(item['positions']).reshape(-1,3)
        tf = np.array(item['matrix']).reshape(4,4).T
        world = trimesh.transform_points(points,tf)
        lo, hi = world.min(0), world.max(0)
        name = item['name']; source_name = item['source'].get('sourcePart','') or ''
        label = (name+' '+source_name).lower()
        reason = None
        if name == '바닥 그림자':
            reason = 'viewer shadow plane, not a robot part (older snapshot compatibility)'
        elif item['source'].get('feature') == 'wheel-cover':
            reason = 'user-requested removal: mecanum wheel covers and their dedicated mounts/fasteners'
        elif source_name == 'T_06. Rear_plate_V2NR.SLDPRT':
            # This chassis closure remains behind the lift. It is not a tank
            # support, even though it intersects the old broad removal box.
            reason = None
        elif any(t in label for t in ('water_tank','pump','nozzle','mast','side_8_pipe','2nd_3030')):
            reason = 'tank/pump/spray assembly'
        elif lo[0] > -.49 and hi[1] > .33 and lo[1] > .25:
            reason = 'former tank/pump upper supports: clear flat rear deck'
        elif 'rail pipe' in label:
            reason = 'CAD external rail fixture (farm supplies rails)'
        row = dict(name=name,source=item['source'],bounds_viewer_m=[lo.tolist(),hi.tolist()])
        if reason:
            removed.append(dict(**row,reason=reason));continue
        transformed = world @ rot.T
        faces = array(item['indices']).reshape(-1,3) if item['indices'] else np.arange(len(points)).reshape(-1,3)
        normals = None
        if item['normals']:
            normals = array(item['normals']).reshape(-1,3) @ np.linalg.inv(tf[:3,:3]) @ rot.T
            normals /= np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
        colors = None
        if item['colors']:
            raw = array(item['colors'])
            colors = raw.reshape(len(points),-1)[:,:3].astype(float)
            if raw.dtype.kind == 'u': colors /= np.iinfo(raw.dtype).max
        p=mesh(s,f'/S300/Part_{i:04d}',transformed,faces,item['color'],normals,colors,
               roughness=item['roughness'],metallic=item['metallic'])
        p.GetPrim().SetCustomDataByKey('sourceName',name)
        kept.append(row)
    if not any(r['source'].get('sourcePart') == 'T_06. Rear_plate_V2NR.SLDPRT' for r in kept):
        raise ValueError('Required original rear panel is missing from the S300 snapshot')
    # Layout plate only: thickness and mounting location are not a structural design.
    box(s,'/S300/FlatRearDeck',(0,-.105,.303),(.57,1.17,.012),(.18,.23,.23))
    s.GetRootLayer().Save()
    return dict(kept=kept,removed=removed,deck_top_m=.309,static_base=True,
                rear_panel='original T_06. Rear_plate_V2NR.SLDPRT retained',
                wheel_covers='removed with dedicated mounts/fasteners; wheels retained',
                design='S300 V10 assembled viewer geometry, full triangles')


def tool_asset(output):
    src=trimesh.load(TOOL,force='scene',process=False)
    s=new_stage(output,'/Gripper')
    # glTF is metres/Y-up. Restore the engineering CAD XYZ (Z-up) exactly once.
    rot=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)
    names=[]
    for i,name in enumerate(sorted(src.graph.nodes_geometry)):
        tf,key=src.graph[name];g=src.geometry[key]
        vertices=trimesh.transform_points(g.vertices,tf)@rot.T
        normals=g.vertex_normals@np.linalg.inv(tf[:3,:3])@rot.T
        normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
        mat=g.visual.material
        rgba=np.asarray(mat.baseColorFactor if mat.baseColorFactor is not None else [180,180,180,255])/255
        # Per-part decomposition, never a convex hull enclosing the whole gripper aperture.
        p=mesh(s,f'/Gripper/Part_{i:04d}',vertices,g.faces,rgba[:3],normals,
               approximation='convexDecomposition')
        p.GetPrim().SetCustomDataByKey('sourceName',name)
        names.append(name)
    s.GetRootLayer().Save()
    return dict(parts=len(names),source_names=names,jaw_gap_mm=44,mode='fixed assembled pose; belt and jaw actuation not enabled')


def robot_asset(tool_path, output):
    s=new_stage(output,'/Robot')
    s.GetDefaultPrim().GetReferences().AddReference(str(ROBOT))
    for p in list(s.Traverse()):
        if p.IsInstance(): p.SetInstanceable(False)
    removed=[]
    for path in ['link6/tcp/tomato_gripper','link6/visuals/assy_gripper_ver_6','link6/visuals/d435',
                 'link6/collisions/assy_gripper_ver_6','link6/collisions/mesh_2']:
        p=s.GetPrimAtPath('/Robot/'+path)
        if p:
            p.SetActive(False);removed.append(str(p.GetPath()))
    restored=[]
    # Same missing-CollisionAPI repair as rl/assets.build_robot; no old ring geometry.
    for p in s.Traverse():
        if '/collisions/' in str(p.GetPath()) and (p.IsA(UsdGeom.Mesh) or p.IsA(UsdGeom.Cube)):
            UsdPhysics.CollisionAPI.Apply(p)
            if p.IsA(UsdGeom.Mesh):
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexHull')
            UsdGeom.Imageable(p).MakeInvisible();restored.append(str(p.GetPath()))
    tool=UsdGeom.Xform.Define(s,'/Robot/link6/tcp/GripperV51')
    tool.GetPrim().GetReferences().AddReference(str(tool_path))
    tool.AddTranslateOp().Set(Gf.Vec3d(0,-.206,.018))
    tool.AddRotateZOp().Set(180.)
    # CAD rear adapter centre (0,-194,-18) mm mates 12 mm beyond TCP.
    spacer=UsdGeom.Cylinder.Define(s,'/Robot/link6/tcp/MountSpacer')
    spacer.CreateAxisAttr('Y');spacer.CreateRadiusAttr(.037);spacer.CreateHeightAttr(.012)
    spacer.AddTranslateOp().Set(Gf.Vec3d(0,-.006,0))
    spacer.CreateDisplayColorAttr([Gf.Vec3f(.26,.29,.3)])
    UsdPhysics.CollisionAPI.Apply(spacer.GetPrim())
    s.GetRootLayer().Save()
    return dict(restored_collision_paths=restored,removed_old_tool_paths=removed,
                mount='12 mm layout spacer; custom adapter to RB5 bolt interface not mechanically verified',
                mass='RB5 source inertia retained; new tool payload inertia not calibrated')


def farm_asset(output):
    # Strong sublayer adds collision APIs to the original shared prototypes.
    # All instance transforms, triangles, foliage and materials remain intact.
    s=Usd.Stage.CreateNew(str(output));s.GetRootLayer().subLayerPaths=[str(FARM)]
    s.SetDefaultPrim(s.GetPrimAtPath('/World'))
    UsdGeom.SetStageUpAxis(s, 'Z')
    UsdGeom.SetStageMetersPerUnit(s, 1.)
    count=0
    for p in s.TraverseAll():
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('none')
            count+=1
    s.GetRootLayer().Save()
    if count==0: raise RuntimeError('Original farm prototype meshes not found')
    return dict(prototype_triangle_colliders=count,plants=54,geometry_reduction=False,
                dynamics='original full meshes with static triangle collisions; elastic plant runtime not attached')


def camera(s,path,eye,target):
    c=UsdGeom.Camera.Define(s,path)
    c.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
    c.CreateFocalLengthAttr(22.);c.CreateClippingRangeAttr(Gf.Vec2f(.02,100.))


def build(a):
    a.output.mkdir(parents=True,exist_ok=True)
    dest=a.output/'scene.usda'
    if dest.exists(): raise FileExistsError(dest)
    assets=a.output/'assets';assets.mkdir(exist_ok=True)
    report={'schema':'s300-harvest-isaac-v1','source_sha256':{str(p):sha(p) for p in (FARM,ROBOT,TOOL,a.s300)},
            'training_eligible':False,'harvesting_validated':False}
    report['s300']=s300_asset(a.s300,assets/'s300.usdc')
    report['tool']=tool_asset(assets/'gripper_v51.usdc')
    report['robot']=robot_asset(assets/'gripper_v51.usdc',assets/'rb5_lift_v51.usda')
    if a.farm_layout == 'reviewed':
        from latest_farm import build_latest
        report['farm']=build_latest(assets/'full_farm.usda', FARM, a.farm_scale)
        report['source_sha256'].update(report['farm']['source_sha256'])
    else:
        report['farm']=farm_asset(assets/'full_farm.usda')
    s=new_stage(dest,'/World')
    phy=UsdPhysics.Scene.Define(s,'/World/PhysicsScene')
    phy.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));phy.CreateGravityMagnitudeAttr(9.81)
    # Authored PhysX schema attributes also work with the offline usd-core wheel.
    phy.GetPrim().CreateAttribute('physxScene:enableGPUDynamics',Sdf.ValueTypeNames.Bool).Set(True)
    phy.GetPrim().CreateAttribute('physxScene:broadphaseType',Sdf.ValueTypeNames.Token).Set('GPU')
    phy.GetPrim().CreateAttribute('physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt).Set(120)
    phy.GetPrim().AddAppliedSchema('PhysxSceneAPI')
    f=UsdGeom.Xform.Define(s,'/World/Greenhouse');f.GetPrim().GetReferences().AddReference(str(assets/'full_farm.usda'))
    f.AddScaleOp().Set(Gf.Vec3d(a.farm_scale if a.farm_layout == 'legacy' else 1.))
    root=UsdGeom.Xform.Define(s,'/World/HarvestRobot');root.AddTranslateOp().Set(Gf.Vec3d(*a.position))
    body=UsdGeom.Xform.Define(s,'/World/HarvestRobot/S300');body.GetPrim().GetReferences().AddReference(str(assets/'s300.usdc'))
    rb=UsdGeom.Xform.Define(s,'/World/HarvestRobot/Robot');rb.GetPrim().GetReferences().AddReference(str(assets/'rb5_lift_v51.usda'))
    rb.ClearXformOpOrder()
    rb.AddTranslateOp().Set(Gf.Vec3d(0,-.14,.659124213455997))
    box(s,'/World/Ground',(0,4,-.12),(16,24,.04),(.21,.23,.22))
    dome=UsdLux.DomeLight.Define(s,'/World/Dome');dome.CreateIntensityAttr(750.)
    sun=UsdLux.DistantLight.Define(s,'/World/Sun');sun.CreateIntensityAttr(2200.)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-35,20,30))
    x,y,z=a.position
    view_scale=a.farm_scale/.5
    camera(s,'/World/RobotCamera',(x+.4,y-1.45,1.55),(x,y+.2,.95))
    UsdGeom.Camera(s.GetPrimAtPath('/World/RobotCamera')).GetFocalLengthAttr().Set(16.)
    camera(s,'/World/FarmCamera',(0,-1.1*view_scale,2.2*view_scale),(0,2*view_scale,1.2*view_scale))
    camera(s,'/World/ToolCamera',(x+.55,y-.85,2.0),(x,y-.6,1.75))
    if a.farm_layout == 'reviewed':
        camera(s,'/World/PlantCamera',(.15*view_scale,.15,1.35),(-.7*view_scale,1.5,.95))
        camera(s,'/World/PairedRowsCamera',(0,4.9*view_scale,1.8),(0,1.6,.9))
    s.GetRootLayer().Save()
    report['scene']=str(dest);report['farm_scale']=a.farm_scale;report['robot_position']=a.position
    report['generated_sha256']={str(p.relative_to(a.output)):sha(p) for p in sorted(assets.iterdir())}
    report['generated_sha256']['scene.usda']=sha(dest)
    report['scope']='assembly + rigid collision baseline; not a validated harvesting/bending simulation'
    (a.output/'assembly_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('scene','farm_scale','scope')},ensure_ascii=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--s300',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--farm-scale',type=float,default=.5,
                   help='Reviewed house size is 0.5, matching farmily_greenhouse_robot.usd; no mesh reduction')
    p.add_argument('--farm-layout',choices=('reviewed','legacy'),default='reviewed',
                   help='reviewed: full v8/v9 stems and corrected Y-only GLB trusses; legacy: old v2 farm')
    p.add_argument('--position',type=float,nargs=3,default=(0.,1.6,0.),
                   help='Full-size robot in the aisle centre; never scaled with the house')
    build(p.parse_args())
