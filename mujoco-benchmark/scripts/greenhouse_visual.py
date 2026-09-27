"""Append the original Isaac greenhouse as static, collision-free visual meshes."""
import argparse,hashlib,json,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pxr import Usd,UsdGeom,UsdShade

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'nvidia-sim/scenes/farmily_greenhouse_robot.usd'

def nums(a):return ' '.join(format(float(v),'.10g') for v in np.asarray(a).ravel())

def add_house(model_xml,output_xml,source=SOURCE):
    source=Path(source).resolve();model_xml=Path(model_xml).resolve();output_xml=Path(output_xml).resolve()
    tree=ET.parse(model_xml);root=tree.getroot();asset=root.find('asset');world=root.find('worldbody')
    if world.find("geom[@name='house_visual_0']") is not None:raise ValueError('House already attached')
    stage=Usd.Stage.Open(str(source));cache=UsdGeom.XformCache()
    if UsdGeom.GetStageUpAxis(stage)!='Z' or UsdGeom.GetStageMetersPerUnit(stage)!=1.:raise ValueError('Expected original Z-up metre scene')
    groups={};inventory=[];hidden=[]
    for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Greenhouse'),Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):continue
        mesh=UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility()==UsdGeom.Tokens.invisible:hidden.append(str(prim.GetPath()));continue
        # Original asset binds materials directly; avoid thousands of binding API warnings.
        targets=prim.GetRelationship('material:binding').GetTargets()
        if targets:mat=UsdShade.Material(stage.GetPrimAtPath(targets[0]))
        else:mat,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        rgba=[.5,.5,.5,1.];roughness=.5
        if mat:
            shader=mat.ComputeSurfaceSource()[0]
            if shader:
                color=shader.GetInput('diffuseColor').Get();opacity=shader.GetInput('opacity').Get();rough=shader.GetInput('roughness').Get()
                if color is not None:rgba[:3]=list(color)
                if opacity is not None:rgba[3]=float(opacity)
                if rough is not None:roughness=float(rough)
        key=tuple(rgba+[roughness]);g=groups.setdefault(key,dict(v=[],f=[],count=0))
        transform=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        v=np.asarray(mesh.GetPointsAttr().Get(),dtype=float);v=v@transform[:3,:3].T+transform[:3,3]
        idx=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
        for n in mesh.GetFaceVertexCountsAttr().Get():
            faces.extend([[idx[offset],idx[offset+j],idx[offset+j+1]] for j in range(1,n-1)]);offset+=n
        faces=np.asarray(faces,dtype=int)
        if (mesh.GetOrientationAttr().Get()=='leftHanded') != (np.linalg.det(transform[:3,:3])<0):faces=faces[:,::-1]
        g['v'].append(v);g['f'].append(faces+g['count']);g['count']+=len(v)
        inventory.append(dict(prim=str(prim.GetPath()),vertices=len(v),triangles=len(faces)))
    for i,(key,g) in enumerate(groups.items()):
        name=f'house_visual_{i}';v=np.concatenate(g['v']);f=np.concatenate(g['f'])
        ET.SubElement(asset,'mesh',name=name,vertex=nums(v),face=' '.join(map(str,f.ravel())))
        ET.SubElement(asset,'material',name=name,rgba=nums(key[:4]),shininess=str(1-key[4]),specular='.3')
        ET.SubElement(world,'geom',name=name,type='mesh',mesh=name,material=name,contype='0',conaffinity='0',density='0',group='2',margin='0',gap='0')
    ET.SubElement(world,'light',name='house_fill',directional='true',pos='0 0 10',dir='0.2 0.1 -1',diffuse='.7 .7 .7',ambient='.3 .3 .3',specular='.2 .2 .2',castshadow='false')
    # Use the original scene ground only as a visual plane; no contacts or inertia.
    ground=UsdGeom.Xformable(stage.GetPrimAtPath('/World/GroundPlane'))
    pos=np.asarray(cache.GetLocalToWorldTransform(ground.GetPrim()))[3,:3]
    ET.SubElement(world,'geom',name='house_ground_visual',type='plane',pos=nums(pos),size='30 30 .1',rgba='.32 .34 .30 1',contype='0',conaffinity='0',density='0',group='2')
    # Model meshes are inline; preserve any pre-existing external file references.
    for node in root.iter():
        if node.get('file') and not Path(node.get('file')).is_absolute():node.set('file',str(model_xml.parent/node.get('file')))
    output_xml.parent.mkdir(parents=True,exist_ok=True);tree.write(output_xml,encoding='unicode')
    import mujoco as mj
    m=mj.MjModel.from_xml_path(str(output_xml));mj.mj_saveModel(m,str(output_xml.with_suffix('.mjb')))
    ids=[i for i in range(m.ngeom) if (m.geom(i).name or '').startswith('house_')]
    assert np.all(m.geom_contype[ids]==0) and np.all(m.geom_conaffinity[ids]==0)
    assert np.all(m.geom_bodyid[ids]==0)
    record=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_layers=[dict(path=l.realPath,sha256=hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest()) for l in stage.GetUsedLayers() if l.realPath and Path(l.realPath).is_file()],meshes=len(inventory),triangles=sum(x['triangles'] for x in inventory),visual_geoms=len(ids),collision_geoms=0,hidden=hidden,inventory=inventory,scope='Original greenhouse geometry and diffuse colors; fixed world visuals only, no added joints or collision. USD lighting/shaders are not reproduced exactly.')
    output_xml.with_suffix('.house.json').write_text(json.dumps(record,indent=2));return record

def main():
    p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('--output',type=Path);p.add_argument('--view',action='store_true');p.add_argument('--view-only',action='store_true');p.add_argument('--state',type=Path,help='Optional saved states.npz; show first frame');p.add_argument('--lookat',type=float,nargs=3,default=[0,0,1.5]);p.add_argument('--distance',type=float,default=9);a=p.parse_args()
    if not a.view_only:
        if a.output is None:p.error('--output is required for conversion')
        if a.output.exists():p.error('Use a new output file')
        info=add_house(a.model,a.output);print({k:v for k,v in info.items() if k not in ('inventory','source_layers')},flush=True)
    if a.view or a.view_only:
        import mujoco as mj,mujoco.viewer
        m=mj.MjModel.from_binary_path(str(a.model if a.view_only else a.output.with_suffix('.mjb')));d=mj.MjData(m)
        if a.state:
            with np.load(a.state) as states:d.qpos[:]=states['qpos'][0]
        mj.mj_forward(m,d)
        with mj.viewer.launch_passive(m,d) as view:
            view.cam.lookat[:]=a.lookat;view.cam.distance=a.distance;view.cam.azimuth=135;view.cam.elevation=-20
            while view.is_running():view.sync();time.sleep(1/30)
        time.sleep(.5)
if __name__=='__main__':main()
