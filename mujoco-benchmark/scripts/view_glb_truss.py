"""Visual GLB inspection: authored coordinates, origin pivot, Y-only rotation."""
import argparse,json,struct,time,threading
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def read_glb(path, named=False):
    blob=Path(path).read_bytes();magic,version,length=struct.unpack_from('<III',blob)
    if magic!=0x46546c67 or version!=2 or length!=len(blob):raise ValueError('Invalid GLB')
    offset=12;binary=None;doc=None
    while offset<len(blob):
        n,kind=struct.unpack_from('<II',blob,offset);chunk=blob[offset+8:offset+8+n];offset+=8+n
        if kind==0x4e4f534a:doc=json.loads(chunk)
        if kind==0x004e4942:binary=chunk
    def accessor(i):
        a=doc['accessors'][i];v=doc['bufferViews'][a['bufferView']]
        if 'sparse' in a or v.get('buffer',0)!=0:raise ValueError('Unsupported accessor')
        dtype=np.dtype({5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']]);width={'SCALAR':1,'VEC3':3,'VEC4':4}[a['type']]
        return np.ndarray((a['count'],width),dtype,buffer=binary,offset=v.get('byteOffset',0)+a.get('byteOffset',0),strides=(v.get('byteStride',width*dtype.itemsize),dtype.itemsize)).copy()
    meshes=[]
    def visit(i,parent):
        node=doc['nodes'][i]
        if 'matrix' in node:local=np.array(node['matrix']).reshape(4,4,order='F')
        else:
            local=np.eye(4);local[:3,:3]=Rotation.from_quat(node.get('rotation',[0,0,0,1])).as_matrix()@np.diag(node.get('scale',[1,1,1]));local[:3,3]=node.get('translation',[0,0,0])
        world=parent@local
        if 'mesh' in node:
            for primitive in doc['meshes'][node['mesh']]['primitives']:
                if primitive.get('mode',4)!=4:raise ValueError('Only triangles supported')
                vertices=accessor(primitive['attributes']['POSITION']);vertices=vertices@world[:3,:3].T+world[:3,3]
                indices=accessor(primitive['indices']).reshape(-1,3)
                material=doc.get('materials',[])[primitive['material']] if 'material' in primitive else {}
                color=material.get('pbrMetallicRoughness',{}).get('baseColorFactor',[.2,.5,.08,1])
                meshes.append((node.get('name',f'node_{i}'),vertices,indices,color) if named else (vertices,indices,color))
        for child in node.get('children',[]):visit(child,world)
    for i in doc['scenes'][doc.get('scene',0)]['nodes']:visit(i,np.eye(4))
    return meshes

def build(path):
    root=ET.Element('mujoco',model=Path(path).stem+'_GLB_Y_ONLY');ET.SubElement(root,'compiler',angle='radian');ET.SubElement(root,'visual');root.find('visual').append(ET.Element('global',offwidth='960',offheight='720'))
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='1 1 2',diffuse='.8 .8 .8',ambient='.5 .5 .5');body=ET.SubElement(world,'body',name='glb_origin')
    meshes=read_glb(path)
    for i,(v,f,c) in enumerate(meshes):
        ET.SubElement(assets,'mesh',name=f'm{i}',vertex=' '.join(map(str,v.ravel())),face=' '.join(map(str,f.ravel())))
        ET.SubElement(body,'geom',type='mesh',mesh=f'm{i}',contype='0',conaffinity='0',rgba=' '.join(map(str,c)))
    return ET.tostring(root,encoding='unicode'),np.concatenate([v for v,_,_ in meshes])

def main():
    p=argparse.ArgumentParser();p.add_argument('glb',type=Path);p.add_argument('--y-deg',type=float,default=0);p.add_argument('--screenshot',type=Path);args=p.parse_args()
    if args.screenshot:import render_backend
    import mujoco as mj
    import mujoco.viewer
    xml,vertices=build(args.glb);m=mj.MjModel.from_xml_string(xml);d=mj.MjData(m);bid=m.body('glb_origin').id
    angle=[args.y_deg];guard=threading.Lock()
    def key(k):
        with guard:
            if k==91:angle[0]-=5
            elif k==93:angle[0]+=5
            elif k==48:angle[0]=0
            print(f'GLB Y 회전: {angle[0]:.1f}° / X,Z 추가 회전: 0°',flush=True)
    def update():
        with guard:a=np.deg2rad(angle[0])/2
        m.body_quat[bid]=[np.cos(a),0,np.sin(a),0];mj.mj_forward(m,d)
    def axes(scene):
        for i,(name,color) in enumerate(zip(['GLB +X','GLB +Y (rotation axis)','GLB +Z'],[[1,.1,.1,1],[.1,1,.1,1],[.1,.3,1,1]])):
            end=np.eye(3)[i]*.18;g=scene.geoms[scene.ngeom];scene.ngeom+=1
            mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32));mj.mjv_connector(g,mj.mjtGeom.mjGEOM_ARROW,.003,np.zeros(3),end)
            g=scene.geoms[scene.ngeom];scene.ngeom+=1;mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_SPHERE,np.full(3,.002),end,np.eye(3).ravel(),np.array(color,dtype=np.float32));g.label=name
    update();center=(vertices.min(0)+vertices.max(0))/2;distance=max(.7,np.linalg.norm(np.ptp(vertices,axis=0))*1.5)
    def camera(cam):cam.lookat[:]=center;cam.distance=distance;cam.azimuth=110;cam.elevation=-15
    if args.screenshot:
        r=mj.Renderer(m,height=720,width=960);cam=mj.MjvCamera();camera(cam);opt=mj.MjvOption();opt.label=mj.mjtLabel.mjLABEL_SELECTION;r.update_scene(d,camera=cam,scene_option=opt);axes(r.scene)
        from PIL import Image
        args.screenshot.parent.mkdir(parents=True,exist_ok=True);Image.fromarray(r.render()).save(args.screenshot);r.close();return
    print('원본 GLB 좌표/원점 유지. [ : Y -5도 / ] : Y +5도 / 0 : 원본. 마우스는 카메라만 회전.',flush=True)
    with mj.viewer.launch_passive(m,d,key_callback=key) as viewer:
        camera(viewer.cam);viewer.opt.label=mj.mjtLabel.mjLABEL_SELECTION
        while viewer.is_running():
            with viewer.lock():update();viewer.user_scn.ngeom=0;axes(viewer.user_scn)
            viewer.sync();time.sleep(1/30)
if __name__=='__main__':main()
