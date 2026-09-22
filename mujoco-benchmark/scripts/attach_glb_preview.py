"""Attach authored GLB at its origin; Y-only world rotation. Visual preview only."""
import argparse,json,copy,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from view_glb_truss import build,read_glb

ROOT=Path(__file__).resolve().parents[2]
def main():
    p=argparse.ArgumentParser();p.add_argument('glb',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--y-deg',type=float,default=0);p.add_argument('--segment',type=int,default=8);p.add_argument('--view',action='store_true');p.add_argument('--axes',action='store_true');args=p.parse_args()
    import mujoco as mj
    import mujoco.viewer
    if not 0<=args.segment<=15 or not np.isfinite(args.y_deg):p.error('segment 0~15, finite Y angle required')
    old=ROOT/'mujoco-benchmark/models/plant_mujoco_optimized.xml';tree=ET.parse(old);root=tree.getroot();world=root.find('worldbody')
    m=mj.MjModel.from_xml_path(str(old));d=mj.MjData(m);mj.mj_forward(m,d)
    stem=world.find("body[@name='STEM_MainStem_00']")
    for n in list(stem.iter('body')):
        for c in list(n.findall('body')):
            if c.get('name','').startswith('TRUSS_'):n.remove(c)
    for c in list(world):
        if c.tag=='body' and c is not stem:world.remove(c)
    retained={n.get('name') for n in stem.iter('body')}
    for c in list(root.find('contact')):
        if c.get('body1') not in retained or c.get('body2') not in retained:root.find('contact').remove(c)
    for tag in ['equality','actuator','sensor','tendon','keyframe']:
        for c in root.findall(tag):root.remove(c)
    name=f'STEM_MainStem_{args.segment:02d}';parent=next(n for n in stem.iter('body') if n.get('name')==name);bid=m.body(name).id
    gid=next(i for i in range(m.ngeom) if m.geom_bodyid[i]==bid and m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
    center=d.geom_xpos[gid];axis=d.geom_xmat[gid].reshape(3,3)[:,2]
    xml,vertices=build(args.glb);glb=ET.fromstring(xml);basis=Rotation.from_euler('x',90,degrees=True).as_matrix()
    rotation=basis@Rotation.from_euler('y',args.y_deg,degrees=True).as_matrix()
    # Choose the stem surface on the side occupied by the original truss.
    heading=basis@vertices.mean(0);normal=heading-axis*np.dot(axis,heading);normal/=np.linalg.norm(normal)
    attachment=center+normal*m.geom_size[gid,0]
    body=glb.find("worldbody/body[@name='glb_origin']")
    for asset in glb.find('asset'):root.find('asset').append(copy.deepcopy(asset))
    for g in body.findall('geom'):g.set('density','0')
    localrot=d.xmat[bid].reshape(3,3).T@rotation;q=Rotation.from_matrix(localrot).as_quat()
    body.set('quat',' '.join(map(str,q[[3,0,1,2]])));body.set('pos',' '.join(map(str,d.xmat[bid].reshape(3,3).T@(attachment-d.xpos[bid]))));parent.append(body)
    root.set('model','GLB_ATTACHED_Y_ONLY_PREVIEW')
    args.output.mkdir(parents=True,exist_ok=False);tree.write(args.output/'scene.xml',encoding='unicode')
    meta=dict(source_glb=str(args.glb.resolve()),existing_stem=str(old),attachment_world_m=attachment.tolist(),parent=name,y_deg=args.y_deg,scale=1,world_rotation_matrix=rotation.tolist(),glb_to_mujoco_basis=basis.tolist(),axis_mapping={'GLB_X':'MuJoCo_X','GLB_Y':'MuJoCo_Z','GLB_Z':'-MuJoCo_Y'},visual_only=True,physics_ready=False)
    (args.output/'scene.json').write_text(json.dumps(meta,indent=2))
    a=mj.MjModel.from_xml_path(str(args.output/'scene.xml'));ad=mj.MjData(a);mj.mj_forward(a,ad);gb=a.body('glb_origin').id
    np.testing.assert_allclose(rotation@np.array([0.,1.,0.]),[0,0,1],atol=1e-10);np.testing.assert_allclose(ad.xmat[gb].reshape(3,3),rotation,atol=1e-10);np.testing.assert_allclose(ad.xpos[gb],attachment,atol=1e-10)
    for n in retained:
        i=m.body(n).id;j=a.body(n).id;np.testing.assert_allclose(d.xpos[i],ad.xpos[j],atol=1e-10)
    print('기존 주줄기 보존 / GLB +Y → 월드 +Z 확인 / 추가 회전은 GLB Y만. 시각 전용.',flush=True)
    if args.view:
        with mj.viewer.launch_passive(a,ad) as viewer:
            viewer.cam.lookat[:]=attachment+rotation@vertices.mean(0)*.6;viewer.cam.distance=1.25;viewer.cam.azimuth=110;viewer.cam.elevation=-12
            if args.axes:viewer.opt.label=mj.mjtLabel.mjLABEL_SELECTION
            while viewer.is_running():
                if args.axes:
                    with viewer.lock():
                        viewer.user_scn.ngeom=0
                        draw_glb_axes(viewer.user_scn,attachment,rotation)
                viewer.sync();time.sleep(1/30)
def draw_glb_axes(scene,origin,rotation):
    import mujoco as mj
    for i,(label,color) in enumerate(zip(['GLB +X','GLB +Y = WORLD +Z','GLB +Z'],[[1,.1,.1,1],[.1,1,.1,1],[.1,.3,1,1]])):
        end=np.asarray(origin)+rotation[:,i]*.16
        g=scene.geoms[scene.ngeom];scene.ngeom+=1
        mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_ARROW,np.zeros(3),origin,np.eye(3).ravel(),np.array(color,dtype=np.float32))
        mj.mjv_connector(g,mj.mjtGeom.mjGEOM_ARROW,.003,origin,end)
        g=scene.geoms[scene.ngeom];scene.ngeom+=1
        mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_SPHERE,np.full(3,.002),end,np.eye(3).ravel(),np.array(color,dtype=np.float32));g.label=label

if __name__=='__main__':main()
