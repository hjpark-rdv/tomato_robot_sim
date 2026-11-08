#!/usr/bin/env python3
"""Known-colour, known-distance render control for the mounted RGB-D sensor.

Run with Isaac Python. This is an optical calibration fixture, not farm physics.
"""
import argparse
import json
from recording_storage import load_array
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
p.add_argument('--gt',action='store_true',help='Also validate GT identity, depth alignment, and hidden/out-of-view fruit exclusion')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'renderer':'RaytracedLighting'})
exit_code=0
try:
    import numpy as np
    import omni.usd
    import omni.timeline
    from pxr import Gf,UsdGeom,UsdLux
    from mounted_rgbd import mount,RGBDRecorder,BODY,K,optical_pose
    omni.usd.get_context().new_stage();s=omni.usd.get_context().get_stage()
    s.SetDefaultPrim(UsdGeom.Xform.Define(s,'/World').GetPrim())
    UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
    UsdGeom.Xform.Define(s,BODY)
    metadata=mount(s);pose=np.array([[0.,0.,0.,0.,0.,0.,1.]])
    optical=optical_pose(pose[0],metadata)
    colors=[[1,0,0],[0,1,0],[0,0,1],[1,1,0]]
    centers=[(-.16,-.16),(.16,-.16),(-.16,.16),(.16,.16)]
    def patch(name,x,y,z,size,color,source=None):
        v=np.array([[x-size,y-size,z],[x+size,y-size,z],[x+size,y+size,z],[x-size,y+size,z]])
        world=v@optical[:3,:3].T+optical[:3,3]
        base='/World/Greenhouse/Plants/Plant_00/Truss_00/' if a.gt else '/World/'
        m=UsdGeom.Mesh.Define(s,base+name)
        m.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in world])
        m.CreateFaceVertexCountsAttr([4]);m.CreateFaceVertexIndicesAttr([0,1,2,3])
        m.CreateDoubleSidedAttr(True);m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if source:m.GetPrim().SetCustomDataByKey('sourceName',source)
    patch('Background',0,0,2.,1.5,[.4,.4,.4])
    names=['Fruit_01','MainStem','CompoundLeaf_01','Fruit_02']
    for n,((x,y),color) in enumerate(zip(centers,colors)):patch('Marker'+str(n),x,y,1.,.10,color,names[n])
    if a.gt:
        patch('HiddenFruit',-.24,-.24,1.5,.10,[1,0,0],'Fruit_03')
        patch('OutsideFruit',10,0,1.,.1,[1,0,0],'Fruit_04')
        patch('PartialOccludingLeaf',-.176,-.24,.8,.02,[0,1,0],'CompoundLeaf_02')
        (a.output/'motion_manifest.json').write_text('{}')
    light=UsdLux.DomeLight.Define(s,'/World/Light');light.CreateIntensityAttr(1200)
    timeline=omni.timeline.get_timeline_interface();timeline.set_auto_update(False);timeline.stop()
    class StaticClock:
        def get_num_physics_steps(self):return 0
        def get_simulation_time(self):return 0.
    recorder=RGBDRecorder(s,a.output/'rgbd',metadata,120,2,[BODY],gt_assembly=a.output if a.gt else None)
    recorder.capture(app,StaticClock(),pose,np.zeros(10),0,0.,'optical_fixture',sync=lambda:None)
    recorder.close()
    from PIL import Image
    image=np.array(Image.open(a.output/'rgbd/frame_00000/rgb.png'))
    depth=load_array(a.output/'rgbd/frame_00000/depth.npy')
    checks=[]
    for (x,y),color in zip(centers,colors):
        u,v=np.rint([K[0,0]*x+K[0,2],K[1,1]*y+K[1,2]]).astype(int)
        rgb=np.median(image[v-3:v+4,u-3:u+4],axis=(0,1))
        d=float(np.nanmedian(depth[v-3:v+4,u-3:u+4]))
        expected=np.array(color,dtype=bool)
        good_color=bool(rgb[expected].min()>rgb[~expected].max()+30)
        checks.append(dict(pixel=[int(u),int(v)],expected_rgb=color,measured_rgb=rgb.tolist(),
            expected_optical_depth_m=1.,measured_depth_m=d,pass_color=good_color,
            pass_depth=bool(abs(d-1.)<.001)))
    center=float(np.nanmedian(depth[317:324,237:244]))
    report=dict(scope='known planes, rendering only, no physics or harvesting',
        marker_checks=checks,background_center_depth_m=center,expected_background_depth_m=2.,
        passed=all(c['pass_color'] and c['pass_depth'] for c in checks) and abs(center-2.)<.001,
        depth_units='metres',segmentation_saved=a.gt)
    if a.gt:
        labels=load_array(a.output/'rgbd/frame_00000/semantic.npy')
        objects=json.loads((a.output/'rgbd/frame_00000/objects.json').read_text())['objects']
        expected=[1,2,7,1]
        correct=[int(labels[c['pixel'][1],c['pixel'][0]])==x for c,x in zip(checks,expected)]
        keys=[r['key'] for r in objects]
        occlusion_uv=np.rint([K[0,0]*(-.176/.8)+K[0,2],K[1,1]*(-.24/.8)+K[1,2]]).astype(int)
        partial_leaf=int(labels[occlusion_uv[1],occlusion_uv[0]])==7
        hidden_absent=not any(k.endswith(('/Fruit_03','/Fruit_04')) for k in keys)
        report['gt']=dict(correct_classes=correct,hidden_and_outside_absent=hidden_absent,
            partial_occlusion_remains_leaf=partial_leaf,visible_keys=keys)
        report['passed'] &= all(correct) and hidden_absent and partial_leaf
    (a.output/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise RuntimeError('RGB-D optical calibration failed')
except Exception:
    import traceback
    exit_code=1;error=traceback.format_exc();print(error,flush=True)
    (a.output/'runtime_error.txt').write_text(error)
finally:app.close(exit_code=exit_code)
