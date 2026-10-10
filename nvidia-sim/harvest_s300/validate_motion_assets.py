"""Check adapter geometry, joint limits and original source preservation offline."""
import argparse
import json
from collections import Counter
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from motion_assets import TOOL,sha
from motion_kinematics import ToolKinematics


def inventory(stage,root):
    c=Counter()
    for p in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()):
        name=p.GetCustomDataByKey('sourceName')
        if name and p.IsA(UsdGeom.Mesh):
            m=UsdGeom.Mesh(p)
            c[(name,len(m.GetPointsAttr().Get()),len(m.GetFaceVertexIndicesAttr().Get()))]+=1
    return c


def validate(folder):
    m=json.loads((folder/'motion_manifest.json').read_text())
    source=Usd.Stage.Open(m['source_scene']);stage=Usd.Stage.Open(str(folder/'scene.usda'))
    checks={}
    checks['source_usd_hash_preserved']=sha(m['source_scene'])==m['source_sha256']
    checks['motion_scene_hash_matches']=sha(folder/'scene.usda')==m['scene_sha256']
    checks['all_tool_part_vertices_faces_preserved']=inventory(source,TOOL)==inventory(stage,'/World/ActuatedTool')
    checks['all_active_plant_vertices_faces_preserved']=inventory(source,m['plant']['source_plant'])==inventory(stage,m['plant']['root'])
    checks['old_static_tool_inactive']=not stage.GetPrimAtPath(TOOL).IsActive()
    checks['old_static_plant_inactive']=not stage.GetPrimAtPath(m['plant']['source_plant']).IsActive()
    actual=[]
    for side in ('L','R'):
        j=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath('/World/ActuatedTool/Joints/jaw_'+side))
        actual.append([j.GetLowerLimitAttr().Get(),j.GetUpperLimitAttr().Get()])
    actual=np.array(actual)
    extremes=np.array([.044+actual[1,0]-actual[0,1],.044+actual[1,1]-actual[0,0]])
    checks['cad_28_96mm_limits_preserved']=bool(np.allclose(extremes,[.028,.096],atol=1e-8))
    checks['no_fake_grasp_constraint']=not any('grasp' in str(p.GetPath()).lower() and p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
    checks['native_break_thresholds_preserved']=all(
        UsdPhysics.FixedJoint(stage.GetPrimAtPath(f['joint'])).GetBreakForceAttr().Get()==3.
        and abs(UsdPhysics.FixedJoint(stage.GetPrimAtPath(f['joint'])).GetBreakTorqueAttr().Get()-.08)<1e-7 for f in m['plant']['fruits'])
    k=ToolKinematics(stage);c=UsdGeom.XformCache()
    expected=np.asarray(c.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/ActuatedTool/fixed'))).T
    checks['zero_joint_fk_matches_composed_usd']=bool(np.allclose(k.fk(np.zeros(7)),expected,atol=1e-8))
    result=dict(checks=checks,passed=sum(checks.values()),total=len(checks),all_passed=all(checks.values()),
                scope='asset/kinematic checks, not contact or harvesting validation',jaw_limit_m=extremes.tolist(),
                fruit_aabb_dimensions_m=[f['dimensions_m'] for f in m['plant']['fruits']],
                training_eligible=False)
    (folder/'asset_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['all_passed']:raise RuntimeError('Asset adapter validation failed')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);a=p.parse_args();validate(a.assembly)
