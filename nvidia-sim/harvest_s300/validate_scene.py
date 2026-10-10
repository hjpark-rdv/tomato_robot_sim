#!/usr/bin/env python3
"""Structural/geometry checks; these do not assert collision-free harvesting."""
import argparse
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
from build_scene import FARM, sha


def validate(folder):
    manifest=json.loads((folder/'assembly_manifest.json').read_text())
    s=Usd.Stage.Open(str(folder/'scene.usda'))
    source=Usd.Stage.Open(str(FARM))
    reviewed=manifest['farm'].get('mode')=='reviewed_v8_v9_glb_full'
    farm=Usd.Stage.Open(str(folder/('assets/house_collision.usda' if reviewed else 'assets/full_farm.usda')))
    checks={}
    checks['source_hashes_unchanged']=all(sha(Path(p))==v for p,v in manifest['source_sha256'].items())
    checks['generated_hashes_match']=all(sha(folder/p)==v for p,v in manifest['generated_sha256'].items())
    extra={}
    if reviewed:
        from validate_latest_farm import validate_latest
        new_checks,extra=validate_latest(folder,s,manifest['farm'])
        checks.update(new_checks)
    else:
        checks['farm_full_54_plants']=len(s.GetPrimAtPath('/World/Greenhouse/Plants').GetChildren())==54
    fields=['points','faceVertexCounts','faceVertexIndices','normals']
    same=True;meshes=0;triangles=0
    for p in source.TraverseAll():
        if not p.IsA(UsdGeom.Mesh):continue
        q=farm.GetPrimAtPath(p.GetPath());meshes+=1
        for name in fields:
            same &= p.GetAttribute(name).Get()==q.GetAttribute(name).Get()
        triangles+=len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get())
        same &= q.HasAPI(UsdPhysics.CollisionAPI)
    checks['original_meshes_and_normals_unchanged']=bool(same)
    roots=['/World/HarvestRobot/S300','/World/HarvestRobot/Robot']
    for root in roots:
        checks[root+'_finite']=all(np.isfinite(np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get())).all()
          for p in Usd.PrimRange(s.GetPrimAtPath(root),Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh))
    checks['robot_has_seven_motion_joints']=sum(p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint) for p in s.Traverse())==7
    checks['old_tool_absent']=not s.GetPrimAtPath('/World/HarvestRobot/Robot/link6/visuals/assy_gripper_ver_6').IsActive()
    kept=manifest['s300']['kept']
    checks['pump_and_tank_removed']=not any(any(t in row['name'].lower() for t in ('water_tank','pump','nozzle')) for row in kept)
    if 'rear_panel' in manifest['s300']:
        panels=[r for r in kept if r['source'].get('sourcePart')=='T_06. Rear_plate_V2NR.SLDPRT']
        panel_prims=[p for p in Usd.PrimRange(s.GetPrimAtPath(roots[0]))
                     if p.GetCustomDataByKey('sourceName')=='T_06_Rear_plate_V2NR-1']
        checks['original_rear_panel_present_with_collision']=bool(len(panels)==1 and len(panel_prims)==1
            and panel_prims[0].HasAPI(UsdPhysics.CollisionAPI))
        checks['wheel_covers_and_dedicated_mounts_removed']=not any(
            r['source'].get('feature')=='wheel-cover' for r in kept)
    cache=UsdGeom.XformCache();bounds=UsdGeom.BBoxCache(0,['default','render'])
    tcp=s.GetPrimAtPath('/World/HarvestRobot/Robot/link6/tcp')
    tool=s.GetPrimAtPath(str(tcp.GetPath())+'/GripperV51')
    tf=cache.GetLocalToWorldTransform(tool)*cache.GetLocalToWorldTransform(tcp).GetInverse()
    mount=np.array(tf.Transform(Gf.Vec3d(0,-.194,-.018)))
    checks['gripper_mount_frame_matches']=bool(np.allclose(mount,[0,-.012,0],atol=1e-9))
    lift=s.GetPrimAtPath('/World/HarvestRobot/Robot/world/visuals/assy_lift')
    lift_bottom=bounds.ComputeWorldBound(lift).ComputeAlignedRange().GetMin()[2]
    deck_top=manifest['robot_position'][2]+manifest['s300']['deck_top_m']
    checks['lift_sits_on_deck']=abs(lift_bottom-deck_top)<1e-6
    s300_bounds=bounds.ComputeWorldBound(s.GetPrimAtPath(roots[0])).ComputeAlignedRange()
    dimensions=np.array(s300_bounds.GetMax())-s300_bounds.GetMin()
    checks['no_viewer_floor_in_robot']=bool(np.all(dimensions<2.))
    bad_materials=[]
    for p in s.Traverse():
        r=p.GetRelationship('material:binding')
        if r:
            for target in r.GetTargets():
                if not s.GetPrimAtPath(target):bad_materials.append(str(target))
    checks['material_targets_resolve']=not bad_materials
    report=dict(checks={k:bool(v) for k,v in checks.items()},passed=bool(all(checks.values())),
        original_prototype_meshes=meshes,original_prototype_faces=triangles,
        farm_collision_instances=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(s.GetPrimAtPath('/World/Greenhouse'),Usd.TraverseInstanceProxies())),
        s300_dimensions_m=dimensions.tolist(),gripper_adapter_in_tcp_m=mount.tolist(),
        missing_materials=bad_materials,physics_validated=False,elasticity_validated=False,
        plant_asset_validation=extra)
    (folder/'geometry_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);validate(p.parse_args().folder)
