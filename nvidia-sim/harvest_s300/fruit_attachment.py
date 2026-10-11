"""Separate fruit-neck attachment hypotheses; no collider/material edits.

Soft angular limits provide passive resistance rather than a motor drive.
The native PhysX force/torque break thresholds remain the release mechanism.
All parameters are uncalibrated hypotheses, not measured tomato properties.
"""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frame_from_axis(axis):
    x=np.asarray(axis,float)
    if x.shape!=(3,) or not np.isfinite(x).all() or np.linalg.norm(x)<1e-9:
        raise ValueError('A finite nonzero neck axis is required')
    x=x/np.linalg.norm(x)
    helper=np.eye(3)[np.argmin(np.abs(x))]
    y=np.cross(helper,x);y/=np.linalg.norm(y)
    return np.column_stack((x,y,np.cross(x,y)))


def configure_joint(stage,path,*,kind,force_n,torque_nm,twist_k=.01,swing_k=.03,damping=.00015,deadband_deg=1.):
    """Configure the already positioned joint; angular coefficients are authored schema values."""
    if kind not in ('fixed','elastic'):raise ValueError('Unknown attachment kind')
    values=(force_n,torque_nm,twist_k,swing_k,damping,deadband_deg)
    if any(not math.isfinite(v) or v<=0 for v in values) or deadband_deg>=30:
        raise ValueError('Positive finite coefficients and a small deadband are required')
    prim=stage.GetPrimAtPath(path)
    if not prim or not prim.IsA(UsdPhysics.Joint):raise ValueError('Missing source joint')
    prim.SetTypeName('PhysicsJoint' if kind=='elastic' else 'PhysicsFixedJoint')
    j=UsdPhysics.Joint(prim)
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateBreakForceAttr(force_n);j.CreateBreakTorqueAttr(torque_nm)
    if kind=='elastic':
        for axis in ('transX','transY','transZ'):
            limit=UsdPhysics.LimitAPI.Apply(prim,axis)
            limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
        for axis,k in [('rotX',twist_k),('rotY',swing_k),('rotZ',swing_k)]:
            limit=UsdPhysics.LimitAPI.Apply(prim,axis)
            limit.CreateLowAttr(-deadband_deg);limit.CreateHighAttr(deadband_deg)
            prim.AddAppliedSchema('PhysxLimitAPI:'+axis)
            for name,value in [('stiffness',k),('damping',damping),('restitution',0.)]:
                prim.CreateAttribute('physxLimit:'+axis+':'+name,Sdf.ValueTypeNames.Float).Set(value)
    return dict(kind=kind,force_n=force_n,torque_nm=torque_nm,twist_stiffness_authored=twist_k,
        swing_stiffness_authored=swing_k,damping_authored=damping,deadband_deg=deadband_deg,
        release='native PhysX constraint break; no timer, pose teleport or custom angular release',
        coefficient_note='Raw PhysxLimitAPI coefficients; native fixture response measured, not calibrated continuous tissue stiffness',
        rotation_model='passive soft angular limits' if kind=='elastic' else 'fixed',
        calibrated=False)


def _outside_joint_hash(layer,paths):
    normalized=Sdf.Layer.CreateAnonymous()
    normalized.ImportFromString(layer.ExportToString())
    normalized.subLayerPaths=[]
    edits=Sdf.BatchNamespaceEdit()
    for path in paths:edits.Add(path,Sdf.Path.emptyPath)
    if not normalized.Apply(edits):raise ValueError('Could not mask attachment specs for audit')
    return hashlib.sha256(normalized.ExportToString().encode()).hexdigest()


def revise(assembly,output,kind='elastic',force_n=3.,torque_nm=.008):
    assembly=Path(assembly).resolve();output=Path(output).resolve()
    source=assembly/'scene.usda';manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if sha(source)!=manifest['scene_sha256']:raise ValueError('Source scene hash mismatch')
    if 'fruit_attachment' in manifest:raise ValueError('Refusing to revise an already revised attachment')
    prefix=manifest['target']['name'].split('_Fruit_')[0]+'_Fruit_'
    fruits=[f for f in manifest['plant']['fruits'] if f['name'].startswith(prefix)]
    if not fruits:raise ValueError('Empty current truss')
    original=Usd.Stage.Open(str(source));cache=UsdGeom.XformCache()
    output.mkdir(parents=True,exist_ok=False);shutil.copy2(source,output/'scene.usda')
    stage=Usd.Stage.Open(str(output/'scene.usda'))
    sublayers=[str(Path(Sdf.ComputeAssetPathRelativeToLayer(original.GetRootLayer(),p)).resolve())
               for p in original.GetRootLayer().subLayerPaths]
    stage.GetRootLayer().subLayerPaths=sublayers
    rows=[]
    for f in fruits:
        source_joint=UsdPhysics.Joint(original.GetPrimAtPath(f['joint']))
        if source_joint.GetPrim().GetTypeName()!='PhysicsFixedJoint':raise ValueError('Expected fixed source attachment')
        if list(map(str,source_joint.GetBody0Rel().GetTargets()))!=[f['anchor']] or list(map(str,source_joint.GetBody1Rel().GetTargets()))!=[f['path']]:
            raise ValueError('Fruit/neck mapping mismatch')
        poses=[np.asarray(cache.GetLocalToWorldTransform(original.GetPrimAtPath(p))).T for p in (f['anchor'],f['path'])]
        pivot=poses[0][:3,:3]@np.asarray(source_joint.GetLocalPos0Attr().Get())+poses[0][:3,3]
        frame=frame_from_axis(pivot-poses[0][:3,3])
        j=UsdPhysics.Joint(stage.GetPrimAtPath(f['joint']))
        if kind=='elastic':
            # X follows the actual distal neck at rest, never an assumed world axis.
            for i,pose in enumerate(poses):
                quat=R.from_matrix(pose[:3,:3].T@frame).as_quat()
                getattr(j,f'CreateLocalRot{i}Attr')().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*map(float,quat[:3]))))
        config=configure_joint(stage,f['joint'],kind=kind,force_n=force_n,torque_nm=torque_nm)
        rows.append(dict(name=f['name'],joint=f['joint'],anchor=f['anchor'],fruit=f['path'],
            pivot_world=pivot.tolist(),neck_axis_world=frame[:,0].tolist(),**config))
        f['break_force_n']=force_n;f['break_torque_nm']=torque_nm
        if f['path']==manifest['target']['path']:
            manifest['target'].update(break_force_n=force_n,break_torque_nm=torque_nm)
    paths=[f['joint'] for f in fruits]
    before=_outside_joint_hash(original.GetRootLayer(),paths)
    after=_outside_joint_hash(stage.GetRootLayer(),paths)
    if before!=after:raise ValueError('Unexpected edits outside selected attachment joints')
    stage.GetRootLayer().Save()
    if sha(source)!=manifest['scene_sha256']:raise ValueError('Source modified unexpectedly')
    manifest['fruit_attachment']=dict(kind=kind,source_assembly=str(assembly),source_scene_sha256=sha(source),
        scope='Only current-truss fruit-neck joints; all other source properties unchanged',
        joints=rows,preservation_audit=dict(passed=True,outside_joint_spec_sha256=before,
            resolved_sublayers_unchanged=True),calibrated=False,training_eligible=False)
    manifest['scene_sha256']=sha(output/'scene.usda')
    (output/'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    shutil.copy2(assembly/'skin.npz',output/'skin.npz')
    return manifest['fruit_attachment']


def relative_angles(q0,q1):
    """Twist around joint-local X and swing; q1 relative to q0, xyzw."""
    rel=(R.from_quat(q0).inv()*R.from_quat(q1)).as_quat()
    denom=np.hypot(rel[...,0],rel[...,3])
    if np.any(denom<1e-8):raise ValueError('Twist undefined at a 180 degree swing')
    twist=np.zeros_like(rel);twist[...,0]=rel[...,0]/denom;twist[...,3]=rel[...,3]/denom
    angle=(2*np.arctan2(twist[...,0],twist[...,3])+np.pi)%(2*np.pi)-np.pi
    swing=(R.from_quat(rel)*R.from_quat(twist).inv()).magnitude()
    return angle,swing


def analyze_rotations(stage,manifest,states,breaks):
    """Measured pre-break twist/bend, with the same neck-axis convention for fixed controls."""
    paths=states['body_paths'].tolist();times=np.asarray(states['time']);poses=np.asarray(states['poses'])
    cache=UsdGeom.XformCache();prefix=manifest['target']['name'].split('_Fruit_')[0]+'_Fruit_'
    rows=[]
    for f in manifest['plant']['fruits']:
        if not f['name'].startswith(prefix):continue
        j=UsdPhysics.Joint(stage.GetPrimAtPath(f['joint']))
        rest=[np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T for p in (f['anchor'],f['path'])]
        pivot=rest[0][:3,:3]@np.asarray(j.GetLocalPos0Attr().Get())+rest[0][:3,3]
        frame=frame_from_axis(pivot-rest[0][:3,3])
        events=[e for e in breaks if e['joint']==f['joint']]
        last=events[0]['t'] if events else times[-1]
        select=times<=last+1e-10
        anchor=poses[select,paths.index(f['anchor'])];fruit=poses[select,paths.index(f['path'])]
        q0=(R.from_quat(anchor[:,3:])*R.from_matrix(rest[0][:3,:3].T@frame)).as_quat()
        q1=(R.from_quat(fruit[:,3:])*R.from_matrix(rest[1][:3,:3].T@frame)).as_quat()
        twist,swing=relative_angles(q0,q1);twist=np.unwrap(twist)
        twist_deg=np.rad2deg(twist);swing_deg=np.rad2deg(swing)
        phases=states['phase'][select];phase_evidence={}
        for name in dict.fromkeys(phases.tolist()):
            mask=phases==name
            phase_evidence[name]=dict(max_abs_twist_deg=float(np.abs(twist_deg[mask]).max()),
                max_bend_deg=float(swing_deg[mask].max()),end_twist_deg=float(twist_deg[mask][-1]))
        rows.append(dict(name=f['name'],joint=f['joint'],breaks=events,
            max_abs_twist_before_or_at_break_deg=float(np.abs(twist_deg).max()),
            max_bend_before_or_at_break_deg=float(swing_deg.max()),
            sample_before_break_twist_deg=float(twist_deg[-2]) if events and len(twist)>1 else None,
            sample_before_break_bend_deg=float(swing_deg[-2]) if events and len(swing)>1 else None,
            phase_evidence=phase_evidence,time_s=times[select].tolist(),
            twist_deg=twist_deg.tolist(),bend_deg=swing_deg.tolist()))
    return dict(joints=rows,attachment=manifest.get('fruit_attachment'),
        scope='Measured relative fruit-neck orientation, not world rotation; native reaction wrench is not available here',
        break_cause='Native break event does not distinguish force, twist torque and bending torque',
        training_eligible=False,harvest_success=None)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--kind',choices=['fixed','elastic'],default='elastic')
    p.add_argument('--force-n',type=float,default=3.);p.add_argument('--torque-nm',type=float,default=.008)
    a=p.parse_args();print(json.dumps(revise(a.assembly,a.output,a.kind,a.force_n,a.torque_nm),indent=2))
