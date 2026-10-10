"""Measure belt contact, fruit/neck relative motion and native break evidence.

Reads recorded states only. A completed command or a rotating whole truss does
not establish that the fruit separated from its own pedicel.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def belongs(path, body):
    return path == body or path.startswith(body + '/')


def measure(states, contacts, breaks, target, belts, dof_names, joint_local_positions):
    stamps=np.asarray(states['time']);phases=np.asarray(states['phase'])
    paths=list(states['body_paths']);poses=np.asarray(states['poses']);dt=np.diff(stamps)
    if len(stamps)<2 or not np.isfinite(poses).all() or not np.isfinite(stamps).all() or np.any(dt<=0):
        raise ValueError('Finite states and strictly increasing physics timestamps required')
    tid=paths.index(target['path']);aid=paths.index(target['anchor'])
    fruit=poses[:,tid];neck=poses[:,aid]
    fr=Rotation.from_quat(fruit[:,3:]);nr=Rotation.from_quat(neck[:,3:])
    relative=nr.inv()*fr
    angle=np.rad2deg((relative*relative[0].inv()).magnitude())
    increments=np.rad2deg((relative[1:]*relative[:-1].inv()).magnitude())
    gap=np.linalg.norm(neck[:,:3]+nr.apply(np.tile(joint_local_positions[0],(len(stamps),1)))
                       -fruit[:,:3]-fr.apply(np.tile(joint_local_positions[1],(len(stamps),1))),axis=1)
    q=np.asarray(states['robot_q']);jaw=44+1000*(q[:,dof_names.index('jaw_R')]-q[:,dof_names.index('jaw_L')])
    by_phase={}
    for name in dict.fromkeys(phases[1:]):
        selected=phases[1:]==name
        by_phase[str(name)]=dict(executed_s=float(dt[selected].sum()),steps=int(selected.sum()),
            start_s=float((stamps[:-1][selected])[0]),end_s=float(stamps[1:][selected][-1]),
            jaw_gap_min_mm=float(jaw[1:][selected].min()),jaw_gap_max_mm=float(jaw[1:][selected].max()),
            relative_rotation_travel_deg=float(increments[selected].sum()))
        if 'belt_surface_velocity_local_y_m_s' in states:
            velocity=np.asarray(states['belt_surface_velocity_local_y_m_s'])[1:][selected]
            by_phase[str(name)].update(belt_peak_surface_speed_m_s=np.max(np.abs(velocity),axis=0).tolist(),
                belt_surface_travel_m=np.sum(np.abs(velocity)*dt[selected,None],axis=0).tolist())
    belt_contacts={body:[] for body in belts};other=[]
    for event in contacts:
        matching=[b for b in belts if any(belongs(event[x],b) and belongs(event[y],target['path'])
                                           for x,y in [('a','b'),('b','a')])]
        if matching:
            for body in matching:belt_contacts[body].append(event)
        elif '/ActuatedTool/' in event['a'] or '/ActuatedTool/' in event['b']:
            other.append(event)
    exact_breaks=[b for b in breaks if b['joint']==target['joint']]
    return dict(phase_evidence=by_phase,timestamp_max_gap_s=float(dt.max()),
        target_path=target['path'],pedicel_body=target['anchor'],target_joint=target['joint'],
        native_target_joint_broken=bool(exact_breaks),target_break_events=exact_breaks,
        other_break_events=[b for b in breaks if b['joint']!=target['joint']],
        fruit_rotation_world_max_deg=float(np.rad2deg((fr*fr[0].inv()).magnitude()).max()),
        fruit_rotation_relative_to_pedicel_max_deg=float(angle.max()),
        fruit_rotation_relative_to_pedicel_travel_deg=float(increments.sum()),
        joint_anchor_gap_initial_mm=float(gap[0]*1000),joint_anchor_gap_max_mm=float(gap.max()*1000),
        joint_anchor_gap_final_mm=float(gap[-1]*1000),
        belt_target_contacts={body:dict(points=len(events),max_point_force_n=max((e['force_n'] for e in events),default=0),
            first_time_s=min((e['t'] for e in events),default=None),
            max_penetration_mm=max([0]+[-1000*e['separation_m'] for e in events])) for body,events in belt_contacts.items()},
        other_tool_contact_points=len(other),
        other_tool_worst_contacts=sorted(other,key=lambda e:e['separation_m'])[:8],
        training_eligible=False,harvest_success=None,
        interpretation='Native joint break, relative rotation, contact and completed commands are separate observations; no biological detachment calibration')


def measure_cluster(states, contacts, breaks, fruits, belts):
    """Count exact native fruit joints, including neighboring fruit, without relabeling pushes as harvest."""
    paths=list(states['body_paths']);poses=np.asarray(states['poses']);times=np.asarray(states['time'])
    if len(times)<2 or not np.isfinite(poses).all() or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
        raise ValueError('Finite monotonic measured states required')
    if len({f['path'] for f in fruits})!=len(fruits) or len({f['joint'] for f in fruits})!=len(fruits):
        raise ValueError('Duplicate fruit or break joint mapping')
    rows=[];belt_times={b:set() for b in belts}
    for f in fruits:
        events=[e for e in contacts if any(belongs(e[x],f['path']) and '/ActuatedTool/' in e[y]
                 for x,y in [('a','b'),('b','a')])]
        by_belt={b:[e for e in events if belongs(e['a'],b) or belongs(e['b'],b)] for b in belts}
        for b,es in by_belt.items():belt_times[b].update(e['t'] for e in es)
        detached=[e for e in breaks if e['joint']==f['joint']]
        xyz=poses[:,paths.index(f['path']),:3]
        rotations=Rotation.from_quat(poses[:,paths.index(f['path']),3:])
        final_relative_shift=None
        if f.get('anchor') in paths:
            anchor=poses[:,paths.index(f['anchor'])]
            local=Rotation.from_quat(anchor[:,3:]).inv().apply(xyz-anchor[:,:3])
            final_relative_shift=float(np.linalg.norm(local[-1]-local[0])*1000)
        rows.append(dict(name=f['name'],body=f['path'],joint=f['joint'],
            max_displacement_mm=float(np.linalg.norm(xyz-xyz[0],axis=1).max()*1000),
            final_displacement_mm=float(np.linalg.norm(xyz[-1]-xyz[0])*1000),
            world_rotation_max_deg=float(np.rad2deg((rotations*rotations[0].inv()).magnitude()).max()),
            separation_from_initial_pedicel_frame_final_mm=final_relative_shift,
            recorded_time_after_first_break_s=float(times[-1]-detached[0]['t']) if detached else None,
            native_detached=bool(detached),native_break_events=detached,
            tool_contact_points=len(events),max_point_force_n=max([0]+[e['force_n'] for e in events]),
            max_penetration_mm=max([0]+[-1000*e['separation_m'] for e in events]),
            belt_contact_points={b:len(es) for b,es in by_belt.items()}))
    concurrent=set.intersection(*belt_times.values()) if belt_times else set()
    phases=np.asarray(states['phase'])
    bilateral_time=sum(dt for t,dt,p in zip(times[1:],np.diff(times),phases[1:]) if float(t) in concurrent and p=='belt_spin')
    return dict(fruits=rows,fruit_count=len(rows),native_detached_count=sum(r['native_detached'] for r in rows),
        contacted_fruit_count=sum(r['tool_contact_points']>0 for r in rows),
        both_belts_touch_any_fruit_during_spin_s=float(bilateral_time),
        other_joint_breaks=[e for e in breaks if e['joint'] not in {f['joint'] for f in fruits}],
        policy='Neighboring fruit contact and displacement allowed. Penetration guard unchanged.',
        interpretation='Both belts may touch different fruit. Native detachment count is distinct from completed motion, collection and real crop safety.',
        training_eligible=False,harvest_success=None)


def analyze(assembly, trial):
    from pxr import Usd, UsdGeom, UsdPhysics
    from scipy.spatial.distance import pdist
    from motion_kinematics import ToolKinematics
    assembly=Path(assembly);trial=Path(trial)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    summary=json.loads((trial/'summary.json').read_text());plan=json.loads((trial/'plan.json').read_text())
    if sha(assembly/'scene.usda')!=summary['model_sha256']:
        raise ValueError('Recorded model hash mismatch')
    stage=Usd.Stage.Open(str(assembly/'scene.usda'));target=manifest['target']
    joint=UsdPhysics.Joint(stage.GetPrimAtPath(target['joint']))
    if list(map(str,joint.GetBody0Rel().GetTargets()))!=[target['anchor']] or list(map(str,joint.GetBody1Rel().GetTargets()))!=[target['path']]:
        raise ValueError('Fruit-pedicel break joint identity mismatch')
    local=[np.asarray(joint.GetLocalPos0Attr().Get()),np.asarray(joint.GetLocalPos1Attr().Get())]
    names=json.loads((trial/'native_metadata.json').read_text())['robot_dofs']
    with np.load(trial/'states.npz') as states:
        result=measure(states,json.loads((trial/'contacts.json').read_text()),
                       json.loads((trial/'break_events.json').read_text()),target,manifest['tool']['belts'],names,local)
        cluster=measure_cluster(states,json.loads((trial/'contacts.json').read_text()),
            json.loads((trial/'break_events.json').read_text()),manifest['plant']['fruits'],manifest['tool']['belts'])
    cluster.update(model_sha256=summary['model_sha256'],completed=summary['completed'],stop_reason=summary['stop_reason'],
        grasp_reference=manifest.get('grasp_reference'),tread_prototype=manifest['tool'].get('tread_prototype'))
    if 'contact_guard' in summary:
        cluster['contact_guard']=summary['contact_guard']
    cluster['valid_complete_run_detachment_count']=cluster['native_detached_count'] if (
        summary['completed'] and summary.get('actual_self_collision_audit_passed') is True) else None
    (trial/'cluster_evidence.json').write_text(json.dumps(cluster,ensure_ascii=False,indent=2)+'\n')
    collider=stage.GetPrimAtPath(target['path']+'/Collider')
    vertices=np.asarray(UsdGeom.Mesh(collider).GetPointsAttr().Get())
    world=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(collider)).T
    points=vertices@world[:3,:3].T+world[:3,3]
    goal=ToolKinematics(stage).fk(np.asarray(plan['goal_q']))
    local_points=(points-goal[:3,3])@goal[:3,:3]
    width=float(np.ptp(local_points[:,0]))
    minimum=manifest['tool']['jaw_limits_mm'][0]
    contact_minimum=manifest['tool'].get('tread_prototype',{}).get('minimum_contact_gap_mm',minimum)
    result.update(model_sha256=summary['model_sha256'],assembly=str(assembly),trial=str(trial),
        input_sha256={n:sha(trial/n) for n in ('plan.json','summary.json','states.npz','contacts.json','break_events.json')},
        analysis_source_sha256=sha(Path(__file__)),completed=summary['completed'],stop_reason=summary['stop_reason'],
        requested_spin=plan.get('belt_spin'),
        grip_geometry=dict(nominal_minimum_gap_mm=minimum,minimum_contact_gap_mm=contact_minimum,
            tread_prototype=manifest['tool'].get('tread_prototype'),target_width_along_grip_axis_mm=width*1000,
            target_maximum_point_distance_mm=float(pdist(points).max()*1000),
            width_shortfall_mm=contact_minimum-width*1000,
            initial_fruit_extents_in_goal_tool_mm=(np.array([local_points.min(0),local_points.max(0)])*1000).tolist(),
            scope='Original mesh/convex hull support; PhysX cooking/contact offset and actual deformation are separate'),
        attachment=dict(type=joint.GetPrim().GetTypeName(),break_force_n=joint.GetBreakForceAttr().Get(),
            break_torque_nm=joint.GetBreakTorqueAttr().Get(),
            behavior='Fixed before native break; no relative fruit-neck torsion compliance or cumulative twist fatigue'))
    (trial/'belt_evidence.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);p.add_argument('trial',type=Path)
    args=p.parse_args();print(json.dumps(analyze(args.assembly,args.trial),ensure_ascii=False,indent=2))
