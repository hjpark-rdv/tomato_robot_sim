#!/usr/bin/env python3
"""Compare native S300 motion traces without rerunning or interpolating physics.

Replicated environments are compared separately with reference world zero.
Shared greenhouse contacts stay in each environment's comparison, while any
cross-environment contact is a failure. Actor-level scalar tensor contacts are
reported separately: they cannot prove collider/impulse-vector equivalence.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.optimize import linear_sum_assignment


ENV_PATTERN=re.compile(r'^/World/MotionEnvs/env_(\d+)(?=/|$)')
MOTION_ROOTS=('/World/HarvestRobot','/World/ActuatedTool','/World/ElasticTarget')
DEFAULT_TOLERANCES=dict(position_m=1e-6,rotation_rad=1e-5,joint_rad=1e-5,
                        timestamp_s=1e-12,separation_m=1e-6,impulse_ns=1e-6)


def normalized_path(path):
    return ENV_PATTERN.sub('/World',str(path))


def environment_id(path):
    path=str(path);match=ENV_PATTERN.match(path)
    if match:return int(match.group(1))
    if any(path==root or path.startswith(root+'/') for root in MOTION_ROOTS):return 0
    return None


def load_json(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default


def json_safe(value):
    """Represent invalid numerical diagnostics explicitly as null, never NaN JSON."""
    if isinstance(value,dict):return {key:json_safe(item) for key,item in value.items()}
    if isinstance(value,(tuple,list)):return [json_safe(item) for item in value]
    if isinstance(value,np.generic):return json_safe(value.item())
    if isinstance(value,(float,np.floating)) and not np.isfinite(value):return None
    return value


def maximum(values):
    return float(np.max(values)) if np.size(values) else 0.


def quaternion_angle(a,b):
    """Sign-invariant orientation error in radians; xyzw storage convention."""
    a=a/np.linalg.norm(a,axis=-1,keepdims=True)
    b=b/np.linalg.norm(b,axis=-1,keepdims=True)
    # Quaternion product conjugate(a)*b, stable near zero unlike arccos(dot).
    vector=a[...,3,None]*b[...,:3]-b[...,3,None]*a[...,:3]-np.cross(a[...,:3],b[...,:3])
    scalar=np.sum(a*b,axis=-1)
    return 2*np.arctan2(np.linalg.norm(vector,axis=-1),np.abs(scalar))


def body_map(paths,environment):
    result={}
    for index,path in enumerate(paths):
        if environment_id(path)!=environment:continue
        canonical=normalized_path(path)
        if canonical in result:raise ValueError('Duplicate normalized body path: '+canonical)
        result[canonical]=index
    return result


def joint_trace(data,environment):
    if 'all_environment_robot_q' in data:
        values=data['all_environment_robot_q']
        if values.ndim==3 and environment<values.shape[1]:return values[:,environment]
    if environment==0:return data['robot_q']
    return None


def prismatic_joint(name):
    return name.startswith('jaw_') or 'lift' in name or name.endswith('height_joint')


def compare_arrays(reference,actual,baseline_map,candidate_map,tolerances,reference_names,candidate_names):
    expected=set(baseline_map);found=set(candidate_map);paths=sorted(expected&found)
    samples=min(len(reference['time']),len(actual['time']))
    times_a=np.asarray(reference['time']);times_b=np.asarray(actual['time'])
    same_length=len(times_a)==len(times_b)
    timestamp_delta=maximum(np.abs(times_a[:samples]-times_b[:samples]))
    finite=all(np.isfinite(data[key]).all() for data in (reference,actual)
               for key in ('time','poses','robot_q','command'))
    if paths and samples:
        a=reference['poses'][:samples,[baseline_map[path] for path in paths]]
        b=actual['poses'][:samples,[candidate_map[path] for path in paths]]
        norms=np.linalg.norm(a[...,3:7],axis=-1)*np.linalg.norm(b[...,3:7],axis=-1)
        finite=finite and bool(np.all(norms>1e-12))
        position_error=np.linalg.norm(a[...,:3]-b[...,:3],axis=-1)
        rotation_error=quaternion_angle(a[...,3:7],b[...,3:7]) if finite else np.full(position_error.shape,np.inf)
        pose_exact=bool(np.array_equal(a,b))
        worst=np.unravel_index(np.argmax(position_error),position_error.shape)
        worst_pose=dict(body=paths[worst[1]],sample=int(worst[0]),time_s=float(times_a[worst[0]]))
    else:
        position_error=rotation_error=np.array([np.inf]);pose_exact=False;worst_pose=None
    qa=reference.get('_selected_q');qb=actual.get('_selected_q')
    common_names=[name for name in reference_names if name in candidate_names]
    name_match=bool(reference_names) and set(reference_names)==set(candidate_names) and len(reference_names)==len(candidate_names)
    q_report=dict(available=qa is not None and qb is not None,name_inventory_matches=name_match)
    q_pass=False
    if q_report['available'] and name_match:
        finite=finite and bool(np.isfinite(qa).all() and np.isfinite(qb).all())
        reference_ids=[reference_names.index(name) for name in common_names]
        actual_ids=[candidate_names.index(name) for name in common_names]
        delta=np.abs(qa[:samples,reference_ids]-qb[:samples,actual_ids])
        per_joint=np.max(delta,axis=0) if samples else np.full(len(common_names),np.inf)
        # Native lift and jaws are prismatic; report metres separately from radians.
        limits=np.array([tolerances['position_m'] if prismatic_joint(name)
                         else tolerances['joint_rad'] for name in common_names])
        q_pass=bool(np.all(per_joint<=limits))
        q_report.update(exact=bool(np.array_equal(qa[:samples,reference_ids],qb[:samples,actual_ids])),
                        maximum_abs_error=float(per_joint.max()),
                        per_joint=[dict(name=name,unit='m' if prismatic_joint(name) else 'rad',
                                        max_abs_error=float(value),tolerance=float(limit))
                                   for name,value,limit in zip(common_names,per_joint,limits)],within_tolerance=q_pass)
    command_match=False
    command_error=None
    if name_match:
        ca=reference['command'][:samples,[reference_names.index(name) for name in common_names]]
        cb=actual['command'][:samples,[candidate_names.index(name) for name in common_names]]
        command_match=same_length and bool(np.array_equal(ca,cb))
        command_error=maximum(np.abs(ca-cb))
    phase_match=same_length and np.array_equal(reference['phase'],actual['phase'])
    inventory_match=expected==found
    passed=(finite and inventory_match and same_length and timestamp_delta<=tolerances['timestamp_s'] and
            maximum(position_error)<=tolerances['position_m'] and
            maximum(rotation_error)<=tolerances['rotation_rad'] and q_pass and command_match and phase_match)
    return dict(passed=bool(passed),all_values_finite=finite,body_inventory_matches=inventory_match,
                missing_body_paths=sorted(expected-found)[:12],unexpected_body_paths=sorted(found-expected)[:12],
                bodies_compared=len(paths),reference_samples=len(times_a),actual_samples=len(times_b),
                same_sample_count=same_length,timestamps_exact=bool(np.array_equal(times_a,times_b)),
                maximum_timestamp_error_s=timestamp_delta,phase_sequence_exact=bool(phase_match),
                poses_exact=pose_exact,maximum_position_error_m=maximum(position_error),
                maximum_rotation_error_rad=maximum(rotation_error),worst_position_error=worst_pose,
                joint_coordinates=q_report,commands_exact=command_match,maximum_command_error=command_error,
                partial_common_prefix_only=not same_length)


def split_contacts(events):
    by_environment=defaultdict(list);shared=[];cross=[]
    for event in events:
        first=environment_id(event['a']);second=environment_id(event['b'])
        if first is not None and second is not None and first!=second:
            cross.append(event);continue
        environment=first if first is not None else second
        if environment is None:shared.append(event)
        else:by_environment[environment].append(event)
    return by_environment,shared,cross


def native_contact_groups(events):
    groups=defaultdict(list)
    for event in events:
        a=normalized_path(event['a']);b=normalized_path(event['b'])
        reverse=a>b
        pair=(b,a) if reverse else (a,b)
        impulse=np.asarray(event['impulse_ns'],float)*(-1 if reverse else 1)
        key=(float(event['t']),str(event['phase']),*pair)
        # force_n remains a distinct recorded measurement; impulse vector is primary.
        groups[key].append(np.r_[float(event['separation_m']),impulse,float(event.get('force_n',0.))])
    return groups


def contact_comparison(reference,actual,tolerances):
    reference_native=all('impulse_ns' in event and event.get('collider_identity_known',True) for event in reference)
    actual_native=all('impulse_ns' in event and event.get('collider_identity_known',True) for event in actual)
    finite=all(np.isfinite(float(event['t'])) and np.isfinite(float(event['separation_m'])) and
               np.isfinite(float(event.get('force_n',0.))) for event in reference+actual)
    if not reference_native or not actual_native:
        def actor_metrics(events):
            return dict(events=len(events),max_recorded_force_n=max((event.get('force_n',0.) for event in events),default=0.),
                        minimum_separation_m=min((event['separation_m'] for event in events),default=None),
                        unknown_actor_events=sum(event.get('actor_identity_known') is False for event in events))
        return dict(passed=None,comparable_native_collider_impulses=False,all_values_finite=finite,
                    reason='Actor normal-force tensor data and native collider impulse vectors are different evidence',
                    reference=actor_metrics(reference),actual=actor_metrics(actual))
    a=native_contact_groups(reference);b=native_contact_groups(actual)
    keys=set(a)|set(b);mismatch=[];max_separation=max_impulse=max_force=0.;points=0;exact=True
    for key in sorted(keys):
        left=a.get(key,[]);right=b.get(key,[])
        if len(left)!=len(right):
            exact=False
            if len(mismatch)<12:mismatch.append(dict(t=key[0],phase=key[1],a=key[2],b=key[3],reference_points=len(left),actual_points=len(right)))
            continue
        if not left:continue
        left=np.asarray(left);right=np.asarray(right)
        finite=finite and bool(np.isfinite(left).all() and np.isfinite(right).all())
        # Point order in PhysX callback manifolds is not stable. Match same pair
        # and exact timestamp points using separation and canonical impulse.
        # No time interpolation, contact omission, or force-metric substitution.
        if len(left)==1:assignment=np.array([0])
        elif np.array_equal(left,right):assignment=np.arange(len(left))
        else:
            cost=np.abs(left[:,None,0]-right[None,:,0])/tolerances['separation_m']
            cost+=np.linalg.norm(left[:,None,1:4]-right[None,:,1:4],axis=2)/tolerances['impulse_ns']
            _,assignment=linear_sum_assignment(cost)
        right=right[assignment];exact=exact and bool(np.array_equal(left,right))
        max_separation=max(max_separation,maximum(np.abs(left[:,0]-right[:,0])))
        max_impulse=max(max_impulse,maximum(np.linalg.norm(left[:,1:4]-right[:,1:4],axis=1)))
        max_force=max(max_force,maximum(np.abs(left[:,4]-right[:,4])));points+=len(left)
    same_keys=set(a)==set(b)
    passed=(finite and same_keys and not mismatch and max_separation<=tolerances['separation_m'] and max_impulse<=tolerances['impulse_ns'])
    return dict(passed=bool(passed),comparable_native_collider_impulses=True,all_values_finite=finite,
                reference_points=len(reference),actual_points=len(actual),matched_points=points,
                timestamp_phase_collider_pair_keys_match=same_keys,manifold_point_counts_match=not mismatch,
                exact_order_independent=exact and same_keys,maximum_separation_error_m=max_separation,
                maximum_canonical_point_impulse_error_ns=max_impulse,
                maximum_recorded_point_force_error_n=max_force,first_mismatched_manifolds=mismatch,
                force_metric='native norm of individual contact point impulse / dt; tensor actor-normal force not substituted')


def compare_runs(baseline,candidate,tolerances=None):
    tolerances=dict(DEFAULT_TOLERANCES if tolerances is None else tolerances)
    if any(not np.isfinite(value) or value<=0 for value in tolerances.values()):raise ValueError('Tolerances must be finite and positive')
    with np.load(baseline/'states.npz') as source:reference={key:source[key] for key in source.files}
    with np.load(candidate/'states.npz') as source:actual={key:source[key] for key in source.files}
    ref_summary=load_json(baseline/'summary.json',{});new_summary=load_json(candidate/'summary.json',{})
    ref_native=load_json(baseline/'native_metadata.json',{});new_native=load_json(candidate/'native_metadata.json',{})
    ref_events=load_json(baseline/'contacts.json',[]);new_events=load_json(candidate/'contacts.json',[])
    ref_by_env,ref_shared,ref_cross=split_contacts(ref_events);new_by_env,new_shared,new_cross=split_contacts(new_events)
    baseline_map=body_map(reference['body_paths'],0)
    environment_ids=sorted({environment_id(path) for path in actual['body_paths'] if environment_id(path) is not None})
    reference['_selected_q']=joint_trace(reference,0);environment_results=[]
    for environment in environment_ids:
        actual['_selected_q']=joint_trace(actual,environment)
        states=compare_arrays(reference,actual,baseline_map,body_map(actual['body_paths'],environment),tolerances,
                              ref_native.get('robot_dofs',[]),new_native.get('robot_dofs',[]))
        contacts=contact_comparison(ref_by_env[0],new_by_env[environment],tolerances)
        environment_results.append(dict(environment=environment,states=states,contacts=contacts))
    shared=contact_comparison(ref_shared,new_shared,tolerances)
    hashes=dict(reference=ref_summary.get('model_sha256'),actual=new_summary.get('model_sha256'))
    hashes['matches']=bool(hashes['reference'] and hashes['reference']==hashes['actual'])
    settings_keys=('hz','solver','armature')
    settings={key:dict(reference=ref_summary.get('physics_settings',{}).get(key),
                       actual=new_summary.get('physics_settings',{}).get(key)) for key in settings_keys}
    settings_match=all(row['reference'] is not None and row['reference']==row['actual'] for row in settings.values())
    outcome_match=all(ref_summary.get(key)==new_summary.get(key) for key in ('completed','stop_reason'))
    contacts_comparable=all(row['contacts']['passed'] is not None for row in environment_results)
    passed=(bool(environment_results) and hashes['matches'] and settings_match and outcome_match and
            not ref_cross and not new_cross and all(row['states']['passed'] and row['contacts']['passed'] is True
                                                   for row in environment_results) and shared['passed'] is True)
    return json_safe(dict(baseline=str(baseline),candidate=str(candidate),passed=bool(passed),
                optimization_physics_equivalent=bool(passed),numeric_tolerances=tolerances,
                model_hashes=hashes,physical_solver_settings=settings,solver_settings_match=settings_match,
                completion_and_stop_reason_match=outcome_match,contact_evidence_comparable=contacts_comparable,
                source_environments=len(environment_results),environment_results=environment_results,
                cross_environment_contacts=dict(reference_count=len(ref_cross),actual_count=len(new_cross),
                    first_actual_events=new_cross[:5]),shared_static_contact_events=shared,
                interpretation='Finite trace regression only; equivalence is not validated harvest success or proof for untested trajectories'))


def shared_timestamp_indices(reference,actual,tolerance=1e-12):
    """Pair actual stored timestamps; no interpolation of trajectories/contacts."""
    if not np.isfinite(tolerance) or tolerance<=0:
        raise ValueError('Timestamp matching tolerance must be finite and positive')
    reference=np.asarray(reference,dtype=float);actual=np.asarray(actual,dtype=float)
    for values in (reference,actual):
        if values.ndim!=1 or not len(values) or not np.isfinite(values).all() or np.any(np.diff(values)<=0):
            raise ValueError('Common-time diagnostic needs finite, strictly increasing nonempty timestamps')
    left=[];right=[];i=j=0
    while i<len(reference) and j<len(actual):
        delta=reference[i]-actual[j]
        if abs(delta)<=tolerance:
            left.append(i);right.append(j);i+=1;j+=1
        elif delta<0:i+=1
        else:j+=1
    return np.asarray(left,dtype=int),np.asarray(right,dtype=int)


def relative_quaternions(initial,values):
    initial=initial/np.linalg.norm(initial,axis=-1,keepdims=True)
    values=values/np.linalg.norm(values,axis=-1,keepdims=True)
    vector=initial[...,3,None]*values[...,:3]-values[...,3,None]*initial[...,:3]-np.cross(initial[...,:3],values[...,:3])
    scalar=np.sum(initial*values,axis=-1,keepdims=True)
    return np.concatenate((vector,scalar),axis=-1)


def authored_first_frame_bias(folder,data,mapping):
    """Read authored world poses offline; a recorded first frame is post-warmup."""
    summary=load_json(folder/'summary.json',{});scene=summary.get('scene')
    result=dict(available=False,scene=scene,source='immutable authored source world transforms before native initialization',
                first_frame_scope='first saved native frame after initialization, not authored rest')
    if not scene or not Path(scene).is_file():
        result['reason']='Source scene unavailable';return result
    path=Path(scene);result['scene_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    result['scene_hash_matches_summary']=result['scene_sha256']==summary.get('model_sha256')
    try:
        from pxr import Usd,UsdGeom
    except ImportError:
        result['reason']='Offline USD Python bindings unavailable';return result
    stage=Usd.Stage.Open(str(path))
    if stage is None:
        result['reason']='Source USD stage could not be opened';return result
    cache=UsdGeom.XformCache(Usd.TimeCode.Default())
    rows=[];missing=[]
    for canonical,index in mapping.items():
        prim=stage.GetPrimAtPath(canonical)
        if not prim:
            missing.append(canonical);continue
        matrix=cache.GetLocalToWorldTransform(prim)
        quaternion=matrix.ExtractRotationQuat();v=quaternion.GetImaginary()
        authored=np.array([*matrix.ExtractTranslation(),*v,float(quaternion.GetReal())])
        first=np.asarray(data['poses'][0,index],float)
        rows.append(dict(body=canonical,position_bias_m=float(np.linalg.norm(first[:3]-authored[:3])),
                         rotation_bias_rad=float(quaternion_angle(authored[3:7],first[3:7])),
                         authored_world_position_m=authored[:3].tolist(),first_world_position_m=first[:3].tolist()))
    result.update(available=bool(rows),bodies_evaluated=len(rows),missing_body_paths=missing,
                  max_position_bias_m=max((row['position_bias_m'] for row in rows),default=None),
                  max_rotation_bias_rad=max((row['rotation_bias_rad'] for row in rows),default=None),
                  worst_bodies=sorted(rows,key=lambda row:row['position_bias_m'],reverse=True)[:12],
                  scope='World body poses; fruit centers/COMs may differ from visual mesh centers; no assumed zero joint state')
    return result


def contact_time_diagnostic(events,summary,start,end):
    """Retain force scope and integrate native impulses in Ns, once only."""
    if any(not np.isfinite(float(e['t'])) or not np.isfinite(float(e['separation_m'])) or
           not np.isfinite(float(e.get('force_n',0.))) or
           ('impulse_ns' in e and not np.isfinite(e['impulse_ns']).all()) for e in events):
        raise ValueError('Nonfinite contact evidence in common-time diagnostic')
    warmup=[e for e in events if e['t']<=start]
    selected=[e for e in events if start<e['t']<=end+1e-12]
    native=all('impulse_ns' in e and e.get('collider_identity_known',True) for e in selected)
    result=dict(contact_point_records=len(selected),interval_s=[start,end],
                warmup_and_start_records_excluded=len(warmup),
                max_recorded_point_force_n=max((e.get('force_n',0.) for e in selected),default=0.),
                minimum_recorded_separation_m=min((e['separation_m'] for e in selected),default=None),
                bit_exact_comparable=False,passed=None)
    if not native:
        result.update(scope='Tensor actor-level normal-only force; friction and collider impulse vectors unavailable',
                      integrated_native_impulse_available=False)
        return result
    by_pair={}
    for e in selected:
        a=normalized_path(e['a']);b=normalized_path(e['b']);reverse=a>b
        pair=(b,a) if reverse else (a,b);impulse=np.asarray(e['impulse_ns'],float)*(-1 if reverse else 1.)
        row=by_pair.setdefault(pair,dict(vector=np.zeros(3),norm_sum=0.,records=0))
        row['vector']+=impulse;row['norm_sum']+=float(np.linalg.norm(impulse));row['records']+=1
    result.update(scope='Native collider point impulses, already Ns; sum once over recorded interval, no dt multiplication',
                  integrated_native_impulse_available=True,collider_pairs=len(by_pair),
                  sum_of_point_impulse_norms_ns=sum(row['norm_sum'] for row in by_pair.values()),
                  largest_pairs=[dict(a=pair[0],b=pair[1],point_records=row['records'],
                    integrated_canonical_impulse_ns=row['vector'].tolist(),sum_point_impulse_norms_ns=row['norm_sum'])
                    for pair,row in sorted(by_pair.items(),key=lambda item:item[1]['norm_sum'],reverse=True)[:15]],
                  interpretation='Time-integrated diagnostic; manifold count, solver step and force peaks can differ with Hz; not contact equivalence')
    return result


def compare_common_time(baseline,candidate,timestamp_tolerance=1e-12):
    """Different-Hz/startup diagnostics; never a universal physics PASS gate."""
    with np.load(baseline/'states.npz') as source:ref={key:source[key] for key in source.files}
    with np.load(candidate/'states.npz') as source:new={key:source[key] for key in source.files}
    ia,ib=shared_timestamp_indices(ref['time'],new['time'],timestamp_tolerance)
    if not len(ia):raise ValueError('No stored shared timestamps; no interpolation performed')
    ref_map=body_map(ref['body_paths'],0);new_map=body_map(new['body_paths'],0)
    paths=sorted(set(ref_map)&set(new_map))
    if not paths:raise ValueError('No shared canonical body paths')
    a=np.asarray(ref['poses'][ia[:,None],[ref_map[p] for p in paths]],float)
    b=np.asarray(new['poses'][ib[:,None],[new_map[p] for p in paths]],float)
    first_a=np.asarray(ref['poses'][0,[ref_map[p] for p in paths]],float)
    first_b=np.asarray(new['poses'][0,[new_map[p] for p in paths]],float)
    finite=all(np.isfinite(data[key]).all() for data in (ref,new) for key in ('poses','robot_q','command'))
    if not finite or any(np.any(np.linalg.norm(v[...,3:7],axis=-1)<=1e-12) for v in (a,b,first_a,first_b)):
        raise ValueError('Nonfinite or invalid quaternion native pose in common-time diagnostic')
    position=np.linalg.norm(a[...,:3]-b[...,:3],axis=-1)
    rotation=quaternion_angle(a[...,3:7],b[...,3:7])
    first_position=np.linalg.norm(first_a[:,:3]-first_b[:,:3],axis=-1)
    first_rotation=quaternion_angle(first_a[:,3:7],first_b[:,3:7])
    relative_position=np.linalg.norm((a[...,:3]-first_a[None,:,:3])-(b[...,:3]-first_b[None,:,:3]),axis=-1)
    relative_rotation=quaternion_angle(relative_quaternions(first_a[None,:,3:7],a[...,3:7]),
                                      relative_quaternions(first_b[None,:,3:7],b[...,3:7]))
    ref_native=load_json(baseline/'native_metadata.json',{});new_native=load_json(candidate/'native_metadata.json',{})
    ref_summary=load_json(baseline/'summary.json',{});new_summary=load_json(candidate/'summary.json',{})
    reference_names=ref_native.get('robot_dofs',[]);candidate_names=new_native.get('robot_dofs',[])
    names=[name for name in reference_names if name in candidate_names]
    joints=[]
    for name in names:
        left=reference_names.index(name);right=candidate_names.index(name)
        q_a=ref['robot_q'][ia,left];q_b=new['robot_q'][ib,right]
        c_a=ref['command'][ia,left];c_b=new['command'][ib,right]
        joints.append(dict(name=name,unit='m' if prismatic_joint(name) else 'rad',
            maximum_native_q_error=float(np.max(np.abs(q_a-q_b))),
            first_native_q_reference=float(ref['robot_q'][0,left]),first_native_q_candidate=float(new['robot_q'][0,right]),
            maximum_command_error=float(np.max(np.abs(c_a-c_b))),
            native_q_exact=bool(np.array_equal(q_a,q_b)),commands_exact=bool(np.array_equal(c_a,c_b))))
    start=float(max(ref['time'][0],new['time'][0]));end=float(min(ref['time'][-1],new['time'][-1]))
    contacts={}
    for label,folder,summary in [('reference',baseline,ref_summary),('candidate',candidate,new_summary)]:
        by_environment,shared,cross=split_contacts(load_json(folder/'contacts.json',[]))
        contacts[label]=contact_time_diagnostic(by_environment[0]+shared,summary,start,end)
        contacts[label].update(environment=0,shared_static_records=len(shared),
                               cross_environment_record_count=len(cross),first_cross_environment_records=cross[:5])
    target={}
    for label,folder,data,mapping in [('reference',baseline,ref,ref_map),('candidate',candidate,new,new_map)]:
        assembly=Path(load_json(folder/'summary.json',{}).get('scene','')).parent
        manifest=load_json(assembly/'motion_manifest.json',{});path=manifest.get('target',{}).get('path')
        if path and normalized_path(path) in mapping:
            index=mapping[normalized_path(path)];values=data['poses'][:,index,:3]
            target[label]=dict(body=normalized_path(path),first_world_position_m=values[0].tolist(),
                maximum_from_own_first_frame_m=maximum(np.linalg.norm(values-values[0],axis=-1)),
                final_from_own_first_frame_m=float(np.linalg.norm(values[-1]-values[0])),
                first_frame_to_authored_target_center_m=(float(np.linalg.norm(values[0]-np.asarray(manifest['target']['center'])))
                    if manifest.get('target',{}).get('center') is not None else None))
    by_body=[dict(body=p,maximum_world_position_error_m=float(position[:,i].max()),
                 maximum_world_rotation_error_rad=float(rotation[:,i].max()),
                 first_frame_position_error_m=float(first_position[i]),first_frame_rotation_error_rad=float(first_rotation[i]),
                 maximum_displacement_from_own_initial_error_m=float(relative_position[:,i].max()),
                 maximum_relative_rotation_error_rad=float(relative_rotation[:,i].max())) for i,p in enumerate(paths)]
    worst=np.unravel_index(np.argmax(position),position.shape)
    def startup(folder,summary):
        validity=load_json(folder/'initialization_validity.json',{})
        runtime=load_json(folder/'runtime_settings.json',{})
        hz=summary.get('physics_settings',{}).get('hz');steps=validity.get('physics_warmup_steps')
        return dict(hz=hz,solver=summary.get('physics_settings',{}).get('solver'),
                    warmup_steps=steps,nominal_warmup_physics_s=steps/hz if steps is not None and hz else None,
                    initialization_valid=summary.get('initialization_valid'),
                    gravity_neutral_warmup=runtime.get('gravity_neutral_warmup'),
                    armature_setup=runtime.get('armature_setup'))
    return json_safe(dict(schema='harvest-s300-common-time-diagnostic-v1',baseline=str(baseline),candidate=str(candidate),
        diagnostic_only=True,passed=None,optimization_physics_equivalent=None,
        timestamp_matching='Only stored timestamps within numerical matching tolerance; no interpolation/resimulation',
        timestamp_matching_tolerance_s=timestamp_tolerance,shared_samples=len(ia),
        reference_samples=len(ref['time']),candidate_samples=len(new['time']),
        shared_interval_s=[float(ref['time'][ia[0]]),float(ref['time'][ia[-1]])],
        entire_duration_covered=abs(ref['time'][-1]-new['time'][-1])<=timestamp_tolerance,
        maximum_matched_timestamp_difference_s=maximum(np.abs(ref['time'][ia]-new['time'][ib])),
        model_hashes=dict(reference=ref_summary.get('model_sha256'),candidate=new_summary.get('model_sha256'),
                          matches=bool(ref_summary.get('model_sha256') and ref_summary.get('model_sha256')==new_summary.get('model_sha256'))),
        startup=dict(reference=startup(baseline,ref_summary),candidate=startup(candidate,new_summary)),
        initial_world=dict(poses_exact=bool(np.array_equal(first_a,first_b)),
            maximum_position_difference_m=maximum(first_position),maximum_rotation_difference_rad=maximum(first_rotation),
            absolute_trajectory_attribution='Different first frames/startup mean absolute differences include startup bias; own-initial differences are also retained'),
        authored_rest_bias=dict(reference=authored_first_frame_bias(baseline,ref,ref_map),
                                candidate=authored_first_frame_bias(candidate,new,new_map)),
        bodies_compared=len(paths),body_inventory_matches=set(ref_map)==set(new_map),
        missing_body_paths=sorted(set(ref_map)-set(new_map)),unexpected_body_paths=sorted(set(new_map)-set(ref_map)),
        maximum_world_position_error_m=maximum(position),maximum_world_rotation_error_rad=maximum(rotation),
        maximum_displacement_from_own_initial_error_m=maximum(relative_position),
        maximum_relative_rotation_error_rad=maximum(relative_rotation),
        worst_world_position_error=dict(body=paths[worst[1]],time_s=float(ref['time'][ia[worst[0]]])),
        per_body=by_body,native_robot_joint_metrics=joints,
        plant_joint_q_series=dict(available=False,reason='Current NPZ records body poses and robot q; no native plant q history'),
        phase_sequence_at_shared_times_exact=bool(np.array_equal(ref['phase'][ia],new['phase'][ib])),
        target=target,contacts=contacts,
        interpretation='Different Hz/solver/warmup is a physical experiment. Metrics are diagnostic, no arbitrary universal tolerance or contact bit-exact PASS.'))


def plot_target_displacement(folders,output):
    """Plot stored native target poses with both initial-reference conventions.

    This is offline evidence only. No interpolation, resimulation, target
    transform editing, or physics-equivalence judgment is performed.
    """
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,layout='constrained')
    evidence=[]
    for folder in folders:
        folder=Path(folder);summary=load_json(folder/'summary.json',{})
        manifest=load_json(Path(summary['scene']).parent/'motion_manifest.json',{})
        target=manifest['target'];path=normalized_path(target['path'])
        with np.load(folder/'states.npz') as data:
            index=body_map(data['body_paths'],0)[path]
            time=np.asarray(data['time'],float);position=np.asarray(data['poses'][:,index,:3],float)
        if not np.isfinite(position).all():raise ValueError('Nonfinite target poses: '+str(folder))
        own=np.linalg.norm(position-position[0],axis=1)*1000
        authored=np.linalg.norm(position-np.asarray(target['center'],float),axis=1)*1000
        label=folder.name
        axes[0].plot(time,own,label=label,linewidth=1.2)
        axes[1].plot(time,authored,label=label,linewidth=1.2)
        evidence.append(dict(run=str(folder),model_sha256=summary.get('model_sha256'),target_body=path,
                             first_native_to_authored_target_center_mm=float(authored[0])))
    for axis in axes:
        axis.grid(alpha=.25);axis.set_ylabel('Target displacement (mm)');axis.legend(fontsize=8)
    axes[0].set_title('From each run\'s first native frame (after initialization)')
    axes[1].set_title('From authored target center (same world reference when model hashes match)')
    axes[1].set_xlabel('Stored physical timestamp (s)')
    figure.savefig(output,dpi=150);plt.close(figure)
    return dict(file=str(output),series=evidence,physics_equivalence=None,
                scope='Measured native body origin; source target center is geometric, not a measured fruit COM')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('baseline',type=Path);parser.add_argument('candidate',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--common-time-grid',action='store_true',help='Different-Hz/startup diagnostic at stored shared timestamps; no physics-equivalence PASS')
    parser.add_argument('--plot-target-displacement',type=Path,help='Optional offline PNG of both native target displacement references')
    for name,value in DEFAULT_TOLERANCES.items():parser.add_argument('--'+name.replace('_','-'),type=float,default=value)
    args=parser.parse_args()
    result=(compare_common_time(args.baseline,args.candidate,args.timestamp_s) if args.common_time_grid
            else compare_runs(args.baseline,args.candidate,{name:getattr(args,name) for name in DEFAULT_TOLERANCES}))
    if args.plot_target_displacement:
        result['target_displacement_plot']=plot_target_displacement([args.baseline,args.candidate],args.plot_target_displacement)
    payload=json.dumps(result,indent=2,allow_nan=False)
    if args.output:
        with args.output.open('x') as stream:stream.write(payload+'\n')
    print(payload)
