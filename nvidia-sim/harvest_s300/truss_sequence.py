"""Remaining-fruit selection for the existing native runner, with no reset API.

SIM-GT selection uses measured fruit positions and exact native break joint IDs.
This changes arm goals, not scene poses, collision policy or joint properties.
"""
from numbers import Integral
import hashlib
import json
import numpy as np


def truss_fruits(manifest):
    target=manifest['target']['name']
    if '_Fruit_' not in target:raise ValueError('Target has no explicit truss identity')
    prefix=target.split('_Fruit_')[0]+'_Fruit_'
    fruits=[f for f in manifest['plant']['fruits'] if f['name'].startswith(prefix)]
    if not fruits or len({f['joint'] for f in fruits})!=len(fruits) or len({f['path'] for f in fruits})!=len(fruits):
        raise ValueError('Empty or ambiguous truss fruit mapping')
    return fruits


def inventory(fruits,breaks):
    broken={e['joint'] for e in breaks}
    return dict(detached=[f['name'] for f in fruits if f['joint'] in broken],
                remaining=[f['name'] for f in fruits if f['joint'] not in broken])


def horizontal_truss_direction(fruits, breaks, body_paths, poses, reference_direction,
                               fallback_direction=None, min_attached_fruits=2):
    """Top-view fruit long axis, signed toward the plant from the robot side.

    Fit only attached fruits in this exact truss. Z is excluded from the fit;
    this is SIM-GT yaw selection, not a pitch command or visual perception.
    An ambiguous/one-fruit remainder retains the last verified truss direction.
    Opting into a larger minimum also retains that direction for a sparse
    remainder: a final transverse pair does not establish the truss long axis.
    The default minimum preserves the original fit and returned evidence.
    """
    if isinstance(min_attached_fruits,bool) or not isinstance(min_attached_fruits,Integral) or min_attached_fruits<2:
        raise ValueError('Minimum attached fruits must be an integer of at least two')
    reference=np.asarray(reference_direction,dtype=float).copy()
    if reference.shape!=(3,) or not np.isfinite(reference).all() or np.linalg.norm(reference[:2])<1e-9:
        raise ValueError('Finite horizontal sign reference is required')
    remaining=set(inventory(fruits,breaks)['remaining'])
    attached=[f for f in fruits if f['name'] in remaining]
    paths=list(body_paths)
    centers=np.asarray([poses[paths.index(f['path']),:3] for f in attached],dtype=float).reshape(-1,3)
    if not np.isfinite(centers).all():raise ValueError('Nonfinite truss-axis source positions')
    singular=np.zeros(2);reason=None
    if len(centers)<2:
        reason='fewer_than_two_attached_fruits'
    else:
        _,values,vt=np.linalg.svd(centers[:,:2]-centers[:,:2].mean(axis=0),full_matrices=False)
        singular[:len(values)]=values
        if len(centers)<min_attached_fruits:
            reason='insufficient_attached_fruits_for_longitudinal_axis'
        elif singular[0]<.001 or singular[0]<=1.05*singular[1]:
            reason='ambiguous_xy_long_axis'
        else:
            direction=np.r_[vt[0],0.]
            if direction@reference<0:direction=-direction
    if reason:
        if fallback_direction is None:raise ValueError('Cannot determine truss yaw: '+reason)
        direction=np.asarray(fallback_direction,dtype=float).copy()
        if direction.shape!=(3,) or not np.isfinite(direction).all() or abs(direction[2])>1e-9 or np.linalg.norm(direction[:2])<1e-9:
            raise ValueError('Fallback must be a finite horizontal direction')
    direction=direction/np.linalg.norm(direction)
    direction[2]=0.
    evidence=dict(direction_world=direction.tolist(),
                yaw_world_deg=float(np.degrees(np.arctan2(direction[1],direction[0]))),
                fruit_names=[f['name'] for f in attached],centers_world_m=centers.tolist(),
                xy_singular_values_m=singular.tolist(),fallback_reason=reason,
                method='attached-fruit XY principal axis; sign follows robot-side/previous entry',
                scope='SIM-GT top-view yaw only; no realtime tracking within a motion cycle')
    if min_attached_fruits!=2:
        evidence['axis_fit_evidence']=dict(attached_fruit_count=len(attached),
            min_attached_fruits=int(min_attached_fruits),axis_fit_used=reason is None,
            retained_previous_verified_direction=reason is not None,
            retained_direction_world=direction.tolist() if reason is not None else None)
    return evidence


def validate_max_fruit_attempts(value):
    """Finite user-controlled retry budget; failure never means impossible."""
    if isinstance(value,bool) or not isinstance(value,Integral) or not 1<=value<=8:
        raise ValueError('Fruit attempt budget must be an integer within 1..8')
    return int(value)


def validate_single_fruit_when_remaining(value):
    if isinstance(value,bool) or not isinstance(value,Integral) or not 0<=value<=3:
        raise ValueError('Individual sparse-fruit threshold must be an integer within 0..3')
    return int(value)


def planning_fingerprint(group,start_q,axis,parameters):
    """Exact immutable planning inputs, excluding changing retry counters.

    A fingerprint is diagnostic deduplication evidence, not a reachability
    certificate. In particular, changing any physical q/center/axis invalidates
    the old failure. The caller owns a cache limited to unchanged native state.
    """
    q=np.asarray(start_q,float);centers=np.asarray(group['measured_centers_world_m'],float)
    centroid=np.asarray(group['measured_centroid_world_m'],float)
    if q.shape!=(7,) or centers.shape!=(len(group['fruits']),3) or centroid.shape!=(3,) or not np.isfinite(np.r_[q,centers.ravel(),centroid]).all():
        raise ValueError('Finite actual planning q and selected centers are required')
    direction=None if axis is None else np.asarray(axis['direction_world'],float)
    if direction is not None and (direction.shape!=(3,) or not np.isfinite(direction).all()):
        raise ValueError('Finite actual planning direction is required')
    payload=dict(fruits=group['fruits'],centers_world_m=centers.tolist(),centroid_world_m=centroid.tolist(),
                 start_q=q.tolist(),direction_world=None if direction is None else direction.tolist(),parameters=parameters)
    serialized=json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False)
    return hashlib.sha256(serialized.encode()).hexdigest()


def validated_rejected_groups(fruits,rejected_groups):
    names={f['name']for f in fruits};result=set()
    for group in rejected_groups:
        if isinstance(group,str) or not group or len(set(group))!=len(group) or any(n not in names for n in group):
            raise ValueError('Rejected planning group contains unknown or duplicate fruit names')
        result.add(tuple(sorted(group)))
    return result


def unproductive_pairs(cycle_plans,fruit_joints):
    """Pairs whose completed native cycle left both exact fruit joints attached.

    This evidence can opt into a different centering on the next attempt. It
    does not certify that either fruit is unreachable, and does not consume or
    extend the caller's finite attempt budget. Planning rejects, interrupted
    cycles and incomplete/contradictory inventories provide no such evidence.
    The caller must supply the name-to-native-joint map saved with these cycles.
    """
    if not isinstance(fruit_joints,dict) or not fruit_joints or any(
            not isinstance(name,str) or not name or not isinstance(joint,str) or not joint
            for name,joint in fruit_joints.items()) or len(set(fruit_joints.values()))!=len(fruit_joints):
        raise ValueError('Unambiguous exact native fruit joint mapping is required')
    known=set(fruit_joints)

    def attached_inventory(value):
        if not isinstance(value,dict):return None
        remaining=value.get('remaining');detached=value.get('detached')
        if not isinstance(remaining,list) or not isinstance(detached,list):return None
        if any(not isinstance(name,str) for name in remaining+detached):return None
        if len(set(remaining))!=len(remaining) or len(set(detached))!=len(detached):return None
        remaining=set(remaining);detached=set(detached)
        if remaining&detached or remaining|detached!=known:return None
        return remaining

    result=set()
    for cycle in cycle_plans:
        if not isinstance(cycle,dict) or cycle.get('status')!='motion_completed' or cycle.get('stop_reason') is not None:
            continue
        pair=cycle.get('fruits')
        if not isinstance(pair,list) or len(pair)!=2 or any(not isinstance(n,str) for n in pair):continue
        if len(set(pair))!=2 or not set(pair)<=known:continue
        # Pair evidence never broadens the current exact truss's grasp scope.
        prefixes=[n.split('_Fruit_')[0] if '_Fruit_' in n else None for n in pair]
        if prefixes[0] is None or prefixes[0]!=prefixes[1]:continue
        start=cycle.get('start_s');end=cycle.get('end_s')
        if isinstance(start,bool) or isinstance(end,bool):continue
        try:
            start=float(start);end=float(end)
            if not np.isfinite([start,end]).all() or end<=start:continue
            planned=cycle.get('planned_end_s')
            if planned is not None and (isinstance(planned,bool) or not np.isfinite(float(planned)) or end<float(planned)-1e-7):continue
        except (TypeError,ValueError):continue
        first=cycle.get('measured_frame_index');last=cycle.get('end_frame_index')
        if isinstance(first,bool) or isinstance(last,bool) or not isinstance(first,Integral) or not isinstance(last,Integral) or first<0 or last<=first:
            continue
        after=attached_inventory(cycle.get('inventory_after'))
        if after is None or not set(pair)<=after:continue
        if 'inventory_before' in cycle:
            before=attached_inventory(cycle['inventory_before'])
            if before is None or not set(pair)<=before:continue
        result.add(tuple(sorted(pair)))
    return result


def select_group(fruits,breaks,body_paths,poses,attempts,max_attempts=2,order='lowest-first',
                 single_fruit_when_remaining=0,rejected_groups=()):
    max_attempts=validate_max_fruit_attempts(max_attempts)
    single_fruit_when_remaining=validate_single_fruit_when_remaining(single_fruit_when_remaining)
    rejected=validated_rejected_groups(fruits,rejected_groups)
    if order not in ('lowest-first','highest-first'):raise ValueError('Unknown truss selection order')
    remaining=set(inventory(fruits,breaks)['remaining'])
    eligible=[f for f in fruits if f['name'] in remaining and attempts.get(f['name'],0)<max_attempts]
    positions={f['name']:np.asarray(poses[body_paths.index(f['path']),:3],float) for f in eligible}
    if any(not np.isfinite(x).all() for x in positions.values()):raise ValueError('Nonfinite measured fruit')
    # Cover every fruit before spending a second attempt on one missed group.
    sign=1 if order=='lowest-first' else -1
    eligible.sort(key=lambda f:(attempts.get(f['name'],0),sign*positions[f['name']][2],f['name']))
    if not eligible:return None
    individual=bool(single_fruit_when_remaining and len(remaining)<=single_fruit_when_remaining)
    group=None;alternative_parent=None
    for first_index,first in enumerate(eligible):
        candidate=[first]
        for other in ([] if individual else eligible[:first_index]+eligible[first_index+1:]):
            delta=positions[other['name']]-positions[first['name']]
            if attempts.get(other['name'],0)==attempts.get(first['name'],0) and abs(delta[2])<=.035 and np.linalg.norm(delta)<=.06:
                candidate.append(other);break
        key=tuple(sorted(f['name']for f in candidate))
        if key in rejected:
            if len(candidate)>1 and (first['name'],) not in rejected:
                alternative_parent=list(key);candidate=[first]
            else:continue
        group=candidate;break
    if group is None:return None
    result=dict(fruits=[f['name'] for f in group],
                measured_centers_world_m=[positions[f['name']].tolist() for f in group],
                measured_centroid_world_m=np.mean([positions[f['name']] for f in group],axis=0).tolist())
    if single_fruit_when_remaining:
        result['individual_sparse_aim']=dict(enabled=True,threshold=single_fruit_when_remaining,
            attached_fruit_count=len(remaining),selected_individually=individual,
            height_offset_changed=False,
            scope='Current exact truss only; measured center of selected attached fruit; entry yaw unchanged')
    if rejected:
        result['planning_rejection_alternative']=dict(parent_group=alternative_parent,
            centering_changed_to_individual=alternative_parent is not None,
            excluded_exact_groups=[list(g)for g in sorted(rejected)],
            scope='Alternatives for unchanged native planning boundary only; no physical attempt or detachment implied')
    return result


def select_near_scope_continuation(fruits,breaks,body_paths,poses,attempts,initial_truss_key,
                                  max_attempts=2,order='lowest-first',
                                  single_fruit_when_remaining=0,rejected_groups=()):
    """Opt-in initial-truss-first selection among already eligible active fruits.

    The caller provides the exact rail-side scope. No side policy, retry count,
    measured pose, joint state or rejection is changed here. When the initial
    truss has no selectable group, existing per-truss selection considers the
    other scoped trusses. Exhaustion is a finite search result, not impossible.
    """
    from row_harvest_targets import truss_key,select_scoped_group
    max_attempts=validate_max_fruit_attempts(max_attempts)
    single_fruit_when_remaining=validate_single_fruit_when_remaining(single_fruit_when_remaining)
    keys={truss_key(f['name'])for f in fruits}
    if initial_truss_key not in keys:
        raise ValueError('Initial truss must belong to the supplied exact eligible fruit scope')
    for field in ('name','path','joint'):
        if len({f[field]for f in fruits})!=len(fruits):raise ValueError('Duplicate scoped fruit '+field)
    rejected=validated_rejected_groups(fruits,rejected_groups)
    initial=[f for f in fruits if truss_key(f['name'])==initial_truss_key]
    if any(len({truss_key(n)for n in g})!=1 for g in rejected):
        raise ValueError('Rejected group crosses exact truss boundaries')
    initial_rejected=[g for g in rejected if all(truss_key(n)==initial_truss_key for n in g)]
    group=select_group(initial,breaks,body_paths,poses,attempts,max_attempts=max_attempts,order=order,
        single_fruit_when_remaining=single_fruit_when_remaining,rejected_groups=initial_rejected)
    initial_remaining=inventory(initial,breaks)['remaining']
    evidence=dict(initial_truss_key=initial_truss_key,initial_truss_remaining=initial_remaining,
        initial_truss_attempt_exhausted=[n for n in initial_remaining if attempts.get(n,0)>=max_attempts],
        initial_truss_rejected_exact_groups=[list(g)for g in sorted(initial_rejected)],
        max_attempts_per_fruit=max_attempts,eligible_truss_keys=sorted(keys),
        selection='Initial exact truss first, then existing finite selection within other eligible trusses',
        measured_positions_preserved=True,attempt_counts_preserved=True,physical_state_reset=False,
        training_eligible=False,harvest_success=None)
    if group is not None:
        group['truss_key']=initial_truss_key;reason='initial_truss_group_available'
    else:
        others=[f for f in fruits if truss_key(f['name'])!=initial_truss_key]
        other_rejected=[g for g in rejected if all(truss_key(n)!=initial_truss_key for n in g)]
        group=(select_scoped_group(others,breaks,body_paths,poses,attempts,max_attempts=max_attempts,
            order=order,single_fruit_when_remaining=single_fruit_when_remaining,
            rejected_groups=other_rejected)if others else None)
        reason=('initial_truss_all_detached' if not initial_remaining else
                'initial_truss_finite_attempt_or_rejection_exhaustion')
    evidence.update(selected_truss_key=None if group is None else group['truss_key'],
        switched_truss=group is not None and group['truss_key']!=initial_truss_key,
        reason=reason,selection_exhausted=group is None)
    if group is not None:group['near_scope_continuation']=evidence.copy()
    return group,evidence


def near_scope_truss_direction(group,all_fruits,breaks,body_paths,poses,robot_position_world,
                              verified_axes=None,min_attached_fruits=10):
    """Own-truss measured XY PCA; a different truss's retained yaw is never used.

    An initial dense fit is required for a previously unseen truss. Sparse
    remainders may retain only that same exact truss's verified axis. The caller
    records returned evidence by its truss_key for subsequent measured cycles.
    """
    from row_harvest_targets import truss_key,group_truss_fruits
    subset=group_truss_fruits(group,all_fruits);key=truss_key(subset[0]['name'])
    if group.get('truss_key',key)!=key:raise ValueError('Selected group truss key disagrees with exact fruit names')
    robot=np.asarray(robot_position_world,float)
    if robot.shape!=(3,) or not np.isfinite(robot).all():raise ValueError('Finite measured robot-side position required')
    attached=set(inventory(subset,breaks)['remaining']);paths=list(body_paths)
    centers=np.asarray([poses[paths.index(f['path']),:3]for f in subset if f['name']in attached],float).reshape(-1,3)
    if not len(centers) or not np.isfinite(centers).all():raise ValueError('Selected truss requires finite attached fruit positions')
    if any(n not in attached for n in group['fruits']):raise ValueError('Selected group includes a detached fruit')
    reference=centers.mean(0)-robot;reference[2]=0.
    axes={}if verified_axes is None else verified_axes
    if not isinstance(axes,dict):raise ValueError('Verified axes must be keyed by exact truss identity')
    previous=axes.get(key);fallback=None
    if previous is not None:
        names=previous.get('fruit_names')if isinstance(previous,dict)else None
        if (not isinstance(names,list) or not names or any(truss_key(n)!=key for n in names) or
                previous.get('truss_key',key)!=key or not set(names)<={f['name']for f in subset}):
            raise ValueError('Retained axis does not belong to this exact truss')
        fallback=previous['direction_world']
    axis=horizontal_truss_direction(subset,breaks,paths,poses,reference,
        fallback_direction=fallback,min_attached_fruits=min_attached_fruits)
    axis.update(truss_key=key,near_scope_axis=dict(selected_truss_key=key,
        source='Selected exact truss attached-fruit measured XY PCA',
        measured_robot_position_world_m=robot.tolist(),robot_to_attached_centroid_world=reference.tolist(),
        own_truss_verified_axis_available=previous is not None,
        other_truss_retained_axis_used=False,physical_state_reset=False))
    return axis


def measured_sequence_evidence(sequence,states,fruits,breaks,hz):
    """Verify global time and that each new aim used its saved measured state."""
    stamps=np.asarray(states['time']);dt=np.diff(stamps)
    if len(dt)==0 or not np.isfinite(stamps).all() or not np.allclose(dt,1/hz,atol=1e-10,rtol=0):
        raise ValueError('Whole-truss physics timestamps have a gap or reset')
    if not np.isfinite(states['poses']).all():raise ValueError('Nonfinite sequence pose')
    paths=list(states['body_paths']);by_name={f['name']:f for f in fruits}
    errors=[];authored_initial_offsets=[]
    for cycle in sequence['cycle_plans']:
        if 'measured_centers_world_m' not in cycle:continue
        actual=np.array([states['poses'][cycle['measured_frame_index'],paths.index(by_name[name]['path']),:3]
                         for name in cycle['fruits']])
        error=float(np.max(np.abs(actual-np.asarray(cycle['measured_centers_world_m']))))
        # The first group is selected before native warmup from authored GT.
        # Its historical field name says "measured", but the sequence records
        # that distinct source. Verify against the source, and report native
        # warmup/float conversion displacement separately. Later aims remain
        # subject to the unchanged exact measured-boundary check.
        authored_first=(cycle.get('index')==0 and cycle['measured_frame_index']==0 and
            (sequence.get('group_policy') or {}).get('schema')=='s300-harvest-group-policy-v1' and
            sequence.get('initial_aim_source')=='authored fruit positions; subsequent aims use measured boundary state')
        if authored_first:
            authored=np.asarray([by_name[name]['center'] for name in cycle['fruits']],float)
            stored=np.asarray(cycle['measured_centers_world_m'],float)
            if authored.shape!=stored.shape or not np.isfinite(authored).all() or not np.isfinite(stored).all() or np.max(np.abs(authored-stored))>1e-8:
                raise ValueError('Initial group position differs from authored source')
            authored_initial_offsets.append(error)
            continue
        if error>1e-8:raise ValueError('Selected fruit position differs from stored boundary state')
        errors.append(error)
    result=inventory(fruits,breaks)
    if result!=sequence['inventory']:raise ValueError('Sequence inventory differs from exact native joint events')
    return dict(**result,fruit_count=len(fruits),native_detached_count=len(result['detached']),
        timestamp_contiguous=True,max_dt_s=float(dt.max()),physics_steps=len(dt),
        measured_target_position_max_error_m=max(errors,default=0.),
        authored_initial_group_count=len(authored_initial_offsets),
        authored_initial_to_native_position_max_delta_m=max(authored_initial_offsets,default=0.),
        all_selected_truss_joints_broken=not result['remaining'],
        collection_verified=False,biological_calibration=False,training_eligible=False,
        scope='Continuous measured physics timestamps and exact native joint events; not collection evidence')
