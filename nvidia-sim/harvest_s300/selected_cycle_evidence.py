"""Exact selected-fruit cycle evidence, separate from legacy manifest target.

Read-only reporting. Existing native_target_detached/target_displacement fields
retain their original manifest target meaning; they never substitute for the
fruit or group actually selected by a sequential/resumed manipulation cycle.
"""
import math
import numpy as np


MANIPULATION_PHASES=frozenset(('approach','entry_hold','insert','infeed','close','belt_start_open',
    'belt_spin','belt_finish','cycle_open','cycle_open_hold','cycle_reclose','grip_hold','twist'))


def build_selected_cycle_evidence(manifest,sequence,breaks,body_paths=None,poses=None,times=None,events=(),*,legacy_summary=None,measured_phases=None):
    root=manifest['plant']['root'].rstrip('/')
    fruits={}
    for f in manifest['plant']['fruits']:
        name=f['name'];body=f['path'];joint=f['joint']
        if (name in fruits or body!=root+'/'+name or joint!=root+'/HarvestJoints/'+name):
            raise ValueError('Exact cycle fruit/body/native joint identity mismatch')
        fruits[name]=dict(name=name,body=body,collider=body+'/Collider',native_joint=joint)
    legacy_name=manifest.get('target',{}).get('name');legacy=fruits.get(legacy_name)
    if legacy_name and legacy is None:raise ValueError('Legacy manifest target is absent from exact fruit inventory')
    arrays=body_paths is not None or poses is not None or times is not None
    if arrays:
        if body_paths is None or poses is None or times is None:raise ValueError('All measured cycle arrays are required together')
        names=list(map(str,body_paths));p=np.asarray(poses,float);t=np.asarray(times,float)
        if (len(set(names))!=len(names) or p.shape!=(len(t),len(names),7) or not len(t) or
                t.ndim!=1 or not np.isfinite(p).all() or not np.isfinite(t).all() or
                np.any(np.diff(t)<=0)):
            raise ValueError('Finite increasing measured cycle trace/exact body mapping required')
        ids={name:i for i,name in enumerate(names)}
    if measured_phases is not None:
        if not arrays:raise ValueError('Measured cycle phases require the exact measured trace arrays')
        measured_phases=np.asarray(measured_phases)
        if measured_phases.shape!=t.shape or any(not isinstance(x,(str,np.str_)) or not x for x in measured_phases):
            raise ValueError('Measured cycle phase/timestamp mapping mismatch')
    joint_events={f['native_joint']:[] for f in fruits.values()}
    for event in breaks:
        if event.get('joint') not in joint_events:continue
        stamp=event.get('t')
        if isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<0:
            raise ValueError('Cycle native break timestamp is invalid')
        joint_events[event['joint']].append(dict(event))
    cycles=[]
    for cycle in (sequence or {}).get('cycle_plans',[]):
        selected=cycle.get('fruits',[])
        if not selected or len(set(selected))!=len(selected) or any(name not in fruits for name in selected):
            raise ValueError('Exact planned/selected cycle fruit mapping mismatch')
        status=cycle.get('status');recorded=status in ('motion_completed','stopped')
        start=cycle.get('start_s');end=cycle.get('end_s')
        if recorded and (not isinstance(start,(int,float)) or not isinstance(end,(int,float)) or
                not math.isfinite(start) or not math.isfinite(end) or start<0 or end<0 or
                (status=='motion_completed' and end<start)):
            raise ValueError('Recorded cycle requires its actual finite start/end timestamps')
        first=last=None;baseline=start
        if arrays and recorded:
            first=cycle.get('measured_frame_index');last=cycle.get('end_frame_index')
            if (isinstance(first,bool) or isinstance(last,bool) or not isinstance(first,int) or
                    not isinstance(last,int) or not 0<=first<=last<len(t) or abs(t[last]-end)>1e-8):
                raise ValueError('Recorded cycle measured frame/timestamp mapping mismatch')
            baseline=float(t[first])
        selected_rows=[]
        for name in selected:
            row=dict(fruits[name]);evidence=joint_events[row['native_joint']]
            before=[e for e in evidence if baseline is not None and e['t']<=baseline]
            through=[e for e in evidence if end is not None and e['t']<=end]
            fresh=[e for e in through if start is not None and e['t']>=start and
                   e.get('native_new_event') is not False and e.get('type')!='imported_native_prior']
            row.update(native_detached_at_cycle_start=bool(before) if recorded else None,
                native_detached_at_cycle_end=bool(through) if recorded else None,
                native_break_evidence_through_cycle_end=through if recorded else [],
                new_native_break_events_during_cycle=fresh if recorded else [],
                displacement_max_from_measured_cycle_baseline_m=None,
                displacement_final_from_measured_cycle_baseline_m=None,
                max_single_contact_point_force_n=None)
            if arrays and recorded:
                if row['body'] not in ids:raise ValueError('Selected fruit lacks its exact measured native body')
                xyz=p[first:last+1,ids[row['body']],:3];delta=np.linalg.norm(xyz-xyz[0],axis=1)
                row.update(displacement_max_from_measured_cycle_baseline_m=float(delta.max()),
                           displacement_final_from_measured_cycle_baseline_m=float(delta[-1]))
            if recorded:
                contact=[]
                for event in events:
                    stamp=event.get('t');force=event.get('force_n')
                    if (isinstance(stamp,(int,float)) and math.isfinite(stamp) and baseline<=stamp<=end and
                            isinstance(force,(int,float)) and math.isfinite(force) and
                            any(event.get(side)==row['body'] or str(event.get(side,'')).startswith(row['body']+'/') for side in ('a','b'))):
                        contact.append(float(force))
                if events:row['max_single_contact_point_force_n']=max(contact,default=0.)
            selected_rows.append(row)
        actual_phases=[] if measured_phases is None or not recorded else list(dict.fromkeys(map(str,measured_phases[first:last+1])))
        started=(bool(set(actual_phases)&MANIPULATION_PHASES) if measured_phases is not None and recorded
                 else bool(recorded and end>=start))
        cycles.append(dict(index=cycle.get('index'),status=status,planned_start_s=start,actual_end_s=end,
            recorded_physical_cycle=recorded,planned_manipulation_started=started,
            stopped_before_planned_manipulation=bool(recorded and status=='stopped' and not started),
            manipulation_start_evidence=('actual recorded phase names' if measured_phases is not None else
                'legacy cycle-start timestamp proxy; preparation may be included'),
            actual_recorded_phases=actual_phases,
            measured_cycle_baseline_time_s=baseline if arrays and recorded else None,
            selected_fruits=selected_rows,selected_names=selected,actual_trace_measurements_available=bool(arrays and recorded),
            legacy_manifest_target_is_selected=legacy_name in selected,
            all_selected_native_joints_detached_at_cycle_end=(all(row['native_detached_at_cycle_end'] for row in selected_rows) if recorded else None)))
    summary=legacy_summary or {}
    return dict(schema='exact-selected-fruit-cycle-evidence-v1',legacy_manifest_target=legacy,
        legacy_summary_fields=dict(native_target_detached=summary.get('native_target_detached'),
            target_displacement_max_m=summary.get('target_displacement_max_m'),
            target_displacement_final_m=summary.get('target_displacement_final_m'),target_force_max_n=summary.get('target_force_max_n')),
        legacy_field_scope='Existing target fields describe only the original manifest target, including imported prior release; they do not describe a different selected fruit or prove this cycle detached it',
        cycles=cycles,physical_response_changed=False,training_eligible=False,collection_verified=False,harvest_success=None,
        measurement_scope='Exact native joint events and optional stored measured body positions; displacement includes any later falling motion and is not an attached-fruit damage metric')
