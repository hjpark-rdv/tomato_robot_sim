"""Opt-in timing changes for an already planned, unchanged entry trajectory."""
import math
from numbers import Real


def retime_entry(phases, metadata, scale):
    """Lengthen only approach/withdraw, preserving all commands and belt timing.

    A scale of one returns the exact original objects. Other scales construct
    new phase tuples and metadata entries only where their duration changes;
    joint waypoint arrays, jaw gaps and belt velocities remain untouched.
    """
    if isinstance(scale,bool) or not isinstance(scale,Real) or not math.isfinite(scale) or not 1.<=scale<=8.:
        raise ValueError('Entry duration scale must be finite and within 1..8')
    if scale==1.:
        return phases,metadata
    rows=metadata.get('phases')
    if not phases or not isinstance(rows,list) or len(rows)!=len(phases):
        raise ValueError('Entry retiming needs matching executable and metadata phases')
    indices={name:[] for name in ('approach','withdraw')}
    for index,(phase,row) in enumerate(zip(phases,rows)):
        if len(phase)!=6 or not isinstance(row,dict):
            raise ValueError('Malformed executable/metadata phase')
        name,duration,_,_,_,_=phase
        if not isinstance(duration,Real) or not math.isfinite(duration) or duration<=0 or \
                row.get('name')!=name or row.get('duration_s')!=duration:
            raise ValueError('Executable/metadata phase duration or identity mismatch')
        if name in indices:indices[name].append(index)
    if any(len(found)!=1 for found in indices.values()):
        raise ValueError('Entry retiming requires exactly one approach and one withdraw')
    approach_index=indices['approach'][0]
    withdraw_index=indices['withdraw'][0]
    if withdraw_index<=approach_index:
        raise ValueError('Withdraw must follow approach')
    cycle=metadata.get('belt_cycle')
    if cycle is not None:
        if not isinstance(cycle,dict) or not isinstance(cycle.get('start_after_settle_s'),Real) or \
                not math.isfinite(cycle['start_after_settle_s']) or cycle['start_after_settle_s']<0:
            raise ValueError('Malformed belt-cycle start time')
    retimed=list(phases)
    retimed_rows=list(rows)
    changes=[]
    for index in (approach_index,withdraw_index):
        phase=phases[index]
        duration=float(phase[1]*scale)
        retimed[index]=(phase[0],duration,*phase[2:])
        retimed_rows[index]=dict(rows[index],duration_s=duration)
        changes.append(dict(phase=phase[0],original_duration_s=float(phase[1]),duration_s=duration))
    result=dict(metadata,phases=retimed_rows)
    approach_delta=retimed[approach_index][1]-phases[approach_index][1]
    if cycle is not None:
        result['belt_cycle']=dict(cycle,start_after_settle_s=cycle['start_after_settle_s']+approach_delta)
    result['entry_timing']=dict(duration_scale=float(scale),phases=changes,
        belt_window_start_shift_s=float(approach_delta),
        total_duration_added_s=float(sum(c['duration_s']-c['original_duration_s'] for c in changes)),
        joint_waypoints_unchanged=True,jaw_commands_unchanged=True,belt_commands_unchanged=True,
        scope='Timing-only approach/withdraw change; same trajectory, model and physical validity criteria')
    return retimed,result
