"""Explicit absolute depth selection after completed native harvest cycles.

Only the existing robot waypoint planner consumes this value.  This never
changes plant poses, drive properties, collisions, or the finite attempt budget.
"""
import math


def completed_native_cycles(records,truss_key=None):
    """Count completed physical timelines, not rejected or stopped proposals."""
    if truss_key is not None:
        from row_harvest_targets import truss_key as exact_truss_key
        if not isinstance(truss_key,str) or exact_truss_key(truss_key+'_Fruit_01')!=truss_key:
            raise ValueError('Exact selected truss key required for scoped completed cycles')
    completed=0
    for record in records:
        if not isinstance(record,dict) or record.get('status')!='motion_completed' or record.get('stop_reason') is not None:
            continue
        if truss_key is not None:
            fruits=record.get('fruits')
            if not isinstance(fruits,list) or not fruits or any(not isinstance(n,str) for n in fruits):
                continue
            try:keys={exact_truss_key(n)for n in fruits}
            except ValueError:continue
            if keys!={truss_key}:continue
            if record.get('truss_key',truss_key)!=truss_key:continue
        start=record.get('start_s');end=record.get('end_s');planned=record.get('planned_end_s')
        frames=(record.get('measured_frame_index'),record.get('end_frame_index'))
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in (start,end,planned)):
            continue
        if end<=start or end<planned-1e-8:
            continue
        if any(isinstance(x,bool) or not isinstance(x,int) for x in frames) or frames[1]<=frames[0]:
            continue
        completed+=1
    return completed


def grasp_depth_for_truss_cycle(base_depth_mm,later_depth_mm,records,truss_key):
    """A newly selected exact truss starts at base depth, despite other cycles."""
    count=completed_native_cycles(records,truss_key=truss_key)
    value,evidence=grasp_depth_for_cycle(base_depth_mm,later_depth_mm,count)
    evidence.update(selected_truss_key=truss_key,
        completed_cycle_count_scope='Exact selected truss only; other trusses, stopped/rejected and mixed-truss records excluded')
    return value,evidence


def grasp_depth_for_cycle(base_depth_mm,later_depth_mm,completed_cycles):
    """Return an absolute depth and evidence; the override never accumulates."""
    for value in ([base_depth_mm] if later_depth_mm is None else [base_depth_mm,later_depth_mm]):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 15<=value<=100:
            raise ValueError('Grasp depth must be finite within15..100mm')
    if isinstance(completed_cycles,bool) or not isinstance(completed_cycles,int) or completed_cycles<0:
        raise ValueError('Completed native cycle count must be a nonnegative integer')
    active=later_depth_mm is not None and completed_cycles>=1
    value=later_depth_mm if active else base_depth_mm
    return float(value),dict(
        enabled=later_depth_mm is not None,base_depth_mm=float(base_depth_mm),
        later_depth_mm=None if later_depth_mm is None else float(later_depth_mm),
        completed_native_cycles=completed_cycles,effective_grasp_depth_mm=float(value),
        source='later_depth_after_completed_native_cycle' if active else 'global_grasp_depth',
        cumulative_translation=False,
        scope='Existing waypoint/IK/full-FCL replanning only; physical model and native detach criteria unchanged')
