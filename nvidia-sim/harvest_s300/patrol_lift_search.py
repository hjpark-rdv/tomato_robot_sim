"""Bounded camera-height proposals using observed depth and actual lift state.

These are ordinary scan commands. The existing native planner, full-path
audits, lift actuator and a fresh camera exposure must execute each proposal.
"""
import copy

import numpy as np


def propose_lift_scan(item, *, station_m, measured_lift_m, limits_m,
                      after_planning_failure=False):
    """Return at most three distinct measured-view height proposals per group."""
    if not item.get('visible_now') or measured_lift_m is None:
        return None
    limits=np.asarray(limits_m,float)
    current=float(measured_lift_m)
    if (limits.shape!=(2,) or not np.isfinite(np.r_[limits,current,station_m]).all()
            or limits[0]>=limits[1] or not limits[0]<=current<=limits[1]):
        raise ValueError('Finite measured lift state and actual ordered limits required')
    suggested=item.get('suggested_lift_m')
    if suggested is None or not np.isfinite(suggested):
        return None
    history=item.get('camera_lift_search',[])
    if len(history)>=3:
        return None
    baseline=float(history[0]['source_measured_lift_m']) if history else current
    suggested_valid=limits[0]<=suggested<=limits[1]
    if not after_planning_failure and suggested_valid and abs(suggested-current)<=.06:
        return None
    attempted=[current]
    attempted.extend(float(row['goal_lift_m'])for row in history)
    attempted.extend(float(row['source_measured_lift_m'])for row in history)
    # An out-of-range alignment estimate is evidence, not a valid clamped
    # harvest pose. A distinct in-range edge view may expose another group.
    margin=min(.03,float(limits[1]-limits[0])/4)
    candidates=[]
    if suggested_valid:
        candidates.append((float(suggested),'observed_depth_alignment_proposal'))
    else:
        edge=limits[0]+margin if suggested<limits[0] else limits[1]-margin
        candidates.append((float(edge),'different_valid_view_for_out_of_range_alignment'))
    if after_planning_failure:
        candidates.extend((baseline+delta,'bounded_alternative_view_after_finite_planning_failure')
                          for delta in (.10,-.10,.20,-.20))
    for goal,reason in candidates:
        if not limits[0]+margin<=goal<=limits[1]-margin:
            continue
        if min(abs(goal-value)for value in attempted)<.04:
            continue
        proof=dict(goal_lift_m=goal,source_measured_lift_m=current,
            actual_lift_limits_m=limits.tolist(),observed_suggested_lift_m=float(suggested),
            suggested_alignment_within_limits=bool(suggested_valid),reason=reason,
            source_observation=copy.deepcopy(item.get('last_observation')),
            source_observation_index=item.get('last_observation_index'),
            source_station_m=float(station_m),proposal_index=len(history),
            physical_pose_changed=False,actual_execution_and_new_observation_required=True,
            uses_hidden_gt_center=False,training_eligible=False,harvest_success=None)
        return dict(action='lift_scan',reason=reason,target_key=item['key'],
            target=copy.deepcopy(item),current_station_y_m=float(station_m),
            next_station_y_m=float(station_m),next_lift_m=goal,lift_proposal=proof,
            physics_execution_required=True,requires_camera_observation_after_motion=True)
    return None


def remember_lift_proposal(item, decision):
    """Persist the finite search budget in the existing camera queue."""
    proof=copy.deepcopy(decision['lift_proposal'])
    history=item.setdefault('camera_lift_search',[])
    if proof['proposal_index']!=len(history) or len(history)>=3:
        raise ValueError('Camera lift proposal is stale or exceeds the finite budget')
    history.append(proof)


def lift_bounds_for_assembly(assembly):
    """Read the existing robot's physical limits; never assume a new stroke."""
    from pathlib import Path
    from pxr import Usd
    from motion_kinematics import ToolKinematics
    return ToolKinematics(Usd.Stage.Open(str(Path(assembly)/'scene.usda'))).bounds[:,0].tolist()
