"""Recover the last verified horizontal entry axis without failed-plan drift."""
import copy
import json
from pathlib import Path

import numpy as np

from compact_motion_assets import file_sha256


def _axis(value):
    if not isinstance(value,dict) or 'direction_world' not in value:
        raise ValueError('Recorded resume axis has no world direction')
    direction=np.asarray(value['direction_world'],dtype=float)
    if (direction.shape!=(3,) or not np.isfinite(direction).all() or
            abs(direction[2])>1e-12 or abs(np.linalg.norm(direction)-1.)>1e-6):
        raise ValueError('Recorded resume axis must be finite unit XY direction')
    yaw=float(np.degrees(np.arctan2(direction[1],direction[0])))
    if 'yaw_world_deg' in value:
        recorded=float(value['yaw_world_deg'])
        delta=(recorded-yaw+180.)%360.-180.
        if not np.isfinite(recorded) or abs(delta)>1e-4:
            raise ValueError('Recorded resume axis yaw/direction disagree')
    result=copy.deepcopy(value);result['direction_world']=direction.tolist()
    result.setdefault('yaw_world_deg',yaw)
    return result


def resolve_resume_axis(context,checkpoint_directory):
    """Return (axis, provenance) from measured-cycle metadata or audited plan.

    Rejected/stopped proposals never establish the previously used direction.
    Legacy checkpoints without an axis can use their initial plan only when a
    complete self-collision audit binds its exact bytes to the same scene SHA.
    The caller already loaded and hash-validated this native checkpoint.
    """
    metadata=context['metadata'];sequence=metadata.get('sequence',{})
    checkpoint=Path(checkpoint_directory).resolve()
    common=dict(checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        scene_sha256=metadata['scene_sha256'],failed_proposals_used=False,
        scope='Recorded verified horizontal entry direction; no random/manual yaw substitution')
    for row in reversed(sequence.get('cycle_plans',[])):
        if row.get('status') not in ('motion_completed','executing') or row.get('truss_axis') is None:
            continue
        result=_axis(row['truss_axis'])
        return result,dict(common,source='completed_or_executing_cycle_metadata',
            cycle_index=row.get('index'),cycle_status=row['status'],cycle_plan=row.get('plan'))
    if sequence.get('initial_axis') is not None:
        return _axis(sequence['initial_axis']),dict(common,source='checkpoint_sequence_initial_axis')
    trial=checkpoint.parent.parent;plan_path=trial/'plan.json';audit_path=trial/'self_collision_preflight.json'
    if not plan_path.is_file() or not audit_path.is_file():
        raise ValueError('Resume axis missing: no verified cycle axis, initial axis or audited legacy first plan')
    audit=json.loads(audit_path.read_text())
    if audit.get('passed') is not True or audit.get('complete') is not True:
        raise ValueError('Legacy resume axis plan has no complete passing self-collision audit')
    if audit.get('scene_sha256')!=metadata['scene_sha256']:
        raise ValueError('Legacy resume axis audit scene SHA mismatch')
    plan_sha=file_sha256(plan_path)
    if audit.get('input_sha256')!=plan_sha:
        raise ValueError('Legacy resume axis audit input SHA mismatch')
    plan=json.loads(plan_path.read_text())
    value=plan.get('truss_axis')
    if value is None:
        value=dict(direction_world=plan.get('approach_direction_world'),method='audited legacy initial plan entry direction')
    result=_axis(value)
    if plan.get('approach_direction_world') is not None:
        entry=_axis(dict(direction_world=plan['approach_direction_world']))
        if not np.allclose(entry['direction_world'],result['direction_world'],rtol=0.,atol=1e-6):
            raise ValueError('Legacy resume truss axis differs from its actual entry direction')
    return result,dict(common,source='legacy_initial_plan_with_bound_complete_self_audit',
        plan=str(plan_path),plan_sha256=plan_sha,audit=str(audit_path),audit_sha256=file_sha256(audit_path),
        audit_passed=True,audit_complete=True)
