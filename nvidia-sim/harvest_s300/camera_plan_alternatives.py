"""Finite existing camera heading/IK-branch alternatives; no execution loop.

Headings remain the inner round-robin order for each seed. This module does
not change a planner, motion budget, geometry, simulator or default seed.
"""
import copy
import hashlib
import json
from numbers import Integral, Real
from pathlib import Path

import numpy as np


def _seed(value, *, auto=False):
    if value is None and auto:
        return None
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or not 0 <= value <= 8:
        raise ValueError('Existing staging seed must be an integer0..8'+(' orNoneauto' if auto else ''))
    return int(value)


def _heading(value):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or
            not np.isfinite(value) or not -180. <= value <= 180.):
        raise ValueError('Finite existing camera heading within+/-180degrees required')
    return float(value)


def camera_plan_alternatives(headings, configured_seed_index, *, seed_indices=None):
    """Enumerate finite proposals, preserving the configured single-seed default.

    An explicit seed list is opt-in, unique and limited to the nine existing
    planner seeds. It cannot contain auto/None. Runtime proposal/physics caps
    remain the caller's responsibility and are never increased here.
    """
    configured = _seed(configured_seed_index, auto=True)
    angles = [_heading(value) for value in headings]
    if not angles or len(set(angles)) != len(angles):
        raise ValueError('Nonempty unique existing camera headings required')
    seeds = [configured] if seed_indices is None else [_seed(value) for value in seed_indices]
    if not seeds or len(seeds) > 9 or len(set(seeds)) != len(seeds):
        raise ValueError('Explicit staging seed list must contain1..9unique existing seeds')
    return [dict(index=index, heading_offset_deg=heading, seed_index=seed,
                 seed_selection='configured' if seed_indices is None else 'explicit_alternative')
            for index, (seed, heading) in enumerate((seed, heading) for seed in seeds for heading in angles)]


def alternative_key(alternative):
    """Keep rejection/exhaustion state distinct for every heading and IK seed."""
    return (_heading(alternative['heading_offset_deg']), _seed(alternative['seed_index'], auto=True))


def rejection_sets(alternatives, initial_groups=()):
    """Independent copies; rejecting a group on one branch cannot poison another."""
    keys = [alternative_key(item) for item in alternatives]
    if len(keys) != len(set(keys)):
        raise ValueError('Duplicate camera alternative key')
    common = {tuple(group) for group in initial_groups}
    return {key: common.copy() for key in keys}


def selected_seed_evidence(alternative, planning):
    """Record the actual existing planner branch, distinguishing auto from seed0."""
    heading, requested = alternative_key(alternative)
    selected = _seed(planning.get('selected_seed_index'))
    if requested is not None and requested != selected:
        raise ValueError('Actual selected staging seed differs from requested camera alternative')
    return dict(camera_heading_offset_deg=heading,
        requested_staging_seed_index=requested, selected_staging_seed_index=selected,
        staging_seed_selection='auto' if requested is None else 'explicit',
        alternative_key=[heading, requested], planner_and_physics_validation_required=True)


def same_group_auto_fallbacks(alternative,planning,group,*,plan_path,scene_sha256,
                             self_audit_path,environment_audit_path,
                             explicit_seed_indices=None,tested=()):
    """Retry seeds5/8 only after a hash-bound auto-plan geometric rejection.

    These are same-observation, same-heading proposals at an unchanged physics
    boundary.  The caller retains its original total proposal cap and must run
    the existing complete audits before execution.  Missing/inconclusive
    reports, IK errors and explicit-seed modes cannot enable this fallback.
    """
    heading,requested=alternative_key(alternative)
    if requested is not None or explicit_seed_indices is not None:return []
    if not isinstance(scene_sha256,str) or len(scene_sha256)!=64:return []
    try:selected=_seed(planning.get('selected_seed_index'))
    except (AttributeError,ValueError):return []
    fruits=tuple(group.get('fruits',()))
    if not fruits:return []
    plan=Path(plan_path)
    if not plan.is_file():return []
    plan_hash=hashlib.sha256(plan.read_bytes()).hexdigest();proof=None
    for kind,path in [('self',self_audit_path),('environment',environment_audit_path)]:
        path=Path(path)
        if not path.is_file():continue
        try:
            data=path.read_bytes();report=json.loads(data)
            if not isinstance(report,dict) or report.get('passed')is not False or report.get('error'):continue
            if kind=='self':
                bound=(report.get('input_sha256')==plan_hash and report.get('scene_sha256')==scene_sha256)
                hit=any(isinstance(row,dict) and isinstance(row.get('hits'),list) and row['hits']
                    for row in report.get('phases',[]) if isinstance(report.get('phases'),list))
            else:
                hashes=report.get('source_sha256',{})
                bound=(hashes.get('plan')==plan_hash and hashes.get('scene')==scene_sha256)
                hit=(report.get('status')=='blocked' and bool(report.get('hard_hits')))
            if bound and hit:
                proof=dict(kind=kind,path=str(path),sha256=hashlib.sha256(data).hexdigest(),
                    plan_sha256=plan_hash,scene_sha256=scene_sha256,
                    complete=report.get('complete'),geometric_blocking_witness=True,
                    unexamined_suffix_is_clear=False)
                break
        except (OSError,ValueError,TypeError,AttributeError):continue
    if proof is None:return []
    tried=set(tested);result=[]
    for seed in (5,8):
        if seed==selected or (fruits,heading,seed) in tried:continue
        result.append(dict(index=alternative.get('index'),heading_offset_deg=heading,seed_index=seed,
            seed_selection='auto_geometric_rejection_fallback',same_group=copy.deepcopy(group),
            fallback_evidence=dict(rejected_auto_selected_seed_index=selected,
                parent_alternative_key=[heading,None],exact_fruit_group=list(fruits),audit=copy.deepcopy(proof),
                same_measured_boundary=True,heading_unchanged=True,physics_executed=False,
                proposal_budget_increased=False,requires_existing_ik_and_full_audits=True)))
    return result
