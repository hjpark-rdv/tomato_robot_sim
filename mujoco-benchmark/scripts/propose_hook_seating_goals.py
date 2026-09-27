"""Export SIM-GT ring-centre goals from an existing run, WITHOUT moving the robot.

The output is NOT a current action14 dataset, executable trace, or success label.
A downstream offline planner must connect goals through verified SE(3) paths.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

from hook_seating_geometry import Capsule, aligned_rotation, seating_geometry, seating_goal


def hash_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def generate_goals(targets, rear_wires, nominal_position, nominal_rotation, *, ring_radius,
                   hook_ring_offset, surface_gap_m, max_surface_gap_m,
                   tilts_deg, fractions, include_axis_aligned=False, max_goals=32):
    """Deterministic pose seeds. Ranking is proximity, NOT success probability."""
    if not targets or not rear_wires:
        raise ValueError('Need actual target and rear-wire capsule inventories')
    if len({t.name for t in targets}) != len(targets):
        raise ValueError('Duplicate target identity')
    p = np.asarray(nominal_position, dtype=float)
    offset = np.asarray(hook_ring_offset, dtype=float)
    if p.shape != (3,) or offset.shape != (3,) or not np.isfinite([p,offset]).all():
        raise ValueError('Invalid ring position or hook-body offset')
    if (not tilts_deg or not fractions or not np.isfinite(tilts_deg).all()
            or not np.isfinite(fractions).all() or any(abs(t)>90 for t in tilts_deg)
            or any(not 0 < f < 1 for f in fractions)):
        raise ValueError('Use finite tilts in [-90,90] and interior fractions')
    if (not math.isfinite(surface_gap_m) or not math.isfinite(max_surface_gap_m)
            or not 0 <= surface_gap_m <= max_surface_gap_m):
        raise ValueError('Need 0 <= requested gap <= geometric screening gap')
    if isinstance(max_goals,bool) or not isinstance(max_goals,int) or max_goals<1:
        raise ValueError('max_goals must be a positive integer')
    work = len(targets)*len(rear_wires)*len(fractions)*(len(tilts_deg)**2+int(include_axis_aligned))
    if work > 50000:
        raise ValueError('Explicit proposal budget exceeded (50000)')
    accepted, rejected = [], Counter()
    attempted = 0
    for target in targets:
        rotations = [(f'local_x_{x:g}_z_{z:g}', nominal_rotation*Rotation.from_euler('xz',[x,z],degrees=True))
                     for x,z in itertools.product(tilts_deg, repeat=2)]
        if include_axis_aligned:
            rotations.append(('normal_aligned_to_SIM_GT_capsule',aligned_rotation(target,nominal_rotation)))
        for (label,rot), fraction,wire in itertools.product(rotations,fractions,rear_wires):
            attempted += 1
            try:
                goal=seating_goal(target,wire,rot,fraction=fraction,surface_gap_m=surface_gap_m)
                metric=seating_geometry(target,goal['ring_position_xyz'],rot,rear_wires,
                                         ring_radius=ring_radius,max_surface_gap_m=max_surface_gap_m)
            except ValueError as error:
                rejected['invalid_proposal: '+str(error)] += 1
                continue
            if not metric['geometric_seating_candidate']:
                for reason in metric['reasons']: rejected[reason] += 1
                continue
            pos=np.array(goal['ring_position_xyz'])
            goal.update(geometry=metric,rotation_seed=label,
                        position_delta_from_baseline_m=float(np.linalg.norm(pos-p)),
                        rotation_delta_from_baseline_rad=float((rot*nominal_rotation.inv()).magnitude()),
                        hook_body_position_xyz=(pos-rot.apply(offset)).tolist())
            accepted.append(goal)
    accepted.sort(key=lambda g:(g['rotation_delta_from_baseline_rad'],g['position_delta_from_baseline_m'],
                               g['target_geom'],g['rear_wire_geom'],g['target_fraction']))
    # Keep tilted/axis-aligned families alive: nearest-only truncation can
    # otherwise fill every slot with nominal-orientation goals.
    groups={}
    for goal in accepted:
        groups.setdefault((goal['target_geom'],goal['rotation_seed']),[]).append(goal)
    goals=[]
    depth=0
    while len(goals)<min(max_goals,len(accepted)):
        for family in groups.values():
            if depth<len(family) and len(goals)<max_goals:goals.append(family[depth])
        depth+=1
    for i,g in enumerate(goals):g['goal_id']=f'seat_goal_{i:04d}'
    return dict(schema='hook_seating_goals_v1',attempted=attempted,accepted_before_limit=len(accepted),
                rejected_proposal_count=attempted-len(accepted),
                exported_count=len(goals),omitted_by_limit=max(0,len(accepted)-max_goals),
                rejection_condition_counts=dict(rejected),goals=goals,
                accepted_pose_families=len(groups),exported_pose_families=len({(g['target_geom'],g['rotation_seed']) for g in goals}),
                target_capsules_world=[t.json() for t in targets],
                rear_wire_capsules_ring=[w.json() for w in rear_wires],
                search=dict(tilts_deg=list(tilts_deg),fractions=list(fractions),
                            include_axis_aligned=include_axis_aligned,surface_gap_m=surface_gap_m,
                            max_surface_gap_m=max_surface_gap_m,ring_radius_m=ring_radius),
                position_frame='world; ring_position_xyz is RING centre, NOT Hook body origin',
                rotation_frame='world_from_hook; quaternion xyzw; local x/z tilt composition',
                ranking='round-robin target/rotation families; nearest pose within family; not success probability',
                requires=['whole_robot_IK','all_phase_environment_check','forward_physics_execution',
                          'target_identity_contact','retention_controls','non_target_contact_audit'],
                diagnostic_only=True,offline_teacher_goal=True,physics_executed=False,
                training_eligible=False,hook_success=None,
                scope='SIM-GT terminal poses only; no collision-free approach or guaranteed hook')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--candidate',default='predicted_00000')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--surface-gap-m',type=float,required=True,
                        help='Explicit geometric goal gap, not a safety/contact threshold')
    parser.add_argument('--max-seating-gap-m',type=float,default=.0015,
                        help='Geometry-only proposal limit; legacy diagnostic default, not real safety')
    parser.add_argument('--tilt-deg',type=float,nargs='+',default=[-15.,0.,15.])
    parser.add_argument('--fraction',type=float,nargs='+',default=[.5])
    parser.add_argument('--include-axis-aligned',action='store_true')
    parser.add_argument('--max-goals',type=int,default=32)
    args=parser.parse_args()
    root,out=args.run.resolve(),args.output.resolve()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.candidate):parser.error('Invalid candidate ID')
    if out==root or root in out.parents or out.exists():parser.error('Use a NEW output outside the source run')
    # Reuse the current native asset loader and exact target/rear identification.
    from diagnose_contact_timing import load_engine
    from hook_retention_diagnostic import HookProbe,capsule_endpoints
    from geometry import RING_RADIUS
    from suite import RING
    e,rows,plan=load_engine(root,args.candidate)
    e.reset()
    initial_qpos=e.data.qpos.copy();initial_qvel=e.data.qvel.copy()
    probe=HookProbe(e,hold_start=plan['seconds'])
    centre,orientation=probe.frame(e.data)
    targets=[Capsule(e.model.geom(g).name,*capsule_endpoints(e.model,e.data,g)) for g in probe.target_ids]
    rear=[]
    for g in probe.rear_ids:
        a,b,r=capsule_endpoints(e.model,e.data,g)
        rear.append(Capsule(e.model.geom(g).name,orientation.inv().apply(a-centre),orientation.inv().apply(b-centre),r))
    last=plan['waypoints'][-1]
    result=generate_goals(targets,rear,last['position_xyz'],Rotation.from_quat(last['orientation_xyzw']),
                          ring_radius=RING_RADIUS,hook_ring_offset=RING,surface_gap_m=args.surface_gap_m,
                          max_surface_gap_m=args.max_seating_gap_m,tilts_deg=args.tilt_deg,
                          fractions=args.fraction,include_axis_aligned=args.include_axis_aligned,max_goals=args.max_goals)
    if not np.array_equal(initial_qpos,e.data.qpos) or not np.array_equal(initial_qvel,e.data.qvel):
        raise RuntimeError('Goal generation changed engine state')
    paths=[root/'manifest.json',root/'replay_assets/model.mjb',root/'replay_assets/reference.json',
           root/'replay_assets/initial_trace.json',root/'candidates'/args.candidate/'plan.json',
           root/'candidates'/args.candidate/'trace.json']
    result.update(source_run=str(root),source_candidate=args.candidate,target=e.target,
                  source_sha256={str(p.relative_to(root)):hash_file(p) for p in paths},
                  proposal_code_sha256={p:hash_file(Path(__file__).with_name(p)) for p in
                                        ['hook_seating_geometry.py','propose_hook_seating_goals.py']},
                  initial_state_unchanged=True)
    out.mkdir(parents=True,exist_ok=False)
    (out/'seating_goals.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ['attempted','accepted_before_limit','exported_count','omitted_by_limit']}))
    print('GEOMETRIC GOALS ONLY:',out/'seating_goals.json')
    return 0 if result['goals'] else 2


if __name__=='__main__':
    raise SystemExit(main())
