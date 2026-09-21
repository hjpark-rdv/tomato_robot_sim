"""Dataset design and calibrated image crops; independent of Isaac imports."""
import copy
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from pose_candidates import base_candidate, candidate_waypoints


def write_json(path, value):
    """Atomic result publication so a stopped run never resumes partial JSON."""
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def throughput_estimate(requested, completed, session_completed, initialization_s, rollout_s, truncated=False):
    """Estimate from this invocation only; debug prefixes cannot predict full trials."""
    per_candidate = rollout_s / session_completed if session_completed > 0 and not truncated else None
    return dict(requested_candidates=requested, completed_candidates=completed,
                pending_candidates=max(0, requested-completed), session_completed_candidates=session_completed,
                initialization_wall_s=initialization_s, rollout_wall_s=rollout_s,
                estimated_remaining_minutes=per_candidate*max(0, requested-completed)/60 if per_candidate is not None else None,
                estimated_total_minutes=(initialization_s+per_candidate*requested)/60 if per_candidate is not None else None,
                estimate_basis='measured full candidate throughput including planning/reset; candidate lengths vary')

BOUNDS = {
    'azimuth_deg': (-45., 45.), 'elevation_deg': (-15., 15.),
    'roll_deg': (-20., 20.), 'pitch_deg': (-15., 15.),
    'offset_x_m': (-.015, .015), 'offset_y_m': (-.015, .015), 'offset_z_m': (-.015, .015),
    'pre_hook_distance_m': (.14, .20), 'insertion_distance_m': (.002, .014),
    'lift_offset_m': (-.010, .010),
}


def candidates(count, seed=42):
    if count < 1:
        raise ValueError('Candidate count must be positive')
    # Nominal is a previous attempt, not an asserted successful trajectory.
    nominal = dict(base_candidate(), insertion_distance_m=.004, lift_offset_m=0.)
    result = [nominal]
    if count > 1:
        sampler = qmc.Sobol(d=len(BOUNDS), scramble=True, seed=seed)
        unit = sampler.random_base2(int(math.ceil(math.log2(count - 1))))[:count - 1]
        samples = qmc.scale(unit, [b[0] for b in BOUNDS.values()], [b[1] for b in BOUNDS.values()])
        for values in samples:
            p = copy.deepcopy(nominal)
            p.update(dict(zip(BOUNDS, map(float, values))))
            p['offset_xyz_m'] = [p.pop('offset_'+a+'_m') for a in 'xyz']
            result.append(p)
    for i, p in enumerate(result):
        p.update(candidate_id=f'candidate_{i:05d}', stage='coarse', parent_id=None)
    return result


def waypoints(center, neck, params, radius=None):
    if params.get('trajectory_mode') == 'staged6d':
        from trajectory_search import waypoints as staged_waypoints
        return staged_waypoints(center, neck, params, radius)
    rotation, poses, direction = candidate_waypoints(center, neck, params)
    outward = -direction
    insertion = params['insertion_distance_m'] - .004
    delta = Rotation.from_euler('z', params['azimuth_deg'], degrees=True)
    delta = delta * Rotation.from_euler('y', -params['elevation_deg'], degrees=True)
    lift = delta.apply([0., 0., params['lift_offset_m']])
    return rotation, [(name, p + (outward * insertion if name != 'pull' else 0.)
                       + (lift if name in ('rise', 'pull') else 0.)) for name, p in poses], direction


def crop_rgbd(rgb, depth, K, world_from_camera, target_world, extent_m=.20):
    """Square context window projected at target depth. No segmentation or resize.

    Preserve hidden targets; never use depth or visibility to manufacture a
    target-centred clean image. Truncation and depth validity are reported.
    """
    rgb, depth, K = np.asarray(rgb), np.asarray(depth), np.asarray(K, float)
    if rgb.shape[:2] != depth.shape or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('RGB and depth dimensions must match')
    if extent_m <= 0:
        raise ValueError('Crop extent must be positive')
    pose = np.asarray(world_from_camera)
    target = (np.asarray(target_world) - pose[:3, 3]) @ pose[:3, :3]
    if target[2] <= 0:
        raise ValueError('Target centre behind camera; change observation pose')
    uv = K @ target; uv = uv[:2] / uv[2]
    h, w = depth.shape
    if not (0 <= uv[0] < w and 0 <= uv[1] < h):
        raise ValueError('Target centre outside frame; change observation pose')
    half = np.array([K[0, 0], K[1, 1]]) * extent_m / target[2] / 2
    requested = [int(np.floor(uv[0]-half[0])), int(np.floor(uv[1]-half[1])),
                 int(np.ceil(uv[0]+half[0])), int(np.ceil(uv[1]+half[1]))]
    x0, y0, x1, y1 = requested
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
    local_K = K.copy(); local_K[0, 2] -= x0; local_K[1, 2] -= y0
    valid = np.isfinite(depth) & (depth > 0)
    return (rgb[y0:y1, x0:x1].copy(), depth[y0:y1, x0:x1].copy(), valid,
            dict(bounds_xyxy=[x0,y0,x1,y1], requested_xyxy=requested,
                 truncated=requested != [x0,y0,x1,y1], physical_extent_at_target_m=extent_m,
                 target_pixel=uv.tolist(), target_optical_z_m=float(target[2]),
                 K_local=local_K.tolist(), resized=False,
                 local_depth_valid_fraction=float(valid[y0:y1,x0:x1].mean())))


def classify_result(metrics, limits):
    """Contact is an event; gentle target-fruit contact alone is not failure."""
    events = []
    failure = metrics.get('planning_failure_reason')
    if failure:
        reason = failure.get('reason', '')
        label = 'ik_failure' if 'ik' in reason else 'planning_failure'
        return dict(result=label, events=[label], hook_success=False, executed=False)
    if metrics.get('physics_valid') is False or metrics.get('abort_reason')=='invalid_penetration':
        return dict(result='invalid_physics',events=['invalid_penetration'],hook_success=False,
                    executed=True,physics_valid=False,exclude_from_valid_trajectory_analysis=True)
    if metrics.get('non_target_contact_count', 0):
        events.append('non_target_contact')
    if (metrics['target_max_displacement_m'] > limits['target_displacement_m'] or
            metrics['main_stem_max_displacement_m'] > limits['main_displacement_m']):
        events.append('excessive_displacement')
    if metrics.get('target_broken') or metrics.get('other_broken'):
        events.append('joint_break')
    dangerous = metrics.get('max_non_target_force_N', 0.) > limits['dangerous_non_target_force_N']
    if dangerous:
        events.append('dangerous_non_target_contact')
    success = bool(metrics.get('retained_hook') and metrics.get('inserted')
                   and not any(e in events for e in ('excessive_displacement', 'joint_break', 'dangerous_non_target_contact'))
                   and not metrics.get('hooked_non_target'))
    if metrics.get('abort_reason') == 'debug_step_limit':
        return dict(result='incomplete', events=events+['incomplete'], hook_success=False, executed=True)
    if success:
        label = 'success_target_hook'; events.append(label)
    else:
        events.append('miss')
        label = ('excessive_displacement' if 'excessive_displacement' in events else
                 'non_target_contact' if dangerous or metrics.get('hooked_non_target') else 'miss')
    return dict(result=label, events=events, hook_success=success, executed=True)
