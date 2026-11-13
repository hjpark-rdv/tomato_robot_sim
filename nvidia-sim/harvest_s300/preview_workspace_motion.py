"""Bounded random workspace motions for the assembly viewer, using joint FK.

This is a kinematic preview, not a Newton dynamics or hardware executor. The
collision model is a private copy with the displayed mount and plate transforms;
the assembly's native masks/exclusions are retained, and hidden farm geometry is
excluded. Each displayed edge is sampled at <=0.002 radians for the arm and
<=0.5mm for the lift; saved arm and lift arrays keep radians/metres separate.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import time

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from newton_inputs import sha256
from newton_ompl import Planner, PlanningError, MAX_EDGE_STEP, PENETRATION_LIMIT
from newton_patrol_motion import VELOCITY, ACCELERATION


# Approximate the user's drawn front/vertical rectangle as a shallow 3-D box.
# Coordinates refer to the centre of the declared gripper harvest volume.
WORKSPACE = np.array([[.35, -.34, .30], [.78, -.06, 1.04]])
CAPTURE_CENTER = np.array([0., .079, .037])
MAX_LIFT_STEP = .0005


def workspace_definition(side):
    """The opposite rectangle is mirrored across the chassis centreline.

    Targets are still solved against the actual asymmetric robot installation;
    mirroring a rectangle does not imply mirroring joint angles or clearance.
    """
    if side not in ('right', 'left'):
        raise ValueError('Workspace side must be right or left')
    bounds = WORKSPACE.copy()
    if side == 'left':
        bounds[:, 0] = -bounds[::-1, 0]
    return bounds


def transform(position, matrix):
    result = np.eye(4)
    result[:3, :3] = matrix
    result[:3, 3] = position
    return result


def refit_body_bvh(model, body):
    """Refit compiled body BVH after moving local colliders.

    mj_setConst does not rebuild these boxes. Body BVH boxes use the inertial
    frame and child indices are relative to the body's BVH start address.
    """
    start = int(model.body_bvhadr[body])
    count = int(model.body_bvhnum[body])
    if start < 0 or count == 0:
        raise ValueError('Moved collider owner has no compiled collision BVH')
    inertial = Rotation.from_quat(model.body_iquat[body][[1, 2, 3, 0]]).as_matrix()
    for node in range(start + count - 1, start - 1, -1):
        gid = int(model.bvh_nodeid[node])
        if gid >= 0:
            rotation = Rotation.from_quat(model.geom_quat[gid][[1, 2, 3, 0]]).as_matrix()
            centre = model.geom_pos[gid] + rotation @ model.geom_aabb[gid, :3]
            model.bvh_aabb[node, :3] = inertial.T @ (centre-model.body_ipos[body])
            model.bvh_aabb[node, 3:] = np.abs(inertial.T @ rotation) @ model.geom_aabb[gid, 3:]
        else:
            children = np.asarray(model.bvh_child[node], int) + start
            if np.any(children < start) or np.any(children >= start+count):
                raise ValueError('Unexpected body BVH child indices')
            boxes = model.bvh_aabb[children]
            lo = np.min(boxes[:, :3]-boxes[:, 3:], axis=0)
            hi = np.max(boxes[:, :3]+boxes[:, 3:], axis=0)
            model.bvh_aabb[node] = np.r_[(lo+hi)/2., (hi-lo)/2.]


def collision_model(source, initial_q, meta, displayed, output, *, layout_key=None):
    """Make native FK/colliders agree with the actual displayed installation."""
    model = copy.copy(source)
    data = mj.MjData(model)
    data.qpos[:] = initial_q
    mj.mj_kinematics(model, data)
    platform = meta['platform_id']
    root = model.body('Robot_link1').id
    parent = transform(data.xpos[platform], data.xmat[platform].reshape(3, 3))
    root_world = transform(data.xpos[root], data.xmat[root].reshape(3, 3))
    delta = displayed[root] @ np.linalg.inv(root_world)
    local_delta = np.linalg.inv(parent) @ delta @ parent
    local_root = transform(model.body_pos[root],
        Rotation.from_quat(model.body_quat[root][[1, 2, 3, 0]]).as_matrix())
    local_root = local_delta @ local_root
    model.body_pos[root] = local_root[:3, 3]
    model.body_quat[root] = Rotation.from_matrix(local_root[:3, :3]).as_quat()[[3, 0, 1, 2]]
    plate_delta = np.linalg.inv(parent) @ displayed[meta['plate_id']]
    export = json.loads((Path(meta['assets']) / 'model/export.json').read_text())
    moved = []
    removed = []
    for row in export['geometry']:
        source_path = row.get('source', '')
        if '/farmily_lift_platform/collisions/link0/' in source_path:
            adjustment = local_delta
        elif source_path.endswith('/collisions/real_mounting_plate'):
            adjustment = plate_delta
            if layout_key == 'side':
                removed.append(row['name'])
        else:
            continue
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, row['name'])
        if gid < 0 or model.geom_bodyid[gid] != platform:
            raise ValueError('Mount collision geometry missing or unexpected owner')
        local = transform(model.geom_pos[gid],
            Rotation.from_quat(model.geom_quat[gid][[1, 2, 3, 0]]).as_matrix())
        local = adjustment @ local
        model.geom_pos[gid] = local[:3, 3]
        model.geom_quat[gid] = Rotation.from_matrix(local[:3, :3]).as_quat()[[3, 0, 1, 2]]
        moved.append(row['name'])
    if len(moved) != 2:
        raise ValueError('Expected base and plate native collision proxies')
    robot = np.array([model.body(int(bid)).name.startswith(('Robot_', 'Tool_'))
                      for bid in model.geom_bodyid])
    keep = robot | (model.geom_type == mj.mjtGeom.mjGEOM_PLANE)
    if layout_key == 'side':
        from preview_side_collision import CHASSIS_ASSEMBLIES
        # layout_boxes supplies conservative component boxes for every part of
        # these disconnected CAD assemblies, instead of their combined hull.
        for name in CHASSIS_ASSEMBLIES:
            gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if gid < 0 or model.body(int(model.geom_bodyid[gid])).name != 'Robot_world':
                raise ValueError('V6 chassis assembly collider missing or changed')
            removed.append(name)
    for name in removed:
        keep[model.geom(name).id] = False
    model.geom_contype[~keep] = 0
    model.geom_conaffinity[~keep] = 0
    mj.mj_setConst(model, data)
    refit_body_bvh(model, platform)
    data.qpos[:] = initial_q
    mj.mj_kinematics(model, data)
    error = max(np.max(np.abs(transform(data.xpos[bid], data.xmat[bid].reshape(3, 3))
                              - displayed[bid])) for bid in meta['arm_bodies'])
    if error > 1e-9:
        raise ValueError(f'Collision/display FK mismatch: {error}')
    # Save the tiny installation patch instead of duplicating the 559MB CAD
    # model for every preview. The source model hash fixes all unchanged data.
    path = output / 'workspace_collision_patch.npz'
    moved_ids = np.array([model.geom(name).id for name in moved])
    bvh_start = int(model.body_bvhadr[platform])
    bvh_end = bvh_start + int(model.body_bvhnum[platform])
    np.savez_compressed(path, root_body_id=root, body_pos=model.body_pos[root],
        body_quat=model.body_quat[root], moved_geom_ids=moved_ids,
        geom_pos=model.geom_pos[moved_ids], geom_quat=model.geom_quat[moved_ids],
        disabled_geom_ids=np.flatnonzero(~keep), platform_bvh_aabb=model.bvh_aabb[bvh_start:bvh_end])
    patch_hash = sha256(path)
    configuration_hash = hashlib.sha256((meta['model_sha256'] + patch_hash).encode()).hexdigest()
    return model, dict(native_display_fk_max_error=float(error),
        collision_configuration_sha256=configuration_hash, collision_patch_sha256=patch_hash,
        collision_patch_file=path.name, moved_native_colliders=moved,
        **(dict(removed_native_colliders=removed) if removed else {}),
        **(dict(replacement_chassis_colliders=list(CHASSIS_ASSEMBLIES)) if layout_key=='side' else {}),
        moved_collider_bvh_refitted=True,
        collision_scope='Robot self, chassis, lift, plate and ground; hidden farm excluded')


def capture(data, tool):
    matrix = data.xmat[tool].reshape(3, 3)
    return data.xpos[tool] + matrix @ CAPTURE_CENTER


def duration(points):
    derivative = np.max(np.abs(np.diff(points, axis=0)), axis=0) * (len(points) - 1)
    axes = slice(1, 8) if points.shape[1] == 7 else slice(2, 8)
    return 1.1 * max(.7, float(np.max(1.875 * derivative / VELOCITY[axes])),
                    float(np.sqrt(np.max(5.774 * derivative / ACCELERATION[axes]))))


def edge_steps(a, b):
    delta = np.asarray(b)-np.asarray(a)
    if delta.shape == (7,):
        return max(1, int(math.ceil(abs(delta[0]) / MAX_LIFT_STEP)),
                   int(math.ceil(np.linalg.norm(delta[1:]) / MAX_EDGE_STEP)))
    if delta.shape == (6,):
        return max(1, int(math.ceil(np.linalg.norm(delta) / MAX_EDGE_STEP)))
    raise ValueError('Expected six arm axes or lift plus six arm axes')


class WorkspaceMotion:
    def __init__(self, segments, report):
        self.segments = segments
        self.report = report
        self.loop = [(i, False) for i in range(1, len(segments))]
        self.loop += [(i, True) for i in range(len(segments) - 1, 0, -1)]
        self.loop_duration = sum(segments[i]['duration_s'] for i, _ in self.loop)

    def sample(self, elapsed):
        entry = self.segments[0]
        cycle = 0
        if elapsed < entry['duration_s']:
            index, reverse, local = 0, False, max(0., elapsed)
        else:
            total = elapsed - entry['duration_s']
            cycle = int(total // self.loop_duration)
            local = total % self.loop_duration
            for index, reverse in self.loop:
                span = self.segments[index]['duration_s']
                if local < span:
                    break
                local -= span
        segment = self.segments[index]
        u = np.clip(local / segment['duration_s'], 0., 1.)
        s = u**3 * (10. + u * (-15. + 6. * u))
        if reverse:
            s = 1. - s
        points = segment['points']
        f = s * (len(points) - 1)
        k = min(int(f), len(points) - 2)
        q = points[k] + (f - k) * (points[k + 1] - points[k])
        target = segment['start_capture_m'] if reverse else segment['target_capture_m']
        sample = dict(segment=index, reverse=reverse, cycle=cycle,
                      target_capture_m=target, entry=index == 0)
        if len(q) == 7:
            sample['lift_height_m'] = float(q[0])
        return q, sample


def plan_workspace_motion(source, initial, meta, displayed, output, *, seed=20261112, count=24,
                          move_lift=True, side='right'):
    started = time.monotonic()
    workspace = workspace_definition(side)
    q = np.asarray(initial['qpos']).copy()
    # The displayed lift and six arm angles are passed by the preview caller.
    model, layout = collision_model(source, q, meta, displayed, output)
    planner = Planner(model, initial, model_sha256=layout['collision_configuration_sha256'])
    planner.fixed = q.copy()
    planner.deadline = started + 45.
    if not planner.valid_arm(q[2:8]):
        raise ValueError(f'Displayed start is in collision: {planner.stats}')
    tool = planner.tool_id
    start_configuration = q[1:8].copy() if move_lift else q[2:8].copy()
    nominal = displayed[tool, :3, :3]
    if side == 'left':
        nominal = Rotation.from_euler('z', 180., degrees=True).as_matrix() @ nominal
    rng = np.random.default_rng(seed)
    segments, rejected = [], []
    checks = 0
    worst = 0.
    max_step = 0.
    max_lift_step = 0.
    max_position_error = 0.

    def apply(configuration):
        if move_lift:
            planner.fixed[1] = configuration[0]
            return configuration[1:]
        return configuration

    def valid(configuration):
        arm = apply(configuration)
        return planner.valid_arm(arm)

    def connection(configuration, goal, rotation, entry=False):
        nonlocal checks, worst, max_step, max_lift_step, max_position_error
        arm = apply(configuration)
        start_pos, start_rot = planner._pose(arm)
        start_capture = start_pos + start_rot @ CAPTURE_CENTER
        start_lift = float(planner.fixed[1])
        target_lift = float(np.clip(goal[2]-.15, .20, .75)) if move_lift else start_lift
        rotations = Slerp([0., 1.], Rotation.from_matrix([start_rot, rotation]))
        n = max(2, int(math.ceil(np.linalg.norm(goal - start_capture) / .025)),
                int(math.ceil(abs(target_lift-start_lift)/.02)))
        points = [configuration.copy()]
        measured_centres = [start_capture.copy()]
        for u in np.linspace(0., 1., n + 1)[1:]:
            centre = (1. - u) * start_capture + u * goal
            orient = rotations(u).as_matrix()
            previous_arm = points[-1][1:] if move_lift else points[-1]
            lift = (1.-u)*start_lift + u*target_lift
            planner.fixed[1] = lift
            target_arm = planner.ik(dict(position=centre - orient @ CAPTURE_CENTER,
                quaternion_xyzw=Rotation.from_matrix(orient).as_quat()), previous_arm)
            target = np.r_[lift, target_arm] if move_lift else target_arm
            a = points[-1]
            steps = edge_steps(a, target)
            arm_delta = (target-a)[1:] if move_lift else target-a
            max_step = max(max_step, float(np.linalg.norm(arm_delta) / steps))
            if move_lift:
                max_lift_step = max(max_lift_step, float(abs(target[0]-a[0])/steps))
            for v in np.linspace(0., 1., steps + 1):
                sample = a + v * (target - a)
                checks += 1
                if not valid(sample):
                    raise PlanningError('Intermediate assembly collision or joint limit')
                actual = capture(planner.data, tool)
                heading = planner.data.xmat[tool].reshape(3, 3) @ nominal.T
                if abs(math.atan2(heading[1, 0], heading[0, 0])) > math.radians(15.):
                    raise PlanningError('Intermediate gripper heading exceeds 15 degrees')
                if not entry and (np.any(actual < workspace[0]) or np.any(actual > workspace[1])):
                    raise PlanningError('Intermediate gripper centre leaves the workspace')
                for contact in planner.data.contact:
                    if planner.robot_geoms[int(contact.geom1)] or planner.robot_geoms[int(contact.geom2)]:
                        worst = max(worst, -float(contact.dist))
            planner._pose(apply(target))
            actual = capture(planner.data, tool)
            error = float(np.linalg.norm(actual - centre))
            max_position_error = max(max_position_error, error)
            if error > .004:
                raise PlanningError('Capture-centre IK error exceeds 4mm')
            points.append(target)
            measured_centres.append(actual.copy())
        points = np.asarray(points)
        return dict(points=points, duration_s=duration(points),
            start_capture_m=start_capture.tolist(), target_capture_m=goal.tolist(),
            measured_capture_m=measured_centres[-1].tolist(),
            measured_centres=measured_centres, entry=entry,
            start_lift_m=start_lift, target_lift_m=target_lift)

    # Enter from the current preview posture; this connector may start outside
    # the box. All subsequent motions, including reverse playback, stay inside.
    if side == 'right':
        entry = connection(start_configuration, np.array([.46, -.20, .82]), nominal, entry=True)
    else:
        # Change working side above the chassis. During this transfer the tool
        # must rotate 180 degrees; the +/-15 degree harvesting heading applies
        # after arrival, not while changing sides. No collision exemption is
        # made: independently audit this connector together with the loop.
        goal = np.array([-.46, -.20, .95])
        apply(start_configuration)
        start_pos, start_rot = planner._pose(q[2:8])
        start_capture = start_pos + start_rot @ CAPTURE_CENTER
        target_arm = planner.ik(dict(position=goal - nominal @ CAPTURE_CENTER,
            quaternion_xyzw=Rotation.from_matrix(nominal).as_quat()), q[2:8])
        direct = planner.edge(q[2:8], target_arm)
        if direct:
            arm_path = np.array([q[2:8], target_arm])
        else:
            # Preserve the existing native masks and 1mm collision criterion.
            arm_path = np.asarray(planner._ompl(q[2:8], target_arm))
        planner._pose(target_arm)
        points = np.column_stack((np.full(len(arm_path), q[1]), arm_path)) if move_lift else arm_path
        entry = dict(points=points, duration_s=duration(points), entry=True,
            start_capture_m=start_capture.tolist(), target_capture_m=goal.tolist(),
            measured_capture_m=capture(planner.data, tool).tolist(),
            start_lift_m=float(q[1]), target_lift_m=float(q[1]),
            side_change=True, connector_planner='Checked joint segment' if direct else 'OMPL RRTConnect')
    segments.append(entry)
    configuration = entry['points'][-1]
    for attempt in range(count * 10):
        if len(segments) >= count + 1 or time.monotonic() >= planner.deadline:
            break
        goal = rng.uniform(workspace[0] + .012, workspace[1] - .012)
        if np.linalg.norm(goal - np.asarray(segments[-1]['measured_capture_m'])) < .12:
            continue
        yaw = float(rng.uniform(-15., 15.))
        orient = Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ nominal
        try:
            segment = connection(configuration, goal, orient)
        except PlanningError as error:
            rejected.append(dict(target_capture_m=goal.tolist(), reason=str(error)))
            continue
        segments.append(segment)
        configuration = segment['points'][-1]
        print(f'Workspace target {len(segments)-1}/{count}: {np.round(goal,3)}', flush=True)
    planner.deadline = math.inf
    if len(segments) < 9:
        raise ValueError(f'Too few reachable random targets: {len(segments)-1}')
    # Audit only accepted paths independently; failed exploratory candidates
    # are recorded, and can never become displayed interpolation points.
    worst = 0.
    checks = 0
    max_step = 0.
    max_lift_step = 0.
    centres = []
    lifts = []
    maximum_heading = 0.
    maximum_transfer_heading = 0.
    for segment in segments:
        for a, b in zip(segment['points'][:-1], segment['points'][1:]):
            steps = edge_steps(a, b)
            max_step = max(max_step, float(np.linalg.norm((b-a)[1:] if move_lift else b-a)/steps))
            if move_lift:
                max_lift_step = max(max_lift_step, float(abs(b[0]-a[0])/steps))
            for u in np.linspace(0., 1., steps + 1):
                if not valid(a + u * (b-a)):
                    raise ValueError('Final accepted path audit failed')
                checks += 1
                actual = capture(planner.data, tool)
                lifts.append(float(planner.data.qpos[1]))
                heading = planner.data.xmat[tool].reshape(3, 3) @ nominal.T
                if not segment.get('side_change', False):
                    maximum_heading = max(maximum_heading, abs(math.degrees(math.atan2(heading[1, 0], heading[0, 0]))))
                else:
                    maximum_transfer_heading = max(maximum_transfer_heading,
                        abs(math.degrees(math.atan2(heading[1, 0], heading[0, 0]))))
                if not segment['entry']:
                    if np.any(actual < workspace[0]) or np.any(actual > workspace[1]):
                        raise ValueError('Final accepted path leaves workspace')
                    centres.append(actual.copy())
                for contact in planner.data.contact:
                    if planner.robot_geoms[int(contact.geom1)] or planner.robot_geoms[int(contact.geom2)]:
                        worst = max(worst, -float(contact.dist))
    if maximum_heading > 15.:
        raise ValueError('Final accepted path exceeds 15 degree heading limit')
    if move_lift and max(lifts)-min(lifts) < .25:
        raise ValueError('Coordinated preview did not exercise enough lift travel')
    points = np.concatenate([s['points'] for s in segments])
    offsets = np.r_[0, np.cumsum([len(s['points']) for s in segments])]
    arrays = dict(joint_positions_rad=points[:, 1:] if move_lift else points,
        segment_offsets=offsets, durations_s=[s['duration_s'] for s in segments])
    if move_lift:
        arrays['lift_positions_m'] = points[:, 0]
    np.savez_compressed(output / 'workspace_trajectory.npz', **arrays)
    centres = np.asarray(centres)
    report = dict(schema='s300-workspace-motion-preview-v2', seed=seed,
        source_model_sha256=meta['model_sha256'], source_initial_sha256=meta['initial_sha256'],
        **layout, workspace_side=side, workspace_world_bounds_m=workspace.tolist(),
        capture_center_tool_m=CAPTURE_CENTER.tolist(), nominal_tool_rotation_world=nominal.tolist(),
        heading_limit_scope='Within selected workspace; changing working side is audited for collision, not harvesting heading',
        initial_positions=q.tolist(), random_target_count=len(segments)-1,
        rejected_candidates=rejected, accepted_path_checks=checks,
        maximum_sampled_penetration_m=max(0., worst), penetration_limit_m=PENETRATION_LIMIT,
        maximum_audit_joint_step_rad=max_step, joint_step_limit_rad=MAX_EDGE_STEP,
        lift_motion_enabled=move_lift, visited_lift_bounds_m=[min(lifts),max(lifts)],
        maximum_audit_lift_step_m=max_lift_step, lift_step_limit_m=MAX_LIFT_STEP,
        maximum_abs_heading_deg=maximum_heading,
        maximum_side_change_heading_deg=maximum_transfer_heading,
        trajectory_columns=['lift_m','base_rad','shoulder_rad','elbow_rad','wrist1_rad','wrist2_rad','wrist3_rad']
            if move_lift else ['base_rad','shoulder_rad','elbow_rad','wrist1_rad','wrist2_rad','wrist3_rad'],
        maximum_attempted_ik_capture_error_m=max_position_error,
        visited_capture_world_bounds_m=[centres.min(0).tolist(), centres.max(0).tolist()],
        loop='forward/reverse accepted paths; no reset jump',
        planning_seconds=time.monotonic()-started, physics_steps=0,
        planner='Existing Jacobian IK with Cartesian waypoints and sampled joint-edge collision checks',
        scope='Kinematic assembly motion preview, not Newton GPU dynamics, harvesting or hardware validation',
        segments=[{k:v for k,v in s.items() if k not in ('points','measured_centres')} for s in segments])
    motion = WorkspaceMotion(segments, report)
    report['entry_duration_s'] = segments[0]['duration_s']
    report['loop_duration_s'] = motion.loop_duration
    (output / 'workspace_motion.json').write_text(json.dumps(report, indent=2)+'\n')
    return motion
