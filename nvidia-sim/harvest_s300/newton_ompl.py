"""OMPL arm planning against the simulator's unchanged native collision model.

This module only evaluates FK and collision queries on CPU. Newton remains the
physics executor. No collision masks, body exclusions or physical state are
modified. Paths are sampled collision checks, not continuous collision proofs.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time
import weakref

import mujoco as mj
import numpy as np
from ompl import base as ob, geometric as og, util as ou

from newton_patrol_motion import VELOCITY, ACCELERATION

ARM = slice(2, 8)
JOINT_NAMES = ['root_joint', 'farmily_lift_height_joint', 'base', 'shoulder',
               'elbow', 'wrist1', 'wrist2', 'wrist3', 'jaw_L', 'jaw_R']
MAX_EDGE_STEP = .002
PENETRATION_LIMIT = .001
# Same tracking envelope used by the command controller. This applies only to
# measured poses and their connector, never to commanded/planned joint targets.
MEASURED_BOUNDS_TOLERANCE = np.array([.002, .002] + [.01] * 6 + [.001, .001])


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class PlanningError(ValueError):
    pass


def timed_trajectory(path, names):
    """Same straight segments as audited, with rest-to-rest quintic timing."""
    path = np.asarray(path, float)
    if path.ndim != 2 or path.shape[1] != 6 or len(path) < 2 or not np.isfinite(path).all():
        raise PlanningError('A finite six-axis path with at least two states is required')
    if len(path) > 513:
        raise PlanningError('Path exceeds 512 trajectory points')
    elapsed = 0.
    points = []
    for previous, target in zip(path[:-1], path[1:]):
        delta = np.abs(target - previous)
        duration = 1.05 * max(.05, float(np.max(1.875 * delta / VELOCITY[ARM])),
                             float(np.sqrt(np.max((10 / math.sqrt(3)) * delta / ACCELERATION[ARM]))))
        elapsed += duration
        if elapsed > 300:
            raise PlanningError('Trajectory duration exceeds the command controller 300s limit')
        points.append(dict(positions=target.tolist(), velocities=[], accelerations=[],
                           effort=[], time_from_start=elapsed))
    return dict(joint_names=list(names), start_time=0., points=points)


class Planner:
    def __init__(self, model, initial, *, model_sha256, source=None):
        self.model = model
        self.data = mj.MjData(model)
        if model.nq != 10 or model.nv != 10 or model.njnt != 10:
            raise PlanningError('Planner requires the ten-axis static-plant model')
        self.names = [model.joint(i).name for i in range(10)]
        if self.names != JOINT_NAMES or not np.array_equal(model.jnt_qposadr, np.arange(10)):
            raise PlanningError('Unexpected robot joint order; refusing implicit remapping')
        if not np.all(model.jnt_limited):
            raise PlanningError('Finite joint bounds required')
        self.bounds = np.array(model.jnt_range, float)
        self.initial = np.asarray(initial['qpos'], float).copy()
        if 'eq_active' in initial:
            self.data.eq_active[:] = initial['eq_active']
        self.model_sha256 = model_sha256
        self.source = source
        self.robot_geoms = np.array([model.body(int(b)).name.startswith(
            ('Robot_', 'RB5_', 'Tool_', 'S300_', 'Lift_')) for b in model.geom_bodyid])
        if not np.any(self.robot_geoms):
            raise PlanningError('No robot collision bodies found')
        self.tool_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'Tool_fixed')
        if self.tool_id < 0:
            raise PlanningError('Tool_fixed origin is required for Cartesian goals')
        self.fixed = self.initial.copy()
        self.deadline = math.inf
        self.stats = {}
        self.space = ob.RealVectorStateSpace(6)
        bounds = ob.RealVectorBounds(6)
        for i, (lo, hi) in enumerate(self.bounds[ARM]):
            bounds.setLow(i, float(lo)); bounds.setHigh(i, float(hi))
        self.space.setBounds(bounds)
        # OMPL measures each edge in Euclidean joint-space distance. This
        # guarantees no coordinate changes more than 0.002 rad per sample.
        self.space.setLongestValidSegmentFraction(MAX_EDGE_STEP / self.space.getMaximumExtent())
        self.si = ob.SpaceInformation(self.space)
        reference = weakref.proxy(self)
        self.si.setStateValidityChecker(lambda state: reference.valid_arm(
            np.array([state[i] for i in range(6)])))
        self.si.setup()
        ou.setLogLevel(ou.LogLevel.LOG_NONE)

    @classmethod
    def from_inputs(cls, path):
        path = Path(path).resolve()
        manifest = json.loads((path / 'inputs.json').read_text())
        hashes = {}
        for name in ('input_model.mjb', 'initial_state.npz'):
            expected = manifest['input_files'][name]['sha256']
            actual = sha256(path / name)
            if actual != expected:
                raise PlanningError('Input SHA256 mismatch: ' + name)
            hashes[name] = actual
        return cls(mj.MjModel.from_binary_path(str(path / 'input_model.mjb')),
                   dict(np.load(path / 'initial_state.npz', allow_pickle=False)),
                   model_sha256=hashes['input_model.mjb'], source=dict(
                       directory=str(path), input_hashes=hashes, manifest_sha256=sha256(path / 'inputs.json')))

    def ready(self):
        return dict(kind='ready', planner='OMPL RRTConnect', ompl_version='2.0.1',
                    model_sha256=self.model_sha256, source=self.source,
                    joint_names=self.names, arm_joint_names=self.names[ARM],
                    initial_positions=self.initial.tolist(), position_limits=self.bounds.tolist(),
                    measured_bounds_tracking_envelope=MEASURED_BOUNDS_TOLERANCE.tolist(),
                    collision_limit_m=PENETRATION_LIMIT, max_edge_joint_step_rad=MAX_EDGE_STEP,
                    tool_frame='Tool_fixed body origin', pose_reference_frame='world')

    def expired(self):
        return time.monotonic() >= self.deadline

    def valid_full(self, q, *, remember=True, measured=False):
        if self.expired():
            self.stats['deadline_expired'] = True
            return False
        self.stats['state_checks'] = self.stats.get('state_checks', 0) + 1
        q = np.asarray(q, float)
        if q.shape != (10,) or not np.isfinite(q).all():
            return False
        margin = MEASURED_BOUNDS_TOLERANCE if measured else 0.
        if np.any(q < self.bounds[:, 0] - margin - 1e-9) or np.any(q > self.bounds[:, 1] + margin + 1e-9):
            return False
        started = time.monotonic()
        self.data.qpos[:] = q
        # No mj_step, dynamics solver, gravity or actuator mutation occurs.
        mj.mj_kinematics(self.model, self.data)
        mj.mj_collision(self.model, self.data)
        self.stats['collision_query_seconds'] = self.stats.get('collision_query_seconds', 0.) + time.monotonic() - started
        for contact in self.data.contact:
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if (self.robot_geoms[g1] or self.robot_geoms[g2]) and contact.dist < -PENETRATION_LIMIT:
                if remember:
                    self.stats['last_collision'] = dict(
                        geom1=self.model.geom(g1).name, geom2=self.model.geom(g2).name,
                        body1=self.model.body(int(self.model.geom_bodyid[g1])).name,
                        body2=self.model.body(int(self.model.geom_bodyid[g2])).name,
                        penetration_m=float(-contact.dist))
                return False
        return True

    def valid_arm(self, arm):
        q = self.fixed.copy(); q[ARM] = arm
        return self.valid_full(q)

    def edge(self, a, b, *, full=False, measured=False):
        self.stats['explicit_edge_checks'] = self.stats.get('explicit_edge_checks', 0) + 1
        a, b = np.asarray(a, float), np.asarray(b, float)
        if full:
            # The measured-state connector includes slide joints in metres;
            # limit them to 0.5mm instead of treating them as angular axes.
            resolution = np.array([.0005, .0005] + [MAX_EDGE_STEP] * 6 + [.0005, .0005])
            steps = max(1, int(math.ceil(np.linalg.norm((b-a) / resolution))))
        else:
            steps = max(1, int(math.ceil(np.linalg.norm(b-a) / MAX_EDGE_STEP)))
        for i in range(steps + 1):
            q = a + (b-a) * (i / steps)
            if not (self.valid_full(q, measured=measured) if full else self.valid_arm(q)):
                return False
        return True

    def _pose(self, arm):
        self.data.qpos[:] = self.fixed
        self.data.qpos[ARM] = arm
        mj.mj_kinematics(self.model, self.data)
        mj.mj_comPos(self.model, self.data)
        return self.data.xpos[self.tool_id].copy(), self.data.xmat[self.tool_id].reshape(3, 3).copy()

    def ik(self, goal, start):
        pos = np.asarray(goal['position'], float)
        quat = np.asarray(goal['quaternion_xyzw'], float)
        if pos.shape != (3,) or quat.shape != (4,) or not np.isfinite(np.r_[pos, quat]).all():
            raise PlanningError('Cartesian goal requires finite position[3] and quaternion_xyzw[4]')
        norm = float(np.linalg.norm(quat))
        if abs(norm - 1) > .01:
            raise PlanningError('Cartesian goal quaternion must be normalized')
        quat = quat / norm
        target_matrix = np.zeros(9)
        mj.mju_quat2Mat(target_matrix, np.r_[quat[3], quat[:3]])
        target_matrix = target_matrix.reshape(3, 3)
        rng = np.random.default_rng(42)
        seeds = [start.copy()] + [np.clip(start + rng.normal(0, .4, 6),
                   self.bounds[ARM, 0], self.bounds[ARM, 1]) for _ in range(3)]
        for seed in seeds:
            arm = seed.copy()
            for _ in range(100):
                if self.expired():
                    raise PlanningError('Planning timeout during Cartesian IK')
                current_pos, current_matrix = self._pose(arm)
                error_rotation = np.zeros(4); angular = np.zeros(3)
                mj.mju_mat2Quat(error_rotation, (target_matrix @ current_matrix.T).ravel())
                if error_rotation[0] < 0:
                    error_rotation *= -1
                mj.mju_quat2Vel(angular, error_rotation, 1.)
                delta_pos = pos - current_pos
                self.stats['ik_iterations'] = self.stats.get('ik_iterations', 0) + 1
                self.stats['ik_position_error_m'] = float(np.linalg.norm(delta_pos))
                self.stats['ik_rotation_error_rad'] = float(np.linalg.norm(angular))
                if np.linalg.norm(delta_pos) <= .002 and np.linalg.norm(angular) <= .02:
                    if self.valid_arm(arm):
                        return arm
                    break
                jacp = np.zeros((3, 10)); jacr = np.zeros((3, 10))
                mj.mj_jacBody(self.model, self.data, jacp, jacr, self.tool_id)
                jac = np.vstack((jacp[:, ARM], .25 * jacr[:, ARM]))
                error = np.r_[delta_pos, .25 * angular]
                change = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 1e-5, error)
                change *= min(1., .12 / max(1e-12, float(np.max(np.abs(change)))))
                arm = np.clip(arm + change, self.bounds[ARM, 0], self.bounds[ARM, 1])
        raise PlanningError('No collision-valid Cartesian IK within the bounded seeds and time budget')

    def _goal(self, goal, start):
        if 'position' in goal or 'quaternion_xyzw' in goal:
            return self.ik(goal, start)
        names = goal['joint_names']
        target = np.asarray(goal['positions'], float)
        if not names or len(names) != len(set(names)) or not set(names).issubset(self.names[ARM]):
            raise PlanningError('Joint goal accepts unique arm joint names only')
        if target.shape != (len(names),) or not np.isfinite(target).all():
            raise PlanningError('Joint goal needs one finite position per named arm joint')
        result = start.copy()
        for name, value in zip(names, target):
            result[self.names[ARM].index(name)] = value
        return result

    def _ompl(self, start, target):
        a, b = self.space.allocState(), self.space.allocState()
        try:
            for i in range(6):
                a[i] = float(start[i]); b[i] = float(target[i])
            problem = ob.ProblemDefinition(self.si)
            problem.setStartAndGoalStates(a, b)
            planner = og.RRTConnect(self.si)
            planner.setRange(.25)
            planner.setProblemDefinition(problem); planner.setup()
            remaining = max(0., self.deadline - time.monotonic() - .15)
            if remaining <= 0:
                raise PlanningError('Planning timeout before RRTConnect')
            planner.solve(remaining)
            if not problem.hasExactSolution():
                raise PlanningError('No exact OMPL path within the planning time budget')
            path = problem.getSolutionPath()
            return [np.array([path.getState(i)[j] for j in range(6)]) for i in range(path.getStateCount())]
        finally:
            # OMPL 2's nanobind allocState result owns its state. Calling
            # freeState manually double-frees it when Python drops the wrapper.
            del a, b

    def plan(self, request):
        started = time.monotonic()
        self.stats = dict(state_checks=0, explicit_edge_checks=0, collision_query_seconds=0.,
                          deadline_expired=False)
        self.si.getMotionValidator().resetMotionCounter()
        result = dict(id=request.get('id'), success=False, trajectory=None, error=None)
        try:
            timeout = float(request.get('timeout_s', 2.))
            if not np.isfinite(timeout) or not .05 <= timeout <= 10:
                raise PlanningError('timeout_s must be within 0.05..10 seconds')
            self.deadline = started + timeout
            self.fixed = np.asarray(request['start_positions'], float).copy()
            measured = np.asarray(request['measured_positions'], float)
            # Preserve the observed start even when validation rejects early.
            # Nonfinite inputs remain JSON-serializable rejected diagnostics.
            for name, value in (('commanded_start_positions', self.fixed),
                                ('measured_start_positions', measured)):
                self.stats[name] = [float(v) if np.isfinite(v) else None for v in value] if value.shape == (10,) else None
            if measured.shape == (10,) and np.isfinite(measured).all():
                excess = np.maximum(0., np.maximum(self.bounds[:, 0]-measured,
                                                   measured-self.bounds[:, 1]))
                self.stats['measured_joint_bounds_violations'] = [dict(
                    joint_name=self.names[i], measured_position=float(measured[i]),
                    position_bounds=self.bounds[i].tolist(), excess=float(excess[i]),
                    allowed_tracking_envelope=float(MEASURED_BOUNDS_TOLERANCE[i]))
                    for i in range(10) if excess[i] > 0.]
            if not self.valid_full(self.fixed):
                raise PlanningError('Commanded start is outside bounds or in collision')
            if not self.valid_full(measured, measured=True):
                raise PlanningError('Measured start exceeds the tracking bounds envelope or is in collision')
            # Evaluate original measured q and every interpolated connector
            # state; never clamp away actual penetration near joint limits.
            if not self.edge(measured, self.fixed, full=True, measured=True):
                raise PlanningError('Measured-to-commanded start connector is in collision or timed out')
            self.stats['measured_start_max_error'] = float(np.max(np.abs(measured-self.fixed)))
            start = self.fixed[ARM].copy()
            target = self._goal(request['goal'], start)
            if not self.valid_arm(target):
                raise PlanningError('Goal is outside joint bounds or in collision')
            if self.edge(start, target):
                path = [start, target]; self.stats['route'] = 'direct_validated'
            else:
                path = self._ompl(start, target); self.stats['route'] = 'rrt_connect_detour'
                # Deterministic greedy shortcuts, each using the same strict
                # check. Keep time for final audit and fail closed on timeout.
                tries = 0; i = 0
                while i + 2 < len(path) and tries < 32 and self.deadline-time.monotonic() > .4:
                    for j in range(len(path)-1, i+1, -1):
                        tries += 1
                        if self.edge(path[i], path[j]):
                            path[i+1:j] = []; break
                        if tries >= 32 or self.deadline-time.monotonic() <= .4:
                            break
                    i += 1
                self.stats['shortcut_checks'] = tries
            if len(path) > 513:
                raise PlanningError('OMPL path exceeds 512 trajectory points')
            for a, b in zip(path[:-1], path[1:]):
                if not self.edge(a, b):
                    raise PlanningError('Final path collision audit failed or timed out')
            trajectory = timed_trajectory(path, self.names[ARM])
            if self.expired():
                raise PlanningError('Planning timeout during final validation')
            result.update(success=True, trajectory=trajectory)
            self.stats.update(path_positions=np.asarray(path).tolist(), target_positions=target.tolist(),
                              trajectory_duration_s=trajectory['points'][-1]['time_from_start'],
                              fixed_positions=self.fixed.tolist())
        except (ValueError, KeyError, TypeError, OverflowError, np.linalg.LinAlgError) as error:
            result['error'] = str(error)
        finally:
            validator = self.si.getMotionValidator()
            self.stats.update(ompl_edge_checks=validator.getCheckedMotionCount(),
                              ompl_invalid_edges=validator.getInvalidMotionCount())
            result['report'] = dict(planner='OMPL RRTConnect', ompl_version='2.0.1',
                model_sha256=self.model_sha256, source=self.source,
                planning_seconds=time.monotonic()-started, **self.stats,
                collision_limit_m=PENETRATION_LIMIT, max_edge_joint_step_rad=MAX_EDGE_STEP,
                measured_bounds_tracking_envelope=MEASURED_BOUNDS_TOLERANCE.tolist(),
                collision_policy='Original MuJoCo masks/exclusions; robot contacts with penetration >1mm rejected',
                collision_guarantee='Sampled at <=0.002rad Euclidean arm step; runtime GPU collision guard remains enabled',
                physics_execution='Newton GPU, not this CPU FK/collision worker',
                tool_frame='Tool_fixed body origin', pose_reference_frame='world')
            self.deadline = math.inf
        return result
