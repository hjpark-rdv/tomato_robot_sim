"""Deterministic recovery of a collision-free robot ready configuration.

This module searches only robot joint configuration q.  It never changes plant
geometry, collision masks, contact permissions, material parameters, margins,
actuators, or the robot base transform.  Search exhaustion is reported as
unresolved_within_budget, never as a proof of infeasibility.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.stats import qmc

from approach_connection import JointSpace, LimitReached, vector


@dataclass
class StartSearchBudget:
    max_candidates: int = 512
    max_distance_queries: int = 3000000
    seconds: float = 180.0

    def __post_init__(self):
        if isinstance(self.max_candidates, bool) or not isinstance(self.max_candidates, int) or self.max_candidates < 1:
            raise ValueError("max_candidates must be a positive integer")
        if isinstance(self.max_distance_queries, bool) or not isinstance(self.max_distance_queries, int) or self.max_distance_queries < 1:
            raise ValueError("max_distance_queries must be a positive integer")
        if not math.isfinite(self.seconds) or self.seconds <= 0:
            raise ValueError("seconds must be finite and positive")
        self.started = time.monotonic()
        self.candidates = 0
        self.distance_queries = 0

    def remaining(self):
        return self.seconds - (time.monotonic() - self.started)

    def candidate(self):
        if self.candidates >= self.max_candidates:
            raise LimitReached("start_candidate_budget")
        if self.remaining() <= 0:
            raise LimitReached("start_wall_time_budget")
        self.candidates += 1

    def query(self):
        if self.distance_queries >= self.max_distance_queries:
            raise LimitReached("start_distance_query_budget")
        if self.remaining() <= 0:
            raise LimitReached("start_wall_time_budget")
        self.distance_queries += 1


def normalized_change(q, original, space):
    q = vector(q, space.n)
    original = vector(original, space.n)
    return float(np.linalg.norm((q - original) / space.span))


def deterministic_candidates(original, space, *, radii=(.01, .025, .05, .1, .2, .4),
                             samples_per_radius=24, seed=20260928):
    """Yield unique candidates in expanding normalized joint-space shells.

    Radii are search extents, not safety thresholds.  Axis moves are emitted
    before Sobol directions at each radius so simple retract motions are tested
    early.  The original q is emitted first as immutable baseline evidence.
    """
    original = vector(original, space.n)
    if not space.contains(original):
        raise ValueError("Original start is outside joint bounds")
    radii = tuple(float(r) for r in radii)
    if not radii or any(not math.isfinite(r) or r <= 0 or r > 1 for r in radii):
        raise ValueError("radii must be finite normalized fractions in (0,1]")
    if isinstance(samples_per_radius, bool) or not isinstance(samples_per_radius, int) or samples_per_radius < 1:
        raise ValueError("samples_per_radius must be positive")

    seen = set()
    def emit(q):
        q = np.clip(np.asarray(q, dtype=float), space.low, space.high)
        key = q.tobytes()
        if key in seen:
            return None
        seen.add(key)
        return q.copy()

    first = emit(original)
    if first is not None:
        yield dict(q=first, source="original", radius=0.0, sample_index=0)

    sampler = qmc.Sobol(d=space.n, scramble=True, seed=seed)
    total = len(radii) * samples_per_radius
    # Preserve Sobol balance properties: generate a power-of-two block, then
    # take the deterministic prefix needed by this bounded search.
    power = int(math.ceil(math.log2(max(1, total))))
    points = sampler.random_base2(power)[:total]
    cursor = 0
    for radius in radii:
        for axis in range(space.n):
            for sign in (-1.0, 1.0):
                q = original.copy()
                q[axis] += sign * radius * space.span[axis]
                value = emit(q)
                if value is not None:
                    yield dict(q=value, source="axis", radius=radius,
                               sample_index=axis * 2 + int(sign > 0), axis=axis, sign=sign)
        for index in range(samples_per_radius):
            direction = 2.0 * points[cursor] - 1.0
            cursor += 1
            scale = float(np.max(np.abs(direction)))
            if scale <= 1e-12:
                continue
            direction /= scale
            value = emit(original + radius * space.span * direction)
            if value is not None:
                yield dict(q=value, source="sobol", radius=radius, sample_index=index)


class StartStateEvaluator:
    """Exact native robot/environment non-overlap + existing FCL self collision."""

    def __init__(self, scene, selfchecker, space, budget, probe_horizon_m=.03):
        self.scene = scene
        self.selfchecker = selfchecker
        self.space = space
        self.budget = budget
        self.probe_horizon_m = float(probe_horizon_m)
        if not math.isfinite(self.probe_horizon_m) or self.probe_horizon_m <= 0:
            raise ValueError("probe_horizon_m must be positive")
        self.cache = {}

    def evaluate(self, q):
        from environment_preflight import nearby_indices
        q = vector(q, self.space.n)
        key = q.tobytes()
        if key in self.cache:
            return dict(self.cache[key], cached=True)
        self.budget.candidate()
        row = dict(q=q.tolist(), valid=False, cached=False,
                   joint_bounds=self.space.contains(q), self_collision=None,
                   minimum_distance_m=None, minimum_distance_is_lower_bound=False,
                   closest_pair=None, closest_segment_world=None)
        if not row["joint_bounds"]:
            row["status"] = "joint_bounds"
            self.cache[key] = row
            return dict(row)
        pair = self.selfchecker.check(q)
        if pair is not None:
            row.update(status="self_collision", self_collision=pair)
            self.cache[key] = row
            return dict(row)

        self.scene.set_robot(q)
        minimum = None
        closest = None
        segment = None
        for i, robot_name in enumerate(self.scene.robot_names):
            indices = nearby_indices(
                self.scene.robot_positions[i], self.scene.robot_radii[i],
                self.scene.environment_positions, self.scene.environment_radii,
                self.probe_horizon_m)
            for j in indices:
                self.budget.query()
                env_name = self.scene.environment_names[int(j)]
                distance, witness = self.scene.distance(i, int(j), self.probe_horizon_m)
                if not math.isfinite(distance) or not np.isfinite(witness).all():
                    raise ValueError("Non-finite native distance result")
                if minimum is None or distance < minimum:
                    minimum = float(distance)
                    closest = [robot_name, env_name]
                    segment = np.asarray(witness, dtype=float).reshape(2, 3).tolist()
                # Ready/park state has no contact exception: exact non-overlap.
                if distance <= 0.0:
                    row.update(status="environment_collision",
                               minimum_distance_m=float(distance),
                               closest_pair=closest,
                               closest_segment_world=segment)
                    self.cache[key] = row
                    return dict(row)
        if minimum is None:
            minimum = self.probe_horizon_m
            row["minimum_distance_is_lower_bound"] = True
        row.update(status="valid", valid=True, minimum_distance_m=float(minimum),
                   closest_pair=closest, closest_segment_world=segment)
        self.cache[key] = row
        return dict(row)


def search_start(original, space, evaluator, *, radii=(.01, .025, .05, .1, .2, .4),
                 samples_per_radius=24, seed=20260928, max_valid=12):
    """Bounded search.  Returns evidence even when no candidate is found."""
    if isinstance(max_valid, bool) or not isinstance(max_valid, int) or max_valid < 1:
        raise ValueError("max_valid must be positive")
    records, valid = [], []
    stop_reason = None
    try:
        for proposal in deterministic_candidates(
                original, space, radii=radii,
                samples_per_radius=samples_per_radius, seed=seed):
            row = evaluator.evaluate(proposal["q"])
            row.update(source=proposal["source"], radius=proposal["radius"],
                       sample_index=proposal["sample_index"],
                       normalized_change=normalized_change(proposal["q"], original, space))
            if "axis" in proposal:
                row.update(axis=proposal["axis"], sign=proposal["sign"])
            records.append(row)
            if row["valid"]:
                valid.append(row)
                if len(valid) >= max_valid:
                    stop_reason = "valid_candidate_target"
                    break
    except LimitReached as exc:
        stop_reason = str(exc)
    if stop_reason is None:
        stop_reason = "candidate_stream_exhausted"
    valid.sort(key=lambda r: (
        r["normalized_change"],
        -float(r["minimum_distance_m"] if r["minimum_distance_m"] is not None else -np.inf),
        tuple(r["q"])))
    return dict(status=("valid_candidates_found" if valid else "unresolved_within_budget"),
                impossible=None, stop_reason=stop_reason,
                evaluated_candidates=len(records), valid_candidates=len(valid),
                records=records, ranked_valid=valid,
                budget=dict(candidates=evaluator.budget.candidates,
                            distance_queries=evaluator.budget.distance_queries,
                            wall_s=time.monotonic() - evaluator.budget.started))


def choose_canonical(search_result, hold_results):
    """Pick minimum-change candidate among hold-valid candidates.

    hold_results maps q tuple to a record with validity.passed.  Clearance is a
    secondary tie-break only; it is not a new universal threshold.
    """
    eligible = []
    for row in search_result.get("ranked_valid", []):
        hold = hold_results.get(tuple(float(x) for x in row["q"]))
        if hold and hold.get("validity", {}).get("passed"):
            eligible.append((row, hold))
    if not eligible:
        return None
    eligible.sort(key=lambda item: (
        item[0]["normalized_change"],
        -float(item[0]["minimum_distance_m"]),
        tuple(item[0]["q"])))
    row, hold = eligible[0]
    return dict(q=row["q"], search=row, hold=hold,
                selection_rule="minimum normalized joint change; native clearance secondary")


def hold_rows(q, seconds=2.0, command_hz=60):
    """Minimal immutable hold trace used to define a new start snapshot."""
    q = vector(q)
    if not math.isfinite(seconds) or seconds <= 0 or command_hz <= 0:
        raise ValueError("Positive hold duration/rate required")
    n = int(math.ceil(seconds * command_hz)) + 1
    return [dict(joints=q.tolist(), command=q.tolist(), phase="ready") for _ in range(n)]
