"""Opt-in, fail-closed nominal robot/environment clearance screening.

Uses the loaded MuJoCo collision geoms, not a second greenhouse model.
This is a discrete GT diagnostic, NOT a CCD certificate or a real sensor guard.
MuJoCo is imported lazily so configuration/sampling tests need only NumPy.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

SCHEMA = "environment_preflight_v1"
ENV_POLICY = "FARMILY_ENV_PREFLIGHT_POLICY"
MAX_REPORTED_VIOLATIONS = 64
CHECKER_REVISION = "audit_completion_v2"


@dataclass(frozen=True)
class Policy:
    # Required explicitly: there is no unmeasured 'safe clearance' default.
    clearance_m: float
    max_revolute_step_rad: float = 0.01
    max_prismatic_step_m: float = 0.001
    max_time_step_s: float = 1 / 120
    max_samples: int = 50000
    max_distance_queries: int = 2000000
    timeout_s: float = 120.0
    # Exact geom names and phase names only. No wildcard exclusions.
    allowed_contacts: tuple = ()

    def __post_init__(self):
        if not math.isfinite(self.clearance_m) or self.clearance_m < 0:
            raise ValueError("clearance_m must be finite and >= 0")
        for key in ("max_revolute_step_rad", "max_prismatic_step_m", "max_time_step_s", "timeout_s"):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
        for key in ("max_samples", "max_distance_queries"):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{key} must be a positive integer")
        for item in self.allowed_contacts:
            if not isinstance(item, dict) or set(item) != {"robot_geom", "environment_geom", "phases"}:
                raise ValueError("Each allowed contact needs exact robot_geom, environment_geom, phases")
            for key in ("robot_geom", "environment_geom"):
                if not isinstance(item[key], str) or not item[key] or any(c in item[key] for c in "*?["):
                    raise ValueError("Allowed contacts require exact, nonempty geom names")
            phases = item["phases"]
            if not isinstance(phases, (list, tuple)) or not phases or any(not isinstance(p, str) or not p or p == "*" for p in phases):
                raise ValueError("Allowed contacts require explicit nonempty phase names")

    @classmethod
    def from_dict(cls, obj):
        if not isinstance(obj, dict) or "clearance_m" not in obj:
            raise ValueError("Policy must explicitly set clearance_m")
        obj = dict(obj)
        obj["allowed_contacts"] = tuple(obj.get("allowed_contacts", ()))
        return cls(**obj)

    def snapshot(self):
        settings = asdict(self)
        encoded = json.dumps(settings, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return {"schema": SCHEMA, "settings": settings, "sha256": hashlib.sha256(encoded.encode()).hexdigest()}


def load_policy(manifest=None):
    """Recorded policy survives resume; a conflicting environment override fails."""
    saved = (manifest or {}).get("environment_preflight_policy")
    path = os.environ.get(ENV_POLICY)
    supplied = Policy.from_dict(json.loads(Path(path).read_text())) if path else None
    if saved is None:
        return supplied
    if saved.get("schema") != SCHEMA:
        raise ValueError("Unsupported recorded environment preflight schema")
    recorded = Policy.from_dict(saved["settings"])
    if recorded.snapshot()["sha256"] != saved.get("sha256"):
        raise ValueError("Recorded environment preflight policy hash mismatch")
    if supplied is not None and supplied.snapshot()["sha256"] != saved["sha256"]:
        raise ValueError("Environment policy differs from saved run; use a new output directory")
    return recorded


class BudgetExceeded(RuntimeError):
    pass


def plan_samples(commands, phases, duration_s, joint_steps, policy, command_hz=60.0):
    """Match RobotEngine's piecewise-linear command interpolation, including endpoints.

    Each phase transition uses BOTH adjacent policies (intersection of permissions).
    Spatial/time subdivision is discrete: it does not prove swept-volume clearance.
    """
    commands = np.asarray(commands, dtype=float)
    joint_steps = np.asarray(joint_steps, dtype=float)
    if commands.ndim != 2 or len(commands) < 2 or not np.isfinite(commands).all():
        raise ValueError("Need at least two finite command rows")
    if joint_steps.shape != (commands.shape[1],) or not np.isfinite(joint_steps).all() or np.any(joint_steps <= 0):
        raise ValueError("Invalid per-joint sampling limits")
    if len(phases) != len(commands) or any(not isinstance(p, str) or not p for p in phases):
        raise ValueError("Every command needs an explicit phase")
    if not math.isfinite(command_hz) or command_hz <= 0:
        raise ValueError("command_hz must be positive")
    end = (len(commands) - 1) / command_hz
    if not math.isfinite(duration_s) or duration_s <= 0 or duration_s > end + 1e-6:
        raise ValueError("Duration outside recorded commands")
    count = 1
    yield 0.0, commands[0].copy(), (phases[0],)
    for i in range(len(commands) - 1):
        t0 = i / command_hz
        if t0 >= duration_s:
            break
        t1 = min((i + 1) / command_hz, duration_s)
        q0 = commands[i]
        q1 = q0 + (commands[i + 1] - q0) * ((t1 - t0) * command_hz)
        pieces = max(1, math.ceil(float(np.max(np.abs(q1 - q0) / joint_steps))), math.ceil((t1 - t0) / policy.max_time_step_s))
        active = tuple(dict.fromkeys((phases[i], phases[i + 1])))
        for j in range(1, pieces + 1):
            if count >= policy.max_samples:
                raise BudgetExceeded("sample_budget")
            alpha = j / pieces
            count += 1
            yield t0 + alpha * (t1 - t0), q0 + alpha * (q1 - q0), active


def pair_allowed(policy, robot_geom, environment_geom, phases):
    allowed_phases = set()
    for item in policy.allowed_contacts:
        if item["robot_geom"] == robot_geom and item["environment_geom"] == environment_geom:
            allowed_phases.update(item["phases"])
    return bool(phases) and set(phases).issubset(allowed_phases)


def classify_environment_geom(name):
    """Name-based diagnostic class only; never changes gate/allow semantics."""
    if name.startswith("gutter_collision_"):
        return "gutter"
    if name.startswith("neighbor_stem_collision_"):
        return "neighbor_stem"
    if name.startswith("neighbor_truss_collision_fruit_"):
        return "neighbor_fruit"
    if name.startswith("neighbor_truss_collision_rachis_"):
        return "neighbor_rachis"
    if name.startswith("neighbor_truss_collision_pedicel_"):
        return "neighbor_pedicel"
    if name.startswith("neighbor_truss_collision_peduncle_"):
        return "neighbor_peduncle"
    if name.startswith("glb_col_") or "__glb_col_" in name:
        return "glb_plant"
    return "other"


def require_native_distance_pipeline(mj, model):
    """Do not silently change the physics model to fix a query backend setting."""
    disabled_bit = getattr(mj.mjtDisableBit, "mjDSBL_NATIVECCD", None)
    enabled_bit = getattr(mj.mjtEnableBit, "mjENBL_NATIVECCD", None)
    if disabled_bit is not None:
        enabled = not (int(model.opt.disableflags) & int(disabled_bit))
    elif enabled_bit is not None:
        enabled = bool(int(model.opt.enableflags) & int(enabled_bit))
    else:
        raise ValueError("Native convex distance capability is not verified for this MuJoCo version")
    if not enabled:
        raise ValueError("Legacy convex distance pipeline is not supported; model flags were NOT changed")


def nearby_indices(position, radius, env_positions, env_radii, threshold):
    """Bounding spheres only cull provably distant pairs; planes have inf radius."""
    position = np.asarray(position)
    env_positions, env_radii = np.asarray(env_positions), np.asarray(env_radii)
    if position.shape != (3,) or env_positions.shape != (len(env_radii), 3):
        raise ValueError("Invalid geometry position array")
    if not np.isfinite(position).all() or not np.isfinite(env_positions).all():
        raise ValueError("Non-finite geometry position")
    if not math.isfinite(radius) or radius < 0 or np.isnan(env_radii).any() or np.any(env_radii < 0):
        raise ValueError("Invalid geometry bounding radius")
    distances = np.linalg.norm(env_positions - position, axis=1)
    return np.flatnonzero(distances <= radius + env_radii + threshold)


def screen(backend, commands, phases, duration_s, policy, collect_all_violations=False):
    """Keep collision evidence and scan completeness separate, including on abort.

    Counts are sampled pair hits, not distinct physical contact events. Detailed
    first-hit records are capped, but class summaries and distinct-key counts
    continue across the visited path. A known violation cannot become unknown
    just because a later sample exceeds a budget or fails.
    """
    started = time.perf_counter()
    result = dict(schema=SCHEMA, checker_revision=CHECKER_REVISION,
                  status="inconclusive", passed=False, complete=False,
                  policy=policy.snapshot(), samples_checked=0, distance_queries=0,
                  last_sample_time_s=None, last_completed_sample_time_s=None,
                  first_violation=None, violations=[], violation_summary={},
                  scan_stop_reason=None, minimum_queried_distance_m=None,
                  scope="GT initial environment; nominal commands; discrete screening only",
                  exclusions=["visual-only geoms", "environment/environment pairs", "robot self-collision"],
                  warnings=["No continuous collision guarantee", "No plant deformation or actuator tracking simulated",
                            "No real-observation collision model", "Missing visual-only colliders remain untested"])
    seen_keys, stored_records = set(), {}
    class_counts, class_minima, class_first, class_worst = {}, {}, {}, {}
    try:
        result["inventory"] = backend.inventory
        result["backend"] = backend.version
        if not backend.robot_names or not backend.environment_names:
            raise ValueError("Need nonempty robot AND environment collision geometry")
        for item in policy.allowed_contacts:
            if item["robot_geom"] not in backend.robot_names or item["environment_geom"] not in backend.environment_names:
                raise ValueError("Allowlist refers to missing or misclassified geometry")
            if not set(item["phases"]).issubset(phases):
                raise ValueError("Allowlist refers to a phase absent from this trace")
        query_limit = policy.clearance_m + 0.01
        result["distance_query_horizon_m"] = query_limit
        # A policy is fixed for this screen. Compile exact pairs once instead
        # of scanning hundreds of permission records for every distance query.
        permissions = {}
        for item in policy.allowed_contacts:
            key = (item["robot_geom"], item["environment_geom"])
            permissions.setdefault(key, set()).update(item["phases"])
        for timestamp, q, active_phases in plan_samples(commands, phases, duration_s, backend.joint_steps, policy):
            if time.perf_counter() - started > policy.timeout_s:
                raise BudgetExceeded("time_budget")
            backend.set_robot(q)
            result["samples_checked"] += 1
            result["last_sample_time_s"] = timestamp
            for robot_index, robot_name in enumerate(backend.robot_names):
                indices = nearby_indices(backend.robot_positions[robot_index], backend.robot_radii[robot_index],
                                         backend.environment_positions, backend.environment_radii, query_limit)
                for env_index in indices:
                    env_name = backend.environment_names[env_index]
                    if active_phases and set(active_phases).issubset(permissions.get((robot_name, env_name), ())):
                        continue
                    if result["distance_queries"] >= policy.max_distance_queries:
                        raise BudgetExceeded("distance_query_budget")
                    if time.perf_counter() - started > policy.timeout_s:
                        raise BudgetExceeded("time_budget")
                    distance, segment = backend.distance(robot_index, int(env_index), query_limit)
                    result["distance_queries"] += 1
                    if not math.isfinite(distance) or not np.isfinite(segment).all():
                        raise ValueError("Non-finite distance result")
                    if distance < query_limit:
                        previous = result["minimum_queried_distance_m"]
                        result["minimum_queried_distance_m"] = distance if previous is None else min(previous, distance)
                    if distance <= policy.clearance_m:
                        env_class = classify_environment_geom(env_name)
                        record = dict(time_s=timestamp, phases=list(active_phases),
                            robot_geom=robot_name, environment_geom=env_name, environment_class=env_class,
                            distance_m=distance, command=q.tolist(),
                            segment_world=np.asarray(segment).reshape(2, 3).tolist())
                        if result["first_violation"] is None:
                            result["first_violation"] = record
                        result.update(status="blocked", reason="clearance_violation")
                        class_counts[env_class] = class_counts.get(env_class, 0) + 1
                        class_first.setdefault(env_class, record)
                        if env_class not in class_minima or distance < class_minima[env_class]:
                            class_minima[env_class] = distance
                            class_worst[env_class] = record
                        key = (robot_name, env_name, tuple(active_phases))
                        if key not in seen_keys:
                            seen_keys.add(key)
                            if len(result["violations"]) < MAX_REPORTED_VIOLATIONS:
                                # Keep distance_m/time_s as the original first hit.
                                stored = dict(record, last_time_s=timestamp, sample_hit_count=0,
                                              minimum_distance_m=distance, minimum_time_s=timestamp)
                                stored_records[key] = stored
                                result["violations"].append(stored)
                        if key in stored_records:
                            stored = stored_records[key]
                            stored["last_time_s"] = timestamp
                            stored["sample_hit_count"] += 1
                            if distance < stored["minimum_distance_m"]:
                                stored["minimum_distance_m"] = distance
                                stored["minimum_time_s"] = timestamp
                        if not collect_all_violations:
                            result["scan_stop_reason"] = "first_violation"
                            return result
            result["last_completed_sample_time_s"] = timestamp
        result["complete"] = True
        result["scan_stop_reason"] = "end_of_path"
        if result["first_violation"] is None:
            result.update(status="sampled_clear", passed=True, reason="all_samples_passed")
    except BudgetExceeded as exc:
        result["scan_stop_reason"] = str(exc)
        if result["first_violation"] is None:
            result["reason"] = str(exc)
    except Exception as exc:
        result.update(scan_stop_reason="check_error", error=f"{type(exc).__name__}: {exc}")
        if result["first_violation"] is None:
            result["reason"] = "check_error"
    finally:
        omitted = len(seen_keys) - len(result["violations"])
        result["violation_summary"] = dict(
            sample_hits_by_environment_class=class_counts,
            minimum_distance_m_by_environment_class=class_minima,
            first_by_environment_class=class_first,
            worst_by_environment_class=class_worst,
            observed_unique_pair_phase_count=len(seen_keys),
            reported_unique_pair_phase_count=len(result["violations"]),
            omitted_unique_pair_phase_count=omitted,
            details_truncated=omitted > 0,
            reported_limit=MAX_REPORTED_VIOLATIONS,
            full_path_requested=bool(collect_all_violations),
            full_path_collected=bool(collect_all_violations and result["complete"]),
            counts_scope="visited samples only; pair/phase keys and sampled pair hits, not physical contact events",
        )
        result["wall_s"] = time.perf_counter() - started
    return result


class MuJoCoScene:
    """Separate MjData at the SAME initial state as RobotEngine.reset()."""
    def __init__(self, engine, policy):
        import mujoco as mj
        self.mj = mj
        self.model = m = engine.model
        require_native_distance_pipeline(mj, m)
        self.data = d = mj.MjData(m)
        mj.mj_resetData(m, d)
        self.qids = np.asarray(engine.qids, dtype=int)
        d.qpos[self.qids] = engine.initial
        mj.mj_forward(m, d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.geom_xpos).all() or np.any(d.warning.number):
            raise ValueError("Initial model state has non-finite values or MuJoCo warnings")
        self.version = "MuJoCo " + mj.__version__
        robot_bodies = set()
        for bid in range(1, m.nbody):
            name = m.body(bid).name or ""
            if name.startswith("Robot_") or name == "Hook" or int(m.body_parentid[bid]) in robot_bodies:
                robot_bodies.add(bid)
        # Include explicitly paired geoms even if their contact masks are zero.
        paired = set(map(int, np.r_[m.pair_geom1, m.pair_geom2]))
        active = [i for i in range(m.ngeom) if int(m.geom_contype[i]) or int(m.geom_conaffinity[i]) or i in paired]
        self.robot_ids = np.array([i for i in active if int(m.geom_bodyid[i]) in robot_bodies], dtype=int)
        self.environment_ids = np.array([i for i in active if int(m.geom_bodyid[i]) not in robot_bodies], dtype=int)
        supported = {int(getattr(mj.mjtGeom, "mjGEOM_" + k)) for k in ("PLANE", "SPHERE", "CAPSULE", "ELLIPSOID", "CYLINDER", "BOX", "MESH")}
        if any(int(m.geom_type[i]) not in supported for i in active):
            raise ValueError("Unsupported geom type; hfield/SDF/flex need a separate tested checker")
        if getattr(m, "nflex", 0):
            raise ValueError("Flex collision surfaces are not covered by this checker")
        if any(int(m.geom_type[i]) == int(mj.mjtGeom.mjGEOM_PLANE) for i in self.robot_ids):
            raise ValueError("Robot plane geom is unsupported")
        names = lambda ids: [m.geom(int(i)).name or f"#geom_{i}" for i in ids]
        self.robot_names = names(self.robot_ids)
        self.environment_names = names(self.environment_ids)
        self.robot_radii = np.array(m.geom_rbound[self.robot_ids], dtype=float)
        self.environment_radii = np.array(m.geom_rbound[self.environment_ids], dtype=float)
        for j, gid in enumerate(self.environment_ids):
            if int(m.geom_type[gid]) == int(mj.mjtGeom.mjGEOM_PLANE):
                self.environment_radii[j] = np.inf
        self.environment_positions = d.geom_xpos[self.environment_ids].copy()
        self.robot_positions = d.geom_xpos[self.robot_ids].copy()
        self.joint_steps = []
        for qid in self.qids:
            found = np.flatnonzero(m.jnt_qposadr == qid)
            if len(found) != 1:
                raise ValueError("Robot qpos index must address one scalar joint")
            joint_type = int(m.jnt_type[found[0]])
            if joint_type not in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)):
                raise ValueError("Only scalar robot hinge/slide joints supported")
            self.joint_steps.append(policy.max_prismatic_step_m if joint_type == int(mj.mjtJoint.mjJNT_SLIDE) else policy.max_revolute_step_rad)
        # Do not ignore MuJoCo masks between active geoms: an excluded pair is
        # still a geometric obstacle unless explicitly allowed in our policy.
        self.inventory = dict(robot_collision_geoms=self.robot_names, environment_collision_geoms=self.environment_names,
                              ignored_visual_geom_count=m.ngeom - len(active),
                              mesh_semantics="MuJoCo collision mesh (convex), not visual mesh triangles",
                              native_convex_distance=True)
        self.segment = np.empty(6)

    def set_robot(self, q):
        self.data.qpos[self.qids] = q
        self.mj.mj_kinematics(self.model, self.data)
        self.robot_positions = self.data.geom_xpos[self.robot_ids]

    def distance(self, robot_index, environment_index, limit):
        self.segment[:] = 0
        value = self.mj.mj_geomDistance(self.model, self.data, int(self.robot_ids[robot_index]),
                                      int(self.environment_ids[environment_index]), limit, self.segment)
        return float(value), self.segment.copy()


def check_engine(engine, rows, seconds, policy, collect_all_violations=False):
    try:
        commands = np.asarray([row["command"] for row in rows], dtype=float)
        if commands.ndim != 2 or commands.shape[1] != len(engine.qids):
            raise ValueError("Trace/robot joint count mismatch")
        if not np.allclose(commands[0], engine.initial, rtol=0, atol=1e-12):
            raise ValueError("Trace first command differs from reset robot pose")
        phases = [row["phase"] for row in rows]
        scene = MuJoCoScene(engine, policy)
        result = screen(scene, commands, phases, seconds, policy, collect_all_violations=collect_all_violations)
    except Exception as exc:
        result = dict(schema=SCHEMA, status="inconclusive", passed=False, complete=False,
                      reason="setup_error", error=f"{type(exc).__name__}: {exc}", policy=policy.snapshot())
    return result


def save_report(folder, result):
    """Separate diagnostics: never pretend a rejected plan was physically executed."""
    import html
    folder = Path(folder)
    (folder / "environment_preflight.json").write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    pretty = html.escape(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    (folder / "environment_preflight.html").write_text(
        '<meta charset="utf-8"><title>Environment preflight</title>'
        '<h1>Nominal path / environment check</h1><p>GT, frozen environment; discrete samples. '
        'NOT a physics replay, CCD guarantee, or harvest result.</p>'
        '<p>Inspect first_violation for time, phase, geom pair and robot command. '
        'A sampled_clear result does not guarantee a collision-free rollout.</p><pre>' + pretty + '</pre>', encoding="utf-8")


def save_run_summary(root, results):
    """Companion report with honest denominators, linked from the existing dashboard."""
    import html
    root = Path(root)
    checked = [r for r in results if "environment_preflight" in r]
    if not checked:
        return
    rows = []
    for row in checked:
        audit = row["environment_preflight"]
        hit = audit.get("first_violation") or {}
        identity = str(row["candidate_id"])
        values = [identity, audit["status"], row.get("physics_executed", False),
                  hit.get("time_s"), hit.get("phases"), hit.get("robot_geom"),
                  hit.get("environment_geom"), hit.get("distance_m"), audit.get("reason")]
        cells = "".join("<td>" + html.escape(str(v)) + "</td>" for v in values)
        from urllib.parse import quote
        url = "candidates/" + quote(identity, safe="") + "/environment_preflight.html"
        rows.append("<tr>" + cells + '<td><a href="' + url + '">details</a></td></tr>')
    executed = sum(r.get("physics_executed") is True for r in checked)
    page = ('<meta charset="utf-8"><h1>Environment preflight</h1>'
            f'<p>Checked decisions: {len(checked)}. Physics executed: {executed}. '
            f'Held before physics: {len(checked)-executed}.</p>'
            '<p>GT / frozen scene / discrete nominal path. A held decision is NOT a failed physical harvest. '
            'The legacy dashboard may count decisions as attempts; use the counters here for guarded runs.</p>'
            '<table border="1"><tr><th>Candidate</th><th>Status</th><th>Physics executed</th><th>Time (s)</th>'
            '<th>Phases</th><th>Robot geom</th><th>Environment geom</th><th>Distance (m)</th><th>Reason</th><th>Audit</th></tr>'
            + "".join(rows) + '</table>')
    (root / "environment_preflight_index.html").write_text(page, encoding="utf-8")
    index = root / "index.html"
    if index.exists():
        link = '<p><a href="environment_preflight_index.html">Environment preflight: checked / held / physically executed</a></p>'
        text = index.read_text(encoding="utf-8")
        text = text.replace("</body>", link + "</body>") if "</body>" in text else text + link
        index.write_text(text, encoding="utf-8")
