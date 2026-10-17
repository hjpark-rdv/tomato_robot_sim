"""Read-only preparation of existing S300 commands for a Newton comparison.

This module reuses the portable MuJoCo adapter and its command sampler. It does
not create trajectories, advance physics, update source caches or initialize an
Isaac application. The output is an immutable comparison input, not a dataset.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import shutil
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


REPO = Path(__file__).resolve().parents[2]
ADAPTER = REPO / "mujoco-benchmark/scripts/s300_smoke"
if str(ADAPTER) not in sys.path:
    sys.path.insert(0, str(ADAPTER))
from plan_adapter import DOF_NAMES, sample_plan


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + "\n")


def _verified(path, expected=None):
    path = Path(path).resolve(strict=True)
    digest = sha256(path)
    if expected is not None and digest != expected:
        raise ValueError("Source SHA mismatch: " + str(path))
    return dict(path=str(path), sha256=digest)


def _source_files(model_dir, meta):
    model_dir = Path(model_dir).resolve()
    assembly = Path(meta["source_assembly"])
    trial = Path(meta["source_trial"])
    files = {
        "model_xml": _verified(model_dir / "scene.xml", meta["model_sha256"]),
        "export_metadata": _verified(model_dir / "export.json"),
        "source_usdc": _verified(assembly / "composed.usdc", meta["source_usdc_sha256"]),
        "source_manifest": _verified(meta.get("source_manifest", assembly / "motion_manifest.json"),
                                     meta.get("source_manifest_sha256")),
    }
    if meta.get("source_states_kind") == "planning_only_no_recorded_states":
        if meta.get("source_states_sha256") is not None or meta.get("trace_sha256") is not None:
            raise ValueError("Planning-only input cannot declare recorded-state hashes")
    else:
        files["source_states"] = _verified(trial / "states.npz", meta["source_states_sha256"])
    for index, window in enumerate(meta.get("windows", [])):
        files[f"window_{index:02d}_manifest"] = _verified(window["source_manifest"],
            window.get("source_manifest_sha256"))
        files[f"window_{index:02d}_plan"] = _verified(window["source_plan"],
            window.get("source_plan_sha256"))
    for name, path, expected in [
        ("source_plan", model_dir / "source_plan.json", meta.get("source_plan_sha256")),
        ("tool_trace", model_dir / "tool_trace.npz", meta.get("trace_sha256")),
        ("source_trial_plan", trial / "plan.json", None),
    ]:
        if path.is_file():
            files[name] = _verified(path, expected)
    return files


def compare_physical_model(source, candidate):
    """Prove visual removal preserves every active collision/physical field.

    Compiler storage addresses, render materials and acceleration-tree layout
    may change. Physical objects and mesh data are compared by source names;
    no numerical tolerance or approximate geometry is used to obtain a pass.
    """
    import mujoco as mj

    checks, mismatches = {}, []
    reference_digest, actual_digest = hashlib.sha256(), hashlib.sha256()
    def same(label, a, b):
        a, b = np.asarray(a), np.asarray(b)
        passed = bool(a.shape == b.shape and np.array_equal(a, b))
        checks[label] = passed
        if not passed:
            mismatches.append(label)
        for digest, value in ((reference_digest, a), (actual_digest, b)):
            digest.update(f"{label}:{value.dtype}:{value.shape}".encode())
            digest.update(value.tobytes())
    def names(model, kind, count):
        return [mj.mj_id2name(model, kind, i) or "" for i in range(count)]

    for count in ("nbody", "njnt", "nq", "nv", "nu", "neq", "nexclude", "nsite", "ntendon", "npair", "nkey"):
        same(count, getattr(source, count), getattr(candidate, count))
    for kind, count in ((mj.mjtObj.mjOBJ_BODY, "nbody"), (mj.mjtObj.mjOBJ_JOINT, "njnt"),
                         (mj.mjtObj.mjOBJ_ACTUATOR, "nu"), (mj.mjtObj.mjOBJ_EQUALITY, "neq"),
                         (mj.mjtObj.mjOBJ_SITE, "nsite")):
        same("names." + count, names(source, kind, getattr(source, count)),
             names(candidate, kind, getattr(candidate, count)))
    # This asset contains body exclusions rather than explicit geom-pair rules.
    # Reject an unfamiliar pair topology rather than comparing remapped IDs.
    if source.npair or candidate.npair:
        raise ValueError("Visual separation proof requires explicit geom-pair remapping support")
    body_layout = {"body_geomadr", "body_geomnum", "body_bvhadr", "body_bvhnum"}
    site_visual = {"site_matid", "site_rgba"}
    for key in dir(source):
        if ((key.startswith(("body_", "jnt_", "dof_", "actuator_", "eq_", "site_", "tendon_"))
             and key not in body_layout | site_visual)
                or key in ("qpos0", "qpos_spring", "exclude_signature")):
            value = getattr(source, key)
            if isinstance(value, np.ndarray):
                same(key, value, getattr(candidate, key))
    for key in dir(source.opt):
        if key.startswith("_"):
            continue
        value = getattr(source.opt, key)
        if isinstance(value, (np.ndarray, int, float, bool, np.integer, np.floating)):
            same("option." + key, value, getattr(candidate.opt, key))
    for key in ("meanmass", "meaninertia"):
        same("stat." + key, getattr(source.stat, key), getattr(candidate.stat, key))

    original_active = np.flatnonzero((source.geom_contype != 0) | (source.geom_conaffinity != 0))
    new_active = np.flatnonzero((candidate.geom_contype != 0) | (candidate.geom_conaffinity != 0))
    left = {source.geom(int(g)).name: int(g) for g in original_active}
    right = {candidate.geom(int(g)).name: int(g) for g in new_active}
    same("active_geom_names", sorted(left), sorted(right))
    missing = sorted(set(left) - set(right))
    added = sorted(set(right) - set(left))
    common = sorted(set(left) & set(right))
    li, ri = [left[name] for name in common], [right[name] for name in common]
    for key in dir(source):
        # dataid is remapped to asset names below. matid/rgba are appearance.
        if key.startswith("geom_") and key not in ("geom_dataid", "geom_matid", "geom_rgba"):
            value = getattr(source, key)
            if isinstance(value, np.ndarray):
                same(key, value[li], getattr(candidate, key)[ri])
    physical_meshes = {}
    for name in common:
        a, b = left[name], right[name]
        if source.geom_type[a] == mj.mjtGeom.mjGEOM_MESH:
            ma, mb = int(source.geom_dataid[a]), int(candidate.geom_dataid[b])
            same("mesh_asset_for." + name, source.mesh(ma).name, candidate.mesh(mb).name)
            physical_meshes[source.mesh(ma).name] = (ma, mb)
        elif source.geom_dataid[a] >= 0 or candidate.geom_dataid[b] >= 0:
            raise ValueError("Visual separation proof needs a new non-mesh asset mapping: " + name)
    for name, (a, b) in physical_meshes.items():
        prefix = "mesh." + name + "."
        for field in ("mesh_pos", "mesh_quat", "mesh_scale", "mesh_extrema"):
            same(prefix + field, getattr(source, field)[a], getattr(candidate, field)[b])
        for data, address, count in (("mesh_vert", "mesh_vertadr", "mesh_vertnum"),
                ("mesh_face", "mesh_faceadr", "mesh_facenum"),
                ("mesh_normal", "mesh_normaladr", "mesh_normalnum"),
                ("mesh_facenormal", "mesh_faceadr", "mesh_facenum"),
                ("mesh_polynormal", "mesh_polyadr", "mesh_polynum")):
            aa, ac = int(getattr(source, address)[a]), int(getattr(source, count)[a])
            ba, bc = int(getattr(candidate, address)[b]), int(getattr(candidate, count)[b])
            same(prefix + data, getattr(source, data)[aa:aa + ac], getattr(candidate, data)[ba:ba + bc])
        graphs = []
        for model, mesh in ((source, a), (candidate, b)):
            adr = int(model.mesh_graphadr[mesh])
            size = (2 + 3 * int(model.mesh_graph[adr]) + 6 * int(model.mesh_graph[adr + 1])) if adr >= 0 else 0
            graphs.append(model.mesh_graph[adr:adr + size] if adr >= 0 else np.asarray([], dtype=np.int32))
        same(prefix + "convex_graph", *graphs)
        # Polygon adjacency is packed globally. Compare each mesh's counts and
        # vertex/polygon-index payload, excluding the relocated storage offsets.
        polygon_rows, vertex_maps = [], []
        for model, mesh in ((source, a), (candidate, b)):
            pa, pn = int(model.mesh_polyadr[mesh]), int(model.mesh_polynum[mesh])
            counts = model.mesh_polyvertnum[pa:pa + pn]
            start = int(model.mesh_polyvertadr[pa]) if pn else 0
            total = int(counts.sum())
            polygon_rows.append((counts, model.mesh_polyvert[start:start + total]))
            va, vn = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
            counts = model.mesh_polymapnum[va:va + vn]
            addresses = model.mesh_polymapadr[va:va + vn]
            nonempty = counts > 0
            start = int(addresses[nonempty].min()) if np.any(nonempty) else 0
            total = int(counts.sum())
            vertex_maps.append((counts, model.mesh_polymap[start:start + total]))
            if model.mesh_octnum[mesh]:
                raise ValueError("Visual separation proof requires explicit SDF octree support")
        same(prefix + "polygon_vertex_counts", polygon_rows[0][0], polygon_rows[1][0])
        same(prefix + "polygon_vertices", polygon_rows[0][1], polygon_rows[1][1])
        same(prefix + "vertex_polygon_counts", vertex_maps[0][0], vertex_maps[1][0])
        same(prefix + "vertex_polygon_indices", vertex_maps[0][1], vertex_maps[1][1])
    removed = sorted(set(names(source, mj.mjtObj.mjOBJ_GEOM, source.ngeom)) -
                     set(names(candidate, mj.mjtObj.mjOBJ_GEOM, candidate.ngeom)))
    result = dict(schema="s300-discard-visual-physical-proof-v1", passed=not mismatches,
        exact_numeric_comparison=True, source_geoms=source.ngeom, variant_geoms=candidate.ngeom,
        source_active_geoms=len(left), variant_active_geoms=len(right),
        removed_visual_geom_names=removed, missing_active_geoms=missing, unexpected_active_geoms=added,
        physical_mesh_assets_compared=len(physical_meshes), checks=len(checks),
        mismatched_fields=mismatches, source_semantic_sha256=reference_digest.hexdigest(),
        variant_semantic_sha256=actual_digest.hexdigest(),
        original_geometry_kept_for_display=True, collision_approximation=False,
        ignored_compiler_layout_fields=sorted(body_layout),
        interpretation="Non-colliding render geometry removal only; every active geometry and physical field verified by name")
    if mismatches:
        raise ValueError("Discard-visual physical semantics differ: " + ", ".join(mismatches[:12]))
    return result


def discard_noncolliding_visual_geoms(source, spec):
    """Compile a render-free model while retaining source dynamic constants.

    MuJoCo computes angular ``dof_length`` from *all* geoms, including visual
    ones (engine_setconst.c). It weights velocity for sleeping, so retain the
    source values explicitly instead of treating this difference as harmless.
    The runners upload this verified MJB as their physical source_model; the
    MJCF remains the import description and does not encode this derived array.
    """
    spec.compiler.discardvisual = True
    model = spec.compile()
    if source.nv != model.nv:
        raise ValueError("Visual removal unexpectedly changed the DOF count")
    compiled_length = model.dof_length.copy()
    model.dof_length[:] = source.dof_length
    proof = compare_physical_model(source, model)
    proof["restored_compiler_derived_fields"] = {
        "dof_length": dict(
            changed_dof_indices=np.flatnonzero(compiled_length != source.dof_length).tolist(),
            original=source.dof_length.tolist(),
            discarded_visual_compiler_value=compiled_length.tolist(),
            restored=model.dof_length.tolist(),
            reason="Preserve the original angular velocity scale used for sleeping",
            primary_source="https://github.com/google-deepmind/mujoco/blob/3.15.0/src/engine/engine_sleep.c",
        )}
    proof["authoritative_physics_input"] = "input_model.mjb; XML-only recompilation does not retain derived dof_length restoration"
    return model, proof


def initialize_native_model(model_dir, hz, *, discard_visual_geoms=False, allow_multiple_windows=False):
    """Return a native starting state and the existing Session gravity preload.

    Uses ``mj_forward``/Jacobians only. A valid existing MJB may be read, but
    neither the compiled cache nor any other source file is written. The fruit
    angular damping setup is exactly the existing ``Attachments`` adapter.
    ``qfrc_applied`` is returned separately because the executor applies the
    fixed preload before each physics step. Dynamic robot bias feedforward is
    still the executor's responsibility; it must not be frozen from this state.
    """
    import mujoco as mj
    from run_smoke import Attachments

    if not np.isfinite(hz) or hz <= 0:
        raise ValueError("hz must be positive and finite")
    directory = Path(model_dir).resolve(strict=True)
    meta = json.loads((directory / "export.json").read_text())
    provenance = _source_files(directory, meta)
    if not meta.get("robot", {}).get("dynamic_tool"):
        raise ValueError("A finite-actuator dynamic S300 model is required")
    if len(meta.get("windows", [])) > 1 and not allow_multiple_windows:
        raise ValueError("Prepare one existing plant window per comparison input; opt in to allow_multiple_windows for persistent route diagnostics")
    cache, certificate = directory / "compiled_scene.mjb", directory / "compiled_scene.json"
    cached = json.loads(certificate.read_text()) if certificate.is_file() else {}
    valid_cache = (cache.is_file() and cached.get("model_xml_sha256") == meta["model_sha256"]
                   and cached.get("mujoco_version") == mj.__version__
                   and cached.get("mjb_sha256") == sha256(cache))
    model = (mj.MjModel.from_binary_path(str(cache)) if valid_cache else
             mj.MjModel.from_xml_path(str(directory / "scene.xml")))
    variant_proof = None
    if discard_visual_geoms:
        original = model
        spec = mj.MjSpec.from_file(str(directory / "scene.xml"))
        model, variant_proof = discard_noncolliding_visual_geoms(original, spec)
        meta = dict(meta, physics_model_variant=dict(mode="discard_noncolliding_visual_geoms",
            source_model_sha256=meta["model_sha256"], compiler_discardvisual=True,
            physical_semantics_verified=variant_proof["passed"], collision_approximation=False,
            native_geoms=model.ngeom, native_active_geoms=variant_proof["variant_active_geoms"]))
        del original, spec
    model.opt.timestep = 1. / hz
    if not hasattr(model, "geom_surfacevel"):
        raise RuntimeError("Preparation requires the existing MuJoCo surface-velocity adapter")
    model.flg_surfacevel = True
    model.geom_surfacevel[:] = 0
    data = mj.MjData(model)
    for name, value in meta["robot"]["initial_q"].items():
        data.qpos[model.joint(name).qposadr] = value
    for name, value in meta["robot"]["initial_qvel"].items():
        data.qvel[model.joint(name).dofadr] = value
    qadr = np.asarray([int(model.joint(name).qposadr[0]) for name in DOF_NAMES])
    vadr = np.asarray([int(model.joint(name).dofadr[0]) for name in DOF_NAMES])
    aids = np.asarray([model.actuator("drive_" + name).id for name in DOF_NAMES])
    attachments = Attachments(model, meta["attachments"])
    mj.mj_forward(model, data)
    plant_dofs = np.asarray([i for i in range(model.nv) if i not in vadr and
                            model.jnt_type[model.dof_jntid[i]] != mj.mjtJoint.mjJNT_FREE], dtype=int)
    preload = np.zeros(model.nv)
    preload[plant_dofs] = data.qfrc_bias[plant_dofs]
    for body, parent in zip(attachments.ids, attachments.parents):
        jp = np.zeros((3, model.nv))
        jr = np.zeros_like(jp)
        mj.mj_jac(model, data, jp, jr, data.xpos[body], int(parent))
        preload[plant_dofs] += (jp.T @ (-model.body_mass[body] * model.opt.gravity))[plant_dofs]
    if data.time != 0 or not all(np.isfinite(x).all() for x in (data.qpos, data.qvel, preload)):
        raise ValueError("Invalid offline initial state")
    return dict(model=model, data=data, metadata=meta, provenance=provenance,
                preload=preload, plant_dofs=plant_dofs, robot_qpos_addresses=qadr,
                robot_dof_addresses=vadr, actuator_ids=aids, attachments=attachments,
                loaded_cached_model=bool(valid_cache), mujoco_version=mj.__version__,
                visual_removal_proof=variant_proof)


def _window_body_sources(meta):
    """Exact source ownership and display namespace of each persistent window."""
    windows = meta.get("windows", [])
    if not windows:
        manifest = json.loads(Path(meta["source_manifest"]).read_text())
        yield dict(prefix="", display_root=manifest["plant"]["root"]), manifest
        return
    for window in windows:
        manifest = json.loads(Path(window["source_manifest"]).read_text())
        source_root = manifest["plant"]["root"]
        display_root = window.get("display_root", source_root)
        if (not isinstance(display_root, str) or not display_root.startswith("/World/")
                or display_root.endswith("/")):
            raise ValueError("Each window requires an absolute /World display_root")
        yield dict(window, display_root=display_root), manifest


def _display_path(path, source_root, display_root):
    if path == source_root:
        return display_root
    if not isinstance(path, str) or not path.startswith(source_root + "/"):
        raise ValueError("Window body/joint path is outside its exact source plant: " + str(path))
    return display_root + path[len(source_root):]


def window_fruit_specs(meta):
    """Return native fruit identities with exact per-window display joint paths."""
    rows, names, joints = [], set(), set()
    for window, manifest in _window_body_sources(meta):
        source_root = manifest["plant"]["root"]
        prefix = window.get("prefix", "")
        specs = window.get("fruit_specs", meta.get("fruit_specs", []))
        source_fruits = {item["name"]: item for item in manifest["plant"]["fruits"]}
        for item in specs:
            if item["name"] not in source_fruits:
                raise ValueError("Window fruit is absent from its exact source manifest: " + item["name"])
            source = source_fruits[item["name"]]
            row = dict(item, name=prefix + item["name"])
            for key in ("path", "joint", "anchor"):
                if key in source:
                    if key in item and item[key] != source[key]:
                        raise ValueError("Window fruit source identity mismatch: " + key)
                    row[key] = _display_path(source[key], source_root, window["display_root"])
            if row["name"] in names or row.get("joint") in joints:
                raise ValueError("Duplicate persistent-window fruit identity")
            names.add(row["name"]); joints.add(row.get("joint")); rows.append(row)
    return rows


def body_path_mapping(model, meta):
    """Map every exact source owner through its declared persistent namespace."""
    result = dict(meta["robot"]["body_names"])
    native = set(meta["rods"] + meta["fruits"])
    for window, source_manifest in _window_body_sources(meta):
        source_root = source_manifest["plant"]["root"]
        prefix = window.get("prefix", "")
        for item in source_manifest["plant"]["bodies"] + source_manifest["plant"]["fruits"]:
            name = prefix + item["name"]
            if name in native:
                path = _display_path(item["path"], source_root, window["display_root"])
                old = result.setdefault(path, name)
                if old != name:
                    raise ValueError("Ambiguous source body mapping: " + path)
    for path, name in result.items():
        if model.body(name).id <= 0:
            raise ValueError("Mapped body is not an integrated native body: " + path)
    if len(set(result.values())) != len(result):
        raise ValueError("Source body mapping is not one-to-one")
    return result


def excluded_body_evidence(meta, paths):
    """Explain omissions already made by the portable exporter, with lineage.

    Historical loose fruit are not silently relabelled as current harvests or
    active-plant failures. Unexplained omissions remain explicitly unresolved.
    """
    manifest = json.loads(Path(meta["source_manifest"]).read_text())
    route = manifest.get("route_checkpoint", {})
    outgoing = route.get("outgoing", {})
    detached = {row["path"]: row for row in outgoing.get("detached_fruits", [])}
    checkpoint = Path(route.get("checkpoint", "")) / "checkpoint.json"
    expected = outgoing.get("checkpoint_metadata_sha256")
    checkpoint_proof = None
    checkpoint_metadata = None
    if expected:
        checkpoint_proof = dict(path=str(checkpoint), expected_metadata_sha256=expected,
                                verified=False)
        if checkpoint.is_file():
            # Existing checkpoints hash canonical JSON *without* their own
            # metadata_sha256 field. Its pretty-printed file byte hash belongs
            # to a different domain and is also preserved as evidence.
            from native_route_checkpoint import _json_bytes
            checkpoint_proof["file_sha256"] = sha256(checkpoint)
            try:
                checkpoint_metadata = json.loads(checkpoint.read_text())
                unsigned = {key: value for key, value in checkpoint_metadata.items()
                            if key != "metadata_sha256"}
                computed = hashlib.sha256(_json_bytes(unsigned)).hexdigest()
                checkpoint_proof.update(recorded_metadata_sha256=checkpoint_metadata.get("metadata_sha256"),
                    computed_metadata_sha256=computed,
                    metadata_sha_verified=(expected == computed == checkpoint_metadata.get("metadata_sha256")))
                data_path = checkpoint.parent / "checkpoint.npz"
                actual_npz = sha256(data_path) if data_path.is_file() else None
                expected_npz = checkpoint_metadata.get("checkpoint_npz_sha256")
                checkpoint_proof.update(expected_npz_sha256=expected_npz, actual_npz_sha256=actual_npz,
                    npz_sha_verified=bool(expected_npz and actual_npz == expected_npz))
                checkpoint_proof["verified"] = (checkpoint_proof["metadata_sha_verified"] and
                                                  checkpoint_proof["npz_sha_verified"])
            except (OSError, ValueError, TypeError, AttributeError) as error:
                checkpoint_proof["verification_error"] = str(error)
        else:
            checkpoint_proof["verification_error"] = "Historical checkpoint file is absent"
        checkpoint_proof["scope"] = "Auxiliary historical loose-fruit lineage; not a current physics input"
    evidence = []
    for path in paths:
        fruit = detached.get(path)
        inline_events = fruit.get("native_break_events", []) if fruit else []
        events_match = bool(checkpoint_metadata is not None and inline_events and
            all(event in checkpoint_metadata.get("break_events", []) for event in inline_events))
        proved = bool(fruit and inline_events and checkpoint_proof and
                      checkpoint_proof["verified"] and events_match)
        classification = ("historical_native_detached_fruit" if proved else
            "historical_native_detached_fruit_unverified_history" if inline_events else
            "unexplained_portable_export_omission")
        evidence.append(dict(source_body=path,
            classification=classification,
            omitted_by="Existing source MuJoCo adapter before this Newton experiment",
            native_break_events=inline_events, checkpoint_break_events_match=events_match,
            historical_provenance_verified=proved,
            checkpoint_metadata=checkpoint_proof, checkpoint_time_s=outgoing.get("checkpoint_time_s"),
            source_plant=fruit.get("source_plant") if fruit else None,
            current_target_collision_coverage_claim=False))
    return evidence


def _plan(meta, initial_command, source_plan):
    plan = copy.deepcopy(source_plan)
    plan.setdefault("settle_seconds", 2.)
    plan.setdefault("initial_command", np.asarray(initial_command).tolist())
    plan.setdefault("initial_gap_m", .044)
    plan.setdefault("belt_ramp_seconds", .5)
    plan.setdefault("dof_names", list(DOF_NAMES))
    if list(plan["dof_names"]) != list(DOF_NAMES):
        raise ValueError("Source plan actuator ordering differs")
    # Single-window exports retain their source rail origin. A reused adapter
    # plan already carrying an explicit origin must agree with that source.
    manifest = json.loads(Path(meta["source_manifest"]).read_text())
    origin = float(manifest["rail"]["initial_station_y_m"])
    if "rail_origin_station_m" in plan and abs(float(plan["rail_origin_station_m"]) - origin) > 1e-8:
        raise ValueError("Plan rail origin differs from the single-window model")
    plan["rail_origin_station_m"] = origin
    return plan


def command_timeline(meta, initial_command, hz, *, command_source="plan", seconds=None,
                     start_s=0., freeze_phase=None, free_hz=None, belt_hold_hz=None):
    """Sample commands on solver-start timestamps; never sample measured poses.

    Recorded Isaac rows save an applied command at the *end* of its step.
    Consequently the row after a solver-start timestamp is the command for
    that interval. Plan sampling delegates the existing continuous sampler.
    ``start_s``/``freeze_phase`` change commands only: the physical state stays
    the original initial state, so these are explicitly diagnostic inputs.
    """
    from run_smoke import belt_velocity_from_physx

    if not np.isfinite(hz) or hz <= 0 or not np.isfinite(start_s) or start_s < 0:
        raise ValueError("Positive finite hz and nonnegative finite start_s required")
    if command_source not in ("plan", "recorded"):
        raise ValueError("command_source must be plan or recorded")
    if belt_hold_hz is not None:
        from newton_belt_hold import validate_belt_hold_hz
        validate_belt_hold_hz(belt_hold_hz, hz)
        if command_source != "plan" or freeze_phase is not None:
            raise ValueError("belt_hold_hz requires the continuous plan without freeze_phase")
    if free_hz is not None:
        if not np.isfinite(free_hz) or not 0 < free_hz <= hz:
            raise ValueError("free_hz must be positive, finite, and no higher than contact hz")
        if command_source != "plan":
            raise ValueError("Phase-frequency scheduling requires the existing continuous plan sampler")
        if any(abs(value - round(value)) > 1e-8 for value in (hz, free_hz)):
            raise ValueError("Variable-frequency scheduling requires integer Hz for its exact master clock")
    if freeze_phase is not None and command_source != "plan":
        raise ValueError("freeze_phase is supported for existing plan phases only")
    trial = Path(meta["source_trial"])
    plan = None
    if command_source == "plan":
        # A preserved copy in the model folder is verified by preparation.
        source_plan = Path(meta.get("prepared_source_plan", trial / "plan.json"))
        plan = _plan(meta, initial_command, json.loads(source_plan.read_text()))
        maximum = float(plan["settle_seconds"]) + sum(float(x["duration_s"]) for x in plan["phases"])
        if freeze_phase is not None:
            elapsed = float(plan["settle_seconds"])
            matches = []
            for row in plan["phases"]:
                if row["name"] == freeze_phase:
                    matches.append(elapsed + .5 * float(row["duration_s"]))
                elapsed += float(row["duration_s"])
            if len(matches) != 1:
                raise ValueError("freeze_phase must identify exactly one existing phase")
            source_start = matches[0]
        else:
            source_start = float(start_s)
    else:
        with np.load(trial / "states.npz", allow_pickle=False) as archive:
            saved = {key: archive[key].copy() for key in
                     ("time", "command", "phase", "belt_surface_velocity_local_y_m_s")}
        source_time = saved["time"]
        if len(source_time) < 2 or source_time[0] != 0 or not np.isfinite(source_time).all() or np.any(np.diff(source_time) <= 0):
            raise ValueError("Recorded source timestamps must start at zero and strictly increase")
        if saved["command"].shape != (len(source_time), len(DOF_NAMES)) or saved["belt_surface_velocity_local_y_m_s"].shape != (len(source_time), 2):
            raise ValueError("Recorded source command dimensions differ")
        if not np.isfinite(saved["command"]).all() or not np.isfinite(saved["belt_surface_velocity_local_y_m_s"]).all():
            raise ValueError("Recorded commands contain nonfinite values")
        maximum, source_start = float(source_time[-1]), float(start_s)
    if source_start >= maximum:
        raise ValueError("Requested source start is outside the existing command timeline")
    duration = maximum - source_start if seconds is None else float(seconds)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("seconds must be positive and finite")
    if freeze_phase is None:
        duration = min(duration, maximum - source_start)
    # Match Session.execute's start-time termination: the final complete step
    # may end less than one dt beyond a non-grid phase duration. Flooring would
    # omit the actual source's last step (82.7947s plan ->82.8s at60Hz).
    rows = None
    master_hz = None
    master_ticks = None
    if free_hz is not None:
        from run_harvest import phase_frequency
        # Use the exact existing phase policy at each solver-start time. An
        # integer lattice avoids accumulated float-time drift selecting a
        # different phase near boundaries. A free step can straddle a nominal
        # phase boundary, exactly as Session.execute's policy does; no new
        # waypoint, phase clipping, or artificial contact exemption is added.
        master_hz = math.lcm(int(round(hz)), int(round(free_hz)))
        edges, rows, samples = [0], [], []
        tick = 0
        while tick < duration * master_hz - 1e-7:
            stamp = source_start if freeze_phase is not None else source_start + tick / master_hz
            row = sample_plan(plan, float(stamp))
            frequency = phase_frequency(row["phase"], hz, free_hz)
            tick += master_hz // int(round(frequency))
            edges.append(tick)
            samples.append(stamp)
            rows.append(row)
        master_ticks = np.asarray(edges, dtype=np.int64)
        steps = len(rows)
        time_edges = master_ticks.astype(float) / master_hz
        source_sample_time = np.asarray(samples)
    else:
        steps = int(np.ceil(duration * hz - 1e-7))
        time_edges = np.arange(steps + 1, dtype=float) / hz
        source_sample_time = (np.full(steps, source_start) if freeze_phase is not None else
                              source_start + time_edges[:-1])
    if steps < 1:
        raise ValueError("Requested duration contains no complete physics step")
    time = time_edges[:-1]
    if command_source == "plan":
        if rows is None:
            rows = [sample_plan(plan, float(t)) for t in source_sample_time]
        commands = np.asarray([row["command"] for row in rows])
        belts = np.asarray([row["authored_belt_speed"] for row in rows])
        phases = np.asarray([row["phase"] for row in rows])
        command_record_time = source_sample_time.copy()
    else:
        # Tolerate only representation roundoff at exact source-step edges.
        epsilon = min(1e-10, float(np.min(np.diff(source_time))) * 1e-7)
        indices = np.searchsorted(source_time, source_sample_time + epsilon, side="right")
        indices = np.minimum(indices, len(source_time) - 1)
        commands = saved["command"][indices]
        belts = saved["belt_surface_velocity_local_y_m_s"][indices]
        phases = saved["phase"][indices]
        command_record_time = source_time[indices]
    dt = np.diff(time_edges)
    timeline = dict(time=time, time_edges=time_edges, dt=dt,
                    source_sample_time_s=source_sample_time,
                    source_command_record_time_s=command_record_time, command=commands,
                    phase=phases, authored_belt_speed=belts,
                    belt_surface_velocity_local_y_m_s=belt_velocity_from_physx(belts),
                    dof_names=np.asarray(DOF_NAMES))
    if master_ticks is not None:
        timeline.update(time_edges_master_ticks=master_ticks,
                        master_hz=np.asarray(master_hz, dtype=np.int64))
    frequencies, counts = np.unique(np.rint(1. / dt).astype(int), return_counts=True)
    description = dict(command_source=command_source, hz=float(hz), steps=steps,
                       free_hz=free_hz, master_hz=master_hz,
                       steps_by_hz={str(rate): int(count) for rate, count in zip(frequencies, counts)},
                       frequency_policy="Existing run_harvest.phase_frequency at each solver-start timestamp",
                       simulated_s=float(time_edges[-1]), nominal_duration_s=duration,
                       source_start_s=source_start,
                       source_duration_s=maximum, requested_seconds=seconds,
                       freeze_phase=freeze_phase, physical_state_advanced_to_source_start=False,
                       source_command_time_convention=("Existing sample_plan at solver-start time" if plan is not None else
                           "Recorded command row belongs to the interval ending at its timestamp; right-edge lookup"),
                       feedback_guards_required=True, motion_executed=False)
    if belt_hold_hz is not None:
        from newton_belt_hold import resample_belt_hold
        timeline, hold_evidence = resample_belt_hold(
            timeline, plan, belt_hold_hz, hz, source_start_s=source_start)
        frequencies, counts = np.unique(np.rint(1. / timeline['dt']).astype(int), return_counts=True)
        description.update(steps=len(timeline['time']), belt_hold_frequency=hold_evidence,
            master_hz=int(timeline['master_hz']) if 'master_hz' in timeline else None,
            steps_by_hz={str(rate): int(count) for rate, count in zip(frequencies, counts)},
            frequency_policy="Existing phase-frequency rows; only stationary belt_finish intervals equally resampled")
    return timeline, description, plan


def _snapshot_xml(source, destination, *, discard_visual_geoms=False):
    """Preserve existing XML parameters and resolve source asset paths explicitly."""
    source = Path(source).resolve()
    tree = ET.parse(source)
    root = tree.getroot()
    if root.findall(".//include"):
        raise ValueError("Flatten source includes with the existing exporter before snapshotting")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    if discard_visual_geoms:
        compiler.set("discardvisual", "true")
    meshdir = (source.parent / compiler.get("meshdir", "")).resolve()
    texturedir = (source.parent / compiler.get("texturedir", "")).resolve()
    dependencies = {}
    for asset in root.findall("./asset/*"):
        name = asset.get("file")
        if name:
            path = ((meshdir if asset.tag == "mesh" else texturedir) / name).resolve(strict=True)
            dependencies[str(path)] = _verified(path)
            asset.set("file", str(path))
    compiler.set("meshdir", str(meshdir))
    compiler.set("texturedir", str(texturedir))
    tree.write(destination, encoding="unicode")
    return list(dependencies.values())


def write_display_evidence(output, metadata, provenance, breaks):
    """Bind real Newton poses to the unchanged existing Isaac replay renderer.

    Rendering uses source USD hashes, while the physics model keeps its own
    separately recorded SHA. The original all-world Newton break events remain
    byte-preserved; legacy renderer events select recorded world zero and add
    exact source joint paths plus end-of-step release timestamps.
    """
    output = Path(output)
    assembly = Path(metadata["source_assembly"]).resolve(strict=True)
    manifest_path = Path(metadata["source_manifest"])
    _verified(manifest_path, metadata.get("source_manifest_sha256"))
    manifest = json.loads(manifest_path.read_text())
    scene = assembly / "scene.usda"
    scene_proof = _verified(scene, manifest["scene_sha256"])
    by_name = {fruit["name"]: fruit["joint"] for fruit in metadata["fruit_specs"]}
    events = []
    for source in breaks:
        if int(source.get("world", 0)) != 0:
            continue
        event = dict(source)
        if event.get("name") not in by_name:
            raise ValueError("Newton release has no exact source fruit identity")
        joint = by_name[event["name"]]
        if event.get("joint", joint) != joint:
            raise ValueError("Newton release source joint mapping differs")
        stamp = float(event["release_effective_time_s"])
        if not np.isfinite(stamp) or stamp < 0:
            raise ValueError("Finite actual release-effective timestamp required")
        event.update(joint=joint, t=stamp, display_world=0,
                     timestamp_scope="Actual end of the physics step disabling the attachment")
        events.append(event)
    raw = output / "break_events.json"
    preserved = output / "newton_break_events_all_worlds.json"
    if preserved.exists() or (output / "native_metadata.json").exists():
        raise FileExistsError("Display evidence already exists; use a new run output")
    if raw.is_file():
        if json.loads(raw.read_text()) != breaks:
            raise ValueError("Existing Newton break evidence differs from supplied events")
        shutil.copy2(raw, preserved)
    else:
        _json(preserved, breaks)
    _json(raw, events)
    _json(output / "native_metadata.json", dict(schema="s300-newton-native-display-mapping-v1",
        robot_dofs=list(DOF_NAMES), source_body_to_native_name=provenance["source_body_to_native_name"],
        physics_engine="Newton SolverMuJoCo", display_scene=str(scene),
        display_scene_sha256=scene_proof["sha256"],
        body_pose_storage="Actual recorded world position + xyzw quaternion; no physics replay",
        display_world=0, all_world_break_evidence=str(preserved), training_eligible=False))
    return dict(scene=str(scene), model_sha256=scene_proof["sha256"],
        model_hash_scope="Existing renderer compatibility: source display USD, separate from physical engine model SHA",
        display_source_model_sha256=scene_proof["sha256"],
        display_world=0, all_world_break_events_sha256=sha256(preserved),
        display_break_events_sha256=sha256(raw), physics_rerun_for_display=False)


def prepare_inputs(model_dir, output, hz, *, command_source="plan", seconds=None,
                   start_s=0., freeze_phase=None, free_hz=None, discard_visual_geoms=False,
                   allow_multiple_windows=False):
    """Save one immutable native adapter input for an independent Newton runner."""
    import mujoco as mj

    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError("Use a new output folder: " + str(output))
    source = Path(model_dir).resolve(strict=True)
    prepared = initialize_native_model(source, hz, discard_visual_geoms=discard_visual_geoms,
                                       allow_multiple_windows=allow_multiple_windows)
    model, data, meta = prepared["model"], prepared["data"], prepared["metadata"]
    if len(meta.get("windows", [])) > 1:
        meta = dict(meta, fruit_specs=window_fruit_specs(meta))
        if {item["name"] for item in meta["fruit_specs"]} != {item["name"] for item in meta["attachments"]}:
            raise ValueError("Persistent fruit specs and physical attachment ownership differ")
    sampling_meta = dict(meta, prepared_source_plan=str(source / "source_plan.json"))
    timeline, description, plan = command_timeline(
        sampling_meta, data.qpos[prepared["robot_qpos_addresses"]], hz,
        command_source=command_source, seconds=seconds, start_s=start_s,
        freeze_phase=freeze_phase, free_hz=free_hz)
    mapping = body_path_mapping(model, meta)
    if meta.get("source_states_kind") == "planning_only_no_recorded_states":
        source_body_paths = []
    else:
        with np.load(Path(meta["source_trial"]) / "states.npz", allow_pickle=False) as archive:
            source_body_paths = list(map(str, archive["body_paths"]))
    excluded = [path for path in source_body_paths if path not in mapping]
    excluded_proof = excluded_body_evidence(meta, excluded)
    output.mkdir(parents=True, exist_ok=False)
    dependencies = _snapshot_xml(source / "scene.xml", output / "input_scene.xml",
                                 discard_visual_geoms=discard_visual_geoms)
    # Runners use this signed MJB as the authoritative physical source_model.
    # XML supports import metadata; compiling it alone does not preserve the
    # explicitly retained compiler-derived constants of render separation.
    mj.mj_saveModel(model, str(output / "input_model.mjb"))
    np.savez_compressed(output / "initial_state.npz", qpos=data.qpos.copy(), qvel=data.qvel.copy(),
                        eq_active=data.eq_active.copy(), ctrl=data.ctrl.copy(),
                        robot_qpos_addresses=prepared["robot_qpos_addresses"],
                        robot_dof_addresses=prepared["robot_dof_addresses"], actuator_ids=prepared["actuator_ids"],
                        plant_dofs=prepared["plant_dofs"], plant_preload=prepared["preload"],
                        runtime_dof_damping=model.dof_damping.copy(),
                        runtime_dof_length=model.dof_length.copy(),
                        body_positions=data.xpos.copy(), body_quaternions_wxyz=data.xquat.copy(),
                        body_names=np.asarray([model.body(i).name or "world" for i in range(model.nbody)]),
                        actuator_kp=model.actuator_gainprm[prepared["actuator_ids"], 0].copy())
    np.savez_compressed(output / "commands.npz", **timeline)
    _json(output / "source_export.json", meta)
    if plan is not None:
        _json(output / "sampled_plan.json", plan)
    files = {name: _verified(output / name) for name in
             ("input_scene.xml", "input_model.mjb", "initial_state.npz", "commands.npz", "source_export.json")}
    if prepared["visual_removal_proof"] is not None:
        _json(output / "physics_semantic_proof.json", prepared["visual_removal_proof"])
        files["physics_semantic_proof.json"] = _verified(output / "physics_semantic_proof.json")
    report = dict(schema="s300-newton-reused-input-v1", model_directory=str(source),
                  prepared_with_mujoco_version=mj.__version__, input_files=files,
                  source_files=prepared["provenance"], source_asset_dependencies=dependencies,
                  code_sha256={str(p.relative_to(REPO)): sha256(p) for p in
                      [Path(__file__), ADAPTER / "plan_adapter.py", ADAPTER / "run_smoke.py", ADAPTER / "run_harvest.py"]},
                  timeline=description, source_body_to_native_name=mapping,
                  source_recorded_bodies_excluded_from_portable_model=excluded,
                  excluded_source_body_evidence=excluded_proof,
                  unexplained_source_body_omissions=[row["source_body"] for row in excluded_proof
                      if row["classification"] == "unexplained_portable_export_omission"],
                  model_counts=dict(nbody=model.nbody, nq=model.nq, nv=model.nv,
                                    ngeom=model.ngeom, nu=model.nu, neq=model.neq),
                  physics_model_variant=meta.get("physics_model_variant"),
                  noncolliding_visual_geoms_discarded=bool(discard_visual_geoms),
                  dynamic_tool=True, mocap_bodies=int(model.nmocap),
                  multiple_windows_opt_in=bool(allow_multiple_windows),
                  preload="Existing Session fixed initial Jacobian preload including attached-fruit mass",
                  robot_feedforward="Recompute current qfrc_bias at every real physics step and divide by finite actuator kp",
                  runtime_adapter="Existing passive fruit angular springs, angular damping and threshold release require executor support",
                  belt_sign="Physical local-Y = belt_velocity_from_physx(authored local-Y), preserving measured inward direction",
                  physical_steps=0, source_caches_written=False, training_eligible=False, harvest_success=None,
                  limitations=[
                      "Uses the documented MuJoCo adapter; no PhysX/Newton contact equivalence is asserted.",
                      "input_model.mjb is authoritative and version-specific; the Newton runtime must use the same MuJoCo version.",
                      "input_scene.xml is an import description; XML-only recompilation must restore runtime_dof_length before a physics-equivalence claim.",
                      "runtime_dof_damping and fixed preload must be applied by the executor after model import.",
                      "Command timeline sampling does not replace measured opening/arrival guards.",
                      "Excluded source bodies include previously released/frozen historical bodies; no new obstacle-coverage claim.",
                      "start_s/freeze_phase change command sampling only; physical state always remains the original initial state.",
                  ])
    _json(output / "inputs.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--hz", type=float, default=120.)
    parser.add_argument("--command-source", choices=("plan", "recorded"), default="plan")
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--start-s", type=float, default=0.)
    parser.add_argument("--freeze-phase")
    parser.add_argument("--free-hz", type=float,
                        help="Existing Session free-motion frequency; contact/open/reclose phases retain --hz")
    parser.add_argument("--discard-visual-geoms", action="store_true",
                        help="Opt-in remove only noncolliding visual geoms from physics; exact active-physics proof required")
    parser.add_argument("--allow-multiple-windows", action="store_true",
                        help="Opt-in persistent route model; preserve every declared window identity")
    args = parser.parse_args(argv)
    report = prepare_inputs(args.model, args.output, args.hz, command_source=args.command_source,
                            seconds=args.seconds, start_s=args.start_s,
                            freeze_phase=args.freeze_phase, free_hz=args.free_hz,
                            discard_visual_geoms=args.discard_visual_geoms,
                            allow_multiple_windows=args.allow_multiple_windows)
    print(json.dumps(dict(output=str(args.output.resolve()), **report["timeline"],
                          model_counts=report["model_counts"], physical_steps=0), indent=2), flush=True)


if __name__ == "__main__":
    main()
