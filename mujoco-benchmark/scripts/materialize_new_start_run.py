"""Create an immutable run snapshot with a new robot start q.

The source candidate geometry and planning model are preserved.  Candidate
traces are intentionally NOT copied, so a new-start run cannot silently reuse
an old recorded prefix whose first q belonged to the invalid historical start.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from initial_configuration_recovery import hold_rows


REQUIRED_SOURCE = (
    "manifest.json",
    "planning_inputs.json",
    "replay_assets/model.mjb",
    "replay_assets/reference.json",
    "replay_assets/initial_trace.json",
    "replay_assets/planning_model.pkl",
    "candidates.json",
)


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def load_start(path):
    doc = read(path)
    q = np.asarray(doc.get("q"), dtype=float)
    if q.ndim != 1 or not len(q) or not np.isfinite(q).all():
        raise ValueError("canonical start file has no finite q")
    return q, doc


def materialize(source_run, start_file, destination, *, model=None, reference=None,
                hold_seconds=2.0):
    source = Path(source_run).resolve()
    start_file = Path(start_file).resolve()
    destination = Path(destination).resolve()
    if destination.exists() or destination == source or source in destination.parents:
        raise ValueError("Use a new destination outside the source run")
    for name in REQUIRED_SOURCE:
        if not (source / name).is_file():
            raise ValueError(f"Missing source asset: {name}")

    q, start_doc = load_start(start_file)
    manifest = read(source / "manifest.json")
    planning = read(source / "planning_inputs.json")

    import plan_candidates as planner
    planner.initialize(str(source))
    env, kin, _ = planner.MODEL
    if q.shape != np.asarray(planner.START).shape:
        raise ValueError("Start q dimension differs from robot")
    bounds = np.asarray(kin.bounds, dtype=float)
    if np.any(q < bounds[0]) or np.any(q > bounds[1]):
        raise ValueError("Start q outside robot joint limits")
    ring_position, _ = kin.fk(q)

    model = Path(model).resolve() if model else (source / "replay_assets/model.mjb").resolve()
    reference = Path(reference).resolve() if reference else (source / "replay_assets/reference.json").resolve()
    if not model.is_file() or not reference.is_file():
        raise ValueError("Selected model/reference is missing")

    destination.mkdir(parents=True)
    assets = destination / "replay_assets"
    assets.mkdir()

    # Large immutable assets remain links.  The new initial trace is owned by
    # the derived snapshot and therefore cannot mutate the source.
    (assets / "model.mjb").symlink_to(model)
    (assets / "reference.json").symlink_to(reference)
    (assets / "planning_model.pkl").symlink_to(
        (source / "replay_assets/planning_model.pkl").resolve())

    new_planning = copy.deepcopy(planning)
    new_planning["inputs"]["start"] = q.tolist()
    new_planning["ring_position"] = np.asarray(ring_position, dtype=float).tolist()
    write(destination / "planning_inputs.json", new_planning)
    if (source / "action_frame.json").is_file():
        (destination / "action_frame.json").write_bytes(
            (source / "action_frame.json").read_bytes())
    write(assets / "initial_trace.json", hold_rows(q, seconds=hold_seconds))

    # Candidate definitions are historical geometry controls.  Do not copy
    # candidates/<id>/trace.json because those commands have the old start q.
    (destination / "candidates.json").write_bytes(
        (source / "candidates.json").read_bytes())

    model_hash = sha(model)
    old_hash = sha(source / "replay_assets/model.mjb")
    new_manifest = copy.deepcopy(manifest)
    new_manifest.update(
        model_sha256=model_hash,
        source_run=str(source),
        source_model_sha256=old_hash,
        start_contract_schema="canonical_robot_start_v1",
        historical_initial_q=np.asarray(planner.START, dtype=float).tolist(),
        initial_q=q.tolist(),
        start_contract_file=str(start_file),
        start_contract_sha256=sha(start_file),
        new_start_prefix_policy="replan_from_new_start; recorded source traces not copied",
        diagnostic_only=True,
        training_eligible=False,
        hook_success=None)
    write(destination / "manifest.json", new_manifest)

    provenance = dict(
        schema="new_start_snapshot_v1",
        source_run=str(source),
        destination=str(destination),
        original_q=np.asarray(planner.START, dtype=float).tolist(),
        new_q=q.tolist(),
        q_delta=(q - np.asarray(planner.START, dtype=float)).tolist(),
        source_model_sha256=old_hash,
        selected_model=str(model),
        selected_model_sha256=model_hash,
        selected_reference=str(reference),
        selected_reference_sha256=sha(reference),
        planning_model_sha256=sha(source / "replay_assets/planning_model.pkl"),
        source_candidates_sha256=sha(source / "candidates.json"),
        source_initial_trace_sha256=sha(source / "replay_assets/initial_trace.json"),
        derived_initial_trace_sha256=sha(assets / "initial_trace.json"),
        recorded_prefix_reuse_allowed=False,
        impossible=None)
    write(destination / "new_start_provenance.json", provenance)

    # Re-open through the actual planner contract.  This checks ring_position
    # and start synchronization rather than trusting the JSON edit.
    planner.initialize(str(destination))
    if not np.allclose(planner.START, q, atol=1e-12, rtol=0):
        raise RuntimeError("Derived planner did not load the new start q")
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_run", type=Path)
    parser.add_argument("start_file", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", type=Path,
                        help="Optional replacement MJB, e.g. covered dense scene")
    parser.add_argument("--reference", type=Path,
                        help="Reference matching --model")
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    args = parser.parse_args(argv)
    if bool(args.model) != bool(args.reference):
        parser.error("--model and --reference must be supplied together")
    result = materialize(
        args.source_run, args.start_file, args.output,
        model=args.model, reference=args.reference,
        hold_seconds=args.hold_seconds)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
