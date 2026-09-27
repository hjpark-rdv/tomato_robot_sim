"""Read-only audit of an existing candidate; optional static failure-pose viewer.

The original run is never modified, no policy is retrained and no physics is run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import time

from environment_preflight import Policy, check_engine, save_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Existing per-target physics run with replay_assets and candidates")
    parser.add_argument("--candidate", default="predicted_00000")
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="NEW directory, outside the source run")
    parser.add_argument("--gui", action="store_true", help="Show first rejected NOMINAL pose; not a dynamics replay")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.candidate):
        parser.error("Invalid candidate ID")
    root, output = args.run.resolve(), args.output.resolve()
    if output == root or root in output.parents:
        parser.error("Output must be outside the source run")
    folder = root / "candidates" / args.candidate
    policy = Policy.from_dict(json.loads(args.policy.read_text()))
    manifest = json.loads((root / "manifest.json").read_text())
    plan = json.loads((folder / "plan.json").read_text())
    if not plan.get("preflight", {}).get("passed"):
        parser.error("Candidate did not pass the existing IK/self-collision planner")
    rows = json.loads((folder / "trace.json").read_text())
    from robot_engine import RobotEngine
    assets = root / "replay_assets"
    engine = RobotEngine(assets / "model.mjb", assets / "initial_trace.json", manifest["hz"],
                         reference=assets / "reference.json", target=manifest.get("target", "Tomato_05"))
    output.mkdir(parents=True, exist_ok=False)
    result = check_engine(engine, rows, plan["seconds"], policy)
    result.update(source_run=str(root), candidate_id=args.candidate,
                  trace_sha256=hashlib.sha256((folder / "trace.json").read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256((assets / "model.mjb").read_bytes()).hexdigest(),
                  source_modified=False, physics_executed=False)
    save_report(output, result)
    print(json.dumps({k: result.get(k) for k in ("status", "reason", "first_violation", "wall_s")}, indent=2))
    print("Report:", output / "environment_preflight.html")
    if args.gui:
        hit = result.get("first_violation")
        if hit is None:
            print("No failure pose to display; no physics or synthetic success replay will be generated.")
        else:
            show_pose(engine, hit)
    return 0 if result.get("passed") else 2 if result.get("status") == "blocked" else 3


def show_pose(engine, hit):
    import numpy as np
    import mujoco as mj
    import mujoco.viewer
    from view_camera import target_camera
    data = mj.MjData(engine.model)
    mj.mj_resetData(engine.model, data)
    data.qpos[engine.qids] = hit["command"]
    mj.mj_forward(engine.model, data)
    frozen = data.qpos.copy()
    print("KINEMATIC PLAN PREVIEW ONLY: first rejected sample, not a physical rollout.")
    print("Free camera: drag/scroll. Close the window to exit.")
    with mj.viewer.launch_passive(engine.model, data) as viewer:
        with viewer.lock():
            target_camera(engine.model, data, viewer.cam, engine.target)
            viewer.cam.type = mj.mjtCamera.mjCAMERA_FREE
            for point in hit["segment_world"]:
                if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                    break
                geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_SPHERE, np.full(3, .003),
                               np.asarray(point), np.eye(3).ravel(), np.array([1., .15, .1, 1.]))
                viewer.user_scn.ngeom += 1
        while viewer.is_running():
            # Keep model state read-only; allow camera movement without repeatedly setting cam.
            with viewer.lock():
                data.qpos[:] = frozen
                mj.mj_forward(engine.model, data)
            viewer.sync()
            time.sleep(1 / 30)


if __name__ == "__main__":
    raise SystemExit(main())
