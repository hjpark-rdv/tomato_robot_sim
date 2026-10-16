#!/usr/bin/env python3
"""Bounded, manually stepped Newton connection probe for an existing USD stage.

This is an idle integration diagnostic. It neither replays a harvest controller
nor certifies PhysX-only belt and break semantics. Every requested physics step
is recorded; initialization does not contain an unrecorded settle/reset phase.
Run with the installed Isaac Sim ``python.sh`` rather than the MuJoCo venv.
"""

from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _save_report(output: Path, report: dict) -> None:
    temporary = output / "probe.json.tmp"
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    temporary.replace(output / "probe.json")


def _source_inventory(stage, Usd, UsdPhysics) -> dict:
    bodies, colliders, joints, unsupported = [], [], [], []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = UsdPhysics.RigidBodyAPI(prim)
            bodies.append({"path": path, "enabled": bool(body.GetRigidBodyEnabledAttr().Get()),
                           "kinematic": bool(body.GetKinematicEnabledAttr().Get())})
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append({"path": path,
                              "enabled": bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())})
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            joints.append({"path": path, "type": prim.GetTypeName(),
                           "enabled": bool(joint.GetJointEnabledAttr().Get()),
                           "body0": [str(x) for x in joint.GetBody0Rel().GetTargets()],
                           "body1": [str(x) for x in joint.GetBody1Rel().GetTargets()]})
        special = {}
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if ("surfaceVelocity" in name or name in ("physics:breakForce", "physics:breakTorque")) and attr.HasAuthoredValueOpinion():
                value = attr.Get()
                # USD uses Inf as the unbreakable default; retain its text safely.
                special[name] = str(value)
        if special:
            unsupported.append({"path": path, "attributes": special})
    return {"rigid_bodies": bodies, "colliders": colliders, "joints": joints,
            "physics_scenes": [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)],
            "physx_semantics_requiring_explicit_adapter": unsupported,
            "counts": {"rigid_bodies": len(bodies), "enabled_rigid_bodies": sum(x["enabled"] for x in bodies),
                       "colliders": len(colliders), "enabled_colliders": sum(x["enabled"] for x in colliders),
                       "joints": len(joints), "enabled_joints": sum(x["enabled"] for x in joints)}}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--render-every", type=int, default=2)
    parser.add_argument("--device", default="cuda:0", choices=("cuda:0", "cpu"))
    parser.add_argument("--solver", choices=("mujoco", "xpbd"), default="mujoco")
    parser.add_argument("--mujoco-cpu", action="store_true")
    parser.add_argument("--nconmax", type=int, default=2048)
    parser.add_argument("--njmax", type=int, default=8192)
    parser.add_argument("--newton-contacts", action="store_true",
                        help="Explicit collision-pipeline comparison; default uses MuJoCo contacts.")
    parser.add_argument("--no-cuda-graph", action="store_true")
    parser.add_argument("--minimal-rendering", action="store_true",
                        help="Use Isaac's supported minimal renderer for a physics connection diagnostic.")
    parser.add_argument("--stack-dump-seconds", type=float, default=0.0,
                        help="Optional periodic Python stack dumps for a bounded import diagnosis.")
    parser.add_argument("--preserve-unordered-bodies", action="store_true",
                        help="Use public USD parser bodies_follow_joint_ordering=False to preserve loop-only endpoints.")
    args = parser.parse_args(argv)
    if args.steps < 0 or not 0 < args.hz <= 2000 or args.render_every <= 0:
        parser.error("steps >= 0, 0 < hz <= 2000, render-every > 0 required")
    if args.nconmax <= 0 or args.njmax <= 0:
        parser.error("positive contact/constraint capacities required")
    if args.stack_dump_seconds < 0:
        parser.error("stack-dump-seconds must be nonnegative")
    source = args.scene.resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema": "s300-newton-connection-probe-v1", "status": "starting", "phase": "bootstrap",
              "source_scene": str(source), "source_sha256": _sha256(source),
              "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "training_eligible": False, "harvest_success": None,
              "physics_validity_evaluated": False,
              "code_files": {str(path): _sha256(path) for path in
                             (Path(__file__).resolve(), Path(__file__).with_name("native_newton_import.py").resolve())},
              "controller": "idle; no harvest command applied", "requested_steps": args.steps,
              "recorded_steps": 0, "warmup_steps": 0, "dropped_steps": 0, "resets": 0,
              "exceptions": [], "notes": [
                  "Belt surface velocity and PhysX breakForce/breakTorque require explicit adapters.",
                  "Import or idle stability alone does not establish harvest semantics or real-time throughput.",
                  "Source stage is read-only; solver selection is authored in its anonymous session layer.",
              ]}
    _save_report(output, report)
    app = None
    times, body_q, joint_q, joint_qd, step_times, render_times = [], [], [], [], [], []
    started = time.perf_counter()
    if args.stack_dump_seconds:
        faulthandler.dump_traceback_later(args.stack_dump_seconds, repeat=True)
    try:
        from isaacsim import SimulationApp

        extra_args = ["--/exts/isaacsim.core.simulation_manager/default_engine=newton",
                      "--enable", "isaacsim.physics.newton", "--enable", "isaacsim.physics.newton.tensors",
                      "--/app/player/useFixedTimeStepping=false", "--/app/asyncRendering=false",
                      "--/app/asyncRenderingLowLatency=false", "--/app/settings/persistent=false"]
        app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720,
                             "disable_viewport_updates": not args.gui,
                             "renderer": "MinimalRendering" if args.minimal_rendering else "RealTimePathTracing",
                             "extra_args": extra_args})
        import numpy as np
        import newton
        import omni.timeline
        import omni.usd
        import warp as wp
        from isaacsim.physics.newton import (MuJoCoSolverConfig, NewtonConfig, XPBDSolverConfig,
                                            acquire_stage, configure_newton, get_active_physics_engine,
                                            switch_newton_solver)
        from pxr import Usd, UsdPhysics
        from native_newton_import import preserve_unordered_bodies

        solver_cfg = (MuJoCoSolverConfig(nconmax=args.nconmax, njmax=args.njmax,
                                        use_mujoco_cpu=args.mujoco_cpu,
                                        use_mujoco_contacts=not args.newton_contacts,
                                        save_to_mjcf=str(output / "imported_model.xml"))
                      if args.solver == "mujoco" else XPBDSolverConfig())
        cfg = NewtonConfig(time_step_app=False, update_fabric=True,
                           use_cuda_graph=not args.no_cuda_graph and args.device.startswith("cuda"),
                           physics_frequency=args.hz, collapse_fixed_joints=False, solver_cfg=solver_cfg)
        configure_newton(cfg)
        report.update(phase="open_source", bootstrap_seconds=time.perf_counter() - started)
        _save_report(output, report)
        print(f"Newton probe opening {source}", flush=True)
        context = omni.usd.get_context()
        if not context.open_stage(str(source)):
            raise RuntimeError("USD context refused source stage")
        for _ in range(4):
            app.update()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("No USD stage after open")
        report["source_inventory"] = _source_inventory(stage, Usd, UsdPhysics)
        report["source_open_seconds"] = time.perf_counter() - started - report["bootstrap_seconds"]
        print(f"Newton probe source open; {report['source_inventory']['counts']}", flush=True)
        # Never persist Newton opinions into an original execution asset.
        stage.SetEditTarget(stage.GetSessionLayer())
        scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes) != 1:
            raise RuntimeError(f"Expected exactly one explicit PhysicsScene; found {len(scenes)}")
        if not switch_newton_solver(args.solver, scenes[0]):
            raise RuntimeError("Newton solver schema selection failed")
        configure_newton(cfg)
        ns = acquire_stage()
        if ns is None:
            raise RuntimeError("Newton extension has no stage")
        # Render updates can dispatch timeline events, but automatic stepping is off.
        report["phase"] = "initialize_newton"
        _save_report(output, report)
        omni.timeline.get_timeline_interface().play()
        # time_step_app=False permits manual initialization without an app
        # update that could dispatch costly rendering work before the phase is
        # recorded. step_sim synchronizes the timeline playing flag itself.
        before_init_steps = int(ns.simulation_step_count)
        if before_init_steps != 0:
            raise RuntimeError(f"Unexpected autonomous pre-initialization steps: {before_init_steps}")
        report.update(phase="initialize_newton", active_engine=get_active_physics_engine(),
                      time_step_app=ns.cfg.time_step_app, init_start_seconds=time.perf_counter() - started)
        _save_report(output, report)
        initialize_started = time.perf_counter()
        with preserve_unordered_bodies(newton.ModelBuilder, args.preserve_unordered_bodies) as import_audit:
            ns.initialize_newton(args.device)
        report["native_import_compatibility"] = import_audit
        wp.synchronize()
        report["initialize_seconds"] = time.perf_counter() - initialize_started
        if not ns.initialized or ns.model is None or ns.state_0 is None or getattr(ns, "_init_failed", False):
            raise RuntimeError("Newton initialization failed; inspect native stdout log for parser/solver error")
        model = ns.model
        report["newton_model"] = {name: int(getattr(model, name, 0)) for name in
                                  ("body_count", "shape_count", "joint_count", "joint_dof_count",
                                   "joint_coord_count", "world_count", "particle_count")}
        report["newton_model"].update(body_labels=[str(x) for x in model.body_label],
                                      shape_labels=[str(x) for x in model.shape_label],
                                      joint_labels=[str(x) for x in model.joint_label],
                                      joint_type=model.joint_type.numpy().tolist(),
                                      joint_q_start=model.joint_q_start.numpy().tolist(),
                                      joint_qd_start=model.joint_qd_start.numpy().tolist())
        report["newton_model"].update(shape_body=model.shape_body.numpy().tolist(),
                                      body_mass=model.body_mass.numpy().tolist(),
                                      joint_parent=model.joint_parent.numpy().tolist(),
                                      joint_child=model.joint_child.numpy().tolist())
        shape_flags = model.shape_flags.numpy()
        report["newton_model"]["colliding_shape_count"] = int(
            np.count_nonzero(shape_flags & int(newton.ShapeFlags.COLLIDE_SHAPES)))
        source_target_fruits = [x["path"] for x in report["source_inventory"]["rigid_bodies"]
                                if x["enabled"] and not x["kinematic"] and
                                x["path"].startswith("/World/ElasticTarget/") and
                                "/Truss_" in x["path"] and "_Fruit_" in x["path"]]
        labels = set(report["newton_model"]["body_labels"])
        report["active_target_fruit_body_mapping"] = {
            "source_paths": source_target_fruits,
            "mapped_paths": [p for p in source_target_fruits if p in labels],
            "missing_paths": [p for p in source_target_fruits if p not in labels],
            "all_mapped": bool(source_target_fruits) and all(p in labels for p in source_target_fruits)}
        if source_target_fruits and not report["active_target_fruit_body_mapping"]["all_mapped"]:
            raise RuntimeError("Active target fruit bodies were lost by USD import; "
                               "use --preserve-unordered-bodies for the explicit compatibility diagnostic")
        report["parser_results"] = {k: v for k, v in ns.parsing_results.items()
                                    if isinstance(v, (str, int, float, bool, type(None)))}
        report["parser_mapping"] = {k: {str(kk): str(vv) for kk, vv in v.items()}
                                    for k, v in ns.parsing_results.items()
                                    if isinstance(v, dict) and ("map" in k or "indices" in k)}
        report["physics_hz_from_source"] = float(getattr(ns, "physics_frequency", args.hz))
        report["manual_dt"] = 1.0 / args.hz
        report["initial_step_count"] = int(ns.simulation_step_count)
        report["phase"] = "manual_steps"
        _save_report(output, report)

        initial_body = ns.state_0.body_q.numpy().copy()
        initial_joint = ns.state_0.joint_q.numpy().copy()
        initial_velocity = ns.state_0.joint_qd.numpy().copy()
        np.savez_compressed(output / "initial_state.npz", body_q=initial_body,
                            joint_q=initial_joint, joint_qd=initial_velocity)
        loop_started = time.perf_counter()
        for index in range(args.steps):
            step_started = time.perf_counter()
            ns.step_sim(1.0 / args.hz)
            wp.synchronize()
            step_times.append(time.perf_counter() - step_started)
            state = ns.state_0
            current_body, current_joint, current_velocity = (state.body_q.numpy(), state.joint_q.numpy(),
                                                            state.joint_qd.numpy())
            if not all(np.isfinite(x).all() for x in (current_body, current_joint, current_velocity)):
                raise RuntimeError(f"Nonfinite state at requested step {index + 1}")
            times.append((index + 1) / args.hz)
            body_q.append(current_body)
            joint_q.append(current_joint)
            joint_qd.append(current_velocity)
            if args.gui and (index + 1) % args.render_every == 0:
                expected_count = int(ns.simulation_step_count)
                render_started = time.perf_counter()
                ns.update_fabric()
                app.update()
                wp.synchronize()
                render_times.append(time.perf_counter() - render_started)
                if int(ns.simulation_step_count) != expected_count:
                    raise RuntimeError("GUI/app update advanced autonomous Newton physics")
            report["recorded_steps"] = index + 1
            if (index + 1) % max(1, round(args.hz)) == 0:
                print(f"Newton probe {index + 1}/{args.steps}; sim {(index + 1) / args.hz:.3f}s", flush=True)
                _save_report(output, report)
        wp.synchronize()
        report["loop_seconds"] = time.perf_counter() - loop_started
        np.savez_compressed(output / "states.npz", time=np.asarray(times), body_q=np.asarray(body_q),
                            joint_q=np.asarray(joint_q), joint_qd=np.asarray(joint_qd),
                            physics_step_seconds=np.asarray(step_times), gui_update_seconds=np.asarray(render_times))
        report.update(status="completed_idle_probe", phase="complete",
                      final_step_count=int(ns.simulation_step_count), simulation_seconds=args.steps / args.hz,
                      physics_step_seconds_sum=float(sum(step_times)), gui_update_seconds_sum=float(sum(render_times)),
                      nominal_loop_rtf=(args.steps / args.hz / report["loop_seconds"]),
                      original_source_sha256_after=_sha256(source),
                      max_idle_body_displacement_m=float(np.linalg.norm(ns.state_0.body_q.numpy()[:, :3] - initial_body[:, :3], axis=1).max(initial=0)))
        if report["source_sha256"] != report["original_source_sha256_after"]:
            raise RuntimeError("Original source stage changed")
        if int(ns.simulation_step_count) != args.steps:
            raise RuntimeError("Actual Newton step count differs from requested fixed-step count")
    except BaseException as exc:
        report["status"] = "failed_connection_probe"
        report["exceptions"].append({"phase": report["phase"], "type": type(exc).__name__,
                                     "message": str(exc), "traceback": traceback.format_exc()})
        print(traceback.format_exc(), file=sys.stderr, flush=True)
    finally:
        if args.stack_dump_seconds:
            faulthandler.cancel_dump_traceback_later()
        if times and not (output / "states.npz").exists():
            # Preserve all completed finite states even when a later step fails.
            import numpy as np

            np.savez_compressed(output / "states.npz", time=np.asarray(times), body_q=np.asarray(body_q),
                                joint_q=np.asarray(joint_q), joint_qd=np.asarray(joint_qd),
                                physics_step_seconds=np.asarray(step_times), gui_update_seconds=np.asarray(render_times))
        report["original_source_sha256_after"] = _sha256(source)
        report["total_seconds_before_app_close"] = time.perf_counter() - started
        _save_report(output, report)
        if app is not None:
            try:
                app.close()
            except BaseException as exc:
                report["exceptions"].append({"phase": "close", "type": type(exc).__name__,
                                             "message": str(exc), "traceback": traceback.format_exc()})
                _save_report(output, report)
    return 0 if report["status"] == "completed_idle_probe" else 1


if __name__ == "__main__":
    raise SystemExit(main())
