"""Build, validate, train or replay the ring-harvesting task in the supplied greenhouse."""

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher
from ready_pose import POLICY_SCHEMA

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", choices=["build", "test", "train", "play", "scripted", "inspect"], default="inspect")
parser.add_argument("--scene", choices=["greenhouse", "fixture"], default="greenhouse")
parser.add_argument("--target-fruit", default=None)
parser.add_argument("--stem-position", type=float, nargs=3, default=None)
parser.add_argument("--stem-yaw", type=float, default=None)
parser.add_argument("--stem-scale", type=float, default=None)
parser.add_argument("--lift-start-below", type=float, default=None, help="Initial reference height below cluster centre (m)")
parser.add_argument("--lift-height-reference", choices=["mount", "ring"], default=None)
parser.add_argument("--lift-speed", type=float, default=None, help="Maximum commanded lift speed (m/s)")
parser.add_argument("--num-envs", type=int, default=1)
parser.add_argument("--curriculum", choices=["hook", "inserted", "approach"], default=None)
parser.add_argument("--steps", type=int, default=600)
parser.add_argument("--timesteps", type=int, default=100000)
parser.add_argument("--checkpoint", type=Path)
parser.add_argument("--run-dir", type=Path, default=Path(__file__).parent / "runs/default")
parser.add_argument("--snapshot", type=Path, help="Save an overview of the original scene as PNG")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--position-jitter", type=float, default=None, help="Reset position jitter in metres")
parser.add_argument("--break-force", type=float, default=None, help="Pedicel joint breaking force in N")
parser.add_argument("--break-torque", type=float, default=None, help="Pedicel joint breaking torque in N*m")
parser.add_argument("--rebuild", action="store_true")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args = parser.parse_args()
checkpoint_config = {}
if args.checkpoint is not None:
    metadata_path = args.checkpoint.parent / "config.json"
    if metadata_path.exists():
        checkpoint_config = json.loads(metadata_path.read_text())
        checkpoint_scene = checkpoint_config.get("scene", "fixture")
        if checkpoint_scene != args.scene:
            parser.error(f"Checkpoint belongs to {checkpoint_scene}, requested {args.scene}. Train a new greenhouse policy or explicitly use --scene fixture.")
    elif args.checkpoint.exists():
        parser.error("Checkpoint requires adjacent config.json to identify its training scene.")
if args.scene == "greenhouse" and checkpoint_config and checkpoint_config.get("policy_schema") != POLICY_SCHEMA:
    parser.error("This checkpoint uses the old fixed-lift policy. PICK_READY with a movable lift requires a new 7-action/39-observation policy.")
args.lift_start_below = args.lift_start_below if args.lift_start_below is not None else checkpoint_config.get("lift_start_below", .4)
args.lift_height_reference = args.lift_height_reference or checkpoint_config.get("lift_height_reference", "mount")
args.lift_speed = args.lift_speed if args.lift_speed is not None else checkpoint_config.get("lift_speed", .1)
if not math.isfinite(args.lift_start_below) or args.lift_start_below < 0 or not math.isfinite(args.lift_speed) or not 0 < args.lift_speed <= .25:
    parser.error("lift-start-below must be nonnegative and lift-speed must be in (0, 0.25] m/s")
args.target_fruit = args.target_fruit or checkpoint_config.get("target_fruit", "Tomato_08")
args.stem_position = args.stem_position if args.stem_position is not None else checkpoint_config.get("stem_position", (-.75, .55, .32))
args.stem_yaw = args.stem_yaw if args.stem_yaw is not None else checkpoint_config.get("stem_yaw", 0.)
args.stem_scale = args.stem_scale if args.stem_scale is not None else checkpoint_config.get("stem_scale", .5)
for threshold in ("break_force", "break_torque"):
    if getattr(args, threshold) is None and checkpoint_config.get("override_" + threshold, False):
        setattr(args, threshold, checkpoint_config[threshold])
args.curriculum = args.curriculum or checkpoint_config.get("curriculum") or ("approach" if args.scene == "greenhouse" else "hook")
if args.position_jitter is None:
    args.position_jitter = 0. if args.scene == "greenhouse" else .001
if args.scene == "greenhouse" and args.curriculum != "approach":
    parser.error("The original cluster currently supports --curriculum approach. The hook/inserted fixture poses intersect neighboring fruit and are not valid original-scene starts.")
if args.scene == "greenhouse" and args.mode == "scripted":
    parser.error("The scripted diagnostic is calibrated for --scene fixture. Use inspect, test, train, or play in the greenhouse.")
if args.scene == "greenhouse" and args.num_envs != 1:
    parser.error("The original greenhouse currently requires --num-envs 1.")
if args.scene == "greenhouse" and args.position_jitter != 0:
    parser.error("Original-scene start randomization is not implemented; use --position-jitter 0.")
if not math.isfinite(args.stem_scale) or args.stem_scale <= 0 or not all(math.isfinite(v) for v in [*args.stem_position, args.stem_yaw]):
    parser.error("stem-scale must be positive, and stem position/yaw must be finite.")
if args.device != "cpu":
    parser.error("This native-joint reset implementation requires --device cpu (PhysX direct GPU API cannot apply these USD edits).")
if args.num_envs < 1 or args.steps < 1 or args.timesteps < 1:
    parser.error("num-envs, steps, and timesteps must be positive")
if (not 0 <= args.position_jitter <= 0.01 or
        not all(math.isfinite(v) and v > 0 for v in [args.break_force, args.break_torque] if v is not None)):
    parser.error("position-jitter must be 0..0.01 m and break thresholds must be positive")
if args.mode == "play" and args.checkpoint is None:
    parser.error("--mode play requires --checkpoint")
if args.snapshot:
    args.enable_cameras = True
launcher = AppLauncher(args)  # Isaac Sim's default fast shutdown avoids ML-library unload conflicts.
app = launcher.app

import torch
from assets import ROBOT_ASSET, build_robot
torch.set_num_threads(4)


def physics_test(env):
    """Native force break + reattachment, with no geometry-triggered joint deletion."""
    zeros = torch.zeros(env.num_envs, env.cfg.action_space, device=env.device)
    env.reset()
    for _ in range(30):
        obs, rewards, _, _, _ = env.step(zeros)
        assert torch.isfinite(obs["policy"]).all(), "Non-finite observation"
        assert torch.isfinite(rewards).all(), "Non-finite reward"
    assert env.total_episodes == 0, f"Resting task terminates before robot applies force: {env.last_episode}"
    reports = []
    for cycle in range(3):
        before = env.total_invalid_breaks
        force = torch.zeros(env.num_envs, 1, 3, device=env.device)
        force[:, :, 0] = 10.0
        env.fruit.set_external_force_and_torque(force, torch.zeros_like(force))
        for _ in range(30):
            env.step(zeros)
            if env.total_invalid_breaks >= before + env.num_envs:
                break
        assert env.total_invalid_breaks == before + env.num_envs, "Native joint did not break or non-hook force was rewarded"
        assert env.total_successes == 0, "Non-hook force must never count as harvest success"
        env.fruit.set_external_force_and_torque(torch.zeros_like(force), torch.zeros_like(force))
        episodes = env.total_episodes
        for _ in range(30):
            env.step(zeros)
        assert env.total_episodes == episodes, "Joint reattachment did not survive gravity/rest"
        assert not env._broken.any(), "Broken state leaked across reset"
        reports.append({"cycle": cycle + 1, "native_breaks": env.num_envs, "reattachment_stable": True})
    if env.num_envs > 1:
        # Recreating one constraint must not disturb the other environments.
        before = env.total_invalid_breaks
        neighbor_pose = env.fruit.data.root_pos_w[1:].clone()
        force.zero_()
        force[0, 0, 0] = 10.0
        env.fruit.set_external_force_and_torque(force, torch.zeros_like(force))
        for _ in range(30):
            env.step(zeros)
            if env.total_invalid_breaks > before:
                break
        env.fruit.set_external_force_and_torque(torch.zeros_like(force), torch.zeros_like(force))
        assert env.total_invalid_breaks == before + 1, "Reset or force leaked into neighboring environments"
        assert torch.max(torch.abs(env.fruit.data.root_pos_w[1:] - neighbor_pose)) < 0.0001
    # Verify that the actual aperture admits the fruit, while the arc is solid.
    from omni.physx import get_physx_scene_query_interface
    from isaaclab.utils.math import quat_apply
    from geometry import FRUIT_RADIUS, RING_RADIUS
    pos, quat = env.ring_pose()
    query = get_physx_scene_query_interface()
    def robot_hits(local, radius):
        center = pos[0] + quat_apply(quat[:1], torch.tensor([local], device=env.device))[0]
        hits = []
        def collect(hit):
            if "/Robot/" in hit.collision:
                hits.append(hit.collision)
            return True
        query.overlap_sphere(radius, tuple(center.tolist()), collect, False)
        return hits
    assert not robot_hits([-0.012, 0, 0], FRUIT_RADIUS), "The collision model blocks the aperture"
    assert robot_hits([-RING_RADIUS, 0, 0], 0.001), "The ring arc has no physical collider"
    env.cfg.curriculum = "hook"
    env.reset()
    start_successes = env.total_successes
    for _ in range(240):
        action = torch.zeros_like(zeros)
        action[:, 0] = torch.where(env.episode_length_buf < 18, 0.35, 0.0)
        action[:, 1] = torch.where(env.episode_length_buf >= 18, 0.4, 0.0)
        env.step(action)
        if env.total_successes >= start_successes + 3 * env.num_envs:
            break
    assert env.total_successes >= start_successes + 3 * env.num_envs, "Scripted ring-pedicel contact did not harvest repeatedly"
    result = {"scene": "fixture", "physics_cycles": reports, "observations": list(obs["policy"].shape),
              "aperture_clear": True, "arc_solid": True,
              "independent_reset": env.num_envs > 1,
              "scripted_valid_harvests": env.total_successes - start_successes}
    print("[TEST] " + json.dumps(result), flush=True)
    return result


def greenhouse_test(env):
    zeros = torch.zeros(env.num_envs, env.cfg.action_space, device=env.device)
    from isaaclab.utils.math import quat_apply
    expected_arm = torch.tensor([[env.ready_joints[name] for name in env.ready_joints]], device=env.device)
    assert torch.max(torch.abs(env.robot.data.joint_pos[:, env.arm_ids]-expected_arm)) < 1e-5, "Start pose differs from ROS PICK_READY"
    def reference_height():
        if env.cfg.lift_height_reference == "ring":
            return float(env.ring_pose()[0][0, 2])
        return float((env.robot.data.body_pos_w[:, env.platform_id] + quat_apply(
            env.robot.data.body_quat_w[:, env.platform_id], env.mount_offset))[0, 2])
    initial_reference = reference_height()
    measured_offset = env.provenance["cluster_center_height_m"] - initial_reference
    assert abs(measured_offset-env.cfg.lift_start_below) < 1e-5, "Wrong lift height reference or offset"
    assert env.cfg.action_space == 7 and env.cfg.observation_space == 39
    original = torch.stack([f.data.root_pos_w.clone() for f in env.fruits])
    for _ in range(120):
        obs, rewards, _, _, _ = env.step(zeros)
        assert torch.isfinite(obs["policy"]).all() and torch.isfinite(rewards).all()
    assert env.total_episodes == 0, f"Original cluster breaks at rest: {env.last_episode}"
    drift = float(torch.max(torch.abs(torch.stack([f.data.root_pos_w for f in env.fruits])-original)))
    assert drift < .001, f"Original fruit placement drifted by {drift} m"
    # A lift-only action must translate the tool with the platform rather than
    # making arm IK counteract it. Also verify negative actions lower the lift.
    initial_lift = float(env.robot.data.joint_pos[0, env.lift_id])
    initial_ring_z = float(env.ring_pose()[0][0, 2])
    initial_arm = env.robot.data.joint_pos[:, env.arm_ids].clone()
    action = torch.zeros_like(zeros)
    action[:, 6] = 1.
    for _ in range(12):
        env.step(action)
    for _ in range(12):
        env.step(zeros)
    high_lift = float(env.robot.data.joint_pos[0, env.lift_id])
    high_ring_z = float(env.ring_pose()[0][0, 2])
    assert high_lift-initial_lift > .005, "Positive lift action did not raise the platform"
    assert abs((high_ring_z-initial_ring_z)-(high_lift-initial_lift)) < .002, "Arm IK canceled the lift movement"
    action[:, 6] = -1.
    for _ in range(12):
        env.step(action)
    for _ in range(12):
        env.step(zeros)
    low_lift = float(env.robot.data.joint_pos[0, env.lift_id])
    assert high_lift-low_lift > .005, "Negative lift action did not lower the platform"
    assert torch.max(torch.abs(env.robot.data.joint_pos[:, env.arm_ids]-initial_arm)) < .01, "Lift-only action disturbed the arm ready posture"
    assert env.total_episodes == 0, "Lift motion caused unintended fruit break or episode termination"
    for cycle in range(3):
        before = env.total_invalid_breaks
        force = torch.zeros(1, 1, 3, device=env.device)
        force[0, 0, 0] = 10.
        env.fruit.set_external_force_and_torque(force, torch.zeros_like(force))
        for _ in range(30):
            env.step(zeros)
            if env.total_invalid_breaks > before:
                break
        assert env.total_invalid_breaks == before+1, "Target joint force break was not detected"
        assert env.total_successes == 0, "Non-gripper force counted as harvest"
        env.fruit.set_external_force_and_torque(torch.zeros_like(force), torch.zeros_like(force))
        before_episodes = env.total_episodes
        for _ in range(60):
            env.step(zeros)
        assert env.total_episodes == before_episodes, "Cluster did not remain attached after reset"
    # A neighboring fruit breaking must fail even while the target joint remains intact.
    neighbor_index = (env.target_index+1) % len(env.fruits)
    before = env.total_invalid_breaks
    env.fruits[neighbor_index].set_external_force_and_torque(force, torch.zeros_like(force))
    for _ in range(30):
        env.step(zeros)
        if env.total_invalid_breaks > before:
            break
    assert env.total_invalid_breaks == before+1 and env.last_episode[-1]["wrong_fruit_break"], "Wrong-fruit break was not classified as failure"
    for _ in range(60):
        env.step(zeros)
    assert env.total_invalid_breaks == before+1, "Neighbor force/reset leaked into the next episode"
    restored = torch.stack([f.data.root_pos_w for f in env.fruits])
    expected = torch.tensor([[s["pose"][:3]] for s in env.fruit_specs], device=env.device)
    assert torch.max(torch.abs(restored-expected)) < .001, "Reset changed the source cluster placement"
    assert abs(reference_height()-initial_reference) < .001, "Reset did not restore the specified lift height"
    assert torch.max(torch.abs(env.robot.data.joint_pos[:, env.arm_ids]-expected_arm)) < .001, "Reset did not restore ROS PICK_READY"
    result = dict(scene="greenhouse", policy_schema=POLICY_SCHEMA, ready_state="PICK_READY",
                  height_reference=env.cfg.lift_height_reference, measured_start_offset_m=measured_offset,
                  lift_raised_m=high_lift-initial_lift, lift_lowered_m=high_lift-low_lift,
                  arm_ik_preserves_lift_motion=True, original_fruits=len(env.fruits), wrong_fruit_failure_verified=True, placement_drift_m=drift,
                  force_break_reset_cycles=3, observations=list(obs["policy"].shape), harvest_policy_validated=False)
    print("[TEST] " + json.dumps(result), flush=True)
    return result


def main():
    if args.rebuild or args.mode == "build" or not ROBOT_ASSET.exists():
        build_robot()
    if args.mode == "build":
        return
    from harvest_env import HarvestEnv, HarvestEnvCfg
    cfg = HarvestEnvCfg()
    cfg.seed = args.seed
    cfg.sim.device = args.device
    cfg.scene.num_envs = args.num_envs
    cfg.curriculum = args.curriculum
    cfg.position_jitter = args.position_jitter
    cfg.override_break_force = args.break_force is not None
    cfg.override_break_torque = args.break_torque is not None
    cfg.break_force = args.break_force if args.break_force is not None else 3.
    cfg.break_torque = args.break_torque if args.break_torque is not None else .08
    cfg.target_fruit = args.target_fruit
    cfg.stem_position = tuple(args.stem_position)
    cfg.stem_yaw = args.stem_yaw
    cfg.stem_scale = args.stem_scale
    cfg.lift_start_below = args.lift_start_below
    cfg.lift_height_reference = args.lift_height_reference
    cfg.lift_speed = args.lift_speed
    if args.scene == "greenhouse":
        from greenhouse_env import GreenhouseHarvestEnv
        env_class = GreenhouseHarvestEnv
        cfg.position_jitter = 0.
        cfg.break_randomization = 0.
        cfg.action_space = 7
        cfg.observation_space = 39
        cfg.max_target_distance = 2.
        cfg.insertion_reward_scale = .5
        cfg.episode_length_s = 20.
        cfg.robot.actuators["lift"].velocity_limit_sim = .25
    else:
        env_class = HarvestEnv
    if args.mode == "test":
        cfg.curriculum = "approach" if args.scene == "greenhouse" else "inserted"
        cfg.position_jitter = 0.0
        cfg.break_randomization = 0.0
    cfg.viewer.eye = (1.5, 1.5, 1.8)
    cfg.viewer.lookat = (0, 0, 1.0)
    env = env_class(cfg)
    if args.scene == "greenhouse":
        centers = torch.tensor(env.provenance["fruit_centers_world"])
        center = centers.mean(dim=0).tolist()
        overview_eye = (-1.05, center[1]-.9, center[2]+.65)
        overview_target = (-.35, (center[1]+cfg.robot.init_state.pos[1])*.5, center[2]-.1)
    if env.sim.has_gui():
        focus = env.nominal_ring_pos[0].cpu().numpy()
        if args.scene == "greenhouse":
            env.sim.set_camera_view(eye=overview_eye, target=overview_target)
        else:
            env.sim.set_camera_view(eye=focus + [0.18, -0.25, 0.12], target=focus)
    args.run_dir.mkdir(parents=True, exist_ok=True)
    if args.scene == "greenhouse":
        (args.run_dir / "scene_provenance.json").write_text(json.dumps(env.provenance, indent=2) + "\n")
    try:
        if args.snapshot:
            if args.scene != "greenhouse":
                raise ValueError("--snapshot currently supports the original greenhouse scene")
            import omni.replicator.core as rep
            from omni.physx import get_physx_interface
            env.sim.physics_sim_view.update_articulations_kinematic()
            get_physx_interface().update_transformations(False, True, True)
            from PIL import Image
            from pxr import UsdGeom, Gf
            camera = UsdGeom.Camera.Define(env.stage, "/World/RLOverviewCamera")
            camera.CreateFocalLengthAttr(24.)
            camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100.))
            view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*overview_eye), Gf.Vec3d(*overview_target), Gf.Vec3d(0, 0, 1))
            UsdGeom.Xformable(camera).MakeMatrixXform().Set(view.GetInverse())
            product = rep.create.render_product(str(camera.GetPath()), (1280, 720))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(product)
            for _ in range(10):
                env.sim.render()
            pixels = rgb.get_data()
            if not getattr(pixels, "size", 0):
                raise RuntimeError("Renderer returned an empty snapshot")
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(pixels).save(args.snapshot)
            rgb.detach(product)
            product.destroy()
            print(f"[SNAPSHOT] {args.snapshot}", flush=True)
        if args.mode == "test":
            result = greenhouse_test(env) if args.scene == "greenhouse" else physics_test(env)
            (args.run_dir / "physics_test.json").write_text(json.dumps(result, indent=2) + "\n")
        elif args.mode == "train":
            from stable_baselines3 import PPO
            from stable_baselines3.common.callbacks import CheckpointCallback
            from isaaclab_rl.sb3 import Sb3VecEnvWrapper
            vec = Sb3VecEnvWrapper(env)
            if args.checkpoint:
                model = PPO.load(str(args.checkpoint), env=vec, device="cpu")
            else:
                model = PPO("MlpPolicy", vec, n_steps=256, batch_size=64, n_epochs=5,
                            learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                            policy_kwargs={"net_arch": [128, 128]}, device="cpu", seed=args.seed, verbose=1)
            (args.run_dir / "config.json").write_text(json.dumps({
                "policy_schema": POLICY_SCHEMA if args.scene == "greenhouse" else "fixture-v1",
                "lift_start_below": cfg.lift_start_below, "lift_height_reference": cfg.lift_height_reference,
                "lift_speed": cfg.lift_speed, "action_space": cfg.action_space, "observation_space": cfg.observation_space,
                "scene": args.scene, "target_fruit": args.target_fruit, "curriculum": cfg.curriculum, "num_envs": env.num_envs, "seed": args.seed,
                "break_force": cfg.break_force, "break_torque": cfg.break_torque,
                "override_break_force": cfg.override_break_force, "override_break_torque": cfg.override_break_torque,
                "stem_position": cfg.stem_position, "stem_yaw": cfg.stem_yaw, "stem_scale": cfg.stem_scale,
                "position_jitter_m": cfg.position_jitter, "break_randomization": cfg.break_randomization,
                "physics_device": cfg.sim.device, "translation_step_m": cfg.translation_step,
                "rotation_step_rad": cfg.rotation_step,
                "dt": cfg.sim.dt, "decimation": cfg.decimation,
            }, indent=2) + "\n")
            model.learn(total_timesteps=args.timesteps, reset_num_timesteps=not bool(args.checkpoint),
                        callback=CheckpointCallback(save_freq=max(1, 10000 // env.num_envs), save_path=str(args.run_dir)))
            model.save(str(args.run_dir / "policy"))
            summary = {"scene": args.scene, "episodes": env.total_episodes, "successes": env.total_successes,
                       "invalid_breaks": env.total_invalid_breaks, "model_timesteps": model.num_timesteps}
            (args.run_dir / "training_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print("[TRAIN] " + json.dumps(summary), flush=True)
        else:
            obs, _ = env.reset()
            if args.mode == "play":
                from stable_baselines3 import PPO
                model = PPO.load(str(args.checkpoint), device="cpu")
            for step in range(args.steps):
                if not app.is_running():
                    break
                if args.mode == "play":
                    action, _ = model.predict(obs["policy"].cpu().numpy(), deterministic=True)
                    action = torch.as_tensor(action, device=env.device)
                elif args.mode == "inspect":
                    action = torch.zeros(env.num_envs, env.cfg.action_space, device=env.device)
                else:
                    # Diagnostic only: bring the rear arc into the pedicel then pull along its axis.
                    action = torch.zeros(env.num_envs, env.cfg.action_space, device=env.device)
                    age = env.episode_length_buf
                    action[:, 0] = torch.where(age < 18, 0.35, 0.0)
                    action[:, 1] = torch.where(age >= 18, 0.4, 0.0)
                obs, _, _, _, _ = env.step(action)
            summary = {"scene": args.scene, "episodes": env.total_episodes, "successes": env.total_successes,
                       "invalid_breaks": env.total_invalid_breaks, "recent_episodes": env.last_episode}
            (args.run_dir / "evaluation_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print("[EVAL] " + json.dumps({k: v for k, v in summary.items() if k != "recent_episodes"}), flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    import sys
    import traceback
    code = 0
    try:
        main()
    except BaseException:
        traceback.print_exc()
        code = 1
    finally:
        import omni.kit.app
        # Preserve failures even when Kit's fast shutdown exits the process directly.
        omni.kit.app.get_app().post_quit(code)
        app.close()
    sys.exit(code)
