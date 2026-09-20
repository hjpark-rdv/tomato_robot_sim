"""RGB + joint near-approach guard experiment in the original elastic scene.

    The fixed reference is a previously recorded joint trajectory, not an
    online ground-truth object-relative plan. Evaluation reads plant state
    only after each command and never feeds those values to the guard.
"""
import json
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np
import torch

from hook_motion import RobotKinematics
from rgb_camera import ToolRGBViews, intrinsics, world_from_camera


def run_rgb(env, args, app):
    directory = args.run_dir
    directory.mkdir(parents=True, exist_ok=True)
    reference_path = Path(__file__).parent/'configs/tomato05_rgb_reference.npz'
    reference = np.load(reference_path, allow_pickle=False)
    meta = json.loads(reference_path.with_suffix('.json').read_text())
    if env.robot.joint_names != meta['joint_names'] or not np.allclose(env.cfg.stem_position, meta['stem_position']) or env.cfg.stem_scale != meta['stem_scale'] or env.cfg.stem_yaw != 0:
        raise ValueError('Fixed RGB reference requires the original calibrated fixture and joint order')
    env.reset()
    if not np.allclose(env.robot.data.joint_pos[0].cpu().numpy(), reference['initial_joints'], atol=1e-4):
        raise ValueError('Robot initial state differs from the recorded reference')
    stiffness = torch.full_like(env.targets, 800.)
    stiffness[:, env.lift_id] = 50000.
    damping = torch.full_like(env.targets, 40.)
    damping[:, env.lift_id] = 2500.
    env.robot.write_joint_stiffness_to_sim(stiffness)
    env.robot.write_joint_damping_to_sim(damping)
    kin = RobotKinematics(env)
    names = ['current', 'left', 'right'] if args.rgb_action == 'compare' else [args.rgb_camera]
    cameras = ToolRGBViews(env, kin, names)
    writers = {name: imageio.get_writer(str(directory/(name+'_2x.mp4')), fps=60, codec='libx264', quality=8) for name in names}
    # 30 Hz observations, encoded at 60 fps: playback only is doubled.
    frames = {name: 0 for name in names}
    observations, evaluation, commands = [], [], []
    analysis = []
    raw_writer = imageio.get_writer(str(directory/(args.rgb_camera+'_raw_2x.mkv')),
        fps=60, codec='ffv1', pixelformat='bgr0', macro_block_size=1) if args.rgb_analysis else None
    initial_centers = env.elastic.fruit_centers().copy()
    initial_rods = env.elastic.poses()[:, :3].copy()
    guard = None
    profile = cameras.profiles.get(args.rgb_camera)
    if args.rgb_action != 'compare':
        from rgb_guard import RGBMotionGuard
        guard = RGBMotionGuard(intrinsics(profile))
    seeded = False
    trigger = None
    phase = 'reference'
    reference_time = 0.
    retreat = []
    hold_steps = 0
    step = 0
    env.contact_diagnostics = []
    metadata = dict(mode=args.rgb_action, camera=args.rgb_camera, roi=args.rgb_roi,
                    reference=meta, replay_rate=args.rgb_replay_rate,
                    controller_inputs=['RGB', 'joint_positions', 'calibrated_camera_extrinsics', 'fixed_joint_reference'],
                    excluded_inputs=['depth', 'force', 'contact_identity', 'object_ground_truth'],
                    physics_dt=env.physics_dt, control_dt=env.step_dt, camera_hz=30, playback_speed=2,
                    camera_profiles={name: {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in p.items()} for name,p in cameras.profiles.items()},
                    scope='near-approach motion guard; not full RGB target acquisition or harvesting',
                    housing_and_bracket_collision_validated=False)
    metadata['exact_rgb_analysis_saved'] = args.rgb_analysis
    (directory/'config.json').write_text(json.dumps(metadata, indent=2)+'\n')
    try:
        while app.is_running():
            if phase == 'reference':
                index = min(int(reference_time), len(reference['commands'])-1)
                next_index = min(index+1, len(reference['commands'])-1)
                fraction = reference_time-index
                command = reference['commands'][index]*(1-fraction)+reference['commands'][next_index]*fraction
            elif phase == 'retreat':
                if not retreat:
                    phase, hold_steps = 'hold', 60
                    continue
                command = retreat.pop(0)
            else:
                if hold_steps <= 0:
                    break
                hold_steps -= 1
                command = commands[-1]
            executed_phase = phase
            env.targets[:] = torch.as_tensor(command, device=env.device, dtype=torch.float32)
            env.robot.set_joint_position_target(env.targets)
            env.contact_diagnostic_step = step
            # No MotionTrial.step / env.step / _get_dones here: those paths
            # have oracle contact guards and ground-truth episode termination.
            for _ in range(env.cfg.decimation):
                env.scene.write_data_to_sim()
                env.sim.step(render=False)
                env.scene.update(env.physics_dt)
            joints = env.robot.data.joint_pos[0].cpu().numpy().copy()
            commands.append(np.asarray(command).copy())
            if step % 2 == 0:
                images = cameras.read(joints)
                row = dict(step=step, time_s=(step+1)*env.step_dt, phase=phase,
                           reference_step=reference_time, joints=joints.tolist())
                if reference_time >= meta['seed_frame_step'] and not seeded:
                    for name, pixels in images.items():
                        imageio.imwrite(directory/(name+'_seed.png'), pixels)
                    if guard:
                        guard.initialize(images[args.rgb_camera], args.rgb_roi,
                                         world_from_camera(kin, joints, profile))
                    seeded = True
                elif guard and seeded and phase == 'reference':
                    row.update(guard.observe(images[args.rgb_camera], world_from_camera(kin, joints, profile)))
                    if row['stop'] and trigger is None:
                        trigger = dict(row)
                        if args.rgb_action == 'guard':
                            # Position hold then retrace already executed joint
                            # commands at half the reference speed; no plant state.
                            recent = commands[max(0, len(commands)-121):]
                            retreat = [joints.copy()]*6
                            for old in reversed(recent):
                                retreat.extend([old.copy(), old.copy()])
                            phase = 'retreat'
                            print('[RGB STOP] '+json.dumps(trigger), flush=True)
                if raw_writer is not None:
                    raw_writer.append_data(images[args.rgb_camera])
                    active = guard is not None and seeded and executed_phase == 'reference' and (trigger is None or trigger['step'] == step)
                    analysis.append(dict(step=step, time_s=row['time_s'], phase=executed_phase,
                        active=active, decision=guard.last.copy() if guard and seeded else None,
                        visual=guard.visual if active else None))
                observations.append(row)
                for name, pixels in images.items():
                    if guard and name == args.rgb_camera and seeded:
                        pixels = guard.annotate(pixels, draw_tracks=phase == 'reference' and trigger is None)
                    cv2.putText(pixels, f'{name} | {phase} | t={(step+1)*env.step_dt:.2f}s | 2x', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
                    writers[name].append_data(pixels)
                    frames[name] += 1
                    if step == 0 or step % 120 == 0:
                        imageio.imwrite(directory/f'{name}_step_{step:04d}.png', pixels)
            # Separate privileged evaluator. None of these values select actions.
            centers = env.elastic.fruit_centers()
            ids = env.elastic.chains['STEM_MainStem']
            evaluation.append(dict(step=step, phase=executed_phase,
                target_displacement_m=float(np.linalg.norm(centers[env.target_index]-initial_centers[env.target_index])),
                main_displacement_m=float(np.linalg.norm(env.elastic.poses()[ids,:3]-initial_rods[ids],axis=1).max()),
                target_broken=bool(env._broken[0]), other_broken=bool(env._other_broken)))
            if phase == 'reference':
                reference_time += args.rgb_replay_rate
                if reference_time >= len(reference['commands']):
                    phase, hold_steps = 'hold', 60
            if step % 120 == 0:
                print('[RGB PROGRESS] '+json.dumps(dict(step=step, phase=phase, reference_step=reference_time)), flush=True)
            step += 1
        summary = dict(steps=step, frames=frames, trigger=trigger,
            detection_triggered=trigger is not None,
            guarded_retreat_completed=args.rgb_action=='guard' and trigger is not None and not retreat and hold_steps==0,
            max_target_displacement_m=max(r['target_displacement_m'] for r in evaluation),
            max_main_displacement_m=max(r['main_displacement_m'] for r in evaluation),
            target_broken=any(r['target_broken'] for r in evaluation),
            other_broken=any(r['other_broken'] for r in evaluation),
            harvest_success=None)
        (directory/'result.json').write_text(json.dumps(summary, indent=2)+'\n')
        print('[RGB RESULT] '+json.dumps(summary), flush=True)
    finally:
        (directory/'observations.json').write_text(json.dumps(observations)+'\n')
        (directory/'evaluation_only.json').write_text(json.dumps(evaluation)+'\n')
        (directory/'contacts_evaluation_only.json').write_text(json.dumps(env.contact_diagnostics)+'\n')
        np.save(directory/'commands.npy', np.asarray(commands))
        for writer in writers.values():
            writer.close()
        if raw_writer is not None:
            raw_writer.close()
            (directory/'rgb_analysis.json').write_text(json.dumps(analysis)+'\n')
        cameras.close()
