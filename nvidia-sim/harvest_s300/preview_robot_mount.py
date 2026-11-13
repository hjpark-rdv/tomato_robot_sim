"""CAD assembly and optional checked joint-motion preview; no dynamics execution."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import subprocess
import threading
import time

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
import viser

from newton_inputs import sha256
from newton_scene_chunks import PackedScene, pack_scene, stream_manifest
from newton_viser import geom_mesh, populate_scene
from preview_gripper_camera import camera_mount_spec, populate_camera_scene, update_camera, optical_world
from preview_conveyor_bin import (
    conveyor_bin_spec,conveyor_grip_parts,draw_conveyor,update_conveyor,
    unload_frame,load_unload_record,
)


# The original lift CAD separates the motor/gearbox, connectors and fasteners
# into 40 connected meshes. The mast and five ground supports stay fixed.
LIFT_MOTOR_COMPONENTS = {0, 3, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 18, 19, *range(22, 48)}
LAYOUT_LABELS = {'original': 'V1 · 기존 배치', 'compact': 'V2 · 리프트 밀착 배치',
                 'outboard': 'V3 · 로봇암 외측 배치', 'storage': 'V4 · 기본 상판 + 토마토 보관함',
                 'conveyor': 'V5 · 밀착 컨베이어 보관함',
                 'side': 'V6 · 리프트 옆면 장착'}
CONVEYOR_LAYOUTS=('conveyor','side')
STORAGE_LAYOUTS=('storage',*CONVEYOR_LAYOUTS)


def box_edges(bounds):
    low, high = np.asarray(bounds)
    corners = np.array([[x, y, z] for x in (low[0], high[0])
                        for y in (low[1], high[1]) for z in (low[2], high[2])])
    return np.array([[corners[i], corners[i ^ (1 << k)]]
        for i in range(8) for k in range(3) if i < i ^ (1 << k)])


def storage_bin_parts(bounds, deck_top):
    """Open-topped, ribbed concept bin; dimensions are assumed, not supplier CAD."""
    low, high = np.asarray(bounds, float)
    centre = (low + high) / 2.
    size = high - low
    wall, floor, rim = .012, .018, .025
    parts = []
    def part(name, position, dimensions, color=(49, 96, 78)):
        parts.append(dict(name=name, position=np.asarray(position).tolist(),
                          dimensions=np.asarray(dimensions).tolist(), color=color))
    part('floor', [*centre[:2], low[2]+floor/2.], [*size[:2], floor])
    for axis in (0, 1):
        other = 1-axis
        for side, edge in enumerate((low[axis], high[axis])):
            sign = 1 if side == 0 else -1
            p = centre.copy()
            p[axis] = edge + sign*(wall/2.+.012)
            d = size.copy()
            d[axis] = wall
            part(f'wall_{axis}_{side}', p, d)
            p[axis] = edge + sign*rim/2.
            p[2] = high[2] - rim/2.
            d[axis], d[2] = rim, rim
            part(f'rim_{axis}_{side}', p, d, (35, 69, 56))
            # Exterior ribs convey the plastic container shape without a lid.
            for j, along in enumerate(np.linspace(low[other]+.05, high[other]-.05,
                                                  7 if axis == 0 else 4)):
                p = centre.copy()
                p[axis], p[other] = edge+sign*.009, along
                d = np.array([.018, .018, size[2]-.035])
                part(f'rib_{axis}_{side}_{j}', p, d, (43, 83, 68))
    # Raised feet clear the original lift's low CAD feet while the container
    # remains fixed to the chassis, independently of carriage height.
    foot_height = low[2]-deck_top
    if foot_height > 0:
        for i, x in enumerate((low[0]+.055, high[0]-.055)):
            for j, y in enumerate((low[1]+.20, high[1]-.20)):
                part(f'foot_{i}_{j}', [x, y, deck_top+foot_height/2.],
                     [.055, .065, foot_height], (35, 44, 40))
    return parts


def preview_layouts(visual, export, chassis_zero, fixed_ids):
    """Measure a preview-only translation from the existing CAD boundaries."""
    sources = {row['name']: row.get('source', '') for row in export['geometry']}
    console = []
    fixed = []
    for gid in range(visual.ngeom):
        source = sources.get(visual.geom(gid).name, '')
        if '/S300Chassis/' not in source and gid not in fixed_ids:
            continue
        vertices = np.asarray(geom_mesh(visual, gid).vertices)
        world = vertices @ chassis_zero[:3, :3].T + chassis_zero[:3, 3]
        if gid in fixed_ids:
            fixed.append(world)
        else:
            # The control cabinet is the tall chassis component at the rear.
            # Wheels and deck sit below .35m; all original cabinet vertices
            # above that height are retained, including protruding controls.
            part = world[(world[:, 2] > .35) & (world[:, 1] > .30)]
            if len(part):
                console.append(part)
    if not console or not fixed:
        raise ValueError('Control cabinet or fixed lift CAD bounds missing')
    console = np.concatenate(console)
    fixed = np.concatenate(fixed)
    gap = .02
    shift = float(console[:, 1].min() - gap - fixed[:, 1].max())
    if not .05 < shift < .8:
        raise ValueError(f'Unexpected compact lift translation: {shift}')
    # This is an explicitly provisional tray envelope, not a manufactured
    # tower model or an obstacle used by the existing audited V1 motion.
    deck_top = .314
    tray = [[-.25, -.33, deck_top], [.25, .07, deck_top + .90]]
    compact = dict(label=LAYOUT_LABELS['compact'], shift_world_m=[0., shift, 0.],
            control_box_world_bounds_m=[console.min(0).tolist(), console.max(0).tolist()],
            original_fixed_lift_world_bounds_m=[fixed.min(0).tolist(), fixed.max(0).tolist()],
            fixed_lift_to_control_box_gap_m=gap, tray_tower_world_bounds_m=tray,
            tray_tower_assumed_size_m=[.50, .40, .90],
            scope='Layout comparison only; provisional tray envelope and V2 motion not collision-validated')
    outboard = dict(compact, label=LAYOUT_LABELS['outboard'], arm_outboard_default_m=.80,
        arm_outboard_direction_world=[0., -1., 0.],
        scope='Static layout preview with extended plate; full motion, structure and stability not validated')
    # Latest layout: retain the compact mast and original-length plate/arm
    # mount. Replace the tall tray envelope with one low, wide storage bin.
    storage = dict(label=LAYOUT_LABELS['storage'], shift_world_m=[0., shift, 0.],
        storage_bin_world_bounds_m=[[-.28, -.45, .36], [.28, .65, .64]],
        storage_bin_assumed_size_m=[.56, 1.10, .28], storage_bin_open_top=True,
        storage_bin_chassis_fixed=True, storage_bin_deck_top_m=deck_top,
        storage_bin_front_overhang_m=.10,
        scope='Static concept layout with original plate and open tomato bin; no motion or load validation')
    conveyor=conveyor_bin_spec(deck_top)
    conveyor_layout=dict(label=LAYOUT_LABELS['conveyor'],shift_world_m=[0.,shift,0.],
        storage_bin_world_bounds_m=conveyor['bounds'],storage_bin_assumed_size_m=conveyor['nominal_size_m'],
        storage_bin_open_top=True,storage_bin_chassis_fixed=True,storage_bin_deck_top_m=deck_top,
        support_feet_count=0,chassis_gap_m=0.,conveyor=conveyor,
        scope='Flush conveyor bin concept; standalone unloading physics and arm reach checks are separate')
    side=dict(conveyor_layout,label=LAYOUT_LABELS['side'],horizontal_plate=False,
        lift_motor_visible=False,
        mounting_normal_world=[0.,-1.,0.],adapter_size_m=[.20,.020,.22],
        mount_centre_below_carriage_top_m=.11,motor_clearance_below_adapter_m=.015,
        static_arm_positions_rad=np.deg2rad([-60.,0.,60.,-60.,90.,0.]).tolist(),
        nominal_tool_rotation_world=[[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]],
        scope='Side-mounted concept; motion requires a matching checked plan; structure and dynamics not validated')
    return dict(original=dict(label=LAYOUT_LABELS['original'], shift_world_m=[0., 0., 0.]),
                compact=compact, outboard=outboard, storage=storage,conveyor=conveyor_layout,side=side)


def read(path):
    return json.loads(Path(path).read_text())


def prepare(assets, output, elbow):
    assets = assets.resolve(strict=True)
    config = read(assets / 'patrol_assets.json')
    if config.get('start_modes'):
        assets = Path(config['start_modes'][elbow]).resolve(strict=True)
        config = read(assets / 'patrol_assets.json')
    inputs = assets / 'inputs'
    model = mj.MjModel.from_binary_path(str(inputs / 'input_model.mjb'))
    initial = dict(np.load(inputs / 'initial_state.npz', allow_pickle=False))
    packed = stream_manifest(assets)
    if packed:
        section = read(packed)['chunks'][0]
        visual = PackedScene(packed.parent / section['geometry'], section['geometry_sha256'])
    else:
        raise ValueError('This preview requires the existing packed CAD geometry')
    export = read(assets / 'model/export.json')
    base_names = {row['name'] for row in export['geometry']
                  if '/farmily_lift_platform/visuals/link0/' in row.get('source', '')}
    base_ids = [i for i in range(visual.ngeom) if visual.geom(i).name in base_names]
    if not base_ids:
        raise ValueError('Original RB5 base housing not found; refusing to leave an upright duplicate')
    platform_id = model.body('Robot_farmily_lift_platform').id
    if any(visual.geom_bodyid[i] != platform_id for i in base_ids):
        raise ValueError('Unexpected RB5 base housing owner')
    base_local = np.concatenate([geom_mesh(visual, i).vertices for i in base_ids])
    base_center = (base_local.min(0) + base_local.max(0)) / 2
    # Separate the base housing from the platform's CAD mesh group. All its
    # original vertices, colors and local transforms are retained.
    # Packed visuals may contain fewer plant bodies than the physics model.
    # Keep preview-only bodies outside both sets of original body indices.
    if visual.nbody < model.nbody:
        reserved = [f'Preview_reserved_{i}' for i in range(visual.nbody, model.nbody)]
        visual._body_ids.update({name: visual.nbody + i for i, name in enumerate(reserved)})
        visual.body_names = np.append(visual.body_names, reserved)
        visual.nbody = model.nbody
    base_id = visual.nbody
    visual.body_names = np.append(visual.body_names, 'Preview_RB5_base')
    visual._body_ids['Preview_RB5_base'] = base_id
    visual.nbody += 1
    visual.geom_bodyid[base_ids] = base_id
    plate_names = {row['name'] for row in export['geometry']
                   if row.get('source', '').endswith(('/real_lift_carriage/mounting_plate',
                                                     '/collisions/real_mounting_plate'))}
    plate_ids = [i for i in range(visual.ngeom) if visual.geom(i).name in plate_names]
    if not plate_ids:
        raise ValueError('Lift mounting plate not found')
    if any(visual.geom_bodyid[i] != platform_id for i in plate_ids):
        raise ValueError('Unexpected lift mounting plate owner')
    plate_id = visual.nbody
    visual.body_names = np.append(visual.body_names, 'Preview_lift_plate')
    visual._body_ids['Preview_lift_plate'] = plate_id
    visual.nbody += 1
    visual.geom_bodyid[plate_ids] = plate_id
    motor_parts = {f'fixed_{i:02d}' for i in LIFT_MOTOR_COMPONENTS}
    motor_names = {row['name'] for row in export['geometry']
                   if '/real_lift_fixed/' in row.get('source', '')
                   and row['source'].rsplit('/', 1)[-1] in motor_parts}
    motor_ids = [i for i in range(visual.ngeom) if visual.geom(i).name in motor_names]
    chassis_id = model.body('Robot_world').id
    if len(motor_ids) != len(motor_parts) or any(visual.geom_bodyid[i] != chassis_id for i in motor_ids):
        raise ValueError('Expected all 40 original fixed lift motor CAD components')
    motor_local = np.concatenate([geom_mesh(visual, i).vertices for i in motor_ids])
    reference = mj.MjData(model)
    reference.qpos[:] = initial['qpos']
    reference.qpos[model.joint('farmily_lift_height_joint').qposadr[0]] = 0.
    mj.mj_kinematics(model, reference)
    chassis_zero = np.eye(4)
    chassis_zero[:3, :3] = reference.xmat[chassis_id].reshape(3, 3)
    chassis_zero[:3, 3] = reference.xpos[chassis_id]
    platform_zero = np.eye(4)
    platform_zero[:3, :3] = reference.xmat[platform_id].reshape(3, 3)
    platform_zero[:3, 3] = reference.xpos[platform_id]
    carriage_names={row['name'] for row in export['geometry']
        if row.get('source','').endswith('/real_lift_carriage/back_carriage')}
    carriage_local=np.concatenate([geom_mesh(visual,i).vertices for i in range(visual.ngeom)
        if visual.geom(i).name in carriage_names])
    carriage_world=carriage_local@platform_zero[:3,:3].T+platform_zero[:3,3]
    motor_world = motor_local @ chassis_zero[:3, :3].T + chassis_zero[:3, 3]
    plate = config['upright_yaw_mount']['lowered_platform']['plate_world_bounds_zero_lift_m']
    motor_top_gap = .01
    attached_zero = chassis_zero.copy()
    attached_zero[2, 3] += plate[0][2] - motor_top_gap - motor_world[:, 2].max()
    motor_mount = np.linalg.inv(platform_zero) @ attached_zero
    motor_id = visual.nbody
    visual.body_names = np.append(visual.body_names, 'Preview_lift_motor')
    visual._body_ids['Preview_lift_motor'] = motor_id
    visual.nbody += 1
    visual.geom_bodyid[motor_ids] = motor_id
    # Separate the mast and its four original supports from the fixed chassis.
    # V1 uses the exact same world transforms; V2 can translate the whole lift
    # without also moving the control cabinet or leaving the feet behind.
    fixed_names = {row['name'] for row in export['geometry']
                   if '/real_lift_fixed/' in row.get('source', '') and row['name'] not in motor_names}
    fixed_ids = [i for i in range(visual.ngeom) if visual.geom(i).name in fixed_names]
    if len(fixed_ids) != 5 or any(visual.geom_bodyid[i] != chassis_id for i in fixed_ids):
        raise ValueError('Expected mast and four original lift support CAD meshes')
    layouts = preview_layouts(visual, export, chassis_zero, fixed_ids)
    frame_id = visual.nbody
    visual.body_names = np.append(visual.body_names, 'Preview_lift_frame')
    visual._body_ids['Preview_lift_frame'] = frame_id
    visual.nbody += 1
    visual.geom_bodyid[fixed_ids] = frame_id
    selected = [i for i in range(visual.ngeom)
                if visual.body(int(visual.geom_bodyid[i])).name.startswith(('Robot_', 'Tool_', 'Preview_'))]
    base_geometry = output / 'preview_geometry_base.npz'
    pack_scene(visual, selected, base_geometry)
    scripts = Path(__file__).resolve().parent
    asset_python = scripts.parent / '.asset-venv/bin/python'
    subprocess.run([str(asset_python), str(scripts/'robot_vertex_colors.py'),
        str(assets/'model/export.json'), str(base_geometry), str(output/'preview_scene.npz'),
        '--report', str(output/'robot_vertex_colors.json')], check=True)
    base_geometry.unlink()
    scene = PackedScene(output / 'preview_scene.npz')
    return model, initial, scene, dict(
        assets=str(assets), model_sha256=sha256(inputs / 'input_model.mjb'),
        initial_sha256=sha256(inputs / 'initial_state.npz'), platform_id=platform_id,
        base_id=base_id, plate_id=plate_id, motor_id=motor_id,
        frame_id=frame_id, chassis_id=chassis_id, source_fixed_lift_geometry=sorted(fixed_names),
        layouts=layouts,
        source_camera_mount=config['mounted_rgbd'],
        gripper_camera_mount=camera_mount_spec(scene, config['mounted_rgbd']),
        motor_mount_relative_to_plate=motor_mount.tolist(), motor_top_gap_m=motor_top_gap,
        source_motor_geometry=sorted(motor_names),
        source_motor_world_bounds_m=[motor_world.min(0).tolist(), motor_world.max(0).tolist()],
        base_center_local_m=base_center.tolist(),
        carriage_zero_lift_bounds_m=[carriage_world.min(0).tolist(),carriage_world.max(0).tolist()],
        plate_zero_lift_bounds_m=plate, preview_geometry_sha256=sha256(output / 'preview_scene.npz'),
        appearance_report=read(output/'robot_vertex_colors.json'),
        source_base_visuals=sorted(base_names), source_plate_geometry=sorted(plate_names), geometry_count=len(selected),
        arm_bodies=[i for i in range(model.nbody)
                    if model.body(i).name.startswith(('Robot_link', 'Tool_'))])


def body_poses(model, initial, meta, lift, hanging, arm_positions=None, plate_rise=0., layout='original',
               arm_outboard_m=None, jaw_positions=None):
    data = mj.MjData(model)
    data.qpos[:] = initial['qpos']
    if arm_positions is not None:
        data.qpos[2:8] = arm_positions
    if jaw_positions is not None:
        data.qpos[8:10] = jaw_positions
    data.qpos[model.joint('farmily_lift_height_joint').qposadr[0]] = lift
    mj.mj_kinematics(model, data)
    preview_ids = [meta['base_id'], meta['plate_id'], meta['motor_id']]
    if 'frame_id' in meta:
        preview_ids.append(meta['frame_id'])
    poses = np.tile(np.eye(4), (max(preview_ids) + 1, 1, 1))
    poses[:model.nbody, :3, :3] = data.xmat.reshape(-1, 3, 3)
    poses[:model.nbody, :3, 3] = data.xpos
    platform = poses[meta['platform_id']].copy()
    poses[meta['base_id']] = platform
    poses[meta['plate_id']] = platform
    seat = (platform @ np.r_[meta['base_center_local_m'], 1.])[:3]
    bounds = np.asarray(meta['plate_zero_lift_bounds_m'])
    seat[2] = bounds[1, 2] + lift
    target = seat.copy()
    if layout == 'side':
        side=meta['layouts']['side']
        carriage=np.asarray(meta['carriage_zero_lift_bounds_m'])
        # The native base mates on its lower Z face. Turn that axis toward
        # the open side of the mast and seat it on a short vertical adapter.
        source_seat=np.array([platform[0,3],platform[1,3],bounds[1,2]+lift])
        target=np.array([(carriage[0,0]+carriage[1,0])/2.,
            carriage[0,1]-side['adapter_size_m'][1],
            carriage[1,2]+lift-side['mount_centre_below_carriage_top_m']])
        delta=np.eye(4)
        delta[:3,:3]=Rotation.from_euler('x',90,degrees=True).as_matrix()
        delta[:3,3]=target-delta[:3,:3]@source_seat
        ids=meta['arm_bodies']+[meta['base_id']]
        poses[ids]=delta@poses[ids]
    elif hanging:
        target[2] = bounds[0, 2] + lift
        delta = np.eye(4)
        # Rotate about world Y: the elbow stays on the open side of the mast.
        delta[:3, :3] = np.diag([-1., 1., -1.])
        delta[:3, 3] = target - delta[:3, :3] @ seat
        ids = meta['arm_bodies'] + [meta['base_id']]
        poses[ids] = delta @ poses[ids]
    raised = meta['arm_bodies'] + [meta['base_id'], meta['plate_id']]
    if layout != 'side':
        poses[raised, 2, 3] += plate_rise
    # Motor meshes retain their original CAD orientation. The motor is a rigid
    # attachment below the moving plate, including its new mounting offset.
    poses[meta['motor_id']] = poses[meta['plate_id']] @ np.asarray(meta['motor_mount_relative_to_plate'])
    if layout == 'side':
        motor_top=target[2]-side['adapter_size_m'][2]/2.-side['motor_clearance_below_adapter_m']
        poses[meta['motor_id']]=poses[meta['chassis_id']].copy()
        poses[meta['motor_id'],2,3]+=motor_top-meta['source_motor_world_bounds_m'][1][2]
    else:
        target[2] += plate_rise
    if 'frame_id' in meta:
        poses[meta['frame_id']] = poses[meta['chassis_id']]
    if layout != 'original':
        shift = np.asarray(meta['layouts'][layout]['shift_world_m'])
        moved = meta['arm_bodies'] + [meta['platform_id'], *preview_ids]
        poses[moved, :3, 3] += shift
        target += shift
    if layout == 'outboard':
        distance = meta['layouts']['outboard']['arm_outboard_default_m'] if arm_outboard_m is None else arm_outboard_m
        if not np.isfinite(distance) or not 0. <= distance <= 1.20:
            raise ValueError('Arm outboard distance must be within 0..1.20m')
        shift = np.array([0., -distance, 0.])
        poses[meta['arm_bodies'] + [meta['base_id']], :3, 3] += shift
        target += shift
    return poses, target


def extension_bounds(meta, seat, lift, plate_rise):
    """A continuous plate from the existing tip to the outboard base seat."""
    bounds = np.asarray(meta['plate_zero_lift_bounds_m'], float).copy()
    bounds[:, 2] += lift + plate_rise
    bounds[:, 1] += meta['layouts']['outboard']['shift_world_m'][1]
    original_tip = bounds[0, 1]
    new_tip = float(seat[1]) - .11
    if new_tip >= original_tip:
        return None
    bounds[:, 1] = [new_tip, original_tip + .02]
    return bounds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assets', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=5906)
    parser.add_argument('--elbow-mode', choices=['up', 'down'], default='up')
    parser.add_argument('--lift-height', type=float, default=.75)
    parser.add_argument('--pose-file', type=Path, help='Optional checked six-axis static posture for comparison')
    parser.add_argument('--plate-rise-m', type=float, default=.35,
                        help='Move plate and mounted arm upward on the carriage (preview only)')
    parser.add_argument('--workspace-motion', action='store_true',
                        help='Plan random joint motions inside the marked front workspace')
    parser.add_argument('--storage-workspace', action='store_true',
                        help='Check bilateral grip poses and jaw cycles in the V4 bin layout')
    parser.add_argument('--storage-plan', type=Path,
                        help='Load an audited V4 grip_motion.json with matching source and installation')
    parser.add_argument('--conveyor-workspace',action='store_true',help='Check dense near-body poses for the flush V5 conveyor bin')
    parser.add_argument('--conveyor-plan',type=Path,help='Load an audited near-body V5 grip_motion.json')
    parser.add_argument('--conveyor-pose-file',type=Path,
                        help='V5 parked working posture, separate from legacy V1/V4 poses')
    parser.add_argument('--side-plan',type=Path,help='Load an audited V6 side-mounted grip_motion.json')
    parser.add_argument('--side-workspace',action='store_true',help='Generate and audit V6 bilateral grip motions')
    parser.add_argument('--side-seed-coverage',type=Path,
                        help='Optional previous V6 coverage as IK seeds; every new pose and path is rechecked')
    parser.add_argument('--unload-report',type=Path,help='Show a recorded standalone Newton GPU bin-unloading test')
    parser.add_argument('--workspace-seed', type=int, default=20261112)
    parser.add_argument('--workspace-side', choices=['right', 'left'], default='right',
                        help='Select original right or marked opposite left workspace')
    parser.add_argument('--fixed-lift', action='store_true',
                        help='Compare arm-only workspace motion at the initial lift height')
    parser.add_argument('--autoplay', action='store_true', help='Play workspace motion when a browser connects')
    parser.add_argument('--layout', choices=tuple(LAYOUT_LABELS), default='original',
                        help='Initial preview layout; V6 is a static side-mount comparison only')
    args = parser.parse_args()
    if not np.isfinite(args.lift_height) or not 0 <= args.lift_height <= .75:
        parser.error('Lift height must be within 0..0.75m')
    if not np.isfinite(args.plate_rise_m) or not 0 < args.plate_rise_m <= .60:
        parser.error('Plate rise must be greater than 0 and at most 0.60m')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    model, initial, scene, meta = prepare(args.assets, output, args.elbow_mode)
    candidate = None
    if args.pose_file:
        candidate = read(args.pose_file)
        if candidate['joint_names'] != [model.joint(i).name for i in range(2, 8)]:
            raise ValueError('Preview posture joint order mismatch')
        positions = np.asarray(candidate['arm_positions_rad'], float)
        if (positions.shape != (6,) or not np.isfinite(positions).all() or
                np.any(positions < model.jnt_range[2:8, 0]) or np.any(positions > model.jnt_range[2:8, 1])):
            raise ValueError('Preview posture outside actual joint limits')
    motion = None
    if args.workspace_motion:
        from preview_workspace_motion import plan_workspace_motion, WORKSPACE, CAPTURE_CENTER
        motion_initial = {key: value.copy() for key, value in initial.items()}
        motion_initial['qpos'][1] = args.lift_height
        if candidate:
            motion_initial['qpos'][2:8] = candidate['arm_positions_rad']
        displayed, _ = body_poses(model, initial, meta, args.lift_height, True,
                                  motion_initial['qpos'][2:8], args.plate_rise_m)
        motion = plan_workspace_motion(model, motion_initial, meta, displayed, output,
                                       seed=args.workspace_seed, move_lift=not args.fixed_lift,
                                       side=args.workspace_side)
    storage_motion = None
    storage_coverage = None
    storage_plan_file = None
    if args.storage_workspace or args.storage_plan:
        from preview_grip_workspace import plan_storage_motion, load_storage_motion
        storage_initial = {key: value.copy() for key,value in initial.items()}
        storage_initial['qpos'][1] = args.lift_height
        if candidate: storage_initial['qpos'][2:8] = candidate['arm_positions_rad']
        displayed, _ = body_poses(model, initial, meta, args.lift_height, True,
            storage_initial['qpos'][2:8], args.plate_rise_m, 'storage')
        parts = storage_bin_parts(meta['layouts']['storage']['storage_bin_world_bounds_m'],
                                  meta['layouts']['storage']['storage_bin_deck_top_m'])
        if args.storage_plan:
            storage_motion = load_storage_motion(args.storage_plan,meta,model,storage_initial,
                displayed,scene,parts,output/'storage_validation')
            storage_plan_file = args.storage_plan.resolve()
        else:
            plan_output = output/'storage_grip'
            plan_output.mkdir()
            storage_motion = plan_storage_motion(model,storage_initial,meta,displayed,scene,parts,plan_output)
            storage_plan_file = plan_output/'grip_motion.json'
        storage_coverage = read(storage_plan_file.parent/storage_motion.report['coverage_file'])
    conveyor_motion=None;conveyor_coverage=None;conveyor_plan_file=None
    conveyor_spec=meta['layouts']['conveyor']['conveyor']
    conveyor_parts=conveyor_grip_parts(conveyor_spec)
    if args.conveyor_workspace or args.conveyor_plan:
        from preview_grip_workspace import plan_storage_motion,load_storage_motion
        conveyor_initial={key:value.copy()for key,value in initial.items()}
        conveyor_initial['qpos'][1]=args.lift_height
        if candidate:conveyor_initial['qpos'][2:8]=candidate['arm_positions_rad']
        conveyor_nominal=None
        if args.conveyor_pose_file:
            conveyor_pose=read(args.conveyor_pose_file)
            conveyor_nominal=conveyor_pose.get('nominal_tool_rotation_world')
            if conveyor_pose['joint_names']!=[model.joint(i).name for i in range(2,8)]:
                raise ValueError('V5 parked posture joint order mismatch')
            conveyor_initial['qpos'][2:8]=conveyor_pose['arm_positions_rad']
            conveyor_initial['qpos'][1]=float(conveyor_pose.get('reference_lift_m',args.lift_height))
            if not np.isfinite(conveyor_initial['qpos'][1]) or not 0<=conveyor_initial['qpos'][1]<=.75:
                raise ValueError('V5 parked lift must be within 0..0.75m')
        displayed,_=body_poses(model,initial,meta,conveyor_initial['qpos'][1],True,
            conveyor_initial['qpos'][2:8],args.plate_rise_m,'conveyor')
        if args.conveyor_plan:
            conveyor_motion=load_storage_motion(args.conveyor_plan,meta,model,conveyor_initial,
                displayed,scene,conveyor_parts,output/'conveyor_validation',require_posture_guard=True)
            conveyor_plan_file=args.conveyor_plan.resolve()
        else:
            folder=output/'conveyor_grip';folder.mkdir()
            conveyor_motion=plan_storage_motion(model,conveyor_initial,meta,displayed,scene,conveyor_parts,
                folder,profile='near',layout_key='conveyor',nominal_rotation_world=conveyor_nominal)
            conveyor_plan_file=folder/'grip_motion.json'
        conveyor_coverage=read(conveyor_plan_file.parent/conveyor_motion.report['coverage_file'])
    side_motion=None;side_coverage=None;side_plan_file=None
    if args.side_plan or args.side_workspace:
        from preview_grip_workspace import load_storage_motion,plan_storage_motion
        side_initial={key:value.copy()for key,value in initial.items()}
        side_q=side_initial['qpos'].copy()
        side_q[1]=args.lift_height
        side_q[2:8]=meta['layouts']['side']['static_arm_positions_rad']
        if args.side_plan:
            side_report=read(args.side_plan)
            if side_report.get('layout_key')!='side':raise ValueError('V6 requires a side-mount plan')
            side_q=np.asarray(side_report['initial_positions'],float)
        if side_q.shape!=(10,)or not np.isfinite(side_q).all()or side_q[0]!=side_initial['qpos'][0]:
            raise ValueError('Invalid V6 initial state')
        side_initial['qpos'][:]=side_q
        displayed,_=body_poses(model,initial,meta,side_q[1],True,side_q[2:8],0.,'side',jaw_positions=side_q[8:10])
        if args.side_plan:
            side_motion=load_storage_motion(args.side_plan,meta,model,side_initial,displayed,scene,
                conveyor_parts,output/'side_validation',require_posture_guard=True)
            side_plan_file=args.side_plan.resolve()
        else:
            folder=output/'side_grip';folder.mkdir()
            side_motion=plan_storage_motion(model,side_initial,meta,displayed,scene,conveyor_parts,
                folder,profile='near',layout_key='side',
                seed_coverage=args.side_seed_coverage,
                nominal_rotation_world=meta['layouts']['side']['nominal_tool_rotation_world'])
            side_plan_file=folder/'grip_motion.json'
        side_coverage=read(side_plan_file.parent/side_motion.report['coverage_file'])
    grip_motions={'storage':storage_motion,'conveyor':conveyor_motion,'side':side_motion}
    grip_coverages={'storage':storage_coverage,'conveyor':conveyor_coverage,'side':side_coverage}
    grip_plan_files={'storage':storage_plan_file,'conveyor':conveyor_plan_file,'side':side_plan_file}
    unload_report=None;unload_states=None
    if args.unload_report:
        unload_report,unload_states=load_unload_record(args.unload_report,conveyor_spec)
    has_motion = motion is not None or any(v is not None for v in grip_motions.values())
    server = viser.ViserServer(host='0.0.0.0', port=args.port,
                               label='Farmily · 로봇 설치 형상', verbose=False)
    if server.get_port() != args.port:
        server.stop()
        raise RuntimeError(f'Port {args.port} is busy')
    server.scene.set_up_direction('+z')
    server.scene.add_grid('/ground', plane='xy', width=4., height=4.,
                          cell_size=.1, section_size=.5)
    status = server.gui.add_markdown('**리프트 상판 아래 매달림형**\n\n로봇 형상을 준비하고 있습니다.')
    layout_choice = server.gui.add_dropdown('배치 버전', options=tuple(LAYOUT_LABELS.values()),
                                             initial_value=LAYOUT_LABELS[args.layout])
    show_tray = server.gui.add_checkbox('트레이타워 배치틀', initial_value=True,
                                      visible=args.layout in ('compact', 'outboard'))
    show_bin = server.gui.add_checkbox('토마토 보관함 표시', initial_value=True,
                                      visible=args.layout in STORAGE_LAYOUTS)
    show_checks = server.gui.add_checkbox('검사 지점 표시',initial_value=True,
        visible=grip_motions.get(args.layout) is not None)
    arm_outboard = server.gui.add_slider('로봇암 돌출거리 (m)', min=0., max=1.20, step=.01,
        initial_value=meta['layouts']['outboard']['arm_outboard_default_m'], visible=args.layout == 'outboard')
    mount = server.gui.add_dropdown('설치 방향', options=('매달림형', '기존 정립형'), initial_value='매달림형')
    plate_position = server.gui.add_dropdown('받침판 위치', options=('상단 · 변경 위치', '하단 · 이전 위치'), initial_value='상단 · 변경 위치')
    posture = server.gui.add_dropdown('로봇 자세', options=('기존 자세', '그리퍼 아래쪽 후보'),
        initial_value='그리퍼 아래쪽 후보') if candidate else None
    height = server.gui.add_slider('리프트 높이 (m)', min=0., max=.75, step=.01, initial_value=args.lift_height)
    default_view = ('보관함 배치' if args.layout=='side'
                    else '양쪽 검사' if grip_motions.get(args.layout) is not None
                    else '보관함 배치' if args.layout in STORAGE_LAYOUTS else '외측 배치' if args.layout == 'outboard'
                    else '배치 비교' if args.layout == 'compact'
                    else '작업 영역' if motion else '전체 사선')
    view_order = height.order + .5
    quit_event = threading.Event()
    server.gui.add_button('화면 닫기').on_click(lambda _: quit_event.set())
    server.scene.add_frame('/bodies', show_axes=False)
    handles, camera_handles = populate_camera_scene(server, scene, meta['gripper_camera_mount'])
    tray_bounds = np.asarray(meta['layouts']['compact']['tray_tower_world_bounds_m'])
    tray_lines = [box_edges(tray_bounds)]
    for level in np.linspace(tray_bounds[0, 2], tray_bounds[1, 2], 7)[1:-1]:
        bounds = tray_bounds.copy()
        bounds[:, 2] = level
        tray_lines.append(box_edges(bounds))
    tray_frame = server.scene.add_line_segments('/tray_tower_envelope',
        points=np.concatenate(tray_lines), colors=(40, 135, 245),
        thickness=1.8, thickness_units='screen', visible=args.layout in ('compact', 'outboard'))
    bin_layout = meta['layouts']['storage']
    bin_parts = storage_bin_parts(bin_layout['storage_bin_world_bounds_m'],
                                   bin_layout['storage_bin_deck_top_m'])
    bin_frame = server.scene.add_frame('/tomato_storage_bin', show_axes=False,
                                        visible=args.layout == 'storage')
    for part in bin_parts:
        server.scene.add_box(f'/tomato_storage_bin/{part["name"]}',
            position=part['position'], dimensions=part['dimensions'], color=part['color'])
    conveyor_handles=draw_conveyor(server,conveyor_spec)
    unload_button=server.gui.add_button('GPU 배출 결과 보기',visible=args.layout=='conveyor' and unload_states is not None)
    unload_tomatoes=[]
    if unload_states is not None:
        for i in range(unload_states['tomato_positions'].shape[1]):
            unload_tomatoes.append(server.scene.add_icosphere(f'/unload_tomato_{i}',radius=.018,
                color=(200+(i%3)*10,35+(i%4)*5,25),visible=False))
    extension = server.scene.add_box('/arm_mount_extension', color=(170, 167, 169),
                                     dimensions=(.23, .02, .01), visible=False)
    side_adapter=server.scene.add_box('/side_mount_adapter',color=(170,174,180),
        dimensions=meta['layouts']['side']['adapter_size_m'],visible=False)
    # Viser invokes callbacks synchronously for server-side value assignments.
    # Restoring height/posture while selecting a layout can therefore re-enter
    # update() on this same thread; a plain Lock deadlocks that transition.
    lock = threading.RLock()
    playback = dict(elapsed_s=0., running=False, frames=0)
    unloading=dict(enabled=False,elapsed_s=0.,running=False)
    def selected_layout():
        return next(key for key, label in LAYOUT_LABELS.items() if label == layout_choice.value)
    def active_motion():
        if unloading['enabled']:return None
        return motion if selected_layout() == 'original' else grip_motions.get(selected_layout())
    def render(poses):
        with server.atomic():
            for bid, handle in handles.items():
                handle.position = poses[bid, :3, 3]
                xyzw = Rotation.from_matrix(poses[bid, :3, :3]).as_quat()
                handle.wxyz = xyzw[[3, 0, 1, 2]]
            update_camera(camera_handles, poses, scene, meta['gripper_camera_mount'], selected_layout())
    def update(_=None):
        with lock:
            arm = candidate['arm_positions_rad'] if posture and posture.value == '그리퍼 아래쪽 후보' else None
            rise = args.plate_rise_m if plate_position.value == '상단 · 변경 위치' else 0.
            layout = selected_layout()
            if layout == 'side':
                arm=meta['layouts']['side']['static_arm_positions_rad']
                rise=0.
            grip=grip_motions.get(layout)
            view_lift = height.value
            jaws = None
            if grip is not None:
                state, _ = grip.sample(0. if unloading['enabled'] else playback['elapsed_s'])
                view_lift, arm, jaws = state[0],state[1:7],state[7:9]
            poses, seat = body_poses(model, initial, meta, view_lift, mount.value == '매달림형', arm, rise,
                                    layout, arm_outboard.value, jaw_positions=jaws)
            render(poses)
            handles[meta['plate_id']].visible=layout!='side'
            handles[meta['motor_id']].visible=layout!='side'
            side_adapter.visible=layout=='side'
            if side_adapter.visible:
                side_adapter.position=seat+np.array([0.,meta['layouts']['side']['adapter_size_m'][1]/2.,0.])
            tray_frame.visible = layout in ('compact', 'outboard') and show_tray.value
            bin_frame.visible = layout == 'storage' and show_bin.value
            conveyor_handles['root'].visible=layout in CONVEYOR_LAYOUTS and show_bin.value
            if not unloading['enabled']:
                update_conveyor(conveyor_handles,conveyor_spec,0.,np.zeros(31),0.)
                for tomato in unload_tomatoes:tomato.visible=False
            support = extension_bounds(meta, seat, height.value, rise) if layout == 'outboard' else None
            extension.visible = support is not None
            if support is not None:
                extension.dimensions = tuple(support[1]-support[0])
                extension.position = (support[1]+support[0])/2.
            status.content = (f'**{layout_choice.value} · {mount.value}**\n\n'
                f'리프트 {height.value:.2f}m · 베이스 장착면 높이 {seat[2]:.3f}m\n\n'
                f'받침판: {plate_position.value}\n\n'
                '마우스로 회전·확대할 수 있습니다.\n\n형상 배치만 표시하며 동작·물리 검증은 실행하지 않습니다.')
            checked = bool(layout == 'original' and arm is not None and mount.value == '매달림형' and
                           rise == 0. and abs(height.value - candidate['reference_lift_m']) < 1e-8)
            if layout == 'side':
                status.content=(f'**{layout_choice.value} · 정지 형상**\n\n'
                    '수평 상판 제거 · 이동 캐리지 옆면에 베이스 장착\n\n'
                    '가정 어댑터: 폭 200 × 높이 220 × 두께 20mm\n\n'
                    f'베이스 중심 높이 {seat[2]*100:.1f}cm · 리프트 모터 제거\n\n'
                    '기존 20cm 컨베이어 보관함 유지\n\n'
                    '이번 버전은 형상만 표시합니다. 모션·도달범위·간섭 검증은 실행하지 않았습니다.')
            elif layout in STORAGE_LAYOUTS:
                plate_length = np.diff(np.asarray(meta['plate_zero_lift_bounds_m']), axis=0)[0, 1]
                status.content = (f'**{layout_choice.value} · 형상 비교**\n\n'
                    f'상판 원래 길이 {plate_length*1000:.0f}mm · 로봇암 장착 위치 복원\n\n'
                    '차체 위 토마토 보관함 · 윗면 열림\n\n'
                    f'가정 치수: 길이 1,100 × 폭 560 × 높이 {200 if layout=="conveyor" else 280}mm\n\n'
                    f'리프트 {height.value:.2f}m · 보관함은 차체에 고정\n\n'
                    '형상 미리보기입니다. 보관함을 포함한 동작·적재하중 검증은 별도입니다.')
            elif layout != 'original':
                delta = meta['layouts']['compact']['shift_world_m'][1]
                status.content = (f'**{layout_choice.value} · 형상 비교**\n\n'
                    f'리프트를 컨트롤박스 쪽으로 {delta*100:.1f}cm 이동 · 간격 2cm\n\n'
                    + (f'로봇암을 트레이 앞쪽으로 {arm_outboard.value*100:.0f}cm 추가 이동 · 연장 상판\n\n'
                       if layout == 'outboard' else '') +
                    f'리프트 높이 {height.value:.2f}m · 베이스 장착면 {seat[2]:.3f}m\n\n'
                    '파란 틀: 트레이타워 가정 공간 500×400×900mm\n\n'
                    '상판·모터·로봇은 리프트와 함께 이동합니다.\n\n'
                    '새 배치는 형상 비교용입니다. 동작·간섭·지지 강성·전도 안정성은 미검증입니다.')
            elif arm is not None:
                status.content += ('\n\n아래쪽 후보: 관절 범위와 정적 간섭 확인. 이동 경로는 미검증입니다.' if checked else
                                   '\n\n이전 받침판 위치에서 확인한 자세입니다. 변경 배치의 간섭은 미검증입니다.')
            report = dict(schema='s300-hanging-robot-preview-v1', **meta,
                port=args.port, mount='리프트 옆면' if layout=='side' else mount.value, lift_height_m=height.value,
                layout=layout, layout_label=layout_choice.value,
                layout_shift_world_m=meta['layouts'][layout]['shift_world_m'],
                arm_outboard_m=arm_outboard.value if layout == 'outboard' else 0.,
                arm_mount_extension_bounds_m=support.tolist() if support is not None else None,
                source_plate_length_m=float(np.diff(np.asarray(meta['plate_zero_lift_bounds_m']), axis=0)[0, 1]),
                horizontal_plate_visible=layout!='side',
                lift_motor_visible=layout!='side',
                side_mount_adapter_visible=side_adapter.visible,
                side_mount_adapter_position_m=list(side_adapter.position) if side_adapter.visible else None,
                tray_envelope_visible=tray_frame.visible,
                active_tray_tower_world_bounds_m=tray_bounds.tolist() if layout in ('compact', 'outboard') else None,
                storage_bin_visible=bin_frame.visible,
                active_storage_bin_world_bounds_m=meta['layouts'][layout]['storage_bin_world_bounds_m'] if layout in STORAGE_LAYOUTS else None,
                storage_bin_geometry=conveyor_parts if layout in CONVEYOR_LAYOUTS else bin_parts if layout=='storage'else None,
                conveyor_visible=conveyor_handles['root'].visible,
                conveyor_unloading_report=str(args.unload_report)if args.unload_report else None,
                mounting_contact_world_m=seat.tolist(), body_poses=poses.tolist(),
                active_camera_body='Tool_fixed' if layout=='side' else 'Robot_link5',
                world_from_camera_optical=optical_world(poses,scene,meta['gripper_camera_mount'],layout,meta['source_camera_mount']).tolist(),
                physics_steps=0, motion_validation_performed=False,
                scope='CAD appearance preview; source physics models and patrol assets are not modified')
            report.update(arm_positions_rad=list(arm) if arm is not None else initial['qpos'][2:8].tolist(),
                posture='옆면 장착 형상용 정지 자세' if layout=='side' else posture.value if posture else '기존 자세',
                candidate_static_check_applies=checked,
                plate_rise_m=rise, plate_position='상판 없음' if layout=='side' else plate_position.value,
                candidate_pose=candidate if arm is not None and layout!='side' else None)
            if motion and layout == 'original':
                report.update(workspace_motion='workspace_motion.json',
                              motion_validation_performed=True, dynamics_validation_performed=False,
                              scope='Checked kinematic joint-motion preview; Newton dynamics not executed')
            if grip is not None:
                report.update(grip_motion_file=str(grip_plan_files[layout]),
                    grip_summary=grip.report['summary'],
                    posture_policy=grip.report.get('posture_policy'),
                    posture_audit=grip.report.get('posture_audit'),
                    motion_validation_performed=True,dynamics_validation_performed=False,
                    scope='Audited bilateral reach, insertion and jaw-cycle preview; no dynamics or fruit retention validation')
            (output / 'preview.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
            if active_motion() is None and not unloading['enabled']:
                (output/'workspace_runtime.json').write_text(json.dumps(dict(
                    layout=layout, running=False, lift_height_m=height.value, physics_steps=0,
                    motion_validation_performed=False, body_poses=poses.tolist()))+'\n')
    if has_motion:
        from preview_workspace_motion import WORKSPACE, CAPTURE_CENTER
        # The audited trajectory is tied to this installation. Other mounting
        # choices remain available in the ordinary static preview mode.
        for control in (mount, plate_position, height, posture):
            if control is not None:
                control.disabled = True
        def box_outline(name, bounds, color):
            return server.scene.add_line_segments(name, points=box_edges(bounds), colors=color,
                thickness=1.5, thickness_units='screen')
        workspaces = [box_outline('/workspace', WORKSPACE, (0, 220, 120))] if motion else []
        if motion and args.workspace_side == 'left':
            workspaces.append(box_outline('/opposite_workspace', motion.report['workspace_world_bounds_m'], (255, 65, 40)))
        tool_id = model.body('Tool_fixed').id
        box_outline(f'/bodies/b{tool_id}/capture_volume',
            [CAPTURE_CENTER-np.array([.048, .079, .042]),
             CAPTURE_CENTER+np.array([.048, .079, .042])], (250, 210, 40))
        server.scene.add_icosphere(f'/bodies/b{tool_id}/capture_center', radius=.008,
            color=(0, 160, 220), position=CAPTURE_CENTER)
        first_motion = next(v for v in (motion,storage_motion,conveyor_motion,side_motion)if v is not None)
        target_marker = server.scene.add_icosphere('/workspace_target', radius=.018,
            color=(255, 140, 20), position=first_motion.segments[0]['target_capture_m'])
        grip_scenes={}
        for key,grip in grip_motions.items():
            if grip is None:continue
            work=[box_outline('/'+key+'_workspace/'+side,np.array(bounds),(245,65,45))
                  for side,bounds in grip.report['regions'].items()]
            grid = grip_coverages[key]['grid']
            points = server.scene.add_point_cloud('/'+key+'_grip_checks',
                points=np.array([r['target_capture_m']for r in grid]),
                colors=np.array([(35,190,125)if r['grip_cycle_checked']else(60,150,230)if r['reachable']else(220,95,85)
                                 for r in grid],dtype=np.uint8),point_size=.009,point_shape='circle')
            grip_scenes[key]=dict(workspaces=work,points=points)
        start_button = server.gui.add_button('랜덤 동작 시작 / 계속')
        pause_button = server.gui.add_button('일시정지')
        reset_button = server.gui.add_button('처음부터 재생')
        home_button = server.gui.add_button('선택한 시작 자세 보기',visible=False)
        speed = server.gui.add_slider('재생 배속', min=.25, max=2., step=.25, initial_value=1.)
        storage_seek_buttons = []
        if any(v is not None for v in grip_motions.values()):
            def seek_storage(side, lowest=False):
                with lock:
                    grip=grip_motions.get(selected_layout())
                    if grip is None:return
                    unloading['enabled']=False;unloading['running']=False
                    start_button.label='그리핑 검사 시작 / 계속'
                    selected_index=None
                    if lowest:
                        grid=grip_coverages[selected_layout()]['grid']
                        candidates=[i for i,r in enumerate(grid) if r['side']==side and r['grip_cycle_checked']]
                        if not candidates:return
                        selected_index=min(candidates,key=lambda i:(grid[i]['target_capture_m'][2],
                            abs(grid[i]['target_capture_m'][0]),grid[i]['target_capture_m'][1]))
                    elapsed = 0.
                    for segment in grip.segments:
                        if (segment['side'] == side and segment.get('coverage_index') is not None
                                and (selected_index is None or segment['coverage_index']==selected_index)):
                            break
                        elapsed += segment['duration_s']
                    playback.update(elapsed_s=elapsed,running=True)
                update()
            for side,label in (('right','오른쪽 구간 보기'),('left','왼쪽 구간 보기')):
                button = server.gui.add_button(label)
                button.on_click(lambda _, side=side: seek_storage(side))
                storage_seek_buttons.append(button)
            if grip_motions.get('side') is not None:
                for side,label in (('right','오른쪽 아래 보기'),('left','왼쪽 아래 보기')):
                    button=server.gui.add_button(label)
                    button.on_click(lambda _,side=side:seek_storage(side,lowest=True))
                    storage_seek_buttons.append(button)
        def play(_):
            with lock:
                if unloading['enabled']:unloading['running']=True
                if active_motion() is not None:
                    playback['running'] = True
        def pause(_):
            with lock:
                playback['running'] = False
                unloading['running']=False
        def reset(_):
            with lock:
                if unloading['enabled']:unloading.update(elapsed_s=0.,running=True)
                if active_motion() is not None:
                    playback.update(elapsed_s=0., running=True)
        def show_home(_):
            with lock:
                grip=grip_motions.get(selected_layout())
                if grip is None:return
                unloading.update(enabled=False,running=False)
                playback.update(elapsed_s=0.,running=False)
                start_button.label='그리핑 검사 시작 / 계속'
            update()
        start_button.on_click(play)
        pause_button.on_click(pause)
        reset_button.on_click(reset)
        home_button.on_click(show_home)
    def camera(client, selected_view=default_view):
        if selected_view == '카메라 장착부':
            tool = handles[scene.body('Tool_fixed').id]
            rotation = Rotation.from_quat(np.asarray(tool.wxyz)[[1,2,3,0]]).as_matrix()
            target = np.asarray(tool.position) + rotation @ np.array([0., .045, .080])
            with client.atomic():
                client.camera.up_direction = (0.,0.,1.)
                client.camera.position = target + .65*rotation[:,1] + np.array([0.,0.,.08])
                client.camera.look_at = target
                client.camera.fov = np.deg2rad(45.)
            return
        workspace_x = -.20 if args.workspace_side == 'left' else .20
        target = np.array([workspace_x, -.18, .80]) if selected_view == '작업 영역' else np.array([0., -.24, .85])
        if selected_view == '배치 비교':
            target = np.array([0., .32, .85])
        if selected_view == '외측 배치':
            target = np.array([0., -.15, .85])
        if selected_view == '보관함 배치':
            target = np.array([0., .28, .80])
        if selected_view == '양쪽 검사':
            target = np.array([0., .25, .75])
        offset = {'작업 영역': [0., -2.1, .12], '전체 사선': [1.1, -1.45, .35], '정면': [0., -1.9, .10],
                  '측면': [1.9, 0., .10], '로봇 확대': [.85, -1.05, .10], '위에서': [0., -.25, 2.6],
                  '배치 비교': [-1.9, 0., .25], '외측 배치': [-2.3, 0., .25],
                  '보관함 배치': [-1.9, -.40, .65], '양쪽 검사': [0., -1.7, 1.9]}[selected_view]
        # Keep the same vertical orbit axis for every preset. An exactly
        # overhead view with Z up is singular, so tilt the top preset slightly
        # rather than turning the orbit axis onto world Y.
        with client.atomic():
            client.camera.up_direction = (0., 0., 1.)
            client.camera.position = target + offset
            client.camera.look_at = target
            client.camera.fov = np.deg2rad(50. if selected_view in ('작업 영역', '위에서') else
                                         60. if selected_view in ('배치 비교', '외측 배치', '보관함 배치', '양쪽 검사') else 75.)
    @server.on_client_connect
    def on_connect(client):
        # View selection and camera movement belong to this browser only.
        # In particular a QA/second browser must not rotate another user's view.
        view = client.gui.add_dropdown('시점',
            options=('작업 영역', '전체 사선', '정면', '측면', '로봇 확대', '위에서', '배치 비교', '외측 배치', '보관함 배치', '양쪽 검사', '카메라 장착부'),
            initial_value=default_view, order=view_order)
        view.on_update(lambda event: camera(client, event.target.value))
        camera(client)
        # select_layout() already applies the requested initial autoplay.
        # A second browser must preserve an explicit pause/home selection.
    mount.on_update(update)
    plate_position.on_update(update)
    if posture:posture.on_update(update)
    height.on_update(lambda _: update() if active_motion() is None else None)
    arm_outboard.on_update(lambda _: update() if selected_layout() == 'outboard' else None)
    show_tray.on_update(update)
    show_bin.on_update(update)
    if has_motion:
        def toggle_checks(_=None):
            for key,handles in grip_scenes.items():
                handles['points'].visible=key==selected_layout() and show_checks.value
        show_checks.on_update(toggle_checks)
    def begin_unloading(_=None):
        if unload_states is None or selected_layout()!='conveyor':return
        with lock:
            playback.update(elapsed_s=0.,running=False)
            unloading.update(enabled=True,elapsed_s=0.,running=True)
            if has_motion:
                target_marker.visible=False
                start_button.label='배출 재생 / 계속'
                for control in (start_button,pause_button,reset_button,speed):control.disabled=False
        update()
    unload_button.on_click(begin_unloading)
    def select_layout(_=None):
        with lock:
            layout = selected_layout()
            unloading.update(enabled=False,running=False,elapsed_s=0.)
            active = active_motion()
            playback.update(elapsed_s=0., running=bool(active is not None and args.autoplay))
            show_tray.visible = layout in ('compact', 'outboard')
            show_bin.visible = layout in STORAGE_LAYOUTS
            show_checks.visible = grip_motions.get(layout) is not None
            unload_button.visible=layout=='conveyor'and unload_states is not None
            arm_outboard.visible = layout == 'outboard'
            mount.visible=plate_position.visible=layout!='side'
            if posture is not None:posture.visible=layout!='side'
            for control in (mount, plate_position, height, posture):
                if control is not None:
                    control.disabled = active is not None
            if has_motion:
                home_button.visible=bool(active is not None and
                    (layout=='side' or layout=='conveyor'and active.report.get('start_search')))
                for control in (start_button, pause_button, reset_button, speed):
                    control.disabled = active is None
                for control in storage_seek_buttons:
                    control.visible = grip_motions.get(layout) is not None
                start_button.label = '그리핑 검사 시작 / 계속' if layout in STORAGE_LAYOUTS else '랜덤 동작 시작 / 계속'
                for handle in workspaces:
                    handle.visible = layout == 'original'
                target_marker.visible = active is not None
                for key,handles in grip_scenes.items():
                    for handle in handles['workspaces']:handle.visible=layout==key
                    handles['points'].visible=layout==key and show_checks.value
                if active is not None:
                    mount.value = '매달림형'
                    plate_position.value = '상단 · 변경 위치'
                    if posture:
                        posture.value = '그리퍼 아래쪽 후보'
            height.value = args.lift_height
        update()
    layout_choice.on_update(select_layout)
    select_layout()
    (output / 'ready.json').write_text(json.dumps(dict(port=args.port, url=f'http://localhost:{args.port}',
        physics_steps=0, ready=True)) + '\n')
    print(f'Hanging robot preview ready: http://localhost:{args.port}', flush=True)
    signal.signal(signal.SIGTERM, lambda *_: quit_event.set())
    signal.signal(signal.SIGINT, lambda *_: quit_event.set())
    try:
        last = time.monotonic()
        last_record = last - 1.
        while not quit_event.wait(1/30 if has_motion or unload_states is not None else .25):
            now = time.monotonic()
            if unloading['enabled'] and selected_layout()=='conveyor':
                with lock:
                    if unloading['running']and server.get_clients():
                        unloading['elapsed_s']+=(now-last)*(speed.value if has_motion else 1.)
                    if unloading['elapsed_s']>=unload_states['time'][-1]:
                        unloading['elapsed_s']=float(unload_states['time'][-1]);unloading['running']=False
                    frame=unload_frame(unload_states,unloading['elapsed_s'])
                    update_conveyor(conveyor_handles,conveyor_spec,float(frame['gate_angle']),
                        frame['roller_angles'],float(frame['belt_distance']))
                    for handle,position in zip(unload_tomatoes,frame['tomato_positions']):
                        handle.visible=True;handle.position=position
                    if now-last_record>=.5:
                        positions=frame['tomato_positions']
                        record_index=max(0,min(len(unload_states['time'])-1,
                            int(np.searchsorted(unload_states['time'],unloading['elapsed_s'],side='right'))-1))
                        discharged=int(unload_states['discharged_count'][record_index])
                        status.content=(f'**V5 · GPU 배출 결과 재생 · {unloading["elapsed_s"]:.1f}초**\n\n'
                            f'배출문 {abs(np.degrees(frame["gate_angle"])):.0f}도 · 시험 열매 {discharged}/{len(unload_tomatoes)}개 배출\n\n'
                            '높이 20cm · 차체와 틈 0mm · 바닥 전체 벨트와 하부 롤러\n\n'
                            '보관함 단독 Newton GPU 물리 시험 기록입니다. 팔은 초기 자세로 고정합니다.')
                        runtime=dict(layout='conveyor',mode='unload_gpu_replay',**unloading,
                            discharged=discharged,gate_angle_rad=float(frame['gate_angle']),
                            belt_distance_m=float(frame['belt_distance']),
                            tomato_positions=positions.tolist(),physics_live=False,
                            physics_source=str(args.unload_report))
                        (output/'workspace_runtime.json').write_text(json.dumps(runtime)+'\n')
                        last_record=now
                last=now
                continue
            if active_motion() is not None:
                with lock:
                    active = active_motion()
                    layout = selected_layout()
                    if active is None:
                        last = now
                        continue
                    if playback['running'] and server.get_clients():
                        playback['elapsed_s'] += (now-last) * speed.value
                    configuration, sample = active.sample(playback['elapsed_s'])
                    lift = sample.get('lift_height_m', args.lift_height)
                    arm = configuration[1:7] if len(configuration) in (7,9) else configuration
                    jaws = configuration[7:9] if len(configuration) == 9 else None
                    poses, seat = body_poses(model, initial, meta, lift, True,
                                          arm, args.plate_rise_m,layout,jaw_positions=jaws)
                    render(poses)
                    if layout=='side':
                        side_adapter.position=seat+np.array([0.,meta['layouts']['side']['adapter_size_m'][1]/2.,0.])
                    target_marker.position = sample['target_capture_m']
                    if layout in STORAGE_LAYOUTS:
                        index = sample.get('coverage_index')
                        target_marker.visible = index is not None
                        if index is not None:
                            target_marker.position = grip_coverages[layout]['grid'][index]['target_capture_m']
                    playback['frames'] += 1
                    if now-last_record >= .5:
                        centre = (poses[tool_id, :3, 3] + poses[tool_id, :3, :3] @ CAPTURE_CENTER)
                        runtime = dict(**playback, **sample, arm_positions_rad=arm.tolist(),
                            layout=layout,
                            active_camera_body='Tool_fixed' if layout=='side' else 'Robot_link5',
                            world_from_camera_optical=optical_world(poses,scene,meta['gripper_camera_mount'],layout,meta['source_camera_mount']).tolist(),
                            capture_center_world_m=centre.tolist(), physics_steps=0,
                            plate_mount_height_m=float(seat[2]),
                            motor_world_position_m=poses[meta['motor_id'], :3, 3].tolist() if layout!='side' else None,
                            lift_motor_visible=layout!='side',
                            side_mount_adapter_position_m=list(side_adapter.position) if layout=='side' else None)
                        if jaws is not None:runtime['jaw_positions_m'] = jaws.tolist()
                        runtime['lift_height_m'] = lift
                        height.value = lift
                        (output/'workspace_runtime.json').write_text(json.dumps(runtime)+'\n')
                        label = '진입' if sample['entry'] else f"목표 {sample['segment']}/{len(active.segments)-1}"
                        state = '재생 중' if playback['running'] else '일시정지'
                        if layout in STORAGE_LAYOUTS:
                            grip=grip_motions[layout]
                            approach_mm=grip.report['approach_distance_m']*1000
                            phases = dict(transfer='다음 위치로 이동',approach=f'{approach_mm:.0f}mm 진입',close='그리퍼 닫기',
                                          open='그리퍼 열기',retreat='후퇴')
                            summary = grip.report['summary']
                            region_z=next(iter(grip.report['regions'].values()))
                            status.content = (f'**{"V6 · 옆면 장착" if layout=="side" else "V5 · 바로 옆" if layout=="conveyor" else "V4 · 양쪽"} 그리핑 검사 · {state}**\n\n'
                                f'{"오른쪽" if sample["side"] == "right" else "왼쪽"} · {phases[sample["phase"]]} · {playback["elapsed_s"]:.1f}초\n\n'
                                f'리프트 {lift:.3f}m · 보관함은 차체에 고정\n\n'
                                + ('기둥과 최소 10cm · 엘보우 20–140도 · 손목 치켜듦 제한\n\n' if grip.report.get('posture_policy') else '') +
                                ('카메라: 중앙 브래킷 · 수확 중심보다 위로 유지\n\n' if
                                 (grip.report.get('posture_policy') or {}).get('camera_height_reference') == 'capture_center' else
                                 '카메라: 이동 중에도 수확 영역 위로 유지\n\n' if
                                 (grip.report.get('posture_policy') or {}).get('minimum_camera_above_grip_m') is not None else '') +
                                ('yaw 한 바퀴 회전 금지 · 이동 경로 전체 검사\n\n' if
                                 (grip.report.get('posture_policy') or {}).get('grip_yaw_working_range_deg') is not None else '') +
                                (f'시작 자세 탐색: 기존 {grip.report["start_search"]["baseline_complete_targets"]}곳 → '
                                 f'{sum(s["complete_grip_cycles"]for s in summary.values())}곳 · '
                                 f'시작 리프트 {grip.report["initial_positions"][1]*100:.1f}cm\n\n' if grip.report.get('start_search') else '') +
                                f'완료 경로: 오른쪽 {summary["right"]["complete_grip_cycles"]}곳 / 왼쪽 {summary["left"]["complete_grip_cycles"]}곳\n\n'
                                '초록 점: 진입·닫기·열기·후퇴 통과 / 파랑: 자세만 확인 / 빨강: 해 못 찾음\n\n'
                                f'빨간 틀: {"보관함 옆 2–22cm" if layout in CONVEYOR_LAYOUTS else "가정한 양쪽 영역"} · 높이 {region_z[0][2]:.2f}–{region_z[1][2]:.2f}m\n\n'
                                '기구학·간섭 검사입니다. 토마토 접촉력·파지력 검증은 아닙니다.')
                            last_record = now
                            last = now
                            continue
                        lift_label = '상판·모터·로봇 동시 이동' if active.report['lift_motion_enabled'] else '높이 고정'
                        status.content = (f'**V1 · 기존 배치 · {state}**\n\n'
                            f'{label} · {playback["elapsed_s"]:.1f}초 · {speed.value:g}배속\n\n'
                            f'리프트 {lift:.3f}m · {lift_label}\n\n' +
                            ('빨간 박스: 반대편 작업 영역 · 녹색: 기존 영역 · 주황 점: 목표\n\n'
                             if args.workspace_side == 'left' else '녹색 박스: 작업 영역 · 주황 점: 목표\n\n') +
                            '관절 궤적 미리보기입니다. 동역학은 실행하지 않습니다.')
                        last_record = now
            last = now
    finally:
        server.stop()


if __name__ == '__main__':
    main()
