"""Derive a V6 camera-patrol start without changing the physical model."""
from pathlib import Path
import copy

import mujoco as mj
import numpy as np

from side_farm import read,write,initial_for_plan
from newton_inputs import sha256
from newton_prepared_assets import verify_manifest_files


def prepare(source,output,profile):
    source,output,profile=Path(source).resolve(),Path(output).resolve(),Path(profile).resolve()
    config=read(source/'patrol_assets.json');home=read(profile);common=source/'inputs';proof=read(common/'inputs.json')
    verify_manifest_files(common,proof['input_files'])
    if sha256(common/'input_model.mjb')!=home['model_sha256']:raise ValueError('Camera ready calibration belongs to another physical model')
    np.testing.assert_allclose(config['mounted_rgbd']['body_from_camera_optical'],home['body_from_camera_optical'],rtol=0,atol=1e-12)
    model=mj.MjModel.from_binary_path(str(common/'input_model.mjb'))
    # The existing GPU controller already adds qfrc_bias/kp. At the absolute
    # upper lift target that command is clipped by the original ctrlrange,
    # causing ~7.4mm sag. Reserve command headroom instead of weakening the
    # 5mm reach check or modifying the motor, gravity or physical joint limits.
    home=copy.deepcopy(home)
    lift=model.joint('farmily_lift_height_joint');address=int(lift.qposadr[0])
    actuator=next(i for i in range(model.nu)if model.actuator_trnid[i,0]==lift.id)
    mass=float(model.body_subtreemass[int(lift.bodyid[0])]);kp=float(model.actuator_gainprm[actuator,0])
    gravity_offset=mass*float(np.linalg.norm(model.opt.gravity))/kp
    working_max=float(np.floor((min(model.jnt_range[lift.id,1],model.actuator_ctrlrange[actuator,1])-gravity_offset-.0025)*1000.)/1000.)
    home['initial_command'][address]=min(home['initial_command'][address],working_max)
    home['lift_control_headroom']=dict(subtree_mass_kg=mass,kp=kp,gravity_feedforward_offset_m=gravity_offset,
        working_max_m=working_max,physical_ctrlrange_m=model.actuator_ctrlrange[actuator].tolist(),
        physical_model_unchanged=True,reach_tolerance_unchanged=True)
    output.mkdir(parents=True,exist_ok=False);inputs=output/'inputs';inputs.mkdir()
    generated={'inputs.json','initial_state.npz','canonical_full_state.npz'}
    for path in common.iterdir():
        if path.name not in generated:(inputs/path.name).symlink_to(path)
    initial=dict(np.load(common/'initial_state.npz',allow_pickle=False))
    fresh=initial_for_plan(model,initial,home)
    if abs(float(fresh['qpos'][0]))>1e-12:raise ValueError('Camera patrol must start at rail entrance')
    np.savez_compressed(inputs/'initial_state.npz',**fresh)
    # The archived full-plant canonical reference has extra plant DOFs; retain
    # them, updating only its robot fields and the corresponding frozen poses.
    canonical=dict(np.load(common/'canonical_full_state.npz',allow_pickle=False))
    canonical['qpos'][canonical['robot_qpos_addresses']]=home['initial_command']
    canonical['ctrl'][canonical['actuator_ids']]=home['initial_command']
    canonical['qvel'][canonical['robot_dof_addresses']]=0.
    canonical_data=mj.MjData(model);canonical_data.qpos[:]=fresh['qpos'];mj.mj_kinematics(model,canonical_data)
    ids=[model.body(str(n)).id for n in canonical['body_names']]
    canonical['body_positions']=canonical_data.xpos[ids];canonical['body_quaternions_wxyz']=canonical_data.xquat[ids]
    np.savez_compressed(inputs/'canonical_full_state.npz',**canonical)
    for name,row in proof['input_files'].items():
        row['path']=str(inputs/name)
        if name in generated:row['sha256']=sha256(inputs/name)
    proof['camera_ready_calibration']=dict(profile=str(profile),sha256=sha256(profile),target_coordinates_used=False)
    write(inputs/'inputs.json',proof)
    data=mj.MjData(model);data.qpos[:]=fresh['qpos'];mj.mj_kinematics(model,data)
    tool=model.body('Tool_fixed').id;rotation=data.xmat[tool].reshape(3,3)
    pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=data.xpos[tool]
    optical=pose@np.array(home['body_from_camera_optical'])
    centre=data.xpos[tool]+rotation@np.array([0.,.079,.037])
    for path in source.iterdir():
        if path.name not in ('inputs','patrol_assets.json','scene_stream'):(output/path.name).symlink_to(path)
    config=copy.deepcopy(config);config['inputs']=str(inputs)
    config['mounted_rgbd'].update(reference_ready_q=home['initial_command'][1:8],
        reference_world_from_camera_optical=optical.tolist(),mount_calibration_revision='V6 recessed Tool_fixed camera',
        mount_description=config['side_installation']['preview_metadata']['gripper_camera_mount']['mount_description'])
    config['side_camera_patrol']=dict(ready_command=home['initial_command'],profile_sha256=sha256(profile),
        source_assets=str(source),physical_model_sha256=home['model_sha256'],
        lift_control_headroom=home['lift_control_headroom'],
        camera_foreaft_m=float(home['service_centre_y_from_rail_origin_m']-optical[1,3]),
        camera_to_grasp_height_m=float(centre[2]-optical[2,3]),
        target_source='Fresh occlusion-aware RGB/optical-depth/SIM-GT observations; no authored fruit targets')
    write(output/'patrol_assets.json',config);write(output/'camera_ready.json',home)
    stream=copy.deepcopy(read(source/'scene_stream/manifest.json'));dest=output/'scene_stream';dest.mkdir()
    for section in stream['chunks']:
        for key in ('geometry','catalog'):
            path=dest/section[key]
            if not path.exists():path.symlink_to(source/'scene_stream'/section[key])
    stream.update(assets=str(output),mount=config['mounted_rgbd'])
    stream['source_signatures']={}
    for name in stream['source_hashes']:
        stream['source_hashes'][name]=sha256(output/name)
        stat=(output/name).stat();stream['source_signatures'][name]=[stat.st_size,stat.st_mtime_ns,stat.st_ino]
    write(dest/'manifest.json',stream)
    verify_manifest_files(inputs,proof['input_files'])
    return config
