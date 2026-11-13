"""Prepare the checked V6 installation in the existing static tomato farm.

Farm/fruit placement and the existing geometric harvest approximation are
retained. The new model is independent of the original patrol assets.
"""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_inputs import sha256, command_timeline
from newton_prepared_assets import link_manifest_assets, verify_manifest_files
from preview_robot_mount import body_poses
from preview_workspace_motion import collision_model
from run_newton_camera_patrol import scan_plan


def read(path): return json.loads(Path(path).read_text())
def write(path, value): Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
def vector(value): return ' '.join(format(float(x), '.17g') for x in value)


def initial_for_plan(model, initial, plan):
    """Start a fresh run at the planned survey pose with matching actuator holds."""
    state={k:np.asarray(v).copy()for k,v in initial.items()}
    command=np.asarray(plan['initial_command'],float)
    if command.shape!=state['robot_qpos_addresses'].shape or not np.isfinite(command).all():
        raise ValueError('Plan start must match the robot coordinates')
    state['qpos'][state['robot_qpos_addresses']]=command
    state['ctrl'][state['actuator_ids']]=command
    state['qvel'][state['robot_dof_addresses']]=0.
    if 'qacc_warmstart'in state:state['qacc_warmstart'][:]=0.
    data=mj.MjData(model);data.qpos[:]=state['qpos'];mj.mj_kinematics(model,data)
    ids=[model.body(str(n)).id for n in state['body_names']]
    state['body_positions']=data.xpos[ids].copy();state['body_quaternions_wxyz']=data.xquat[ids].copy()
    return state


class FarmObstacles:
    """Already compiled boxes follow their real chassis/carriage parent bodies."""
    def __init__(self, model, boxes):
        self.rows = [dict(gid=model.geom(f'v6_box_{i:03d}').id, box=box) for i, box in enumerate(boxes)]
        self.aliases = {model.geom(r['gid']).name:r['box']['name'] for r in self.rows}
    def update(self, lift): pass


def prepare(preview, output):
    preview, output = Path(preview).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    meta = read(preview/'preview.json')
    assets = Path(meta['assets']); common = assets/'inputs'
    proof = read(common/'inputs.json'); verify_manifest_files(common, proof['input_files'])
    report = read(meta['grip_motion_file'])
    if report['layout_key'] != 'side': raise ValueError('Checked V6 installation required')
    source = mj.MjModel.from_binary_path(str(common/'input_model.mjb'))
    initial = dict(np.load(common/'initial_state.npz', allow_pickle=False))
    q = np.array(report['initial_positions']); initial['qpos'][:] = q
    displayed, _ = body_poses(source, initial, meta, q[1], True, q[2:8], 0., 'side', jaw_positions=q[8:])
    patched, installation = collision_model(source, q, meta, displayed, output, layout_key='side')
    tree = ET.parse(common/'input_scene.xml'); root = tree.getroot()
    root.find('compiler').set('discardvisual', 'false')
    geom_names = {source.geom(i).name for i in range(source.ngeom)}
    mesh_names = {source.mesh(i).name for i in range(source.nmesh)}
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'geom' and child.get('name') not in geom_names: parent.remove(child)
            elif child.tag == 'mesh' and child.get('name') not in mesh_names: parent.remove(child)
    bodies = {b.get('name'):b for b in root.iter('body')}
    robot_root = source.body('Robot_link1').id
    bodies['Robot_link1'].set('pos', vector(patched.body_pos[robot_root]))
    bodies['Robot_link1'].set('quat', vector(patched.body_quat[robot_root]))
    moved = set(installation['moved_native_colliders'])
    removed = set(installation['removed_native_colliders'])
    for geom in root.iter('geom'):
        name = geom.get('name'); gid = source.geom(name).id
        # Preserve every authored farm mask. Only the old plate/combined hulls
        # are replaced by the same conservative V6 component boxes.
        active=bool(source.geom_contype[gid] or source.geom_conaffinity[gid])
        geom.set('contype', str(0 if name in removed else int(source.geom_contype[gid]) | (4 if active and source.body(int(source.geom_bodyid[gid])).name.startswith(('Robot_link','Tool_')) else 0)))
        geom.set('conaffinity', str(0 if name in removed else int(source.geom_conaffinity[gid])))
        for field in ('friction','solref','solimp'):
            geom.set(field, vector(getattr(source,'geom_'+field)[gid]))
        for field in ('gap','margin','solmix'):
            geom.set(field, format(float(getattr(source,'geom_'+field)[gid]),'.17g'))
        for field in ('condim','priority'):
            geom.set(field,str(int(getattr(source,'geom_'+field)[gid])))
        if name in moved:
            geom.set('pos', vector(patched.geom_pos[gid])); geom.set('quat', vector(patched.geom_quat[gid]))
    data=mj.MjData(patched); data.qpos[:]=q; mj.mj_kinematics(patched,data)
    boxes=report['obstacle_boxes']
    for i, box in enumerate(boxes):
        parent = 'Robot_farmily_lift_platform' if box.get('follows_lift') else 'Robot_world'
        bid=source.body(parent).id; r=data.xmat[bid].reshape(3,3)
        pos=r.T@(np.array(box['position'])-data.xpos[bid])
        quat=Rotation.from_matrix(r.T).as_quat()[[3,0,1,2]]
        ET.SubElement(bodies[parent], 'geom', name=f'v6_box_{i:03d}', type='box',
            pos=vector(pos), quat=vector(quat), size=vector(np.array(box['dimensions'])/2.),
            contype='0', conaffinity='4', density='0', gap='0', rgba='0 0 0 0')
    inputs=output/'inputs'; inputs.mkdir()
    tree.write(inputs/'input_scene.xml', encoding='unicode')
    print('Compiling V6 farm with original farm colliders and 162 moving chassis/carriage boxes', flush=True)
    model=mj.MjModel.from_xml_path(str(inputs/'input_scene.xml'))
    if [model.body(i).name for i in range(model.nbody)] != [source.body(i).name for i in range(source.nbody)]:
        raise ValueError('Farm body identities changed')
    if model.nq != source.nq or model.nv != source.nv: raise ValueError('Robot coordinates changed')
    # Retain explicit compiled properties of the unchanged robot/farm bodies.
    mj.mj_saveModel(model, str(inputs/'input_model.mjb'))
    d=mj.MjData(model); d.qpos[:]=q; mj.mj_forward(model,d)
    for n in ('Robot_link1','Tool_fixed'):
        bid=model.body(n).id
        np.testing.assert_allclose(d.xpos[bid],displayed[bid,:3,3],atol=1e-8,rtol=0)
        np.testing.assert_allclose(d.xmat[bid].reshape(3,3),displayed[bid,:3,:3],atol=1e-8,rtol=0)
    generated={'input_scene.xml','input_model.mjb','initial_state.npz','canonical_full_state.npz',
               'sampled_plan.json','commands.npz','source_export.json'}
    link_manifest_assets(common, inputs, set(proof['input_files'])-generated)
    for name in ('initial_state.npz','canonical_full_state.npz'):
        state=dict(np.load(common/name,allow_pickle=False))
        state['qpos'][state['robot_qpos_addresses']]=q
        state['ctrl'][state['actuator_ids']]=q
        ids=[model.body(str(n)).id for n in state['body_names']]
        state['body_positions']=d.xpos[ids]; state['body_quaternions_wxyz']=d.xquat[ids]
        np.savez_compressed(inputs/name,**state)
    plan=scan_plan(q); write(inputs/'sampled_plan.json',plan)
    physics=read(common/'source_export.json')
    physics.update(prepared_source_plan=str(inputs/'sampled_plan.json'),
        side_installation=dict(preview=str(preview),preview_metadata=meta,obstacle_boxes=boxes,
            original_model_sha256=sha256(common/'input_model.mjb'),plant_geometry_changed=False,
            geometric_harvest_only=True,physical_fruit_detachment=False))
    write(inputs/'source_export.json',physics)
    timeline,description,_=command_timeline(physics,q,220,free_hz=60)
    np.savez_compressed(inputs/'commands.npz',**timeline); write(inputs/'timeline.json',description)
    proof['timeline']=description
    for name in (*proof['input_files'],'sampled_plan.json'):
        proof['input_files'][name]=dict(path=str(inputs/name),sha256=sha256(inputs/name))
    write(inputs/'inputs.json',proof)
    for name in ('inventory.json','sensor_catalog.json','model','scene_stream'):
        (output/name).symlink_to(assets/name)
    config=read(assets/'patrol_assets.json')
    config['inputs']=str(inputs); config['source_patrol_assets']=str(assets)
    mount=dict(config['mounted_rgbd']); spec=meta['gripper_camera_mount']
    mount['body_from_camera_optical']=spec['body_from_camera_optical']
    mount['body_path']=next(p for p,n in proof['source_body_to_native_name'].items() if n=='Tool_fixed')
    config.update(mounted_rgbd=mount,side_installation=physics['side_installation'])
    write(output/'patrol_assets.json',config)
    audit=dict(source_assets=str(assets),preview=str(preview),model=str(inputs/'input_model.mjb'),
        model_sha256=sha256(inputs/'input_model.mjb'),original_source_model_sha256=sha256(common/'input_model.mjb'),
        nq=model.nq,nv=model.nv,nbody=model.nbody,ngeom=model.ngeom,added_v6_boxes=len(boxes),
        farm_masks_preserved=True,physics_steps=0,geometric_harvest_only=True)
    write(output/'side_farm_preparation.json',audit)
    print(json.dumps(audit,indent=2),flush=True)
    return model,meta


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('preview',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();prepare(a.preview,a.output)
