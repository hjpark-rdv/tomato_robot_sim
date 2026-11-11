"""Explicit ideal harvest experiment: main-stem obstacles and measured volume credit.

This is not physical picking or detachment evidence. Original assets are never
overwritten. The existing GPU runner, planner and state continuation are reused.
"""
import argparse
import copy
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_inputs import sha256
from newton_prepared_assets import verify_manifest_files, link_manifest_assets

MODE = 'main_stem_only_geometric_harvest'
# Capture the roller assembly through its end caps (CAD Z=-4..79mm).
# Retain the original Z=37mm aim centre, width and insertion depth.
LOW = np.array([-.048, 0., -.005])
HIGH = np.array([.048, .158, .079])


def volume_bounds(policy=None):
    """Use a run's declared volume; old immutable assets retain their old box."""
    policy=policy or {}
    low=np.asarray(policy.get('tool_box_min_m',LOW),float)
    high=np.asarray(policy.get('tool_box_max_m',HIGH),float)
    if (low.shape!=(3,) or high.shape!=(3,) or not np.isfinite(low).all() or
            not np.isfinite(high).all() or np.any(high<=low)):
        raise ValueError('Finite ordered 3D harvest volume required')
    return low.copy(),high.copy()


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def collision_variant(original, metadata):
    if original.npair:
        raise ValueError('Explicit contact pairs need exact mapping before disabling plant contacts')
    owned = [name for w in metadata['windows'] for name in w['bodies']]
    if not owned or len(set(owned)) != len(owned):
        raise ValueError('Exact unique plant body ownership required')
    owned = set(owned)
    fruits = {s['name'] for s in metadata['attachments']}
    if not fruits <= owned:
        raise ValueError('Unowned fruit')
    model = copy.copy(original)
    disabled, retained = [], []
    for i in range(model.ngeom):
        body = model.body(int(model.geom_bodyid[i])).name
        if body not in owned:
            continue
        row = dict(geom=model.geom(i).name, body=body, geom_id=i,
                   source_masks=[int(original.geom_contype[i]), int(original.geom_conaffinity[i])])
        if re.fullmatch(r'(?:p\d+_)?Stem_MainStem_\d+', body):
            retained.append(row)
        else:
            model.geom_contype[i] = 0
            model.geom_conaffinity[i] = 0
            disabled.append(row)
    if not retained or not disabled:
        raise ValueError('No main stem or plant geometry mapped')
    for field in ('contype', 'conaffinity'):
        array = getattr(model, 'body_'+field)
        array[:] = 0
        np.bitwise_or.at(array, model.geom_bodyid, getattr(model, 'geom_'+field))
    allowed = {'geom_contype', 'geom_conaffinity', 'body_contype', 'body_conaffinity'}
    unchanged = []
    for name in dir(original):
        left = getattr(original, name)
        if isinstance(left, np.ndarray) and name not in allowed:
            if not np.array_equal(left, getattr(model, name)):
                raise ValueError('Unexpected model change: '+name)
            unchanged.append(name)
    return model, dict(mode=MODE, disabled_geoms=disabled, main_stem_geoms=retained,
        exact_unchanged_arrays=unchanged, changed_arrays=sorted(allowed),
        fruit_collision=False, thin_branch_collision=False, main_stem_collision=True,
        robot_and_structural_collisions_unchanged=True, physical_detachment_enabled=False,
        harvest_rule='Measured fruit center inside measured tool box after reaching goal; all colors',
        tool_box_min_m=LOW.tolist(), tool_box_max_m=HIGH.tolist(),
        training_eligible=False, harvest_success=None)


def prepare_inputs(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    proof = json.loads((source/'inputs.json').read_text())
    verify_manifest_files(source, proof['input_files'])
    meta = json.loads((source/'source_export.json').read_text())
    original = mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    model, policy = collision_variant(original, meta)
    policy['source_model_sha256'] = sha256(source/'input_model.mjb')
    output.mkdir(parents=True, exist_ok=False)
    changed = {'input_model.mjb', 'input_scene.xml', 'source_export.json'}
    link_manifest_assets(source, output, set(proof['input_files'])-changed)
    if not (output/'sampled_plan.json').exists():
        (output/'sampled_plan.json').symlink_to(source/'sampled_plan.json')
    tree = ET.parse(source/'input_scene.xml')
    # Original XML still contains discarded visual geometry. Remove only
    # geometry already absent from the signed binary, then retain zero-mask
    # colliders through the active-window compiler so body/geom identities stay
    # identical. Camera rendering continues to use the original visual model.
    present = {model.geom(i).name for i in range(model.ngeom)}
    present_meshes = {model.mesh(i).name for i in range(model.nmesh)}
    for parent in tree.iter():
        for child in list(parent):
            if child.tag=='geom' and child.get('name') and child.get('name') not in present:
                parent.remove(child)
            elif child.tag=='mesh' and child.get('name') not in present_meshes:
                parent.remove(child)
    tree.find('compiler').set('discardvisual', 'false')
    disable = {r['geom'] for r in policy['disabled_geoms']}
    matched = set()
    for geom in tree.iter('geom'):
        if geom.get('name') in disable:
            geom.set('contype', '0'); geom.set('conaffinity', '0')
            matched.add(geom.get('name'))
    if matched != disable:
        raise ValueError('XML/binary geometry mismatch')
    tree.write(output/'input_scene.xml', encoding='unicode')
    mj.mj_saveModel(model, str(output/'input_model.mjb'))
    policy['model_sha256'] = sha256(output/'input_model.mjb')
    meta['geometric_harvest'] = policy
    write(output/'source_export.json', meta)
    for name in changed:
        proof['input_files'][name] = dict(path=str(output/name), sha256=sha256(output/name))
    proof['geometric_harvest'] = policy
    proof.setdefault('code_sha256', {})[str(Path(__file__).resolve())] = sha256(__file__)
    write(output/'inputs.json', proof)
    write(output/'geometric_harvest.json', policy)
    return policy


def prepare_static_inputs(source, output):
    """Compile the ideal mode once; retain the original masked input as proof."""
    from newton_active_window import prepare
    output=Path(output)
    masked=output.with_name(output.name+'_masked')
    policy=prepare_inputs(source,masked)
    metadata=json.loads((masked/'source_export.json').read_text())
    reduction=prepare(masked,output,metadata['windows'][0]['id'],freeze_all_plants=True)
    policy=dict(policy,all_plants_static=True,reduction=reduction,
                model_sha256=sha256(output/'input_model.mjb'))
    write(output/'geometric_harvest.json',policy)
    return policy


def prepare_assets(source, output, *, static_plants=True):
    source, output = Path(source).resolve(), Path(output).resolve()
    config = json.loads((source/'patrol_assets.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    for name in ('model', 'inventory.json', 'sensor_catalog.json'):
        (output/name).symlink_to(source/name)
    policy = (prepare_static_inputs if static_plants else prepare_inputs)(source/'inputs',output/'inputs')
    config.update(inputs=str(output/'inputs'), harvest_mode=MODE, geometric_harvest=policy,
        collision_scope='Only thick main stems of prepared plants collide; all fruit/thin plant geometry has zero masks. Robot/structure unchanged.',
        source_patrol_assets=str(source))
    if static_plants:
        config['dynamics']='Robot coordinates only; every plant body fixed at verified measured pose, with thick-stem collision retained'
    write(output/'patrol_assets.json', config)
    return policy


def revise_volume_assets(source, output):
    """New signed metadata for the taller box; share every physical asset byte."""
    source,output=Path(source).resolve(strict=True),Path(output).resolve()
    config=json.loads((source/'patrol_assets.json').read_text())
    output.mkdir(parents=True,exist_ok=False)
    revision=dict(source_assets=str(source),tool_box_min_m=LOW.tolist(),tool_box_max_m=HIGH.tolist(),
                  dimensions_m=(HIGH-LOW).tolist(),physical_model_unchanged=True,
                  rationale='Roller end-cap extent Z=-4..79mm; 84mm region centred at original Z=37mm')
    if config.get('start_modes'):
        for mode,path in config['start_modes'].items():revise_volume_assets(path,output/mode)
        config['start_modes']={mode:str(output/mode) for mode in config['start_modes']}
        for path in source.iterdir():
            if path.name not in ('patrol_assets.json',*config['start_modes']):
                (output/path.name).symlink_to(path)
    else:
        if config.get('harvest_mode')!=MODE:raise ValueError('Volume revision requires existing geometric harvest')
        original=Path(config['inputs']).resolve(strict=True)
        proof=json.loads((original/'inputs.json').read_text())
        verify_manifest_files(original,proof['input_files'])
        dest=output/'inputs';dest.mkdir()
        changed={'source_export.json','geometric_harvest.json'}
        link_manifest_assets(original,dest,set(proof['input_files'])-changed)
        for path in original.iterdir():
            if path.name not in changed|{'inputs.json'} and not (dest/path.name).exists():
                (dest/path.name).symlink_to(path)
        meta=json.loads((original/'source_export.json').read_text())
        policy=dict(config['geometric_harvest'],tool_box_min_m=LOW.tolist(),tool_box_max_m=HIGH.tolist(),
                    model_sha256=sha256(dest/'input_model.mjb'),
                    harvest_volume_revision=revision)
        meta['geometric_harvest']=policy
        write(dest/'source_export.json',meta);write(dest/'geometric_harvest.json',policy)
        for name in changed:proof['input_files'][name]=dict(path=str(dest/name),sha256=sha256(dest/name))
        proof['geometric_harvest']=policy;proof['harvest_volume_revision']=revision
        proof.setdefault('code_sha256',{})[str(Path(__file__).resolve())]=sha256(__file__)
        write(dest/'inputs.json',proof)
        verify_manifest_files(dest,proof['input_files'])
        if sha256(dest/'input_model.mjb')!=sha256(original/'input_model.mjb'):
            raise ValueError('Harvest volume revision changed the physical model')
        for path in source.iterdir():
            if path.name not in ('inputs','patrol_assets.json'):(output/path.name).symlink_to(path)
        config.update(inputs=str(dest),geometric_harvest=policy)
    config['harvest_volume_revision']=revision
    write(output/'patrol_assets.json',config)
    return revision


def geometric_plan(plan):
    """Keep existing preparation/insertion/return commands; replace belt work by hold."""
    result = copy.deepcopy(plan)
    phases = result['phases']
    a = next(i for i,p in enumerate(phases) if p['name']=='approach')
    b = next(i for i,p in enumerate(phases) if p['name']=='withdraw')
    if b <= a:
        raise ValueError('Expected existing approach followed by withdraw')
    hold = copy.deepcopy(phases[a])
    hold.update(name='geometric_capture_hold', duration_s=.3,
        waypoint_q=[phases[a]['waypoint_q'][-1]]*2)
    for key in ('rail_waypoint_positions_m', 'rail_native_command_positions_m'):
        if key in hold:
            hold[key] = [hold[key][-1]]*2
    # Drop any cached per-waypoint data specific to the approach. The adapter
    # consumes waypoint_q and the explicit rail positions.
    result['phases'] = phases[:a+1]+[hold]+phases[b:]
    for phase in result['phases']:
        phase['belt_local_y_m_s'] = [0., 0.]
    result['total_duration_s'] = result.get('settle_seconds',0.)+sum(p['duration_s'] for p in result['phases'])
    result.update(harvest_mode=MODE, physical_detachment_enabled=False,
        training_eligible=False, harvest_success=None)
    return result


def capture_receipt(model, initial, plan, states_path, fruit_names, already=(), *, volume=None):
    """Credit at a recorded reached pose, never from a command alone or swept path."""
    from plan_adapter import sample_plan
    low,high=volume_bounds(volume)
    with np.load(states_path, allow_pickle=False) as saved:
        phase = saved['phase']; ids = np.flatnonzero(phase=='geometric_capture_hold')
        if not len(ids):
            return dict(mode=MODE, reached=False, reason='capture_pose_not_executed', harvested=[])
        index = int(ids[-1]); q = saved['qpos'][index].copy()
        timestamp = float(saved['time'][index])
    t = plan.get('settle_seconds',0.)
    for part in plan['phases']:
        t += part['duration_s']
        if part['name']=='approach': break
    command = sample_plan(plan, t)['command']
    actual = mj.MjData(model); actual.qpos[:] = q; mj.mj_kinematics(model,actual)
    goal = mj.MjData(model); goal.qpos[:] = q
    goal.qpos[initial['robot_qpos_addresses']] = command; mj.mj_kinematics(model,goal)
    tool = model.body('Tool_fixed').id
    r = actual.xmat[tool].reshape(3,3); rg = goal.xmat[tool].reshape(3,3)
    pe = float(np.linalg.norm(actual.xpos[tool]-goal.xpos[tool]))
    angle = float(Rotation.from_matrix(rg.T@r).magnitude())
    reached = pe <= .005 and angle <= np.deg2rad(3.)
    rows = []
    if reached:
        for name in sorted(set(fruit_names)-set(already)):
            point = (actual.xpos[model.body(name).id]-actual.xpos[tool])@r
            if np.all(point >= low) and np.all(point <= high):
                rows.append(dict(name=name, center_tool_m=point.tolist()))
    return dict(mode=MODE, reached=bool(reached), timestamp_s=timestamp, state_index=index,
        states_sha256=sha256(states_path), translation_error_m=pe, rotation_error_rad=angle,
        reach_limits=dict(translation_m=.005, rotation_deg=3.),
        tool_box_min_m=low.tolist(), tool_box_max_m=high.tolist(), harvested=rows,
        physical_detachment_verified=False, all_colors_inside_counted=True,
        scope='All prepared plants, fruit centers in actual tool volume at reached pose',
        training_eligible=False, harvest_success=None)


def sensor_poses(poses, harvested):
    """Hide logically collected fruit before GPU ray tracing, including occlusion."""
    result = dict(poses)
    for name in harvested:
        if name not in result: raise ValueError('Unknown logically harvested fruit: '+name)
        result[name] = np.asarray(result[name]).copy()
        result[name][:3] = [0., 0., -1000.]
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path); parser.add_argument('output', type=Path)
    parser.add_argument('--inputs-only',action='store_true')
    parser.add_argument('--volume-only',action='store_true',help='Derive a taller capture-volume profile without rebuilding physics or farm')
    a = parser.parse_args()
    if a.volume_only:
        if a.inputs_only:parser.error('--volume-only and --inputs-only are exclusive')
        print(json.dumps(revise_volume_assets(a.source,a.output),indent=2))
    else:
        policy = (prepare_static_inputs if a.inputs_only else prepare_assets)(a.source, a.output)
        print(json.dumps(dict(output=str(a.output), disabled=len(policy['disabled_geoms']),
            main_stem=len(policy['main_stem_geoms']), mode=MODE)))
