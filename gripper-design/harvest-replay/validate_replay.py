#!/usr/bin/env python3
"""Independently validate a web replay against its original saved execution.

Run with nvidia-sim/.asset-venv/bin/python. This reads saved arrays and glTF
buffers; it does not start a simulator, write source assets, or import a farm.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import sys
from urllib.parse import unquote, urlsplit

import numpy as np
from sensor_replay import validate_camera
from partial_recording import KIND, partial_recording, published_partial_source


class ReplayMismatch(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise ReplayMismatch(message)


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def local_url(base, url):
    parsed = urlsplit(url)
    require(not parsed.scheme and not parsed.netloc, 'Validation requires local bundle URLs: '+url)
    return (Path(base).parent / unquote(parsed.path)).resolve()


def read_binary(path):
    data = Path(path).read_bytes()
    return gzip.decompress(data) if data[:2] == b'\x1f\x8b' else data


def read_glb(path):
    data = read_binary(path)
    require(len(data) >= 20, 'Truncated GLB')
    magic, version, length = struct.unpack_from('<4sII', data)
    require(magic == b'glTF' and version == 2 and length == len(data), 'Invalid GLB header')
    offset = 12
    document = None
    binary = b''
    while offset < length:
        size, kind = struct.unpack_from('<II', data, offset)
        offset += 8
        require(offset + size <= length, 'Truncated GLB chunk')
        chunk = data[offset:offset+size]
        offset += size
        if kind == 0x4e4f534a:
            require(document is None, 'Duplicate glTF JSON chunk')
            document = json.loads(chunk)
        elif kind == 0x004e4942:
            binary = chunk
    require(document is not None, 'Missing glTF JSON')
    return document, binary


_COMPONENT = {5120:np.dtype('i1'), 5121:np.dtype('u1'), 5122:np.dtype('<i2'),
              5123:np.dtype('<u2'), 5125:np.dtype('<u4'), 5126:np.dtype('<f4')}
_WIDTH = {'SCALAR':1, 'VEC2':2, 'VEC3':3, 'VEC4':4, 'MAT2':4, 'MAT3':9, 'MAT4':16}


def accessor(document, binary, index):
    item = document['accessors'][index]
    require('sparse' not in item, 'Sparse accessor needs explicit validator support')
    view = document['bufferViews'][item['bufferView']]
    require(view.get('buffer', 0) == 0, 'Only embedded glTF buffers are supported')
    dtype = _COMPONENT[item['componentType']]
    width = _WIDTH[item['type']]
    offset = int(view.get('byteOffset', 0)) + int(item.get('byteOffset', 0))
    stride = int(view.get('byteStride', width * dtype.itemsize))
    count = int(item['count'])
    require(count >= 0 and stride >= width*dtype.itemsize, 'Invalid accessor dimensions')
    end = offset + (max(0,count-1)*stride + width*dtype.itemsize if count else 0)
    require(end <= len(binary), 'Accessor exceeds glTF BIN buffer')
    result = np.ndarray((count,width), dtype=dtype, buffer=binary,
                        offset=offset, strides=(stride,dtype.itemsize))
    if item.get('normalized') and dtype.kind in 'iu':
        limit = np.iinfo(dtype)
        result = result.astype(np.float64) / limit.max
        if dtype.kind == 'i':
            result = np.maximum(result, -1)
    return result


def field_path(record, *names):
    for name in names:
        value = record.get(name)
        if value:
            return Path(value)
    raise ReplayMismatch('Missing source path: '+', '.join(names))


def checked_hash(path, expected, label):
    require(Path(path).is_file(), 'Missing '+label+': '+str(path))
    actual = sha256(path)
    require(actual == expected, label+' SHA mismatch')
    return actual


def decode_array(replay_path, descriptor, dtype, shape):
    path = local_url(replay_path, descriptor['url'])
    raw = read_binary(path)
    expected = int(np.prod(shape, dtype=np.int64))*np.dtype(dtype).itemsize
    require(len(raw) == expected, 'Binary length mismatch: '+str(path))
    if descriptor.get('sha256'):
        # File hashes identify the served file, including compression.
        checked_hash(path, descriptor['sha256'], path.name)
    result = np.frombuffer(raw, dtype=dtype).reshape(shape)
    require(np.isfinite(result).all(), 'Nonfinite exported array: '+str(path))
    return result


def same_array(actual, expected, label):
    require(actual.shape == expected.shape, label+' shape mismatch')
    require(np.array_equal(actual, expected), label+' differs from recorded source values')


C = np.array([[1.,0.,0.], [0.,0.,1.], [0.,-1.,0.]])


def quaternion_matrices(quaternions):
    q = np.asarray(quaternions, dtype=np.float64)
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    require(np.all(norms > 0), 'Zero recorded quaternion')
    x,y,z,w = np.moveaxis(q/norms, -1, 0)
    result = np.empty(q.shape[:-1]+(3,3), dtype=np.float64)
    result[...,0,0] = 1-2*(y*y+z*z)
    result[...,0,1] = 2*(x*y-z*w)
    result[...,0,2] = 2*(x*z+y*w)
    result[...,1,0] = 2*(x*y+z*w)
    result[...,1,1] = 1-2*(x*x+z*z)
    result[...,1,2] = 2*(y*z-x*w)
    result[...,2,0] = 2*(x*z-y*w)
    result[...,2,1] = 2*(y*z+x*w)
    result[...,2,2] = 1-2*(x*x+y*y)
    return result


def event_time(event):
    return float(event.get('time_s', event.get('t', event.get('release_effective_time_s', -1))))


def validate_skins(document, binary, info, assembly, manifest, paths, poses):
    source_skins = {}
    for plant, filename in [(manifest['plant'], 'skin.npz')]+[
            (row['plant'],row['skin_file']) for row in manifest.get('elastic_neighbors',[])]:
        with np.load(assembly/filename, allow_pickle=False) as arrays:
            for index, path in enumerate(plant['skin_paths']):
                source_skins[path] = {key:arrays[f'{index}_{key}'] for key in
                                      ('vertices','ids','weights','body_paths','rest')}
    checks = info['skin_checks']
    require(len(checks) == len(source_skins), 'Missing source skin meshes')
    require({row['path'] for row in checks} == set(source_skins), 'Skin paths differ from source manifest')
    require(info['skinned_mesh_count'] == len(checks), 'Skin count metadata mismatch')
    body_nodes = {row['body_path']:row['node_index'] for row in info['body_nodes']}
    body_indices = {path:index for index,path in enumerate(paths)}
    origin = np.asarray(info['origin_world_m'], dtype=float)
    node_by_name = {node['name']:node for node in document['nodes'] if 'name' in node}
    maximum_error = 0.
    tested_vertices = 0
    total_vertices = 0
    for row in checks:
        original = source_skins[row['path']]
        total_vertices += len(original['vertices'])
        node = node_by_name[row['node']]
        require(node.get('extras',{}).get('source_path') == row['path'], 'Skin node source path mismatch')
        require(node['mesh'] == row['mesh_index'] and node['skin'] == row['skin_index'], 'Skin node/mesh binding mismatch')
        gltf_skin = document['skins'][node['skin']]
        bones = original['body_paths'].tolist()
        require(gltf_skin['joints'] == [body_nodes[path] for path in bones], 'Skin bone order/path mismatch')
        inverse = accessor(document,binary,gltf_skin['inverseBindMatrices']).reshape(-1,4,4).transpose(0,2,1)
        expected_inverse = np.repeat(np.eye(4)[None], len(bones), axis=0)
        expected_inverse[:,:3,3] = -(original['rest']-origin)@C.T
        same_array(inverse, expected_inverse.astype('<f4'), 'Inverse bind matrices')
        require(np.max(np.abs(original['weights'].sum(axis=1)-1)) <= 1e-12, 'Source skin weights are not normalized')
        for primitive in document['meshes'][node['mesh']]['primitives']:
            attr = primitive['attributes']
            require(all(key in attr for key in ('POSITION','JOINTS_0','WEIGHTS_0','_SOURCE_VERTEX')), 'Incomplete source skin attributes')
            vertex_ids = accessor(document,binary,attr['_SOURCE_VERTEX']).reshape(-1).astype(np.int64)
            require(np.all((vertex_ids>=0)&(vertex_ids<len(original['vertices']))), 'Skin source vertex out of range')
            points = accessor(document,binary,attr['POSITION'])
            joints = accessor(document,binary,attr['JOINTS_0'])
            weights = accessor(document,binary,attr['WEIGHTS_0'])
            same_array(points, ((original['vertices'][vertex_ids]-origin)@C.T).astype('<f4'), 'Skin rest positions')
            expected_joints = np.zeros((len(vertex_ids),4), dtype=np.uint16)
            expected_weights = np.zeros((len(vertex_ids),4), dtype=np.float32)
            influences = original['ids'].shape[1]
            expected_joints[:,:influences] = original['ids'][vertex_ids]
            expected_weights[:,:influences] = original['weights'][vertex_ids]
            same_array(joints,expected_joints,'Skin influence IDs')
            same_array(weights,expected_weights,'Skin influence weights')
            pick = np.unique(np.linspace(0,len(vertex_ids)-1,min(32,len(vertex_ids))).astype(int))
            if not len(pick):
                continue
            selected = vertex_ids[pick]
            joint_indices = joints[pick].astype(int)
            bone_ids = np.array([body_indices[path] for path in bones])
            for tick in sorted(set((0,len(poses)//2,len(poses)-1))):
                state = poses[tick,bone_ids].astype(float)
                rotations = quaternion_matrices(state[:,3:])
                ids = original['ids'][selected]
                expected = ((np.einsum('nkij,nkj->nki',rotations[ids],
                    original['vertices'][selected,None]-original['rest'][ids])+
                    state[:,:3][ids])*original['weights'][selected,:,None]).sum(axis=1)
                expected = (expected-origin)@C.T
                bone_rotation = np.einsum('ij,njk,lk->nil',C,rotations,C)
                bone_position = (state[:,:3]-origin)@C.T
                local = points[pick,None,:].astype(float)+inverse[:,:3,3][joint_indices]
                got = ((np.einsum('nkij,nkj->nki',bone_rotation[joint_indices],local)+
                        bone_position[joint_indices])*weights[pick,:,None]).sum(axis=1)
                error = float(np.max(np.abs(got-expected)))
                # Bound arithmetic/representation error from actual float32
                # coordinates, not a physical or geometric success tolerance.
                scale = 1+max(float(np.max(np.abs(original['vertices'][selected]))),
                              float(np.max(np.abs(original['rest']))),float(np.max(np.abs(state[:,:3]))))
                bound = 64*np.finfo(np.float32).eps*scale
                require(error <= bound, 'glTF skinning differs from source measured-bone formula')
                maximum_error = max(maximum_error,error)
                tested_vertices += len(pick)
    return dict(meshes=len(checks),source_vertices=total_vertices,
                sampled_vertex_frames=tested_vertices,maximum_sampled_position_error_m=maximum_error,
                influence_ids_exact=True,influence_weights_float32_exact=True,
                source_vertex_mapping_exact=True,inverse_bind_float32_exact=True)


def validate(replay_path, allow_partial_diagnostic=False):
    replay_path = Path(replay_path).resolve()
    replay = read_json(replay_path)
    require(replay.get('schema') == 'farmily-harvest-replay-v1','Unknown replay schema')
    source = replay['source']
    assembly = field_path(source,'assembly')
    trial = field_path(source,'trial')
    partial = None
    if source.get('record_kind') == KIND:
        require(allow_partial_diagnostic is True,'Partial replay requires explicit diagnostic validation opt-in')
        partial = partial_recording(assembly,trial)
        expected_source = published_partial_source(partial,trial)
        require(all(source.get(k) == v for k,v in expected_source.items()),
                'Published partial source identity/diagnostic flags differ')
        require(replay.get('result_summary') == partial['result'],
                'Partial diagnostic cannot acquire normal completion or harvest credit')
        marker = replay.get('partial_diagnostic',{})
        require(marker.get('schema') == 'incomplete-native-replay-v1' and
                marker.get('complete') is False and marker.get('checkpoint_eligible') is False and
                marker.get('harvest_credit') == 0 and marker.get('source_descriptor') == partial['descriptor'] and
                marker.get('runtime_error') == partial['runtime_error'],
                'Partial source descriptor/error or incomplete marker mismatch')
        require(not replay.get('camera'),'Partial unfinalized camera must not be substituted')
    summary = {} if partial else read_json(trial/'summary.json')
    runtime = read_json(trial/'runtime_settings.json')
    manifest = read_json(assembly/'motion_manifest.json')
    native = read_json(trial/'native_metadata.json')
    states_path = partial['states_path'] if partial else trial/'states.npz'
    source_scene = assembly/('composed.usdc' if (assembly/'composed.usdc').is_file() else 'scene.usda')
    source_hash = checked_hash(source_scene,source['scene_sha256'],'Actual source scene')
    states_hash = checked_hash(states_path,source['states_sha256'],'Original recorded states')
    for name,path in [('summary',trial/'summary.json'),('runtime',trial/'runtime_settings.json'),
                      ('manifest',assembly/'motion_manifest.json')]:
        if source.get(name+'_sha256'):
            checked_hash(path,source[name+'_sha256'],name)
    require(runtime.get('native_scene_sha256') == source_hash,
            'Runtime physical scene SHA differs from actual replay scene')
    wrapper_hash = sha256(assembly/'scene.usda')
    recorded_wrapper = partial['native']['model_sha256'] if partial else summary['model_sha256']
    require(recorded_wrapper == wrapper_hash,'Source wrapper differs from recorded execution')
    require(runtime.get('source_scene_sha256',wrapper_hash) == wrapper_hash,'Runtime wrapper SHA mismatch')
    engine = replay['engine']
    if isinstance(engine,dict) and 'recorded_result' in engine:
        require(engine['recorded_result'] == summary,'Published result differs from source summary')
    result = replay.get('result_summary',{})
    for key in (() if partial else ('completed','stop_reason','harvest_success','physics_steps','physics_wall_s','simulated_s','training_eligible')):
        if key in result:
            require(result[key] == summary.get(key),'Published result differs from source summary: '+key)
    require(source.get('all_frames_exported') is True,'Replay omits source frames')
    require(source.get('physics_rerun') is False and replay.get('source_geometry_modified') is False,
            'Replay must preserve recorded execution and source geometry')
    info_path = local_url(replay_path,replay['scene_info_url'])
    info = read_json(info_path)
    require(info['source_scene_sha256'] == source_hash,'Scene metadata provenance mismatch')
    geometry_reuse=replay.get('geometry_reuse')
    reference_poses=None
    if geometry_reuse is not None:
        require(not partial,'Partial traces cannot reuse an actual complete scene')
        require(info.get('geometry_reuse')==geometry_reuse,'Scene/replay reuse evidence differs')
        from scene_reuse import verify_reference_proof
        with np.load(states_path,allow_pickle=False) as current:
            current_paths=current['body_paths'].tolist()
        reference_info,reference_glb,reference_poses=verify_reference_proof(geometry_reuse,
            scene_sha256=source_hash,manifest_sha256=sha256(assembly/'motion_manifest.json'),
            body_paths=current_paths)
        reference_validation=validate(Path(geometry_reuse['reference_replay']))
        require(reference_validation['passed'] is True,'Cached original scene did not independently validate')
        for field in ('origin_world_m','body_nodes','source_inventory','skin_checks','groups',
                      'skinned_mesh_count','unique_glTF_meshes','static_instanced_mesh_count',
                      'visible_source_meshes','source_triangles_with_instances','transmission_nodes'):
            require(info[field]==reference_info[field],'Cached immutable scene metadata differs: '+field)
    else:
        require(info.get('geometry_reuse') is None,'Unexpected cached scene metadata')
    glb_path = local_url(replay_path,replay['scene_url'])
    glb_hash = hashlib.sha256(read_binary(glb_path)).hexdigest()
    require(glb_hash == info['sha256'], 'Web scene GLB SHA mismatch')
    if replay.get('scene_gzip_url'):
        compressed = local_url(replay_path,replay['scene_gzip_url'])
        checked_hash(compressed,info['compressed_sha256'],'Web compressed scene')
        require(read_binary(compressed) == read_binary(glb_path),'Compressed scene does not contain exact GLB')
    document,binary = read_glb(glb_path)
    with np.load(states_path,allow_pickle=False) as original:
        times = original['time']
        poses = original['poses']
        paths = original['body_paths'].tolist()
        require(np.isfinite(poses).all() and np.isfinite(times).all(),'Nonfinite original recording')
        require(np.all(np.diff(times)>0),'Source timestamps not increasing')
        require(replay['motion']['shape'] == list(poses.shape),'Published motion shape differs from source')
        motion = decode_array(replay_path,replay['motion'],'<f4',poses.shape)
        published_time = decode_array(replay_path,replay['time'],'<f8',times.shape)
        same_array(motion,poses.astype('<f4'),'All recorded poses')
        same_array(published_time,times.astype('<f8'),'All exact timestamps')
        require(replay['phase'] == original['phase'].tolist(),'Published phases differ from recording')
        require(replay['frames'] == len(times) and replay['body_count'] == len(paths),'Frame/body count mismatch')
        require(replay['duration_s'] == float(times[-1]-times[0]),'Published duration mismatch')
        require(replay['time']['start_s'] == float(times[0]) and replay['time']['end_s'] == float(times[-1]),'Published time range mismatch')
        gap = .044+original['robot_q'][:,native['robot_dofs'].index('jaw_R')]-original['robot_q'][:,native['robot_dofs'].index('jaw_L')]
        same_array(np.asarray(replay['jaw_gap_m']),gap.astype(float),'Measured gripper gap')
        bindings = info['body_nodes']
        require(len(bindings) == len(paths),'Missing recorded body mapping')
        require(len({row['node'] for row in bindings}) == len(paths),'Duplicate replay body node mapping')
        names = {node.get('name'):i for i,node in enumerate(document['nodes']) if 'name' in node}
        origin = np.asarray(info['origin_world_m'])
        for index,(path,row) in enumerate(zip(paths,bindings)):
            require(row['body_index'] == index and row['body_path'] == path,'Body order/path mismatch')
            require(names.get(row['node']) == row['node_index'],'Mapped body node missing in GLB')
            node = document['nodes'][row['node_index']]
            initial=poses[0,index] if reference_poses is None else reference_poses[index]
            same_array(np.asarray(row['reference_pose_xyzw']),initial.astype(float),'Reference body pose')
            require(np.allclose(node['translation'],C@(initial[:3].astype(float)-origin.astype(float)),rtol=0,atol=1e-12),'Initial glTF body translation mismatch')
            require(np.allclose(quaternion_matrices(node['rotation']),C@quaternion_matrices(initial[3:])@C.T,rtol=0,atol=1e-12),'Initial glTF body rotation mismatch')
        events_file = partial['events_path'] if partial else trial/('cumulative_break_events.json' if (trial/'cumulative_break_events.json').is_file() else 'break_events.json')
        events = read_json(events_file) if events_file.is_file() else []
        require(len(replay['releases']) == len(events),'Release event count mismatch')
        for exported,recorded in zip(replay['releases'],events):
            require(all(exported.get(key) == value for key,value in recorded.items()),'Release event source fields changed')
            require(event_time(exported) == event_time(recorded),'Release timestamp mismatch')
            require(float(times[0]) <= event_time(exported) <= float(times[-1]),'Release event outside recorded time range')
        transmission = replay.get('transmission')
        transmission_evidence = None
        if transmission:
            require(transmission['shape'] == [len(times),3,16], 'Transmission frame count mismatch')
            require(transmission.get('matrix_order') == 'column-major', 'Unknown transmission matrix layout')
            linkage = decode_array(replay_path,transmission,'<f4',tuple(transmission['shape']))
            require(len(transmission['nodes']) == 3 and all(name in names for name in transmission['nodes']), 'Transmission node mapping missing')
            # Reuse the mechanical reference kinematics, independently driven
            # by the original measured jaw DOFs rather than exported commands.
            sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/harvest_s300'))
            from motion_kinematics import transmission_transforms
            basis = np.eye(4)
            basis[:3,:3] = C
            checked_ticks = sorted(set((0,len(times)//2,len(times)-1)))
            diagnostic=transmission.get('measured_cad_travel_diagnostic')
            outside=np.flatnonzero((gap<.0279)|(gap>.0961)).tolist()
            require(not outside or diagnostic is not None,
                'Out-of-nominal measured jaw travel requires explicit display diagnostic provenance')
            if diagnostic is not None:
                require(diagnostic.get('enabled')is True and diagnostic.get('nominal_gap_bounds_m')==[.0279,.0961],
                    'Unknown measured CAD travel diagnostic')
                require(diagnostic.get('outside_nominal_frame_indices')==outside and
                    diagnostic.get('minimum_measured_gap_m')==float(gap.min()) and
                    diagnostic.get('maximum_measured_gap_m')==float(gap.max()),
                    'Measured CAD travel diagnostic differs from original actual jaw values')
                require(all(diagnostic.get(key)is False for key in ('actual_jaw_pose_changed',
                    'measured_gap_clamped','physics_or_planning_limits_changed')),
                    'Measured CAD travel diagnostic may not change actual physics or clamp jaws')
                checked_ticks=sorted(set(checked_ticks+outside))
            for tick in checked_ticks:
                reference = transmission_transforms(float(gap[tick]),measured_display_diagnostic=diagnostic is not None)
                require(transmission['nodes'] == ['transmission_'+name for name in reference], 'Transmission ordering mismatch')
                expected_linkage = np.array([(basis@matrix@basis.T).T.reshape(-1)
                                             for matrix in reference.values()],dtype='<f4')
                same_array(linkage[tick],expected_linkage,'Measured jaw transmission matrices')
            transmission_evidence = dict(nodes=3,frames=len(times),sampled_frames=checked_ticks,measured_gap_reference_exact=True)
            if diagnostic is not None:transmission_evidence['measured_cad_travel_diagnostic']=diagnostic
        skins = validate_skins(document,binary,info,assembly,manifest,paths,poses)
        quantization_position = float(np.max(np.abs(motion[:,:,:3].astype(float)-poses[:,:,:3])))
        quantization_quaternion = float(np.max(np.abs(motion[:,:,3:].astype(float)-poses[:,:,3:])))
    inventory = info['source_inventory']
    require(len(inventory) == info['visible_source_meshes'],'Visible source inventory count mismatch')
    require(len({row['path'] for row in inventory}) == len(inventory),'Duplicate source visual paths')
    triangle_counts = []
    for mesh in document['meshes']:
        count = 0
        for primitive in mesh['primitives']:
            require(primitive.get('mode',4) == 4,'Expected triangle geometry')
            require('POSITION' in primitive['attributes'],'Missing visible vertex positions')
            size = document['accessors'][primitive['indices']]['count'] if 'indices' in primitive else document['accessors'][primitive['attributes']['POSITION']]['count']
            require(size%3 == 0,'Incomplete triangle primitive')
            count += size//3
        triangle_counts.append(count)
    for row in inventory:
        require(0 <= row['mesh'] < len(triangle_counts),'Inventory references missing mesh')
        require(row['triangles'] == triangle_counts[row['mesh']],'Source triangle count differs from GLB geometry')
    require(sum(row['triangles'] for row in inventory) == info['source_triangles_with_instances'],'Total source triangle count mismatch')
    actual_instances = 0
    group_counts = {key:0 for key in info['groups']}
    parents = {child:index for index,node in enumerate(document['nodes']) for child in node.get('children',[])}
    for index,node in enumerate(document['nodes']):
        if 'mesh' not in node:
            continue
        attributes = node.get('extensions',{}).get('EXT_mesh_gpu_instancing',{}).get('attributes')
        count = len(accessor(document,binary,attributes['TRANSLATION'])) if attributes else 1
        if attributes:
            require(all(len(accessor(document,binary,value)) == count for value in attributes.values()),'Instancing attribute count mismatch')
        actual_instances += count
        ancestor = index
        while ancestor in parents:
            ancestor = parents[ancestor]
        group = document['nodes'][ancestor].get('name')
        require(group in group_counts,'Visual mesh outside known scene groups')
        group_counts[group] += count
    require(actual_instances == len(inventory),'Exported GLB omits/duplicates source mesh instances')
    for group,row in info['groups'].items():
        expected = sum(item['group'] == group for item in inventory)
        require(row['visible_source_meshes'] == expected == group_counts[group], 'Scene group coverage mismatch: '+group)
    require(group_counts.get('background_plants',0) > 0 and group_counts.get('greenhouse',0) > 0,'Missing full-farm background or house geometry')
    require(group_counts.get('robot',0)>0 and group_counts.get('target',0)>0,'Missing robot or active target visuals')
    camera = validate_camera(replay_path,replay['camera'],assembly,trial) if replay.get('camera') else None
    return dict(schema='farmily-harvest-replay-independent-validation-v1',passed=True,
                replay=str(replay_path),source_trial=str(trial),source_assembly=str(assembly),
                source_scene_sha256=source_hash,source_states_sha256=states_hash,web_glb_sha256=glb_hash,
                frames=len(times),bodies=len(paths),duration_s=float(times[-1]-times[0]),
                exact_original_timestamps=True,all_frames_preserved=True,all_body_paths_mapped=True,
                maximum_position_quantization_m=quantization_position,
                maximum_quaternion_component_quantization=quantization_quaternion,
                phases_exact=True,release_events_exact=len(events),runtime_physical_model_hash_verified=True,
                geometry_reuse_verified=geometry_reuse is not None,
                geometry_reference_replay=geometry_reuse['reference_replay'] if geometry_reuse else None,
                scene_source_meshes=len(inventory),scene_mesh_instances=actual_instances,
                scene_triangles_with_instances=info['source_triangles_with_instances'],
                scene_groups=group_counts,skins=skins,transmission=transmission_evidence,camera=camera,physics_rerun=False,
                record_kind=KIND if partial else 'recorded_run',
                diagnostic_only=bool(partial),checkpoint_eligible=False if partial else None,
                harvest_credit=0 if partial else None,finalized_native_audit_claimed=False if partial else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path, help='Published run directory or replay.json')
    parser.add_argument('--output', type=Path, help='Write JSON validation evidence to a new file')
    parser.add_argument('--allow-partial-diagnostic', action='store_true', help='Validate only as incomplete actual diagnostic data; grants no harvest/checkpoint credit')
    args = parser.parse_args()
    replay_path = args.bundle/'replay.json' if args.bundle.is_dir() else args.bundle
    evidence = validate(replay_path,args.allow_partial_diagnostic)
    output = json.dumps(evidence, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(output)
    print(output, end='')


if __name__ == '__main__':
    try:
        main()
    except (ReplayMismatch, KeyError, OSError, ValueError) as error:
        print(json.dumps({'passed':False, 'error':str(error)}, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
