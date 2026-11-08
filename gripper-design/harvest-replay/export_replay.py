#!/usr/bin/env python3
"""Export an immutable S300 USD scene and measured states for browser replay.

Run using nvidia-sim/.asset-venv/bin/python. No physics is advanced. Source
triangles/materials, recorded timestamps and every recorded body are retained.
Static copies share geometry with EXT_mesh_gpu_instancing; rod meshes use
standard glTF skins with the same measured-bone formula as DeformationVisuals.
"""
import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import importlib.util
import json
import copy
import os
from pathlib import Path
import struct
import sys
import time

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdShade
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'nvidia-sim/harvest_s300'))
from motion_pose_display import MeasuredPoseDisplay, restore_recorded_display_fixture
from motion_kinematics import transmission_transforms
from recording_storage import share_file
from sensor_replay import export_camera
from partial_recording import partial_recording, published_partial_source

_spec = importlib.util.spec_from_file_location('assembly_web_export',
    REPO / 'gripper-design/robot-assembly/export_model.py')
_assembly_export = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_assembly_export)
source_mesh = _assembly_export.source_mesh
sha = _assembly_export.sha
C = _assembly_export.TO_GLTF
C4 = np.eye(4)
C4[:3, :3] = C
TRANSMISSION = '/World/ActuatedTool/fixed/Transmission/'


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def digest_array(array):
    a = np.ascontiguousarray(array)
    h = hashlib.sha256()
    h.update(str(a.shape).encode())
    h.update(a.dtype.str.encode())
    h.update(a.tobytes())
    return h.hexdigest()


class GLB:
    def __init__(self):
        self.doc = dict(asset=dict(version='2.0', generator='Farmily measured USD replay'),
                        scene=0, scenes=[dict(nodes=[])], nodes=[], meshes=[], materials=[],
                        accessors=[], bufferViews=[], buffers=[], skins=[])
        self.binary = bytearray()
        self.materials = {}
        self.meshes = {}

    def accessor(self, values, kind, component=5126, target=None, bounds=False):
        dtype = {5126: '<f4', 5123: '<u2', 5125: '<u4', 5121: 'u1'}[component]
        a = np.asarray(values, dtype=dtype)
        if np.issubdtype(a.dtype, np.floating) and not np.isfinite(a).all():
            raise ValueError('Nonfinite glTF buffer')
        while len(self.binary) % 4:
            self.binary.append(0)
        view = dict(buffer=0, byteOffset=len(self.binary), byteLength=a.nbytes)
        if target:
            view['target'] = target
        self.binary.extend(a.tobytes())
        vi = len(self.doc['bufferViews'])
        self.doc['bufferViews'].append(view)
        item = dict(bufferView=vi, componentType=component, count=len(a), type=kind)
        if bounds and len(a):
            item['min'] = a.min(axis=0).reshape(-1).tolist()
            item['max'] = a.max(axis=0).reshape(-1).tolist()
        ai = len(self.doc['accessors'])
        self.doc['accessors'].append(item)
        return ai

    def material(self, values, vertex_colors=False):
        rgb, metallic, roughness = values
        key = tuple(rgb), float(metallic), float(roughness), vertex_colors
        if key not in self.materials:
            index = len(self.doc['materials'])
            self.doc['materials'].append(dict(name=f'material_{index}', doubleSided=True,
                pbrMetallicRoughness=dict(baseColorFactor=[*(rgb if not vertex_colors else (1, 1, 1)), 1],
                                         metallicFactor=float(metallic), roughnessFactor=float(roughness))))
            self.materials[key] = index
        return self.materials[key]

    def node(self, name, matrix=None, parent=None, **kwargs):
        n = dict(name=name, **kwargs)
        if matrix is not None:
            n['matrix'] = np.asarray(matrix).T.reshape(-1).tolist()
        index = len(self.doc['nodes'])
        self.doc['nodes'].append(n)
        if parent is None:
            self.doc['scenes'][0]['nodes'].append(index)
        else:
            self.doc['nodes'][parent].setdefault('children', []).append(index)
        return index

    def finish(self, path):
        self.doc['buffers'] = [dict(byteLength=len(self.binary))]
        if not self.doc['skins']:
            del self.doc['skins']
        header = json.dumps(self.doc, separators=(',', ':'), ensure_ascii=False).encode()
        header += b' ' * ((-len(header)) % 4)
        self.binary += b'\0' * ((-len(self.binary)) % 4)
        total = 12 + 8 + len(header) + 8 + len(self.binary)
        with path.open('wb') as out:
            out.write(struct.pack('<4sII', b'glTF', 2, total))
            out.write(struct.pack('<I4s', len(header), b'JSON'))
            out.write(header)
            out.write(struct.pack('<I4s', len(self.binary), b'BIN\0'))
            out.write(self.binary)


def geometry(glb, prim, skin=None):
    """Reuse the established material/subset/normals reader, preserving faces."""
    points, faces, normals, colors, parts, left_handed = source_mesh(prim)
    source_count = len(points)
    mapping = np.arange(source_count)
    if skin is not None:
        # source_mesh expands split normals; carry the exact source vertex ids
        # through that expansion rather than guessing nearest vertices.
        mesh = UsdGeom.Mesh(prim)
        if mesh.GetNormalsInterpolation() in ('faceVarying', 'uniform') and len(mesh.GetNormalsAttr().Get() or []):
            mapping = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), int)
        if len(mapping) != source_count:
            raise ValueError('Skin/source vertex mapping mismatch: ' + str(prim.GetPath()))
        points = skin['vertices'][mapping]
        points = (points - skin['origin']) @ C.T
        ids = skin['ids'][mapping]
        weights = skin['weights'][mapping]
        if ids.shape[1] > 4:
            raise ValueError('More than four skin influences need explicit support')
        ji = np.zeros((len(ids), 4), dtype=np.uint16)
        we = np.zeros((len(ids), 4), dtype=np.float32)
        ji[:, :ids.shape[1]] = ids
        we[:, :weights.shape[1]] = weights
    else:
        points = points @ C.T
    normals = normals @ C.T if len(normals) else normals
    if left_handed:
        faces = faces[:, ::-1]
    # Content hashing also merges repeated non-instance imported GLB geometry.
    key = tuple(digest_array(x) for x in (points.astype('<f4'), faces.astype('<u4'),
                 normals.astype('<f4'), np.asarray(colors if colors is not None else [])))
    key += tuple((digest_array(face_ids), tuple(material[0]), float(material[1]), float(material[2]))
                 for face_ids, material in parts)
    if skin is not None:
        key += (digest_array(ji), digest_array(we))
    if key in glb.meshes:
        return glb.meshes[key], len(faces), np.array([points.min(0), points.max(0)])
    primitives = []
    for face_ids, material in parts:
        if not len(face_ids):
            continue
        selected = faces[face_ids]
        unique, inverse = np.unique(selected.reshape(-1), return_inverse=True)
        attributes = dict(POSITION=glb.accessor(points[unique], 'VEC3', target=34962, bounds=True))
        if len(normals):
            attributes['NORMAL'] = glb.accessor(normals[unique], 'VEC3', target=34962)
        if colors is not None:
            attributes['COLOR_0'] = glb.accessor(np.c_[colors[unique], np.ones(len(unique))], 'VEC4', target=34962)
        if skin is not None:
            attributes['JOINTS_0'] = glb.accessor(ji[unique], 'VEC4', component=5123, target=34962)
            attributes['WEIGHTS_0'] = glb.accessor(we[unique], 'VEC4', target=34962)
            attributes['_SOURCE_VERTEX'] = glb.accessor(mapping[unique].astype(np.uint32),
                                                       'SCALAR', component=5125, target=34962)
        primitives.append(dict(attributes=attributes,
            indices=glb.accessor(inverse.astype(np.uint32), 'SCALAR', component=5125, target=34963),
            material=glb.material(material, vertex_colors=colors is not None), mode=4))
    index = len(glb.doc['meshes'])
    glb.doc['meshes'].append(dict(name=f'geometry_{index:05}', primitives=primitives))
    glb.meshes[key] = index
    return index, len(faces), np.array([points.min(0), points.max(0)])


def body_matrix(pose, origin):
    tf = np.eye(4)
    tf[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    tf[:3, 3] = pose[:3]
    out = C4 @ tf @ C4.T
    out[:3, 3] -= C @ origin
    return out


def source_transform(tf, origin):
    out = C4 @ tf @ C4.T
    out[:3, 3] -= C @ origin
    return out


def read_skins(assembly, manifest, index, origin):
    answer = {}
    for plant, name in [(manifest['plant'], 'skin.npz')] + [
            (r['plant'], r['skin_file']) for r in manifest.get('elastic_neighbors', [])]:
        with np.load(assembly / name, allow_pickle=False) as n:
            for i, path in enumerate(plant['skin_paths']):
                bodies = n[f'{i}_body_paths'].tolist()
                answer[path] = dict(vertices=n[f'{i}_vertices'], ids=n[f'{i}_ids'],
                    weights=n[f'{i}_weights'], rest=n[f'{i}_rest'],
                    body_ids=np.array([index[p] for p in bodies]), body_paths=bodies,
                    origin=origin)
    return answer


def group_name(path):
    if path.startswith('/World/HarvestRobot') or path.startswith('/World/ActuatedTool'):
        return 'robot'
    if path.startswith('/World/ElasticTarget'):
        return 'target'
    if path.startswith('/World/ElasticNeighbors'):
        return 'active_neighbors'
    if path.startswith('/World/Greenhouse/Plants'):
        return 'background_plants'
    return 'greenhouse'


def decompose_matrices(matrices):
    a = np.asarray(matrices)
    scales = np.linalg.norm(a[:, :3, :3], axis=1)
    if np.any(scales <= 1e-15):
        raise ValueError('Degenerate source transform')
    rotations = a[:, :3, :3] / scales[:, None, :]
    flip = np.linalg.det(rotations) < 0
    scales[flip, 0] *= -1
    rotations[flip, :, 0] *= -1
    quats = Rotation.from_matrix(rotations).as_quat()
    reconstructed = Rotation.from_quat(quats).as_matrix() * scales[:, None, :]
    error = float(np.max(np.abs(reconstructed - a[:, :3, :3])))
    return a[:, :3, 3], quats, scales, error


def compress_file(source, dest):
    with source.open('rb') as inp, dest.open('wb') as out:
        with gzip.GzipFile(fileobj=out, mode='wb', compresslevel=6, mtime=0) as comp:
            while chunk := inp.read(4 * 1024 * 1024):
                comp.write(chunk)


def export(assembly, trial, output, title=None, sensor_data=None, allow_partial_diagnostic=False, reuse_scene_from=None,
           allow_measured_linkage_diagnostic=False):
    started = time.monotonic()
    assembly, trial, output = map(lambda x: Path(x).resolve(), (assembly, trial, output))
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use a new output directory: ' + str(output))
    output.mkdir(parents=True, exist_ok=True)
    source = assembly / ('composed.usdc' if (assembly / 'composed.usdc').is_file() else 'scene.usda')
    partial = None
    if allow_partial_diagnostic and not (trial/'summary.json').is_file():
        if sensor_data is not None:
            raise ValueError('Partial camera metadata must not be substituted from another capture')
        partial = partial_recording(assembly, trial)
    states = partial['states_path'] if partial else trial / 'states.npz'
    before = sha(source)
    state_sha = sha(states)
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    summary = {} if partial else json.loads((trial / 'summary.json').read_text())
    runtime = json.loads((trial / 'runtime_settings.json').read_text()) if (trial / 'runtime_settings.json').is_file() else {}
    wrapper_sha = sha(assembly / 'scene.usda') if (assembly / 'scene.usda').is_file() else None
    if summary.get('model_sha256') and wrapper_sha != summary['model_sha256']:
        raise ValueError('Recorded source scene.usda SHA mismatch')
    if runtime.get('native_scene_sha256') and before != runtime['native_scene_sha256']:
        raise ValueError('Recorded actual native composed scene SHA mismatch')
    with np.load(states, allow_pickle=False) as n:
        poses = n['poses']
        stamps = n['time']
        paths = n['body_paths'].tolist()
        phases = n['phase'].tolist()
        robot = n['robot_q']
    if poses.shape != (len(stamps), len(paths), 7) or len(set(paths)) != len(paths):
        raise ValueError('Invalid measured-state shape/body mapping')
    if not np.isfinite(poses).all() or not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError('Finite poses and strictly increasing original timestamps required')
    dofs = json.loads((trial / 'native_metadata.json').read_text())['robot_dofs']
    gap = .044 + robot[:, dofs.index('jaw_R')] - robot[:, dofs.index('jaw_L')]
    reused_info = None
    reuse_proof = None
    if reuse_scene_from is not None:
        if partial or (trial/'belt_fixture.json').exists():
            raise ValueError('Partial/transient-fixture records cannot reuse geometry')
        from scene_reuse import reference_identity
        from validate_replay import validate
        reference_path=Path(reuse_scene_from).resolve()
        if reference_path.is_dir():reference_path=reference_path/'replay.json'
        reference_validation=validate(reference_path)
        if reference_validation.get('passed') is not True:
            raise ValueError('Reference replay has not passed independent validation')
        reuse_proof,reused_info,scene_path,_=reference_identity(reference_path,
            scene_sha256=before,manifest_sha256=sha(assembly/'motion_manifest.json'),body_paths=paths)
        reused_info=copy.deepcopy(reused_info)
        scene_relative=os.path.relpath(scene_path,output)
        # Same served URL permits the existing browser model cache to work.
        # No farm GLB bytes are copied or regenerated for this recording.
        fixture=None;origin=np.asarray(reused_info['origin_world_m'],float)
        index={p:i for i,p in enumerate(paths)}
        body_nodes=reused_info['body_nodes'];bounds=np.asarray(reused_info['bounds_m'])
        skin_checks=reused_info['skin_checks'];inventory=reused_info['source_inventory']
        geom_counts={name:value['visible_source_meshes'] for name,value in reused_info['groups'].items()}
        groups={name:value['node'] for name,value in reused_info['groups'].items()}
        total_triangles=reused_info['source_triangles_with_instances']
        instanced_count=reused_info['static_instanced_mesh_count']
        max_instance_error=reused_info['maximum_instance_decomposition_error']
        transmission_nodes={name.removeprefix('transmission_'):None for name in reused_info['transmission_nodes']}
        scene_bytes=reused_info['bytes'];scene_sha=reused_info['sha256']
        compressed_scene_sha=reused_info['compressed_sha256'];compressed_scene_bytes=scene_path.stat().st_size
        unique_meshes=reused_info['unique_glTF_meshes']
        glb=None;dest=None
        print('[scene] reusing independently verified identical physical model',before,flush=True)
    else:
        stage = Usd.Stage.Open(str(source))
        if UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1.:
            raise ValueError('Expected metre/Z-up source scene')
        fixture = restore_recorded_display_fixture(stage, trial)
        display = MeasuredPoseDisplay(stage, paths)
        display.update(poses[0])
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for name, tf in transmission_transforms(float(gap[0])).items():
                prim = stage.GetPrimAtPath(TRANSMISSION + name)
                if prim:
                    UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(tf.T.tolist()))
        index = {p: i for i, p in enumerate(paths)}
        origin = poses[0, index.get('/World/HarvestRobot/Robot/world', 0), :3].copy()
        origin[2] = 0.
        skins = read_skins(assembly, manifest, index, origin)
        cache = UsdGeom.XformCache()
        glb = GLB()
        groups = {name: glb.node(name) for name in ('greenhouse', 'background_plants', 'target', 'active_neighbors', 'robot')}
        body_nodes = []
        for i, (path, pose) in enumerate(zip(paths, poses[0])):
            tf = body_matrix(pose, origin)
            # Joint transforms must remain TRS so GLTFLoader can create bones.
            node = glb.node(f'body_{i:04}', parent=groups[group_name(path)],
                            translation=tf[:3, 3].tolist(),
                            rotation=Rotation.from_matrix(tf[:3, :3]).as_quat().tolist())
            body_nodes.append(dict(node=f'body_{i:04}', node_index=node, body_path=path,
                                   body_index=i, reference_pose_xyzw=pose.tolist()))
        transmission_nodes = {}
        for name, tf in transmission_transforms(float(gap[0])).items():
            node = glb.node('transmission_' + name, matrix=C4 @ tf @ C4.T,
                            parent=body_nodes[index['/World/ActuatedTool/fixed']]['node_index'])
            transmission_nodes[name] = node
        owners = {}
        for path in paths:
            owners[path] = body_nodes[index[path]]['node_index']
        for name, node in transmission_nodes.items():
            owners[TRANSMISSION + name] = node
        local_cache, geometry_cache = {}, {}
        instances = defaultdict(list)
        inventory = []
        bounds = np.array([np.full(3, np.inf), np.full(3, -np.inf)])
        skin_checks = []
        geom_counts = Counter()
        total_triangles = 0
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            path = str(prim.GetPath())
            image = UsdGeom.Imageable(prim)
            if '/collisions/' in path or image.ComputeVisibility() == 'invisible' or image.ComputePurpose() == 'guide':
                continue
            tf = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            skin = skins.get(path)
            group = group_name(path)
            prototype = str(prim.GetPrimInPrototype().GetPath()) if prim.IsInstanceProxy() else path
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            material_key = str(material.GetPath()) if material else ''
            key = (prototype, material_key, path if skin is not None else '')
            if key not in geometry_cache:
                geometry_cache[key] = geometry(glb, prim, skin)
            mesh_index, triangles, box = geometry_cache[key]
            total_triangles += triangles
            geom_counts[group] += 1
            if skin is not None:
                if not np.allclose(tf, np.eye(4), atol=1e-9, rtol=0):
                    raise ValueError('Rod skin has a nonidentity world parent transform: ' + path)
                inverse_bind = np.repeat(np.eye(4)[None], len(skin['body_ids']), axis=0)
                inverse_bind[:, :3, 3] = -(skin['rest'] - origin) @ C.T
                si = len(glb.doc['skins'])
                glb.doc['skins'].append(dict(name=f'rod_skin_{si}',
                    joints=[body_nodes[i]['node_index'] for i in skin['body_ids']],
                    inverseBindMatrices=glb.accessor(inverse_bind.transpose(0, 2, 1).reshape(-1, 16), 'MAT4')))
                skin_node = glb.node(f'skin_{si:04}', parent=groups[group], mesh=mesh_index, skin=si,
                                     extras=dict(source_path=path, group=group))
                # Independently compare shader formula to existing CPU skinning at
                # start/mid/end, using actual export float32 storage precision.
                ids = skin['ids']; rest = skin['rest']; weights = skin['weights']
                pick = np.unique(np.linspace(0, len(ids)-1, min(256, len(ids))).astype(int))
                ticks = sorted(set((0, len(stamps)//2, len(stamps)-1)))
                maximum = 0.
                for tick in ticks:
                    p = poses[tick, skin['body_ids']]
                    r = Rotation.from_quat(p[:, 3:]).as_matrix()
                    source_points = skin['vertices'][pick]
                    expected = ((np.einsum('nkij,nkj->nki', r[ids[pick]],
                        source_points[:, None] - rest[ids[pick]]) + p[:, :3][ids[pick]]) * weights[pick, :, None]).sum(1)
                    encoded = poses[tick, skin['body_ids']].astype(np.float32).astype(float)
                    er = Rotation.from_quat(encoded[:, 3:]).as_matrix()
                    got = ((np.einsum('nkij,nkj->nki', er[ids[pick]],
                        source_points.astype(np.float32).astype(float)[:, None] - rest[ids[pick]]) +
                        encoded[:, :3][ids[pick]]) * weights[pick, :, None]).sum(1)
                    maximum = max(maximum, float(np.linalg.norm(got - expected, axis=1).max()))
                skin_checks.append(dict(path=path, node=f'skin_{si:04}', node_index=skin_node,
                                       skin_index=si, mesh_index=mesh_index,
                                       vertices=len(ids), influences=ids.shape[1],
                                       maximum_float32_position_error_m=maximum))
                world_box = box
            else:
                owner = prim
                while owner and str(owner.GetPath()) not in owners:
                    owner = owner.GetParent()
                owner_path = str(owner.GetPath()) if owner else None
                world_tf = source_transform(tf, origin)
                if owner_path:
                    if owner_path not in local_cache:
                        local_cache[owner_path] = np.linalg.inv(source_transform(np.asarray(
                            cache.GetLocalToWorldTransform(stage.GetPrimAtPath(owner_path))).T, origin))
                    local = local_cache[owner_path] @ world_tf
                    glb.node(f'visual_{len(inventory):05}', matrix=local, parent=owners[owner_path], mesh=mesh_index,
                             extras=dict(source_path=path, measured_owner_path=owner_path, group=group))
                else:
                    instances[(group, mesh_index)].append(world_tf)
                corners = np.array(np.meshgrid(*zip(box[0], box[1]), indexing='ij')).reshape(3, -1).T
                transformed = corners @ world_tf[:3, :3].T + world_tf[:3, 3]
                world_box = np.array([transformed.min(0), transformed.max(0)])
            bounds[0] = np.minimum(bounds[0], world_box[0])
            bounds[1] = np.maximum(bounds[1], world_box[1])
            inventory.append(dict(path=path, group=group, triangles=triangles, mesh=mesh_index,
                                  skinned=skin is not None))
            if len(inventory) % 1000 == 0:
                print(f'[scene] {len(inventory)} visible meshes; {len(glb.doc["meshes"])} unique geometries', flush=True)
        instanced_count = 0
        max_instance_error = 0.
        for (group, mesh_index), matrices in instances.items():
            if len(matrices) == 1:
                glb.node(f'static_{mesh_index:05}', parent=groups[group], matrix=matrices[0], mesh=mesh_index)
                continue
            translation, rotation, scale, error = decompose_matrices(matrices)
            if error > 1e-8:
                for i, tf in enumerate(matrices):
                    glb.node(f'static_{mesh_index:05}_{i}', parent=groups[group], matrix=tf, mesh=mesh_index)
                continue
            max_instance_error = max(max_instance_error, error)
            attributes = dict(TRANSLATION=glb.accessor(translation, 'VEC3', target=34962),
                              ROTATION=glb.accessor(rotation, 'VEC4', target=34962),
                              SCALE=glb.accessor(scale, 'VEC3', target=34962))
            glb.node(f'instances_{mesh_index:05}', parent=groups[group], mesh=mesh_index,
                     extensions=dict(EXT_mesh_gpu_instancing=dict(attributes=attributes)))
            instanced_count += len(matrices)
        if instanced_count:
            glb.doc['extensionsUsed'] = ['EXT_mesh_gpu_instancing']
            glb.doc['extensionsRequired'] = ['EXT_mesh_gpu_instancing']
        dest = output / 'scene.glb'
        glb.finish(dest)
        print('[scene] compressing', dest.stat().st_size, 'bytes', flush=True)
        compress_file(dest, output / 'scene.glb.gz')
        scene_bytes = dest.stat().st_size
        scene_sha = sha(dest)
        with gzip.open(output/'scene.glb.gz', 'rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != scene_sha:
                raise ValueError('Compressed scene does not preserve original GLB')
        share_file(output/'scene.glb.gz', output.parent/'_shared')
        scene_relative='scene.glb.gz'
        compressed_scene_sha=sha(output/'scene.glb.gz')
        compressed_scene_bytes=(output/'scene.glb.gz').stat().st_size
        unique_meshes=len(glb.doc['meshes'])
    poses32 = poses.astype('<f4')
    with gzip.GzipFile(filename=str(output / 'poses.bin.gz'), mode='wb', mtime=0) as f:
        f.write(poses32.tobytes())
    stamps.astype('<f8').tofile(output / 'times.bin')
    linkage = np.empty((len(stamps), 3, 16), dtype='<f4')
    outside_cad=np.flatnonzero((gap<.0279)|(gap>.0961))
    linkage_diagnostic=None
    if allow_measured_linkage_diagnostic:
        linkage_diagnostic=dict(enabled=True,nominal_gap_bounds_m=[.0279,.0961],
            minimum_measured_gap_m=float(gap.min()),maximum_measured_gap_m=float(gap.max()),
            outside_nominal_frame_indices=outside_cad.tolist(),
            actual_jaw_pose_changed=False,measured_gap_clamped=False,physics_or_planning_limits_changed=False,
            scope='Recorded display only; exact CAD linkage equations within triangle closure; out-of-nominal measured travel is not a validated hardware motion')
    for tick, width in enumerate(gap):
        for j, tf in enumerate(transmission_transforms(float(width),
                measured_display_diagnostic=allow_measured_linkage_diagnostic).values()):
            linkage[tick, j] = (C4 @ tf @ C4.T).T.reshape(-1)
    with gzip.GzipFile(filename=str(output / 'transmission.bin.gz'), mode='wb', mtime=0) as f:
        f.write(linkage.tobytes())
    releases = []
    events = partial['events_path'] if partial else trial / ('cumulative_break_events.json' if (trial / 'cumulative_break_events.json').exists() else 'break_events.json')
    if events.exists():
        releases = json.loads(events.read_text())
        fruit_rows = manifest['plant']['fruits'] + [f for r in manifest.get('elastic_neighbors', []) for f in r['plant']['fruits']]
        fruits_by_joint = {f['joint']: f for f in fruit_rows}
        fruits_by_name = {f.get('name', f['path'].rsplit('/', 1)[-1]): f for f in fruit_rows}
        normalized = []
        for event in releases:
            row = dict(event)
            event_time = event.get('release_effective_time_s', event.get('t', event.get('time_s')))
            if event_time is None:
                raise ValueError('Release event lacks an actual recorded timestamp')
            row['time_s'] = float(event_time)
            fruit = fruits_by_joint.get(event.get('joint')) or fruits_by_name.get(event.get('name'))
            if fruit:
                row.setdefault('body_path', fruit['path'])
                row.setdefault('name', fruit.get('name', fruit['path'].rsplit('/', 1)[-1]))
            normalized.append(row)
        releases = normalized
    scene_info = dict(schema='farmily-harvest-replay-scene-v1', scene_axes='glTF Y-up metres',
        source_axes='USD Z-up metres', origin_world_m=origin.tolist(),
        target_body_path=manifest['target']['path'],
        target_center_world_m=poses[0, index[manifest['target']['path']], :3].tolist(),
        source_scene=str(source), source_scene_sha256=before,
        source_wrapper_sha256=wrapper_sha, recorded_runtime_native_scene_sha256=runtime.get('native_scene_sha256'),
        runtime_native_hash_verified=bool(runtime.get('native_scene_sha256')), file=scene_relative, encoding='gzip', bytes=scene_bytes,
        sha256=scene_sha, compressed_file=scene_relative, compressed_sha256=compressed_scene_sha,
        bounds_m=bounds.tolist(), body_nodes=body_nodes,
        transmission_nodes=['transmission_' + n for n in transmission_nodes],
        groups={k: dict(node=k, visible_source_meshes=geom_counts[k]) for k in groups},
        visible_source_meshes=len(inventory), source_triangles_with_instances=total_triangles,
        unique_glTF_meshes=unique_meshes, static_instanced_mesh_count=instanced_count,
        maximum_instance_decomposition_error= max_instance_error,
        skinned_mesh_count=len(skin_checks), skin_checks=skin_checks,
        source_surface_preserved=True, source_inventory=inventory,
        fixture=fixture, physics_rerun=False)
    if reuse_proof is not None:scene_info['geometry_reuse']=reuse_proof
    write_json(output/'replay_scene.json', scene_info)
    p_error = float(np.max(np.abs(poses32[:, :, :3].astype(float) - poses[:, :, :3])))
    q_error = float(np.max(np.abs(poses32[:, :, 3:].astype(float) - poses[:, :, 3:])))
    replay = dict(schema='farmily-harvest-replay-v1', title=title or trial.name,
        scene_url=scene_relative, scene_encoding='gzip', scene_gzip_url=scene_relative, scene_info_url='replay_scene.json',
        target_body_path=manifest['target']['path'],
        target_center_world_m=scene_info['target_center_world_m'],
        motion=dict(url='poses.bin.gz', compression='gzip', dtype='float32', shape=list(poses.shape),
                    bytes_uncompressed=poses32.nbytes, sha256=sha(output/'poses.bin.gz'),
                    source_axes='USD Z-up world; xyz + quaternion xyzw',
                    maximum_position_quantization_m=p_error, maximum_quaternion_component_quantization=q_error),
        time=dict(url='times.bin', dtype='float64', shape=[len(stamps)], sha256=sha(output/'times.bin'),
                  start_s=float(stamps[0]), end_s=float(stamps[-1]), original_timestamps=True,
                  minimum_interval_s=float(np.diff(stamps).min()), maximum_interval_s=float(np.diff(stamps).max())),
        phase=phases, jaw_gap_m=gap.tolist(), releases=releases,
        transmission=dict(url='transmission.bin.gz', compression='gzip', dtype='float32',
            shape=list(linkage.shape), nodes=scene_info['transmission_nodes'], matrix_order='column-major'),
        engine=dict(physics_engine=summary.get('physics_engine', 'Isaac Sim / PhysX'),
                    backend=summary.get('backend', (summary.get('physics_settings') or {}).get('device')),
                    solver=(summary.get('physics_settings') or {}).get('solver'),
                    hz=(summary.get('physics_settings') or {}).get('hz')),
        result_summary=dict(completed=summary.get('completed'), stop_reason=summary.get('stop_reason'),
                    harvest_success=summary.get('harvest_success'),
                    released_fruits=len(releases), new_break_count=summary.get('native_new_break_count'),
                    physics_steps=summary.get('physics_steps'),
                    physics_wall_s=summary.get('physics_wall_s'), simulated_s=summary.get('simulated_s'),
                    training_eligible=summary.get('training_eligible', False)),
        source=dict(assembly=str(assembly), trial=str(trial), scene_sha256=before,
                    wrapper_sha256=wrapper_sha, recorded_model_sha256=summary.get('model_sha256'),
                    recorded_runtime_native_scene_sha256=runtime.get('native_scene_sha256'),
                    runtime_native_hash_verified=bool(runtime.get('native_scene_sha256')),
                    summary_sha256=sha(trial/'summary.json') if not partial else None,
                    runtime_sha256=sha(trial/'runtime_settings.json') if (trial/'runtime_settings.json').is_file() else None,
                    manifest_sha256=sha(assembly/'motion_manifest.json'),
                    states_sha256=state_sha, all_frames_exported=True,
                    physics_rerun=False, new_simulation=False),
        body_count=len(paths), frames=len(stamps), duration_s=float(stamps[-1]-stamps[0]),
        source_geometry_modified=False, export_elapsed_s=time.monotonic()-started)
    if partial:
        replay['source'].update(published_partial_source(partial, trial))
        replay['source']['recorded_model_sha256'] = wrapper_sha
        replay['result_summary'] = partial['result']
        replay['engine'].update(backend=runtime.get('device'), solver=runtime.get('solver'), hz=runtime.get('hz'))
        replay['partial_diagnostic'] = dict(schema='incomplete-native-replay-v1',
            complete=False, checkpoint_eligible=False, harvest_credit=0,
            camera_unavailable_reason='Native exception interrupted camera finalization; no substituted camera data',
            source_descriptor=partial['descriptor'], runtime_error=partial['runtime_error'])
    if reuse_proof is not None:replay['geometry_reuse']=reuse_proof
    if linkage_diagnostic is not None:
        replay['transmission']['measured_cad_travel_diagnostic']=linkage_diagnostic
    camera = None if partial else export_camera(assembly, trial, output, sensor_data)
    if camera is not None:
        replay['camera'] = camera
    if before != sha(source) or state_sha != sha(states):
        raise ValueError('Source asset or recorded states changed during export')
    write_json(output/'replay.json', replay)
    # Both manifests now point to verified lossless bytes; remove only the redundant GLB.
    if dest is not None:dest.unlink()
    print(json.dumps(dict(output=str(output), frames=len(stamps), bodies=len(paths),
        source_meshes=len(inventory), unique_meshes=unique_meshes,
        instanced_meshes=instanced_count, skinned_meshes=len(skin_checks),
        scene_bytes=scene_bytes, scene_gzip_bytes=compressed_scene_bytes,
        scene_geometry_reused=reuse_proof is not None,
        elapsed_s=replay['export_elapsed_s']), ensure_ascii=False), flush=True)
    return replay


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('assembly', type=Path)
    p.add_argument('trial', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--title')
    p.add_argument('--sensor-data', type=Path, help='Camera directory from this recording; external captures require source-SHA provenance')
    p.add_argument('--allow-partial-diagnostic', action='store_true', help='Opt in to an explicitly incomplete actual partial trace, never a normal result/checkpoint')
    p.add_argument('--reuse-scene-from',type=Path,help='Opt-in reuse of an independently verified replay scene with identical actual physical scene/manifest SHA and complete body mapping; exports new actual motion/camera only')
    p.add_argument('--allow-measured-linkage-diagnostic',action='store_true',help='Display-only exact linkage equations for measured travel outside the nominal CAD slider range; retains actual raw jaw poses and records all affected frames')
    a = p.parse_args()
    export(a.assembly, a.trial, a.output, a.title, a.sensor_data, a.allow_partial_diagnostic,a.reuse_scene_from,
           a.allow_measured_linkage_diagnostic)
