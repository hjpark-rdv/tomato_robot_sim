"""Read-only recording of exact PhysX body poses for offline video rendering.

The existing candidate planner/executor is unchanged. No render-time updates,
kinematic playback, or USD edits occur during physics execution.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from pose_worker import (state, state_comparison, structure, plan, execute,
                         RobotKinematics, DEFAULT_LIMITS, classify, write_json)


def body_poses(env):
    return np.concatenate((env.robot.data.body_state_w[0, :, :7].cpu().numpy(),
        np.stack([f.data.root_state_w[0, :7].cpu().numpy() for f in env.fruits]),
        env.elastic._visual_transforms()[0])).copy()


def export_scene(env, destination, extra_bodies=()):
    """Export original visible geometry; retain the original elastic skin weights."""
    from pxr import Usd, UsdGeom, UsdShade
    from omni.physx import get_physx_interface
    import trimesh
    env.elastic.sync_visuals()
    env.sim.physics_sim_view.update_articulations_kinematic()
    get_physx_interface().update_transformations(False, True, True)
    cache = UsdGeom.XformCache()
    robot_paths = list(env.robot.root_physx_view.link_paths[0])
    fruit_paths = [s['path'] for s in env.fruit_specs]
    elastic_paths = env.elastic.paths + [s['anchor'] for s in env.fruit_specs]
    paths = robot_paths + fruit_paths + elastic_paths + [p for p, _ in extra_bodies]
    elastic_start = len(robot_paths) + len(fruit_paths)
    owners = {p: i for i, p in enumerate(paths)}
    # A native-teleported kinematic fixture may still have its parking transform
    # in USD. Bind its child geometry in body-local coordinates, then place it
    # using the recorded native pose, not that stale render transform.
    extra_frames = {}
    if extra_bodies:
        from scipy.spatial.transform import Rotation
        for path, pose in extra_bodies:
            usd_frame = np.asarray(cache.GetLocalToWorldTransform(env.stage.GetPrimAtPath(path)))
            extra_frames[owners[path]] = (usd_frame, Rotation.from_quat(np.asarray(pose)[[4, 5, 6, 3]]).as_matrix(), np.asarray(pose)[:3])
    skins = {str(s[0].GetPath()): s for s in env.elastic.skin}
    rigids = {str(op.GetAttr().GetPrim().GetPath()): idx for op, _, _, idx in env.elastic.rigid_visuals}
    arrays = {'body_rest': body_poses(env), 'elastic_visual_rest': env.elastic.visual_rest}
    if extra_bodies:
        arrays['body_rest'] = np.concatenate((arrays['body_rest'], np.asarray([p for _, p in extra_bodies])))
    objects = []; unsupported = []; hidden = 0
    for prim in Usd.PrimRange.Stage(env.stage, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim): continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputeVisibility() == 'invisible' or imageable.ComputePurpose() in ('guide', 'proxy'):
            hidden += 1; continue
        path = str(prim.GetPath())
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
            counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32)
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
            if not len(vertices) or not len(indices): continue
        else:
            shape = None
            if prim.IsA(UsdGeom.Cube):
                shape = trimesh.creation.box(extents=np.full(3, UsdGeom.Cube(prim).GetSizeAttr().Get()))
            elif prim.IsA(UsdGeom.Sphere):
                shape = trimesh.creation.icosphere(subdivisions=3, radius=UsdGeom.Sphere(prim).GetRadiusAttr().Get())
            elif prim.IsA(UsdGeom.Cylinder):
                cylinder = UsdGeom.Cylinder(prim)
                shape = trimesh.creation.cylinder(radius=cylinder.GetRadiusAttr().Get(), height=cylinder.GetHeightAttr().Get(), sections=32)
                axis = str(cylinder.GetAxisAttr().Get())
                if axis == 'X': shape.vertices = shape.vertices[:, [2, 0, 1]]
                if axis == 'Y': shape.vertices = shape.vertices[:, [1, 2, 0]]
            elif prim.IsA(UsdGeom.Capsule):
                capsule = UsdGeom.Capsule(prim)
                shape = trimesh.creation.capsule(radius=capsule.GetRadiusAttr().Get(),
                    height=capsule.GetHeightAttr().Get(), count=[8, 16])
                shape.vertices -= (shape.bounds[0] + shape.bounds[1]) / 2
                axis = str(capsule.GetAxisAttr().Get())
                if axis == 'X': shape.vertices = shape.vertices[:, [2, 0, 1]]
                if axis == 'Y': shape.vertices = shape.vertices[:, [1, 2, 0]]
            elif prim.GetTypeName() == 'Plane':
                width = prim.GetAttribute('width').Get() or 200
                length = prim.GetAttribute('length').Get() or 200
                shape = trimesh.creation.box(extents=[width, length, .001])
            if shape is None:
                unsupported.append(dict(path=path, type=prim.GetTypeName())); continue
            vertices = np.asarray(shape.vertices, dtype=np.float32)
            indices = np.asarray(shape.faces, dtype=np.int32).reshape(-1)
            counts = np.full(len(shape.faces), 3, dtype=np.int32)
        tf = np.asarray(cache.GetLocalToWorldTransform(prim))
        world = vertices @ tf[:3, :3] + tf[3, :3]
        key = 'mesh_%04d' % len(objects)
        binding = dict(kind='static')
        if path in skins:
            _, points, _, ids, weights, _, _ = skins[path]
            arrays[key+'_skin_points'] = points.astype(np.float32)
            arrays[key+'_skin_ids'] = ids.astype(np.int32)
            arrays[key+'_skin_weights'] = weights.astype(np.float32)
            binding = dict(kind='skin')
        elif path in rigids:
            binding = dict(kind='rigid', body=elastic_start+rigids[path])
        else:
            ancestor = prim
            while ancestor and not ancestor.IsPseudoRoot():
                if str(ancestor.GetPath()) in owners:
                    binding = dict(kind='rigid', body=owners[str(ancestor.GetPath())]); break
                ancestor = ancestor.GetParent()
        if binding.get('body') in extra_frames:
            usd_frame, native_r, native_p = extra_frames[binding['body']]
            world = (world-usd_frame[3, :3]) @ np.linalg.inv(usd_frame[:3, :3]) @ native_r.T + native_p
        color = [.55, .55, .55]
        display = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        if display: color = list(display[0])
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if material:
            for node in Usd.PrimRange(material.GetPrim()):
                if not node.IsA(UsdShade.Shader): continue
                shader = UsdShade.Shader(node)
                for name in ('diffuseColor', 'diffuse_color_constant', 'base_color'):
                    value = shader.GetInput(name).Get() if shader.GetInput(name) else None
                    if value is not None and hasattr(value, '__len__') and len(value) >= 3:
                        color = list(value[:3]); break
        arrays[key+'_vertices'] = world.astype(np.float32)
        arrays[key+'_counts'] = counts
        arrays[key+'_indices'] = indices
        objects.append(dict(key=key, path=path, binding=binding, color=color))
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, **arrays)
    manifest = dict(objects=objects, body_paths=paths, elastic_start=elastic_start,
        target_center=env._target_geometry()[0][0].tolist(), target_id='Tomato_05',
        fruit_specs=[dict(name=s['name'], body=len(robot_paths)+i, center=np.asarray(s['center']).tolist())
                     for i,s in enumerate(env.fruit_specs)],
        hidden_original_geometry_count=hidden, unsupported_visible_geometry=unsupported,
        mesh_count=len(objects), vertex_count=sum(len(arrays[o['key']+'_vertices']) for o in objects),
        appearance='Original visible USD meshes and constant material colours; offline studio shading, no RTX textures',
        motion='Actual PhysX body transforms and original elastic visual skinning; no new dynamics or inferred movement')
    write_json(destination.with_suffix('.json'), manifest)
    if unsupported: raise RuntimeError('Visible geometry cannot be silently omitted: '+str(unsupported))
    print('[VIDEO GEOMETRY]', manifest['mesh_count'], manifest['vertex_count'], flush=True)


def run_capture(env, args, app):
    from pose_collision import SelfCollisionCheck
    if args.pose_candidate is None or args.pose_video_source is None:
        raise ValueError('pose-video requires candidate and source experiment')
    source = args.pose_video_source
    experiment = json.loads((source/'experiment.json').read_text())
    model_update = args.pose_video_model_update
    changed_sources = {}
    for name, digest in experiment['source_sha256'].items():
        current = hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
        if current != digest:
            if not model_update or name != 'assets.py':
                raise RuntimeError('Physics source differs from original experiment: '+name)
            changed_sources[name] = dict(original=digest, current=current)
    if model_update and 'assets.py' not in experiment['source_sha256']:
        changed_sources['assets.py'] = dict(original=None,
            current=hashlib.sha256((Path(__file__).parent/'assets.py').read_bytes()).hexdigest(),
            note='Historical experiment did not fingerprint assets.py; repaired live colliders verified below')
    if model_update and set(changed_sources) != {'assets.py'}:
        raise RuntimeError('Model-update recording requires an explicit assets.py change')
    directory = args.run_dir; directory.mkdir(parents=True, exist_ok=True)
    params = json.loads(args.pose_candidate.read_text())
    original = source/'dataset/scene_0001'/params['candidate_id']
    baseline = json.loads((original/'candidate.json').read_text())
    if params != baseline['parameters']: raise ValueError('Candidate parameters changed')
    env.reset(); first = state(env); env.reset(); initial = state(env)
    reference = dict(np.load(source/'audit/initial_state.npz', allow_pickle=False))
    checks = dict(repeated=state_comparison(first, initial), baseline=state_comparison(reference, initial))
    if not all(c['passed'] for c in checks.values()): raise RuntimeError('Initial state mismatch')
    kin = RobotKinematics(env); env.pose_search_kin = kin
    identity = structure(env, directory); checker = SelfCollisionCheck(env, kin)
    planned, preflight = plan(env, kin, checker, params)
    if planned is None: raise RuntimeError('Previously valid candidate failed planning: '+str(preflight))
    old_commands = np.load(original/'planned_commands.npy')
    if not np.array_equal(planned['commands'], old_commands): raise RuntimeError('Planned commands changed')
    checks['planned_commands'] = dict(passed=True, same_as_baseline=True, count=len(old_commands))
    if model_update:
        from pxr import UsdGeom, UsdPhysics
        rails = []
        for i in range(2):
            path = identity['hook_prim'] + f'/RailCollision/rail_{i:02d}'
            prim = env.stage.GetPrimAtPath(path)
            if not prim or not prim.IsA(UsdGeom.Capsule) or not prim.HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError('Missing repaired wire collider: '+path)
            shape = UsdGeom.Capsule(prim)
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True:
                raise RuntimeError('Disabled repaired wire collider: '+path)
            radius = float(shape.GetRadiusAttr().Get())
            height = float(shape.GetHeightAttr().Get())
            if not np.isclose(radius, .001) or not np.isclose(height, .02840287):
                raise RuntimeError('Repaired wire dimensions differ from CAD')
            rails.append(dict(path=path, radius_m=radius, centerline_length_m=height))
        checks['collision_repair'] = dict(passed=True, rails=rails)
    if args.pose_video_export:
        export_scene(env, args.pose_video_export)
        checks['after_export'] = state_comparison(initial, state(env))
        if not checks['after_export']['passed']: raise RuntimeError('Export modified initial physics state')
    frames = [body_poses(env)]; times = [0.]
    original_update = env.scene.update
    tick = 0
    def observe(dt):
        nonlocal tick
        original_update(dt); tick += 1
        # Exactly every 4 control ticks: 15 simulation frames/s, encoded at 30 fps = 2x.
        if tick % (env.cfg.decimation*4) == 0:
            frames.append(body_poses(env)); times.append(tick*env.physics_dt)
    env.scene.update = observe
    try:
        metrics, trace = execute(env, planned, identity, app, DEFAULT_LIMITS.copy())
    finally:
        env.scene.update = original_update
    final_time = tick*env.physics_dt
    # Keep the exact final pose as an endpoint (renderer does not stretch the timeline).
    if final_time > times[-1]+1e-10: frames.append(body_poses(env)); times.append(final_time)
    labels = classify(metrics, DEFAULT_LIMITS)
    old_trace = json.loads((original/'trace.json').read_text())
    equal_length = len(trace) == len(old_trace)
    joint_error = float(np.max(np.abs(np.asarray([r['joints'] for r in trace])-np.asarray([r['joints'] for r in old_trace])))) if equal_length else None
    checks['reexecution'] = dict(same_steps=equal_length, maximum_joint_difference=joint_error,
        same_result=labels['result']==baseline['result'], same_retention=metrics['retained_hook']==baseline['retained_hook'],
        same_first_contact=metrics['first_contact_object']==baseline['first_contact_object'],
        target_displacement_difference=abs(metrics['target_max_displacement_m']-baseline['target_max_displacement_m']),
        main_displacement_difference=abs(metrics['main_stem_max_displacement_m']-baseline['main_stem_max_displacement_m']))
    checks['reexecution']['passed'] = (equal_length and joint_error <= 1e-6 and
        all(checks['reexecution'][k] for k in ('same_result','same_retention','same_first_contact')) and
        checks['reexecution']['target_displacement_difference']<=1e-6 and checks['reexecution']['main_displacement_difference']<=1e-6)
    checks['model_update'] = dict(enabled=model_update, changed_sources=changed_sources,
        passed=bool(model_update and all(checks[k]['passed'] for k in
                    ('repeated', 'baseline', 'planned_commands', 'collision_repair'))),
        comparison='Same candidate, initial state and planned commands; changed collision response is expected')
    np.savez_compressed(directory/'motion.npz', poses=np.asarray(frames), times_s=np.asarray(times))
    write_json(directory/'trace.json', trace); write_json(directory/'contacts.json', env.contact_diagnostics)
    write_json(directory/'recording.json', dict(candidate_id=params['candidate_id'], parameters=params,
        source_experiment=str(source), **metrics, **labels, validation=checks,
        simulated_duration_s=final_time, captured_frames=len(frames), simulation_capture_fps=15,
        output_fps=30, playback_speed=2., physics_dt=env.physics_dt, control_dt=env.step_dt,
        body_pose_order='robot articulation bodies, fruit rigid bodies, elastic rods, attachment frames',
        renderer='offline; actual PhysX poses; original USD geometry'))
    if not (checks['reexecution']['passed'] or checks['model_update']['passed']):
        raise RuntimeError('Recording changed or did not reproduce original experiment; inspect recording.json')
    print('[VIDEO CAPTURE COMPLETE]', params['candidate_id'], len(frames), labels['result'], flush=True)
