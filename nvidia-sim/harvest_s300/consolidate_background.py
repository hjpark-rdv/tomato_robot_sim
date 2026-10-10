"""Opt-in exact static source-part collision consolidation; offline USD only.

The original assembly stays a sublayer. Its visual geometry and dynamic model
are retained. Seven existing local triangle libraries replace only background
plant colliders through references at their unchanged source-part transforms.
Collision shape identity/manifolds change, so native equivalence remains an
explicit experiment and this assembly is not training eligible.

For this full S300 asset, native SI wrapper-only assemblies failed on the second
simulation step despite equal composed physics. Use --author-motion-specs-in-root
variant for the bounded native experiment; default wrapper output remains an
offline diagnostic. The 2026-10-10 CPU PGS120 7-second pulse with direct specs
matched all saved baseline poses, joints and contact records exactly, but showed
no speed gain. Evidence lives in performance/consolidation_native_validation;
this does not establish harvest success or equivalence on untested trajectories.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


PLANTS = '/World/Greenhouse/Plants/'
HOUSE = '/World/Greenhouse/House'
PRESERVED = ('/World/ElasticTarget', '/World/ActuatedTool',
             '/World/HarvestRobot', HOUSE, '/World/PhysicsScene', '/World/Ground')
MOTION_ROOTS = ('/World/ElasticTarget', '/World/ActuatedTool',
                '/World/HarvestRobot', '/World/PhysicsScene')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(2**20), b''):
            digest.update(block)
    return digest.hexdigest()


def world(cache, prim):
    return np.asarray(cache.GetLocalToWorldTransform(prim), float).T


def motion_spec_snapshot(layer):
    """Hash exact authored root specs without flattening their composition."""
    result = {}
    for path in MOTION_ROOTS:
        spec = layer.GetPrimAtPath(path)
        if not spec:
            continue
        isolated = Sdf.Layer.CreateAnonymous('motion_spec.usda')
        Sdf.CreatePrimInLayer(isolated, Sdf.Path(path).GetParentPath())
        if not Sdf.CopySpec(layer, path, isolated, path):
            raise AssertionError('Cannot snapshot authored motion spec: ' + path)
        counts = dict(prim_specs=0, property_specs=0)

        def count(prim):
            counts['prim_specs'] += 1
            counts['property_specs'] += len(prim.properties)
            for child in prim.nameChildren:
                count(child)

        count(spec)
        result[path] = dict(sha256=hashlib.sha256(isolated.ExportToString().encode()).hexdigest(),
                           type_name=spec.typeName, specifier=str(spec.specifier), **counts)
    return result


def copy_motion_specs_in_root(source_layer, destination_layer):
    """Copy only authored motion roots; inherited farm geometry stays layered."""
    snapshot = motion_spec_snapshot(source_layer)
    if '/World/ElasticTarget' not in snapshot:
        raise ValueError('Direct-spec variant requires motion specs in the actual source root')
    with Sdf.ChangeBlock():
        for path in snapshot:
            Sdf.CreatePrimInLayer(destination_layer, Sdf.Path(path).GetParentPath())
            if not Sdf.CopySpec(source_layer, path, destination_layer, path):
                raise AssertionError('Cannot copy authored motion spec: ' + path)
    if motion_spec_snapshot(destination_layer) != snapshot:
        raise AssertionError('Copied authored motion specs differ from the source root')
    return snapshot


def copy_stage_metrics(source, destination):
    metrics = dict(meters_per_unit=UsdGeom.GetStageMetersPerUnit(source),
                   up_axis=str(UsdGeom.GetStageUpAxis(source)),
                   kilograms_per_unit=UsdPhysics.GetStageKilogramsPerUnit(source),
                   time_codes_per_second=source.GetTimeCodesPerSecond(),
                   frames_per_second=source.GetFramesPerSecond())
    UsdGeom.SetStageMetersPerUnit(destination, metrics['meters_per_unit'])
    UsdGeom.SetStageUpAxis(destination, metrics['up_axis'])
    UsdPhysics.SetStageKilogramsPerUnit(destination, metrics['kilograms_per_unit'])
    destination.SetTimeCodesPerSecond(metrics['time_codes_per_second'])
    destination.SetFramesPerSecond(metrics['frames_per_second'])
    return metrics


def enabled(prim):
    return (prim.HasAPI(UsdPhysics.CollisionAPI) and
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False)


def meshes(prim):
    return [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Mesh)]


def physical_snapshot(stage):
    """Preserve all joint/drive/limit/mass/filter fields and retained geometry."""
    result = {}
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if not any(path == root or path.startswith(root + '/') for root in PRESERVED):
            continue
        attrs = {a.GetName(): repr(a.Get()) for a in prim.GetAttributes()
                 if a.GetName().startswith(('physics:', 'physx', 'drive:', 'limit:', 'xformOp'))}
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            attrs['mesh_geometry_sha256'] = hashlib.sha256(
                np.asarray(mesh.GetPointsAttr().Get(), np.float32).tobytes() +
                np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), np.int32).tobytes() +
                np.asarray(mesh.GetFaceVertexCountsAttr().Get(), np.int32).tobytes()).hexdigest()
        result[path] = dict(type=prim.GetTypeName(), schemas=str(prim.GetMetadata('apiSchemas')),
            attrs=attrs, rels={r.GetName(): [str(t) for t in r.GetTargets()]
                              for r in prim.GetRelationships()})
    return result


def inventory(stage):
    bodies, plant_bodies, active_colliders, background, house, dofs = [], 0, 0, 0, 0, 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append(path)
            plant_bodies += path.startswith('/World/ElasticTarget/')
        if enabled(prim):
            active_colliders += path.startswith('/World/ElasticTarget/')
            background += path.startswith(PLANTS)
            house += path.startswith(HOUSE + '/')
        if prim.IsA(UsdPhysics.Joint) and path.startswith('/World/ElasticTarget/'):
            for axis in ('rotX', 'rotY', 'rotZ'):
                if UsdPhysics.DriveAPI(prim, axis):
                    low = prim.GetAttribute('limit:' + axis + ':physics:low').Get()
                    high = prim.GetAttribute('limit:' + axis + ':physics:high').Get()
                    if low is None or high is None or low <= high:
                        dofs += 1
    return dict(body_paths=sorted(bodies), body_count=len(bodies), plant_bodies=plant_bodies,
                active_colliders=active_colliders, background_plant_colliders=background,
                house_colliders=house, plant_rotational_dofs=dofs)


def _preflight(stage, metadata, arrays):
    active = metadata.get('active_plant', 22)
    groups = [g for g in metadata['groups'] if g['plant'] != active and g['mode'] == 'all']
    if not groups or len({g['path'] for g in groups}) != len(groups):
        raise ValueError('Expected unique background source-part roots')
    cache = UsdGeom.XformCache()
    representatives, instances, disabled_paths, prototype_keys = {}, [], [], {}
    for group in groups:
        path, key = group['path'], group['geometry']
        if not path.startswith(PLANTS) or '/Plant_%02d/' % active in path:
            raise ValueError('Only original background plant source parts may be replaced')
        part = stage.GetPrimAtPath(path)
        if not part or not part.IsActive():
            raise ValueError('Background source part is missing: ' + path)
        prototype = str(part.GetPrototype().GetPath()) if part.IsInstance() else None
        if prototype is None:
            raise ValueError('Expected an original shared background source-part instance: ' + path)
        if key in prototype_keys and prototype_keys[key] != prototype:
            raise ValueError('Cached geometry key is assigned to different source prototypes')
        prototype_keys[key] = prototype
        ancestor = part
        while ancestor and str(ancestor.GetPath()) != '/':
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError('Background source part belongs to a dynamic body: ' + path)
            ancestor = ancestor.GetParent()
        transform = world(cache, part)
        if np.linalg.det(transform[:3, :3]) <= 0:
            raise ValueError('Negative source-part determinant requires an explicit winding variant')
        original = meshes(part)
        info = metadata['geometry'][key]
        if len(original) != len(info['primitive_labels']):
            raise ValueError('Source mesh range count differs from cached geometry: ' + path)
        ranges = []
        for prim, label, (first, count) in zip(original, info['primitive_labels'], info['face_ranges']):
            mesh = UsdGeom.Mesh(prim)
            if not enabled(prim) or UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != 'none':
                raise ValueError('Source must be an enabled static triangle collider: ' + str(prim.GetPath()))
            if mesh.GetOrientationAttr().Get() != 'rightHanded':
                raise ValueError('Non-default source winding requires an explicit orientation variant')
            if mesh.GetDoubleSidedAttr().Get() is not True:
                raise ValueError('This source-part variant requires doubleSided source triangles')
            if len(mesh.GetFaceVertexCountsAttr().Get()) != count or not all(
                    n == 3 for n in mesh.GetFaceVertexCountsAttr().Get()):
                raise ValueError('Cached triangle range does not match source: ' + str(prim.GetPath()))
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            if material and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
                raise ValueError('This default-material variant cannot replace a bound physics material')
            extras = [a.GetName() for a in prim.GetAttributes() if a.HasAuthoredValueOpinion()
                      and a.GetName().startswith(('physics:', 'physx'))
                      and a.GetName() not in ('physics:approximation', 'physics:collisionEnabled')]
            if extras or any('filteredPairs' in r.GetName() for r in prim.GetRelationships()):
                raise ValueError('Source has additional collision policy: ' + str(prim.GetPath()))
            name = prim.GetCustomDataByKey('sourceName') or prim.GetName()
            if name != label:
                raise ValueError('Source triangle identity/order does not match cache')
            ranges.append(dict(source_collider=str(prim.GetPath()), source_name=name,
                               first_triangle=int(first), triangle_count=int(count)))
            disabled_paths.append(str(prim.GetPath()))
        if key not in representatives:
            v, f = arrays[key + '_v'], arrays[key + '_f']
            if hashlib.sha256(v.tobytes() + f.tobytes()).hexdigest() != info['sha256']:
                raise ValueError('Cached geometry hash mismatch: ' + key)
            source_v, source_f, offset = [], [], 0
            inv = np.linalg.inv(transform)
            for prim in original:
                mesh = UsdGeom.Mesh(prim)
                relative = inv @ world(cache, prim)
                if np.linalg.det(relative[:3, :3]) <= 0:
                    raise ValueError('Negative mesh determinant would change baked triangle winding')
                points = np.asarray(mesh.GetPointsAttr().Get(), float)
                source_v.append(points @ relative[:3, :3].T + relative[:3, 3])
                source_f.append(np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), np.int32).reshape(-1, 3) + offset)
                offset += len(points)
            source_v, source_f = np.concatenate(source_v), np.concatenate(source_f)
            if not np.array_equal(source_f, f) or not np.allclose(source_v, v, rtol=0, atol=1e-12):
                raise ValueError('Cached library does not represent the actual USD source triangles')
            storage = np.asarray(v, np.float32)
            if not np.array_equal(storage, np.asarray(source_v, np.float32)):
                raise ValueError('USD float32 points would alter source geometry')
            representatives[key] = dict(source_sha256=info['sha256'], vertices=len(v),
                triangles=len(f), source_coordinate_roundoff_m=float(np.max(abs(source_v-v))),
                float32_storage_roundoff_m=float(np.max(abs(storage.astype(float)-v))),
                triangle_ranges=[dict(relative_collider=r['source_collider'][len(path):],
                                      source_name=r['source_name'], first_triangle=r['first_triangle'],
                                      triangle_count=r['triangle_count']) for r in ranges])
        instances.append(dict(source_part=path, plant=group['plant'], part=group['part'],
                              geometry=key, world_matrix=transform.tolist(), triangle_sources=ranges))
    return representatives, instances, disabled_paths


def build(source, geometry, output, *, author_motion_specs_in_root=False):
    source, geometry, output = Path(source).resolve(), Path(geometry).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Refusing to overwrite an assembly: ' + str(output))
    source_scene = source / 'scene.usda'
    source_manifest = json.loads((source / 'motion_manifest.json').read_text())
    if source_manifest['scene_sha256'] != sha(source_scene):
        raise ValueError('Source scene hash does not match its motion manifest')
    source_stage = Usd.Stage.Open(str(source_scene))
    metadata = json.loads((geometry / 'geometry.json').read_text())
    arrays = np.load(geometry / 'geometry.npz')
    libraries, instances, old_colliders = _preflight(source_stage, metadata, arrays)
    before_inventory = inventory(source_stage)
    before_physics = physical_snapshot(source_stage)
    inputs = {Path(layer.realPath) for layer in source_stage.GetUsedLayers() if layer.realPath}
    inputs.update([source / 'motion_manifest.json', source / 'skin.npz',
                   geometry / 'geometry.json', geometry / 'geometry.npz'])
    input_hashes = {str(path): sha(path) for path in sorted(inputs) if path.is_file()}
    output.mkdir(parents=True)
    asset_dir = output / 'collision_libraries'; asset_dir.mkdir()
    for key in libraries:
        asset = Usd.Stage.CreateNew(str(asset_dir / (key + '.usdc')))
        root = UsdGeom.Xform.Define(asset, '/Library')
        asset.SetDefaultPrim(root.GetPrim())
        copy_stage_metrics(source_stage, asset)
        root.MakeInvisible()
        mesh = UsdGeom.Mesh.Define(asset, '/Library/Collider')
        v, f = arrays[key + '_v'], arrays[key + '_f']
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(v, np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(f), 3, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(f, np.int32).reshape(-1)))
        mesh.CreateOrientationAttr('rightHanded'); mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.asarray([v.min(0), v.max(0)], np.float32)))
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
        mesh.GetPrim().SetCustomDataByKey('sourceGeometrySHA256', libraries[key]['source_sha256'])
        asset.GetRootLayer().Save()
    stage = Usd.Stage.CreateNew(str(output / 'scene.usda'))
    stage.GetRootLayer().subLayerPaths = [str(source_scene)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    # Stage metrics live on layers, not prims; author them explicitly rather
    # than relying on metadata composition through the assembly sublayer.
    metrics = copy_stage_metrics(source_stage, stage)
    copied_specs = (copy_motion_specs_in_root(source_stage.GetRootLayer(), stage.GetRootLayer())
                    if author_motion_specs_in_root else {})
    with Sdf.ChangeBlock():
        for instance in instances:
            stage.OverridePrim(instance['source_part']).SetInstanceable(False)
    with Sdf.ChangeBlock():
        for path in old_colliders:
            prim = stage.GetPrimAtPath(path)
            if not prim or prim.IsInstanceProxy():
                raise ValueError('Static source collider did not become locally overridable: ' + path)
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    for instance in instances:
        path = instance['source_part'] + '/ConsolidatedCollision'
        prim = UsdGeom.Xform.Define(stage, path).GetPrim()
        prim.GetReferences().AddReference('collision_libraries/' + instance['geometry'] + '.usdc')
        prim.SetInstanceable(True)
        instance['collider'] = path + '/Collider'
    stage.GetRootLayer().Save()
    if copied_specs and motion_spec_snapshot(stage.GetRootLayer()) != copied_specs:
        raise AssertionError('Background consolidation changed copied motion root specs')
    after_inventory = inventory(stage)
    after_physics = physical_snapshot(stage)
    if before_physics != after_physics:
        raise AssertionError('Retained house/plant/robot/tool physical state changed')
    for key in ('body_paths', 'plant_bodies', 'active_colliders', 'house_colliders', 'plant_rotational_dofs'):
        if before_inventory[key] != after_inventory[key]:
            raise AssertionError('Preserved inventory changed: ' + key)
    if before_inventory['background_plant_colliders'] != len(old_colliders):
        raise AssertionError('Background replacement inventory is incomplete')
    if after_inventory['background_plant_colliders'] != len(instances):
        raise AssertionError('Consolidated collider count is incorrect')
    cache = UsdGeom.XformCache(); prototypes = set()
    for instance in instances:
        part = stage.GetPrimAtPath(instance['source_part'])
        if not np.array_equal(world(cache, part), instance['world_matrix']):
            raise AssertionError('Original source-part world transform changed')
        collider = stage.GetPrimAtPath(instance['collider'])
        prototypes.add(str(collider.GetPrimInPrototype().GetPath()))
        if not enabled(collider) or not collider.IsInstanceProxy():
            raise AssertionError('Replacement is not an enabled shared triangle instance')
    if len(prototypes) != len(libraries):
        raise AssertionError('Exact collision libraries do not share USD prototypes')
    if any(sha(Path(path)) != digest for path, digest in input_hashes.items()):
        raise AssertionError('An original input asset changed')
    shutil.copy2(source / 'skin.npz', output / 'skin.npz')
    audit = dict(schema='s300-exact-static-background-consolidation-v1', source_assembly=str(source),
        original_inputs_sha256=input_hashes, original_inputs_unchanged=True,
        source_scene_sha256=sha(source_scene), scene_sha256=sha(output / 'scene.usda'),
        before=before_inventory, after=after_inventory, unchanged_physical_snapshot=True,
        stage_metrics=metrics,
        author_motion_specs_in_root=bool(author_motion_specs_in_root),
        copied_motion_root_specs=copied_specs, motion_specs_copied_without_flattening=True if copied_specs else None,
        exact_source_triangle_indices=True, identical_source_float32_coordinates=True,
        source_part_world_transforms_exact=True, full_visual_geometry_retained=True,
        physics_material='unchanged PhysX default; no bound PhysicsMaterialAPI or custom shape policy',
        original_background_colliders_disabled_only_in_new_overlay=len(old_colliders),
        collision_libraries=libraries, source_part_instances=instances,
        usd_collision_prototypes=len(prototypes), native_cooked_prototype_reuse_unverified=True,
        training_eligible=False, harvest_success=None,
        contact_identity='merged background source-part collider; original triangle ranges retained for audit',
        limitations=['Native cooking, contact manifolds and face-index remapping are not certified by this offline audit',
                     'Unchanged unauthored contact/rest offsets may resolve differently because PhysX chooses contact offsets from shape extent',
                     'Same triangle surface union does not establish identical contact solver trajectories',
                     'No speedup or successful harvest is claimed before bounded native A/B validation'])
    (output / 'consolidation_audit.json').write_text(json.dumps(audit, indent=2) + '\n')
    manifest = source_manifest.copy()
    manifest.update(scene_sha256=audit['scene_sha256'], training_eligible=False, harvest_success=None,
        background_consolidation=dict(source_assembly=str(source), source_scene_sha256=audit['source_scene_sha256'],
            audit='consolidation_audit.json', exact_triangles=True, shape_identity_changed=True,
            source_collision_count=len(old_colliders), replacement_collision_count=len(instances),
            shared_geometry_libraries=len(libraries), native_equivalence_validated=False,
            author_motion_specs_in_root=bool(author_motion_specs_in_root)))
    (output / 'motion_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(assembly=str(output), before={k:v for k,v in before_inventory.items() if k!='body_paths'},
                         after={k:v for k,v in after_inventory.items() if k!='body_paths'},
                         shared_libraries=len(libraries), source_unchanged=True), indent=2))
    return audit


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--author-motion-specs-in-root', action='store_true',
                        help='Copy exact authored motion-root specs into the new root; no farm flattening')
    args = parser.parse_args()
    build(args.source, args.geometry, args.output,
          author_motion_specs_in_root=args.author_motion_specs_in_root)
