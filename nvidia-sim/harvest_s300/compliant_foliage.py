"""Opt-in compliant petioles using the existing leaf meshes and contact policy.

One passive 3-axis joint per compound leaf; individual leaflets remain rigid.
This is an uncalibrated reduced model, not FEM or measured tomato stiffness.
"""
import argparse
import json
import re
import shutil
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_assets import joint, rigid, sha, physx
from motion_stage_metrics import copy_stage_metrics, require_motion_units


def canonical_leaf_group(name):
    """Normalize only the actual GLB export's six-hex source suffix.

    The alternate original stem exports four material/submesh names such as
    CompoundLeaf_01_214888 for the same compound leaf. Its owning stem and
    authored geometry determine grouping; the suffix is not another leaf.
    Existing unsuffixed V9/apical vein/midrib names retain their exact behavior.
    """
    match=re.fullmatch(r'((?:V9_Additional_)?CompoundLeaf_\d+|Apical_Young_Leaf_\d+)(?:_[0-9a-fA-F]{6})?(?:_(?:Vein|Midrib).*)?',name)
    return match.group(1) if match else None


def leaf_groups(stage):
    groups = {}
    for p in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget')):
        name = p.GetCustomDataByKey('sourceName') or ''
        group=canonical_leaf_group(name)
        if p.IsA(UsdGeom.Mesh) and group:
            groups.setdefault(group, []).append(str(p.GetPath()))
    return groups


def world_vertices(stage, path):
    p = stage.GetPrimAtPath(path)
    v = np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(), float)
    t = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(p))
    return v @ t[:3, :3] + t[3, :3]


def enabled_colliders(stage):
    return {str(p.GetPath()) for p in stage.Traverse()
            if p.HasAPI(UsdPhysics.CollisionAPI) and
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False}


def armature_values(manifest, dof_names, default=.0005):
    """Exact new leaf DOFs only; all pre-existing plant DOFs keep their value."""
    report = manifest.get('compliant_foliage', {})
    names = {Sdf.Path(x['joint']).name for x in report.get('leaves', [])}
    value = report.get('numerical_armature_kg_m2', default)
    return np.array([value if n.split(':')[0] in names else default for n in dof_names])


def convert(stage, names=None, stiffness=.005, damping=.0008, armature=.000005):
    if not all(np.isfinite(x) and x > 0 for x in (stiffness, damping)):
        raise ValueError('Positive finite stiffness and damping required')
    if not np.isfinite(armature) or armature < 0: raise ValueError('Invalid leaf armature')
    if stage.GetPrimAtPath('/World/ElasticTarget/Leaf_CompoundLeaf_01'):
        raise ValueError('Already converted foliage')
    groups = leaf_groups(stage)
    selected = sorted(groups if names is None else names)
    if not selected or any(n not in groups for n in selected):
        raise ValueError('No matching compound leaves')
    # Snapshot before authoring: remap the original exclusion graph, never
    # introduce broad no-contact policies for leaves versus the robot/fruit.
    old_colliders = enabled_colliders(stage)
    relationships = [(str(p.GetPath()), r.GetName(), list(r.GetTargets()))
                     for p in stage.Traverse() for r in p.GetRelationships()
                     if r.GetName() == 'physics:filteredPairs']
    flat = stage.Flatten()
    mapping = {}; rows = []; rest_error = 0.
    for name in selected:
        visuals = groups[name]
        source_names=sorted({stage.GetPrimAtPath(path).GetCustomDataByKey('sourceName') for path in visuals})
        parents = {str(Sdf.Path(p).GetParentPath()) for p in visuals}
        if len(parents) != 1:
            raise ValueError('Compound leaf spans multiple source bodies: ' + name)
        parent = parents.pop()
        pp = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(parent))).T
        world = np.concatenate([world_vertices(stage, p) for p in visuals])
        # Geometric attachment: closest authored leaf point to the owning
        # stem capsule centreline, not the compound leaf centroid.
        cap = UsdGeom.Capsule(stage.GetPrimAtPath(parent + '/Collider'))
        if not cap: raise ValueError('Missing owning stem capsule')
        ct = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(cap.GetPrim())).T
        axis = ct[:3, 2]; h = cap.GetHeightAttr().Get() / 2
        axial = np.clip((world - ct[:3, 3]) @ axis, -h, h)
        nearest = ct[:3, 3] + axial[:, None] * axis
        pivot = world[np.linalg.norm(world - nearest, axis=1).argmin()]
        center = (world.max(0) + world.min(0)) / 2
        size = np.maximum(np.ptp(world, axis=0), .002)
        mass = max(.0002, .002 * (max(size) / .2) ** 2)
        inertia = mass / 12 * (np.sum(size**2) - size**2)
        body = '/World/ElasticTarget/Leaf_' + name
        pose = np.eye(4); pose[:3, 3] = center
        rigid(stage, body, pose, mass, inertia)
        jid = '/World/ElasticTarget/Joints/Leaf_' + name
        j = joint(stage, jid, parent, body, pp, pose, pivot, k=stiffness, d=damping)
        physx(j.GetPrim(), 'PhysxJointAPI', **{'physxJoint:armature': (Sdf.ValueTypeNames.Float, float(armature))})
        for axis_name in ('rotX', 'rotY', 'rotZ'):
            limit = UsdPhysics.LimitAPI(j.GetPrim(), axis_name)
            limit.CreateLowAttr(-85.); limit.CreateHighAttr(85.)
        paths = list(visuals)
        for vpath in visuals:
            number = Sdf.Path(vpath).name.split('_')[-1]
            paths += [str(p.GetPath()) for p in stage.GetPrimAtPath(parent).GetChildren()
                      if p.GetName().startswith('Appendage_' + number + '_')]
        # Copy composed prim specs so this also works with layered assemblies.
        for old in paths:
            new = body + '/' + Sdf.Path(old).name
            before = world_vertices(stage, old)
            oldtf = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(old))).T
            Sdf.CopySpec(flat, old, stage.GetRootLayer(), new)
            UsdGeom.Xformable(stage.GetPrimAtPath(new)).MakeMatrixXform().Set(
                Gf.Matrix4d((np.linalg.inv(pose) @ oldtf).T.tolist()))
            stage.OverridePrim(old).SetActive(False)
            mapping[old] = new
            rest_error = max(rest_error, float(np.max(np.abs(world_vertices(stage, new) - before))))
        row = dict(name='Leaf_' + name, source_name=name, source_names=source_names,path=body, parent=parent,
                   joint=jid, rest_position=center.tolist(), pivot=pivot.tolist(), mass=mass,
                   stiffness=stiffness, damping=damping, inertia=inertia.tolist(),
                   meshes=[mapping[x] for x in paths])
        rows.append(row)
    for old, relname, targets in relationships:
        p = stage.GetPrimAtPath(mapping.get(old, old))
        if p and p.IsActive():
            p.CreateRelationship(relname).SetTargets([mapping.get(str(x), str(x)) for x in targets])
    # Inherit the owning body's effective filters (including incoming pairs).
    # Otherwise splitting the leaf accidentally re-enables the authored
    # petiole/stem junction overlap which was excluded in the source model.
    for row in rows:
        parent = row['parent']; excluded = {parent}
        # Two leaves previously owned by the same body could not collide.
        # Preserve that relation when both move to separate leaf bodies.
        excluded.update(other['path'] for other in rows
                        if other['parent'] == parent and other['path'] != row['path'])
        for old, _, targets in relationships:
            if old == parent: excluded.update(map(str, targets))
            if parent in list(map(str, targets)): excluded.add(old)
        rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(row['path'])).CreateFilteredPairsRel()
        rel.SetTargets([mapping.get(x, x) for x in sorted(excluded)])
        row['inherited_source_body_exclusions'] = sorted(excluded)
    expected = {mapping.get(x, x) for x in old_colliders}
    if enabled_colliders(stage) != expected:
        raise ValueError('Collider coverage changed')
    if rest_error > 1e-9: raise ValueError('Rest geometry transform changed')
    return dict(kind='compound_leaf_passive_petiole_v1', leaves=rows, path_mapping=mapping,
                added_rotational_dofs=3*len(rows), added_mass_kg=sum(r['mass'] for r in rows),
                initial_geometry_max_error_m=rest_error, collider_coverage_preserved=True,
                numerical_armature_kg_m2=armature, calibrated=False, training_eligible=False,
                source_name_normalization=dict(schema='compound-leaf-source-name-v2',
                    rule='Only actual six-hex GLB source suffixes are normalized; original mesh names/points/triangles are preserved',
                    detected_group_count=len(groups),converted_group_count=len(rows),
                    mapping={source_name:row['source_name'] for row in rows for source_name in row['source_names']}),
                limitations='Leaflets remain rigid; no petiole continuum bending; existing leaf/leaf exclusions retained')


def revise(assembly, output, names=None, stiffness=.005, damping=.0008, armature=.000005):
    assembly = Path(assembly).resolve(); output = Path(output).resolve()
    manifest = json.loads((assembly/'motion_manifest.json').read_text())
    source = assembly/'scene.usda'
    if sha(source) != manifest['scene_sha256']: raise ValueError('Source hash mismatch')
    if 'compliant_foliage' in manifest: raise ValueError('Foliage already converted')
    output.mkdir(parents=True, exist_ok=False)
    stage = Usd.Stage.CreateNew(str(output/'scene.usda'))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    original = Usd.Stage.Open(str(source))
    copy_stage_metrics(original, stage)
    require_motion_units(stage)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    report = convert(stage, names, stiffness, damping, armature)
    # Native server A/B: reparented articulated leaves in layered overlays
    # jump on the second step. Materialize composed values before opening
    # in PhysX; no shape, spring, force limit or contact tolerance changes.
    composed = stage.Flatten()
    stage = None
    composed.Export(str(output/'scene.usda'))
    report['materialized'] = True
    if sha(source) != manifest['scene_sha256']: raise ValueError('Source mutated')
    report.update(source_assembly=str(assembly), source_sha256=sha(source))
    manifest['compliant_foliage'] = report
    manifest['plant']['bodies'].extend(report['leaves'])
    manifest['scene_sha256'] = sha(output/'scene.usda')
    manifest.update(training_eligible=False, harvest_success=None)
    (output/'motion_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    shutil.copy2(assembly/'skin.npz', output/'skin.npz')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('assembly', type=Path); p.add_argument('output', type=Path)
    p.add_argument('--leaves', nargs='+'); p.add_argument('--stiffness', type=float, default=.005)
    p.add_argument('--damping', type=float, default=.0008)
    p.add_argument('--armature', type=float, default=.000005, help='Additional inertia on new leaf joints only, kg m2')
    a = p.parse_args()
    print(json.dumps(revise(a.assembly, a.output, a.leaves, a.stiffness, a.damping, a.armature), indent=2))
