"""Read-only, opt-in native body and articulation compilation evidence.

USD conservation does not prove that PhysX compiled the same mass frames or
joint configuration. Collect after native initialization and before commanded
motion. This module never initializes/steps physics, edits USD, or writes forces.
Native scalar body queries are intended for CPU diagnostic runs.
"""
from __future__ import annotations

import hashlib
import json

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from motion_replication import remap_path


ARTICULATION_QUERIES = (
    'get_dof_positions', 'get_dof_velocities', 'get_dof_position_targets',
    'get_dof_velocity_targets', 'get_dof_actuation_forces',
    'get_dof_types', 'get_dof_motions', 'get_dof_limits',
    'get_dof_stiffnesses', 'get_dof_dampings', 'get_dof_armatures',
    'get_dof_max_forces', 'get_dof_max_velocities', 'get_dof_friction_coefficients',
    'get_link_transforms', 'get_link_velocities', 'get_masses', 'get_coms', 'get_inertias',
)


def _array(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    elif hasattr(value, 'numpy'):
        value = value.numpy()
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('Native query contains NaN/Inf')
    return result


def _body_owner(prim):
    current = prim
    while current and not current.IsPseudoRoot():
        if current.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(current.GetPath())
        current = current.GetParent()
    return None


def collect_compiled_body_audit(stage, body_paths, *, path_mapping=None,
                                query_interface=None, pose_interface=None,
                                articulation_views=None, device='cpu'):
    """Collect native mass/COM/inertia, materials, poses and optional DOF data.

    ``path_mapping`` maps each clone dynamic root back to its original root.
    Pass the actual rigid-body view's paths, not guessed source body names.
    API errors and missing fields remain explicit incomplete evidence.
    """
    if str(device).startswith('cuda'):
        raise ValueError('Compiled scalar-body audit is restricted to CPU diagnostics')
    if query_interface is None:
        from omni.physx import get_physx_interface, get_physxunittests_interface
        query_interface = get_physxunittests_interface()
        if pose_interface is None:
            pose_interface = get_physx_interface()
    mapping = path_mapping or {}
    canonical = lambda path: remap_path(path, mapping)
    root_before = stage.GetRootLayer().ExportToString()
    session_before = stage.GetSessionLayer().ExportToString()
    cache = UsdGeom.XformCache()
    body_paths = list(map(str, body_paths))
    rows = {}
    errors = []
    for path in body_paths:
        key = canonical(path)
        if key in rows:
            raise ValueError(f'Duplicate canonical native body: {key}')
        prim = stage.GetPrimAtPath(path)
        row = dict(path=path, canonical_path=key, native_mass_information=None,
                   native_pose=None, colliders=[])
        rows[key] = row
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            errors.append(dict(path=path, query='body_identity', error='USD rigid body missing'))
            continue
        row.update(parent_path=canonical(prim.GetParent().GetPath()),
                   usd_world_matrix=_array(cache.GetLocalToWorldTransform(prim)).tolist(),
                   usd_local_matrix=_array(UsdGeom.Xformable(prim).GetLocalTransformation()).tolist(),
                   usd_reset_xform_stack=UsdGeom.Xformable(prim).GetResetXformStack(),
                   usd_mass_attributes={name: _array(prim.GetAttribute(name).Get()).tolist()
                       for name in ('physics:mass', 'physics:density', 'physics:centerOfMass',
                                    'physics:diagonalInertia')
                       if prim.GetAttribute(name) and prim.GetAttribute(name).HasAuthoredValueOpinion()})
        current = prim
        row['usd_articulation_root'] = None
        while current and not current.IsPseudoRoot():
            if current.HasAPI(UsdPhysics.ArticulationRootAPI):
                row['usd_articulation_root'] = canonical(current.GetPath())
                break
            current = current.GetParent()
        try:
            native = query_interface.get_mass_information(path)
            missing = {'mass', 'inertia', 'com'} - native.keys()
            if missing:
                raise ValueError('Mass query missing fields: ' + ','.join(sorted(missing)))
            mass = _array(native['mass'])
            inertia = _array(native['inertia'])
            com = _array(native['com'])
            if mass.size != 1 or inertia.shape != (3,) or com.shape != (3,):
                raise ValueError('Native mass/COM/inertia shape is unexpected')
            if float(mass) < 0 or np.any(inertia < 0):
                raise ValueError('Native mass/inertia is negative')
            row['native_mass_information'] = dict(mass=float(mass), inertia=inertia.tolist(), com=com.tolist())
        except Exception as exc:
            errors.append(dict(path=path, query='native_mass_information', error=str(exc)))
        if pose_interface is not None:
            try:
                pose = pose_interface.get_rigidbody_transformation(path)
                if not pose.get('ret_val'):
                    raise ValueError('Native global body pose was not found')
                position, rotation = _array(pose['position']), _array(pose['rotation'])
                if position.shape != (3,) or rotation.shape != (4,) or np.linalg.norm(rotation) == 0:
                    raise ValueError('Native position/quaternion shape is unexpected')
                row['native_pose'] = dict(position=position.tolist(), rotation_xyzw=rotation.tolist())
            except Exception as exc:
                errors.append(dict(path=path, query='native_pose', error=str(exc)))
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or \
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        owner = _body_owner(prim)
        if owner not in body_paths:
            continue
        path = str(prim.GetPath())
        collider = dict(path=path, canonical_path=canonical(path), native_materials=None,
                        authored_material_relationships={rel.GetName(): [canonical(p) for p in rel.GetTargets()]
                            for rel in prim.GetRelationships() if rel.GetName().startswith('material:binding')})
        try:
            materials = query_interface.get_materials_paths(path)
            if materials is None:
                raise ValueError('Native material query returned no result')
            collider['native_materials'] = sorted(canonical(p) for p in materials)
        except Exception as exc:
            errors.append(dict(path=path, query='native_materials', error=str(exc)))
        rows[canonical(owner)]['colliders'].append(collider)
    articulation_rows = {}
    for role, view in (articulation_views or {}).items():
        entry = dict(count=int(view.count), dof_names=list(view.shared_metatype.dof_names),
                     link_names=list(view.shared_metatype.link_names), fields={}, unavailable=[])
        for name in ARTICULATION_QUERIES:
            method = getattr(view, name, None)
            if method is None:
                entry['unavailable'].append(name)
                continue
            try:
                entry['fields'][name] = _array(method()).tolist()
            except Exception as exc:
                errors.append(dict(path=role, query=name, error=str(exc)))
        articulation_rows[role] = entry
    if stage.GetRootLayer().ExportToString() != root_before or \
            stage.GetSessionLayer().ExportToString() != session_before:
        raise RuntimeError('Read-only compiled audit modified USD')
    # Native hash excludes differing actual namespace strings and mutable
    # initialization poses; original-vs-clone deltas remain in full records.
    physics_rows = {key: dict(native_mass_information=row['native_mass_information'],
                             materials={c['canonical_path']: c['native_materials'] for c in row['colliders']})
                    for key, row in rows.items()}
    return dict(schema='compiled_body_audit_v1', complete=not errors, device=str(device),
                bodies=rows, articulations=articulation_rows, errors=errors,
                native_mass_material_sha256=hashlib.sha256(json.dumps(
                    physics_rows, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                queries=dict(mass='PhysXUnitTests.get_mass_information',
                             material='PhysXUnitTests.get_materials_paths',
                             pose='IPhysx.get_rigidbody_transformation' if pose_interface else 'not requested'),
                units=dict(mass='kg', inertia='kg*m^2 (native returned diagonal)', com='m in native body mass frame',
                           pose='global position m; quaternion x,y,z,w'),
                limitations=['No direct native joint-constraint frame query',
                             'Native principal inertia axes are unavailable in this mass helper',
                             'Read-only compilation evidence does not validate motion dynamics'],
                physics_steps_added=0, source_assets_saved=False, training_eligible=False)


def compare_compiled_body_audits(reference, actual):
    """Report numeric/identity differences without inventing a universal gate."""
    before, after = reference['bodies'], actual['bodies']
    differences = []
    for key in sorted(before.keys() & after.keys()):
        left, right = before[key], after[key]
        row = dict(body=key)
        if left['native_mass_information'] is not None and right['native_mass_information'] is not None:
            for field in ('mass', 'inertia', 'com'):
                row[field + '_maximum_absolute_difference'] = float(np.max(np.abs(
                    np.asarray(left['native_mass_information'][field]) - np.asarray(right['native_mass_information'][field]))))
        else:
            row['native_mass_incomplete'] = True
        materials = lambda body: {c['canonical_path']: c['native_materials'] for c in body['colliders']}
        row['native_material_identity_match'] = materials(left) == materials(right)
        if left.get('native_pose') and right.get('native_pose'):
            row['native_position_error_m'] = float(np.linalg.norm(np.asarray(left['native_pose']['position']) -
                                                                   np.asarray(right['native_pose']['position'])))
            q0 = np.asarray(left['native_pose']['rotation_xyzw']); q1 = np.asarray(right['native_pose']['rotation_xyzw'])
            cosine = abs(np.dot(q0, q1) / (np.linalg.norm(q0) * np.linalg.norm(q1)))
            row['native_rotation_error_deg'] = float(np.degrees(2*np.arccos(np.clip(cosine, 0., 1.))))
        differences.append(row)
    articulation_differences = {}
    for role in reference.get('articulations', {}).keys() & actual.get('articulations', {}).keys():
        left, right = reference['articulations'][role], actual['articulations'][role]
        entry = dict(dof_names_match=left['dof_names'] == right['dof_names'],
                     link_names_match=left['link_names'] == right['link_names'], fields={})
        for field in left['fields'].keys() & right['fields'].keys():
            a, b = np.asarray(left['fields'][field]), np.asarray(right['fields'][field])
            entry['fields'][field] = dict(shape_match=a.shape == b.shape,
                maximum_absolute_difference=float(np.max(np.abs(a-b))) if a.shape == b.shape and a.size else None)
        articulation_differences[role] = entry
    return dict(complete=reference['complete'] and actual['complete'],
                missing_bodies=sorted(before.keys()-after.keys()), extra_bodies=sorted(after.keys()-before.keys()),
                native_mass_material_hash_match=reference['native_mass_material_sha256'] == actual['native_mass_material_sha256'],
                body_differences=differences, articulation_differences=articulation_differences,
                interpretation='Diagnostic A-B measurements; no new physical-validity tolerance or pass gate')
