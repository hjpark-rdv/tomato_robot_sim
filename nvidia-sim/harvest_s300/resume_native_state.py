"""Resume a completed native cycle on the exact same immutable assembly.

This restores measured articulation state and standalone body state. It does
not restore PhysX contact/solver caches, invent zero velocities, replay native
break callbacks, or claim an uninterrupted engine instance.
"""
import copy
import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdPhysics
from scipy.spatial.transform import Rotation

from compact_motion_assets import file_sha256, verify_compact_dependencies
from native_route_checkpoint import load_checkpoint


def _array(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    if hasattr(value, 'numpy'):
        return value.numpy()
    return np.asarray(value)


def require_completed_checkpoint(metadata):
    """Reject a checkpoint saved at a stopped/invalid manipulation state."""
    sequence = metadata.get('sequence')
    if not isinstance(sequence, dict):
        raise ValueError('Resume requires a declared completed-cycle checkpoint')
    if sequence.get('checkpoint_physics_valid') is False:
        raise ValueError('Resume checkpoint was captured after physics invalidity')
    if sequence.get('checkpoint_capture_reason') != 'completed_cycle':
        raise ValueError('Resume requires an explicit completed_cycle capture reason')
    if sequence.get('checkpoint_physics_valid') is not True:
        raise ValueError('Resume requires recorded checkpoint physics validity')
    cycles = sequence.get('cycle_plans', [])
    if not cycles or cycles[-1].get('status') != 'motion_completed':
        raise ValueError('Last checkpoint cycle did not complete')
    record = cycles[-1]
    if record.get('stop_reason') is not None:
        raise ValueError('Cannot resume an invalid or stopped cycle')
    end = record.get('end_s')
    if not isinstance(end, (float, int)) or not np.isfinite(end) or abs(end-metadata['time_s']) > 1e-8:
        raise ValueError('Checkpoint is not at the declared completed-cycle timestamp')


def imported_break_markers(metadata):
    """Local t=0 inventory markers retain, but never replay, prior callbacks."""
    result = []
    for event in metadata['break_events']:
        marker = copy.deepcopy(event)
        marker.update(t=0., type='imported_native_prior', original_t=event['t'],
                      source_scene_sha256=metadata['scene_sha256'],
                      source_checkpoint_time_s=metadata['time_s'],
                      original_event=copy.deepcopy(event), native_new_event=False)
        result.append(marker)
    return result


def _leaf_revision_proof(assembly, manifest, metadata):
    revision = manifest.get('leaf_collision_geometry', {})
    if (revision.get('enabled') is not True or
            revision.get('source_assembly') != metadata['source_assembly'] or
            revision.get('source_scene_sha256') != metadata['scene_sha256'] or
            revision.get('approximation') != 'convexDecomposition' or
            revision.get('authored_triangles_retained') is not True or
            revision.get('non_approximation_properties_preserved') is not True):
        raise ValueError('Resume leaf proxy revision has no exact original checkpoint parent proof')
    files = revision.get('source_file_sha256', {})
    if not files or files.get(metadata['source_scene']) != metadata['scene_sha256']:
        raise ValueError('Resume leaf revision lacks exact source file SHA inventory')
    for path, digest in files.items():
        if not Path(path).is_absolute() or file_sha256(path) != digest:
            raise ValueError('Resume leaf revision parent asset SHA mismatch')
    geometry = revision.get('geometry_audit', {})
    if not geometry or any(geometry.get(key) != 0. for key in
            ('world_vertex_max_abs_error_m', 'rigid_pose_max_abs_error', 'collider_pose_max_abs_error')):
        raise ValueError('Resume leaf revision lacks exact authored geometry/pose comparison')
    left, right = copy.deepcopy(metadata['manifest']), copy.deepcopy(manifest)
    for obj in (left, right):
        for key in ('scene_sha256', 'compact_assets', 'leaf_collision_geometry'):
            obj.pop(key, None)
        if obj.get('training_eligible') is False:
            obj.pop('training_eligible')
        if 'harvest_success' in obj and obj['harvest_success'] is None:
            obj.pop('harvest_success')
    if left != right:
        raise ValueError('Resume leaf revision changed state identity or properties beyond its approved proxy provenance')
    selected = revision.get('collider_changes', [])
    if not selected or any(row.get('before') != 'convexHull' or row.get('after') != 'convexDecomposition' for row in selected):
        raise ValueError('Resume leaf proxy revision changed an unsupported collider representation')
    return dict(kind='explicit_leaf_convex_decomposition_revision',
                original_assembly=metadata['source_assembly'],
                original_scene_sha256=metadata['scene_sha256'],
                revised_assembly=str(assembly), revised_scene_sha256=manifest['scene_sha256'],
                collider_changes=copy.deepcopy(selected),
                authored_geometry_unchanged=True, native_proxy_validation='pending')


def prepare_resume(stage, assembly, manifest, checkpoint, output_json=None, *,
                   allow_leaf_proxy_revision=False, allow_accuracy_proxy_revision=False,
                   allow_attachment_transfer=False,allow_conditioned_start=False):
    """Validate state and disable only proven fruit attachments in SESSION USD.

    Invoke before native physics compilation. The source scene, composed USDC,
    authored collider/filter/material values and unbroken joints remain intact.
    """
    if allow_leaf_proxy_revision and allow_accuracy_proxy_revision:
        raise ValueError('Choose one explicit proxy revision proof; leaf authorization does not authorize tool cooking changes')
    if allow_attachment_transfer and (allow_leaf_proxy_revision or allow_accuracy_proxy_revision):
        raise ValueError('Choose one explicit revision proof; proxy and attachment transfer authorizations cannot be combined')
    if allow_conditioned_start and (allow_leaf_proxy_revision or allow_accuracy_proxy_revision or allow_attachment_transfer):
        raise ValueError('Canonical conditioned start requires the exact same assembly; model revision authorization cannot be combined')
    assembly, checkpoint = Path(assembly).resolve(), Path(checkpoint).resolve()
    scene_sha = file_sha256(assembly/'scene.usda')
    if scene_sha != manifest.get('scene_sha256'):
        raise ValueError('Resume immutable source scene SHA mismatch')
    verify_compact_dependencies(assembly, manifest)
    metadata, arrays = load_checkpoint(checkpoint)
    if metadata.get('sequence',{}).get('terminal_snapshot_publication') is not None:
        from terminal_cycle_checkpoint import verify_publication
        verify_publication(metadata,arrays,checkpoint)
    from robot_controller_state import resolve_robot_controller_state
    controller_state=resolve_robot_controller_state(metadata,arrays,checkpoint)
    revision_proof = None
    if metadata['scene_sha256'] != scene_sha or Path(metadata['source_assembly']).resolve() != assembly:
        if allow_attachment_transfer:
            from eligible_attachment_transfer import validate_resume_revision
            revision_proof = validate_resume_revision(assembly, manifest, metadata)
        elif allow_accuracy_proxy_revision:
            from collision_proxy_accuracy import validate_resume_revision
            revision_proof = validate_resume_revision(assembly, manifest, metadata)
        elif allow_leaf_proxy_revision:
            revision_proof = _leaf_revision_proof(assembly, manifest, metadata)
        else:
            raise ValueError('Resume requires the exact same source assembly/SHA')
    elif metadata['manifest'] != manifest:
        raise ValueError('Resume checkpoint/current manifest mismatch')
    if metadata['source_plant'] != manifest['plant']['source_plant']:
        raise ValueError('Resume active plant mismatch')
    conditioned_start_proof=None
    if allow_conditioned_start:
        from conditioned_start_checkpoint import require_conditioned_checkpoint
        conditioned_start_proof=require_conditioned_checkpoint(metadata,arrays,checkpoint)
    else:require_completed_checkpoint(metadata)
    fruits = {f['joint']: f for f in manifest['plant']['fruits']}
    broken = sorted({e['joint'] for e in metadata['break_events']})
    # Validate all mappings before the first session-layer mutation.
    joints = []
    for path in broken:
        if path not in fruits:
            raise ValueError('Resume break is not a current active-plant fruit attachment: '+path)
        fruit = fruits[path]
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsA(UsdPhysics.Joint):
            raise ValueError('Resume broken native joint absent from exact scene: '+path)
        joint = UsdPhysics.Joint(prim)
        if ([str(p) for p in joint.GetBody0Rel().GetTargets()] != [fruit['anchor']] or
                [str(p) for p in joint.GetBody1Rel().GetTargets()] != [fruit['path']]):
            raise ValueError('Resume fruit attachment body mapping mismatch: '+path)
        joints.append(joint)
    standalone = {f['path'] for f in manifest['plant']['fruits']}
    carrier = manifest.get('rail', {}).get('carrier_body')
    if carrier:
        standalone.add(carrier)
    from background_foliage import declared_bodies
    standalone.update(declared_bodies(stage,manifest))
    for old in manifest.get('route_checkpoint', {}).get('outgoing_history', []):
        standalone.update(f['path'] for f in old.get('detached_fruits', []))
    paths = set(map(str, arrays['body_paths']))
    if not standalone <= paths:
        raise ValueError('Resume standalone native body state is missing')
    for path in standalone:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Resume standalone body is not an exact native rigid body: '+path)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for joint in joints:
            joint.CreateJointEnabledAttr(False)
    report = dict(schema='s300-same-assembly-native-resume-v1',
        source_assembly=str(assembly), checkpoint=str(checkpoint),
        source_scene_sha256=scene_sha, checkpoint_source_scene_sha256=metadata['scene_sha256'],
        explicit_proxy_revision=revision_proof, checkpoint_time_s=metadata['time_s'],
        checkpoint_metadata_sha256=metadata['metadata_sha256'],
        checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],
        source_plant=metadata['source_plant'], disabled_native_broken_joints=broken,
        initial_native_break_count=len(broken),
        original_native_break_events=copy.deepcopy(metadata['break_events']),
        imported_native_prior=imported_break_markers(metadata),
        standalone_body_paths=sorted(standalone),
        original_sequence=copy.deepcopy(metadata['sequence']),
        source_asset_modified=False, native_initialization_validation='pending',
        checkpoint_kind='completed_cycle' if conditioned_start_proof is None else 'canonical_conditioned_start',
        canonical_conditioned_start=conditioned_start_proof,
        controller_state=controller_state['provenance'],
        restart='New PhysX engine instance; measured state restore, without solver/contact warm-start cache continuation',
        training_eligible=False, harvest_success=None)
    if output_json is not None:
        with Path(output_json).open('x') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    return dict(metadata=metadata, arrays=arrays, report=report,controller_state=controller_state,
        checkpoint_kind=report['checkpoint_kind'],conditioned_start_proof=conditioned_start_proof)


def _native_mapping(context, robot, plant, bodies):
    arrays = context['arrays']
    for kind, view in (('robot', robot), ('plant', plant)):
        if view.count != 1:
            raise ValueError('Native checkpoint resume requires one environment')
        names = list(map(str, view.shared_metatype.dof_names))
        if names != list(map(str, arrays[kind+'_dof_names'])):
            raise ValueError('Resume native '+kind+' DOF name/order mismatch')
        if view.max_dofs != len(names):
            raise ValueError('Resume native '+kind+' DOF shape mismatch')
    paths = list(map(str, bodies.prim_paths))
    if len(paths) != len(set(paths)) or set(paths) != set(map(str, arrays['body_paths'])):
        raise ValueError('Resume native rigid-body identity coverage mismatch')
    return {path: i for i, path in enumerate(paths)}


def apply_resume(context, robot, plant, bodies, *, to_native=lambda a: a,
                 indices_to_native=lambda a: a, environment_indices=None,
                 refresh_articulations=None):
    """Set exact measured q/qvel, fixed preload and standalone body state.

    Articulated link poses are not teleported: restoring DOFs must reproduce
    their measured transforms and velocities, which verify_resume reports.
    All native setters receive full view-shaped arrays, even for sparse rows.
    """
    index = _native_mapping(context, robot, plant, bodies)
    arrays = context['arrays']
    ids = indices_to_native(np.array([0], np.uint32)) if environment_indices is None else environment_indices
    for kind, view in (('robot', robot), ('plant', plant)):
        positions = arrays[kind+'_q'][None].copy()
        velocities = arrays[kind+'_qvel'][None].copy()
        view.set_dof_positions(to_native(positions), ids)
        view.set_dof_velocities(to_native(velocities), ids)
    from robot_controller_state import restore_controller_targets
    controller_report=restore_controller_targets(robot,context['controller_state'],ids,
        to_native=to_native,fallback_position=arrays['robot_q'])
    preload = arrays['preload'][None].copy()
    plant.set_dof_actuation_forces(to_native(preload), ids)
    if refresh_articulations is not None:
        # Native SimulationView.update_articulations_kinematic refreshes FK
        # without stepping physics; GPU link state otherwise can remain stale.
        refresh_articulations()
    source = {str(path): i for i, path in enumerate(arrays['body_paths'])}
    selected = context['report']['standalone_body_paths']
    poses = _array(bodies.get_transforms()).copy()
    velocities = _array(bodies.get_velocities()).copy()
    if poses.shape != (len(index), 7) or velocities.shape != (len(index), 6):
        raise ValueError('Resume native body state array shape mismatch')
    for path in selected:
        poses[index[path]] = arrays['body_poses_world'][source[path]]
        velocities[index[path]] = arrays['body_velocities_world'][source[path]]
    body_ids = indices_to_native(np.array([index[path] for path in selected], np.uint32))
    if selected:
        bodies.set_transforms(to_native(poses), body_ids)
        bodies.set_velocities(to_native(velocities), body_ids)
    report = verify_resume(context, robot, plant, bodies)
    report.update(restored_standalone_body_count=len(selected),
                  restored_controller_state=controller_report,
                  restored_preload=arrays['preload'].tolist(),
                  native_articulation_kinematics_refreshed=refresh_articulations is not None,
                  articulated_link_transforms_set_directly=False,
                  exact_native_mapping=True, bitwise_engine_continuity=False)
    return report


def verify_resume(context, robot, plant, bodies):
    """Compare every measured body and DOF; report numeric errors separately."""
    index = _native_mapping(context, robot, plant, bodies)
    arrays = context['arrays']
    order = [index[str(path)] for path in arrays['body_paths']]
    poses = _array(bodies.get_transforms())[order]
    velocities = _array(bodies.get_velocities())[order]
    if poses.shape != arrays['body_poses_world'].shape or velocities.shape != arrays['body_velocities_world'].shape:
        raise ValueError('Resume native comparison shape mismatch')
    expected = arrays['body_poses_world']
    if not np.isfinite(np.r_[poses.ravel(), velocities.ravel()]).all():
        raise ValueError('Nonfinite native state after resume')
    actual_quaternions=poses[:,3:].astype(np.float64)
    expected_quaternions=expected[:,3:].astype(np.float64)
    norms = np.linalg.norm(actual_quaternions, axis=1)
    expected_norms=np.linalg.norm(expected_quaternions,axis=1)
    if np.any(norms == 0.) or np.any(expected_norms == 0.):
        raise ValueError('Invalid native quaternion after resume')
    # Normalize both rotations in float64. Native float32 norm roundoff is
    # not a physical angular difference, and antipodal quaternions agree.
    angle=(Rotation.from_quat(expected_quaternions/expected_norms[:,None]).inv()*
           Rotation.from_quat(actual_quaternions/norms[:,None])).magnitude()
    per_body = []
    for i, path in enumerate(arrays['body_paths']):
        per_body.append(dict(path=str(path), position_error_m=float(np.linalg.norm(poses[i, :3]-expected[i, :3])),
                             orientation_error_rad=float(angle[i]),
                             linear_velocity_error_m_s=float(np.linalg.norm(velocities[i, :3]-arrays['body_velocities_world'][i, :3])),
                             angular_velocity_error_rad_s=float(np.linalg.norm(velocities[i, 3:]-arrays['body_velocities_world'][i, 3:]))))
    report = dict(body_errors=per_body, body_count=len(per_body),
                  verification_scope='Measured values immediately after restore, before first resumed physics step',
                  numeric_errors_are_diagnostics=True, native_validity='pending')
    for key in ('position_error_m', 'orientation_error_rad', 'linear_velocity_error_m_s', 'angular_velocity_error_rad_s'):
        worst = max(per_body, key=lambda row: row[key])
        report['max_'+key] = worst[key]
        report['worst_'+key+'_body'] = worst['path']
    for kind, view in (('robot', robot), ('plant', plant)):
        for suffix, getter in (('q', view.get_dof_positions), ('qvel', view.get_dof_velocities)):
            actual = _array(getter())
            wanted = arrays[kind+'_'+suffix][None]
            if actual.shape != wanted.shape or not np.isfinite(actual).all():
                raise ValueError('Resume native '+kind+' '+suffix+' shape/finite mismatch')
            report[kind+'_'+suffix+'_max_abs_error'] = float(np.max(np.abs(actual-wanted)))
    return report
