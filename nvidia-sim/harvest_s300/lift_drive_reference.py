"""Opt-in lift drive reference; measured geometry remains separate from targets.

This module never changes a simulator, plan, body state, drive stiffness or
collision policy. Callers must validate the checkpoint before constructing the
reference, and must audit the desired path and actual native tracking separately.
"""
import re

import numpy as np


LIFT_JOINT = 'farmily_lift_height_joint'


def _names(value):
    names = list(map(str, value))
    if not names or len(names) != len(set(names)) or LIFT_JOINT not in names:
        raise ValueError('Unique native DOF names including the exact lift are required')
    return names


def _vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('Finite exact native '+label+' vector required')
    return result


def build_lift_drive_reference(metadata, arrays, controller_state, *, enabled=False,
                               camera_harvest=False, lift_bounds_m=(0., .75)):
    """Read a validated checkpoint's native getter bias, only when opted in.

    ``controller_state`` must come from resolve_robot_controller_state after
    load_checkpoint verified its hashes. Logged-setter and unknown legacy
    controller fallbacks are deliberately ineligible for this correction.
    """
    if type(enabled) is not bool or type(camera_harvest) is not bool:
        raise ValueError('Explicit boolean opt-in and camera mode required')
    if not enabled:
        return None
    if not camera_harvest:
        raise ValueError('Lift drive reference is restricted to opted-in camera harvest')
    names = _names(arrays['robot_dof_names'])
    size = len(names)
    native = metadata.get('controller_state', {})
    provenance = controller_state.get('provenance', {})
    digest = metadata.get('metadata_sha256', '')
    if (metadata.get('schema') != 'native_route_checkpoint_v1' or
            not isinstance(digest, str) or not re.fullmatch(r'[0-9a-f]{64}', digest) or
            provenance.get('checkpoint_metadata_sha256') != digest or
            provenance.get('source') != 'checkpoint_native_drive_getters' or
            provenance.get('controller_continuity_verified') is not True or
            provenance.get('position_targets_verified') is not True or
            provenance.get('velocity_targets_verified') is not True or
            native.get('schema') != 'native_robot_drive_targets_v1' or
            native.get('robot_dof_names') != names or
            native.get('position_source') != 'native_get_dof_position_targets' or
            native.get('velocity_source') != 'native_get_dof_velocity_targets'):
        raise ValueError('Validated exact native checkpoint/getter identity is required')
    if metadata.get('sequence', {}).get('checkpoint_physics_valid') is not True:
        raise ValueError('Checkpoint must explicitly record physics validity')
    measured = _vector(arrays['robot_q'], size, 'measured state')
    targets = _vector(native.get('position_targets'), size, 'getter position target')
    velocities = _vector(native.get('velocity_targets'), size, 'getter velocity target')
    resolved = _vector(controller_state.get('position_targets'), size, 'resolved position target')
    resolved_velocity = _vector(controller_state.get('velocity_targets'), size, 'resolved velocity target')
    if not np.array_equal(targets, resolved) or not np.array_equal(velocities, resolved_velocity):
        raise ValueError('Resolved controller targets differ from checkpoint native getters')
    bounds = np.asarray(lift_bounds_m, dtype=float)
    if bounds.shape != (2,) or not np.isfinite(bounds).all() or bounds[0] > bounds[1]:
        raise ValueError('Finite actual prismatic lift limits required')
    index = names.index(LIFT_JOINT)
    actual, target = float(measured[index]), float(targets[index])
    if not bounds[0] <= actual <= bounds[1] or not bounds[0] <= target <= bounds[1]:
        raise ValueError('Measured lift or proven drive target exceeds actual limits')
    stamp = metadata.get('time_s')
    if isinstance(stamp, bool) or not np.isscalar(stamp) or not np.isfinite(stamp):
        raise ValueError('Finite native checkpoint timestamp required')
    return dict(schema='s300-lift-drive-reference-v1', joint_name=LIFT_JOINT,
        joint_kind='prismatic', units='m', checkpoint_metadata_sha256=digest,
        checkpoint_time_s=float(stamp), source='checkpoint_native_drive_getters',
        measured_lift_m=actual, native_position_target_m=target,
        retained_target_minus_measured_m=target-actual,
        lift_bounds_m=bounds.tolist(), physical_geometry_is_measured=True,
        changes_only_named_lift_command=True, stiffness_or_physics_changed=False)


def apply_lift_drive_reference(desired_targets, reference, *, dof_names, bounds):
    """Return separate native targets and evidence; never clamp any joint.

    Desired targets describe the measured/FK path, including the lift. Bounds
    must cover every native DOF. The same limits are checked before and after
    the lift-only correction; arm, rail, jaws and input arrays stay unchanged.
    """
    names = _names(dof_names)
    desired = np.asarray(desired_targets, dtype=float)
    limits = np.asarray(bounds, dtype=float)
    if (desired.ndim not in (1, 2) or desired.shape[-1] != len(names) or
            not desired.size or not np.isfinite(desired).all() or
            limits.shape != (2, len(names)) or not np.isfinite(limits).all() or
            np.any(limits[0] > limits[1])):
        raise ValueError('Finite full native desired command and DOF limits required')
    if np.any(desired < limits[0]) or np.any(desired > limits[1]):
        raise ValueError('Desired physical path exceeds actual joint limits; no clamp')
    command = desired.copy()
    if reference is None:
        return command, dict(enabled=False, commands_changed=False)
    if (reference.get('schema') != 's300-lift-drive-reference-v1' or
            reference.get('joint_name') != LIFT_JOINT or
            reference.get('joint_kind') != 'prismatic' or
            reference.get('source') != 'checkpoint_native_drive_getters' or
            reference.get('physical_geometry_is_measured') is not True):
        raise ValueError('Exact native prismatic lift reference required')
    index = names.index(LIFT_JOINT)
    actual = reference.get('measured_lift_m')
    target = reference.get('native_position_target_m')
    bias = reference.get('retained_target_minus_measured_m')
    values = np.asarray([actual, target, bias], dtype=float)
    if not np.isfinite(values).all() or bias != target-actual:
        raise ValueError('Lift reference must retain its exact native measured/getter difference')
    if not np.array_equal(np.asarray(reference.get('lift_bounds_m')), limits[:, index]):
        raise ValueError('Applied lift limits differ from the validated reference')
    command[..., index] += bias
    if np.any(command < limits[0]) or np.any(command > limits[1]):
        raise ValueError('Corrected native drive command exceeds actual joint limits; no clamp')
    other = [i for i in range(len(names)) if i != index]
    assert np.array_equal(command[..., other], desired[..., other])
    return command, dict(enabled=True, joint_name=LIFT_JOINT, units='m',
        reference_checkpoint_metadata_sha256=reference['checkpoint_metadata_sha256'],
        retained_target_minus_measured_m=float(bias),
        desired_physical_lift_m=np.asarray(desired[..., index]).tolist(),
        native_lift_position_target_m=np.asarray(command[..., index]).tolist(),
        other_dof_commands_unchanged=True, desired_and_command_bounds_valid=True,
        clamped=False, physical_geometry_is_measured=True,
        physical_tracking_and_collision_validation_required=True)
