"""Opt-in fixed lift drive bias measured at a held ready pose.

The nominal geometric plan and the actuator command remain distinct. No scene
state, stiffness, damping, gravity or timestep is changed, and the bias is never
integrated or relearned from fruit/leaf contact during a harvest cycle.
"""
import numpy as np


LIFT_NAME='farmily_lift_height_joint'


def _lift_index(dof_names,lift_name):
    names=list(dof_names)
    if len(names)!=len(set(names)) or names.count(lift_name)!=1:
        raise ValueError('Unique exact lift DOF mapping is required')
    return names.index(lift_name),len(names)


def estimate_static_lift_bias(time_s,measured_q,position_targets,dof_names,*,physics_hz,
                             lift_name=LIFT_NAME,max_bias_m=.015,min_hold_s=.25,
                             max_position_drift_m=.0002,max_target_drift_m=.00001,
                             max_bias_variation_m=.0002):
    """Estimate once from a complete, stationary held-target physics window.

    Position finite differences are reported independently from native qvel;
    PhysX's native qvel can be nonzero while its post-solve position is constant.
    Bounds reject an unsuitable calibration window instead of silently clipping
    its bias. The caller must choose an unloaded ready hold and retain its trace
    and model hash; this function cannot infer whether the robot touched a plant.
    """
    index,count=_lift_index(dof_names,lift_name)
    stamps=np.asarray(time_s,dtype=float);q=np.asarray(measured_q,dtype=float)
    targets=np.asarray(position_targets,dtype=float)
    controls=[physics_hz,max_bias_m,min_hold_s,max_position_drift_m,
              max_target_drift_m,max_bias_variation_m]
    if any(isinstance(v,bool) for v in controls) or not np.isfinite(controls).all() or any(v<=0 for v in controls):
        raise ValueError('Calibration limits and physics Hz must be positive finite numbers')
    if max_bias_m>.05:raise ValueError('Static lift compensation is bounded to at most 50mm')
    if stamps.ndim!=1 or len(stamps)<2 or q.shape!=(len(stamps),count) or targets.shape!=q.shape:
        raise ValueError('Complete timestamp, measured q and native target window required')
    if not np.isfinite(stamps).all() or not np.isfinite(q).all() or not np.isfinite(targets).all():
        raise ValueError('Calibration samples must be finite')
    dt=np.diff(stamps)
    if not np.allclose(dt,1/physics_hz,rtol=0.,atol=1e-10):
        raise ValueError('Calibration timestamps have missing physics samples or a reset')
    if stamps[-1]-stamps[0]<min_hold_s-1e-10:
        raise ValueError('Calibration hold window is too short')
    nominal=q[:,index];drive=targets[:,index];residual=drive-nominal
    if np.ptp(drive)>max_target_drift_m:
        raise ValueError('Lift target moved during the static calibration window')
    if np.ptp(nominal)>max_position_drift_m or np.ptp(residual)>max_bias_variation_m:
        raise ValueError('Lift was not position-settled during calibration')
    bias=float(np.median(residual))
    if abs(bias)>max_bias_m:
        raise ValueError('Measured static lift bias exceeds the opt-in compensation bound')
    return dict(schema='static-lift-position-feedforward-v1',lift_dof=lift_name,
        lift_dof_index=index,offset_m=bias,calibration_start_s=float(stamps[0]),
        calibration_end_s=float(stamps[-1]),calibration_samples=len(stamps),physics_hz=float(physics_hz),
        lift_position_span_m=float(np.ptp(nominal)),lift_target_span_m=float(np.ptp(drive)),
        target_minus_measured_min_m=float(residual.min()),target_minus_measured_max_m=float(residual.max()),
        measured_position_velocity_max_m_s=float(np.max(np.abs(np.diff(nominal)/dt))),
        max_bias_m=float(max_bias_m),one_time_fixed_bias=True,
        nominal_geometric_plan_unchanged=True,actuator_properties_changed=False,
        contact_free_ready_hold_verified_by_caller=False,
        scope='Ready-hold drive compensation; requires full nominal/drive collision audits and actual physics verification')


def lift_drive_command(nominal_command,bias,dof_names,*,lower_limit_m,upper_limit_m):
    """Add the fixed lift offset without changing nominal geometry or other DOFs.

    None returns the exact original object for legacy behavior. No automatic
    clipping hides a limit violation. Native readback, nominal and drive arrays
    must be stored separately by the runner.
    """
    if bias is None:return nominal_command
    if not isinstance(bias,dict) or bias.get('schema')!='static-lift-position-feedforward-v1' or bias.get('one_time_fixed_bias') is not True:
        raise ValueError('One-time static lift compensation evidence is required')
    index,count=_lift_index(dof_names,bias.get('lift_dof'))
    value=bias.get('offset_m')
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or abs(value)>.05:
        raise ValueError('Finite bounded static lift offset required')
    if (isinstance(lower_limit_m,bool) or isinstance(upper_limit_m,bool) or
        not np.isfinite([lower_limit_m,upper_limit_m]).all() or lower_limit_m>=upper_limit_m):
        raise ValueError('Original finite lift travel limits required')
    q=np.asarray(nominal_command)
    if q.ndim not in (1,2) or q.shape[-1]!=count or not np.issubdtype(q.dtype,np.floating) or not np.isfinite(q).all():
        raise ValueError('Finite floating native command array with exact DOF shape required')
    if bias.get('lift_dof_index')!=index:raise ValueError('Calibration lift DOF index differs from execution mapping')
    corrected=q[...,index]+value
    if np.any(corrected<lower_limit_m) or np.any(corrected>upper_limit_m):
        raise ValueError('Compensated lift drive command exceeds original travel limits')
    if value==0:return nominal_command
    result=q.copy();result[...,index]=corrected
    return result
