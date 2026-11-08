"""Read verified native handoffs for replay publication; never alter sources.

Reuses the checkpoint/terminal-state and active-window identity checkers. The
legacy route result's pending label is not a certificate: every recorded native
restore metric and exact controller/body mapping must independently be present
and pass the unchanged guards that run_motion applied before its first step.
"""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'nvidia-sim/harvest_s300'))
from native_route_checkpoint import load_checkpoint
from run_row_harvest import completed_final_checkpoint
from compact_motion_assets import verify_compact_dependencies


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def _route_context(assembly,manifest):
    from pxr import Usd
    from native_route_start import prepare_route_start
    dependency=verify_compact_dependencies(assembly,manifest)
    scene=assembly/dependency['relative_path'] if dependency else assembly/'scene.usda'
    stage=Usd.Stage.Open(str(scene))
    return prepare_route_start(stage,assembly,manifest)


def _finite_error(value,label):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or value<0:
        raise ValueError('Incomplete/nonfinite native route restore metric: '+label)
    return float(value)


def verify_route_result(expected,native,*,standalone_paths,reactivation=None):
    """Check the complete saved result against the existing route-start contract."""
    json.dumps(native,allow_nan=False)
    if (native.get('schema')!='s300-native-active-window-start-v1' or
            native.get('native_initialization_validation') not in ('pending','passed')):
        raise ValueError('Unknown native route validation record')
    for key,value in expected.items():
        if key=='native_initialization_validation':continue
        if native.get(key)!=value:
            raise ValueError('Native route identity/input mismatch: '+key)
    controller=native.get('restored_controller_state',{})
    if any(controller.get(key)!=value for key,value in expected.get('controller_state',{}).items()):
        raise ValueError('Native route restored controller provenance differs from its measured parent')
    if (controller.get('controller_continuity_verified') is not True or
            controller.get('position_targets_verified') is not True or
            controller.get('velocity_targets_verified') is not True or
            controller.get('unknown_position_measured_hold_fallback') is not False or
            controller.get('velocity_drive_left_authored') is not False):
        raise ValueError('Incomplete native route controller verification')
    for key in ('position_targets_max_abs_error','velocity_targets_max_abs_error'):
        if _finite_error(controller.get(key),key)!=0.:
            raise ValueError('Native route controller targets were not reproduced exactly')
    errors=native.get('preserved_body_errors')
    expected_paths=set(expected['measured_parent_body_mapping'])
    if (not isinstance(errors,list) or len(errors)!=len(expected_paths) or
            {row.get('path') for row in errors}!=expected_paths or
            native.get('preserved_body_count')!=len(errors) or
            native.get('native_articulation_kinematics_refreshed') is not True or
            native.get('articulated_link_transforms_set_directly') is not False):
        raise ValueError('Incomplete native route body mapping/result')
    fields=('position_error_m','orientation_error_rad','linear_velocity_error_m_s','angular_velocity_error_rad_s')
    additional=[]
    if reactivation is not None:
        restored=native.get('unharvested_reactivation',{})
        additional=restored.get('native_body_errors',[])
        if (restored.get('source')!=reactivation['proof'] or restored.get('prior_breaks')!=0 or
                restored.get('fruit_resurrected') is not False or
                restored.get('original_preload_command_restored') is not True or
                restored.get('native_q_max_abs_error')!=0. or restored.get('native_qvel_max_abs_error')!=0. or
                not isinstance(additional,list) or len(additional)!=restored.get('native_body_count')):
            raise ValueError('Incomplete archived unharvested-plant native restore')
    for row in errors+additional:
        for field in fields:_finite_error(row.get(field),field)
    for field in fields:
        maximum=max(row[field] for row in errors+additional)
        if _finite_error(native.get('max_'+field),'max_'+field)!=maximum:
            raise ValueError('Native route aggregate/body restore discrepancy: '+field)
    for group in ('standalone','articulated'):
        rows=native.get(group+'_body_errors')
        wanted=[row for row in errors+additional if (row['path'] in standalone_paths)==(group=='standalone')]
        if rows!=wanted:raise ValueError('Native route standalone/articulated mapping differs')
        for field in fields[2:]:
            if _finite_error(native.get(group+'_max_'+field),group+'_max_'+field)!=max((row[field] for row in rows),default=0.):
                raise ValueError('Native route velocity aggregate differs')
    # These are the existing run_motion route restore guards, not new contact
    # tolerances. Articulated link velocities remain diagnostic as before.
    tolerances=dict(position_m=1e-4,orientation_rad=1e-4,robot_q=1e-5,robot_qvel=1e-5,
        standalone_linear_velocity_m_s=1e-4,standalone_angular_velocity_rad_s=1e-4)
    declared=native.get('numerical_restore_tolerances',{})
    if any(declared.get(key)!=value for key,value in tolerances.items()):
        raise ValueError('Missing/changed native route numerical restore guards')
    guards=dict(max_position_error_m=1e-4,max_orientation_error_rad=1e-4,
        standalone_max_linear_velocity_error_m_s=1e-4,standalone_max_angular_velocity_error_rad_s=1e-4,
        robot_q_max_abs_error=1e-5,robot_qvel_max_abs_error=1e-5)
    for key,limit in guards.items():
        if _finite_error(native.get(key),key)>limit:raise ValueError('Native route restore guard failed: '+key)
    for key in ('plant_q_max_abs_error','plant_qvel_max_abs_error'):
        if _finite_error(native.get(key),key)!=0.:raise ValueError('Native route new plant state changed')
    return True


def measured_handoff(trial,assembly,manifest,previous):
    trial=Path(trial).resolve();assembly=Path(assembly).resolve();parent=Path(previous['trial']).resolve()
    ordinary=(trial/'resume_input.json').is_file()
    route=(trial/'route_start_input.json').is_file()
    if ordinary==route:raise ValueError('Missing or ambiguous actual native resume/active-window input')
    input_name='resume_input.json' if ordinary else 'route_start_input.json'
    native_name='resume_native_validation.json' if ordinary else 'route_start_native_validation.json'
    source=read(trial/input_name);checkpoint=Path(source['checkpoint']).resolve()
    if not checkpoint.is_relative_to(parent):
        raise ValueError('Unproven branch/reset cannot join measured-state collection: '+str(trial))
    if not (trial/native_name).is_file():
        raise ValueError('Final actual segment lacks its complete native handoff verification: '+native_name)
    metadata,_=load_checkpoint(checkpoint)
    if metadata['metadata_sha256']!=source['checkpoint_metadata_sha256']:
        raise ValueError('Restored checkpoint metadata SHA mismatch: '+str(trial))
    native=read(trial/native_name)
    if ordinary:
        if native.get('exact_native_mapping') is not True:
            raise ValueError('Actual restored state mapping missing: '+str(trial))
    else:
        # Existing checker rejects rollback, stopped/invalid parents and absent
        # actual-state audits. Never infer provenance from directory names alone.
        final,final_metadata=completed_final_checkpoint(parent,read(parent/'summary.json'))
        if final.resolve()!=checkpoint or final_metadata['metadata_sha256']!=metadata['metadata_sha256']:
            raise ValueError('Active-window handoff does not use the final valid actual parent')
        context=_route_context(assembly,manifest)
        if context['report']!=source:raise ValueError('Active-window input differs from existing model/checkpoint checker')
        verify_route_result(context['report'],native,standalone_paths=set(context['standalone_paths']),
            reactivation=context.get('unharvested_reactivation'))
    return dict(previous_trial=str(parent),checkpoint=str(checkpoint),metadata_sha256=metadata['metadata_sha256'],
        metadata_file_sha256=sha(checkpoint/'checkpoint.json'),resume_sha256=sha(trial/input_name),
        native_validation_sha256=sha(trial/native_name),handoff_kind='same_model_resume' if ordinary else 'active_window_native_start',
        input_file=input_name,native_validation_file=native_name,
        robot_q_max_abs_error=native.get('robot_q_max_abs_error'),max_position_error_m=native.get('max_position_error_m'),
        controller_continuity_verified=native.get('restored_controller_state',{}).get('controller_continuity_verified'),
        existing_route_model_checker_reused=not ordinary,
        native_solver_caches_restart=True,bitwise_engine_continuity=False)
