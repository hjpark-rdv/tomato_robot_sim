"""Explicit canonical idle start proof, separate from completed harvest cycles.

The checkpoint is measured native state. A write-once sidecar binds the final
idle report and summary to the exact checkpoint/model/controller state. Normal
completed-cycle resume remains strict and does not accept this capture kind.
"""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from compact_motion_assets import file_sha256
from native_route_checkpoint import load_checkpoint


SCHEMA = 's300-canonical-conditioned-native-start-v1'
CAPTURE_REASON = 'canonical_conditioned_start'
MARKER_NAME = 'conditioned_start.json'


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise ValueError('Finite '+label+' required')
    return float(value)


def _json_digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),
        ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def conditioned_checkpoint_sequence(report):
    """Declare an idle capture without inventing a completed-cycle record."""
    from foliage_conditioning import validate_report
    validate_report(report)
    if report.get('canonical_conditioned_start') is not True:
        raise ValueError('Conditioned checkpoint requires a completed strict native suffix')
    return dict(checkpoint_capture_reason=CAPTURE_REASON,checkpoint_physics_valid=True,
        completed_cycle=False,cycle_plans=[],attempt_counts={},fruit_joints={},
        conditioning_report_schema=report['schema'],conditioning_report_sha256=_json_digest(report),
        scope='Measured canonical conditioned idle state; no harvest attempt or completed manipulation cycle')


def _verify_evidence(metadata, arrays, checkpoint, report_path, summary_path, native_stage_path):
    from foliage_conditioning import validate_report
    report=json.loads(report_path.read_text());validate_report(report)
    sequence=metadata.get('sequence', {})
    if (sequence.get('checkpoint_capture_reason') != CAPTURE_REASON or
            sequence.get('checkpoint_physics_valid') is not True or
            sequence.get('completed_cycle') is not False or sequence.get('cycle_plans') != [] or
            sequence.get('conditioning_report_sha256') != _json_digest(report) or
            sequence.get('conditioning_report_schema') != report.get('schema')):
        raise ValueError('Canonical conditioned checkpoint has no exact idle capture/report proof')
    if report.get('canonical_conditioned_start') is not True or report.get('completed_cycle') is not False:
        raise ValueError('Conditioning report is not a canonical idle start')
    if report.get('training_eligible') is not False or report.get('harvest_success') is not None:
        raise ValueError('Conditioned idle start cannot be a training or harvest success')
    context=report['source_context'];assembly=Path(metadata['source_assembly']).resolve()
    if (Path(context['source_assembly']).resolve() != assembly or
            context['scene_sha256'] != metadata['scene_sha256'] or
            context['source_plant'] != metadata['source_plant']):
        raise ValueError('Conditioned start source assembly/model/plant mismatch')
    native=json.loads(native_stage_path.read_text())
    if (native.get('source_assembly') != str(assembly) or
            native.get('source_scene_sha256') != metadata['scene_sha256'] or
            native.get('native_scene_sha256') != context['native_scene_sha256']):
        raise ValueError('Conditioned start actual native stage provenance mismatch')
    native_scene=Path(native['native_scene']).resolve()
    valid_scenes={assembly/'scene.usda'}
    dependency=metadata['manifest'].get('compact_assets',{}).get('materialized_dependency')
    if dependency:valid_scenes.add((assembly/dependency['relative_path']).resolve())
    if native_scene not in valid_scenes or file_sha256(native_scene) != context['native_scene_sha256']:
        raise ValueError('Conditioned start actual native model SHA mismatch')
    timestamp=_finite_number(report['final_time_s'],'actual final native time')
    if abs(timestamp-metadata['time_s']) > 1e-8 or abs(timestamp-report['strict_window']['end_time_s'])>1e-8:
        raise ValueError('Conditioned checkpoint is not at actual final native time')
    summary=json.loads(summary_path.read_text())
    if (summary.get('mode') != 'idle' or summary.get('completed') is not True or
            summary.get('stop_reason') is not None or summary.get('initialization_valid') is not True or
            summary.get('canonical_conditioned_start') is not True or
            summary.get('actual_self_collision_audit_passed') is not True or
            summary.get('native_new_break_count') != 0 or summary.get('rail_destination') is not None or
            summary.get('fixture_only') is not False or summary.get('disabled_background_colliders') != 0 or
            summary.get('physics_settings',{}).get('hz') != report['native_hz'] or
            summary.get('model_sha256') != metadata['scene_sha256'] or
            Path(summary.get('scene','')).resolve() != assembly/'scene.usda' or
            abs(_finite_number(summary.get('simulated_s'),'terminal idle summary time')-timestamp)>1e-8):
        raise ValueError('Conditioned idle source summary is not the exact valid final state')
    if metadata.get('controller_state') is None:
        raise ValueError('Canonical conditioned start requires native position and velocity drive-target capture')
    states_path=summary_path.parent/'states.npz'
    audit_path=summary_path.parent/'self_collision_actual.json'
    audit=json.loads(audit_path.read_text())
    if (audit.get('passed') is not True or audit.get('complete') is not True or
            audit.get('scene_sha256') != metadata['scene_sha256'] or
            audit.get('input_sha256') != file_sha256(states_path) or audit.get('physics_rerun') is not False):
        raise ValueError('Canonical conditioned start lacks a complete exact actual-state self audit')
    with np.load(states_path,allow_pickle=False) as stored:
        times=np.asarray(stored['time'],dtype=float)
        poses=np.asarray(stored['poses'],dtype=float)
        q=np.asarray(stored['robot_q'],dtype=float)
        paths=list(map(str,stored['body_paths']))
    expected_times=np.arange(round(timestamp*report['native_hz'])+1)/report['native_hz']
    if (times.shape!=expected_times.shape or not np.isfinite(times).all() or
            not np.allclose(times,expected_times,rtol=0.,atol=1e-8) or
            abs(times[-1]-timestamp)>1e-8 or audit.get('samples') != len(times) or
            paths != list(map(str,arrays['body_paths'])) or
            poses.shape != (len(times),len(paths),7) or not np.isfinite(poses).all() or
            q.shape != (len(times),len(arrays['robot_dof_names'])) or not np.isfinite(q).all() or
            not np.array_equal(poses[-1],arrays['body_poses_world']) or
            not np.array_equal(q[-1],arrays['robot_q'])):
        raise ValueError('Canonical actual-state audit trace does not cover the exact final checkpoint')
    paths=list(map(str,arrays['body_paths']));rows=report['stability']['bodies']
    by_path={row['path']:row for row in rows}
    if len(by_path)!=len(rows) or set(by_path)!=set(paths):
        raise ValueError('Canonical strict stability/native checkpoint body inventory mismatch')
    velocities=np.asarray(arrays['body_velocities_world'],dtype=float)
    for index,path in enumerate(paths):
        row=by_path[path]
        recorded=np.asarray(row['terminal_linear_velocity_m_s']+row['terminal_angular_velocity_rad_s'],dtype=float)
        if recorded.shape!=(6,) or not np.isfinite(recorded).all() or not np.array_equal(recorded,velocities[index]):
            raise ValueError('Canonical strict terminal velocity does not match the measured checkpoint')
    from pxr import Usd
    from foliage_conditioning import conditioning_inventory
    stage=Usd.Stage.Open(str(native_scene))
    leaves,counterparts=conditioning_inventory(stage,metadata['manifest'],paths)
    if leaves!=report['exact_compliant_leaf_colliders'] or counterparts!=report['exact_counterpart_colliders']:
        raise ValueError('Canonical conditioned report/source collider inventory mismatch')
    helper_path=summary_path.parent/'runtime_code_sources'/'foliage_conditioning.py'
    if file_sha256(helper_path) != report.get('helper_source_sha256'):
        raise ValueError('Conditioned start imported helper code SHA mismatch')
    # load_checkpoint already validates all body/DOF identities, finite poses,
    # measured velocities, native controller target schema and NPZ/USDC hashes.
    if checkpoint.name == 'cycle_checkpoints' or checkpoint.parent.name == 'cycle_checkpoints':
        raise ValueError('Conditioned idle checkpoint must not masquerade as a manipulation cycle')
    return report,summary,native


def write_conditioned_start_marker(checkpoint,report_path,summary_path,*,native_stage_path=None):
    """Bind finished raw/strict idle evidence to an immutable measured capture."""
    checkpoint=Path(checkpoint).resolve();report_path=Path(report_path).resolve();summary_path=Path(summary_path).resolve()
    native_stage_path=Path(native_stage_path or summary_path.parent/'native_stage_selection.json').resolve()
    metadata,arrays=load_checkpoint(checkpoint)
    report,summary,native=_verify_evidence(metadata,arrays,checkpoint,report_path,summary_path,native_stage_path)
    marker=dict(schema=SCHEMA,checkpoint=str(checkpoint),
        checkpoint_metadata_sha256=metadata['metadata_sha256'],checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],
        source_assembly=metadata['source_assembly'],source_scene_sha256=metadata['scene_sha256'],
        source_materialized_dependency=copy.deepcopy(metadata['manifest'].get('compact_assets',{}).get('materialized_dependency')),
        actual_native_scene=native['native_scene'],actual_native_scene_sha256=native['native_scene_sha256'],
        source_plant=metadata['source_plant'],actual_final_time_s=metadata['time_s'],
        report=str(report_path),report_sha256=file_sha256(report_path),
        source_summary=str(summary_path),source_summary_sha256=file_sha256(summary_path),
        native_stage_selection=str(native_stage_path),native_stage_selection_sha256=file_sha256(native_stage_path),
        actual_states=str(summary_path.parent/'states.npz'),actual_states_sha256=file_sha256(summary_path.parent/'states.npz'),
        actual_self_audit=str(summary_path.parent/'self_collision_actual.json'),actual_self_audit_sha256=file_sha256(summary_path.parent/'self_collision_actual.json'),
        imported_helper=str(summary_path.parent/'runtime_code_sources'/'foliage_conditioning.py'),
        imported_helper_sha256=report['helper_source_sha256'],
        raw_conditioning_evidence_retained=True,strict_suffix_separate_from_raw_conditioning=True,
        completed_cycle=False,harvest_attempt=False,training_eligible=False,harvest_success=None)
    marker['marker_sha256']=_json_digest(marker)
    path=checkpoint/MARKER_NAME
    with path.open('x',encoding='utf-8') as stream:
        json.dump(marker,stream,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False);stream.write('\n')
    path.chmod(0o444)
    return marker


def require_conditioned_checkpoint(metadata,arrays,checkpoint):
    """Validate every immutable sidecar/source/report hash; no state mutation."""
    checkpoint=Path(checkpoint).resolve();path=checkpoint/MARKER_NAME
    marker=json.loads(path.read_text())
    unsigned={key:value for key,value in marker.items() if key!='marker_sha256'}
    if marker.get('schema') != SCHEMA or marker.get('marker_sha256') != _json_digest(unsigned):
        raise ValueError('Canonical conditioned start marker schema/SHA mismatch')
    if (marker.get('checkpoint') != str(checkpoint) or
            marker.get('checkpoint_metadata_sha256') != metadata['metadata_sha256'] or
            marker.get('checkpoint_npz_sha256') != metadata['checkpoint_npz_sha256'] or
            marker.get('source_assembly') != metadata['source_assembly'] or
            marker.get('source_scene_sha256') != metadata['scene_sha256'] or
            marker.get('source_plant') != metadata['source_plant'] or
            marker.get('actual_final_time_s') != metadata['time_s'] or
            marker.get('source_materialized_dependency') != metadata['manifest'].get('compact_assets',{}).get('materialized_dependency')):
        raise ValueError('Canonical conditioned marker/checkpoint/source identity mismatch')
    paths={}
    for key in ('report','source_summary','native_stage_selection','imported_helper','actual_states','actual_self_audit'):
        target=Path(marker[key])
        if not target.is_absolute() or file_sha256(target) != marker[key+'_sha256']:
            raise ValueError('Canonical conditioned start '+key+' SHA mismatch')
        paths[key]=target
    _,_,native=_verify_evidence(metadata,arrays,checkpoint,paths['report'],paths['source_summary'],paths['native_stage_selection'])
    if marker['actual_native_scene'] != native['native_scene'] or marker['actual_native_scene_sha256'] != native['native_scene_sha256']:
        raise ValueError('Canonical conditioned marker actual native model mismatch')
    return dict(kind='canonical_conditioned_start',marker=str(path),marker_sha256=file_sha256(path),
        source_summary=marker['source_summary'],source_summary_sha256=marker['source_summary_sha256'],
        conditioning_report=marker['report'],conditioning_report_sha256=marker['report_sha256'],
        actual_native_scene=marker['actual_native_scene'],actual_native_scene_sha256=marker['actual_native_scene_sha256'],
        actual_final_time_s=metadata['time_s'],completed_cycle=False,harvest_attempt=False,
        training_eligible=False,harvest_success=None)
