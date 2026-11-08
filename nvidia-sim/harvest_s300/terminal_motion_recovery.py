"""Explicit, hash-bound retreat from one safe terminal opening-guard stop.

No simulator calls or checkpoint rewriting. This never authorizes a previous
good cycle as the initial state and never awards a harvesting attempt.
"""
import copy
import ast
import hashlib
import json
import re
from pathlib import Path

import numpy as np


def _read(path):return json.loads(Path(path).read_text())


def _sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _source_lift_reference(trial,summary):
    reference=summary.get('lift_drive_reference')
    if reference is None:return None,None
    from native_route_checkpoint import load_checkpoint
    from robot_controller_state import resolve_robot_controller_state
    from lift_drive_reference import build_lift_drive_reference
    path=trial/'lift_drive_reference.json';source_input=_read(trial/'resume_input.json')
    if _read(path)!=reference:raise ValueError('Original source lift reference differs from saved native summary')
    checkpoint=Path(source_input['checkpoint']);metadata,arrays=load_checkpoint(checkpoint)
    if metadata['metadata_sha256']!=source_input['checkpoint_metadata_sha256']:
        raise ValueError('Original source lift checkpoint SHA mismatch')
    controller=resolve_robot_controller_state(metadata,arrays,checkpoint)
    reconstructed=build_lift_drive_reference(metadata,arrays,controller,enabled=True,
        camera_harvest=True,lift_bounds_m=reference['lift_bounds_m'])
    if reconstructed!=reference:raise ValueError('Original source lift reference was not derived from its exact native getters')
    return copy.deepcopy(reference),dict(path=str(path),sha256=_sha(path),
        source_checkpoint=str(checkpoint.resolve()),source_checkpoint_metadata_sha256=metadata['metadata_sha256'])


def _source_native_command_dtype(trial,logged):
    if np.asarray(logged).dtype==np.dtype('float32'):return np.dtype('float32'),None
    path=trial/'code/run_motion.py';tree=ast.parse(path.read_text())
    tensor=next((n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name=='tensor'),None)
    if (tensor is None or not tensor.args.defaults or ast.unparse(tensor.args.defaults[0])!='torch.float32' or
            not any(isinstance(n,ast.Call)and ast.unparse(n.func)=='target_tensor.copy_' and
                    len(n.args)==1 and ast.unparse(n.args[0])=='tensor(qcommand)'for n in ast.walk(tree))):
        raise ValueError('Original native command dtype/copy path is not the recorded float32 tensor conversion')
    return np.dtype('float32'),dict(path=str(path),sha256=_sha(path),native_command_dtype='float32',
        logged_command_dtype=str(np.asarray(logged).dtype),tolerance_used=False)


def validate_terminal_forward_suffix(assembly,manifest,checkpoint,metadata,arrays,*,stage=None):
    """Continue only the exact terminal cycle after a proven released RowSupply impact.

    The original invalid stop and all measured state remain immutable. This is
    neither a previous good checkpoint nor a new approach/harvesting attempt.
    """
    from native_route_checkpoint import load_checkpoint
    from compact_motion_assets import verify_compact_dependencies
    from connect_s300_prefix import reconstruct_phase_tuples
    from motion_continuation import source_command_sample
    assembly=Path(assembly).resolve();checkpoint=Path(checkpoint).resolve()
    if checkpoint.name!='route_checkpoint':
        raise ValueError('Forward suffix requires the latest terminal route_checkpoint; no earlier cycle rollback')
    original,state=load_checkpoint(checkpoint)
    if metadata!=original or set(arrays)!=set(state) or any(not np.array_equal(arrays[k],state[k])for k in state):
        raise ValueError('Forward suffix supplied state differs from exact saved terminal checkpoint')
    if (Path(metadata['source_assembly']).resolve()!=assembly or metadata['manifest']!=manifest or
            metadata['scene_sha256']!=manifest['scene_sha256'] or _sha(assembly/'scene.usda')!=manifest['scene_sha256']):
        raise ValueError('Forward suffix immutable assembly/manifest/SHA mismatch')
    dependency=verify_compact_dependencies(assembly,manifest)
    trial=checkpoint.parent;summary=_read(trial/'summary.json');runtime=_read(trial/'runtime_settings.json')
    if (summary.get('completed')is not False or summary.get('stop_reason')!='penetration_over_contact_policy' or
            summary.get('initialization_valid')is not True or summary.get('actual_self_collision_audit_passed')is not True or
            summary.get('model_sha256')!=manifest['scene_sha256'] or runtime.get('hz')!=60 or
            runtime.get('contact_limit_mm')!=1. or summary.get('contact_limit_mm')!=1. or
            runtime.get('experimental_tpu_fruit_close_2mm')is not False):
        raise ValueError('Forward suffix requires exact initialized self-audited60Hz/1mm released-support stop')
    states_path=trial/'states.npz';audit=_read(trial/'self_collision_actual.json')
    if (audit.get('passed')is not True or audit.get('complete')is not True or
            audit.get('scene_sha256')!=manifest['scene_sha256'] or audit.get('input_sha256')!=_sha(states_path) or
            audit.get('physics_rerun')is not False):
        raise ValueError('Forward suffix requires complete exact stored actual-state self audit')
    names=list(map(str,arrays['robot_dof_names']));expected=['farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3']
    if not set(expected+['jaw_L','jaw_R'])<=set(names):raise ValueError('Forward suffix complete named robot/jaw mapping required')
    controller=metadata.get('controller_state',{})
    targets=np.asarray(controller.get('position_targets'),float)
    if (controller.get('position_source')!='native_get_dof_position_targets' or
            controller.get('velocity_source')!='native_get_dof_velocity_targets' or
            controller.get('robot_dof_names')!=names or targets.shape!=(len(names),) or not np.isfinite(targets).all()):
        raise ValueError('Forward suffix requires unchanged separate native drive targets')
    with np.load(states_path,allow_pickle=False)as trace:
        times=trace['time'];paths=list(map(str,trace['body_paths']));last_q=trace['robot_q'][-1];last_pose=trace['poses'][-1]
        last_command=trace['command'][-1];last_phase=str(trace['phase'][-1]);last_belt=trace['belt_surface_velocity_local_y_m_s'][-1]
        native_dtype,native_command_binding=_source_native_command_dtype(trial,last_command)
        if (times.ndim!=1 or len(times)<2 or not np.isfinite(times).all() or
                not np.allclose(np.diff(times),1/60,rtol=0,atol=1e-8) or len(paths)!=len(set(paths)) or
                abs(float(times[-1])-metadata['time_s'])>1e-8 or
                abs(summary['simulated_s']-metadata['time_s'])>1e-8 or summary.get('physics_steps')!=len(times)-1 or
                not np.array_equal(last_q,arrays['robot_q']) or
                not np.array_equal(last_pose[[paths.index(str(p))for p in arrays['body_paths']]],arrays['body_poses_world']) or
                not np.array_equal(last_command.astype(native_dtype),targets.astype(native_dtype)) or
                not np.array_equal(targets,targets.astype(native_dtype).astype(targets.dtype))):
            raise ValueError('Forward suffix checkpoint is not the latest exact measured physical/controller tip')
        last_index=len(times)-1
    breaks=_read(trial/'break_events.json')
    sequence=metadata['sequence'];records=sequence.get('cycle_plans',[])
    if (breaks!=metadata['break_events'] or breaks!=summary.get('native_break_events') or
            _read(trial/'truss_sequence.json')!=sequence or not records):
        raise ValueError('Forward suffix original native events/sequence mismatch')
    record=records[-1]
    if (record.get('status')!='stopped' or record.get('stop_reason')!=summary['stop_reason'] or
            sequence.get('termination')!=summary['stop_reason'] or record.get('end_frame_index')!=last_index or
            abs(record.get('end_s',-1)-metadata['time_s'])>1e-8 or last_phase not in ('cycle_open','close') or
            not record.get('fruits') or not record.get('truss_key')):
        raise ValueError('Forward suffix requires the latest stopped original close/open record')
    relative=record.get('plan');plan_path=(trial/str(relative)).resolve()
    if not isinstance(relative,str) or Path(relative).is_absolute() or not plan_path.is_relative_to(trial) or not plan_path.is_file():
        raise ValueError('Forward suffix requires exact executed relative cycle plan')
    plan=_read(plan_path);reconstruct_phase_tuples(plan);plan_sha=_sha(plan_path)
    total=sum(float(p['duration_s'])for p in plan['phases']);elapsed=metadata['time_s']-record['start_s'];remaining=total-elapsed
    if (not 0<elapsed<total or abs(record.get('planned_end_s',-1)-record['start_s']-total)>1e-8 or
            abs(remaining*60-round(remaining*60))>1e-7):
        raise ValueError('Forward suffix original phase clock is not an exact remaining native tick range')
    sample=source_command_sample(plan,(last_index-1)/60-record['start_s'])
    ids=[names.index(n)for n in expected];jawids=[names.index(n)for n in ('jaw_L','jaw_R')]
    expected_command=targets.astype(native_dtype);expected_command[ids]=sample['q']
    expected_command[jawids]=[-(sample['gap_m']-.044)/2,(sample['gap_m']-.044)/2]
    source_lift_reference,source_lift_binding=_source_lift_reference(trial,summary)
    if source_lift_reference is not None:
        from lift_drive_reference import apply_lift_drive_reference
        index=names.index('farmily_lift_height_joint')
        corrected,_=apply_lift_drive_reference(expected_command[[index]],source_lift_reference,
            dof_names=['farmily_lift_height_joint'],bounds=np.asarray(source_lift_reference['lift_bounds_m'])[:,None])
        expected_command=expected_command.astype(float);expected_command[index]=corrected[0]
    if (sample['phase']!=last_phase or not np.array_equal(last_command,expected_command.astype(last_command.dtype)) or
            not np.array_equal(np.asarray(last_belt),np.asarray(sample['belt_local_y_m_s'],dtype=np.asarray(last_belt).dtype))):
        raise ValueError('Forward suffix stored terminal arm/jaw/belt command differs from original source curve')
    certificates={}
    for filename in ('self_collision_preflight.json','full_environment_preflight.json'):
        p=plan_path.parent/filename;certificate=_read(p)
        binding=certificate.get('input_sha256')if filename.startswith('self_')else certificate.get('source_sha256',{}).get('plan')
        if (certificate.get('passed')is not True or certificate.get('complete')is not True or binding!=plan_sha or
                (certificate.get('hz')if filename.startswith('self_')else certificate.get('native_hz'))!=60):
            raise ValueError('Forward suffix original accepted full-plan certificate mismatch')
        certificates[filename]=dict(path=str(p),sha256=_sha(p))
    scope_data=summary.get('contact_lifecycle_scope',{})
    if scope_data.get('static_farm_support_enabled')is not True:
        raise ValueError('Forward suffix cannot enable a new static-support policy absent from source')
    if stage is None:
        from pxr import Usd
        stage=Usd.Stage.Open(str(assembly/dependency['relative_path']if dependency else assembly/'scene.usda'))
    from harvest_contact_scope import HarvestContactScope
    scope=HarvestContactScope(manifest,stage=stage,static_farm_support_enabled=True,
        released_pair_enabled=last_phase=='close' or scope_data.get('released_pair_enabled')is True)
    contacts_path=trial/'contacts.json';contacts=_read(contacts_path)
    raw=[e for e in contacts if e.get('t')==metadata['time_s'] and e.get('separation_m',0)<-.001 and
         any(x in str(e.get('a'))or x in str(e.get('b'))for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/','/ElasticNeighbors/'))]
    if not raw:raise ValueError('Forward suffix requires explicit original terminal raw violations')
    classified=[scope.classify(e,breaks)for e in raw]
    released_pair=any(c.get('scope')=='released_fruit_pair' for c in classified)
    if any(c.get('continuation_authorized')is not True or not (
            c.get('scope')=='released_fruit_pair' or c.get('scope')=='released_fruit_static_farm_support' and
            c.get('static_farm_support_proof',{}).get('structural_role')=='RowSupply')for c in classified):
        raise ValueError('Forward suffix unresolved terminal contact: exact released fruit pair or native-released fruit/static RowSupply required')
    if last_phase=='close' and not released_pair:
        raise ValueError('Close-phase suffix is limited to a proven released-fruit pair')
    flags={}
    if isinstance(summary.get('leaf_poststep_verification'),bool):flags['verify_leaf_contact_after_step']=summary['leaf_poststep_verification']
    solved=_read(trial/'solved_contact_policy.json')
    for argument,key in [('verify_harvest_contact_after_step','verify_current_truss_fruit_contacts'),
            ('verify_background_leaf_enclosures','verify_background_component_hulls'),('verify_structural_scene_query','verify_structural_scene_queries')]:
        if isinstance(solved.get(key),bool):flags[argument]=solved[key]
    for argument,key in [('released_fruit_ground_diagnostic','enabled'),('released_fruit_gripper_support_diagnostic','gripper_support_enabled'),
            ('released_fruit_same_truss_diagnostic','same_truss_enabled'),('released_fruit_same_plant_diagnostic','same_plant_enabled'),
            ('released_fruit_static_gutter_base_diagnostic','static_gutter_base_enabled'),('released_fruit_static_farm_support_diagnostic','static_farm_support_enabled')]:
        if not isinstance(scope_data.get(key),bool):raise ValueError('Forward suffix missing source lifecycle guard: '+key)
        flags[argument]=scope_data[key]
    flags['belt_ramp_seconds']=float((plan.get('belt_spin')or{}).get('ramp_seconds',0.))
    if not isinstance(summary.get('belt_off_control'),bool):raise ValueError('Forward suffix requires recorded source belt on/off control')
    flags['belt_off_control']=summary['belt_off_control']
    if released_pair:flags['released_fruit_pair_diagnostic']=True
    elif 'released_pair_enabled'in scope_data:flags['released_fruit_pair_diagnostic']=scope_data['released_pair_enabled']
    if source_lift_reference is not None:flags['retain_lift_drive_reference']=True
    for key in ('experimental_tpu_fruit_close_2mm','release_relief_before_withdraw','release_relief_minimum_gap_mm','zero_conveyor_diagnostic'):
        if key in runtime:flags[key]=runtime[key]
    return dict(schema='s300-terminal-forward-suffix-v1',checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],source_assembly=str(assembly),source_trial=str(trial),
        source_scene_sha256=manifest['scene_sha256'],source_states=str(states_path),source_states_sha256=_sha(states_path),
        source_summary_sha256=_sha(trial/'summary.json'),source_actual_self_audit_sha256=_sha(trial/'self_collision_actual.json'),
        source_contacts_sha256=_sha(contacts_path),source_break_events_sha256=_sha(trial/'break_events.json'),
        source_plan_path=str(plan_path),source_plan_sha256=plan_sha,source_accepted_plan_certificates=certificates,
        source_runtime=runtime,source_guard_flags=flags,source_trace_end_s=float(metadata['time_s']),
        source_elapsed_in_plan_s=float(elapsed),source_remaining_s=float(remaining),source_plan_total_s=float(total),
        source_checkpoint_tick=last_index,source_cycle_start_s=float(record['start_s']),source_hz=60,
        source_terminal_belt_command_m_s=np.asarray(last_belt).tolist(),source_current_record=copy.deepcopy(record),
        source_native_command_conversion=native_command_binding,
        measured_initial_q=np.asarray(arrays['robot_q'])[ids].tolist(),measured_initial_jaws_m=np.asarray(arrays['robot_q'])[jawids].tolist(),
        measured_body_paths=list(map(str,arrays['body_paths'])),measured_body_poses_world=np.asarray(arrays['body_poses_world']).tolist(),
        original_completed=False,original_stop_reason=summary['stop_reason'],original_stop_preserved=True,
        raw_terminal_violations=raw,reclassified_terminal_violations=classified,
        previous_sequence=copy.deepcopy(sequence),source_lift_drive_reference=source_lift_reference,
        **(dict(source_lift_drive_reference_binding=source_lift_binding) if source_lift_binding is not None else {}),
        no_earlier_checkpoint=True,resuming_without_rollback=True,physical_history_reset=False,
        harvest_attempt=False,new_approach=False,training_eligible=False,harvest_success=None)


def validate_terminal_recovery(assembly,manifest,checkpoint,metadata,arrays):
    """Prove the latest exact trace ends at a safe opening-guard stop."""
    assembly=Path(assembly).resolve();checkpoint=Path(checkpoint).resolve()
    if checkpoint.name!='route_checkpoint':
        raise ValueError('Recovery requires the terminal route_checkpoint; no earlier cycle rollback')
    trial=checkpoint.parent;summary=_read(trial/'summary.json');runtime=_read(trial/'runtime_settings.json')
    if (Path(metadata['source_assembly']).resolve()!=assembly or metadata['manifest']!=manifest or
            metadata['scene_sha256']!=manifest['scene_sha256'] or
            _sha(assembly/'scene.usda')!=manifest['scene_sha256']):
        raise ValueError('Recovery immutable assembly/manifest/SHA mismatch')
    if (summary.get('completed') is not False or summary.get('stop_reason')!='entry_open_not_reached' or
            summary.get('initialization_valid') is not True or
            summary.get('actual_self_collision_audit_passed') is not True or
            summary.get('model_sha256')!=manifest['scene_sha256'] or
            runtime.get('hz')!=60 or runtime.get('contact_limit_mm')!=1. or
            summary.get('contact_limit_mm')!=1. or
            metadata.get('sequence',{}).get('checkpoint_physics_valid')is False):
        raise ValueError('Recovery requires the exact safe 60Hz/1mm terminal entry_open_not_reached stop')
    states_path=trial/'states.npz';audit=_read(trial/'self_collision_actual.json')
    if (audit.get('passed') is not True or audit.get('complete') is not True or
            audit.get('scene_sha256')!=manifest['scene_sha256'] or
            audit.get('input_sha256')!=_sha(states_path) or audit.get('physics_rerun') is not False):
        raise ValueError('Recovery requires the complete exact actual-state self audit')
    with np.load(states_path,allow_pickle=False)as archive:
        states={key:archive[key].copy()for key in ('time','poses','robot_q','command','phase','body_paths')}
    times=np.asarray(states['time'],float);qs=np.asarray(states['robot_q'],float)
    poses=np.asarray(states['poses'],float);commands=np.asarray(states['command'],float)
    phases=np.asarray(states['phase']).astype(str);paths=list(map(str,states['body_paths']))
    names=list(map(str,arrays['robot_dof_names']))
    if (times.ndim!=1 or len(times)<2 or not np.isfinite(times).all() or np.any(np.diff(times)<=0) or
            qs.shape!=(len(times),len(names)) or commands.shape!=qs.shape or
            poses.shape!=(len(times),len(paths),7) or phases.shape!=times.shape or
            not np.isfinite(qs).all() or not np.isfinite(commands).all() or not np.isfinite(poses).all() or
            len(paths)!=len(set(paths)) or len(names)!=len(set(names))):
        raise ValueError('Recovery requires aligned finite strictly timed actual trace')
    indices=[paths.index(str(path))for path in arrays['body_paths']]
    if (abs(float(times[-1])-metadata['time_s'])>1e-8 or
            abs(summary['simulated_s']-metadata['time_s'])>1e-8 or
            not np.array_equal(qs[-1],arrays['robot_q']) or
            not np.array_equal(poses[-1,indices],arrays['body_poses_world'])):
        raise ValueError('Recovery checkpoint is not the latest exact terminal actual state; no rollback')
    record=metadata['sequence']['cycle_plans'][-1]
    if (record.get('status')!='stopped' or record.get('stop_reason')!='entry_open_not_reached' or
            abs(record.get('end_s',-1)-metadata['time_s'])>1e-8):
        raise ValueError('Recovery only reverses the actual last preparation branch before entry')
    camera_prefix=bool(re.fullmatch(r'prepare_camera_ompl_\d{3}',phases[-1]))
    plan_proof=None
    if camera_prefix:
        # A camera connection has several retimed edges, but its complete
        # ACTUAL traversed prefix is one recovery leg. Never include the scan,
        # an unexecuted command edge, a previous checkpoint or the approach.
        selected=np.flatnonzero(np.array([bool(re.fullmatch(r'prepare_camera_ompl_\d{3}',p))
            for p in phases])&(times>=float(record['start_s'])))
        relative=record.get('plan')
        if not isinstance(relative,str) or Path(relative).is_absolute():
            raise ValueError('Camera recovery requires the exact executed relative plan')
        plan_path=(trial/relative).resolve()
        if not plan_path.is_relative_to(trial) or not plan_path.is_file():
            raise ValueError('Camera recovery executed plan is missing or outside its trial')
        plan=_read(plan_path);plan_sha=_sha(plan_path)
        connection=record.get('camera_prefix_connection',{})
        report_path=Path(connection.get('report','')).resolve()
        if not report_path.is_relative_to(plan_path.parent)or not report_path.is_file():
            raise ValueError('Camera recovery requires its exact accepted connection certificate')
        certificate=_read(report_path)
        if (certificate.get('verified_plan_sha256')!=plan_sha or
                certificate.get('status')!='full_audits_passed' or
                certificate.get('eligible_for_native_execution')is not True or
                certificate.get('full_hz')!=60 or
                certificate.get('full_self',{}).get('passed')is not True or
                certificate.get('full_self',{}).get('complete')is not True or
                certificate.get('full_environment',{}).get('passed')is not True or
                certificate.get('full_environment',{}).get('complete')is not True):
            raise ValueError('Camera recovery executed plan SHA mismatch')
        rows=plan.get('phases',[])
        names_forward=[p.get('name')for p in rows if re.fullmatch(r'prepare_camera_ompl_\d{3}',p.get('name',''))]
        observed=[]
        for label in phases[selected]:
            if not observed or label!=observed[-1]:observed.append(label)
        if (not names_forward or names_forward!=['prepare_camera_ompl_'+f'{i:03d}'for i in range(len(names_forward))] or
                not observed or observed!=names_forward[:len(observed)] or len(observed)>len(names_forward)):
            raise ValueError('Camera recovery requires the contiguous actual prefix from edge000 without skipped/repeated edges')
        feedback=record.get('preparation_open_feedback',{})
        expected_failed=(names_forward[len(observed)]if len(observed)<len(names_forward)else'approach')
        failed=feedback.get(expected_failed,{})
        if (failed.get('reached')is not False or not np.isfinite(failed.get('t',float('nan'))) or
                abs(failed['t']-metadata['time_s'])>1e-8 or
                any(feedback.get(name,{}).get('reached')is not True for name in observed)):
            raise ValueError('Camera recovery requires the exact next-edge opening guard and earlier passed gates')
        opening=plan.get('preparation_opening',{})
        if (opening.get('required_gap_m')!=.096 or opening.get('tracking_tolerance_m')!=.0005 or
                not set(names_forward+['approach'])<=set(opening.get('feedback_before_phases',[])) or
                any(p.get('belt_local_y_m_s')!=[0.,0.]or p.get('end_gap_m')!=.096
                    for p in rows if p.get('name')in names_forward)):
            raise ValueError('Camera recovery must retain the original fully-open guarded belt-off prefix')
        qvel=np.asarray(arrays.get('robot_qvel'),float)
        if qvel.shape!=(len(names),)or not np.isfinite(qvel).all():
            raise ValueError('Camera terminal recovery requires complete measured native robot qvel')
        plan_proof=dict(path=str(plan_path),sha256=plan_sha,
            accepted_connection_report=str(report_path),accepted_connection_report_sha256=_sha(report_path),actual_edges=observed,
            failed_next_phase=expected_failed,unexecuted_edges=names_forward[len(observed):],
            scan_or_manipulation_reversed=False,measured_terminal_robot_qvel=qvel.tolist())
    else:
        if (phases[-1]!='prepare_joint_branch' or
                record.get('preparation_open_feedback',{}).get('approach',{}).get('reached')is not False):
            raise ValueError('Recovery only reverses the actual last preparation branch before entry')
        selected=np.flatnonzero((phases=='prepare_joint_branch')&(times>=float(record['start_s'])))
    names_expected=['farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3']
    if not set(names_expected+['jaw_L','jaw_R'])<=set(names):
        raise ValueError('Recovery native joint mapping is incomplete')
    controller=metadata.get('controller_state',{})
    targets=np.asarray(controller.get('position_targets'),float)
    if (controller.get('position_source')!='native_get_dof_position_targets' or
            controller.get('robot_dof_names')!=names or targets.shape!=(len(names),) or
            not np.isfinite(targets).all() or
            not np.allclose(targets[[names.index('jaw_L'),names.index('jaw_R')]],[-.026,.026],rtol=0,atol=1e-9)):
        raise ValueError('Recovery requires the retained separate native fully-open jaw targets')
    if (len(selected)<2 or selected[-1]!=len(times)-1 or
            not np.array_equal(selected,np.arange(selected[0],len(times)))):
        raise ValueError('Recovery requires one contiguous terminal actual preparation branch')
    start=max(0,int(selected[0])-1)
    rids=[names.index(name)for name in names_expected]
    guard_flags={}
    if isinstance(summary.get('leaf_poststep_verification'),bool):
        guard_flags['verify_leaf_contact_after_step']=summary['leaf_poststep_verification']
    solved_path=trial/'solved_contact_policy.json'
    if solved_path.is_file():
        solved=_read(solved_path)
        for argument,key in [('verify_harvest_contact_after_step','verify_current_truss_fruit_contacts'),
                ('verify_background_leaf_enclosures','verify_background_component_hulls'),
                ('verify_structural_scene_query','verify_structural_scene_queries')]:
            if isinstance(solved.get(key),bool):guard_flags[argument]=solved[key]
    return dict(schema='s300-terminal-opening-guard-recovery-v1',
        checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        source_assembly=str(assembly),source_trial=str(trial),source_scene_sha256=manifest['scene_sha256'],
        source_states=str(states_path),source_states_sha256=_sha(states_path),
        source_summary_sha256=_sha(trial/'summary.json'),source_actual_self_audit_sha256=_sha(trial/'self_collision_actual.json'),
        source_runtime=runtime,source_guard_flags=guard_flags,source_trace_first_index=start,source_trace_last_index=len(times)-1,
        source_lift_drive_reference=copy.deepcopy(summary.get('lift_drive_reference')),
        recovery_controller_binding_pending=(summary.get('lift_drive_reference')is not None or
            runtime.get('retain_lift_drive_reference')is True),
        source_preparation_kind='camera_ompl_prefix'if camera_prefix else'legacy_joint_branch',
        camera_prefix_proof=plan_proof,
        source_trace_start_s=float(times[start]),source_trace_end_s=float(times[-1]),
        reverse_actual_waypoint_q=qs[start:,rids][::-1].tolist(),
        retained_jaw_position_targets_m=targets[[names.index('jaw_L'),names.index('jaw_R')]].tolist(),
        measured_terminal_jaws_m=np.asarray(arrays['robot_q'])[[names.index('jaw_L'),names.index('jaw_R')]].tolist(),
        previous_sequence=copy.deepcopy(metadata['sequence']),
        no_earlier_checkpoint=True,physical_history_reset=False,harvest_attempt=False,
        training_eligible=False,harvest_success=None)


def recovery_plan(proof):
    """Reverse measured seven-DOF waypoints with existing interpolation."""
    if proof.get('schema')=='s300-terminal-forward-suffix-v1':
        from connect_s300_prefix import reconstruct_phase_tuples
        path=Path(proof['source_plan_path'])
        if _sha(path)!=proof['source_plan_sha256']:raise ValueError('Forward suffix original source plan SHA mismatch')
        metadata=_read(path);phases=reconstruct_phase_tuples(metadata)
        metadata['start']=copy.deepcopy(proof['measured_initial_q'])
        metadata['recovery']=copy.deepcopy(proof)
        metadata['terminal_forward_suffix']=dict(schema='s300-terminal-forward-suffix-clock-v1',
            initial_source_elapsed_s=proof['source_elapsed_in_plan_s'],remaining_duration_s=proof['source_remaining_s'],
            source_checkpoint_tick=proof['source_checkpoint_tick'],source_cycle_start_s=proof['source_cycle_start_s'],source_hz=proof['source_hz'],
            proof=copy.deepcopy(proof),command_clock='Original cycle phase clock; no retiming or interpolated truncation',
            initial_actual_state_source='Exact terminal native checkpoint, independent of nominal command')
        metadata.update(harvest_attempt=False,new_approach=False,training_eligible=False,harvest_success=None)
        return phases,metadata
    if proof.get('schema')!='s300-terminal-opening-guard-recovery-v1':
        raise ValueError('Explicit validated terminal recovery proof required')
    qs=np.asarray(proof['reverse_actual_waypoint_q'],float)
    span=float(proof['source_trace_end_s']-proof['source_trace_start_s'])
    if qs.ndim!=2 or qs.shape[1]!=7 or len(qs)<2 or not np.isfinite(qs).all() or not np.isfinite(span) or span<=0:
        raise ValueError('Finite complete actual reverse branch required')
    # Existing smoothstep interpolation peaks at1.5x average. A1.5 duration
    # scale keeps its maximum progress speed no faster than the source trace.
    endpoint=np.array([qs[-1],qs[-1]])
    # A stationary initial sample also lets the unchanged legacy self checker
    # reach the96mm held gap before it audits any moving reverse waypoint.
    phases=[('recovery_initial_hold',1./60,np.array([qs[0],qs[0]]),.096,0.,0.),
            ('recovery_reverse',1.5*span,qs.copy(),.096,0.,0.),
            ('recovery_endpoint_hold',1.,endpoint.copy(),.096,0.,0.),
            ('recovery_stationary_open',1.,endpoint.copy(),.096,0.,0.),
            ('recovery_open_hold',.5,endpoint.copy(),.096,0.,0.)]
    metadata=dict(start=qs[0].tolist(),phases=[dict(name=name,duration_s=duration,
        waypoint_q=q.tolist(),end_gap_m=gap,belt_local_y_m_s=[left,right])for name,duration,q,gap,left,right in phases],
        recovery=copy.deepcopy(proof),initial_gap_m=.096,
        timing='All actual measured preparation samples reversed; existing smoothstep at1.5 source duration',
        harvest_attempt=False,training_eligible=False,harvest_success=None)
    return phases,metadata


def validate_suffix_plan_binding(plan):
    """Bind the opt-in sampler to every unchanged original command field."""
    suffix=plan.get('terminal_forward_suffix',{});proof=suffix.get('proof',{})
    if (suffix.get('schema')!='s300-terminal-forward-suffix-clock-v1' or proof.get('schema')!='s300-terminal-forward-suffix-v1' or
            suffix.get('initial_source_elapsed_s')!=proof.get('source_elapsed_in_plan_s') or
            suffix.get('remaining_duration_s')!=proof.get('source_remaining_s') or
            any(suffix.get(k)!=proof.get(k)for k in ('source_checkpoint_tick','source_cycle_start_s','source_hz')) or
            plan.get('start')!=proof.get('measured_initial_q')):
        raise ValueError('Terminal suffix plan/proven clock or measured initial state mismatch')
    path=Path(proof['source_plan_path'])
    if _sha(path)!=proof['source_plan_sha256']:raise ValueError('Terminal suffix source plan SHA mismatch')
    original=_read(path);current=copy.deepcopy(plan)
    permitted={'start','recovery','terminal_forward_suffix','harvest_attempt','new_approach','training_eligible','harvest_success',
        'resume_rail_origin','rail_motion_scope'}
    if len(current.get('phases',[]))!=len(original.get('phases',[])):
        raise ValueError('Terminal suffix phase count changed')
    displacement=current.get('resume_rail_origin',{}).get('displacement_m')
    for source,row in zip(original['phases'],current['phases']):
        if 'rail_waypoint_positions_m'in row and displacement is not None:
            if row['rail_waypoint_positions_m']!=[displacement]*len(row['waypoint_q']):
                raise ValueError('Terminal suffix rail checker geometry is not the exact measured hold')
            row['rail_waypoint_positions_m']=copy.deepcopy(source.get('rail_waypoint_positions_m'))
    for obj in (original,current):
        for key in permitted:obj.pop(key,None)
    if current!=original:
        raise ValueError('Terminal suffix altered original command, belt, jaw, control or phase metadata')
    return proof


def require_same_recovery_settings(proof,settings):
    """The explicit retreat does not authorize different physics or guards."""
    source=proof['source_runtime']
    binding=proof.get('source_lift_drive_reference_binding',{})
    reference_path=Path(binding.get('path',''))
    if (source.get('retain_lift_drive_reference') is True or proof.get('source_lift_drive_reference') is not None) and not (
            proof.get('schema')=='s300-terminal-forward-suffix-v1' and
            settings.get('resume_original_cycle_suffix')is True and settings.get('retain_lift_drive_reference')is True and
            reference_path.is_file() and binding.get('sha256')==_sha(reference_path)):
        # The current recovery runner has no verified source-reference command
        # binding. Matching flags (or a diagnostic plan) cannot authorize it to
        # silently replace the retained lift drive target with measured q.
        raise ValueError('Recovery controller binding pending: retained source lift drive reference is not supported by the current runner')
    for key in ('hz','solver','device','threads','position_iterations','velocity_iterations',
                'contact_limit_mm','fruit_contact_limit_mm','fruit_fruit_limit_mm',
                'plant_dynamics','collision_filter','isolate_background','contact_backend'):
        if settings.get(key)!=source.get(key):
            raise ValueError('Recovery must preserve source physics/guard setting: '+key)
    for key,value in proof.get('source_guard_flags',{}).items():
        if settings.get(key)!=value:
            raise ValueError('Recovery must preserve source solved-contact guard: '+key)
