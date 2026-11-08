"""Opt-in bounded connection of an exact resumed camera preparation state.

This uses the established OMPL, retiming and full FCL audits.  It neither
imports a fresh-campaign fiction nor steps physics.  Native callers use the
external asset-Python boundary below so their USD/FCL ABI stays untouched.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np

SOURCE_BYTES = Path(__file__).read_bytes()


def required_contact_policy(allowed_truss_key):
    return dict(allowed_truss_key=allowed_truss_key,
        released_fruit_collection_diagnostic=True,released_fruit_gripper_support_diagnostic=True,
        released_fruit_same_truss_diagnostic=True,wheel_floor_overlap_limit_m=.001,
        required_native_penetration_limit_m=.001,self_clearance_m=.002,
        native_penetration_guard_enforced_by='existing native runner')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _read(path):
    return json.loads(Path(path).read_text())


def _write(path, value):
    with Path(path).open('x') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False)+'\n')


def connection_blocks(plan):
    """Find only the existing contiguous preparation and its exact return."""
    phases = plan.get('phases', [])
    names = [p.get('name') for p in phases]
    for name in ('prepare_joint_branch', 'return_joint_branch'):
        if names.count(name) != 1:
            raise ValueError('Exactly one preparation and return joint branch required')
    begin = joint = names.index('prepare_joint_branch')
    back = end = names.index('return_joint_branch')
    if names.count('prepare_lift_branch') != names.count('return_lift_branch'):
        raise ValueError('Preparation lift requires its matching reversed return')
    if names.count('prepare_lift_branch') > 1:
        raise ValueError('Ambiguous preparation lift branch')
    if 'prepare_lift_branch' in names:
        begin, end = names.index('prepare_lift_branch'), names.index('return_lift_branch')
        if begin+1 != joint or back+1 != end:
            raise ValueError('Only contiguous existing lift/joint connector legs may change')
    if not begin <= joint < back <= end:
        raise ValueError('Preparation must precede its return')
    middle = names[joint+1:back]
    if 'approach' not in middle or 'withdraw' not in middle:
        raise ValueError('An unchanged approach through withdraw suffix is required')

    def joined(rows):
        result = []
        for row in rows:
            q = np.asarray(row['waypoint_q'], float)
            if (q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or
                    not np.isfinite(q).all()):
                raise ValueError('Finite seven-DOF connector waypoints required')
            if result and not np.array_equal(result[-1], q[0]):
                raise ValueError('Connector legs are not continuous')
            result.extend(q if not result else q[1:])
            if row.get('belt_local_y_m_s') != [0., 0.]:
                raise ValueError('Preparation and return belts must remain off')
        return np.asarray(result)

    forward = joined(phases[begin:joint+1])
    reverse = joined(phases[back:end+1])
    if not np.array_equal(forward, reverse[::-1]):
        raise ValueError('Existing return must exactly reverse lift/joint preparation')
    if not np.array_equal(np.asarray(plan.get('start'), float), phases[0]['waypoint_q'][0]):
        raise ValueError('Plan start differs from first actual command')
    # The measured rail coordinate and retained drive target are distinct.
    # Both must be finite constant holds throughout the changed legs.
    holds = {}
    for key in ('rail_waypoint_positions_m', 'rail_native_command_positions_m'):
        values = []
        for row in phases[begin:joint+1]+phases[back:end+1]:
            value = np.asarray(row.get(key), float)
            if value.shape != (len(row['waypoint_q']),) or not np.isfinite(value).all():
                raise ValueError('Exact finite rail geometry and command holds required')
            values.extend(value.tolist())
        if any(x != values[0] for x in values):
            raise ValueError('A camera connector cannot introduce rail travel')
        holds[key] = values[0]
    return dict(begin=begin, joint=joint, back=back, end=end,
                start=forward[0], goal=forward[-1], **holds)


def lift_then_arm_path(start, goal):
    """Raise/lower the real carriage first, then move arm to unchanged staging."""
    start, goal = np.asarray(start,float), np.asarray(goal,float)
    if start.shape != (7,) or goal.shape != (7,) or not np.isfinite([start,goal]).all():
        raise ValueError('Finite seven-DOF actual start and staging goal required')
    carriage = start.copy(); carriage[0] = goal[0]
    return np.stack((start,carriage,goal))


def verify_rail_holds(plan, origin):
    """A proposed connector must keep the measured geometry and actual command."""
    block = connection_blocks(plan)
    if (block['rail_waypoint_positions_m'] != origin['displacement_m'] or
            block['rail_native_command_positions_m'] != origin['native_position_target_m']):
        raise ValueError('Connector rail hold does not match measured geometry and retained native command')
    return dict(measured_geometry_displacement_m=origin['displacement_m'],
        retained_native_command_m=origin['native_position_target_m'],
        checkpoint_metadata_sha256=origin['checkpoint_metadata_sha256'],
        requested_new_rail_travel=False)


def proposal_steps(resolution='fine'):
    """Only proposal resolution changes; strict acceptance is always fine."""
    if resolution not in ('fine','coarse'):
        raise ValueError('Proposal resolution must be fine or coarse')
    return np.array([.002]+[np.deg2rad(1.)]*6 if resolution=='fine' else
                    [.006]+[np.deg2rad(4.)]*6)


def fine_path_check(path, bounds, state_valid):
    """Independently certify every edge at2mm/1degree with its own budget."""
    from connect_s300_prefix import JointSpace, CheckBudget, check_polyline
    budget=CheckBudget(10000,120.)
    steps=proposal_steps('fine');space=JointSpace(bounds,steps)
    def valid(q):
        budget.tick()
        return state_valid(q)
    try:
        result=check_polyline(path,space,valid)
        result['complete']=bool(result['passed'])
    except Exception as exc:
        result=dict(passed=False,complete=False,error=type(exc).__name__+': '+str(exc))
    result.update(state_checks=budget.checks,max_state_checks=10000,budget_seconds=120.,
        wall_s=120.-budget.remaining(),steps_m_or_rad=steps.tolist(),
        same_state_predicate=True,proposal_resolution_is_not_certificate=True)
    return result


def splice_camera_connection(plan, path, velocity=None, *, hz=60):
    """Replace only both connector legs, preserving all manipulation commands."""
    from connect_s300_prefix import timed_edges, reconstruct_phase_tuples
    if hz != 60:
        raise ValueError('This diagnostic preserves the existing60Hz physical command clock')
    block = connection_blocks(plan)
    path = np.asarray(path, float)
    if (path.ndim != 2 or path.shape[1] != 7 or len(path) < 2 or
            not np.isfinite(path).all() or not np.array_equal(path[0], block['start']) or
            not np.array_equal(path[-1], block['goal'])):
        raise ValueError('Connection must retain exact measured source and staging endpoints')
    caps = np.asarray([.08]+[.35]*6 if velocity is None else velocity, float)
    phases = plan['phases']; begin, joint, back, end = (block[k] for k in ('begin','joint','back','end'))
    forward = timed_edges(path, caps, hz=hz, name='prepare_camera_ompl', gap=phases[joint]['end_gap_m'])
    reverse = timed_edges(path[::-1], caps, hz=hz, name='return_camera_ompl', gap=phases[back]['end_gap_m'])
    for row in forward+reverse:
        for key in ('rail_waypoint_positions_m', 'rail_native_command_positions_m'):
            row[key] = [block[key]]*len(row['waypoint_q'])
    result = copy.deepcopy(plan)
    result['phases'] = (copy.deepcopy(phases[:begin])+forward+
        copy.deepcopy(phases[joint+1:back])+reverse+copy.deepcopy(phases[end+1:]))
    opening = result.get('preparation_opening')
    if (not isinstance(opening,dict) or
            not isinstance(opening.get('feedback_before_phases'),list) or
            any(not isinstance(name,str) for name in opening['feedback_before_phases']) or
            opening.get('required_gap_m') != .096 or
            any(row['end_gap_m'] != .096 for row in forward)):
        raise ValueError('Existing measured96mm opening policy is required before every new connector leg')
    replaced_names = {row['name'] for row in phases[begin:joint+1]}
    # run_motion guards exact names BEFORE sending the next motion command.
    # Renaming a connector without rebinding this list would bypass its guard.
    # Retain every unrelated original requirement and all numerical settings.
    old_required = [name for name in opening['feedback_before_phases'] if name not in replaced_names]
    opening['feedback_before_phases'] = list(dict.fromkeys(
        [row['name'] for row in forward]+old_required))
    if result.get('belt_cycle') is not None:
        window = result['belt_cycle']; names = window.get('phases', [])
        indices = [i for i,p in enumerate(result['phases']) if p['name'] in names]
        if (not names or not indices or indices != list(range(indices[0],indices[-1]+1)) or
                [result['phases'][i]['name'] for i in indices] != names):
            raise ValueError('Original contiguous belt phase window must remain unchanged')
        delta = sum(p['duration_s'] for p in forward)-sum(p['duration_s'] for p in phases[begin:joint+1])
        expected = plan['belt_cycle']['start_after_settle_s']+delta
        actual = sum(p['duration_s'] for p in result['phases'][:indices[0]])
        duration = sum(result['phases'][i]['duration_s'] for i in indices)
        if not math.isclose(expected, actual, abs_tol=1e-10, rel_tol=0) or not math.isclose(
                window['duration_s'], duration, abs_tol=1e-10, rel_tol=0):
            raise ValueError('Retimed belt clock disagrees with unchanged manipulation phases')
        window['start_after_settle_s'] = expected
    for key in ('ready_lift_branch', 'joint_ready_connector'):
        if key in result:
            result['replaced_original_'+key] = result.pop(key)
    result['camera_prefix_connection'] = dict(
        replaced_forward_phases=[p['name'] for p in phases[begin:joint+1]],
        replaced_return_phases=[p['name'] for p in phases[back:end+1]],
        exact_source_q=block['start'].tolist(), exact_staging_q=block['goal'].tolist(),
        original_manipulation_commands_unchanged=True, rail_geometry_hold_m=block['rail_waypoint_positions_m'],
        retained_rail_command_m=block['rail_native_command_positions_m'],
        native_execution_performed=False, training_eligible=False)
    reconstruct_phase_tuples(result)
    return result


def validate_checkpoint_provenance(assembly, manifest, metadata, arrays, plan, names):
    """Bind the measured start to this exact completed immutable assembly."""
    from resume_native_state import require_completed_checkpoint
    require_completed_checkpoint(metadata)
    if (Path(metadata['source_assembly']).resolve() != Path(assembly).resolve() or
            metadata['scene_sha256'] != manifest['scene_sha256'] or metadata['manifest'] != manifest or
            metadata['source_plant'] != manifest['plant']['source_plant']):
        raise ValueError('Camera connection requires the exact same-model checkpoint and manifest')
    dofs = list(map(str, arrays['robot_dof_names']))
    if not set(names) <= set(dofs):
        raise ValueError('Checkpoint omits a measured lift/arm joint')
    measured = np.asarray(arrays['robot_q'])[[dofs.index(n) for n in names]]
    if not np.array_equal(np.asarray(plan['start']), measured):
        raise ValueError('Camera connection start is not the exact measured checkpoint q')
    return dict(checkpoint_metadata_sha256=metadata['metadata_sha256'],
        checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],
        exact_measured_start_q=measured.tolist(), source_assembly=str(Path(assembly).resolve()),
        current_materialized_dependency=manifest['compact_assets']['materialized_dependency'])


def connect_camera_prefix(assembly, plan, checkpoint, output, *, allowed_truss_key,
                          native_tool_cache, solve_seconds=30., hz=60, seed=2026,
                          joint_path=None, strategy='ompl', proposal_resolution='fine'):
    """One bounded existing OMPL query (or a supplied exact path), then full audit.

    Each call records failure evidence and returns None unless every full audit
    completed and passed. Callers must enforce their campaign query budget.
    """
    if (hz != 60 or isinstance(seed, bool) or seed not in (2026,2027,2028) or
            not math.isfinite(solve_seconds) or not 0 < solve_seconds <= 30.):
        raise ValueError('60Hz and one approved seed with a solve budget in(0,30]seconds required')
    if not allowed_truss_key or not isinstance(allowed_truss_key, str):
        raise ValueError('Exact allowed truss identity required')
    if strategy not in ('ompl','lift-then-arm') or (joint_path is not None and strategy != 'ompl'):
        raise ValueError('Choose an existing OMPL query/path or one explicit lift-then-arm connection')
    steps=proposal_steps(proposal_resolution)
    assembly, plan, checkpoint, output, native_tool_cache = map(lambda p:Path(p).resolve(),
        (assembly, plan, checkpoint, output, native_tool_cache))
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema='s300-resumed-camera-prefix-connection-v1', status='setup_pending',
        source_assembly=str(assembly), source_plan=str(plan), source_checkpoint=str(checkpoint),
        seed=seed, solve_seconds=solve_seconds, strategy=strategy,
        proposal_resolution=proposal_resolution,proposal_steps_m_or_rad=steps.tolist(),
        independent_fine_steps_m_or_rad=proposal_steps('fine').tolist(),
        proposal_budget=dict(max_state_checks=5000,seconds=solve_seconds),
        independent_fine_budget=dict(max_state_checks=10000,seconds=120.),
        contact_policy=required_contact_policy(allowed_truss_key),
        maximum_queries=1 if strategy=='ompl' and joint_path is None else 0,
        full_hz=hz, physics_steps=0, training_eligible=False, harvest_success=None,
        eligible_for_native_execution=False, fresh_campaign_metadata_created=False)
    code = output/'code';code.mkdir()
    (code/'camera_prefix_connection.py').write_bytes(SOURCE_BYTES)
    report['code_sha256'] = _sha(code/'camera_prefix_connection.py')
    started = time.perf_counter(); budget = None; protected = {}
    try:
        from pxr import Usd
        from compact_motion_assets import verify_compact_dependencies
        from connect_s300_prefix import existing_ompl
        from approach_connection import CheckBudget, JointSpace, LimitReached, ompl_connect, check_polyline
        from motion_kinematics import ToolKinematics
        from motion_self_collision import MotionSelfCollision, check_plan
        from native_route_checkpoint import load_checkpoint
        from rail_environment_preflight import (RailEnvironmentCheck, start_checkpoint,
            verified_release_context, verify_initial_alignment, audit_rail_transit)
        source = _read(plan); manifest = _read(assembly/'motion_manifest.json')
        if _sha(assembly/'scene.usda') != manifest['scene_sha256']:
            raise ValueError('Source scene SHA mismatch')
        verify_compact_dependencies(assembly, manifest)
        metadata, arrays = load_checkpoint(checkpoint)
        block = connection_blocks(source)
        protected = {str(p):_sha(p) for p in [plan,native_tool_cache,assembly/'motion_manifest.json',
            assembly/'scene.usda',assembly/'composed.usdc',checkpoint/'checkpoint.json',checkpoint/'checkpoint.npz']}
        if joint_path is not None:protected[str(Path(joint_path).resolve())] = _sha(joint_path)
        report['source_file_sha256'] = protected
        report['native_tool_cache']=dict(path=str(native_tool_cache),sha256=protected[str(native_tool_cache)])
        stage = Usd.Stage.Open(str(assembly/'scene.usda')); kin = ToolKinematics(stage)
        report['checkpoint_provenance'] = validate_checkpoint_provenance(
            assembly, manifest, metadata, arrays, source, kin.names)
        from robot_controller_state import resolve_robot_controller_state
        from motion_continuation import measured_resume_rail_origin
        origin = measured_resume_rail_origin(manifest,dict(metadata=metadata,arrays=arrays,
            controller_state=resolve_robot_controller_state(metadata,arrays,checkpoint)))
        if origin is None:raise ValueError('Measured physical rail checkpoint required')
        report['rail_hold_proof'] = verify_rail_holds(source,origin)
        moving = MotionSelfCollision(stage, kin, clearance_m=.002,
            native_tool_cache=native_tool_cache, assembly=assembly)
        report['native_tool_hull_import'] = moving.metadata['native_tool_hull_import']
        poses, proof = start_checkpoint(assembly, manifest, checkpoint)
        report['initial_alignment'] = verify_initial_alignment(moving, source, manifest, poses,
            checkpoint_robot_state=proof)
        release = verified_release_context(stage, manifest, proof, collection_enabled=True,
            gripper_support_enabled=True, same_truss_enabled=True)
        broken = {e['joint'] for e in proof.get('native_break_events', [])}
        released = {f['path']+'/Collider' for f in manifest['plant']['fruits'] if f['joint'] in broken}
        environment = RailEnvironmentCheck(stage, manifest, moving, allowed_truss_key=allowed_truss_key,
            checkpoint_poses=poses, max_distance_queries=1200000, release_context=release,
            released_colliders=released, scope='full',wheel_floor_overlap_limit_m=.001)
        space = JointSpace(kin.bounds, steps)
        budget = CheckBudget(5000, solve_seconds)
        rejections = []; gaps = sorted({float(source['phases'][i]['end_gap_m'])
            for i in (block['joint'],block['back'])})
        def valid_state(q):
            for gap in gaps:
                hits = moving.check(q, gap, rail_position_m=block['rail_waypoint_positions_m'])
                if hits:
                    if len(rejections)<64:rejections.append(dict(kind='self',q=np.asarray(q).tolist(),gap_m=gap,hits=hits))
                    return False
                checked = environment.check_state(q, gap, rail_position_m=block['rail_waypoint_positions_m'],
                    phase='camera_resumed_prefix_search')
                if not checked['complete']:
                    raise RuntimeError('Environment arbitrary-state audit incomplete: '+str(checked.get('error')))
                if not checked['hard_clear']:
                    if len(rejections)<64:rejections.append(dict(kind='environment',q=np.asarray(q).tolist(),gap_m=gap,check=checked))
                    return False
            return True
        def valid(q):
            budget.tick()
            return valid_state(q)
        if strategy == 'lift-then-arm':
            path = lift_then_arm_path(block['start'], block['goal'])
            report['route'] = dict(status='explicit_lift_then_arm_requires_full_reaudit',
                physical_carriage_only_first=True,TCP_hold_claim=False)
        elif joint_path is None:
            util, backend = existing_ompl(); util.RNG.setSeed(seed); report['ompl'] = backend
            route = ompl_connect(block['start'], block['goal'], space, valid,
                solve_seconds=max(.001,budget.remaining()), range_fraction=.1)
            report['route'] = {k:v for k,v in route.items() if k!='path'}
            path = route.get('path')
            if path is not None and route.get('status') != 'exact':
                raise ValueError('Approximate OMPL paths may never become connection candidates')
        else:
            loaded = _read(joint_path); path = loaded['path'] if isinstance(loaded,dict) else loaded
            report['route'] = dict(status='supplied_exact_path_requires_full_reaudit',source=str(Path(joint_path).resolve()))
        report.update(state_checks=budget.checks,proposal_state_checks=budget.checks,
            proposal_wall_s=solve_seconds-budget.remaining(),rejected_state_examples=rejections,
            proposal_environment_queries=environment.queries)
        if path is None:
            report['status'] = 'finite_search_no_path_not_impossible'
            return None
        # Supplied paths cannot bypass endpoint, finite, bounds or edge checks.
        revised = splice_camera_connection(source, path, hz=hz)
        if not all(space.contains(q) for q in np.asarray(path)):
            raise ValueError('Connected path exceeds existing native joint bounds')
        _write(output/'joint_path.json', dict(path=np.asarray(path).tolist()))
        # Coarse proposals, fine proposals, manual paths and imported paths all
        # need a separate fine edge certificate with the identical predicate.
        before_fine_queries=environment.queries
        report['independent_edge_check']=fine_path_check(np.asarray(path),kin.bounds,valid_state)
        report['independent_fine_environment_queries']=environment.queries-before_fine_queries
        report['environment_queries']=environment.queries
        if not report['independent_edge_check']['passed'] or not report['independent_edge_check']['complete']:
            raise ValueError('Connection failed independent fine edge verification')
        prefix_duration = sum(p['duration_s'] for p in revised['phases'] if p['name'].startswith('prepare_camera_ompl_'))
        if prefix_duration > 120.:
            raise LimitReached('existing_prefix_duration_budget')
        report['prefix_duration_s'] = prefix_duration
        _write(output/'connected_plan.json', revised)
        self_audit = check_plan(moving, revised, hz=hz)
        self_audit.update(scene_sha256=manifest['scene_sha256'],
            input_sha256=_sha(output/'connected_plan.json'),physics_rerun=False)
        _write(output/'full_self60.json', self_audit)
        report.setdefault('audit_file_sha256',{})['full_self60.json']=_sha(output/'full_self60.json')
        report['full_self'] = {k:self_audit[k] for k in ('passed','complete','samples')}
        if not self_audit['passed'] or not self_audit['complete']:
            report['status'] = 'full_self_blocked_or_incomplete'
            return None
        env_report = audit_rail_transit(assembly, output/'connected_plan.json', checkpoint=checkpoint,
            output=output/'full_environment60.json', hz=hz, native_hz=hz, scope='full',
            native_tool_cache=native_tool_cache, allowed_truss_key=allowed_truss_key,
            max_distance_queries=1200000, released_fruit_collection_diagnostic=True,
            released_fruit_gripper_support_diagnostic=True, released_fruit_same_truss_diagnostic=True,
            stop_on_first_hard_hit=True,wheel_floor_overlap_limit_m=.001)
        report['full_environment'] = {k:env_report[k] for k in ('passed','complete','status')}
        report['audit_file_sha256']['full_environment60.json']=_sha(output/'full_environment60.json')
        report['eligible_for_native_execution'] = bool(env_report['passed'] and env_report['complete'] and
            env_report.get('eligible_for_native_execution'))
        report['status'] = 'full_audits_passed' if report['eligible_for_native_execution'] else 'full_environment_blocked_or_incomplete'
        if report['eligible_for_native_execution']:
            if not all(_sha(path)==digest for path,digest in protected.items()):
                raise ValueError('Source SHA changed during connection audit')
            report['verified_plan'] = str(output/'connected_plan.json')
            report['verified_plan_sha256'] = _sha(output/'connected_plan.json')
            return output/'connected_plan.json'
        return None
    except Exception as exc:
        report.update(status='finite_budget_or_connection_error', error=type(exc).__name__+': '+str(exc))
        return None
    finally:
        report['source_files_unchanged'] = all(_sha(path)==digest for path,digest in protected.items())
        if not report['source_files_unchanged']:
            report.update(status='source_sha_changed', eligible_for_native_execution=False)
        if budget is not None:report['state_checks'] = budget.checks
        report['wall_s'] = time.perf_counter()-started
        _write(output/'report.json', report)


def connect_external_camera_prefix(assembly, plan, checkpoint, output, **flags):
    """Run only the existing asset Python; return an immutable audited plan."""
    output = Path(output).resolve(); log = output.with_suffix('.log')
    if output.exists() or log.exists():raise FileExistsError(output)
    required = ('allowed_truss_key', 'native_tool_cache')
    if any(flags.get(key) is None for key in required):raise ValueError('Exact truss and native-tool cache required')
    allowed = set(required)|{'solve_seconds','hz','seed','joint_path','strategy','proposal_resolution'}
    if set(flags)-allowed:raise ValueError('Unknown camera connection flags: '+str(sorted(set(flags)-allowed)))
    python = Path(__file__).resolve().parents[1]/'.asset-venv/bin/python'
    command = [str(python),str(Path(__file__).resolve()),str(assembly),'--plan',str(plan),
        '--checkpoint',str(checkpoint),'--output',str(output)]
    for key,value in flags.items():
        if value is not None:command += ['--'+key.replace('_','-'),str(value)]
    env = os.environ.copy()
    for key in ('PYTHONHOME','PYTHONPATH','PXR_PLUGINPATH_NAME'):env.pop(key,None)
    process = subprocess.run(command,env=env,capture_output=True,text=True)
    log.parent.mkdir(parents=True,exist_ok=True)
    with log.open('x') as stream:stream.write(process.stdout+process.stderr)
    report_path = output/'report.json'
    if process.returncode != 0 or not report_path.exists():
        raise ValueError('Camera prefix connector failed; see '+str(log))
    report = _read(report_path)
    if (report.get('status') != 'full_audits_passed' or report.get('source_files_unchanged') is not True or
            report.get('eligible_for_native_execution') is not True):
        raise ValueError('Camera prefix connection not completely qualified; see '+str(report_path))
    result = Path(report['verified_plan'])
    if result.resolve() != output/'connected_plan.json' or _sha(result) != report['verified_plan_sha256']:
        raise ValueError('Qualified connected plan path/SHA mismatch')
    return result


def reuse_verified_camera_audits(connected_plan, output_self, output_environment, *,
                                 checkpoint, native_tool_cache):
    """Byte-copy exact complete certificates; never issue a new physics/audit claim.

    The caller must use these exact connected-plan bytes for execution. Every
    hashed original source, checkpoint, native cache and certificate must still
    exist unchanged. Old DATA diagnostics without the final guard policy and
    this helper's complete binding are intentionally ineligible.
    """
    connected_plan, checkpoint, native_tool_cache = map(lambda p:Path(p).resolve(),
        (connected_plan,checkpoint,native_tool_cache))
    output_self, output_environment = map(lambda p:Path(p).resolve(),(output_self,output_environment))
    if output_self==output_environment or output_self.exists() or output_environment.exists():
        raise ValueError('Audit reuse requires two distinct new immutable output files')
    folder=connected_plan.parent;report_path=folder/'report.json'
    try:
        report=_read(report_path);plan=_read(connected_plan)
        if (report.get('schema')!='s300-resumed-camera-prefix-connection-v1' or
                report.get('status')!='full_audits_passed' or
                report.get('eligible_for_native_execution') is not True or
                report.get('source_files_unchanged') is not True or
                report.get('full_hz')!=60 or
                Path(report.get('verified_plan','')).resolve()!=connected_plan or
                report.get('verified_plan_sha256')!=_sha(connected_plan) or
                Path(report.get('source_checkpoint','')).resolve()!=checkpoint or
                Path(report.get('native_tool_cache',{}).get('path','')).resolve()!=native_tool_cache or
                report.get('native_tool_cache',{}).get('sha256')!=_sha(native_tool_cache)):
            raise ValueError('Connected plan/report/checkpoint/native-cache identity or eligibility mismatch')
        sources=report.get('source_file_sha256')
        assembly=Path(report['source_assembly']).resolve();source_plan=Path(report['source_plan']).resolve()
        required={assembly/'scene.usda',assembly/'composed.usdc',assembly/'motion_manifest.json',
            checkpoint/'checkpoint.json',checkpoint/'checkpoint.npz',native_tool_cache,source_plan}
        if (not isinstance(sources,dict) or not required<={Path(p) for p in sources} or
                any(not Path(p).is_absolute() or _sha(p)!=digest for p,digest in sources.items())):
            raise ValueError('Missing or stale immutable source-file SHA evidence')
        code=folder/'code/camera_prefix_connection.py'
        if not report.get('code_sha256') or _sha(code)!=report['code_sha256']:
            raise ValueError('Verified helper code snapshot SHA mismatch')
        raw=_read(source_plan);opening=plan.get('preparation_opening',{})
        old_opening=raw.get('preparation_opening',{})
        forward=[p['name'] for p in plan['phases'] if p['name'].startswith('prepare_camera_ompl_')]
        replaced=set(plan.get('camera_prefix_connection',{}).get('replaced_forward_phases',[]))
        feedback=opening.get('feedback_before_phases',[])
        preserved=set(old_opening.get('feedback_before_phases',[]))-replaced
        if (not forward or not isinstance(feedback,list) or not set(forward)<=set(feedback) or
                not preserved<=set(feedback) or replaced.intersection(feedback) or
                'approach' not in feedback or opening.get('required_gap_m')!=.096 or
                {k:v for k,v in opening.items() if k!='feedback_before_phases'}!=
                {k:v for k,v in old_opening.items() if k!='feedback_before_phases'}):
            raise ValueError('Missing or changed measured full-open guard for connected motion')
        files={name:folder/name for name in ('full_self60.json','full_environment60.json')}
        digests=report.get('audit_file_sha256',{})
        if any(digests.get(name)!=_sha(path) for name,path in files.items()):
            raise ValueError('Full audit file SHA mismatch or missing certificate')
        self_report=_read(files['full_self60.json']);environment=_read(files['full_environment60.json'])
        names=[p['name'] for p in plan['phases']]
        if (report.get('full_self',{}).get('passed') is not True or
                report.get('full_self',{}).get('complete') is not True or
                self_report.get('passed') is not True or self_report.get('complete') is not True or
                self_report.get('input_sha256')!=_sha(connected_plan) or
                self_report.get('scene_sha256')!=sources[str(assembly/'scene.usda')] or
                self_report.get('physics_rerun') is not False or
                self_report.get('hz')!=60 or self_report.get('geometry',{}).get('clearance_m')!=.002 or
                [p.get('phase') for p in self_report.get('phases',[])]!=names or
                any(p.get('clear') is not True for p in self_report['phases'])):
            raise ValueError('A complete full60Hz2mm self certificate is required')
        sampling=environment.get('sampling',{});proof=environment.get('checkpoint_proof',{})
        native=environment.get('native_tool_cache',{});env_sha=environment.get('source_sha256',{})
        geometry=environment.get('geometry',{});release=geometry.get('released_fruit_collection_diagnostic',{})
        if (report.get('contact_policy')!=required_contact_policy(environment.get('allowed_truss_key')) or
                not environment.get('allowed_truss_key') or geometry.get('wheel_floor_overlap_limit_m')!=.001 or
                release.get('collection_enabled') is not True or
                release.get('gripper_support_enabled') is not True or
                release.get('same_truss_enabled') is not True):
            raise ValueError('Exact existing released-fruit/common1mm contact policy binding required')
        if (report.get('full_environment',{}).get('passed') is not True or
                report.get('full_environment',{}).get('complete') is not True or
                environment.get('schema')!='s300-rail-environment-preflight-v1' or
                environment.get('passed') is not True or environment.get('complete') is not True or
                environment.get('eligible_for_native_execution') is not True or
                environment.get('native_hz')!=60 or environment.get('requested_hz')!=60 or
                environment.get('strict_native_period_sampling') is not True or
                environment.get('early_terminated') is not False or
                environment.get('hard_intersection_sample_pairs')!=0 or
                environment.get('selected_phases')!=names or sampling.get('passed') is not True or
                sampling.get('complete') is not True or sampling.get('hz')!=60 or
                [p.get('phase') for p in sampling.get('phases',[])]!=names or
                environment.get('samples')!=self_report.get('samples') or
                environment.get('samples')!=sampling.get('samples') or
                not isinstance(environment.get('samples'),int) or environment['samples']<1 or
                Path(proof.get('checkpoint','')).resolve()!=checkpoint or
                proof.get('checkpoint_json_sha256')!=sources[str(checkpoint/'checkpoint.json')] or
                proof.get('npz_sha256')!=sources[str(checkpoint/'checkpoint.npz')] or
                proof.get('metadata_sha256')!=report.get('checkpoint_provenance',{}).get('checkpoint_metadata_sha256') or
                Path(native.get('path','')).resolve()!=native_tool_cache or
                native.get('sha256')!=_sha(native_tool_cache) or native.get('source_model_bound') is not True or
                env_sha.get('plan')!=_sha(connected_plan) or
                env_sha.get('scene')!=sources[str(assembly/'scene.usda')] or
                env_sha.get('manifest')!=sources[str(assembly/'motion_manifest.json')] or
                env_sha.get('materialized_dependency')!=sources[str(assembly/'composed.usdc')]):
            raise ValueError('A complete model/plan/checkpoint/cache-bound60Hz environment certificate is required')
        for source,destination in ((files['full_self60.json'],output_self),
                                   (files['full_environment60.json'],output_environment)):
            destination.parent.mkdir(parents=True,exist_ok=True)
            with destination.open('xb') as stream:stream.write(source.read_bytes())
        if (any(_sha(p)!=digest for p,digest in sources.items()) or
                _sha(output_self)!=digests['full_self60.json'] or
                _sha(output_environment)!=digests['full_environment60.json']):
            raise ValueError('Source/certificate changed while copying verified audits')
        return dict(schema='s300-exact-camera-audit-reuse-v1',
            source_connection_report=str(report_path),source_connection_report_sha256=_sha(report_path),
            connected_plan=str(connected_plan),connected_plan_sha256=_sha(connected_plan),
            checkpoint=str(checkpoint),native_tool_cache=str(native_tool_cache),
            contact_policy=report['contact_policy'],
            source_audits={name:dict(path=str(path),sha256=digests[name]) for name,path in files.items()},
            copied_self=str(output_self),copied_environment=str(output_environment),
            audit_bytes_identical=True,new_FCL_queries=0,new_physics_steps=0,
            reused_existing_complete_certificates=True,training_eligible=False)
    except (KeyError,TypeError,OSError,json.JSONDecodeError) as exc:
        raise ValueError('Verified camera audit reuse rejected: '+type(exc).__name__+': '+str(exc)) from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly',type=Path)
    for name in ('plan','checkpoint','output','native-tool-cache'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--allowed-truss-key',required=True)
    parser.add_argument('--solve-seconds',type=float,default=30.)
    parser.add_argument('--hz',type=int,default=60)
    parser.add_argument('--seed',type=int,choices=(2026,2027,2028),default=2026)
    parser.add_argument('--joint-path',type=Path)
    parser.add_argument('--strategy',choices=('ompl','lift-then-arm'),default='ompl')
    parser.add_argument('--proposal-resolution',choices=('fine','coarse'),default='fine')
    a = parser.parse_args()
    result = connect_camera_prefix(a.assembly,a.plan,a.checkpoint,a.output,
        allowed_truss_key=a.allowed_truss_key,native_tool_cache=a.native_tool_cache,
        solve_seconds=a.solve_seconds,hz=a.hz,seed=a.seed,joint_path=a.joint_path,
        strategy=a.strategy,proposal_resolution=a.proposal_resolution)
    print(json.dumps(dict(verified_plan=None if result is None else str(result))))
    return 0 if result is not None else 2


if __name__ == '__main__':raise SystemExit(main())
