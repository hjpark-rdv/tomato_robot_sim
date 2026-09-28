"""Freeze one evidence-selected recovery round from the d1fdc7c pilot.

Default: four distinct prefix-blocked targets and two prefix-clear/local-blocked
targets. Shared four-family prefixes are counted once. No new scene generation,
physics execution, threshold sweep, or automatic repeated tuning occurs here.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from recovery_design import BASE_COMMIT, recorded_prefix, select_evidence_groups


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path)
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    tmp.replace(path)


def prepare(campaign, prefix_report, output, planning_python, *, prefix_targets=4,
            local_targets=2, model_cache=None, side_campaign=None):
    campaign, prefix_report, output = Path(campaign).resolve(), Path(prefix_report).resolve(), Path(output).resolve()
    if output.exists() or campaign == output or campaign in output.parents:
        raise ValueError('Use a new output outside the original campaign')
    report = read(prefix_report)
    cases = read(campaign/'campaign.json')['cases']
    targets = {(r['scene_id'], r['target']): r for r in cases}
    if len(targets) != len(cases):
        raise ValueError('Duplicate campaign scene/target')
    progress = {}
    for (scene, target) in targets:
        for row in read(campaign/'cases'/scene/target/'case_result.json')['records']:
            phases = (row.get('first_violation') or {}).get('phases', [])
            progress[(scene, target, row['candidate_id'])] = max(
                ({'preapproach': 0, 'approach': 1, 'insert': 2, 'seat': 3, 'hold': 4, 'verify': 5}.get(p, 0)
                 for p in phases), default=0)
    chosen = select_evidence_groups(report, prefix_targets, local_targets, progress)
    queries = []
    for item in chosen:
        scene, target, cid = item['scene'], item['target'], item['candidate']
        root = campaign/'cases'/scene/target/'run'
        policy = root.parent/'base_policy.json'
        if read(root/'manifest.json')['target'] != target:
            raise ValueError('Actual target differs from report')
        inputs = read(root/'planning_inputs.json')
        trace_path = root/'candidates'/cid/'trace.json'
        _, measured = recorded_prefix(read(trace_path), np.asarray(inputs['inputs']['start']))
        if measured['command_sha256'] != item['prefix_sha256'] or measured['samples'] != item['prefix_samples']:
            raise ValueError('Actual prefix differs from committed report')
        if not policy.is_file(): raise ValueError('Missing original exact policy')
        configs = ('recorded',) if item['cohort'] == 'prefix_blocked' else ('recorded', 'neutral', 'neutral_pitch15')
        for design in configs:
            identity = f'q{len(queries):02d}_{scene}_{target}_{design}'
            destination = output/identity
            command = [str(planning_python), str(Path(__file__).with_name('plan_approach_connection.py')),
                       str(root), '--candidate', cid, '--base-policy', str(policy), '--output', str(destination),
                       '--local-design', design, '--ik-seeds', '8', '--max-branches', '3',
                       '--solve-seconds', '8', '--total-seconds', '120', '--seed', '20260928']
            if item['cohort'] == 'local_blocked': command.append('--require-recorded-prefix')
            if model_cache: command.extend(['--model-cache', str(Path(model_cache).resolve())])
            queries.append(dict(**item, query_id=identity, design=design, command=command, output=str(destination),
                                original_trace_sha256=sha(trace_path), original_policy_sha256=sha(policy),
                                physics_executed=False, training_eligible=False, impossible=None))
    if side_campaign is not None:
        side_campaign = Path(side_campaign).resolve()
        side_cases = read(side_campaign/'campaign.json')['cases']
        controls = []
        for case in side_cases:
            scene, target = case['scene_id'], case['target']
            folder = side_campaign/'cases'/scene/target
            for record in read(folder/'case_result.json')['records']:
                if record.get('family') == 'side_mouth' and record.get('reason', {}).get('reason') == 'self_collision':
                    controls.append((scene, target, record['candidate_id'], folder))
        if not controls:
            raise ValueError('Requested side control but no recorded self-collision case exists')
        scene, target, cid, folder = min(controls)
        root, policy = folder/'run', folder/'base_policy.json'
        source_plan = root/'candidates'/cid/'plan.json'
        if read(source_plan)['preflight'].get('reason') != 'self_collision':
            raise ValueError('Side control plan differs from the source record')
        qid = f'q{len(queries):02d}_{scene}_{target}_side_control'
        command = [str(planning_python), str(Path(__file__).with_name('plan_approach_connection.py')),
                   str(root), '--candidate', cid, '--base-policy', str(policy), '--output', str(output/qid),
                   '--ik-seeds', '8', '--max-branches', '3', '--solve-seconds', '8',
                   '--total-seconds', '120', '--seed', '20260928']
        if model_cache: command.extend(['--model-cache', str(Path(model_cache).resolve())])
        queries.append(dict(query_id=qid, cohort='side_self_collision', scene=scene, target=target,
                            candidate=cid, family='side_mouth', design='recorded', command=command,
                            output=str(output/qid), source_plan_sha256=sha(source_plan),
                            original_policy_sha256=sha(policy), physics_executed=False,
                            training_eligible=False, impossible=None))
    if not queries: raise ValueError('No eligible matched queries')
    output.mkdir(parents=True)
    doc = dict(schema='d1_recovery_round_v1', evidence_base=BASE_COMMIT, campaign=str(campaign),
               prefix_report=str(prefix_report), prefix_report_sha256=sha(prefix_report),
               campaign_manifest_sha256=sha(campaign/'campaign.json'), source_groups=report['groups'],
               chosen_groups=len(chosen), omitted_groups=report['groups']-len(chosen), queries=queries,
               selection_scope='Matched diagnostics, not an unbiased success-rate sample',
               physics_budget=8, physical_execution_automatic=False, required_base='latest d1fdc7c tree or descendant',
               note='The local cohort must reuse a separately rechecked recorded prefix; no global reroute fallback.')
    write(output/'recovery_queries.json', doc)
    return doc


def run_plans(doc, output, batch_seconds=1500.):
    """Reuse the existing process-group timeout runner; never execute physics."""
    from run_motion_family_search import run_command
    output = Path(output)
    summary, began = [], time.monotonic()
    for query in doc['queries']:
        row = {k: query[k] for k in ('query_id', 'cohort', 'scene', 'target', 'candidate', 'design')}
        remaining = batch_seconds-(time.monotonic()-began)
        if remaining < 180.:
            row.update(status='not_evaluated_batch_budget', physics_executed=False)
        else:
            try:
                code = run_command(query['command'], output/(query['query_id']+'.log'), 180.)
                path = Path(query['output'])/'connection_result.json'
                result = read(path) if path.is_file() else {}
                row.update(returncode=code, status=result.get('status', 'missing_result'),
                           prefix_reused=result.get('prefix_reused', False),
                           prefix_found=result.get('prefix_found', False),
                           whole_path_passed=result.get('whole_path_passed', False),
                           source_unchanged=result.get('source_unchanged'),
                           runnable_run=result.get('runnable_run'),
                           approach_only_runs=result.get('approach_only_runs', []), physics_executed=False,
                           connection_result=str(path))
                if code not in (0, 2):
                    row.update(status='planner_process_error', whole_path_passed=False, runnable_run=None)
            except TimeoutError:
                row.update(status='planner_process_timeout', physics_executed=False)
        summary.append(row)
        write(output/'recovery_plan_results.json', dict(rows=summary, planned_queries=len(doc['queries']),
              reported_queries=len(summary),
              evaluated_queries=sum('returncode' in r or r['status'] == 'planner_process_timeout' for r in summary),
              budget_skipped=sum(r['status'] == 'not_evaluated_batch_budget' for r in summary),
              wall_s=time.monotonic()-began,
              physics_executed=False, hook_success=None, impossible=None))
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--campaign', type=Path, required=True)
    p.add_argument('--prefix-report', type=Path, default=Path(__file__).resolve().parents[1]/'validation/multifamily_server/prefix_comparison.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--side-campaign', type=Path, help='One fixed side-mouth self-collision control, not 12 duplicates')
    p.add_argument('--planning-python', required=True)
    p.add_argument('--model-cache', type=Path)
    p.add_argument('--run-plans', action='store_true')
    a = p.parse_args()
    doc = prepare(a.campaign, a.prefix_report, a.output, a.planning_python, model_cache=a.model_cache, side_campaign=a.side_campaign)
    print(json.dumps(dict(queries=len(doc['queries']), selected_groups=doc['chosen_groups'], physics_executed=False)))
    if a.run_plans: run_plans(doc, a.output)


if __name__ == '__main__':
    main()
