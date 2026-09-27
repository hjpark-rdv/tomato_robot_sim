"""Read existing search evidence; never plans, steps physics, or creates labels."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

from summarize_target_contact_trial import summarize as summarize_trial


def summarize(root):
    root = Path(root)
    results = json.loads((root/'results.json').read_text())
    families = defaultdict(Counter)
    rows = []
    for report in results['reports']:
        case = report['case']
        folder = root/'cases'/case['scene_id']/case['target']
        recorded = {r['candidate_id'] for r in report['records']}
        proposals_path = folder/'run/candidates.json'
        proposals = json.loads(proposals_path.read_text()) if proposals_path.exists() else []
        records = report['records'] + [dict(candidate_id=c['candidate_id'], family=c['family'],
                     outcome='not_evaluated_interrupted', physics_executed=False)
                     for c in proposals if c['candidate_id'] not in recorded]
        for record in records:
            row = dict(scene=case['scene_id'], target=case['target'], **record)
            tally = families[record['family']]
            tally['proposed'] += 1
            tally['outcome:'+record['outcome']] += 1
            planpath = folder/'run/candidates'/record['candidate_id']/'plan.json'
            if planpath.exists():
                plan = json.loads(planpath.read_text())
                row['planning_preflight'] = plan['preflight']
                row['planned_seconds'] = plan.get('seconds')
                tally['ik_and_robot_check_pass'] += int(bool(plan['preflight'].get('passed')))
            tally['full_audit_complete'] += int(bool(record.get('audit_complete')))
            tally['full_audit_pass'] += int(bool(record.get('audit_complete')) and record.get('audit_status') == 'sampled_clear')
            tally['executed'] += int(bool(record.get('physics_executed')))
            tally['completed'] += int(bool(record.get('completed')))
            tally['geometric_any'] += int(record.get('geometric_samples', 0) > 0)
            tally['retention_evidence'] += int(bool(record.get('contact_retention_evidence')))
            trial = folder/'trials'/record['candidate_id']
            if (trial/'target_contact_trial.json').exists():
                detail = summarize_trial(trial)
                # This is a derived summary. Full 240Hz samples/events remain in trial.
                detail.pop('stop_sample', None)
                row['evidence'] = detail
            rows.append(row)
    return dict(source=str(root.resolve()), expected_cases=results['expected_cases'],
                reported_cases=results['reported_cases'], completed_cases=results['completed_cases'],
                families=dict(families), candidates=rows,
                count_notes='IK column requires both IK and existing robot self/speed checks. Geometric_any includes stopped trajectories; not retention.',
                hook_success=None, training_eligible=False, impossible=None)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists(): p.error('Use a new derived summary output')
    a.output.write_text(json.dumps(summarize(a.campaign), indent=2)+'\n')
