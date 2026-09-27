import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_motion_search import summarize


def test_budget_and_interrupted_proposals_stay_in_denominator(tmp_path):
    case={'scene_id':'s','target':'Tomato_01'}
    records=[dict(candidate_id='a',family='side_mouth',outcome='not_evaluated_budget',physics_executed=False)]
    (tmp_path/'results.json').write_text(json.dumps(dict(reports=[dict(case=case,records=records)],expected_cases=1,reported_cases=1,completed_cases=0)))
    run=tmp_path/'cases/s/Tomato_01/run'; (run/'candidates/a').mkdir(parents=True)
    (run/'candidates.json').write_text(json.dumps([dict(candidate_id=c,family='side_mouth') for c in ('a','b')]))
    (run/'candidates/a/plan.json').write_text(json.dumps(dict(preflight={'passed':True},seconds=3)))
    result=summarize(tmp_path); f=result['families']['side_mouth']
    assert f['proposed']==2 and f['ik_and_robot_check_pass']==1
    assert f['executed']==0 and f['full_audit_pass']==0
    assert f['outcome:not_evaluated_interrupted']==1
    assert result['impossible'] is None and result['hook_success'] is None
