import json
import numpy as np
from gpu_scale_validation import compare_replays


def replay(root,count):
    root.mkdir()
    row=dict(classification={'result':'miss'},abort_reason=None,target_broken=False,other_broken=False,
             inserted=False,retained_hook=False)
    outcomes=[dict(row,env_index=i,first_contact_object=f'/World/envs/env_{i}/Plant/Stem') for i in range(count)]
    (root/'report.json').write_text(json.dumps(dict(complete=True,num_envs=count,fixture={'commands_sha256':'fixture'},motion={'outcomes':outcomes})))
    for i in range(count):
        (root/f'trace_{i}.json').write_text(json.dumps([dict(joints=[0]*7,target_displacement_m=0,main_stem_displacement_m=0,gap_m=.01)]))
        np.savez(root/f'final_state_{i}.npz',elastic_bodies=np.zeros((1,3,13)),fruits=np.zeros((2,1,13)))


def test_replay_comparison_checks_all_slots_and_normalizes_only_environment_prefix(tmp_path):
    a,b=tmp_path/'one',tmp_path/'many';replay(a,1);replay(b,3)
    assert compare_replays(a,b)['passed']
    p=b/'trace_2.json';rows=json.loads(p.read_text());rows[0]['gap_m']+=.001;p.write_text(json.dumps(rows))
    result=compare_replays(a,b)
    assert not result['passed'] and result['checks'][0]['passed'] and not result['checks'][2]['passed']


def test_missing_environment_or_changed_contact_rejects_replay(tmp_path):
    a,b=tmp_path/'one',tmp_path/'many';replay(a,1);replay(b,2)
    p=b/'report.json';r=json.loads(p.read_text());r['motion']['outcomes'][1]['first_contact_object']='/World/envs/env_1/Other/Stem';p.write_text(json.dumps(r))
    assert not compare_replays(a,b)['passed']
    r['motion']['outcomes'].pop();p.write_text(json.dumps(r))
    assert compare_replays(a,b)['reason']=='not all environments executed'
