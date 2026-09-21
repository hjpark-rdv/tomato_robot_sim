import copy
import json
from pathlib import Path
import numpy as np
from gpu_throughput_benchmark import compare_datasets, measure, publish


def fixture(root, env=0):
    root.mkdir()
    def write(name,data):(root/name).write_text(json.dumps(data))
    write('config.json',dict(num_envs=env+1,seed=42,solver='PGS'))
    write('source_sha256.json',{'worker':'abc'})
    write('candidates.json',[{'candidate_id':'candidate_00000'}])
    row=dict(candidate_id='candidate_00000',parameters={'roll_deg':0},executed=True,result='miss',abort_reason=None,
             first_contact_object=f'/World/envs/env_{env}/Stem',target_broken=False,other_broken=False)
    folder=root/'results/candidate_00000';folder.mkdir(parents=True)
    (folder/'candidate.json').write_text(json.dumps(row))
    np.save(folder/'planned_commands.npy',np.array([[0.,1.]]))
    (folder/'trace.json').write_text(json.dumps([dict(joints=[0.,1.],target_displacement_m=.001,main_stem_displacement_m=.002,gap_m=.01)]))
    write('summary.json',dict(execution_complete=True,dataset_complete=True,num_envs=env+1,counts={'miss':1},rollout_wall_s=10.,initialization_wall_s=2.))
    return root


def test_batch_indices_do_not_change_contact_identity(tmp_path):
    a=fixture(tmp_path/'a');b=fixture(tmp_path/'b',63)
    assert compare_datasets(a,b)['passed']


def test_same_label_but_different_motion_is_not_eligible(tmp_path):
    a=fixture(tmp_path/'a');b=fixture(tmp_path/'b',31)
    p=b/'results/candidate_00000/trace.json';trace=json.loads(p.read_text());trace[0]['gap_m']+=.002;p.write_text(json.dumps(trace))
    result=compare_datasets(a,b)
    assert not result['passed'] and 'gap_m' in result['checks'][0]['failures']


def test_incomplete_runs_have_no_throughput_and_cannot_win(tmp_path):
    a=fixture(tmp_path/'a');p=a/'results/candidate_00000/candidate.json';r=json.loads(p.read_text());r['result']='incomplete';p.write_text(json.dumps(r))
    row=measure(a,12.)
    assert not row['complete'] and row['candidates_per_minute'] is None
    assert not compare_datasets(a,a)['passed']
    row['comparison_passed']=True
    assert publish(tmp_path,[row],complete=True)['best_num_envs'] is None


def test_rank_only_complete_matching_workloads_after_all_runs(tmp_path):
    a=fixture(tmp_path/'a');row=measure(a,12.);row['comparison_passed']=True
    fast=copy.deepcopy(row);fast.update(num_envs=128,candidates_per_minute=100.,comparison_passed=False)
    assert publish(tmp_path,[row,fast],complete=False)['best_num_envs'] is None
    assert publish(tmp_path,[row,fast],complete=True)['best_num_envs']==1


def test_different_candidates_or_commands_are_rejected(tmp_path):
    a=fixture(tmp_path/'a');b=fixture(tmp_path/'b')
    np.save(b/'results/candidate_00000/planned_commands.npy',np.array([[1.,1.]]))
    assert not compare_datasets(a,b)['passed']
    (b/'candidates.json').write_text('[]')
    assert compare_datasets(a,b)['failures']==['candidate_list']


def test_concurrent_resume_is_rejected(tmp_path):
    import pytest
    from gpu_throughput_benchmark import acquire_benchmark_lock
    first=acquire_benchmark_lock(tmp_path)
    try:
        with pytest.raises(BlockingIOError):acquire_benchmark_lock(tmp_path)
    finally:first.close()
    second=acquire_benchmark_lock(tmp_path)
    second.close()
