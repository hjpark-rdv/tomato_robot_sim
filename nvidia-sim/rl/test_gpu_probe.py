import json
import numpy as np
from gpu_probe import compare


def fixture(folder,steps,wall,abort=None,broken=False):
    folder.mkdir()
    outcome=dict(abort_reason=abort,target_broken=broken,other_broken=broken,retained_hook=False,
        inserted=False,first_contact_object=None,classification={'result':'miss'})
    report=dict(complete=True,num_envs=1,fixture={'commands_sha256':'same-commands'},
        motion=dict(wall_s=wall,outcomes=[outcome]))
    (folder/'report.json').write_text(json.dumps(report))
    trace=[dict(joints=[0.]*7,target_displacement_m=0.,main_stem_displacement_m=0.,gap_m=.01) for _ in range(steps)]
    (folder/'trace_0.json').write_text(json.dumps(trace))
    np.savez(folder/'initial_state.npz',robot_q=np.zeros(7))


def test_early_gpu_break_is_never_reported_as_speedup(tmp_path):
    a,b=tmp_path/'cpu',tmp_path/'gpu'
    fixture(a,260,26.);fixture(b,1,.8,abort='joint_break',broken=True)
    result=compare(a,b)
    assert not result['comparable'] and result['speedup'] is None


def test_matching_work_can_be_compared_but_pose_drift_rejects_speedup(tmp_path):
    a,b=tmp_path/'cpu',tmp_path/'gpu'
    fixture(a,260,26.);fixture(b,260,13.)
    assert compare(a,b)['speedup']==2.
    path=b/'trace_0.json';trace=json.loads(path.read_text());trace[100]['main_stem_displacement_m']=.005
    path.write_text(json.dumps(trace))
    result=compare(a,b)
    assert not result['comparable'] and result['speedup'] is None


def test_incomplete_execution_has_no_speedup(tmp_path):
    a,b=tmp_path/'cpu',tmp_path/'gpu'
    fixture(a,260,26.);fixture(b,260,13.)
    p=b/'report.json';report=json.loads(p.read_text());report['complete']=False;p.write_text(json.dumps(report))
    assert compare(a,b)['speedup'] is None


def test_missing_native_contact_readback_cannot_validate_matching_misses(tmp_path):
    a,b=tmp_path/'cpu',tmp_path/'gpu'
    fixture(a,260,26.);fixture(b,260,13.)
    p=b/'report.json';report=json.loads(p.read_text())
    report['backend']={'suppress_readback':True};p.write_text(json.dumps(report))
    result=compare(a,b)
    assert not result['comparable'] and result['speedup'] is None
    assert 'contact reports' in result['reason']


def test_changed_numerical_inertia_is_not_same_model_even_if_traces_match(tmp_path):
    a,b=tmp_path/'reference',tmp_path/'regularized'
    fixture(a,260,26.);fixture(b,260,13.)
    p=b/'report.json';r=json.loads(p.read_text());r['arguments']={'joint_armature':.0001};p.write_text(json.dumps(r))
    result=compare(a,b)
    assert not result['same_physical_model']
    assert not result['comparable'] and result['speedup'] is None


def test_failed_native_api_checks_cannot_validate_matching_motion(tmp_path):
    a,b=tmp_path/'reference',tmp_path/'trial'
    fixture(a,260,26.);fixture(b,260,13.)
    p=b/'report.json';r=json.loads(p.read_text());r['api_checks_passed']=False;p.write_text(json.dumps(r))
    result=compare(a,b)
    assert not result['comparable'] and result['speedup'] is None
    assert 'checks failed' in result['reason']
