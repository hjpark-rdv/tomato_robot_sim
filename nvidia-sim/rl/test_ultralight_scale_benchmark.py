import json
from ultralight_scale_benchmark import measure


def sample(tmp_path, *, active=2, native_error=False):
    row=dict(classification={'result':'invalid_physics'},physics_valid=False,
        first_contact_object='/World/envs/env_0/Plant/Stem',max_tool_penetration_mm=.51)
    report=dict(complete=True,num_envs=2,api_checks_passed=True,
        motion_prefix_10s=dict(batch_sim_s=10.,wall_s=20.,active_envs=active),
        motion=dict(wall_s=30.,batch_sim_s=17.,outcomes=[row,row]))
    (tmp_path/'report.json').write_text(json.dumps(report))
    (tmp_path/'run.log').write_text('[omni.physx.plugin] PhysX error: buffer overflow' if native_error else '')


def test_completed_native_run_does_not_hide_invalid_contacts(tmp_path):
    sample(tmp_path)
    result=measure(tmp_path,0,40.,[])
    assert result['complete'] and result['reset_and_break_api_passed']
    assert result['invalid_physics_count']==2 and result['valid_physics_count']==0
    assert result['aggregate_prefix_sim_seconds_per_wall_second']==1.


def test_partial_batch_or_native_error_cannot_be_ranked_as_full_batch(tmp_path):
    sample(tmp_path,active=1,native_error=True)
    result=measure(tmp_path,0,40.,[])
    assert not result['complete']
    assert result['aggregate_prefix_sim_seconds_per_wall_second'] is None
    assert result['native_errors']
