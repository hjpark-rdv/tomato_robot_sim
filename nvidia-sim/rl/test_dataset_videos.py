import copy
from dataset_videos import select,compare

def row(index,result='miss',**kw):
    return dict(candidate_id=f'candidate_{index:05d}',executed=True,result=result,hook_success=False,**kw)

def test_partial_is_not_full_success():
    chosen=select([row(0),row(5,center_entry_safe=True,center_entry_duration_s=1),row(7,'excessive_displacement')])
    assert [r['category'] for r in chosen]==['partial','miss','displacement']
    assert chosen[0]['candidate_id']=='candidate_00005'

def test_incomplete_and_planning_failures_not_replayed():
    assert select([row(0,'incomplete'),dict(candidate_id='candidate_00001',executed=False,result='ik_or_planning_failure')])==[]

def test_discrepancy_cannot_pass():
    source=dict(result='miss',hook_success=False,executed_steps=100,target_max_displacement_m=.01,main_stem_max_displacement_m=.02,
                first_contact_object='/World/envs/env_22/Plant/stem')
    replay=dict(classification=dict(result='miss',hook_success=False),executed_steps=100,target_max_displacement_m=.011,
                main_stem_max_displacement_m=.02,first_contact=dict(object='/World/envs/env_0/Plant/stem'))
    assert compare(source,replay)['passed']
    other=copy.deepcopy(replay);other['classification']['result']='success_target_hook'
    assert not compare(source,other)['passed']
    other=copy.deepcopy(replay);other['target_max_displacement_m']=.014
    assert not compare(source,other)['passed']
    other=copy.deepcopy(replay);other['first_contact']['object']='/World/envs/env_0/Plant/other'
    assert not compare(source,other)['passed']

def test_middle_of_replay_divergence_is_not_hidden_by_same_final_result():
    from dataset_videos import compare_trace
    trace=[dict(joints=[0.]*7,phase='rise',target_displacement_m=0.,main_stem_displacement_m=0.) for _ in range(3)]
    replay=copy.deepcopy(trace)
    assert compare_trace(trace,replay)['passed']
    replay[1]['target_displacement_m']=.005
    assert not compare_trace(trace,replay)['passed']
    replay=copy.deepcopy(trace);replay[1]['joints'][0]=.005
    assert not compare_trace(trace,replay)['passed']


def test_same_result_from_different_plant_models_is_not_a_matching_replay():
    source=dict(result='miss',hook_success=False,center_entry_safe=False,first_contact=None,
                target_max_displacement_m=0.,main_stem_max_displacement_m=0.,executed_steps=20,
                plant_resolution='light')
    replay=dict(source,plant_resolution='full',classification=dict(result='miss',hook_success=False))
    assert not compare(source,replay)['passed']
    assert not compare(source,replay)['checks']['plant_resolution']['passed']
    replay['plant_resolution']='light'
    assert compare(source,replay)['passed']
    source['main_appendage_collisions']='ignore'
    assert not compare(source,replay)['passed']
    replay['main_appendage_collisions']='ignore'
    assert compare(source,replay)['passed']


def test_cpu_gpu_replay_difference_is_not_hidden():
    source=dict(result='miss',hook_success=False,executed_steps=10,target_max_displacement_m=0.,main_stem_max_displacement_m=0.,
                physical_inputs={'backend':{'physics_device':'cpu'}})
    replay=dict(classification={'result':'miss','hook_success':False},executed_steps=10,target_max_displacement_m=0.,main_stem_max_displacement_m=0.)
    assert not compare(source,replay)['checks']['physics_device']['passed']
    replay['physics_device']='cpu'
    assert compare(source,replay)['checks']['physics_device']['passed']

def test_mismatched_replay_is_separate_from_representative_gallery(tmp_path):
    from dataset_videos import gallery
    manifest=dict(total_candidates=1,counts={'miss':1},videos=[dict(title='실패',candidate_id='candidate_00000',
        comparison={'passed':False},video='miss.mp4',source_result='miss',replay_result='miss',comparison_file='comparison.json',candidate_file='candidate.json')])
    gallery(tmp_path,manifest)
    page=(tmp_path/'index.html').read_text()
    assert '대표 영상: 0개 / 별도 진단: 1개' in page
    assert '추가 진단: 재실행 불일치 또는 물리 오류' in page
    assert '이번 시험에는 완전 걸림 성공이 없습니다' in page


def test_replay_match_does_not_override_confirmed_penetration(tmp_path):
    from dataset_videos import gallery,representative_video
    row=dict(title='비목표 접촉',candidate_id='candidate_00049',comparison={'passed':True},
        video='case49.mp4',source_result='non_target_contact',replay_result='non_target_contact',
        comparison_file='comparison.json',candidate_file='candidate.json',
        physical_audit={'passed':False,'reason':'송이 줄기 관통 확인'})
    assert not representative_video(row)
    gallery(tmp_path,dict(total_candidates=64,counts={'miss':8},videos=[row]))
    page=(tmp_path/'index.html').read_text()
    assert '대표 영상: 0개 / 별도 진단: 1개' in page
    assert '물리 검증 실패: 송이 줄기 관통 확인' in page
