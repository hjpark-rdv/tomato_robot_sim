import gzip
import json
from pathlib import Path
import sys
import types
import copy
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_motion_family_search as runner


def snapshot(root,target='Tomato_02',model=b'model'):
    root.mkdir(parents=True)
    for name in runner.REQUIRED:
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes(model if name.endswith('model.mjb') else b'{}')
    runner.write(root/'manifest.json',dict(target=target,model_sha256=runner.file_hash(root/'replay_assets/model.mjb'),count=3,hz=240))
    return root


def manifest(tmp,rows):
    p=tmp/'cases.json';runner.write(p,dict(cases=rows));return p


def test_manifest_identity_split_and_immutable_snapshot(tmp_path):
    root=snapshot(tmp_path/'original')
    cases=runner.load_cases(manifest(tmp_path,[dict(scene_id='scene_0',target='Tomato_02',source_run=str(root))]))
    target=tmp_path/'new';new=runner.prepare_snapshot(cases[0],target)
    assert runner.file_hash(target/'replay_assets/model.mjb')==runner.file_hash(root/'replay_assets/model.mjb')
    assert new['source_candidate_count']==3 and 'count' not in new
    assert all(runner.file_hash(root/k)==v for k,v in cases[0]['hashes'].items())
    with pytest.raises(FileExistsError):runner.prepare_snapshot(cases[0],target)


def test_duplicate_scene_target_and_split_leakage_rejected(tmp_path):
    root=snapshot(tmp_path/'a');other=snapshot(tmp_path/'b')
    a=dict(scene_id='one',target='Tomato_02',source_run=str(root),split='train')
    b=dict(scene_id='two',target='Tomato_02',source_run=str(other),split='test')
    with pytest.raises(ValueError,match='splits'):runner.load_cases(manifest(tmp_path,[a,b]))
    with pytest.raises(ValueError,match='Duplicate'):runner.load_cases(manifest(tmp_path,[a,a]))


def test_missing_or_changed_snapshot_is_not_unreachable(tmp_path):
    root=snapshot(tmp_path/'a')
    row=dict(scene_id='s',target='Tomato_02',source_run=str(root))
    (root/'replay_assets/model.mjb').write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash'):runner.load_cases(manifest(tmp_path,[row]))


def test_trial_read_distinguishes_geom_from_capture(tmp_path):
    p=tmp_path/'trial';p.mkdir()
    runner.write(p/'target_contact_trial.json',dict(physics_executed=True,completed=True,audit_status='sampled_clear',audit_complete=True))
    samples=[dict(seated=True,target_contact=False)]
    (p/'authorized_contact_samples.json.gz').write_bytes(gzip.compress(json.dumps(samples).encode()))
    r=runner.read_trial(p,'x','side_mouth',0)
    assert r['outcome']=='geometric_candidate' and not r['contact_retention_evidence']


def test_collection_discovery_preserves_rejected_scene_evidence(tmp_path):
    root=tmp_path/'collection';snapshot(root/'scene_0000/targets/Tomato_02/physics')
    runner.write(root/'collection.json',dict(scene_splits={'scene_0000':'train'},failures=[{'scene':'scene_0001','reason':'initial_scene_screen_failed'}]))
    out=runner.discover_collection(root,tmp_path/'index.json')
    doc=runner.read(out)
    assert doc['cases'][0]['scene_id']=='scene_0000'
    assert doc['source_scene_failures'][0]['scene']=='scene_0001'


def test_orchestration_reuses_planner_and_trial_and_bounds_physics(tmp_path,monkeypatch):
    """Fake child processes test orchestration ONLY, not native robot success."""
    root=snapshot(tmp_path/'original')
    case=runner.load_cases(manifest(tmp_path,[dict(scene_id='s',target='Tomato_02',source_run=str(root))]))[0]
    policy={'clearance_m':0.,'allowed_contacts':[]}
    monkeypatch.setattr(runner,'native_geometry',lambda run:({},policy))
    candidates=[dict(candidate_id='c'+str(i),family=f,search_parameters={},parent_id=None) for i,f in enumerate(runner.FAMILIES)]
    monkeypatch.setattr(runner,'generate_candidates',lambda *args,**kwargs:(copy.deepcopy(candidates),[]))
    calls=[]
    def fake(command,log,timeout):
        calls.append(command)
        if 'plan_candidates.py' in command[1]:
            run=Path(command[2])
            for c in runner.read(run/'candidates.json'):
                runner.write(run/'candidates'/c['candidate_id']/'plan.json',dict(preflight={'passed':True},parameters=c))
            return 0
        assert '--search-contact-policy' in command
        out=Path(command[command.index('--output')+1]);out.mkdir(parents=True)
        runner.write(out/'target_contact_trial.json',dict(physics_executed=True,completed=True,audit_status='sampled_clear',audit_complete=True,source_unchanged=True))
        (out/'authorized_contact_samples.json.gz').write_bytes(gzip.compress(b'[{"seated":true}]'))
        return 0
    monkeypatch.setattr(runner,'run_command',fake)
    config=dict(planning_python='planner',native_python='native',samples_per_family=1,seed=5,planning_workers=1,
                planning_timeout_s=5.,trial_timeout_s=5.,case_timeout_s=60.,physics_per_target=3,refine_parents=0,
                execute=True,max_target_force_N=5.,max_target_displacement_m=.02)
    result=runner.run_case(case,config,tmp_path/'campaign')
    assert result['complete'],result
    assert result['summary']['physics_attempts']==3
    assert sum(r['outcome']=='not_evaluated_budget' for r in result['records'])==2
    assert result['source_unchanged']
    assert sum('plan_candidates.py' in c[1] for c in calls)==1
    assert sum('target_fruit_contact_trial.py' in c[1] for c in calls)==3
    # Completed case is reused without spending more experiments.
    assert runner.run_case(case,config,tmp_path/'campaign')==result


def test_publish_keeps_error_and_progress_denominators(tmp_path):
    rows=[dict(outcome='geometric_candidate',family='side_mouth',candidate_id='x',physics_executed=True)]
    report=dict(case={'scene_id':'s','target':'Tomato_02'},records=rows,summary=runner.summarize_target(rows),
                complete=True,physics_budget_charged=1,render_queue=[])
    result=runner.publish(tmp_path,[report],2)
    assert result['reported_cases']==1 and result['expected_cases']==2
    assert result['hook_success'] is None
    assert (tmp_path/'index.html').exists() and (tmp_path/'results.csv').exists()


def test_random_batch_uses_existing_generator_and_scene_only_not_old_rollouts(tmp_path,monkeypatch):
    import prepare_motion_search_batch as prep
    model=tmp_path/'model.pkl';model.write_bytes(b'fixture')
    args=types.SimpleNamespace(output=tmp_path/'batch',scenes=2,trusses=1,targets_per_scene=1,
        planning_model=model,native_python='native',source_dir=tmp_path,seed=1,segment_min=6,segment_max=10,
        angle_min=0.,angle_max=180.,truss_scale=.5,gutter_collisions=False,hz=240,timeout_s=5.)
    calls=[]
    def fake(command,log,timeout):
        calls.append(command)
        if command[1].endswith('generate_random_glb_scenes.py'):
            out=Path(command[command.index('--output')+1])
            runner.write(out/'manifest.json',dict(records=[dict(scene='scene_0000',validation={'scene_screen_passed':True}),
                dict(scene='scene_0001',validation={'scene_screen_passed':False})]))
            runner.write(out/'scene_0000/reference.json',dict(fruit_specs=[{'name':'Tomato_01'},{'name':'Tomato_02'}]))
        else:
            assert command[1].endswith('candidate_experiment.py') and '--scene-only' in command
        return 0
    monkeypatch.setattr(prep,'run_command',fake)
    result=runner.read(prep.prepare(args))
    assert len(result['cases'])==1 and result['cases'][0]['target']=='Tomato_01'
    assert len(result['source_scene_failures'])==1
    assert len(result['unselected_targets'])==1
    assert len(calls)==2
