"""Preset selection must never silently change a resumed dataset's physics."""
import json
from test_gpu_dataset_runner import prepare


def test_reference_preserves_original_defaults(tmp_path):
    r=prepare(tmp_path,'--physics-preset','reference960')
    assert r.returncode==0,r.stderr
    c=json.loads((tmp_path/'config.json').read_text())
    assert (c['physics_hz'],c['elastic_joint_armature'],c['position_iterations'])==(960,1e-5,64)


def test_practical_preset_is_saved_and_resume_cannot_mix_models(tmp_path):
    r=prepare(tmp_path)
    assert r.returncode==0,r.stderr
    c=json.loads((tmp_path/'config.json').read_text())
    assert c['physics_hz']==60 and c['elastic_joint_armature']>1e-5
    assert prepare(tmp_path,'--physics-preset','practical60','--resume').returncode==0
    assert prepare(tmp_path,'--physics-preset','reference960','--resume').returncode!=0
    assert prepare(tmp_path,'--physics-preset','practical60','--position-iterations','16','--resume').returncode!=0


def test_explicit_overrides_are_recorded_not_disguised_as_preset(tmp_path):
    r=prepare(tmp_path,'--physics-preset','practical120','--physics-hz','240',
              '--elastic-joint-armature','.0002','--position-iterations','32')
    assert r.returncode==0,r.stderr
    c=json.loads((tmp_path/'config.json').read_text())
    assert (c['physics_hz'],c['elastic_joint_armature'],c['position_iterations'])==(240,.0002,32)


def test_invalid_iteration_count_rejected(tmp_path):
    for v in ('0','256'):
        assert prepare(tmp_path/v,'--position-iterations',v).returncode!=0
