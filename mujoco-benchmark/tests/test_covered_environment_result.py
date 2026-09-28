from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from covered_environment_result import idle_validity

def record(pairs=(),displacement=.01):
    return dict(finite=True,warning_counts=[0]*8,worst_contacts=list(pairs),max_body_displacement_m=displacement)

def test_displacement_is_not_a_universal_gate():
    assert idle_validity(record(displacement=.025))['passed']

def test_background_overlap_is_not_active_physics_violation():
    assert idle_validity(record([dict(categories=['background_obstacle','structural_obstacle'],penetration_m=.03)]))['passed']

def test_robot_and_active_plant_keep_existing_limit():
    for category in ['robot','active_plant']:
        assert not idle_validity(record([dict(categories=[category,'background_obstacle'],penetration_m=.00051)]))['passed']
        assert idle_validity(record([dict(categories=[category,'background_obstacle'],penetration_m=.00049)]))['passed']

def test_warning_and_nonfinite_are_hard_failures():
    r=record();r['finite']=False
    assert not idle_validity(r)['passed']
    r=record();r['warning_counts'][0]=1
    assert not idle_validity(r)['passed']
