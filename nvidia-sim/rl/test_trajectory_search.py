import numpy as np
import pytest
from trajectory_search import candidates, waypoints, center_region
from geometry import RING_RADIUS, WIRE_RADIUS


def test_prefix_reproducible_and_six_independent_parameters():
    assert candidates(8,42)==candidates(16,42)[:8]
    assert candidates(8,42)!=candidates(8,43)


def test_remote_start_and_below_fruit_entry_with_real_collision_size():
    c=np.array([-.7,1.1,.96]);radius=.012
    for p in candidates(8,42):
        r,wp,d=waypoints(c,c+[0,0,.025],p,radius)
        pre,entry,insert,lift=[point for _,point in wp]
        # Ring wire cannot already surround or touch the target at prepose.
        assert np.linalg.norm((pre-c)[:2])>=.17-1e-9
        assert np.linalg.norm((pre-c)[:2])-RING_RADIUS-radius>.12
        assert entry[2]+WIRE_RADIUS==pytest.approx(c[2]-radius-p['entry_clearance_m'])
        assert np.dot(insert-entry,d)==pytest.approx(p['insertion_distance_m'])
        assert np.linalg.norm(lift-insert)==pytest.approx(p['lift_distance_m'])
        np.testing.assert_allclose(r.apply([0,1,0]),[0,0,-1],atol=1e-12)


def test_partial_entry_requires_aperture_alignment_not_just_nearness():
    assert center_region([-.012,0,0],.010)
    assert not center_region([-.012,-.025,0],.010)  # Still below fruit.
    assert not center_region([+.012,0,0],.010)     # Outside rear half.
    assert not center_region([-.025,0,0],.010)     # Wire intersects fruit cross-section.
    assert not center_region([-.012,0,.030],.010) # Sideways miss.


def test_collision_radius_is_required():
    with pytest.raises(ValueError,match='radius'):
        waypoints([0,0,1],[0,0,1.03],candidates(1,42)[0],None)


@pytest.mark.parametrize('params', [candidates(1,42)[0], dict(azimuth_deg=0.,elevation_deg=0.)])
def test_result_plot_accepts_both_parameter_schemas(tmp_path,params):
    from trajectory_report import plot_results
    plot_results(tmp_path,[dict(parameters=params,result='miss')])
    assert (tmp_path/'candidate_results.png').stat().st_size>1000


def test_staged_cli_records_bounds_and_does_not_claim_pull_support(tmp_path):
    import json
    from test_gpu_dataset_runner import prepare
    result=prepare(tmp_path/'valid','--trajectory-mode','staged6d','--candidates','8')
    assert result.returncode==0,result.stderr
    config=json.loads((tmp_path/'valid/config.json').read_text())
    assert config['sampling']=='scrambled_sobol_6d' and len(config['bounds'])==6
    assert prepare(tmp_path/'invalid','--trajectory-mode','staged6d','--goal','pull').returncode!=0
