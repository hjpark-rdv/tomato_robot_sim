"""Capping exports must not remove every tilted family."""
from scipy.spatial.transform import Rotation
from test_hook_seating_geometry import target, wires, RADIUS


def test_truncation_preserves_orientation_diversity():
    from propose_hook_seating_goals import generate_goals
    result=generate_goals([target()],wires(),[0,0,0],Rotation.identity(),
        ring_radius=RADIUS,hook_ring_offset=[0,0,0],surface_gap_m=.0004,
        max_surface_gap_m=.0015,tilts_deg=[-15,0,15],fractions=[.5],max_goals=9)
    assert result['accepted_pose_families']>=5
    assert result['exported_pose_families']==min(9,result['accepted_pose_families'])
    assert any(g['rotation_delta_from_baseline_rad']>.1 for g in result['goals'])
