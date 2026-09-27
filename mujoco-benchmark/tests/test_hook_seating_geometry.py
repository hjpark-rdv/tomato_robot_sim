"""Independent geometric controls. None of these count as harvested tomatoes."""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from hook_seating_geometry import (Capsule, aligned_rotation, capsule_gap,
                                   seating_geometry, seating_goal, segment_distance)

RADIUS, WIRE, STEM = .0275, .001, .001


def wires():
    a = np.linspace(3*np.pi/4, 5*np.pi/4, 17)
    p = RADIUS*np.column_stack([np.cos(a), np.zeros(len(a)), np.sin(a)])
    return [Capsule(f'w{i}', p[i], p[i+1], WIRE) for i in range(16)]


def target(x=-.0252, y0=-.01, y1=.01):
    return Capsule('target', np.array([x, y0, 0.]), np.array([x, y1, 0.]), STEM)


def inspect(t):
    return seating_geometry(t, np.zeros(3), Rotation.identity(), wires(),
                            ring_radius=RADIUS, max_surface_gap_m=.0015)


def test_independent_inner_seating_positive():
    m = inspect(target())
    assert m['geometric_seating_candidate'], m
    assert 0 < m['minimum_rear_surface_gap_m'] < .001
    assert m['hook_success'] is None and m['training_eligible'] is False


@pytest.mark.parametrize('case,reason', [
    (target(.0252), 'not_rear_half'),
    (target(0.), 'too_far_from_rear_wire'),
    (target(-.030), 'not_inside_wire_inner_footprint'),
    (target(-.0265), 'intersects_rear_wire'),
    (target(y0=.0005), 'no_finite_centreline_plane_crossing'),
])
def test_independent_negative_controls(case, reason):
    m = inspect(case)
    assert not m['geometric_seating_candidate']
    assert reason in m['reasons']


def test_coplanar_contact_not_assumed_to_cross():
    t = Capsule('target', np.array([-.0252, 0, -.002]), np.array([-.0252, 0, .002]), STEM)
    assert 'coplanar_or_parallel_axis' in inspect(t)['reasons']


@pytest.mark.parametrize('a,b,c,d,distance', [
    ([0,0,0],[1,0,0],[.5,-1,0],[.5,1,0],0.),
    ([0,0,0],[1,0,0],[.5,-1,2],[.5,1,2],2.),
    ([0,0,0],[1,0,0],[2,1,0],[3,1,0],np.sqrt(2)),
    ([0,0,0],[1,0,0],[.2,1,0],[.7,1,0],1.),
    ([0,0,0],[0,0,0],[0,1,0],[1,1,0],1.),
    ([0,0,0],[0,0,0],[0,1,0],[0,1,0],1.),
])
def test_finite_distance_controls(a,b,c,d,distance):
    assert segment_distance(a,b,c,d) == pytest.approx(distance)
    assert segment_distance(c,d,a,b) == pytest.approx(distance)


def test_goal_exact_gap_and_geometric_screen():
    t = target(x=.12)
    goal = seating_goal(t,wires()[7],Rotation.identity(),fraction=.5,surface_gap_m=.0004)
    assert goal['chosen_wire_surface_gap_m'] == pytest.approx(.0004, abs=1e-12)
    m = seating_geometry(t, goal['ring_position_xyz'], Rotation.from_quat(goal['orientation_xyzw']),
                         wires(), ring_radius=RADIUS, max_surface_gap_m=.0015)
    assert m['geometric_seating_candidate'], m
    assert goal['physics_executed'] is False


def test_goal_follows_target_not_fruit_centre():
    first = target()
    shift = np.array([.017, -.003, .011])
    second = Capsule(first.name, first.a+shift, first.b+shift, first.radius)
    g1 = seating_goal(first, wires()[7], Rotation.identity(), fraction=.5, surface_gap_m=.0004)
    g2 = seating_goal(second, wires()[7], Rotation.identity(), fraction=.5, surface_gap_m=.0004)
    np.testing.assert_allclose(np.array(g2['ring_position_xyz'])-g1['ring_position_xyz'], shift, atol=1e-12)


def test_rotation_knob_changes_actual_goal_pose():
    t = target()
    a = seating_goal(t, wires()[7], Rotation.identity(), fraction=.5, surface_gap_m=.0004)
    r = Rotation.from_euler('xz', [15, -10], degrees=True)
    b = seating_goal(t, wires()[7], r, fraction=.5, surface_gap_m=.0004)
    assert np.linalg.norm(np.array(a['ring_position_xyz'])-b['ring_position_xyz']) > 1e-5
    assert Rotation.from_quat(b['orientation_xyzw']).magnitude() > .1


def test_global_rigid_transform_equivariance():
    t = target()
    rot, shift = Rotation.from_euler('xyz',[27,-31,58],degrees=True), np.array([1.,2.,3.])
    moved = Capsule('target',rot.apply(t.a.copy())+shift,rot.apply(t.b.copy())+shift,t.radius)
    a = seating_goal(t,wires()[6],Rotation.identity(),fraction=.4,surface_gap_m=.0002)
    b = seating_goal(moved,wires()[6],rot,fraction=.4,surface_gap_m=.0002)
    np.testing.assert_allclose(b['ring_position_xyz'],rot.apply(a['ring_position_xyz'])+shift,atol=1e-12)
    s1 = inspect(t)
    s2 = seating_geometry(moved,shift,rot,wires(),ring_radius=RADIUS,max_surface_gap_m=.0015)
    assert s1['geometric_seating_candidate'] == s2['geometric_seating_candidate']
    assert s1['minimum_rear_surface_gap_m'] == pytest.approx(s2['minimum_rear_surface_gap_m'],abs=1e-12)


def test_aligned_normal_and_facing_sign():
    t = Capsule('t',np.array([0.,0.,0.]),np.array([1.,2.,3.]),.001)
    preferred=Rotation.from_euler('xyz',[20,30,40],degrees=True)
    r=aligned_rotation(t,preferred)
    n=(t.b-t.a)/np.linalg.norm(t.b-t.a)
    assert abs(r.apply([0,1,0])@n) == pytest.approx(1.)
    assert r.apply([0,1,0])@preferred.apply([0,1,0]) >= 0


@pytest.mark.parametrize('fraction',[0.,1.,-.1,float('nan'),True])
def test_invalid_fractions(fraction):
    with pytest.raises(ValueError):
        seating_goal(target(),wires()[7],Rotation.identity(),fraction=fraction,surface_gap_m=0)


@pytest.mark.parametrize('gap',[-.001,float('nan'),float('inf'),True])
def test_invalid_gap(gap):
    with pytest.raises(ValueError):
        seating_goal(target(),wires()[7],Rotation.identity(),fraction=.5,surface_gap_m=gap)


def test_cannot_use_roundoff_to_allow_penetration():
    with pytest.raises(ValueError):
        seating_geometry(target(),[0,0,0],Rotation.identity(),wires(),ring_radius=RADIUS,
                         max_surface_gap_m=.0015,roundoff_m=.0005)


def test_input_capsule_not_mutated():
    a=np.array([0.,0.,0.]);b=np.array([0.,.02,0.]);t=Capsule('t',a,b,.001)
    a[:]=7
    np.testing.assert_array_equal(t.a,[0,0,0])
    with pytest.raises(ValueError):t.a[0]=1


def test_parallel_witness_rejected():
    w=Capsule('rear',np.array([-.0275,0,-.01]),np.array([-.0275,0,.01]),WIRE)
    t=Capsule('target',np.array([0,0,-.01]),np.array([0,0,.01]),STEM)
    with pytest.raises(ValueError,match='Parallel'):
        seating_goal(t,w,Rotation.identity(),fraction=.5,surface_gap_m=.0002)


def test_goal_export_marks_truth_and_omissions():
    from propose_hook_seating_goals import generate_goals
    result=generate_goals([target()],wires(),[0,0,0],Rotation.identity(),
        ring_radius=RADIUS,hook_ring_offset=[-.106,.006,0],surface_gap_m=.0004,
        max_surface_gap_m=.0015,tilts_deg=[0],fractions=[.5],max_goals=2)
    assert result['attempted']==16
    assert result['accepted_before_limit']>2
    assert result['exported_count']==2
    assert result['omitted_by_limit']==result['accepted_before_limit']-2
    for g in result['goals']:
        r=Rotation.from_quat(g['orientation_xyzw'])
        np.testing.assert_allclose(np.array(g['hook_body_position_xyz'])+r.apply([-.106,.006,0]),
                                   g['ring_position_xyz'],atol=1e-12)
        assert g['hook_success'] is None and g['physics_executed'] is False


def test_export_prevalidates_inputs_not_silent_zero_goals():
    from propose_hook_seating_goals import generate_goals
    with pytest.raises(ValueError,match='gap'):
        generate_goals([target()],wires(),[0,0,0],Rotation.identity(),
            ring_radius=RADIUS,hook_ring_offset=[0,0,0],surface_gap_m=-1,
            max_surface_gap_m=.0015,tilts_deg=[0],fractions=[.5])


def test_cli_help_does_not_need_mujoco():
    import subprocess
    script=Path(__file__).resolve().parents[1]/'scripts/propose_hook_seating_goals.py'
    r=subprocess.run([sys.executable,str(script),'--help'],capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    assert 'surface-gap-m' in r.stdout
