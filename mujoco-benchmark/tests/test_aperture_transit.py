"""Independent geometry and schema tests; NOT a greenhouse execution test."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from propose_aperture_transit import build_candidate, pivot_position

C = np.array([-.696152523661374, .9158217278439791, .4772317802256101])
Q = np.array([.5000460705641007, -.4999539251905144, .4999539251905144, -.5000460705641007])
R = Rotation.from_quat(Q)
RADIUS = .01390959529856534
GOAL = np.array([-.7022053187722946, .8948237864349288, .4934833653902833])


def fixture():
    poses = [("preapproach", [-.6961211942344877,.7458217307308408,.46032218492704474], 0),
             ("approach", [-.6961430493486849,.8644121334184278,.46032218492704474], 0),
             ("insert", GOAL-np.array([0,0,.04]), 0), ("seat", GOAL, 0),
             ("hold", GOAL, 1.1), ("verify", GOAL+R.apply([0,0,.0002]), .5),
             ("verify", GOAL, .5)]
    return dict(candidate_id="seating_00_under", trajectory_mode="diagnostic_pose_waypoints_v1",
                diagnostic_only=True, training_eligible=False, target_geom="target_proximal",
                source_goal="seat_goal_0000", pose_waypoints=[dict(phase=n,ring_position_xyz=np.asarray(p).tolist(),
                orientation_xyzw=Q.tolist(),minimum_seconds=t) for n,p,t in poses])


def build(source=None, **kwargs):
    return build_candidate(fixture() if source is None else source, C, RADIUS,
                           wire_radius=.001, aperture_inradius_m=.02646, plane_buffer_m=.001, **kwargs)


def ideal_wire_gap(p, rot):
    # Independent geometric model: authored 32-segment, 27.5 mm half-circle,
    # 1 mm wire. The enclosing sphere is not the actual fruit mesh.
    angles=np.linspace(np.pi/2,3*np.pi/2,33)
    v=.0275*np.column_stack([np.cos(angles),np.zeros(33),np.sin(angles)])
    a,b=v[:-1],v[1:];u=b-a;f=rot.inv().apply(C-p)
    t=np.clip(np.sum((f-a)*u,axis=1)/np.sum(u*u,axis=1),0,1)
    return np.min(np.linalg.norm(f-a-t[:,None]*u,axis=1)-RADIUS-.001)


def leg_min(a,b,n=101):
    pa=np.asarray(a['ring_position_xyz']);pb=np.asarray(b['ring_position_xyz'])
    ra=Rotation.from_quat(a['orientation_xyzw']);rb=Rotation.from_quat(b['orientation_xyzw'])
    delta=(rb*ra.inv()).as_rotvec()
    return min(ideal_wire_gap((1-u)*pa+u*pb,Rotation.from_rotvec(u*delta)*ra) for u in np.linspace(0,1,n))


def test_recorded_offset_is_not_an_aperture_transit_offset():
    f=R.inv().apply(C-GOAL)
    assert np.linalg.norm(f[[0,2]]) == pytest.approx(.0218529144983614)
    assert .0275-.001-RADIUS == pytest.approx(.01259040470143466)
    old=fixture()['pose_waypoints']
    assert leg_min(old[2],old[3],1001)<-.009


def test_centered_raise_then_transfer_clears_ideal_fruit_bound():
    rows=build()['pose_waypoints']
    # Prefix is joint-space planned elsewhere; only the local connector is compared.
    gap=min(leg_min(a,b,301) for a,b in zip(rows[1:],rows[2:]))
    assert gap > .002
    old=fixture()['pose_waypoints']
    assert leg_min(old[2],old[3],1001)<0


def test_no_rotation_needed_to_repair_this_local_sphere_path():
    out=build();rows=out['pose_waypoints']
    assert all((Rotation.from_quat(x['orientation_xyzw'])*R.inv()).magnitude()<1e-10 for x in rows)
    np.testing.assert_allclose(rows[2]['ring_position_xyz'], C+[0,0,-.0237484148353268],atol=1e-10)
    np.testing.assert_allclose(rows[3]['ring_position_xyz'], C+[0,0,.0162515851646732],atol=1e-10)


def test_prefix_suffix_and_source_preserved():
    source=fixture();saved=copy.deepcopy(source);out=build(source)
    assert source==saved
    assert out['pose_waypoints'][:2]==source['pose_waypoints'][:2]
    assert out['pose_waypoints'][-3:]==source['pose_waypoints'][-3:]
    last_seat=[p for p in out['pose_waypoints'] if p['phase']=='seat'][-1]
    np.testing.assert_allclose(last_seat['ring_position_xyz'],GOAL,atol=1e-12)
    assert out['target_geom']==source['target_geom'] and out['source_goal']==source['source_goal']
    assert out['physics_executed'] is False and out['hook_success'] is None
    assert out['training_eligible'] is False
    assert not out['connection_proposal']['full_geometry_checked']
    assert out['connection_proposal']['source_verify_is_not_load_validation']


def test_seat_translation_has_explicit_slow_timing():
    rows=build()['pose_waypoints']
    for k in range(4,len(rows)-3):
        distance=np.linalg.norm(np.array(rows[k]['ring_position_xyz'])-rows[k-1]['ring_position_xyz'])
        assert 1.5*distance/rows[k]['minimum_seconds']<=.002+1e-10
        assert rows[k]['minimum_seconds']<=10


def test_tilt_changes_position_with_fruit_pivot():
    source=fixture();goal_r=R*Rotation.from_euler('x',10,degrees=True)
    for row in source['pose_waypoints'][3:]:
        row['orientation_xyzw']=goal_r.as_quat().tolist()
    out=build(source);inserts=[x for x in out['pose_waypoints'] if x['phase']=='insert']
    assert len(inserts)>=3
    depth=out['connection_proposal']['centred_below_plane_m']
    for x in inserts:
        r=Rotation.from_quat(x['orientation_xyzw']);p=np.asarray(x['ring_position_xyz'])
        np.testing.assert_allclose(r.inv().apply(C-p),[0,-depth,0],atol=1e-12)
    assert np.linalg.norm(np.array(inserts[0]['ring_position_xyz'])-inserts[-1]['ring_position_xyz'])>.003


def test_pivot_equivariance():
    rot=Rotation.from_euler('xyz',[.3,.4,.1]);p= pivot_position(C,[.003,-.02,0],R)
    t=np.array([1.,2.,3.])
    np.testing.assert_allclose(pivot_position(rot.apply(C)+t,[.003,-.02,0],rot*R),rot.apply(p)+t)


@pytest.mark.parametrize('value',[0,-.01,float('nan'),float('inf'),True])
def test_reject_bad_radius(value):
    with pytest.raises(ValueError):
        build_candidate(fixture(),C,value,wire_radius=.001,aperture_inradius_m=.02646,plane_buffer_m=.001)


@pytest.mark.parametrize('flag',['diagnostic_only','training_eligible'])
def test_requires_diagnostic_flags(flag):
    s=fixture();s[flag]=not s[flag]
    with pytest.raises(ValueError):build(s)


def test_wrong_plane_side_is_family_rejection_not_forced_goal_change():
    s=fixture();s['pose_waypoints'][3]['ring_position_xyz']=(GOAL-np.array([0,0,.04])).tolist()
    with pytest.raises(ValueError,match='opposite plane side'):build(s)


def test_hard_waypoint_budget():
    with pytest.raises(ValueError,match='budget|limit'):build(max_waypoints=6)


def test_does_not_silently_transform_another_path_schema():
    s=fixture();s['pose_waypoints'].insert(3,copy.deepcopy(s['pose_waypoints'][2]))
    with pytest.raises(ValueError,match='template'):build(s)


def test_cli_hashes_inputs_and_never_overwrites(tmp_path):
    inp=tmp_path/'inputs';inp.mkdir()
    s=inp/'candidates.json';s.write_text(json.dumps([fixture()]))
    p=inp/'planning.json';p.write_text(json.dumps({'inputs':{'geometry':[C.tolist()], 'target_radius':RADIUS}}))
    script=Path(__file__).resolve().parents[1]/'scripts/propose_aperture_transit.py'
    cmd=[sys.executable,str(script),'--source-candidates',str(s),'--planning-inputs',str(p),
         '--output',str(tmp_path/'out'),'--wire-radius-m','.001','--plane-buffer-m','.001','--aperture-inradius-m','.02646']
    old=s.read_bytes()
    result=subprocess.run(cmd,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert s.read_bytes()==old
    report=json.loads((tmp_path/'out/proposal_report.json').read_text())
    assert report['proposed']==1 and report['physics_executed'] is False
    assert len(report['source_sha256'])==2
    assert subprocess.run(cmd,capture_output=True).returncode!=0


def test_rejects_unsafe_candidate_identity():
    s=fixture();s['candidate_id']='../../elsewhere'
    with pytest.raises(ValueError,match='identity'):build(s)


def test_fruit_bound_must_fit_aperture_with_requested_buffer():
    with pytest.raises(ValueError,match='does not fit'):
        build_candidate(fixture(),C,.026,wire_radius=.001,aperture_inradius_m=.02646,plane_buffer_m=.001)
