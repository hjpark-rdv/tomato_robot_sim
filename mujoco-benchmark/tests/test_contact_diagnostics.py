"""Focused checks for diagnostic geometry and private force interpretation."""
from pathlib import Path
import sys
import numpy as np
import mujoco as mj
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from hook_retention_diagnostic import capsule_endpoints, rear_capsule_geometry
from scipy.spatial.transform import Rotation
from diagnose_contact_timing import force_rows


def test_capsule_endpoints_follow_world_rotation():
    m=mj.MjModel.from_xml_string('<mujoco><worldbody><body pos="1 2 3" euler="0 90 0"><geom name="c" type="capsule" size=".002 .01"/></body></worldbody></mujoco>')
    d=mj.MjData(m);mj.mj_forward(m,d)
    a,b,r=capsule_endpoints(m,d,0)
    np.testing.assert_allclose(a,[.99,2,3]);np.testing.assert_allclose(b,[1.01,2,3]);assert r==.002


def test_legacy_seating_does_not_accept_open_side_or_distant_pass():
    rot=Rotation.identity()
    def seated(x):return rear_capsule_geometry([(np.array([x,-.01,0]),np.array([x,.01,0]),.001)],np.zeros(3),rot)[1]
    assert seated(-.0255)
    assert not seated(.0255)
    assert not seated(-.06)


def test_positive_gap_force_kept_and_inactive_contact_distinguished():
    for gap,active in [(.00025,True),(.00075,False)]:
        m=mj.MjModel.from_xml_string(f'<mujoco><option gravity="0 0 0"/><worldbody><geom name="plane" type="plane" size="1 1 .01"/><body pos="0 0 {gap+.02}"><joint type="slide"/><geom name="sphere" type="sphere" size=".02" mass="1"/></body></worldbody><contact><pair geom1="plane" geom2="sphere" margin=".0005" gap=".0005"/></contact></mujoco>')
        d=mj.MjData(m);mj.mj_forward(m,d);rows=force_rows(m,d,pair=[0,1]);assert len(rows)==1
        assert rows[0]['dist_m']>0 and rows[0]['active']==active
        assert (rows[0]['normal_force_N']>0)==active
