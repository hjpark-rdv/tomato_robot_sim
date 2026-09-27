"""Native capsule fixtures, not the production robot/plant or a harvest proof."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

mj = pytest.importorskip('mujoco')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
from hook_seating_geometry import Capsule,capsule_gap,seating_geometry,seating_goal
from pose_candidates import rear_capsule_geometry
from geometry import RING_RADIUS,WIRE_RADIUS


def arc():
    angles=np.linspace(3*np.pi/4,5*np.pi/4,17)
    points=RING_RADIUS*np.column_stack([np.cos(angles),np.zeros(17),np.sin(angles)])
    return [Capsule(f'w{i}',points[i],points[i+1],WIRE_RADIUS) for i in range(16)]


def text(values):return ' '.join(format(float(x),'.17g') for x in np.asarray(values).ravel())


def model_and_data(target,wire_capsules):
    root=ET.Element('mujoco')
    ET.SubElement(root,'option',gravity='0 0 0')
    world=ET.SubElement(root,'worldbody')
    for c in wire_capsules:
        ET.SubElement(world,'geom',name=c.name,type='capsule',fromto=text([c.a,c.b]),size=str(c.radius))
    body=ET.SubElement(world,'body',name='Target')
    ET.SubElement(body,'joint',type='slide',axis='1 0 0')
    ET.SubElement(body,'geom',name=target.name,type='capsule',fromto=text([target.a,target.b]),size=str(target.radius),mass='.01')
    model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    data=mj.MjData(model);mj.mj_forward(model,data)
    return model,data


@pytest.mark.parametrize('x,valid',[(-.0252,True),(.0252,False),(-.030,False),(-.0265,False)])
def test_independent_control_gaps_match_native(x,valid):
    target=Capsule('target',np.array([x,-.01,0]),np.array([x,.01,0]),.001)
    ws=arc();model,data=model_and_data(target,ws)
    native=[mj.mj_geomDistance(model,data,model.geom('target').id,model.geom(w.name).id,.2,np.zeros(6)) for w in ws]
    analytic=[capsule_gap(target,w) for w in ws]
    np.testing.assert_allclose(native,analytic,atol=2e-8,rtol=0)
    metric=seating_geometry(target,[0,0,0],Rotation.identity(),ws,
                            ring_radius=RING_RADIUS,max_surface_gap_m=.0015)
    assert metric['geometric_seating_candidate'] == valid


def test_legacy_upper_gap_bound_can_accept_wire_penetration():
    target=Capsule('target',np.array([-.0265,-.01,0]),np.array([-.0265,.01,0]),.001)
    gap,seated=rear_capsule_geometry([(target.a.copy(),target.b.copy(),target.radius)],np.zeros(3),Rotation.identity())
    assert gap<0 and seated  # Existing legacy behaviour, NOT a production-success assertion.
    metric=seating_geometry(target,[0,0,0],Rotation.identity(),arc(),
                            ring_radius=RING_RADIUS,max_surface_gap_m=.0015)
    assert not metric['geometric_seating_candidate']
    assert 'intersects_rear_wire' in metric['reasons']


@pytest.mark.parametrize('angles',[(0,0),(15,-10),(-20,12)])
def test_generated_pose_has_requested_native_gap(angles):
    target=Capsule('target',np.array([.1,-.012,.05]),np.array([.1,.012,.05]),.001)
    rot=Rotation.from_euler('xz',angles,degrees=True);ws=arc()
    goal=seating_goal(target,ws[7],rot,fraction=.5,surface_gap_m=.0004)
    p=np.array(goal['ring_position_xyz'])
    world_wires=[Capsule(w.name,p+rot.apply(w.a.copy()),p+rot.apply(w.b.copy()),w.radius) for w in ws]
    model,data=model_and_data(target,world_wires)
    measured=mj.mj_geomDistance(model,data,model.geom('target').id,model.geom('w7').id,.1,np.zeros(6))
    assert measured == pytest.approx(.0004,abs=2e-8)


def test_random_segment_distance_against_native():
    rng=np.random.default_rng(93027)
    for i in range(30):
        a,b,c,d=rng.uniform(-.04,.04,(4,3))
        target=Capsule('target',a,b,.0012);wire=Capsule('wire',c,d,.001)
        model,data=model_and_data(target,[wire])
        native=mj.mj_geomDistance(model,data,model.geom('target').id,model.geom('wire').id,.2,np.zeros(6))
        assert capsule_gap(target,wire) == pytest.approx(native,abs=2e-8),i
