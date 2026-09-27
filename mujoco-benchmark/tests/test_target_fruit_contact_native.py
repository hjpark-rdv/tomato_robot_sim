"""Small native contact and CAD-identity controls, not greenhouse execution."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
mj=pytest.importorskip('mujoco')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
from scipy.spatial.transform import Rotation
from diagnose_contact_timing import force_rows
from target_fruit_contact_trial import ContactScope,contact_stats,stop_reasons,TrialLimits,scope_from_engine
from suite import RING
from geometry import RING_RADIUS,WIRE_RADIUS


def test_native_positive_gap_contact_remains_physical_and_measured():
    m=mj.MjModel.from_xml_string('<mujoco><option gravity="0 0 0"/><worldbody><geom name="wire" type="plane" size="1 1 .01"/><body pos="0 0 .02025"><joint type="slide"/><geom name="fruit" type="sphere" size=".02" mass="1"/></body></worldbody><contact><pair geom1="wire" geom2="fruit" margin=".0005" gap=".0005"/></contact></mujoco>')
    d=mj.MjData(m);mj.mj_forward(m,d)
    masks=m.geom_contype.copy();scope=ContactScope('fruit',('wire',),('pedicel',),('wire',))
    raw=force_rows(m,d);assert raw and raw[0]['dist_m']>0 and raw[0]['normal_force_N']>0
    stats=contact_stats(raw,scope,['seat']);force=stats['target_fruit_touch']['force_sum_N']
    assert force>0 and stats['forbidden_contact']['force_sum_N']==0
    assert stop_reasons([stats],0.,0.,True,TrialLimits(force*.5,.005))==['target_force_limit']
    np.testing.assert_array_equal(masks,m.geom_contype)
    assert d.ncon==1


def arc_engine(count=32):
    points=np.column_stack([np.cos(np.linspace(np.pi/2,3*np.pi/2,count+1)),np.zeros(count+1),np.sin(np.linspace(np.pi/2,3*np.pi/2,count+1))])*RING_RADIUS+RING
    geoms=''.join(f'<geom name="w{i}" type="capsule" size="{WIRE_RADIUS}" fromto="'+ ' '.join(map(str,np.r_[points[i],points[i+1]]))+'"/>' for i in range(count))
    geoms+='<geom name="mount" type="box" size=".01 .01 .01"/>'
    m=mj.MjModel.from_xml_string('<mujoco><worldbody><body name="Hook">'+geoms+'</body><body name="Tomato_02" pos="1 0 0"><geom name="glb_col_Tomato_02" type="sphere" size=".01"/></body></worldbody></mujoco>')
    d=mj.MjData(m);mj.mj_forward(m,d)
    e=SimpleNamespace(model=m,data=d,hook=m.body('Hook').id,hookgeoms=np.arange(count+1),fruit=m.body('Tomato_02').id,target='Tomato_02')
    mapping=SimpleNamespace(frame=lambda data:(RING,Rotation.identity()),target_names=['ped0','ped1'],rear_names={f'w{i}' for i in range(8,min(24,count))})
    return e,mapping


def test_native_inventory_includes_front_wire_not_mount():
    e,mapping=arc_engine();scope=scope_from_engine(e,mapping)
    assert len(scope.wires)==32 and 'w0' in scope.wires and 'mount' not in scope.wires
    assert scope.fruit=='glb_col_Tomato_02'


def test_wrong_arc_count_fails_closed():
    e,mapping=arc_engine(31)
    with pytest.raises(ValueError,match='32-wire'):scope_from_engine(e,mapping)
