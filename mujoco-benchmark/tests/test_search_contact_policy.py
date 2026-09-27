import copy
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import target_fruit_contact_trial as tc
from target_truss_identity import target_names,target_rachis_geoms
from environment_preflight import pair_allowed

S=tc.ContactScope('fruit',('front','rear'),('ped',),('rear',),rachis=('own_rachis',),search_policy=True,truss_root='own_root')


def base():
    return dict(clearance_m=0.,allowed_contacts=[dict(robot_geom='rear',environment_geom='ped',phases=['seat','hold','verify'])])


def contact(a,b,force=.1):
    return dict(geoms=[a,b],force6=[force,0,0,0,0,0],dist_m=.0001,active=True)


@pytest.mark.parametrize('phase',['preapproach','approach','insert','seat','hold','verify'])
def test_own_rachis_touch_authorized_but_not_capture(phase):
    c=contact('front','own_rachis')
    assert tc.category(c,S,[phase])=='target_rachis_touch'
    phases=['ready','preapproach','approach','insert','seat','hold','verify']
    p=tc.trial_policy(base(),S,phases)
    assert pair_allowed(p,'front','own_rachis',[phase])


@pytest.mark.parametrize('a,b',[('rear','other_rachis'),('rear','other_fruit'),('mount','own_rachis'),('arm','fruit'),('rear','main_stem'),('rear','gutter')])
def test_authorization_does_not_expand_to_wrong_objects(a,b):
    assert tc.category(contact(a,b),S,['seat'])=='forbidden_contact'


def test_front_pedicel_touch_not_used_as_retention_evidence():
    assert tc.category(contact('front','ped'),S,['seat'])=='target_pedicel_touch'
    assert tc.category(contact('rear','ped'),S,['approach'])=='target_pedicel_touch'
    assert tc.category(contact('rear','ped'),S,['seat'])=='target_pedicel_contact'


def test_rachis_force_is_not_discarded_and_materials_not_changed():
    stats=tc.contact_stats([contact('rear','own_rachis',.6),contact('rear','fruit',.6)],S,['seat'])
    assert tc.stop_reasons([stats],0.,0.,True,tc.TrialLimits(1.,.02))==['target_force_limit']
    assert tc.stop_reasons([stats],0.,.0006,True,tc.TrialLimits(5.,.02))==['physics_penetration']
    b=base();old=copy.deepcopy(b);tc.trial_policy(b,S,['insert','seat','hold','verify'])
    assert b==old


def test_multi_truss_names_are_exact():
    names=target_names('truss_01__Tomato_02','/GLB/truss_01__Attachment_01')
    assert names['fruit']=='truss_01__glb_col_Tomato_02'
    assert names['pedicels'][0]=='truss_01__glb_col_Attachment_01'
    assert names['pedicels'][1]=='truss_01__glb_col_TRUSS_Pedicel_proximal_02_02'
    with pytest.raises(ValueError):target_names('truss_01__Tomato_02','/GLB/Attachment_01')
    assert target_names('Tomato_02','/GLB/Attachment_01')['fruit']=='glb_col_Tomato_02'


def test_native_ownership_is_truss_scoped():
    mj=pytest.importorskip('mujoco')
    from types import SimpleNamespace
    xml='''<mujoco><worldbody>
    <body name="STEM_MainStem_00"><body name="TRUSS_Truss_01_Peduncle_00">
      <body name="Tomato_02"><geom name="glb_col_Tomato_02" type="sphere" size=".01"/></body>
      <body name="TRUSS_Rachis_00"><geom name="glb_col_TRUSS_Rachis_00" type="capsule" size=".001 .01"/></body>
    </body><body name="truss_01__TRUSS_Truss_01_Peduncle_00">
      <body name="truss_01__Tomato_02"><geom name="truss_01__glb_col_Tomato_02" type="sphere" size=".01"/></body>
      <body name="truss_01__TRUSS_Rachis_00"><geom name="truss_01__glb_col_TRUSS_Rachis_00" type="capsule" size=".001 .01"/></body>
    </body></body></worldbody></mujoco>'''
    m=mj.MjModel.from_xml_string(xml)
    before=m.geom_contype.copy()
    e=SimpleNamespace(model=m,target='truss_01__Tomato_02',target_spec={'anchor':'/GLB/truss_01__Attachment_01'},fruit=m.body('truss_01__Tomato_02').id)
    names,root=target_rachis_geoms(e)
    assert names==('truss_01__glb_col_TRUSS_Rachis_00',)
    assert root=='truss_01__TRUSS_Truss_01_Peduncle_00'
    assert np.array_equal(before,m.geom_contype)
    e.fruit=m.body('Tomato_02').id
    with pytest.raises(ValueError):target_rachis_geoms(e)
