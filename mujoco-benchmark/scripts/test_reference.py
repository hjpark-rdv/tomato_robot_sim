import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
from benchmark_mujoco import Engine,HOME
from suite import resample
from metrics import audit

@pytest.fixture(scope='module')
def engine():
    path=HOME/'assets/reference/reference.json'
    if not path.exists():pytest.skip('Run exporter first')
    return Engine(HOME/'models/plant_original_equivalent.xml',json.loads(path.read_text()))

def test_native_mass_com_and_initial_pose_preserved(engine):
    e=engine;e.reset();p=e.poses()
    for i,b in enumerate(e.ref['bodies']):
        body=e.model.body(b['name']);assert float(body.mass[0])==pytest.approx(b['mass'],rel=1e-10)
        np.testing.assert_allclose(p[i,:3],b['pose'][:3],atol=1e-9)
        np.testing.assert_allclose(body.ipos,b['com_pose_xyzw'][:3],atol=1e-9)
    assert e.model.nv==300 and e.model.neq==11
    assert sum(s['body']!=e.ref['tool_path'] for s in e.ref['shapes'])==376

def test_reset_restores_constraints_velocities_and_preload(engine):
    e=engine;e.reset();q=e.data.qpos.copy();e.data.qpos[1]=.1;e.data.qvel[:]=.2;e.data.eq_active[:]=0;e.data.qfrc_applied[:]=123
    e.reset();np.testing.assert_array_equal(q,e.data.qpos);assert not e.data.qvel.any();assert e.data.eq_active.all();np.testing.assert_array_equal(e.preload,e.data.qfrc_applied)

def test_ring_aperture_is_not_one_convex_hull(engine):
    shapes=[s for s in engine.ref['shapes'] if s['body']==engine.ref['tool_path']]
    assert len(shapes)==35
    ring=[s for s in shapes if '/RingCollision/' in s['path']]
    assert len(ring)==32 and all(s['type']=='capsule' for s in ring)
    from suite import RING
    for s in ring:
        a,b=np.asarray(s['ends'])-RING;t=np.clip(-a@(b-a)/np.dot(b-a,b-a),0,1)
        assert np.linalg.norm(a+t*(b-a))-s['radius']>.026

def test_resample_preserves_speed_and_last_pose():
    t=dict(sample_dt=1.,poses=[[0,0,0,0,0,0,1],[1,0,0,0,0,0,1]])
    times,poses=resample(t,120,2)
    assert times[-1]==3.;np.testing.assert_allclose(poses[120,0],1);np.testing.assert_allclose(poses[-1],poses[120])


def test_measured_timestamps_do_not_require_sample_dt():
    trial=dict(times_s=[0.,.01,.03],poses=[[0,0,0,0,0,0,1],[.01,0,0,0,0,0,1],[.03,0,0,0,0,0,1]])
    times,poses=resample(trial,100,0.)
    np.testing.assert_allclose(times,[0.,.01,.02,.03])
    np.testing.assert_allclose(poses[:,0],times)

def test_between_step_crossing_is_not_reported_safe():
    paths=['/ElasticPlant/STEM_MainStem_00','/Harvestables/Tomato_05','/ElasticPlant/TRUSS_Pedicel_proximal_05_00','/Attachment_04']
    rest=[[0,0,0,1,0,0,0]]+[[10,10,10,1,0,0,0]]*3
    r=dict(tool_path='/Hook',bodies=[dict(path=p,pose=q) for p,q in zip(paths,rest)],fruit_specs=[dict(name='Tomato_05',path=paths[1],anchor=paths[3],center=[0,0,0])],shapes=[
        dict(name='stem',path=paths[0]+'/StemCollider',body=paths[0],type='capsule',ends=[[0,0,-1],[0,0,1]],radius=.001),
        dict(name='hook',path='/Hook/RingCollision/0',body='/Hook',type='capsule',ends=[[0,-.01,0],[0,.01,0]],radius=.001)])
    poses=np.zeros((2,5,7));poses[:,:,-1]=1.;poses[:,1:4,:3]=10;poses[0,-1,0]=-.01;poses[1,-1,0]=.01
    result=audit(r,poses,np.array([0,1/60]),[[]])
    assert result['penetration_steps']==0
    assert result['tunneling_suspect_count']>0 and result['result']=='tunneling_suspect'


def test_optimized_preserves_mass_but_explicitly_changes_constraints(engine):
    path=HOME/'models/plant_mujoco_optimized.xml'
    if not path.exists():pytest.skip('Generate optimized model first')
    optimized=Engine(path,engine.ref)
    assert optimized.model.nv==234 and optimized.model.neq==0
    assert np.sum((optimized.model.geom_contype!=0)|(optimized.model.geom_conaffinity!=0))==90+35
    for b in engine.ref['bodies']:
        assert float(optimized.model.body(b['name']).mass[0])==pytest.approx(b['mass'],rel=1e-10)
    np.testing.assert_allclose(optimized.poses()[:-1,:3],engine.poses()[:-1,:3],atol=1e-6)
