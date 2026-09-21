import copy
from types import SimpleNamespace
import numpy as np
import pytest

from gpu_planning import validate_inputs, restore_model
from pose_collision import SelfCollisionCheck


def test_reset_mismatch_rejects_preplanned_path():
    data=dict(geometry=np.zeros((3,3)),start=np.zeros(7),step_dt=1/60,
              lift_id=0,rise_speed=.002,pull_speed=.004)
    validate_inputs(data,copy.deepcopy(data))
    other=copy.deepcopy(data);other['geometry'][0,1]=.001
    with pytest.raises(RuntimeError,match='geometry'):validate_inputs(data,other)
    other=copy.deepcopy(data);other['step_dt']=1/30
    with pytest.raises(RuntimeError,match='step_dt'):validate_inputs(data,other)


def collision_model():
    corners=np.array([[x,y,z] for x in [-.1,.1] for y in [-.1,.1] for z in [-.2,.2]])
    return dict(world=np.eye(4),root='root',manifest={},pairs=np.array([[1,0]]),
        joints=[('root','moving',np.eye(4),np.array([1.,0,0]),'prismatic',0)],
        shapes=[dict(path=link,link=link,group=link,local=np.eye(4),corners=corners,
                     geometry=dict(kind='capsule',radius=.1,height=.2)) for link in ('root','moving')])


def test_collision_model_roundtrip_preserves_overlap_and_separation():
    original=SelfCollisionCheck.from_model(collision_model())
    restored=SelfCollisionCheck.from_model(original.export_model())
    for q in ([0.],[.05],[.5]):
        assert original.check(np.array(q))==restored.check(np.array(q))
    assert restored.check(np.array([0.])) is not None
    assert restored.check(np.array([.5])) is None


def test_offline_restore_preserves_float32_and_has_no_live_simulator():
    model=dict(inputs=dict(geometry=[np.ones(3,dtype=np.float32)]*3,
        start=np.zeros(1,dtype=np.float32),step_dt=1/60,lift_id=0,rise_speed=.002,pull_speed=.004),
        kinematics=dict(chain=[],world=np.eye(4),bounds=np.array([[-1.],[1.]])),collision=collision_model())
    env,kin,checker=restore_model(model)
    assert env.robot.data.joint_pos.numpy().dtype==np.float32
    assert env._target_geometry()[0].numpy().dtype==np.float32
    assert not hasattr(env,'sim')
    assert not hasattr(kin,'env')
    assert checker.check(np.array([.5])) is None


def test_real_process_service_uses_cpu_planner_and_publishes_atomically(tmp_path):
    import json
    import time
    from dataset_design import candidates
    from gpu_planning import PlanningService
    model=dict(inputs=dict(geometry=[np.array(v,dtype=np.float32) for v in ([0,0,1],[0,0,1.02],[0,0,1])],
        start=np.zeros(1,dtype=np.float32),step_dt=1/60,lift_id=0,rise_speed=.002,pull_speed=.004),
        kinematics=dict(chain=[],world=np.eye(4),bounds=np.array([[-1.],[1.]])),collision=collision_model())
    env,_,_=restore_model(model)
    proposals=candidates(4)
    service=PlanningService(tmp_path/'service',model,proposals,2)
    try:
        values=[service.take(p['candidate_id'],env)[0] for p in proposals]
        assert all(v['planned'] is None and v['preflight']['reason']=='endpoint_ik' for v in values)
        assert service.process.wait(timeout=20)==0
        summary=json.loads((service.folder/'summary.json').read_text())
        assert summary['complete'] and summary['completed']==4
        assert not list(service.folder.glob('*.tmp'))
    finally:service.close()


def test_exact_plan_comparison_rejects_changed_commands_and_rejections():
    from gpu_planning_benchmark import compare_plans
    from scipy.spatial.transform import Rotation
    p=dict(commands=np.zeros((3,7)),direction=np.array([1.,0,0]),prehook=np.ones(3),
        target_center=np.zeros(3),target_neck=np.ones(3),phases=['insert']*3,waypoints=[],orientation=Rotation.identity())
    assert compare_plans((p,{'passed':True}),(copy.deepcopy(p),{'passed':True}))
    other=copy.deepcopy(p);other['commands'][1,2]=1e-10
    assert not compare_plans((p,{}),(other,{}))
    assert not compare_plans((None,{'reason':'joint_limits'}),(None,{'reason':'self_collision'}))


def test_planning_children_stop_if_simulator_parent_is_killed(tmp_path):
    import json
    import os
    import pickle
    import signal
    import subprocess
    import sys
    import time
    from pathlib import Path
    model=dict(inputs=dict(geometry=[np.array(v,dtype=np.float32) for v in ([0,0,1],[0,0,1.02],[0,0,1])],
        start=np.zeros(1,dtype=np.float32),step_dt=1/60,lift_id=0,rise_speed=.002,pull_speed=.004),
        kinematics=dict(chain=[],world=np.eye(4),bounds=np.array([[-1.],[1.]])),collision=collision_model())
    model_path=tmp_path/'model.pkl';model_path.write_bytes(pickle.dumps(model))
    helper=tmp_path/'parent.py'
    helper.write_text('''import pickle,sys,time
from pathlib import Path
from dataset_design import candidates
from gpu_planning import PlanningService
root=Path(sys.argv[1])
service=PlanningService(root/'service',pickle.loads((root/'model.pkl').read_bytes()),candidates(1024),2)
(root/'service.pid').write_text(str(service.process.pid))
while True:time.sleep(1)
''')
    parent=subprocess.Popen([sys.executable,str(helper),str(tmp_path)])
    group=None
    def live_group():
        live=[]
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:fields=path.read_text().split(') ',1)[1].split()
            except (OSError,IndexError):continue
            if int(fields[2])==group and fields[0]!='Z':live.append(path.parent.name)
        return live
    try:
        deadline=time.monotonic()+10
        while not (tmp_path/'service.pid').exists() and time.monotonic()<deadline:time.sleep(.05)
        group=int((tmp_path/'service.pid').read_text())
        # Allow the service to start child initialization, then simulate a hard
        # Kit crash; no parent finally block can clean up for us.
        time.sleep(.7);parent.kill();parent.wait(timeout=5)
        deadline=time.monotonic()+15
        while live_group() and time.monotonic()<deadline:time.sleep(.1)
        assert not live_group()
    finally:
        if parent.poll() is None:parent.kill();parent.wait()
        if group and live_group():
            try:os.killpg(group,signal.SIGKILL)
            except ProcessLookupError:pass
