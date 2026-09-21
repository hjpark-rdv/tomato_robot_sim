import json
import os
from pathlib import Path
import subprocess
import sys

from dataset_design import candidates

HERE=Path(__file__).resolve().parent


def prepare(tmp_path, *extra):
    return subprocess.run([sys.executable,str(HERE/'gpu_dataset_runner.py'),
        '--run-dir',str(tmp_path),'--num-envs','1','--candidates','32','--prepare-only',*extra],
        text=True,capture_output=True,env={**os.environ,'PYTHONPATH':str(HERE)})


def test_selected_candidates_keep_original_identity_and_gpu_provenance(tmp_path):
    result=prepare(tmp_path,'--candidate-indices','3,27')
    assert result.returncode==0,result.stderr
    proposal=json.loads((tmp_path/'candidates.json').read_text())
    original=candidates(32)
    assert proposal==[original[3],original[27]]
    config=json.loads((tmp_path/'config.json').read_text())
    assert config['physics_device']=='gpu' and config['tensor_device']=='cpu'
    assert config['solver']=='PGS' and config['experimental']
    assert config['gpu_max_num_partitions']==1
    assert not config['cpu_tgs_equivalence_guaranteed']
    assert prepare(tmp_path,'--candidate-indices','3,27','--resume').returncode==0
    assert prepare(tmp_path,'--candidate-indices','3','--resume').returncode!=0


def test_reject_duplicate_or_out_of_range_indices(tmp_path):
    assert prepare(tmp_path/'duplicates','--candidate-indices','3,3').returncode!=0
    assert prepare(tmp_path/'outside','--candidate-indices','32').returncode!=0
    assert not (tmp_path/'outside'/'config.json').exists()


def test_scheduling_mode_is_recorded_and_cannot_change_on_resume(tmp_path):
    result=prepare(tmp_path,'--schedule','continuous')
    assert result.returncode==0,result.stderr
    assert json.loads((tmp_path/'config.json').read_text())['schedule']=='continuous'
    assert prepare(tmp_path,'--schedule','continuous','--resume').returncode==0
    assert prepare(tmp_path,'--schedule','batch','--resume').returncode!=0


def test_sigterm_cleans_up_independent_gpu_worker(tmp_path):
    import signal
    import time
    fake=tmp_path/'worker';fake.mkdir()
    (fake/'gpu_dataset_sim.py').write_text('''import os,sys,time
from pathlib import Path
root=Path(sys.argv[sys.argv.index('--run-dir')+1])
(root/'child.pid').write_text(str(os.getpid()))
print('[DATASET] fake worker ready',flush=True)
while True:time.sleep(.05)
''')
    run=tmp_path/'run'
    code="import gpu_dataset_runner as m;from pathlib import Path;m.HERE=Path("+repr(str(fake))+");m.main()"
    parent=subprocess.Popen([sys.executable,'-c',code,'--run-dir',str(run),'--num-envs','1','--candidates','1'],
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,env={**os.environ,'PYTHONPATH':str(HERE)})
    child=None
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not (run/'child.pid').exists():
            if parent.poll() is not None:break
            time.sleep(.05)
        assert (run/'child.pid').exists()
        child=int((run/'child.pid').read_text())
        parent.send_signal(signal.SIGTERM)
        assert parent.wait(timeout=10)==130
        try:os.kill(child,0)
        except ProcessLookupError:pass
        else:raise AssertionError('GPU worker survived parent termination')
    finally:
        if parent.poll() is None:parent.kill();parent.wait()
        if child is not None:
            try:os.kill(child,signal.SIGKILL)
            except ProcessLookupError:pass


def test_thousand_environment_request_is_prepared_without_claiming_validation(tmp_path):
    result=prepare(tmp_path,'--num-envs','1000')
    assert result.returncode==0,result.stderr
    config=json.loads((tmp_path/'config.json').read_text())
    assert config['num_envs']==1000 and config['experimental']
    assert not (tmp_path/'validation_complete.json').exists()


def test_native_capacity_error_aborts_owned_worker_and_marks_dataset_invalid(tmp_path):
    fake=tmp_path/'worker';fake.mkdir()
    (fake/'gpu_dataset_sim.py').write_text('''import os,sys,time
from pathlib import Path
root=Path(sys.argv[sys.argv.index('--run-dir')+1])
(root/'child.pid').write_text(str(os.getpid()))
print('[Error] [omni.physx.plugin] PhysX error: increase foundLostPairsCapacity, otherwise, the simulation will miss interactions',flush=True)
while True:time.sleep(.05)
''')
    run=tmp_path/'run'
    code="import gpu_dataset_runner as m;from pathlib import Path;m.HERE=Path("+repr(str(fake))+");m.main()"
    result=subprocess.run([sys.executable,'-c',code,'--run-dir',str(run),'--num-envs','1','--candidates','1'],
                          capture_output=True,text=True,timeout=15,env={**os.environ,'PYTHONPATH':str(HERE)})
    assert result.returncode!=0
    error=json.loads((run/'execution_error.json').read_text())
    assert error['type']=='NativePhysicsError' and not error['dataset_valid']
    child=int((run/'child.pid').read_text())
    try:os.kill(child,0)
    except ProcessLookupError:pass
    else:raise AssertionError('Invalid-physics worker was left running')


def test_large_batch_captures_once_then_starts_physics_without_renderer(tmp_path):
    fake=tmp_path/'worker';fake.mkdir()
    (fake/'gpu_dataset_sim.py').write_text('''import os,sys,json
from pathlib import Path
root=Path(sys.argv[sys.argv.index('--run-dir')+1])
previous=root/'previous.pid'
if previous.exists():
    try:os.kill(int(previous.read_text()),0)
    except ProcessLookupError:pass
    else:raise RuntimeError('The observation simulator is still alive')
previous.write_text(str(os.getpid()))
with (root/'launches.jsonl').open('a') as stream:stream.write(json.dumps(sys.argv)+'\\n')
name='observation_complete.json' if '--observation-only' in sys.argv else 'validation_complete.json'
(root/name).write_text(json.dumps({'passed':True}))
print('[DATASET] fake phase complete',flush=True)
''')
    run=tmp_path/'run'
    code="import gpu_dataset_runner as m;from pathlib import Path;m.HERE=Path("+repr(str(fake))+");m.main()"
    result=subprocess.run([sys.executable,'-c',code,'--run-dir',str(run),'--num-envs','1000','--candidates','1','--validate-only'],
                          capture_output=True,text=True,timeout=15,env={**os.environ,'PYTHONPATH':str(HERE)})
    assert result.returncode==0,result.stderr
    launches=[json.loads(row) for row in (run/'launches.jsonl').read_text().splitlines()]
    assert len(launches)==2 and '--observation-only' in launches[0] and '--enable_cameras' in launches[0]
    assert '--observation-only' not in launches[1] and '--enable_cameras' not in launches[1] and '--headless' in launches[1]
    assert json.loads((run/'config.json').read_text())['observation_mode']=='isolated_single_environment'


def test_live_grid_enables_gui_and_keeps_candidate_and_physics_configuration(tmp_path):
    result=prepare(tmp_path,'--num-envs','16','--view-grid','--keep-open')
    assert result.returncode==0,result.stderr
    config=json.loads((tmp_path/'config.json').read_text())
    assert config['gui'] and config['view_grid'] and config['keep_open']
    assert config['num_envs']==16 and config['gpu_max_num_partitions']==1
    assert json.loads((tmp_path/'candidates.json').read_text())==candidates(32)


def test_keep_open_without_grid_is_rejected(tmp_path):
    assert prepare(tmp_path,'--keep-open').returncode!=0
