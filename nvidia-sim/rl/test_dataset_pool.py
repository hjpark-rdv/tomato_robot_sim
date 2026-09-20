import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest
from dataset_pool import partition,collect
from dataset_design import candidates,write_json


@pytest.mark.parametrize('count,workers,batch',[(1,4,1),(11,4,1),(19,4,4),(1000,4,1),(1000,6,1),(29,6,4),(1000,8,1),(37,8,4)])
def test_shards_cover_original_candidates_once_without_resampling(count,workers,batch):
    original=candidates(count);shards=partition(original,workers,batch)
    flat=[p for shard in shards for p in shard]
    assert sorted(flat,key=lambda p:p['candidate_id'])==original
    for shard in shards:
        for offset in range(0,len(shard),batch):
            ids=[int(p['candidate_id'].split('_')[1]) for p in shard[offset:offset+batch]]
            assert ids==list(range(ids[0],ids[0]+len(ids)))


def test_pool_index_references_worker_data_and_cannot_hide_incomplete(tmp_path):
    shards=partition(candidates(2),2,1)
    for i,shard in enumerate(shards):
        folder=tmp_path/f'worker_{i:02d}'/'results'/shard[0]['candidate_id'];folder.mkdir(parents=True)
        write_json(folder/'candidate.json',dict(candidate_id=shard[0]['candidate_id'],target_id='Tomato_05',
            result='incomplete' if i else 'miss',hook_success=False,observation_path='scene_0001/observation.json'))
    summary=collect(tmp_path,shards,10.,{'0':'complete','1':'complete'})
    assert summary['execution_complete'] and not summary['dataset_complete']
    rows=[json.loads(s) for s in (tmp_path/'candidates.jsonl').read_text().splitlines()]
    assert rows[1]['observation_path']=='worker_01/scene_0001/observation.json'
    assert (tmp_path/rows[1]['candidate_record_path']).exists()
    summary=collect(tmp_path,shards,10.,{'0':'complete','1':'failed'})
    assert not summary['execution_complete']


def test_pool_rejects_unassigned_candidate(tmp_path):
    shards=partition(candidates(2),2,1)
    folder=tmp_path/'worker_00'/'results'/'candidate_00001';folder.mkdir(parents=True)
    write_json(folder/'candidate.json',{'candidate_id':'candidate_00001','result':'miss'})
    with pytest.raises(RuntimeError,match='Unexpected candidate'):collect(tmp_path,shards,0.,{'0':'running','1':'running'})


def test_sigterm_to_pool_reaps_independent_worker(tmp_path):
    """Reproduce shell termination without starting Isaac or leaving an orphan."""
    scripts=tmp_path/'scripts';scripts.mkdir()
    (scripts/'candidate_dataset_runner.py').write_text('''
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path);a,_=p.parse_known_args()
a.run_dir.mkdir(parents=True,exist_ok=True)
(a.run_dir/'config.json').write_text(json.dumps({'target':'Tomato_05'}))
(a.run_dir/'candidates.json').write_text(json.dumps([{'candidate_id':'candidate_00000'}]))
''')
    (scripts/'dataset_sim.py').write_text('''
import argparse,os,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path);a,_=p.parse_known_args()
(a.run_dir/'worker.pid').write_text(str(os.getpid()))
(a.run_dir/'source_sha256.json').write_text('{}')
while True:time.sleep(.1)
''')
    run=tmp_path/'run';worker_pid=None
    wrapper='import sys;from pathlib import Path;import dataset_pool;dataset_pool.HERE=Path(sys.argv.pop(1));dataset_pool.main()'
    env=dict(os.environ,PYTHONPATH=str(Path(__file__).parent))
    with (tmp_path/'log').open('w') as log:
        process=subprocess.Popen([sys.executable,'-c',wrapper,str(scripts),'--workers','1',
            '--candidates','1','--run-dir',str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            marker=run/'worker_00/source_sha256.json';deadline=time.monotonic()+10
            while not marker.exists() and time.monotonic()<deadline:
                assert process.poll() is None
                time.sleep(.05)
            assert marker.exists()
            worker_pid=int((run/'worker_00/worker.pid').read_text())
            process.send_signal(signal.SIGTERM);process.wait(timeout=10)
            with pytest.raises(ProcessLookupError):os.kill(worker_pid,0)
            summary=json.loads((run/'summary.json').read_text())
            assert summary['workers']['0']=='interrupted'
            assert not summary['execution_complete']
        finally:
            if process.poll() is None:process.kill();process.wait()
            if worker_pid is not None:
                try:os.kill(worker_pid,signal.SIGKILL)
                except ProcessLookupError:pass
