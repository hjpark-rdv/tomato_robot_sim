"""Serial GPU mechanics + 1/4-clone motion checks for explicit physics presets.

Uses functional acceptance, not equivalence to the old 960Hz model. The ring
test starts preinserted and does not certify robot insertion or sim-to-real.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time
from gpu_physics_presets import PRESETS
from gpu_physics_errors import invalid_physics_message

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixture',type=Path,required=True)
    p.add_argument('--presets',nargs='+',choices=list(PRESETS),default=['practical120','practical60'])
    p.add_argument('--run-dir',type=Path)
    p.add_argument('--timeout-seconds',type=float,default=600)
    a=p.parse_args()
    if a.timeout_seconds<=0:p.error('timeout must be positive')
    fixture=a.fixture.resolve()
    if not all((fixture/n).exists() for n in ('candidate.json','planned_commands.npy')):p.error('Incomplete fixture')
    root=a.run_dir or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_low_hz_behavior')
    root.mkdir(parents=True,exist_ok=False)
    summary=dict(acceptance='functional mechanics screen, not trajectory equivalence or real-plant calibration',runs={},
                 scope='single original plant ring fixture; one saved robot trajectory repeated in 1 and 4 clones')
    for preset in a.presets:
        cfg=PRESETS[preset]
        for mode,count in [('behavior',1),('motion',1),('motion',4)]:
            name=f'{preset}_{mode}_{count}';out=root/name;out.mkdir()
            command=[sys.executable,'-u',str(HERE/'gpu_probe_worker.py'),'--headless','--mode','gpu','--solver','pgs',
                '--batched-io','--native-replication','--gpu-partitions','1','--num-envs',str(count),
                '--physics-hz',str(cfg['physics_hz']),'--joint-armature',str(cfg['elastic_joint_armature']),
                '--position-iterations',str(cfg['position_iterations']),'--fixture',str(fixture),'--output',str(out)]
            if mode=='behavior':command+=['--behavior-probe']
            (out/'command.json').write_text(json.dumps(command,indent=2)+'\n')
            print('[LOW HZ START]',name,flush=True)
            began=time.monotonic();timed_out=False
            with (out/'run.log').open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                try:
                    while process.poll() is None:
                        try:process.wait(timeout=min(15.,a.timeout_seconds))
                        except subprocess.TimeoutExpired:
                            elapsed=time.monotonic()-began
                            print('[LOW HZ PROGRESS]',name,round(elapsed,1),'wall s',flush=True)
                            if elapsed>a.timeout_seconds:timed_out=True;process.kill();process.wait()
                finally:
                    if process.poll() is None:process.kill();process.wait()
            report=json.loads((out/'report.json').read_text()) if (out/'report.json').exists() else {}
            native_error=any(invalid_physics_message(line,line) for line in (out/'run.log').read_text().splitlines())
            finished=process.returncode==0 and not timed_out and report.get('complete',False) and not native_error
            outcomes=report.get('motion',{}).get('outcomes',[])
            motion_ok=report.get('api_checks_passed',False) and len(outcomes)==count and all(
                row.get('first_contact_object') is not None for row in outcomes)
            passed=finished and (report.get('behavior',{}).get('passed',False) if mode=='behavior' else motion_ok)
            summary['runs'][name]=dict(complete=finished,passed=passed,timeout=timed_out,native_error=native_error,
                exit_code=process.returncode,wall_s=time.monotonic()-began,physics=cfg,
                motion_wall_s=report.get('motion',{}).get('wall_s'),motion_sim_s=report.get('motion',{}).get('batch_sim_s'))
            (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            print('[LOW HZ RESULT]',name,summary['runs'][name],flush=True)
    summary['screened_presets']=[name for name in a.presets if all(summary['runs'][f'{name}_{mode}_{count}']['passed']
        for mode,count in [('behavior',1),('motion',1),('motion',4)])]
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('[LOW HZ OUTPUT]',root,flush=True)


if __name__=='__main__':main()
