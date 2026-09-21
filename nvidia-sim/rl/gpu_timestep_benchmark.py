"""Small timestep screen using identical saved 60 Hz joint commands, no DR."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

from gpu_probe import compare
from gpu_physics_errors import invalid_physics_message

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--envs', type=int, nargs='+', default=[1, 4])
    parser.add_argument('--hz', type=int, nargs='+', choices=[60, 120, 240, 480, 720, 960], default=[960, 720, 480, 240, 120, 60])
    parser.add_argument('--timeout', type=float, default=900)
    args = parser.parse_args()
    root = args.run_dir or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_gpu_timestep')
    root.mkdir(parents=True, exist_ok=False)
    summary = dict(fixture=str(args.fixture.resolve()), control_hz=60, solver='PGS 64/4',
                   production_default_changed=False, experiments=[], comparisons={})
    print('[TIMESTEP OUTPUT]', root, flush=True)
    for count in args.envs:
        for hz in args.hz:
            folder = root/f'env{count}_hz{hz}'
            folder.mkdir()
            command = [sys.executable, '-u', str(HERE/'gpu_probe_worker.py'), '--headless',
                       '--fixture', str(args.fixture.resolve()), '--output', str(folder.resolve()),
                       '--mode', 'gpu', '--solver', 'pgs', '--batched-io', '--native-replication',
                       '--gpu-partitions', '1', '--num-envs', str(count), '--physics-hz', str(hz)]
            (folder/'command.json').write_text(json.dumps(command, indent=2))
            began = time.monotonic()
            with (folder/'run.log').open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    status = process.wait(timeout=args.timeout)
                except BaseException:
                    process.terminate()
                    try: process.wait(timeout=20)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
                    raise
            report_path = folder/'report.json'
            report = json.loads(report_path.read_text()) if report_path.exists() else {}
            native_errors=[line for line in (folder/'run.log').read_text(errors='replace').splitlines()
                           if invalid_physics_message(line,line)]
            summary['experiments'].append(dict(envs=count, hz=hz, exit_code=status,
                total_wall_s=time.monotonic()-began, complete=report.get('complete', False),
                native_physics_errors=native_errors,
                motion=report.get('motion'), api_checks_passed=report.get('api_checks_passed')))
            reference = root/f'env{count}_hz960'
            if hz != 960 and (reference/'report.json').exists() and report.get('complete'):
                summary['comparisons'][folder.name] = compare(reference, folder)
                if native_errors:
                    summary['comparisons'][folder.name].update(comparable=False,speedup=None,reason='Native physics error; invalid experiment')
            (root/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            print('[TIMESTEP RESULT]', count, hz, report.get('motion', {}).get('wall_s'),
                  [v['classification']['result'] for v in report.get('motion', {}).get('outcomes', [])], flush=True)


if __name__ == '__main__': main()
