"""Assemble the existing two-station Isaac assets through the native adapters.

No physics or alternate planner is implemented here. The original assemblies,
trial states and plans are read only; each output directory must be new.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

REPO=Path(__file__).resolve().parents[3]
DATA=Path('/root/docker_share/mujoko_debugging_data')
ASSET_PY=REPO/'nvidia-sim/.asset-venv/bin/python'
HERE=Path(__file__).resolve().parent
DEFAULT_ASSEMBLIES=[
    DATA/'20261011_194707_plant_interaction_comparison/D_target_only',
    DATA/'20261011_205000_target_only_row_harvest/assembly_step01_plant41']
DEFAULT_TRIALS=[
    DATA/'20261011_194707_plant_interaction_comparison/trials/D_target_only_connected',
    DATA/'20261011_205000_target_only_row_harvest/trials/step01_plant41_shallow_yaw_minus45_runtimefix']


def prepare(output,assemblies=None,trials=None):
    output=Path(output).resolve()
    assemblies=[Path(p).resolve() for p in (assemblies or DEFAULT_ASSEMBLIES)]
    trials=[Path(p).resolve() for p in (trials or DEFAULT_TRIALS)]
    if len(assemblies)!=len(trials) or not assemblies:
        raise ValueError('One existing trial is required per assembly')
    manifests=[]
    for assembly,trial in zip(assemblies,trials):
        for p in (assembly/'motion_manifest.json',assembly/'composed.usdc',
                  trial/'plan.json',trial/'states.npz',trial/'harvest_group_policy.json'):
            if not p.is_file():raise FileNotFoundError(p)
        manifests.append(json.loads((assembly/'motion_manifest.json').read_text()))
    output.mkdir(parents=True,exist_ok=False)
    models=[]
    for i,(assembly,trial) in enumerate(zip(assemblies,trials)):
        model=output/f'window_{i:02d}'
        command=[ASSET_PY,HERE/'export_scene.py',assembly,trial,model]
        if i==0:
            command+=['--robot','--background','--ground-collision','top-plane']
            for extra in manifests[1:]:
                command+=['--exclude-background-root',extra['plant']['source_plant']]
        else:command+=['--plant-only']
        print('Exporting source window',i,assembly,flush=True)
        subprocess.run(list(map(str,command)),check=True)
        models.append(model)
    from merge_windows import merge
    combined=output/'model'
    merge(models[0],[(f'P{i:02d}_',m) for i,m in enumerate(models[1:],1)],combined)
    record=dict(model=str(combined),sources=[dict(assembly=str(a),trial=str(t)) for a,t in zip(assemblies,trials)],
                ground_collision='top-plane',background_plant_collision=False,
                training_eligible=False,source_modified=False)
    (output/'preparation.json').write_text(json.dumps(record,indent=2)+'\n')
    return combined


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('output',type=Path)
    p.add_argument('--assembly',type=Path,action='append')
    p.add_argument('--trial',type=Path,action='append')
    a=p.parse_args()
    if bool(a.assembly)!=bool(a.trial):p.error('--assembly and --trial must be supplied together')
    print(prepare(a.output,a.assembly,a.trial))
