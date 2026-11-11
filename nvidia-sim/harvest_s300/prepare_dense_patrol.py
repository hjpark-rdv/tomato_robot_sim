"""Rebuild dense trusses using the existing farm / patrol / robot adapters.

Fresh initial assets only; previous models and runs are never modified. This
entry point preserves the reviewed lowered plate, 70 mm gripper inset, wrist
camera and two elbow modes while regenerating every plant/GT position.
"""
import argparse
import json
from pathlib import Path

from pxr import Usd, UsdGeom

from latest_farm import build_latest
from build_scene import FARM
from prepare_newton_patrol import prepare, write
from upright_yaw_mount import prepare_assets


def build(output, base_scene, support_assets, ready_solutions, *, resume=False):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=resume)
    farm = output / 'farm/full_farm.usda'
    scene = output / 'source_scene.usda'
    if resume:
        report=json.loads(farm.with_suffix('.json').read_text())
        if report['truss_profile']!='dense_cherry_v1' or not scene.is_file():
            raise ValueError('Resume requires the completed dense source farm')
    else:
        (output / 'farm').mkdir()
        report = build_latest(farm, FARM)
        stage = Usd.Stage.CreateNew(str(scene))
        stage.GetRootLayer().subLayerPaths = [str(Path(base_scene).resolve(strict=True))]
        refs = stage.OverridePrim('/World/Greenhouse').GetReferences()
        refs.SetReferences([])
        refs.AddReference(str(farm))
        UsdGeom.SetStageUpAxis(stage, 'Z'); UsdGeom.SetStageMetersPerUnit(stage, 1.)
        stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        stage.GetRootLayer().Save()
    raw = output / 'patrol_source'
    prepare(raw, scene, end_station=.85,resume=resume)
    prepare_assets(raw, output / 'patrol', raw, ready_solutions, plate_drop_m=.30,
                   gripper_inset_m=.07, camera_mount='wrist-top', camera_down_tilt_deg=15.,
                   support_assets=Path(support_assets).resolve(strict=True))
    for mode in ('up', 'down'):
        path = output / 'patrol' / mode / 'patrol_assets.json'
        config = json.loads(path.read_text())
        config['truss_geometry'] = dict(profile=report['truss_profile'],
            report=str(farm.with_suffix('.json')), fresh_initial_assets=True,
            all_farm_trusses_rebuilt=True, count=report['trusses'])
        write(path, config)
    write(output / 'build.json', dict(profile=report['truss_profile'],
        farm_report=str(farm.with_suffix('.json')), assets=str(output / 'patrol'),
        robot='lowered flat plate, 70mm gripper inset, wrist-top portrait camera tilted 15deg',
        old_assets_modified=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--base-scene', type=Path, required=True)
    parser.add_argument('--support-assets', type=Path, required=True)
    parser.add_argument('--ready-solutions', type=Path, required=True)
    parser.add_argument('--resume',action='store_true',help='Verify and reuse completed preparation stages; preserve interrupted stages')
    a = parser.parse_args()
    build(a.output, a.base_scene, a.support_assets, a.ready_solutions,resume=a.resume)
