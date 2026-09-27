"""Read-only endpoint FK/diversity evidence with the existing exported planner.

Run in the Torch/FCL planning Python. No IK replanning and no physics execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import plan_candidates as planner


def verify(root):
    results = json.loads((root/'results.json').read_text())
    rows = []
    for report in results['reports']:
        case = report['case']
        run = root/'cases'/case['scene_id']/case['target']/'run'
        planner.initialize(run)
        kin = planner.MODEL[1]
        for path in sorted((run/'candidates').glob('*/plan.json')):
            plan = json.loads(path.read_text())
            if not plan['preflight'].get('passed'): continue
            trace_path = path.with_name('trace.json')
            trace = json.loads(trace_path.read_text())
            commands = np.array([r['command'] for r in trace])
            points, rotations, errors = [], [], []
            index = 0
            for waypoint in plan['waypoints']:
                index += waypoint['steps']
                p, r = kin.fk(commands[index])
                desired = Rotation.from_quat(waypoint['orientation_xyzw'])
                points.append(p.tolist()); rotations.append(r.as_quat().tolist())
                errors.append(dict(phase=waypoint['phase'], time_s=index/60,
                    position_m=float(np.linalg.norm(p-waypoint['position_xyz'])),
                    rotation_rad=float((r*desired.inv()).magnitude())))
            poses = np.concatenate([np.array(points), np.array(rotations)], axis=1)
            rows.append(dict(scene=case['scene_id'],target=case['target'],
                candidate=plan['candidate_id'],family=plan['parameters']['family'],
                parent_id=plan['parameters'].get('parent_id'),
                parameters=plan['parameters']['search_parameters'],
                trace_sha256=hashlib.sha256(trace_path.read_bytes()).hexdigest(),
                command_sha256=hashlib.sha256(commands.tobytes()).hexdigest(),
                fk_pose_sha256=hashlib.sha256(poses.tobytes()).hexdigest(),
                actual_endpoint_positions=points, actual_endpoint_rotations_xyzw=rotations,
                waypoint_errors=errors,
                max_position_error_m=max(x['position_m'] for x in errors),
                max_rotation_error_rad=max(x['rotation_rad'] for x in errors)))
    return dict(candidates=rows, checked=len(rows),
                unique_command_hashes=len({r['command_sha256'] for r in rows}),
                unique_fk_pose_hashes=len({r['fk_pose_sha256'] for r in rows}),
                endpoint_tolerance_pass=all(r['max_position_error_m']<=.002 and r['max_rotation_error_rad']<=.03 for r in rows),
                scope='Saved-command FK, not physical tracking. Raw endpoint arrays expose actual family diversity.',
                physics_executed=False, training_eligible=False)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Use new evidence output')
    a.output.write_text(json.dumps(verify(a.campaign),indent=2)+'\n')
