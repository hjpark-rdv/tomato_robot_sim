"""Replay RGB + joint observations through the guard without launching Isaac.

Reads only rendered RGB, recorded joints, camera calibration and a user ROI.
Does not read evaluation_only.json, simulator depth, contact IDs or forces.
"""
import argparse
import json
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np

from rgb_camera import JointCameraModel, intrinsics
from rgb_guard import RGBMotionGuard


def replay(directory, camera, roi, output):
    config = json.loads((directory/'config.json').read_text())
    observations = json.loads((directory/'observations.json').read_text())
    profile = dict(config['camera_profiles'][camera])
    profile['tool_from_camera'] = np.asarray(profile['tool_from_camera'])
    model = JointCameraModel(config['reference']['joint_names'], profile)
    guard = RGBMotionGuard(intrinsics(profile))
    capture = cv2.VideoCapture(str(directory/(camera+'_2x.mp4')))
    writer = imageio.get_writer(str(output.with_suffix('.mp4')), fps=60, codec='libx264', quality=8)
    seeded = False
    records = []
    try:
        for row in observations:
            ok, bgr = capture.read()
            if not ok:
                raise RuntimeError('Video and joint observation lengths differ')
            image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            pose = model.pose(row['joints'])
            if row['reference_step'] >= config['reference']['seed_frame_step']:
                if not seeded:
                    guard.initialize(image, roi, pose)
                    seeded = True
                else:
                    records.append(dict(step=row['step'],time_s=row['time_s'],**guard.observe(image,pose)))
                image = guard.annotate(image, draw_tracks=not guard.last['stop'])
            writer.append_data(image)
        if capture.read()[0]:
            raise RuntimeError('Unexpected extra video frames')
    finally:
        capture.release()
        writer.close()
    result = dict(camera=camera,roi=roi,
                  first_stop=next((r for r in records if r['stop']), None),
                  records=records, sensor_inputs_only=True,
                  caveat='Offline compressed RGB replay; not a new closed-loop physics rollout')
    output.with_suffix('.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--camera',choices=['left','right'],default='left')
    parser.add_argument('--roi',type=int,nargs=4,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    replay(args.directory,args.camera,args.roi,args.output)
