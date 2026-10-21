#!/usr/bin/env python3
"""Check changed receiver CAD against the unchanged V5.1 mechanism."""
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import trimesh
import cadquery as cq
from build_v6 import ROOT, P, parts, build, shape_at, box

start = time.time()
build()
receiver = next(p for p in parts if p['name'] == 'L_T01_deep_U_receiver')
assert receiver['shape'].isValid() and len(receiver['shape'].Solids()) == 1
baseline = ROOT.parent / 'inaho_gripper_v5'
old = {p['name']: p for p in json.loads((ROOT / 'docs/V5_BASELINE_PARTS.json').read_text())}
current = json.loads((ROOT / 'parts_manifest.json').read_text())
unchanged = []
for p in current:
    if 'T01' in p['name']:
        continue
    ref = old[p['name']]
    assert abs(p['volume_mm3'] - ref['volume_mm3']) < .01, p['name']
    assert np.allclose(p['bbox_mm'], ref['bbox_mm'], atol=.001), p['name']
    assert np.allclose(p['centre'], ref['centre'], atol=.001), p['name']
    assert p['group'] == ref['group'] and p['spec'] == ref['spec'], p['name']
    unchanged.append(p['name'])
assert len(unchanged) == 351

collisions, positions, cache = [], [], {}
exact = 0
for gap in np.linspace(28, 96, 35):
    tray = shape_at(receiver, gap)
    a = tray.BoundingBox()
    for p in parts:
        if p is receiver:
            continue
        sh = shape_at(p, gap)
        b = sh.BoundingBox()
        if any(min(getattr(a, axis+'max'), getattr(b, axis+'max')) - max(getattr(a, axis+'min'), getattr(b, axis+'min')) <= 1e-6 for axis in 'xyz'):
            continue
        if p['group'] == 'L' and p['name'] in cache:
            volume = cache[p['name']]
        else:
            volume = abs(tray.intersect(sh).Volume())
            exact += 1
            if p['group'] == 'L':
                cache[p['name']] = volume
        if volume > .01:
            collisions.append({'gap_mm': gap, 'part': p['name'], 'intersection_mm3': volume})
    positions.append(float(gap))
print('Receiver motion intersections:', len(collisions), 'exact tests:', exact, flush=True)

stl = trimesh.load(ROOT / 'stl/L_T01_deep_U_receiver.stl', force='mesh', process=True)
stl_ok = bool(stl.is_watertight and stl.is_winding_consistent and stl.volume > 0 and stl.body_count == 1 and np.all(stl.extents < 220))
step = cq.importers.importStep(str(ROOT / 'step_parts/L_T01_deep_U_receiver.step')).val()
assembly = cq.importers.importStep(str(ROOT / 'models/inaho_gripper_v6_assembly.step')).val()
glb = trimesh.load(ROOT / 'models/inaho_gripper_v6_assembly.glb', force='scene')
assert len(glb.geometry) == len(parts) == 356
assert 'L_T01_deep_U_receiver' in glb.geometry and 'L_T01_sloped_receiver' not in glb.geometry
assert abs(step.Volume() - receiver['shape'].Volume()) < .01
# Check the exported CAD, not just parameter labels. This clear fruit-entry
# corridor crossed the old semicircular end cap. Both new cheeks extend
# beyond Y174 and outboard of X61. Floor sections independently measure slope.
entry_blockage = abs(step.intersect(box(60,35,60,y=187.5,z=-108)).Volume())
cheek_volumes = [abs(step.intersect(box(16,16,25,x=sign*66,y=190,z=-65)).Volume()) for sign in [-1,1]]
floor_samples=[]
for y in [35.,155.]:
    section=step.intersect(box(.2,.2,120,y=y,z=-200))
    assert section.Solids(), 'Missing floor at '+str(y)
    floor_samples.append((y,section.BoundingBox().zmin))
measured_slope=float(np.degrees(np.arctan2(floor_samples[1][1]-floor_samples[0][1],120)))
open_geometry_ok=entry_blockage<.01 and min(cheek_volumes)>100 and abs(measured_slope-12)<.01
hashes = json.loads((ROOT / 'docs/V5_BASELINE_SHA256.json').read_text())
baseline_intact = None if not baseline.exists() else all(hashlib.sha256((baseline / name).read_bytes()).hexdigest() == digest for name, digest in hashes.items())
result = dict(revision='V6.0', passed=not collisions and stl_ok and open_geometry_ok and step.isValid() and assembly.isValid() and baseline_intact is not False,
    unchanged_mechanism_parts=len(unchanged), original_V5_files_unchanged=baseline_intact,
    changed_part='L_T01_deep_U_receiver', gap_samples_mm=positions, collision_threshold_mm3=.01,
    exact_intersections=exact, collisions=collisions, receiver_valid_solid=step.isValid(),
    receiver_stl=dict(valid=stl_ok, print_bounds_mm=stl.extents.tolist(), connected_bodies=int(stl.body_count)),
    assembly_step=dict(valid=assembly.isValid(), solids=len(assembly.Solids())), glb_parts=len(glb.geometry),
    open_receiver=dict(valid=open_geometry_ok,entry_corridor_blockage_mm3=entry_blockage,
        projecting_cheeks_volume_mm3=cheek_volumes,measured_floor_slope_deg=measured_slope,
        floor_samples_y_z_mm=floor_samples,slope_direction=P['receiver']['slope_direction']),
    elapsed_seconds=round(time.time()-start, 2), physical_test=False,
    scope='Changed receiver at 35 nominal opening positions against all other components. Rigid geometry only; no fruit flow, print strength or tolerance analysis.')
(ROOT / 'checks/v6_receiver.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
print(json.dumps(result, ensure_ascii=False), flush=True)
assert result['passed'], result
