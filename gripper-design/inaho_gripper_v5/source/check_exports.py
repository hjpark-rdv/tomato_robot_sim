#!/usr/bin/env python3
from pathlib import Path
import json, hashlib, cadquery as cq, trimesh, numpy as np
ROOT=Path(__file__).resolve().parents[1]
result={'stl':[],'step':[],'limits':{'print_volume_mm':[220,220,220],'physical_test':False}}
for f in sorted((ROOT/'stl').glob('*.stl')):
    m=trimesh.load(f,force='mesh',process=True)
    rec={'file':f.name,'watertight':bool(m.is_watertight),'winding_consistent':bool(m.is_winding_consistent),'positive_volume':bool(m.volume>0),'bounds_mm':np.round(m.extents,4).tolist(),'fits_220mm':bool(np.all(m.extents<=220.001)),'connected_bodies':int(m.body_count)}
    result['stl'].append(rec)
for f in sorted((ROOT/'models').glob('*.step'))+sorted((ROOT/'step_parts').glob('*.step')):
    s=cq.importers.importStep(str(f)).val()
    bb=s.BoundingBox();rec={'file':f.name,'bytes':f.stat().st_size,'valid':s.isValid(),'solids':len(s.Solids()),'bbox_mm':[bb.xlen,bb.ylen,bb.zlen]};result['step'].append(rec)
    if f.parent.name=='models': print('STEP',rec,flush=True)
g=trimesh.load(ROOT/'models/inaho_gripper_v5_assembly.glb',force='scene')
result['glb']={'geometry_count':len(g.geometry),'bounds_m':g.bounds.tolist()}
result['all_print_meshes_pass']=all(r['watertight'] and r['winding_consistent'] and r['positive_volume'] and r['fits_220mm'] and r['connected_bodies']==1 for r in result['stl'])
result['all_step_reopened_valid']=all(r['valid'] for r in result['step'])
(ROOT/'checks/export_checks.json').write_text(json.dumps(result,indent=2))
print('STL',len(result['stl']),'PASS',result['all_print_meshes_pass'],flush=True)
print('BAD', [r for r in result['stl'] if not(r['watertight'] and r['connected_bodies']==1)],flush=True)
