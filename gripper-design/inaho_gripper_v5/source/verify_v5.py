#!/usr/bin/env python3
"""Reproducible CAD intersection check. Hardware threads are represented by their
nominal major diameter with clearance bores. Bearing/rail internal mechanics,+flexible-body deformation and motor/coupler internals are outside this check."""
import json,sys,time,itertools,math
import numpy as np
from build_v5 import *
build();start=time.time()
out={'method':'OpenCascade exact solid intersections after AABB broad phase; threshold 0.01 mm3','positions':[],'invalid':[],'multiple_solid_prints':[]}
for p in parts:
    if not p['shape'].isValid():out['invalid'].append(p['name'])
    if p['kind']=='print' and len(p['shape'].Solids())!=1:out['multiple_solid_prints'].append([p['name'],len(p['shape'].Solids())])
gaps=[44.] if '--quick' in sys.argv else sorted(set(np.linspace(28,96,35).tolist()+[94.6,95.0,95.6,95.8]))
cache={};checks=0
for gap in gaps:
    shapes=[shape_at(p,gap) for p in parts]
    bounds=[s.BoundingBox() for s in shapes];coll=[]
    for i,a in enumerate(bounds):
        for j in range(i+1,len(parts)):
            b=bounds[j]
            if min(a.xmax,b.xmax)-max(a.xmin,b.xmin)<1e-6 or min(a.ymax,b.ymax)-max(a.ymin,b.ymin)<1e-6 or min(a.zmax,b.zmax)-max(a.zmin,b.zmin)<1e-6:continue
            pa,pb=parts[i],parts[j]
            same=pa['group']==pb['group'] and not any('H19_' in p['name'] for p in [pa,pb])
            key=(i,j)
            if same and key in cache:v=cache[key]
            else:
                try:v=abs(shapes[i].intersect(shapes[j]).Volume());checks+=1
                except Exception as e:v=-1;print('BOOL ERROR',pa['name'],pb['name'],e,flush=True)
                if same:cache[key]=v
            if v>.01 or v<0:coll.append({'a':pa['name'],'b':pb['name'],'mm3':round(v,5)})
    out['positions'].append({'gap_mm':gap,'collisions':coll})
    print('gap',gap,'collisions',len(coll),'elapsed',round(time.time()-start,1),flush=True)
    if coll and '--quick' in sys.argv:
        for x in coll:print(x,flush=True)
out['exact_boolean_tests']=checks;out['elapsed_seconds']=time.time()-start
out['all_sampled_positions_clear']=all(not p['collisions'] for p in out['positions'])
(ROOT/'checks'/('interference_quick.json' if '--quick' in sys.argv else 'interference_full.json')).write_text(json.dumps(out,indent=2))
print('multiple',out['multiple_solid_prints'],flush=True)
