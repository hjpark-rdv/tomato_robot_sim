from build_v5 import *
import json
build()
report={'method':'OpenCascade swept sampling of explicit staged assembly/service moves, both sides. Flexible belt installation is described separately and not certified by a rigid path.', 'paths':[], 'tool_access':[]}
def overlaps(a,b):
 A=a.BoundingBox();B=b.BoundingBox()
 if any(min(getattr(A,k+'max'),getattr(B,k+'max'))-max(getattr(A,k+'min'),getattr(B,k+'min'))<1e-6 for k in 'xyz'):return 0
 return abs(a.intersect(b).Volume())
def check_path(label,moving,fixed,heights):
 rows=[]
 for h in heights:
  bad=[]
  for a in moving:
   sh=a['shape'].translate((0,0,h))
   for b in fixed:
    v=overlaps(sh,b['shape'])
    if v>.01:bad.append([a['name'],b['name'],round(v,4)])
  rows.append(dict(height_mm=h,collisions=bad))
 report['paths'].append(dict(name=label,moving=[p['name'] for p in moving],fixed=[p['name'] for p in fixed],positions=rows))
 print(label,'collisions',sum(len(x['collisions']) for x in rows),flush=True)
for side in ['L','R']:
 ps=[p for p in parts if p['name'].startswith(side+'_')]
 remove=['C11','C12','C13','C14','H15','H16','H17','C03_upper','C10_clamp_upper','C10_clamp_washer_upper','C10_clamp_nut_upper','C10_top_arm_joint','C10_axle_lock_80','C10_axle_washer_80','C10_axle_nut_80','C04_root_spacer_upper_outer']
 active=[p for p in ps if not any(t in p['name'] for t in remove)]
 tags=['C02_upper_deck','C05_root_bearing_cap_upper','C05_upper_','H10_625_root_upper']
 moving=[p for p in active if any(t in p['name'] for t in tags)]
 fixed=[p for p in active if p not in moving and 'C10_removable_top_arm' not in p['name']]
 check_path(side+' upper deck insertion',moving,fixed,[120,60,30,15,8,4,2,1,0])
 moving=[p for p in active if 'C10_removable_top_arm' in p['name']]
 fixed=[p for p in active if p not in moving]
 check_path(side+' keyed arm insertion',moving,fixed,[40,20,10,5,2,1,0])
 moving=[p for p in ps if 'C14_rear_drive_cover' in p['name']]
 fixed=[p for p in parts if p not in moving and side+'_C14_service_bolt' not in p['name']]
 check_path(side+' hood removal; structural bolts untouched',moving,fixed,[0,1,2,5,10,20,40,80,120])
 moving=[p for p in ps if any(t in p['name'] for t in ['H16_','H17_'])]
 fixed=[p for p in parts if p not in moving and not (p['name'].startswith(side+'_C14') and 'nut' not in p['name'])]
 check_path(side+' timing belt and both pulleys inserted together',moving,fixed,[80,40,20,10,5,2,1,0])
 # All service fasteners retained except the hood fasteners and hood itself.
 fixed=[p for p in parts if not ('C14' in p['name'] and 'nut' not in p['name'])]
 targets=[p for p in ps if 'C11_root_bolt' in p['name'] or 'C11_plate_bolt' in p['name']]
 for p in targets:
  bb=p['shape'].BoundingBox();x,y=bb.center.x,bb.center.y
  tool=cyl(1.4 if 'plate' in p['name'] else 1.7,90,x,y,bb.zmax+.1)
  bad=[[q['name'],round(v,4)] for q in fixed if q is not p and (v:=overlaps(tool,q['shape']))>.01]
  report['tool_access'].append(dict(target=p['name'],hood_removed=True,collisions=bad))
report['all_sampled_paths_clear']=all(not x['collisions'] for p in report['paths'] for x in p['positions'])
report['all_service_tools_clear']=all(not x['collisions'] for x in report['tool_access'])
(ROOT/'checks/assembly_paths.json').write_text(json.dumps(report,indent=2))
print('tools',report['all_service_tools_clear'],flush=True)
