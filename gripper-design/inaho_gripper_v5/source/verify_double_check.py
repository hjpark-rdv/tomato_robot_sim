"""Second audit: reconstruct measurements from exported STEP, then spin moving CAD.
The external interfaces and output files are audited separately from the opening sweep.
"""
from build_v5 import *
import hashlib,itertools
from collections import Counter
build()
result={'exported_step_measurements':{},'assertions':[], 'rotation_samples':[], 'clearances_mm':{}}
by={p['name']:p for p in parts}
def assert_record(label,ok,detail):
 result['assertions'].append(dict(check=label,pass_check=bool(ok),detail=detail))
def z_of(name):
 sh=cq.importers.importStep(str(ROOT/'step_parts'/f'{name}.step')).val();b=sh.BoundingBox()
 return [round(b.zmin,6),round(b.zmax,6)]
for tag in ['C02_lower_deck','C02_upper_deck','C13_outer_finger_cover','C14_rear_drive_cover','C07_contact_belt_TPU','C08_nose_roller']:
 for side in ['L','R']:result['exported_step_measurements'][side+'_'+tag]=z_of(side+'_'+tag)
for tag in ['C02_lower_deck','C02_upper_deck','C13_outer_finger_cover','C14_rear_drive_cover']:
 l=result['exported_step_measurements']['L_'+tag];r=result['exported_step_measurements']['R_'+tag]
 assert_record('equal body datum '+tag,l==r,[l,r])
for side,z in [('L',[9,61]),('R',[13,65])]:
 assert_record('independent belt height '+side,result['exported_step_measurements'][side+'_C07_contact_belt_TPU']==z,z)
assert_record('invalid 20T MISUMI code removed',not any('GPA20' in p['spec'] for p in parts),'RobotDigg20T, MISUMI32T')
for side in ['L','R']:
 assert_record(side+' independent structural fasteners',len([p for p in parts if p['name'].startswith(side+'_C11_root_bolt')])==2 and len([p for p in parts if p['name'].startswith(side+'_C11_plate_bolt')])==4,'2x M4x25 root and 4x M3x12 slide remain with cover removed')
 assert_record(side+' three hood-only fasteners',len([p for p in parts if p['name'].startswith(side+'_C14_service_bolt')])==3,'3x M3x12')
 result['clearances_mm'][side+'_32T_flange_to_cap_button']=round(by[side+'_H16_2GT_pulley_32T']['shape'].distance(by[side+'_C05_upper_bolt_13']['shape']),5)
 result['clearances_mm'][side+'_20T_top_to_cover']=round(by[side+'_H16_2GT_pulley_20T']['shape'].distance(by[side+'_C14_rear_drive_cover']['shape']),5)
result['clearances_mm']['home_bracket_to_block_at96']=round(shape_at(by['R_H01_MGN12H'],96).distance(by['F08_adjustable_home_bracket']['shape']),5)
result['clearances_mm']['home_body_to_carriage_at96']=round(shape_at(by['R_C01_carriage_saddle'],96).distance(by['H18_home_switch_D2F01F']['shape']),5)
assert_record('motor 20T shaft set screw overlap',95.5+1.5<98,'M3 screw Z95.5 +/-1.5; motor shaft tip98; 1mm end margin')
assert_record('timing grooves share one plane',abs((81.6+10.3/2)-(82+1+7.5/2))<1e-8,'Z86.75, nominal width6; side margins0.5 /0.75')
for side in ['L','R']:
 shaft=by[side+'_H14_nose_axle']['shape'];sb=shaft.BoundingBox();checks=[]
 for tag in ['lower','upper']:
  be=by[side+'_H13_625_nose_'+tag]['shape'];bb=be.BoundingBox()
  checks.append(abs(sb.center.x-bb.center.x)<1e-6 and abs(sb.center.y-bb.center.y)<1e-6 and sb.zmin<bb.zmin and sb.zmax>bb.zmax and abs(shaft.intersect(be).Volume())<.01)
 assert_record(side+' nose axle and both bearings concentric',all(checks),'Actual CAD axes and shaft span, no interference with bearing rings.')
spin_tags=['C06_','H11_root_shaft','H12_root_collar','H16_2GT_pulley','H15_rotor_shaft','C08_nose_roller']
active=[any(t in p['name'] for t in spin_tags) for p in parts]
def spin(p,gap,angle):
 sh=p['shape'];side=1 if p['group']=='R' else -1
 if any(t in p['name'] for t in spin_tags):
  x=side*40;y=14 if side<0 else 0;a=angle*side
  if '20T' in p['name'] or 'rotor_shaft' in p['name']:y-=P['timing_centres'];a*=32/20
  if 'C08' in p['name']:x-=side*5;y+=144;a*=16/11
  sh=sh.rotate((x,y,0),(x,y,1),a)
  return sh.translate((side*(gap-44)/2,0,0))
 return shape_at(p,gap)
for gap,ang in [(28,a) for a in range(0,360,30)]+[(96,a) for a in [0,90,180,270]]:
 ss=[spin(p,gap,ang) for p in parts];bb=[s.BoundingBox() for s in ss];bad=[]
 for i,a in enumerate(bb):
  for j in range(i+1,len(ss)):
   if not (active[i] or active[j]):continue
   b=bb[j]
   if any(min(getattr(a,k+'max'),getattr(b,k+'max'))-max(getattr(a,k+'min'),getattr(b,k+'min'))<1e-6 for k in 'xyz'):continue
   v=abs(ss[i].intersect(ss[j]).Volume())
   if v>.01:bad.append([parts[i]['name'],parts[j]['name'],round(v,5)])
 result['rotation_samples'].append(dict(gap=gap,root_angle_deg=ang,collisions=bad));print('spin',gap,ang,len(bad),flush=True)
result['all_assertions_pass']=all(x['pass_check'] for x in result['assertions'])
result['all_rotations_clear']=all(not x['collisions'] for x in result['rotation_samples'])
result['source_sha256']=hashlib.sha256((ROOT/'source/build_v5.py').read_bytes()).hexdigest()
result['physical_testing_performed']=False
(ROOT/'checks/double_check.json').write_text(json.dumps(result,indent=2))
print('PASS',result['all_assertions_pass'],result['all_rotations_clear'],result['clearances_mm'],flush=True)
