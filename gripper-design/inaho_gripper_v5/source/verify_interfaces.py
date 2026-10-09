#!/usr/bin/env python3
from build_v5 import *
import json,csv,itertools
build()
result={'scope':'Nominal rigid CAD and interface stacks; no physical assembly or strength/friction verification.', 'interfaces':[], 'tension_positions':[], 'timing_adjustment_positions':[], 'tool_access':[]}
stacks=[('rail M3x14',14,9.6,2.4,None),('stop M3x28',28,21.6,2.4,None),('tower M4x18',18,11.8,3.2,None),('tower cap M3x10',10,5,2.4,None),('MGN12H M3x8',8,5,None,3.5),('cassette lower M4x18',18,12.8,3.2,None),('independent upper deck + motor bracket M4x25',25,17.8,3.2,None),('motor plate/front M3x12',12,6.6,2.4,None),('independent hood M3x12',12,8.6,2.4,None),('guide lower M3x14',14,10.6,2.4,None),('guide upper M3x14',14,11.2,2.4,None),('root cap lower M3x8',8,5.2,2.4,None),('root cap upper ISO7380 M3x10',10,5.4,2.4,None),('split drum M3x20',20,16,2.4,None),('nose axle lock ISO4026 M3x6 headless; no washer',6,2.1,2.4,None),('nose lower clamp M4x14+t1',14,10.2,3.2,None),('nose upper clamp M4x14+t1',14,9.8,3.2,None),('keyed top arm M3x16',16,10.6,2.4,None),('tension jack ISO4762 M3x25 fully threaded',25,13.6,2.4,None),('outer cover lower M3x10',10,5.2,2.4,None),('outer cover upper M3x10',10,6.1,2.4,None),('NEMA11 M2.5x8+t0.5',8,4.5,None,4),('NEMA17 M3x10+t0.5',10,6.5,None,4.5),('opening motor yoke M4x20',20,13.8,3.2,None),('crank hub M3x20',20,17,2.4,None),('crank pivot M5 thread9.5',9.5,4,4,None),('slider pivot M5 thread9.5; neck clearance2.5',9.5,2.5,4,None),('tray posts M3x16',16,11.6,2.4,None),('fixture M4x22',22,16.8,3.2,None),('home bracket M3x16',16,10.6,2.4,None),('home sensor M2x12+t0.5',12,8.8,1.6,None)]
for name,L,grip,nt,depth in stacks:
    e=round(L-grip,4);r=dict(joint=name,length=L,stack_to_thread=grip,available_thread=e,nut_thickness=nt,blind_depth=depth)
    r.update(pass_check=e>=nt if nt else (e>=2.5 and e<depth))
    if nt:r['protrusion']=round(e-nt,3)
    else:r['bottom_margin']=round(depth-e,3)
    if name=='nose axle lock ISO4026 M3x6 headless; no washer':
        r['joint']='nose axle lock ISO4026 M3x6 headless; no washer'
        r['head_seating_limit']=False
        r['verified_adjustment_mm']=P['nose_lock_adjustment']
        r['adjustment_check']='See repair_audit.json: actual nut engagement and shaft-clearance take-up.'
    result['interfaces'].append(r)
def collide(ss,active):
    bb=[s.BoundingBox() for s in ss];bad=[]
    for i,a in enumerate(bb):
        for j in range(i+1,len(ss)):
            if not(active[i] or active[j]):continue
            b=bb[j]
            if min(a.xmax,b.xmax)-max(a.xmin,b.xmin)<1e-6 or min(a.ymax,b.ymax)-max(a.ymin,b.ymin)<1e-6 or min(a.zmax,b.zmax)-max(a.zmin,b.zmin)<1e-6:continue
            v=abs(ss[i].intersect(ss[j]).Volume())
            if v>.01:bad.append({'a':parts[i]['name'],'b':parts[j]['name'],'mm3':round(v,4)})
    return bad
nose=['C08_nose_roller','C09_nose_spacer','H13_625_nose','H14_nose_axle','C10_internal_nose','C10_removable_top','C10_axle_','C10_top_arm_joint','C10_jack_screw']
for t in [-3.,0.,3.]:
    ss=[];active=[]
    for p in parts:
        moving=any(x in p['name'] for x in nose);sh=p['shape'].translate((0,t,0)) if moving else p['shape']
        if 'C07_contact_belt' in p['name']:
            sh=contact_belt(144+t,-1 if p['group']=='L' else 1)
            if p['group']=='L':sh=sh.mirror('YZ').translate((-40,14,0))
            else:sh=sh.translate((40,0,0))
            moving=True
        ss.append(sh);active.append(moving)
    bad=collide(ss,active)
    length=belt_length(17,12,math.hypot(5,144+t));free=belt_length(17,12,math.hypot(5,143.2))
    result['tension_positions'].append(dict(travel_mm=t,collisions=bad,geometric_strain_percent=100*(length/free-1),belt_state="slack installation envelope; sag not simulated" if length<free else "tensioned geometric envelope; force not calculated"))
    print('nose',t,bad,flush=True)
for t in [-3.,0.,3.]:
    ss=[];active=[]
    for p in parts:
        moving=any(x in p['name'] for x in ['C12_motor_','H15_','H16_2GT_pulley_20T'])
        ss.append(p['shape'].translate((0,t,0)) if moving else p['shape']);active.append(moving)
    # The closed belt is removed during motor travel/end-position access checks.
    for i,p in enumerate(parts):
        if 'H17_timing_belt' in p['name']:ss[i]=ss[i].translate((0,0,1000))
    bad=collide(ss,active);result['timing_adjustment_positions'].append(dict(motor_travel_mm=t,collisions=bad,belt_removed=True));print('motor',t,bad,flush=True)
# A 2.5 mm AF L-key approaches the recessed M3 jack socket through the top window.
for side in [-1,1]:
    tool=axial_hex(2.5,20,(-5,73,39),(0,1,0)).fuse(cyl(1.45,90,-5,73,39))
    if side<0:tool=tool.mirror('YZ')
    tool=tool.translate((side*40,14 if side<0 else 0,0))
    bad=[]
    for p in parts:
        v=abs(tool.intersect(p['shape']).Volume())
        if v>.01:bad.append([p['name'],round(v,4)])
    result['tool_access'].append(dict(side=side,tool='2.5 AF L hex, vertical leg at Y73, horizontal reach20',collisions=bad));print('tool',side,bad,flush=True)
result['all_thread_stacks_pass']=all(p['pass_check'] for p in result['interfaces'])
result['all_tension_positions_clear']=all(not x['collisions'] for x in result['tension_positions'])
result['all_timing_adjustment_positions_clear']=all(not x['collisions'] for x in result['timing_adjustment_positions'])
result['all_tool_paths_clear']=all(not x['collisions'] for x in result['tool_access'])
result['running_clearances_mm']={'root_pulley_to_button_head':.95,'timing_belt_groove_margin_32T':.5,'timing_belt_groove_margin_20T':.75,'pulley20_top_to_hood':2.5,'root_axial_endplay':.2,'nose_axial_endplay':.15,'guide_to_belt_inner_face':.6,'belt_to_roller_flange_axial':1.,'nose_spacer_to_deck_slot_radial':.3,'MGN12H_screw_bottom_margin':.5,'NEMA11_screw_bottom_margin_min':.5,'NEMA17_screw_bottom_margin':1.}
(ROOT/'checks/interfaces_and_tension.json').write_text(json.dumps(result,indent=2))
with (ROOT/'docs/JOINT_STACKS.csv').open('w',encoding='utf-8-sig',newline='') as f:
    fs=list(dict.fromkeys(k for r in result['interfaces'] for k in r));w=csv.DictWriter(f,fieldnames=fs);w.writeheader();w.writerows(result['interfaces'])
