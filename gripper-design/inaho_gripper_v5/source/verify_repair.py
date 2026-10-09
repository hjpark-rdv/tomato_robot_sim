#!/usr/bin/env python3
"""Measure all modeled threaded fasteners and washers from built CAD.

This verifies nominal geometry and the headless nose-lock adjustment envelope.
It does not predict clamp force, fatigue, printed tolerances, or real retention.
"""
import json, math, re
from collections import Counter
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from build_v5 import *

build()
by_name={p['name']:p for p in parts}
def cylinders(shape):
    result=[]
    for face in shape.Faces():
        surf=BRepAdaptor_Surface(face.wrapped)
        if surf.GetType()!=GeomAbs_Cylinder:continue
        c=surf.Cylinder();origin=np.array(c.Location().Coord());axis=np.array(c.Axis().Direction().Coord())
        ts=[np.dot(np.array(v.toTuple())-origin,axis) for v in face.Vertices()]
        if not ts:continue
        result.append(dict(radius=c.Radius(),origin=origin,axis=axis,
                           start=origin+axis*min(ts),end=origin+axis*max(ts),length=float(max(ts)-min(ts))))
    return result
def interval(shape,axis):
    ts=[np.dot(np.array(v.toTuple()),axis) for v in shape.Vertices()]
    return float(min(ts)),float(max(ts))
def cylinder_interval(c,axis):
    ts=[np.dot(c[k],axis) for k in ['start','end']];return float(min(ts)),float(max(ts))
def coaxial(a,b):
    return abs(abs(np.dot(a['axis'],b['axis']))-1)<1e-8 and np.linalg.norm(np.cross(b['origin']-a['origin'],a['axis']))<1e-5
def overlap(a,b):return max(0.,min(a[1],b[1])-max(a[0],b[0]))
def fastener_spec(p):
    spec=p['spec']
    if spec.startswith('Ganter ISO7379'):m=re.search(r'M([\d.]+)x([\d.]+)$',spec)
    else:m=re.match(r'ISO (?:4762|7380-1|4026) M([\d.]+)x([\d.]+)',spec)
    return (float(m[1]),float(m[2])) if m else None
def thread_cylinder(p,diameter):
    candidates=[c for c in cylinders(p['shape']) if abs(c['radius']-diameter/2)<1e-6]
    if not candidates:raise AssertionError('No modeled thread envelope: '+p['name'])
    return max(candidates,key=lambda c:c['length'])
def purchase_thread_interval(p,diameter,L,ti):
    # Compare nut/blind-hole placement with the purchased thread requirement,
    # rather than treating the whole major-diameter CAD cylinder as threaded.
    if p['spec'].startswith('ISO 4762') or p['spec'].startswith('ISO 7380-1'):
        pitch={2.:.4,2.5:.45,3.:.5,4.:.7}[diameter]
        minimum_b={2.:16.,2.5:17.,3.:18.,4.:20.}[diameter]
        runout=3*pitch if 'fully threaded' in p['spec'] else max(3*pitch,L-minimum_b)
        return (ti[0]+runout,ti[1]),dict(minimum_purchase_thread_span_mm=round(L-runout,6),maximum_underhead_unthreaded_mm=round(runout,6),
            basis='Fully threaded requirement including3P runout' if 'fully threaded' in p['spec'] else 'ISO4762 supplier b table; conservative3P runout for short screws')
    if p['spec'].startswith('Ganter ISO7379'):
        # M5 nominal9.5 thread region, minimum9.25 overall region and up to2
        # at its shoulder end are from the supplier standard table. CAD already
        # removes the2mm neck; reduce the tip end by the .25 length tolerance.
        return (ti[0]+.25,ti[1]),dict(minimum_purchase_thread_span_mm=7.25,shoulder_thread_region_min_mm=9.25,
            maximum_thread_relief_mm=P['pivot_thread_relief'],basis='ISO7379 M5 bmin9.25 and g2max2; specified Ganter Ø6 variant and actual relief must be checked')
    return ti,dict(minimum_purchase_thread_span_mm=round(ti[1]-ti[0],6),basis='ISO4026 fully threaded major envelope with modeled end chamfers')
def mates(p):
    n=p['name'];side=n[:1]
    if 'C01_rail_bolt_' in n:return [('blind',side+'_H01_MGN12H')]
    if n.startswith('F07_motor_bolt'):return [('blind','H06_opening_motor')]
    if 'C12_motor_bolt' in n:return [('blind',side+'_H15_NEMA11_belt_motor')]
    if 'C11_root_bolt_' in n:return [('nut',n.replace('C11_root_bolt','C02_upper_root_nut'))]
    if 'C10_axle_lock_' in n:return [('nut',n.replace('axle_lock','axle_nut'))]
    if 'C10_clamp_' in n:return [('nut',n.replace('C10_clamp_','C10_clamp_nut_'))]
    if 'C10_jack_screw' in n:return [('nut',n.replace('screw','captive')),('nut',n.replace('screw','locknut'))]
    if n.startswith('K03_shoulder'):return [('nut',n.replace('shoulder','nut'))]
    return [('nut',n.replace('bolt','nut'))]

report={'scope':'Measured nominal CAD: all independently modeled threaded fasteners and washers. Purchased integral screws, unmodeled threads, material behavior, force, wear, and physical assembly are not certified.',
        'revision':P['revision'],'inventory':{'instances':len(parts),'kinds':dict(Counter(p['kind'] for p in parts)),
        'valid_shapes':all(p['shape'].isValid() for p in parts),'single_solid_prints':all(len(p['shape'].Solids())==1 for p in parts if p['kind']=='print')},
        'threaded_joints':[],'washers':[],'nose_lock_adjustment':[],'pivot_stacks':[],
        'physical_testing_performed':False,'original_purchase_spec_risks':[]}
fasteners=[p for p in parts if p['kind']=='buy' and fastener_spec(p)]
for p in fasteners:
    diameter,L=fastener_spec(p);thread=thread_cylinder(p,diameter);axis=thread['axis'];ti=cylinder_interval(thread,axis)
    purchase_ti,thread_rule=purchase_thread_interval(p,diameter,L,ti)
    for mode,name in mates(p):
        q=by_name[name]
        row={'screw':p['name'],'receiver':name,'receiver_type':mode,'diameter_mm':diameter,'nominal_length_mm':L,'modeled_major_diameter_span_mm':round(thread['length'],6),'purchase_thread_rule':thread_rule}
        if mode=='nut':
            bore=[c for c in cylinders(q['shape']) if abs(c['radius']-(diameter+.04)/2)<1e-6 and coaxial(thread,c)]
            ni=interval(q['shape'],axis);eng=overlap(ti,ni);purchase_eng=overlap(purchase_ti,ni);nt=ni[1]-ni[0]
            row.update(nut_thickness_mm=round(nt,6),full_diameter_engagement_mm=round(eng,6),coaxial=bool(bore),
                       purchase_thread_engagement_mm=round(purchase_eng,6),
                       screw_to_nut_clearance_mm=round(p['shape'].distance(q['shape']),6),
                       pass_check=bool(bore) and eng>=nt-1e-5 and purchase_eng>=nt-1e-5)
            if 'C10_jack_locknut' in name:
                original_rule=dict(p);original_rule['spec']='ISO 4762 M3x25'
                original_ti,_=purchase_thread_interval(original_rule,diameter,L,ti)
                report['original_purchase_spec_risks'].append({'screw':p['name'],'receiver':name,'original_standard_minimum_thread_length_mm':18.,'original_underhead_unthreaded_mm':7.,
                    'nut_distance_from_underhead_mm':[round(ni[0]-ti[0],6),round(ni[1]-ti[0],6)],'original_thread_engagement_mm':round(overlap(original_ti,ni),6),
                    'required_nut_thickness_mm':round(nt,6),'repair':'Explicit fully threaded M3x25 variant; envelope unchanged.'})
        else:
            bore=[c for c in cylinders(q['shape']) if diameter/2<c['radius']<diameter/2+.2 and coaxial(thread,c)]
            c=max(bore,key=lambda c:c['length']) if bore else None
            bi=cylinder_interval(c,axis) if c else (0,0);eng=overlap(ti,bi);purchase_eng=overlap(purchase_ti,bi)
            # Primitive cylinder direction is the insertion direction.
            tip=ti[1];bottom=bi[1];margin=bottom-tip
            row.update(blind_bore_depth_mm=round(c['length'],6) if c else None,
                       engagement_mm=round(eng,6),purchase_thread_engagement_mm=round(purchase_eng,6),bottom_margin_mm=round(margin,6),coaxial=bool(bore),
                       pass_check=bool(bore) and purchase_eng>=diameter-1e-5 and margin>=.49)
        report['threaded_joints'].append(row)

def washer_mate(n):
    if n.startswith('K03_'):return re.sub(r'(?:rear|front)_washer','shoulder',n)
    if 'C10_clamp_washer_' in n:return n.replace('clamp_washer','clamp')
    return n.replace('washer','bolt')
for p in [p for p in parts if p['kind']=='buy' and 'washer' in p['name']]:
    q=by_name[washer_mate(p['name'])];cs=cylinders(p['shape']);inner=min(cs,key=lambda c:c['radius']);axis=inner['axis'];wi=interval(p['shape'],axis)
    shaft=[c for c in cylinders(q['shape']) if coaxial(inner,c) and overlap(wi,cylinder_interval(c,axis))>1e-6 and c['radius']<inner['radius']]
    sc=max(shaft,key=lambda c:c['radius']) if shaft else None
    collision=abs(p['shape'].intersect(q['shape']).Volume())
    report['washers'].append({'washer':p['name'],'screw':q['name'],'bore_mm':round(inner['radius']*2,6),
        'mating_shaft_mm':round(sc['radius']*2,6) if sc else None,'axial_overlap_mm':round(overlap(wi,cylinder_interval(sc,axis)),6) if sc else None,
        'intersection_mm3':round(collision,9),'pass_check':bool(sc) and collision<1e-6})

for side in ['L','R']:
    axle=by_name[side+'_H14_nose_axle'];shaft=thread_cylinder(axle,5.)
    for z,host in [(-5,'C10_internal_nose_carrier'),(80,'C10_removable_top_arm')]:
        screw=by_name[f'{side}_C10_axle_lock_{z}'];nut=by_name[f'{side}_C10_axle_nut_{z}'];carrier=by_name[side+'_'+host]
        c=thread_cylinder(screw,3.);axis=c['axis']
        bore=next(v for v in cylinders(carrier['shape']) if abs(v['radius']-2.575)<1e-6 and coaxial(shaft,v))
        radial_clearance=bore['radius']-shaft['radius'];ni=interval(nut['shape'],axis);nt=ni[1]-ni[0]
        samples=[]
        # Full 1mm travel is a screw/nut/carrier test with shaft removed;
        # the separate clearance-take-up test includes the shifted shaft.
        for travel in [0.,radial_clearance,P['nose_lock_adjustment']]:
            sh=screw['shape'].translate(tuple(axis*travel));tc=thread_cylinder({'name':screw['name'],'shape':sh},3.)
            eng=overlap(cylinder_interval(tc,axis),ni)
            collisions={name:round(abs(sh.intersect(obj['shape']).Volume()),9) for name,obj in [('carrier',carrier),('nut',nut)]}
            samples.append({'insertion_mm':round(travel,6),'full_diameter_nut_engagement_mm':round(eng,6),'intersection_mm3':collisions,
                            'pass_check':eng>=nt-1e-5 and all(v<1e-6 for v in collisions.values())})
        moved_axle=axle['shape'].translate(tuple(axis*radial_clearance))
        moved_screw=screw['shape'].translate(tuple(axis*radial_clearance))
        takeup={'shaft_shift_mm':round(radial_clearance,6),'screw_shift_mm':round(radial_clearance,6),
                'shaft_to_bore_gap_mm':round(moved_axle.distance(carrier['shape']),6),
                'screw_to_shaft_gap_mm':round(moved_screw.distance(moved_axle),6),
                'screw_shaft_intersection_mm3':round(abs(moved_screw.intersect(moved_axle).Volume()),9),
                'shaft_carrier_intersection_mm3':round(abs(moved_axle.intersect(carrier['shape']).Volume()),9)}
        row={'screw':screw['name'],'receiver':nut['name'],'carrier':carrier['name'],'shaft_diameter_mm':5.,'bore_diameter_mm':bore['radius']*2,
             'head_seating_limit':False,'nominal_screw_shaft_gap_mm':round(screw['shape'].distance(axle['shape']),6),
             'nominal_screw_shaft_intersection_mm3':round(abs(screw['shape'].intersect(axle['shape']).Volume()),9),
             'samples':samples,'clearance_take_up':takeup}
        row['pass_check']=all(s['pass_check'] for s in samples) and row['nominal_screw_shaft_gap_mm']<1e-5 and row['nominal_screw_shaft_intersection_mm3']<1e-6 and all(takeup[k]<1e-5 for k in ['shaft_to_bore_gap_mm','screw_to_shaft_gap_mm','screw_shaft_intersection_mm3','shaft_carrier_intersection_mm3'])
        report['nose_lock_adjustment'].append(row)

for sign in [-1,1]:
    for end in [0,1]:
        screw=by_name[f'K03_shoulder_{sign}_{end}'];bush=by_name[f'K04_bush_{sign}_{end}'];link=by_name[f'K02_link_{sign}']
        shoulder=thread_cylinder(screw,6.);axis=shoulder['axis'];si=cylinder_interval(shoulder,axis)
        bi=interval(bush['shape'],axis);li=interval(link['shape'],axis)
        rw=interval(by_name[f'K03_rear_washer_{sign}_{end}']['shape'],axis);fw=interval(by_name[f'K03_front_washer_{sign}_{end}']['shape'],axis)
        sleeve_stack=(bi[1]-bi[0])+(rw[1]-rw[0])+(fw[1]-fw[0]);link_float=(bi[1]-bi[0])-(li[1]-li[0])
        nut=by_name[f'K03_nut_{sign}_{end}'];ni=interval(nut['shape'],axis)
        nut_gap=si[0]-ni[1];pocket_float=.3 if end==1 else .3;minimum_gap=nut_gap-pocket_float
        report['pivot_stacks'].append({'pivot':screw['name'],'shoulder_length_mm':round(shoulder['length'],6),'sleeve_plus_washers_mm':round(sleeve_stack,6),'link_axial_float_mm':round(link_float,6),
            'nut_gap_from_shoulder_mm':round(nut_gap,6),'pocket_axial_float_mm':pocket_float,'minimum_nut_gap_mm':round(minimum_gap,6),'maximum_thread_relief_mm':P['pivot_thread_relief'],
            'interpretation':'Bushing is located by washers; link is 2mm thinner and has nominal 1mm per-face axial float. Verify printed fit and free pivot movement.',
            'pass_check':sleeve_stack<=shoulder['length']+1e-5 and link_float>=1.99 and minimum_gap>=P['pivot_thread_relief']+.19})
        if end==1:
            report['original_purchase_spec_risks'].append({'screw':screw['name'],'receiver':nut['name'],'original_nut_gap_mm':1.,'maximum_thread_relief_mm':2.,'original_thread_engagement_envelope_mm':3.,
                'required_nut_thickness_mm':4.,'repair':'Nut and saddle pocket moved rearward1.5mm; minimum gap2.2mm includes0.3mm pocket float.'})

report['reviewed_threaded_screw_instances']=len(fasteners)
report['reviewed_threaded_joint_instances']=len(report['threaded_joints'])
report['reviewed_threaded_families']=len(set(re.sub(r'^([LR])_', '', re.sub(r'_[\d.-]+(?:_[\d.-]+)*$','',r['screw'])) for r in report['threaded_joints']))
report['all_checks_pass']=report['inventory']['valid_shapes'] and report['inventory']['single_solid_prints'] and all(r['pass_check'] for key in ['threaded_joints','washers','nose_lock_adjustment','pivot_stacks'] for r in report[key])
(ROOT/'checks/repair_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['threaded_joints','washers','nose_lock_adjustment','pivot_stacks']},indent=2),flush=True)
for key in ['threaded_joints','washers','nose_lock_adjustment','pivot_stacks']:
    for row in report[key]:
        if not row['pass_check']:print('FAILED',key,json.dumps(row),flush=True)
if not report['all_checks_pass']:raise SystemExit(1)
