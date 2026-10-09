#!/usr/bin/env python3
"""V5 inaho-form mechanical reconstruction. Original interface dimensions are not public.
All custom geometry is newly dimensioned; purchased items use cited interface drawings.
CAD/STL mm; GLB m/Y-up. No physical-build or harvest-performance claim.
"""
from pathlib import Path
import math,json,csv,sys
import numpy as np
import cadquery as cq
ROOT=Path(__file__).resolve().parents[1]
P=dict(revision='V5.1',gap_nominal=44.,gap_min=28.,gap_max=96.,root_offset=18.,
 drive_radius=16.,tip_radius=11.,belt_thickness=2.,belt_width=52.,roller_centres=144.,
 belt_free_centres=143.2,tension_travel=3.,alpha=14.,beta=0.,belt_z_left=9.,belt_z_right=13.,crank_radius=25.,rod_length=45.,
 bearing_fit=.15,shaft_fit=.15,rail_length=195.,rail_pitch=25.,rail_hole_first=10.,
 timing_motor_teeth=20,timing_driven_teeth=32,timing_pitch=2.,timing_width=6.,timing_length=220.,
 build_volume=[220,220,220],fit_coupon_count=5,units='mm',
 nose_lock_length=6.,nose_lock_socket_af=1.5,nose_lock_adjustment=1.,
 pivot_thread_relief=2.,pivot_saddle_nut_y=-37.5)
COL={'frame':'#34444d','plate':'#4c5f68','accent':'#31a7a0','poly':'#a6b1b9','belt':'#227c7a',
'metal':'#a8b3ba','motor':'#3a404a','guide':'#c4cbb9','tray':'#43535e','hardware':'#626e79','cover':'#28343e'}
parts=[];coupons=[]
def box(dx,dy,h,x=0,y=0,z=0):
    return cq.Solid.makeBox(dx,dy,h,cq.Vector(x-dx/2,y-dy/2,z))
def cyl(r,h,x=0,y=0,z=0,axis=(0,0,1)):
    return cq.Solid.makeCylinder(r,h,cq.Vector(x,y,z),cq.Vector(*axis))
def tube(ro,ri,h,x=0,y=0,z=0): return cyl(ro,h,x,y,z).cut(cyl(ri,h+2,x,y,z-1))
def roundbox(dx,dy,h,x=0,y=0,z=0,r=2):
    return cq.Workplane('XY').box(dx,dy,h).edges('|Z').fillet(r).val().translate((x,y,z+h/2))
def hexprism(af,h,x=0,y=0,z=0):
    return cq.Workplane('XY').polygon(6,af/math.cos(math.pi/6)).extrude(h).val().translate((x,y,z))
def holes(sh,pts,d,z,h):
    for x,y in pts: sh=sh.cut(cyl(d/2,h,x,y,z))
    return sh
def nutpockets(sh,pts,m,z,h=None):
    af={3:5.8,4:7.4,5:8.4,6:10.4}[m]; h=h or {3:2.6,4:3.4,5:4.3,6:5.3}[m]
    for x,y in pts: sh=sh.cut(hexprism(af,h,x,y,z))
    return sh
def slot(d,L,h,x,y,z):
    # total centre travel L along Y
    if L<=0:return cyl(d/2,h,x,y,z)
    return cq.Workplane('XY').center(x,y).slot2D(L+d,d,90).extrude(h).val().translate((0,0,z))
def orient(sh,axis):
    if tuple(axis)==(0,0,1):return sh
    if tuple(axis)==(0,0,-1):return sh.rotate((0,0,0),(1,0,0),180)
    if tuple(axis)==(1,0,0):return sh.rotate((0,0,0),(0,1,0),90)
    if tuple(axis)==(-1,0,0):return sh.rotate((0,0,0),(0,1,0),-90)
    if tuple(axis)==(0,1,0):return sh.rotate((0,0,0),(1,0,0),-90)
    if tuple(axis)==(0,-1,0):return sh.rotate((0,0,0),(1,0,0),90)
    raise ValueError(axis)
def axial_hex(af,h,pos,axis):return orient(hexprism(af,h),axis).translate(pos)
def axial_cyl(r,h,pos,axis):return cyl(r,h,*pos,axis=axis)
def theta(g):
    s=g/2+P['root_offset'];r=P['crank_radius'];L=P['rod_length'];return math.acos((s*s+r*r-L*L)/(2*s*r))
def add(name,sh,group='fixed',kind='print',color='frame',note='',spec='',material='PETG',print_axis='Z',motion=None):
    sh=sh.clean()
    if not sh.isValid() or not sh.Solids():raise ValueError('Invalid '+name)
    rec=dict(name=name,shape=sh,group=group,kind=kind,color=COL[color],note=note,spec=spec,material=material,print_axis=print_axis)
    if motion: rec['motion']=motion
    parts.append(rec);return rec
def addside(side,name,sh,cassette=True,**kw):
    if side<0:sh=sh.mirror('YZ')
    off=(side*(P['gap_nominal']/2+P['root_offset']), P['alpha'] if side<0 and cassette else 0, -P['beta'] if side<0 and cassette else 0)
    sh=sh.translate(off)
    return add(('R_' if side>0 else 'L_')+name,sh,group=('R' if side>0 else 'L'),**kw)
def bearing(x,y,z):return tube(11,4,7,x,y,z)
def washer_shape(m,th=1):return tube({2:2.5,2.5:3,3:3.5,4:4.5,5:5,6:6}[m],(m+.3)/2,th)
def screw_shape(m,L):
    # Z=0 under head; screw extends +Z; head behind seat. Threads omitted.
    sh=cyl(m/2,L).fuse(cyl({2:1.9,2.5:2.25,3:2.75,4:3.5,5:4.25,6:5}[m],m,z=-m))
    return sh.cut(hexprism({2:1.5,2.5:2,3:2.5,4:3,5:4,6:5}[m],m*.7,z=-m-.01))
def nose_set_screw_shape():
    # ISO4026 M3x6: headless, flat tip diameter2, AF1.5 socket depth1.5.
    # Z=0 socket end, +Z tip. Major-diameter envelope; threads omitted.
    L=P['nose_lock_length']
    sh=cyl(1.5,L-.75,z=.25)
    sh=sh.fuse(cq.Solid.makeCone(1.25,1.5,.25))
    sh=sh.fuse(cq.Solid.makeCone(1.5,1.,.5,cq.Vector(0,0,L-.5)))
    return sh.cut(hexprism(P['nose_lock_socket_af'],1.51,z=-.01))
def nut_shape(m):return hexprism({2:4,3:5.5,4:7,5:8,6:10}[m],{2:1.6,3:2.4,4:3.2,5:4,6:5}[m]).cut(cyl((m+.04)/2,8,z=-1))
def hw(name,shape,group='fixed',spec='',**kw):return add(name,shape,group,kind='buy',color='hardware',material='steel',spec=spec,**kw)
def fast(name,m,L,pos,axis=(0,0,-1),side=None,group='fixed'):
    sh=orient(screw_shape(m,L),axis).translate(pos)
    spec=f'ISO 4762 M{m}x{L}'
    if side is None:return hw(name,sh,group,spec)
    return addside(side,name,sh,kind='buy',color='hardware',material='steel',spec=spec)
def nut(name,m,pos,axis=(0,0,1),side=None,group='fixed'):
    sh=orient(nut_shape(m),axis).translate(pos)
    if side is None:return hw(name,sh,group,f'ISO 4032 M{m}')
    return addside(side,name,sh,kind='buy',color='hardware',material='steel',spec=f'ISO 4032 M{m}')
def washer(name,m,th,pos,axis=(0,0,1),side=None,group='fixed'):
    sh=orient(washer_shape(m,th),axis).translate(pos)
    if side is None:return hw(name,sh,group,f'Washer M{m} t{th}')
    return addside(side,name,sh,kind='buy',color='hardware',material='steel',spec=f'Washer M{m} t{th}')
COLLAR_SPEC='Ganter GN705-8-E / DIN705 collar 8ID x16OD x8, M4x6 screw included'
def collar(x,y,z):
    # Include the radial set-screw envelope beyond the circular body.
    return tube(8,4,8,x,y,z).fuse(cyl(2,6,x+4,y,z+4,axis=(1,0,0)))
def coupling(x,y,z,reverse=False):
    sh=cyl(9,25,x,y,z)
    sh=sh.cut(cyl(4 if not reverse else 2.5,12.5,x,y,z-.01)).cut(cyl(2.5 if not reverse else 4,12.52,x,y,z+12.49))
    return sh
def motor(x,y,face,down=True):
    # SY42STH38-1684A verified drawing: 31 square, M3 depth4.5, boss22x2, shaft5x24.
    sh=roundbox(42.3,42.3,38,r=3).fuse(cyl(11,2,z=-2)).fuse(cyl(2.5,24,z=-24))
    sh=sh.cut(box(10,5,20,x=0,y=4.5,z=-24))
    for xx in [-15.5,15.5]:
        for yy in [-15.5,15.5]:sh=sh.cut(cyl(1.6,4.5,xx,yy,z=0))
    if not down:sh=sh.rotate((0,0,0),(1,0,0),180)
    return sh.translate((x,y,face))

def hull_loop(R,r,dx,C,z,h):
    D=math.hypot(dx,C);e=np.array([dx,C])/D;n0=(R-r)/D*e
    v=math.sqrt(1-((R-r)/D)**2)*np.array([-e[1],e[0]])
    n1=n0+v;n2=n0-v;c=np.array([dx,C])
    a=R*n1;b=c+r*n1;d=c+r*n2;f=R*n2
    return cq.Workplane('XY').moveTo(*a).lineTo(*b).threePointArc(tuple(c+r*e),tuple(d)).lineTo(*f).threePointArc(tuple(-R*e),tuple(a)).close().extrude(h).val().translate((0,0,z))
def belt_z(side):return P['belt_z_left'] if side<0 else P['belt_z_right']
def contact_belt(C,side=1):
    z=belt_z(side);w=P['belt_width']
    return hull_loop(18,13,-5,C,z,w).cut(hull_loop(16,11,-5,C,z-1,w+2))
def belt_length(R,r,D):
    a=math.asin((R-r)/D)
    return 2*math.sqrt(D*D-(R-r)**2)+math.pi*(R+r)+2*a*(R-r)
def timing_C():
    R=P['timing_driven_teeth']/math.pi;r=P['timing_motor_teeth']/math.pi
    lo,hi=30.,150.
    for _ in range(70):
        mid=(lo+hi)/2
        if belt_length(R,r,mid)<P['timing_length']:lo=mid
        else:hi=mid
    return (lo+hi)/2
P['timing_centres']=timing_C()
def motor11(x,y,z):
    # OMC 11HS12-0674S Rev A: 28.2 square,31.5 body,23 square M2.5 min4,5x20 D-shaft.
    sh=roundbox(28.2,28.2,31.5,z=z-31.5,r=2).fuse(tube(11,2.5,2,z=z))
    sh=holes(sh,[(a,b) for a in [-11.5,11.5] for b in [-11.5,11.5]],2.7,z-4,4.1)
    return sh.translate((x,y,0))
def collar5(x,y,z):return tube(5,2.5,6,x,y,z).fuse(cyl(1.5,4,x+2.5,y,z+3,axis=(1,0,0)))
def bearing625(x,y,z):return tube(8,2.5,5,x,y,z)
def axial_tube(ro,ri,h,pos,axis):return orient(tube(ro,ri,h),axis).translate(pos)
def side_fast(side,name,m,L,pos,axis=(0,0,-1),cassette=True):
    return addside(side,name,orient(screw_shape(m,L),axis).translate(pos),cassette=cassette,kind='buy',color='hardware',material='steel',spec=f'ISO 4762 M{m}x{L}')
def side_nut(side,name,m,pos,axis=(0,0,1),cassette=True):
    return addside(side,name,orient(nut_shape(m),axis).translate(pos),cassette=cassette,kind='buy',color='hardware',material='steel',spec=f'ISO 4032 M{m}')
def side_washer(side,name,m,th,pos,axis=(0,0,1),cassette=True):
    return addside(side,name,orient(washer_shape(m,th),axis).translate(pos),cassette=cassette,kind='buy',color='hardware',material='steel',spec=f'Washer M{m} t{th}')
def pulley(teeth,z,x=0,y=0):
    # 20T: RobotDigg GT2-20T-5B-6, manufacturer dimensional drawing.
    # Hub8 + flange1 + groove7.5 + flange1 =17.5. Hub up, M3 at4 from hub end.
    # 32T: MISUMI GPA32GT2060-A-P5. Teeth are an interface envelope.
    if teeth==20:
        sh=tube(9,2.5,1,x,y,z).fuse(tube(6.11,2.5,7.5,x,y,z+1)).fuse(tube(9,2.5,1,x,y,z+8.5)).fuse(tube(5,2.5,8,x,y,z+9.5))
        sh=sh.fuse(cyl(1.5,3,x+2.5,y,z+13.5,axis=(1,0,0)))
        sh=sh.fuse(cyl(1.5,3,x,y+2,z+13.5,axis=(0,1,0)))
        return sh
    od={20:12.22,32:19.86}[teeth];fl={20:17.,32:25.}[teeth]
    sh=tube(od/2,2.5,10.3,x,y,z)
    sh=sh.fuse(tube(fl/2,2.5,1.65,x,y,z)).fuse(tube(fl/2,2.5,1.65,x,y,z+8.65))
    return sh
def timing_belt():
    # Neutral line derived from exact unequal-pulley tangent length. Teeth omitted.
    R=32/math.pi;r=20/math.pi;C=P['timing_centres']
    # Pulley tooth-tip radii are pitch radius-0.254. Smooth belt envelope begins there.
    return hull_loop(R+1.10,r+1.10,0,C,83.75,6).cut(hull_loop(R-.254,r-.254,0,C,82.75,8)).rotate((0,0,0),(0,0,1),180)

def build_frame():
    b=box(210,8,21,y=-53,z=16).fuse(box(80,8,68,y=-53,z=-52))
    for x in [-97,97]:b=b.fuse(box(16,8,46,x=x,y=-53,z=-20))
    b=b.cut(box(40.4,10,36.4,y=-53,z=-36.2))
    for x in [-78,78]:b=b.fuse(box(14,8,41,x=x,y=-53,z=-4))
    for x in np.arange(-87.5,88,25):
        b=b.cut(cyl(1.7,12,x,-59,32,axis=(0,1,0))).cut(axial_hex(5.8,2.6,(x,-57,32),(0,1,0)))
    fixedpts=[(-25,-18),(25,-18)]+[(x,z) for x in [-26,26] for z in [-42,6]]
    fixedpts += [(x,-12) for x in [-93.7,93.7]]+[(x,19) for x in [-5,5]]
    for x,z in fixedpts:
        b=b.cut(cyl(2.2 if abs(x) in [25,26] else 1.7,12,x,-59,z,axis=(0,1,0)))
        if z in [-12,19]:b=b.cut(axial_hex(5.8,2.6,(x,-57,z),(0,1,0)))
    for x in [-78,78]:
        for z in [2,12]:b=b.cut(cyl(2.2,12,x,-59,z,axis=(0,1,0)))
    for zz in [14,22]:
        b=b.cut(cyl(1.7,12,101,-59,zz,axis=(0,1,0))).cut(axial_hex(5.8,2.6,(101,-57,zz),(0,1,0)))
    add('F01_rear_rail_backbone',b,print_axis='Y',note='Vertical rear backbone. No bench legs. Motors clear the upper cutouts.')
    rail=box(195,8,12,y=-45,z=26)
    for i,x in enumerate(np.arange(-87.5,88,25)):
        rail=rail.cut(cyl(1.7,10,x,-50,32,axis=(0,1,0))).cut(cyl(3,4,x,-45,32,axis=(0,1,0)))
        fast(f'F02_rail_bolt_{i}',3,14,(x,-45,32),(0,-1,0));nut(f'F02_rail_nut_{i}',3,(x,-57,32),(0,1,0))
    hw('F02_MGN12R_195',rail,spec='HIWIN MGN12R L195, E10, pitch25, eight mounting holes')
    for side in [-1,1]:
        s=P['gap_nominal']/2+18
        block=box(45.4,10,27,y=-41,z=18.5).cut(box(47,5.8,12.2,y=-43.9,z=25.9))
        for x in [-10,10]:
            for z in [22,42]:block=block.cut(cyl(1.6,3.5,x,-36,z,axis=(0,-1,0)))
        addside(side,'H01_MGN12H',block,cassette=False,kind='buy',color='metal',material='steel',spec='HIWIN MGN12H 45.4x27, installed height13, M3 depth3.5')
        a=P['alpha'] if side<0 else 0;dz=-P['beta'] if side<0 else 0
        saddle=box(45,5,36,y=-33.5,z=14).fuse(box(22,5,42,y=-33.5,z=-28)).fuse(box(10,38+a,63,x=24,y=(-31+7+a)/2,z=6+dz))
        for x in [-10,10]:
            for z in [22,42]:
                saddle=saddle.cut(cyl(1.7,7,x,-37,z,axis=(0,1,0)))
                side_fast(side,f'C01_rail_bolt_{x}_{z}',3,8,(x,-31,z),(0,-1,0),False)
        # Keep the M5 nut beyond the shoulder screw's maximum 2mm thread
        # relief. The nominal 2.5mm gap remains >=2.2mm with pocket float.
        saddle=saddle.cut(cyl(2.7,8,0,-37,-18,axis=(0,1,0))).cut(axial_hex(8.4,4.3,(0,P['pivot_saddle_nut_y'],-18),(0,1,0)))
        for y in [-18+a,a]:
            for z in [10.8+dz,61+dz]:
                saddle=saddle.cut(cyl(2.2,20,24,y,z-8)).cut(hexprism(7.4,3.4,24,y,z))
                saddle=saddle.cut(box(7,8,3.4,x=28,y=y,z=z))
        if side>0:saddle=saddle.fuse(box(4.5,13,4,x=20.25,y=-37.5,z=46))
        addside(side,'C01_carriage_saddle',saddle,cassette=False,print_axis='X' if side>0 else 'negX',note='5 mm rail pad. Outboard spine carries both cassette plates. V5.1 M5 pocket moved rearward1.5mm so the nut clears the shoulder thread relief; reprint both saddles when upgrading.')
    # Closed stop at block inner edges; open stops at outer edges.
    for x,w,label in [(0,18.6,'closed'),(-93.7,10,'open_L'),(93.7,10,'open_R')]:
        low=16 if x==0 else -18
        bolt_z=19 if x==0 else -12
        st=box(w,8,26-low,x=x,y=-37,z=low).fuse(box(w,8,10 if x==0 else 12,x=x,y=-45,z=low))
        pts=[-5,5] if x==0 else [x]
        for xx in pts:
            st=st.cut(cyl(1.7,25,xx,-58,bolt_z,axis=(0,1,0)))
            fast(f'F03_stop_{label}_bolt_{xx}',3,28,(xx,-33,bolt_z),(0,-1,0));nut(f'F03_stop_{label}_nut_{xx}',3,(xx,-57,bolt_z),(0,1,0))
        add(f'F03_stop_{label}',st,print_axis='Y',note='Nominal gap end stop; not a load-rated crash stop.')
    # Independent bearing support for the opening shaft; motor sees no link radial load.
    t=box(40,24,32,y=-62,z=-34)
    t=t.cut(cyl(6.2,26,0,-75,-18,axis=(0,1,0)))
    for yy in [-74,-57]:t=t.cut(cyl(11.075,7,0,yy,-18,axis=(0,1,0)))
    for x in [-25,25]:
        t=t.fuse(box(16,7,24,x=x,y=-60.5,z=-30)).cut(cyl(2.2,18,x,-66,-18,axis=(0,1,0)))
        t=t.cut(axial_hex(7.4,3.4,(x,-64,-18),(0,1,0)))
        fast(f'F04_tower_bolt_{x}',4,18,(x,-49,-18),(0,-1,0));nut(f'F04_tower_nut_{x}',4,(x,-64,-18),(0,1,0))
    for x in [-15,15]:
        t=t.cut(cyl(1.7,26,x,-75,-18,axis=(0,1,0)))
        for yy in [-71,-55.4]:
            t=t.cut(axial_hex(5.8,2.6,(x,yy,-18),(0,1,0))).cut(box(6,2.6,6,x=x+(3 if x>0 else -3),y=yy+1.3,z=-21))
    add('F04_opening_bearing_tower',t,print_axis='Y')
    for yy in [-74,-57]:hw(f'H02_opening_608_{yy}',axial_tube(11,4,7,(0,yy,-18),(0,1,0)),spec='608-2Z 8x22x7')
    for label,y,dr in [('rear',-76,1),('front',-48,-1)]:
        cap=axial_tube(18,9.5,2,(0,-76 if dr==1 else -50,-18),(0,1,0))
        for x in [-15,15]:
            cap=cap.cut(cyl(1.7,5,x,y-3,-18,axis=(0,1,0)))
            fast(f'F05_{label}_bolt_{x}',3,10,(x,y,-18),(0,dr,0))
            nut(f'F05_{label}_nut_{x}',3,(x,-71 if dr==1 else -55.4,-18),(0,1,0))
        add(f'F05_bearing_cap_{label}',cap,print_axis='Y')
    hw('H03_opening_shaft',cyl(4,76,0,-107,-18,axis=(0,1,0)),spec='Ground steel shaft Ø8 h6 L76, deburred ends')
    for yy in [-75,-50]:add(f'F06_inner_spacer_{yy}',axial_tube(5,4.1,1,(0,yy,-18),(0,1,0)),color='poly',print_axis='Y')
    for yy in [-83,-49]:hw(f'H04_opening_collar_{yy}',orient(collar(0,0,0),(0,1,0)).translate((0,yy,-18)),spec=COLLAR_SPEC)
    hw('H05_opening_coupler',orient(coupling(0,0,0,True),(0,1,0)).translate((0,-122,-18)),spec='STEPPERONLINE ST-FC04 5x8 coupling Ø18x25; included screws')
    yk=box(62,6,62,y=-131,z=-49).cut(cyl(11.3,8,0,-135,-18,axis=(0,1,0)))
    for x in [-26,26]:
        for z in [-42,6]:
            yk=yk.fuse(box(8,71,8,x=x,y=-92.5,z=z-4))
            yk=yk.cut(cyl(2.2,18,x,-73,z,axis=(0,1,0))).cut(axial_hex(7.4,3.4,(x,-66,z),(0,1,0)))
            yk=yk.cut(box(8,3.4,8,x=x+4,y=-64.3,z=z-4))
            fast(f'F07_yoke_bolt_{x}_{z}',4,20,(x,-49,z),(0,-1,0));nut(f'F07_yoke_nut_{x}_{z}',4,(x,-66,z),(0,1,0))
    for x in [-15.5,15.5]:
        for z in [-33.5,-2.5]:
            yk=yk.cut(cyl(1.7,8,x,-135,z,axis=(0,1,0)))
            washer(f'F07_motor_washer_{x}_{z}',3,.5,(x,-128,z),(0,1,0));fast(f'F07_motor_bolt_{x}_{z}',3,10,(x,-127.5,z),(0,-1,0))
    add('F07_opening_motor_yoke',yk,print_axis='Y')
    hw('H06_opening_motor',orient(motor(0,0,0,False),(0,1,0)).translate((0,-134,-18)),spec='Pololu2267 / SOYO SY42STH38-1684A, 42.3 square, shaft5x24')

def build_cassette(side):
    C=P['roller_centres']; tx=-5
    bz=belt_z(side); w=P['belt_width']; lo=bz-3;hi=bz+w+1
    # Structural decks nest between the outboard saddle faces.
    decks=[]
    for tag,z,h in [('lower',-2,8),('upper',69,8)]:
        deck=hull_loop(20,15,tx,C,z,h).intersect(box(70,210,12,x=19.5,y=78,z=z-1))
        deck=deck.fuse(box(12,30,h,x=23,y=-9,z=z))
        deck=holes(deck,[(24,-18),(24,0)],4.4,z-1,h+2)
        deck=holes(deck,[(-6,35),(-6,65)],3.4,z-1,h+2)
        deck=deck.cut(slot(8.6,6,h+2,tx,C,z-1)).cut(box(9,14,h+2,x=tx,y=122,z=z-1))
        deck=deck.cut(cyl(2.2,h+2,tx,133,z-1))
        nz=1.2 if tag=='lower' else 71.0
        deck=deck.cut(hexprism(7.6,3.4,tx,133,nz)).cut(box(10,8,3.4,x=tx+5,y=130,z=nz))
        deck=deck.cut(cyl(5.75,h+2,0,0,z-1))
        if tag=='lower':
            deck=deck.cut(cyl(8.075,7,0,0,-2)).cut(cyl(18.5,2,0,0,5))
            deck=deck.cut(box(50,70,1,x=0,y=137,z=5))
        else:
            deck=deck.cut(cyl(8.075,7,0,0,70))
            deck=deck.cut(box(12,32,10,x=-3,y=85,z=68))
        for yy in [-13,13]:
            deck=deck.cut(cyl(1.7,h+2,0,yy,z-1))
            zn=1.2 if tag=='lower' else 71.2
            deck=deck.cut(hexprism(5.8,2.6,0,yy,zn)).cut(box(7,7,2.6,x=3.5,y=yy,z=zn))
        # Outer cover anchors are outside the running belt.
        for yy,xx in [(45,16),(100,12)]:
            deck=deck.fuse(box(10,10,h,x=xx,y=yy,z=z)).cut(cyl(1.7,h+2,xx,yy,z-1))
            zn=1.2 if tag=='lower' else 70.5
            deck=deck.cut(hexprism(5.8,2.6,xx,yy,zn)).cut(box(7,6,2.6,x=xx+3.5,y=yy,z=zn))
        # Tray posts fasten to the left lower deck only.
        if side<0 and tag=='lower':
            deck=deck.fuse(box(9,12,8,x=10,y=130,z=-2))
            for xx,yy in [(14,32),(10,130)]:
                deck=deck.cut(cyl(1.7,12,xx,yy,-4)).cut(cyl(3,4,xx,yy,3))
        decks.append(deck)
        addside(side,f'C02_{tag}_deck',deck,color='plate',print_axis='negZ' if tag=='upper' else 'Z')
        for k,yy in enumerate([-18,0]):
            seat=-2 if tag=='lower' else 77;az=(0,0,1) if tag=='lower' else (0,0,-1)
            side_fast(side,f'C02_{tag}_root_bolt_{k}',4,18,(24,yy,seat),az)
            side_nut(side,f'C02_{tag}_root_nut_{k}',4,(24,yy,10.8 if tag=='lower' else 61))
    # Belt backing and internal tension reaction. All nut windows open outboard.
    gu=box(5,88,63,x=-12.9,y=66,z=6)
    for yy in [35,65]:
        gu=gu.fuse(box(11,10,63,x=-9.9,y=yy,z=6))
        for zz in [6,57]:gu=gu.cut(cyl(1.7,14,-6,yy,zz))
        for zz in [8.6,63.4]:gu=gu.cut(hexprism(5.8,2.6,-6,yy,zz)).cut(box(10,6,2.6,x=-1,y=yy,z=zz))
        side_fast(side,f'C03_lower_bolt_{yy}',3,14,(-6,yy,-2),(0,0,1))
        side_nut(side,f'C03_lower_nut_{yy}',3,(-6,yy,8.6))
        side_fast(side,f'C03_upper_bolt_{yy}',3,14,(-6,yy,77),(0,0,-1))
        side_nut(side,f'C03_upper_nut_{yy}',3,(-6,yy,63.4))
    gu=gu.fuse(box(14,8,10,x=-8,y=106,z=34)).cut(cyl(1.7,10,-5,101,39,axis=(0,1,0)))
    gu=gu.cut(axial_hex(5.8,2.6,(-5,107.4,39),(0,1,0))).cut(box(9,2.6,6,x=-.5,y=108.7,z=36))
    addside(side,'C03_inner_belt_backing',gu,color='guide',print_axis='negX' if side>0 else 'X',note='0.6 mm nominal backing clearance. Tension jack uses a top service window and an L hex key.')
    # Root shaft runs in two retained bearings, torque passes through split drum clamp.
    for zz,tag in [(0,'lower'),(70,'upper')]:addside(side,'H10_625_root_'+tag,bearing625(0,0,zz),kind='buy',color='metal',material='steel',spec='625-2Z bearing 5x16x5')
    for tag,z,h in [('lower_outer',-2,2),('lower_inner',5,lo-5),('upper_inner',hi+2,70-hi-2),('upper_outer',75,6.4)]:
        addside(side,'C04_root_spacer_'+tag,tube(4,2.6,h,z=z),color='poly')
    for tag,z,spz,az,seat,nz in [('lower',-4,-2,(0,0,1),-4,1.2),('upper',77,75,(0,0,-1),79,71.2)]:
        cap=tube(17,6.8,2,z=z).fuse(tube(8,6.8,2,z=spz))
        cap=holes(cap,[(0,-13),(0,13)],3.4,z-3,8)
        addside(side,'C05_root_bearing_cap_'+tag,cap,color='frame')
        for yy in [-13,13]:
            if tag=='upper':
                # Button head clears the lowered 32T flange; full nut engagement with L10.
                bs=cyl(1.5,10).fuse(cyl(2.85,1.65,z=-1.65)).cut(hexprism(2,1.2,z=-1.7))
                addside(side,f'C05_{tag}_bolt_{yy}',orient(bs,az).translate((0,yy,seat)),kind='buy',color='hardware',material='steel',spec='ISO 7380-1 M3x10 button head; head OD5.7 x1.65, AF2')
            else:side_fast(side,f'C05_{tag}_bolt_{yy}',3,8,(0,yy,seat),az)
            side_nut(side,f'C05_{tag}_nut_{yy}',3,(0,yy,nz))
    addside(side,'H11_root_shaft',cyl(2.5,105,z=-8),kind='buy',color='metal',material='steel',spec='Ground steel shaft Ø5 h6 L105, custom cut and deburr')
    addside(side,'H12_root_collar',collar5(0,0,-8),kind='buy',color='hardware',material='steel',spec='Ganter GN705-5-E collar 5ID x10OD x6; M3x4 screw included')
    roller=tube(16,2.55,w+2,z=bz-1).fuse(tube(18,2.55,2,z=lo)).fuse(tube(18,2.55,2,z=hi))
    roller=roller.cut(box(.5,45,70,y=0,z=4))
    for yy in [-7,7]:
        for zz in [18,56]:
            roller=roller.cut(cyl(1.7,40,-20,yy,zz,axis=(1,0,0))).cut(cyl(3,15,8,yy,zz,axis=(1,0,0)))
            roller=roller.cut(axial_hex(5.8,12,(-20,yy,zz),(1,0,0)))
            side_fast(side,f'C06_clamp_bolt_{yy}_{zz}',3,20,(8,yy,zz),(-1,0,0));side_nut(side,f'C06_clamp_nut_{yy}_{zz}',3,(-10.4,yy,zz),(1,0,0))
    for tag,half in [('inner',roller.intersect(box(30,50,70,x=-15.25,z=4))),('outer',roller.intersect(box(30,50,70,x=15.25,z=4)))]:
        addside(side,'C06_drive_drum_'+tag,half,color='poly',print_axis='X' if tag=='inner' else 'negX')
    rec=addside(side,'C07_contact_belt_TPU',contact_belt(C,side),color='belt',material='TPU 95A',note=f'52 mm wide; assembled Z {bz} to {bz+w}. V5 design datum, not original measurement. Print free loop C143.2; install C144; verify friction and tracking on bench.')
    rec['print_override']=contact_belt(P['belt_free_centres'],side)
    # Small nose roller and single carrier support BOTH axle ends inside the belt loop.
    idler=tube(11,4.2,w+2,tx,C,bz-1).fuse(tube(13,4.2,2,tx,C,lo)).fuse(tube(13,4.2,2,tx,C,hi))
    for zz in [bz-1,bz+w-4]:idler=idler.cut(cyl(8.075,5,tx,C,zz))
    addside(side,'C08_nose_roller',idler,color='poly')
    for zz,tag in [(bz-1,'lower'),(bz+w-4,'upper')]:addside(side,'H13_625_nose_'+tag,bearing625(tx,C,zz),kind='buy',color='metal',material='steel',spec='625-2Z bearing 5x16x5')
    for zz,h,tag in [(-2,bz+1,'lower'),(bz+4,w-8,'centre'),(bz+w+1,77-(bz+w+1)-.15,'upper')]:addside(side,'C09_nose_spacer_'+tag,tube(4,2.6,h,tx,C,zz),color='poly',note='Upper spacer provides 0.15 mm nominal axial endplay; measure after printing.')
    carrier=box(18,34,6,x=tx,y=135,z=-8).fuse(box(18,34,6,x=tx,y=135,z=77)).fuse(box(8,6,79,x=tx,y=122,z=-2))
    carrier=carrier.cut(cyl(2.575,94,tx,C,-9))
    for zz in [-9,76]:carrier=carrier.cut(slot(4.4,6,8,tx,133,zz))
    for zz in [-5,80]:
        carrier=carrier.cut(cyl(1.7,10,tx,143,zz,axis=(0,1,0))).cut(axial_hex(5.8,2.6,(tx,148,zz),(0,1,0)))
        carrier=carrier.cut(box(12,2.6,6,x=tx+6,y=149.3,z=zz-3))
        # A seated M3x6 cap screw +0.5 washer stopped exactly at first shaft
        # contact and could not take up the 0.075 radial bore clearance.
        # The headless screw adjusts independently in the captured metal nut.
        # This CAD position is first contact, not a simulated preload.
        addside(side,f'C10_axle_lock_{zz}',orient(nose_set_screw_shape(),(0,-1,0)).translate((tx,152.5,zz)),
            kind='buy',color='hardware',material='A2 stainless steel',
            spec='ISO 4026 M3x6 flat point set screw; tip OD2, AF1.5; no washer',
            note='Tighten AF1.5 into the captured M3 nut after carrier/arm installation. Headless screw permits radial clearance take-up. CAD shows first shaft contact; verify shaft retention and free roller rotation on the bench.')
        side_nut(side,f'C10_axle_nut_{zz}',3,(tx,148,zz),(0,1,0))
    # Detachable keyed top arm permits a real vertical assembly path for the upper deck.
    lower=carrier.intersect(box(80,80,86,x=-5,y=130,z=-9)).fuse(box(6.6,4.6,2,x=-5,y=122,z=77))
    upper=carrier.intersect(box(80,80,7,x=-5,y=130,z=77)).cut(box(7,5,2.3,x=-5,y=122,z=76.9))
    lower=lower.cut(cyl(1.7,17,-5,122,66)).cut(hexprism(5.8,2.6,-5,122,70)).cut(box(7,5.8,2.6,x=-1.5,y=122,z=70))
    upper=upper.cut(cyl(1.7,8,-5,122,76))
    addside(side,'C10_internal_nose_carrier',lower,color='accent',print_axis='X' if side>0 else 'negX',note='Keyed removable top arm enables upper-deck insertion. The assembled carrier moves both axle ends together.')
    addside(side,'C10_removable_top_arm',upper,color='accent')
    side_fast(side,'C10_top_arm_joint_bolt',3,16,(-5,122,83));side_nut(side,'C10_top_arm_joint_nut',3,(-5,122,70))
    addside(side,'H14_nose_axle',cyl(2.5,90,tx,C,-8),kind='buy',color='metal',material='steel',spec='Ground steel shaft Ø5 h6 L90; custom cut and deburr')
    for tag,seat,nz,az in [('lower',-9,1.2,(0,0,1)),('upper',84,71.,(0,0,-1))]:
        side_fast(side,'C10_clamp_'+tag,4,14,(tx,133,seat),az)
        side_washer(side,'C10_clamp_washer_'+tag,4,1,(tx,133,-9 if tag=='lower' else 83))
        side_nut(side,'C10_clamp_nut_'+tag,4,(tx,133,nz))
    jack=side_fast(side,'C10_jack_screw',3,25,(-5,94,39),(0,1,0))
    jack['spec']='ISO 4762 M3x25 fully threaded; Bossard BN3 or equivalent; AF2.5'
    jack['note']='Use the fully threaded variant, with complete threads within1.5mm of the head. Standard partial thread length18 leaves an unthreaded shank at the locknut. Verify the locknut travels freely before assembly.'
    side_nut(side,'C10_jack_captive',3,(-5,107.6,39),(0,1,0));side_nut(side,'C10_jack_locknut',3,(-5,99.6,39),(0,1,0))
    build_belt_drive(side)

def build_belt_drive(side):
    C=P['timing_centres']; cy=-C
    # Outboard cantilever supports a removable motor plate behind the rail.
    br=box(10,118,8,x=24,y=-53,z=69).fuse(box(10,22,16,x=24,y=-87,z=61))
    br=br.fuse(box(52,8,5,x=0,y=cy-22,z=73)).fuse(box(52,8,5,x=0,y=cy+22,z=73))
    br=holes(br,[(24,-18),(24,0)],4.4,68,12)
    # Cassette upper deck occupies the front footprint. Keep bracket back of it.
    br=br.cut(box(18,31,12,x=23,y=-8.5,z=68))
    for yy in [cy-22,cy+22]:
        for xx in [-20,20]:
            br=br.cut(cyl(1.7,8,xx,yy,72)).cut(hexprism(5.8,2.6,xx,yy,73))
            side_fast(side,f'C11_plate_bolt_{xx}_{yy:.2f}',3,12,(xx,yy,82),(0,0,-1));side_nut(side,f'C11_plate_nut_{xx}_{yy:.2f}',3,(xx,yy,73))
    # Main attachment standoffs carry load to upper deck at its root screws; shared screw stack uses longer bolts.
    # Replace initial upper root screws/nuts and extend saddle studs using existing two axes.
    for p in list(parts):
        if p['name'] in [f"{'L' if side<0 else 'R'}_C02_upper_root_bolt_{i}" for i in [0,1]]:parts.remove(p)
    for k,yy in enumerate([-18,0]):
        # Foot sits on upper deck and connects to the rear cantilever.
        foot=box(10,10,5,x=24,y=yy,z=77)
        br=br.fuse(foot).fuse(box(10,abs(yy+28)+5,5,x=24,y=(yy-28)/2,z=77))
        side_fast(side,f'C11_root_bolt_{k}',4,25,(24,yy,82),(0,0,-1))
    br=holes(br,[(24,-18),(24,0)],4.4,68,16)
    for yy in [cy-22,cy+22]:
        for xx in [-20,20]:
            br=br.cut(cyl(1.7,16,xx,yy,68)).cut(hexprism(5.8,2.6,xx,yy,73))
            br=br.cut(box(12,6,2.6,x=xx+(6 if xx>0 else -6),y=yy,z=73))
    # Independent hood anchors: none of the service screws secure a deck or motor slide.
    br=br.fuse(box(50,23,5,y=-116,z=73))
    for xx,yy,zz in [(24,-38,77),(-20,-123,78),(20,-123,78)]:
        br=br.fuse(cyl(4,102-zz,xx,yy,zz)).cut(cyl(1.7,12,xx,yy,91))
        br=br.cut(hexprism(5.8,2.6,xx,yy,93)).cut(box(8,5.8,2.6,x=xx+4,y=yy,z=93))
        side_nut(side,f'C14_service_nut_{xx}_{yy}',3,(xx,yy,93))
    # Integral tie saddles keep motor leads on the structure when the hood is removed.
    for xx in [-7,7]:
        lug=box(7,6,12,x=xx,y=-124,z=66)
        lug=lug.cut(box(3.6,10,2,x=xx,y=-124,z=69))
        br=br.fuse(lug)
    for yy in [cy-22,cy+22]:
        for xx in [-20,20]:
            br=br.cut(cyl(1.7,16,xx,yy,68)).cut(hexprism(5.8,2.6,xx,yy,73))
            br=br.cut(box(12,6,2.6,x=xx+(6 if xx>0 else -6),y=yy,z=73))
    addside(side,'C11_rear_motor_bracket',br,color='frame',print_axis='X' if side>0 else 'negX')
    mp=roundbox(50,60,4,y=cy,z=78,r=2).cut(cyl(11.3,6,0,cy,77))
    for xx in [-20,20]:
        for yy in [cy-22,cy+22]:mp=mp.cut(slot(3.4,6,6,xx,yy,77))
    for xx in [-11.5,11.5]:
        for yy in [cy-11.5,cy+11.5]:
            mp=mp.cut(cyl(1.4,6,xx,yy,77))
            side_washer(side,f'C12_motor_washer_{xx}_{yy:.2f}',2.5,.5,(xx,yy,82));side_fast(side,f'C12_motor_bolt_{xx}_{yy:.2f}',2.5,8,(xx,yy,82.5))
    addside(side,'C12_motor_slide_plate',mp,color='plate')
    addside(side,'H15_NEMA11_belt_motor',motor11(0,cy,78),kind='buy',color='motor',material='motor',spec='OMC 11HS12-0674S; 28.2 square x31.5, shaft5x20; M2.5 depth4 min')
    rotor=cyl(2.5,20,0,cy,78).cut(box(7,4,15,x=0,y=cy+4,z=83))
    addside(side,'H15_rotor_shaft',rotor,kind='buy',color='metal',material='steel',spec='Included rotor shaft of H15 motor - do not order separately')
    for teeth,y,z in [(32,0,81.6),(20,cy,82.)]:
        spec='RobotDigg GT2-20T-5B-6; bore5; OD18; L17.5; groove7.5; hub8; 2x M3 at90deg, screws included' if teeth==20 else 'MISUMI GPA32GT2060-A-P5; bore5; W10.3; groove7; M3x3 screws included'
        addside(side,f'H16_2GT_pulley_{teeth}T',pulley(teeth,z,0,y),kind='buy',color='metal',material='aluminum',spec=spec,note='Set BOTH groove centre planes to Z86.75 mm. Different bottom heights are intentional.')
    addside(side,'H17_timing_belt_220',timing_belt(),kind='buy',color='belt',material='rubber',spec='2GT endless timing belt 220mm pitch length x6mm wide, 110 teeth')

def build_covers(side):
    wall=cq.Workplane('XY').polyline([(26,20),(14,149),(16.5,149),(28.5,20)]).close().extrude(83).val().translate((0,0,-4))
    for tag,z,seat,az,nz in [('lower',-4,-4,(0,0,1),1.2),('upper',77,79,(0,0,-1),70.5)]:
        for yy,xx in [(45,16),(100,12)]:
            wall=wall.fuse(box(14,10,2,x=xx+2,y=yy,z=z)).cut(cyl(1.7,6,xx,yy,z-2))
            side_fast(side,f'C13_{tag}_cover_bolt_{yy}',3,10,(xx,yy,seat),az);side_nut(side,f'C13_{tag}_cover_nut_{yy}',3,(xx,yy,nz))
    addside(side,'C13_outer_finger_cover',wall,color='cover',print_axis='X' if side>0 else 'negX',note='Removable outer shield. Inner fruit-contact belt surface stays open.')
    roof=cq.Workplane('XY').polyline([(-29,-130),(32,-130),(32,10),(18,18),(-18,18),(-29,10)]).close().extrude(2).val().translate((0,0,102))
    hood=roof.fuse(box(2,148,8,x=-28,y=-56,z=94)).fuse(box(2,148,8,x=31,y=-56,z=94))
    hood=hood.fuse(box(2,72,46,x=-28,y=-94,z=48)).fuse(box(2,72,46,x=31,y=-94,z=48))
    hood=hood.fuse(box(61,2,54,x=1.5,y=-129,z=48))
    # Rear cable exit plus ventilation slots; no electronics are represented.
    hood=hood.cut(box(20,6,23,x=0,y=-129,z=48))
    for z in [77,85]:hood=hood.cut(box(38,6,3,x=1.5,y=-129,z=z))
    for xx,yy in [(24,-38),(-20,-123),(20,-123)]:
        hood=hood.cut(cyl(1.7,4,xx,yy,101))
        side_fast(side,f'C14_service_bolt_{xx}_{yy}',3,12,(xx,yy,104))
    addside(side,'C14_rear_drive_cover',hood,color='cover',print_axis='negZ',note='Remove only three M3x12 hood screws. Deck M4x25 and four slide M3x12 bolts remain fixed. Structural tie saddles retain motor wires.')

def linkage_transform(sh):return sh.rotate((0,0,0),(1,0,0),90).translate((0,-31,-18))
def build_linkage():
    th=theta(P['gap_nominal']);r=25.;L=45.;s=P['gap_nominal']/2+18
    cr=roundbox(64,14,8,r=6).fuse(cyl(12,10))
    cr=cr.cut(cyl(4.06,12,z=-1)).cut(box(.6,15,12,y=7.5,z=-1))
    cr=cr.cut(cyl(1.7,30,-15,8,5,axis=(1,0,0))).cut(cyl(3.1,8,9,8,5,axis=(1,0,0))).cut(axial_hex(5.8,5,(-12,8,5),(1,0,0)))
    for xx in [-25,25]:cr=cr.cut(cyl(2.7,10,xx,0,-1)).cut(hexprism(8.4,4.3,xx,0,4))
    def tf(sh):return linkage_transform(sh.rotate((0,0,0),(0,0,1),math.degrees(th)))
    add('K01_opening_crank',tf(cr),group='crank',color='accent',print_axis='Y',motion={'pivot':[0,-31,-18],'angle0':th},note='25 mm radius. Independent Ø8 bearing shaft. Split hub clamp; links in front of rear rail.')
    hw('K01_hub_bolt',tf(orient(screw_shape(3,20),(-1,0,0)).translate((10,8,5))),'crank','ISO 4762 M3x20')
    hw('K01_hub_nut',tf(orient(nut_shape(3),(1,0,0)).translate((-9.4,8,5))),'crank','ISO 4032 M3')
    for sign in [-1,1]:
        a=np.array([sign*r*math.cos(th),0.,sign*r*math.sin(th)]);b=np.array([sign*s,0,0.])
        angle=math.degrees(math.atan2(b[2]-a[2],b[0]-a[0]))
        rod=roundbox(45,10,8,x=22.5,z=-10,r=4).fuse(cyl(9,8,z=-10)).fuse(cyl(9,8,x=45,z=-10))
        rod=holes(rod,[(0,0),(45,0)],10.2,-11,10)
        rod=linkage_transform(rod.rotate((0,0,0),(0,0,1),angle)).translate(tuple(a))
        a[1]=-31;a[2]-=18;b[1]=-31;b[2]-=18
        grp='linkR' if sign>0 else 'linkL'
        add(f'K02_link_{sign}',rod,group=grp,color='plate',print_axis='Y',motion={'a0':a.tolist(),'b0':b.tolist()},note='45 mm pin spacing. Replaceable 10 OD /6.15 ID bushes. 1 mm axial float on each link face.')
        for j,pv in enumerate([a,b]):
            x,y,z=pv;pg='crank' if j==0 else ('R' if sign>0 else 'L')
            relief=P['pivot_thread_relief']
            shoulder=cyl(3,12,x,-31,z,axis=(0,1,0)).fuse(cyl(5,4.5,x,-19,z,axis=(0,1,0)))
            shoulder=shoulder.fuse(cyl(2.5,9.5-relief,x,-40.5,z,axis=(0,1,0))).fuse(cyl(1.9,relief,x,-31-relief,z,axis=(0,1,0)))
            hw(f'K03_shoulder_{sign}_{j}',shoulder,pg,'Ganter ISO7379-6-M5-12 shoulder Ø6x12, M5x9.5',note='M5 thread relief is modeled conservatively as2mm. Use the specified Ganter Ø6 variant, not the ISO nominal Ø6.5 substitution. Check actual relief and complete nut engagement before assembly.')
            if j==0:hw(f'K03_nut_{sign}_{j}',tf(nut_shape(5).translate((sign*25,0,4))),pg,'ISO 4032 M5')
            else:nut(f'K03_nut_{sign}_{j}',5,(x,P['pivot_saddle_nut_y'],z),(0,1,0),group=pg)
            washer(f'K03_rear_washer_{sign}_{j}',6,1,(x,-31,z),(0,1,0),group=pg)
            washer(f'K03_front_washer_{sign}_{j}',6,1,(x,-20,z),(0,1,0),group=pg)
            add(f'K04_bush_{sign}_{j}',axial_tube(5,3.075,10,(x,-30,z),(0,1,0)),group=pg,color='poly',print_axis='Y')

def build_tray():
    # Left-moving, sloped receiver with rear outlet and a closed front end.
    cx=-48.;cy=-16.;cz=-20.;L=194.
    sh=cyl(73,L,cx,cy,cz,axis=(0,1,0)).cut(cyl(70,L+2,cx,cy-1,cz,axis=(0,1,0)))
    sh=sh.fuse(cyl(73,3,cx,cy+L-3,cz,axis=(0,1,0)))
    sh=sh.rotate((cx,cy,cz),(cx+1,cy,cz),10).intersect(box(160,220,150,x=cx,y=88,z=-170))
    for x,y in [(14,32),(10,130)]:
        post=box(8,12,58,x=x,y=y,z=-60)
        # Keep post exterior, drilling and nut-loading windows open below the deck.
        post=post.cut(cyl(1.7,25,x,y,-22)).cut(hexprism(5.8,2.6,x,y,-11))
        post=post.cut(box(8,7,2.6,x=x+4,y=y,z=-11))
        sh=sh.fuse(post)
        side_fast(-1,f'T01_post_bolt_{y}',3,16,(x,y,3));side_nut(-1,f'T01_post_nut_{y}',3,(x,y,-11))
    addside(-1,'T01_sloped_receiver',sh,color='tray',print_axis='pipe',note='Left cassette mounted; 10 degree downhill toward rear outlet. Front closed. Printed front-cap down. Fruit-flow tests still required.')

def build_homing():
    # D2F-01F straight terminals. Datum is mounting-hole centre, NOT bottom face.
    # X93.8 minus OP5.5 -> face88.3, nominal gap95.6; final0.2mm switch travel.
    br=box(5,5,33.5,x=101.5,y=-46.5,z=12).fuse(box(5,18,2.5,x=101.5,y=-40,z=43))
    br=br.fuse(box(14.5,17,2.5,x=96.75,y=-40.5,z=43))
    for zz in [14,22]:
        br=br.cut(cyl(1.7,8,101,-51,zz,axis=(0,1,0)))
        fast(f'F08_mount_bolt_{zz}',3,16,(101,-44,zz),(0,-1,0));nut(f'F08_mount_nut_{zz}',3,(101,-57,zz),(0,1,0))
    sw=box(6.5,12.8,5.8,x=92.05,y=-40.6,z=45.5)
    for yy in [-43.85,-37.35]:
        sw=sw.cut(cyl(1.05,8,93.8,yy,44.5))
        cut=cq.Workplane('XY').center(93.8,yy).slot2D(3.8,2.2,0).extrude(5).val().translate((0,0,42))
        br=br.cut(cut)
        washer(f'F08_sensor_washer_{yy}',2,.5,(93.8,yy,51.3))
        fast(f'F08_sensor_bolt_{yy}',2,12,(93.8,yy,51.8));nut(f'F08_sensor_nut_{yy}',2,(93.8,yy,41.4))
    for yy in [-45.68,-40.6,-35.52]:sw=sw.fuse(box(3.5,.9,.4,x=97.05,y=yy,z=48.2))
    add('F08_adjustable_home_bracket',br,color='accent',note='D2F-01F mount above rail. Calibrate measured transition at gap95.6; stop and retreat. Hard stop96 leaves0.2 nominal post-trip travel; verify actual switch overtravel before power. Slots +/-0.8 mm.')
    add('H18_home_switch_D2F01F',sw,kind='buy',color='motor',material='switch',spec='Omron D2F-01F, pin plunger, PCB straight terminals; body and 3 terminals; actuator separate CAD instance')
    add('H19_home_switch_actuator',box(1,1.2,2.9,x=88.3,y=-42.55,z=46.95),kind='buy',color='poly',material='switch',spec='Included actuator of H18 D2F-01F - do not order separately')

def build_adapter():
    # Custom fixture interface, not an asserted robot-flange standard.
    ad=axial_tube(37,13,8,(0,-194,-18),(0,1,0))
    for x in [-78,78]:
        # Lofted/straight diagonal ribs stay outside the central motor.
        points=[(x-5,-57),(x+5,-57),((30 if x>0 else -30)+5,-186),((30 if x>0 else -30)-5,-186)]
        rib=cq.Workplane('XY').polyline(points).close().extrude(14).val().translate((0,0,15))
        end=box(14,12,38,x=x,y=-63,z=-5)
        ad=ad.fuse(rib).fuse(end)
        # Vertical return joins the rim to the diagonal ribs.
        ad=ad.fuse(box(12,10,44,x=30 if x>0 else -30,y=-189,z=-18))
        for z in [2,12]:
            ad=ad.cut(cyl(2.2,17,x,-72,z,axis=(0,1,0))).cut(axial_hex(7.4,3.4,(x,-69,z),(0,1,0)))
            fast(f'A01_bolt_{x}_{z}',4,22,(x,-49,z),(0,-1,0));nut(f'A01_nut_{x}_{z}',4,(x,-69,z),(0,1,0))
    for a in [45,135,225,315]:
        x=25*math.cos(math.radians(a));z=-18+25*math.sin(math.radians(a))
        ad=ad.cut(cyl(3.3,12,x,-196,z,axis=(0,1,0)))
    add('A01_rear_fixture_adapter',ad,color='frame',print_axis='Y',note='Custom 4xØ6.6 on 50 PCD, Ø26 centre opening. Host robot plate and its fasteners are application-specific; not supplied.')

def build_coupons():
    for tag,ds in [('625',[16,16.1,16.15,16.2,16.3]),('shaft5',[5,5.1,5.15,5.2,5.3]),('608',[22,22.1,22.15,22.2,22.3]),('pivot6',[6,6.1,6.15,6.2,6.3])]:
        sh=roundbox(142,30,8,r=2)
        for x,d in zip([-54,-27,0,27,54],ds):sh=sh.cut(cyl(d/2,10,x,0,-1))
        coupons.append(dict(name='FIT_'+tag,shape=sh,note='Increasing X hole diameters: '+str(ds)))
    sh=roundbox(90,30,8,r=2)
    for x,m in [(-28,3),(0,4),(28,5)]:sh=sh.cut(cyl((m+.4)/2,10,x,0,-1)).cut(hexprism({3:5.8,4:7.4,5:8.4}[m],4.3,x,0,0))
    coupons.append(dict(name='FIT_nut_pockets',shape=sh,note='M3, M4, M5 nut-pocket and clearance-hole coupon.'))

def build():
    parts.clear();coupons.clear()
    build_frame();print('frame',len(parts),flush=True)
    for side in [-1,1]:build_cassette(side);print('cassette',side,len(parts),flush=True)
    build_linkage();build_tray();build_adapter();build_homing();build_coupons()
    for side in [-1,1]:build_covers(side)

def shape_at(pt,g):
    sh=pt['shape'];gr=pt['group'];
    if pt['name']=='H19_home_switch_actuator':
        travel=max(0.,g/2+40.5-87.8)
        return sh.cut(box(travel,4,5,x=87.8+travel/2,y=-42.55,z=45.5)) if travel>0 else sh
    s0=P['gap_nominal']/2+18;s=g/2+18
    if gr in ['L','R']:return sh.translate(((1 if gr=='R' else -1)*(s-s0),0,0))
    if gr=='crank':return sh.rotate((0,-31,-18),(0,-32,-18),math.degrees(theta(g)-theta(P['gap_nominal'])))
    if gr in ['linkR','linkL']:
        sign=1 if gr=='linkR' else -1
        a0=np.array(pt['motion']['a0']);b0=np.array(pt['motion']['b0'])
        a=np.array([sign*25*math.cos(theta(g)),-31,-18+sign*25*math.sin(theta(g))]);b=np.array([sign*s,-31,-18])
        ang0=math.atan2((b0-a0)[2],(b0-a0)[0]);ang=math.atan2((b-a)[2],(b-a)[0])
        return sh.translate(tuple(-a0)).rotate((0,0,0),(0,-1,0),math.degrees(ang-ang0)).translate(tuple(a))
    return sh

def print_shape(p):
    sh=p.get('print_override',p['shape'])
    axis=p.get('print_axis','Z')
    if axis=='X':sh=sh.rotate((0,0,0),(0,1,0),90)
    if axis=='negX':sh=sh.rotate((0,0,0),(0,1,0),-90)
    if axis=='negZ':sh=sh.rotate((0,0,0),(1,0,0),180)
    if axis=='pipe':sh=sh.rotate((0,0,0),(1,0,0),-100)
    if axis=='Y':sh=sh.rotate((0,0,0),(1,0,0),90)
    bb=sh.BoundingBox();return sh.translate((-bb.xmin,-bb.ymin,-bb.zmin))

def export_all():
    import trimesh
    for d in ['models','stl','step_parts','checks']: (ROOT/d).mkdir(exist_ok=True)
    ass=cq.Assembly(name='V5_original_form_assembly')
    sc=trimesh.Scene(); Q=trimesh.transformations.rotation_matrix(-math.pi/2,[1,0,0]);sc.graph.update(frame_to='CAD_mm',matrix=Q)
    records=[];print_records=[]
    for i,p in enumerate(parts):
        sh=p['shape'];rgb=tuple(int(p['color'][k:k+2],16)/255 for k in (1,3,5))
        ass.add(sh,name=p['name'],color=cq.Color(*rgb))
        vv,ff=sh.tessellate(.15,.16)
        mesh=trimesh.Trimesh(np.array([[v.x,v.y,v.z] for v in vv])*.001,np.array(ff),process=True)
        mesh.visual=trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(baseColorFactor=[int(c*255) for c in rgb]+[255],metallicFactor=.55 if p['kind']=='buy' else .05,roughnessFactor=.48))
        sc.add_geometry(mesh,node_name=p['name'],geom_name=p['name'],parent_node_name='CAD_mm')
        rec={k:v for k,v in p.items() if k not in ['shape','print_override']}
        bb=sh.BoundingBox();rec.update(valid=sh.isValid(),solid_count=len(sh.Solids()),volume_mm3=round(sh.Volume(),3),bbox_mm=[round(bb.xlen,3),round(bb.ylen,3),round(bb.zlen,3)],centre=[bb.center.x,bb.center.y,bb.center.z])
        if p['kind']=='print':
            ps=print_shape(p);pb=ps.BoundingBox();rec['print_bbox_mm']=[pb.xlen,pb.ylen,pb.zlen]
            cq.exporters.export(ps,str(ROOT/'stl'/f"{p['name']}.stl"),tolerance=.035,angularTolerance=.12)
            sh.exportStep(str(ROOT/'step_parts'/f"{p['name']}.step"),write_pcurves=False)
            print_records.append(rec)
        records.append(rec)
        if i%50==0:print('export',i,len(parts),flush=True)
    for cp in coupons:
        cq.exporters.export(print_shape(cp),str(ROOT/'stl'/f"{cp['name']}.stl"),tolerance=.035,angularTolerance=.12)
        cp['shape'].exportStep(str(ROOT/'step_parts'/f"{cp['name']}.step"),write_pcurves=False)
    from cadquery.occ_impl.exporters.assembly import exportAssembly
    tmp=ROOT/'assembly.step'
    if not exportAssembly(ass,str(tmp),write_pcurves=False):raise RuntimeError('Assembly STEP write failed')
    tmp.replace(ROOT/'models/inaho_gripper_v5_assembly.step')
    sc.export(str(ROOT/'models/inaho_gripper_v5_assembly.glb'))
    for group in ['L','R','fixed','linkage']:
        sub=cq.Assembly(name='V5_'+group)
        for p in parts:
            if (p['group']==group) or (group=='linkage' and p['group'] in ['crank','linkR','linkL']):sub.add(p['shape'],name=p['name'])
        if not exportAssembly(sub,str(ROOT/'models'/f'V5_{group}.step'),write_pcurves=False):raise RuntimeError('Subassembly STEP failed: '+group)
    (ROOT/'parts_manifest.json').write_text(json.dumps(records,indent=2,ensure_ascii=False))
    (ROOT/'parameters.json').write_text(json.dumps(P,indent=2))
    with (ROOT/'docs/PRINT_LIST.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['file','qty','material','print_X_mm','print_Y_mm','print_Z_mm','note'])
        for p in print_records:w.writerow([p['name']+'.stl',1,p['material'],*map(lambda a:round(a,2),p['print_bbox_mm']),p['note']])
        for p in coupons:w.writerow([p['name']+'.stl',1,'same material/profile as main parts','','','',p['note']])
    from collections import Counter,defaultdict
    counts=Counter(p['spec'] for p in parts if p['kind']=='buy' and not p['spec'].startswith('Included'));names=defaultdict(list)
    for p in parts:
        if p['kind']=='buy' and not p['spec'].startswith('Included'):names[p['spec']].append(p['name'])
    with (ROOT/'docs/PURCHASE_BOM.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['specification','quantity','instance IDs'])
        for k,v in sorted(counts.items()):w.writerow([k,v,'; '.join(names[k])])
    print('exported',len(parts),'instances;',len(print_records),'printed instances +',len(coupons),'coupons',flush=True)
if __name__=='__main__':
    build()
    print('BUILT',len(parts),flush=True)
    if '--export' in sys.argv:export_all()
