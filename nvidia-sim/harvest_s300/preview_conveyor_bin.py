"""Flush 200mm conveyor bin geometry shared by preview and unloading fixture."""
from __future__ import annotations

import math
import hashlib
import json
from pathlib import Path
import numpy as np


def conveyor_bin_spec(deck_top=.314):
    low=np.array([-.28,-.45,deck_top]);high=np.array([.28,.65,deck_top+.20])
    parts=[]
    def box(name,position,dimensions,color=(45,83,66),**extra):
        parts.append(dict(name=name,position=list(position),dimensions=list(dimensions),color=color,**extra))
    # Continuous side skirts sit directly on the chassis. The original lift
    # anchor toes are enclosed beneath the conveyor instead of adding feet.
    for side,x in enumerate((-.274,.274)):
        box(f'side_{side}',[x,.10,deck_top+.10],[.012,1.10,.20])
        for j,y in enumerate(np.linspace(-.40,.60,8)):
            box(f'side_rib_{side}_{j}',[x,.0+y,deck_top+.10],[.012,.016,.18],(35,68,53))
        box(f'side_rim_{side}',[x,.10,high[2]-.01],[.024,1.10,.02],(28,57,44))
    box('rear_wall_upper',[0,.644,deck_top+.122],[.536,.012,.156])
    # Two narrow lower notches accommodate the existing lift anchor plates.
    for j,(a,b)in enumerate(((-.268,-.17),(-.125,.125),(.17,.268))):
        box(f'rear_skirt_{j}',[(a+b)/2,.644,deck_top+.022],[b-a,.012,.044])
    box('front_drive_frame',[0,-.438,deck_top+.042],[.536,.024,.084],(32,56,45))
    box('undertray',[0,.10,deck_top+.0485],[.536,1.052,.005],(75,83,83))
    belt_top=deck_top+.086
    roller_radius=.016
    roller_z=belt_top-.018
    roller_y=np.linspace(-.437,.622,31)
    # The belt covers the complete usable floor; all transverse support
    # rollers remain visible at its ends/edges and turn in the unloading test.
    # Project beyond the stationary front frame; a short unpowered lip would
    # otherwise retain the last row of fruit after the belt is emptied.
    belt_start=-.455;belt_end=.632;belt_length=belt_end-belt_start
    box('belt',[0,(belt_start+belt_end)/2,belt_top-.001],[.516,belt_length,.002],(46,53,56),surface=True)
    box('roller_envelope',[0,.0925,roller_z],[.516,1.091,.036],(46,53,56),grip_only=True)
    gate_hinge=[0.,-.45,float(high[2])]
    # Only the wall above the belt opens; the drive housing below it remains
    # fixed. A 2mm floor clearance prevents the closed door cutting the belt.
    gate_parts=[dict(name='gate_panel',position=[0.,-.006,-.056],dimensions=[.536,.012,.112],color=(45,83,66))]
    for i,x in enumerate(np.linspace(-.24,.24,5)):
        gate_parts.append(dict(name=f'gate_rib_{i}',position=[float(x),-.015,-.052],
            dimensions=[.016,.010,.094],color=(35,68,53)))
    # Grip checks use the closed door. The private GPU unloading fixture uses
    # its actual hinge and all contacts, without a box filling the bin interior.
    for p in gate_parts:
        world=np.array(p['position'])+gate_hinge
        parts.append(dict(p,name='door/'+p['name'],position=world.tolist(),gate=True))
    return dict(bounds=[low.tolist(),high.tolist()],nominal_size_m=[.56,1.10,.20],
        deck_top_m=deck_top,belt_top_m=belt_top,belt_width_m=.516,belt_length_m=belt_length,belt_start_y_m=belt_start,
        roller_radius_m=roller_radius,roller_length_m=.516,roller_z_m=roller_z,
        roller_y_m=roller_y.tolist(),gate_hinge_m=gate_hinge,gate_parts=gate_parts,
        gate_open_angle_rad=-math.radians(110),parts=parts,
        support_feet_count=0,chassis_gap_m=0.,outlet_direction_world=[0.,-1.,0.],
        mechanism='Full-width belt on transverse rollers; upper-hinged rear discharge door')


def conveyor_grip_parts(spec):
    return [dict(p)for p in spec['parts']]


def draw_conveyor(server,spec,name='/conveyor_bin'):
    from scipy.spatial.transform import Rotation
    root=server.scene.add_frame(name,show_axes=False,visible=False)
    for p in spec['parts']:
        if p.get('gate')or p.get('grip_only'):continue
        server.scene.add_box(name+'/'+p['name'],position=p['position'],
            dimensions=p['dimensions'],color=p['color'])
    gate=server.scene.add_frame(name+'/gate',position=spec['gate_hinge_m'],show_axes=False)
    for p in spec['gate_parts']:
        server.scene.add_box(name+'/gate/'+p['name'],position=p['position'],
            dimensions=p['dimensions'],color=p['color'])
    rollers=[]
    for i,y in enumerate(spec['roller_y_m']):
        frame=server.scene.add_frame(name+f'/roller_{i}',position=[0,y,spec['roller_z_m']],show_axes=False)
        quat=Rotation.from_euler('y',90,degrees=True).as_quat()[[3,0,1,2]]
        server.scene.add_cylinder(name+f'/roller_{i}/shaft',radius=spec['roller_radius_m'],
            height=spec['roller_length_m']+.012,wxyz=quat,color=(145,157,160))
        # A small axle mark makes actual rotation visible from the side.
        server.scene.add_box(name+f'/roller_{i}/mark',position=[-.262,0,.012],
            dimensions=[.006,.012,.004],color=(238,182,57))
        rollers.append(frame)
    stripes=[]
    for i in range(36):
        stripes.append(server.scene.add_box(name+f'/belt_rib_{i}',
            position=[0,spec['belt_start_y_m']+i*spec['belt_length_m']/36,spec['belt_top_m']+.0007],
            dimensions=[.510,.004,.0014],color=(73,81,84)))
    return dict(root=root,gate=gate,rollers=rollers,stripes=stripes)


def update_conveyor(handles,spec,gate_angle,roller_angles,belt_distance):
    from scipy.spatial.transform import Rotation
    handles['gate'].wxyz=Rotation.from_euler('x',gate_angle).as_quat()[[3,0,1,2]]
    for frame,angle in zip(handles['rollers'],roller_angles):
        frame.wxyz=Rotation.from_euler('x',float(angle)).as_quat()[[3,0,1,2]]
    length=spec['belt_length_m']
    for i,stripe in enumerate(handles['stripes']):
        stripe.position=[0,spec['belt_start_y_m']+(i*length/36-belt_distance)%length,spec['belt_top_m']+.0007]


def unload_frame(states,elapsed):
    times=states['time'];t=float(np.clip(elapsed,times[0],times[-1]))
    k=max(0,min(int(np.searchsorted(times,t))-1,len(times)-2))
    u=(t-times[k])/(times[k+1]-times[k])
    return {key:(1-u)*states[key][k]+u*states[key][k+1]
            for key in ('tomato_positions','gate_angle','roller_angles','belt_distance')}


def load_unload_record(path,spec):
    """Only display a completed, physically accepted record for this geometry."""
    path=Path(path)
    report=json.loads(path.read_text())
    if (report.get('schema')!='s300-conveyor-unload-gpu-v1'or
            report.get('engine',{}).get('backend')!='mujoco_warp_cuda'):
        raise ValueError('Expected a Newton GPU unloading record')
    trials=report.get('trials',{})
    if set(trials)!={'belt_off','belt_on'}or not all(t.get('valid')is True for t in trials.values()):
        raise ValueError('Unloading fixture failed physical validity checks')
    if json.loads(json.dumps(spec))!=report['layout']:
        raise ValueError('Unloading fixture differs from displayed conveyor bin')
    state_path=path.parent/report['state_file']
    if hashlib.sha256(state_path.read_bytes()).hexdigest()!=report['state_sha256']:
        raise ValueError('Unloading state SHA mismatch')
    with np.load(state_path,allow_pickle=False)as archive:
        states=dict(archive)
    times=states['time'];n=len(times);fruit_count=trials['belt_on']['fruit_count']
    shapes=dict(tomato_positions=(n,fruit_count,3),gate_angle=(n,),
        roller_angles=(n,len(spec['roller_y_m'])),belt_distance=(n,),discharged_count=(n,))
    if n<2 or not np.isfinite(times).all()or not np.all(np.diff(times)>0):
        raise ValueError('Invalid unloading timestamps')
    for key,shape in shapes.items():
        if states[key].shape!=shape or not np.isfinite(states[key]).all():
            raise ValueError('Invalid unloading state: '+key)
    counts=states['discharged_count']
    if (np.any(np.diff(counts)<0)or counts.min()<0 or
            counts[-1]!=trials['belt_on']['discharged']or counts[-1]>fruit_count):
        raise ValueError('Invalid unloading outlet-crossing counts')
    return report,states
