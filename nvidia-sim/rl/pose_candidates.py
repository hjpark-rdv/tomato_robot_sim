"""Deterministic pose-only experiment design and conservative result labels."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from contact_planner import motion_waypoints
from geometry import RING_RADIUS,WIRE_RADIUS

NOMINAL = dict(yaw_deg=0.,below_m=.020,pitch_deg=0.,neck_height_offset_m=.003,
               lateral_m=.0135,insertion_m=.004,align_stem=False)
DEFAULT_LIMITS = dict(target_displacement_m=.020,main_displacement_m=.030,
                      dangerous_non_target_force_N=3.,hold_seconds=1.,gap_m=.0015,
                      stable_relative_motion_m=.002,contact_force_floor_N=.01)
CATEGORIES = ('success_target_hook','non_target_contact','miss',
              'excessive_displacement','ik_or_planning_failure')


def rear_capsule_geometry(capsules,ring_position,ring_rotation):
    angles=np.linspace(3*np.pi/4,5*np.pi/4,129)
    arc=np.column_stack([np.cos(angles),np.zeros_like(angles),np.sin(angles)])*RING_RADIUS
    best=float('inf');seated=False
    for a,b,radius in capsules:
        a,b=ring_rotation.inv().apply(np.array([a,b])-ring_position);v=b-a
        t=np.clip(((arc-a)@v)/max(np.dot(v,v),1e-20),0,1)
        gap=float(np.linalg.norm(arc-a-t[:,None]*v,axis=1).min()-WIRE_RADIUS-radius)
        best=min(best,gap)
        u=np.clip(-a[1]/v[1],0,1) if abs(v[1])>1e-10 else .5
        crossing=a+u*v
        inner_face=(crossing[0]<-.005 and np.linalg.norm(crossing[[0,2]])<=RING_RADIUS-WIRE_RADIUS+.0005)
        seated |= bool(gap<=DEFAULT_LIMITS['gap_m'] and abs(crossing[1])<=radius+WIRE_RADIUS and inner_face)
    return best,seated


def base_candidate():
    return dict(azimuth_deg=0.,elevation_deg=0.,roll_deg=0.,pitch_deg=0.,
                offset_xyz_m=[0.,0.,0.],pre_hook_distance_m=.166,orientation_delta_rpy_deg=[0.,0.,0.])


def coarse_candidates():
    """31 one-factor candidates around the previously attempted Tomato_05 path."""
    candidates=[base_candidate()]
    for key,values in [('azimuth_deg',[-60.,-45.,-30.,-15.,15.,30.,45.,60.]),
                       ('elevation_deg',[-15.,15.]),('roll_deg',[-15.,15.]),
                       ('pitch_deg',[-15.,15.]),('pre_hook_distance_m',[.14,.20])]:
        for value in values:
            p=base_candidate();p[key]=value;candidates.append(p)
    for axis in range(3):
        for delta in [-.020,-.010,.010,.020]:
            p=base_candidate();p['offset_xyz_m'][axis]=delta;candidates.append(p)
    # Two modest coupled orientations test interaction without a large grid.
    for sign in [-1,1]:
        p=base_candidate();p.update(azimuth_deg=15.*sign,elevation_deg=10.,roll_deg=10.*sign)
        candidates.append(p)
    for i,p in enumerate(candidates):
        p.update(candidate_id=f'coarse_{i:04d}',stage='coarse',parent_id=None)
    return candidates


def candidate_waypoints(center,neck,params):
    rotation,poses=motion_waypoints(center,neck,NOMINAL)
    delta=Rotation.from_euler('z',params['azimuth_deg'],degrees=True)
    # Elevation rotates approach around its horizontal lateral axis; roll
    # rotates the aperture around approach X. These have distinct geometry.
    delta=delta*Rotation.from_euler('y',-params['elevation_deg'],degrees=True)
    orientation=delta*rotation*Rotation.from_euler('xz',[params['roll_deg'],params['pitch_deg']],degrees=True)
    orientation=orientation*Rotation.from_euler('xyz',params.get('orientation_delta_rpy_deg',[0.,0.,0.]),degrees=True)
    center=np.asarray(center);offset=np.asarray(params['offset_xyz_m'])
    transformed=[(name,center+delta.apply(point-center)+offset) for name,point in poses]
    outward=delta.apply([1.,0.,0.])
    transformed[0]=('preapproach',transformed[1][1]+outward*params['pre_hook_distance_m'])
    return orientation,transformed,-outward


def tolerance_candidates(parent):
    result=[]
    for kind,amounts in [('position',[.002,.005,.010]),('orientation',[2.,5.,10.])]:
        for amount in amounts:
            for axis in range(3):
                for sign in [-1,1]:
                    p=copy.deepcopy(parent)
                    if kind=='position': p['offset_xyz_m'][axis]+=sign*amount
                    else: p['orientation_delta_rpy_deg'][axis]+=sign*amount
                    p.update(candidate_id=f"{parent['candidate_id']}_tol_{kind}_{amount:g}_{axis}_{sign:+d}",
                        stage='tolerance',parent_id=parent['candidate_id'],perturbation_kind=kind,
                        perturbation_magnitude=amount,perturbation_axis=axis,perturbation_sign=sign)
                    result.append(p)
    return result


def classify(metrics,limits=DEFAULT_LIMITS):
    events=[]
    if metrics.get('planning_failure'): events.append('ik_or_planning_failure')
    if metrics.get('first_contact_non_target') or metrics.get('max_non_target_force_N',0)>limits['dangerous_non_target_force_N']:
        events.append('non_target_contact')
    if (metrics.get('target_max_displacement_m',0)>limits['target_displacement_m'] or
        metrics.get('main_stem_max_displacement_m',0)>limits['main_displacement_m']):
        events.append('excessive_displacement')
    success=(metrics.get('retained_hook',False) and not events and
             not metrics.get('target_broken',False) and not metrics.get('other_broken',False))
    if success: events.append('success_target_hook')
    elif 'ik_or_planning_failure' not in events: events.append('miss')
    primary=next(k for k in ('ik_or_planning_failure','excessive_displacement','non_target_contact','success_target_hook','miss') if k in events)
    return dict(result=primary,events=events,hook_success=bool(success))
