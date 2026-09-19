"""Low-dimensional contact-motion search, independent of ROS and Isaac imports.

The geometric score is a screening surrogate, never proof of harvest. Candidates
must subsequently survive full PhysX rollouts with the original cluster.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from geometry import RING_RADIUS, WIRE_RADIUS

PARAMETER_NAMES = ('yaw_deg', 'below_m', 'insertion_m', 'lateral_m', 'neck_height_offset_m', 'pitch_deg')
BOUNDS = np.array([[-115, 90], [.013, .045], [.0035, .014], [-.018, .018], [-.002, .006], [-18, 18]])


def validate_parameters(params):
    if not isinstance(params,dict):
        raise ValueError('Each contact motion must be a JSON object')
    limits=dict(yaw_deg=(-180,180),below_m=(.005,.15),insertion_m=(.003,.020),
                lateral_m=(-.025,.025),neck_height_offset_m=(-.02,.02),pitch_deg=(-60,60),roll_deg=(-60,60),
                speed_m_s=(.001,.10),contact_speed_m_s=(.001,.03),force_limit_N=(.1,1000.))
    for key,(lo,hi) in limits.items():
        if key in params and (not isinstance(params[key],(int,float)) or not np.isfinite(params[key]) or not lo<=params[key]<=hi):
            raise ValueError(f'{key} must be finite in [{lo}, {hi}]')
    if 'align_stem' in params and not isinstance(params['align_stem'],bool):
        raise ValueError('align_stem must be a boolean')
    unknown=set(params)-set(limits)-{'align_stem','target_fruit','screen_clearance_m'}
    if unknown:
        raise ValueError(f'Unknown contact parameters: {sorted(unknown)}')
    return params


def motion_waypoints(center, neck, params):
    center,neck=np.asarray(center,dtype=float),np.asarray(neck,dtype=float)
    if center.shape!=(3,) or neck.shape!=(3,) or not np.isfinite(np.r_[center,neck]).all() or np.linalg.norm(neck-center)<1e-6:
        raise ValueError('Fruit center and neck must be finite, distinct 3D points')
    yaw=np.deg2rad(params.get('yaw_deg',-20.))
    outward=np.array([np.cos(yaw),np.sin(yaw),0.])
    down=np.array([0.,0.,-1.])
    if params.get('align_stem',True):
        down=-(neck-center)/np.linalg.norm(neck-center)
        outward=outward-down*np.dot(outward,down)
        outward/=np.linalg.norm(outward)
    rotation=Rotation.from_matrix(np.column_stack([outward,down,np.cross(outward,down)]))
    rotation=rotation*Rotation.from_euler('z',params.get('pitch_deg',0.),degrees=True)
    if params.get('align_stem',True):
        rotation=rotation*Rotation.from_euler('x',params.get('roll_deg',0.),degrees=True)
        x,y,z=rotation.as_matrix().T
        shift=params.get('lateral_m',0.)*z
        insertion=params.get('insertion_m',.010)
        below=center+x*insertion+shift+y*params.get('below_m',.030)
        height=-y*params.get('neck_height_offset_m',0.)
        poses=[('preapproach',below+x*.17),('below',below),
               ('insert',center+x*insertion+shift),
               ('rise',neck+x*insertion+shift+height),
               ('pull',neck+x*.050+shift+height)]
        return rotation,poses
    shift=params.get('lateral_m',0.)*np.cross(outward,down)
    lower=params.get('below_m',.016)
    insertion=params.get('insertion_m',.012)
    high=neck[2]+params.get('neck_height_offset_m',0.)
    poses=[('preapproach',center+outward*.17+shift-np.array([0,0,lower])),
           ('below',center+outward*insertion+shift-np.array([0,0,lower])),
           ('insert',center+outward*insertion+shift),
           ('rise',np.array([*(center+outward*insertion+shift)[:2],high])),
           ('pull',np.array([*(neck+outward*.050+shift)[:2],high]))]
    return rotation, poses


def pedicel_gap(ring_position, ring_rotation, center, attachment, fruit_radius):
    """Sampled surface gap between the physical half-ring and pedicel capsule."""
    axis=np.asarray(attachment)-center
    axis/=np.linalg.norm(axis)
    a=np.asarray(center)+axis*fruit_radius
    b=np.asarray(attachment)
    angles=np.linspace(np.pi/2,3*np.pi/2,257)
    points=np.column_stack([np.cos(angles),np.zeros_like(angles),np.sin(angles)])*RING_RADIUS
    points=ring_rotation.apply(points)+ring_position
    v=b-a
    t=np.clip(((points-a)@v)/np.dot(v,v),0,1)
    return float(np.linalg.norm(points-a-t[:,None]*v,axis=1).min()-WIRE_RADIUS-.0015)


def fruit_clearance(center, neck, fruit_centers, radii, params):
    """Minimum capsule-to-sphere surface clearance along the nominal motion."""
    rotation, poses=motion_waypoints(center,neck,params)
    angles=np.linspace(np.pi/2,3*np.pi/2,33)
    ring=np.column_stack([np.cos(angles),np.zeros_like(angles),np.sin(angles)])*RING_RADIUS
    ring=rotation.apply(ring)
    positions=np.concatenate([np.linspace(a[1],b[1],max(4,int(np.linalg.norm(a[1]-b[1])/.002)))
                              for a,b in zip(poses[:-1],poses[1:])])
    a=positions[:,None,None,:]+ring[None,:-1,None,:]
    v=np.diff(ring,axis=0)[None,:,None,:]
    d=fruit_centers[None,None,:,:]-a
    t=np.clip(np.sum(d*v,axis=-1)/np.sum(v*v,axis=-1),0,1)
    distance=np.linalg.norm(d-t[:,:,:,None]*v,axis=-1)-radii[None,None,:]-WIRE_RADIUS
    return float(distance.min())


def propose_candidates(center, neck, fruit_centers, radii, seed=42, count=16, align_stem=False):
    """CEM screening of the motion skeleton; deterministic given seed and scene."""
    rng=np.random.default_rng(seed)
    mean=BOUNDS.mean(axis=1)
    std=(BOUNDS[:,1]-BOUNDS[:,0])*.35
    archive=[]
    for generation in range(4):
        samples=np.clip(rng.normal(mean,std,(80,6)),BOUNDS[:,0],BOUNDS[:,1])
        scored=[]
        for x in samples:
            params=dict(zip(PARAMETER_NAMES,x.tolist()),align_stem=align_stem)
            clearance=fruit_clearance(center,neck,fruit_centers,radii,params)
            # Favor actual fruit clearance; modest tilt/yaw regularization only
            # resolves effectively equal candidates.
            score=clearance-abs(x[5])*.000005
            scored.append((score,params,clearance))
        scored.sort(key=lambda x:x[0],reverse=True)
        elite=np.array([[p[k] for k in PARAMETER_NAMES] for _,p,_ in scored[:12]])
        mean=.25*mean+.75*elite.mean(axis=0)
        std=np.maximum(.25*std+.75*elite.std(axis=0),(BOUNDS[:,1]-BOUNDS[:,0])*.015)
        archive.extend(scored)
    archive.sort(key=lambda x:x[0],reverse=True)
    selected=[]
    for _,params,clearance in archive:
        if all(abs(params['yaw_deg']-p['yaw_deg'])>2 or abs(params['pitch_deg']-p['pitch_deg'])>2
               or abs(params['lateral_m']-p['lateral_m'])>.0015 for p in selected):
            selected.append(dict(params,screen_clearance_m=clearance))
        if len(selected)>=count:
            break
    return selected
