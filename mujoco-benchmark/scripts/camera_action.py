"""Camera-axis action encoding; target-centred positions; no rendering imports."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

SCHEMA='farmily_camera_action_v2'
PHASES=['preapproach','entry','insert','rise']
DIAGONAL_PHASES=['preapproach','entry','insert','rise_mid','rise']

def motion_phases(waypoints):
    names=[w['phase'] for w in waypoints]
    if names not in (PHASES,DIAGONAL_PHASES):raise ValueError('Unsupported or unordered trajectory phases')
    return names

FEATURE_NAMES=['approach_x','approach_y','approach_z','hook_qx','hook_qy','hook_qz','hook_qw','entry_clearance_m','lateral_offset_m','insertion_distance_m','lift_x','lift_y','lift_z','lift_distance_m']
URDF=Path(__file__).resolve().parents[2]/'nvidia-sim/robot_usd/rb5_farmily.urdf'


def optical_transform(stream):
    root=ET.parse(URDF).getroot();parents={j.find('child').get('link'):j for j in root.findall('joint')}
    def chain(link):
        if link=='tomato_gripper':return np.eye(4)
        joint=parents[link]
        if joint.get('type')!='fixed':raise ValueError('Camera chain must be fixed')
        origin=joint.find('origin');t=np.eye(4)
        if origin is not None:
            t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        return chain(joint.find('parent').get('link'))@t
    return chain('d435_'+stream+'_optical_frame')


def encode(waypoints,parameters,target_world,world_from_camera):
    phases=motion_phases(waypoints)
    points={w['phase']:w for w in waypoints}
    if any(p not in points for p in PHASES):raise ValueError('Four staged6d phases required')
    xyz=np.array([points[p]['position_xyz'] for p in phases],dtype=float)
    rot=Rotation.from_quat([points[p]['orientation_xyzw'] for p in phases]).as_matrix()
    if not np.allclose(rot,rot[0],rtol=0,atol=1e-8):raise ValueError('Compact action requires constant hook orientation')
    camera=np.asarray(world_from_camera,dtype=float);target=np.asarray(target_world,dtype=float)
    if not np.isfinite(camera).all() or not np.isfinite(xyz).all():raise ValueError('Non-finite geometry')
    rc=camera[:3,:3];np.testing.assert_allclose(rc.T@rc,np.eye(3),atol=1e-8)
    np.testing.assert_allclose(np.linalg.det(rc),1,atol=1e-8)
    approach=xyz[2]-xyz[1];insertion=np.linalg.norm(approach);lift=xyz[-1]-xyz[2];distance=np.linalg.norm(lift)
    if min(insertion,distance)<=0:raise ValueError('Zero motion segment')
    approach=rc.T@(approach/insertion);lift=rc.T@(lift/distance)
    orientation=Rotation.from_matrix(rc.T@rot[0]).as_quat()
    if orientation[3]<0:orientation=-orientation
    offsets=(xyz-target)@rc
    gravity=rc.T@np.array([0.,0.,-1.])
    # Guard against direction/length or parameter metadata drift.
    np.testing.assert_allclose(insertion,parameters['insertion_distance_m'],atol=1e-8)
    np.testing.assert_allclose(distance,parameters['lift_distance_m'],atol=1e-8)
    np.testing.assert_allclose(rc@approach,-rot[0,:,0],atol=1e-8)
    np.testing.assert_allclose(offsets@rc.T+target,xyz,atol=1e-10)
    vector=np.r_[approach,orientation,parameters['entry_clearance_m'],parameters['lateral_offset_m'],insertion,lift,distance]
    return dict(schema=SCHEMA,coordinate_frame='observation_color_optical',axis_convention='x right, y down, z forward',position_origin='initial target fruit center',approach_direction_camera=approach.tolist(),hook_orientation_camera_xyzw=orientation.tolist(),lift_direction_camera=lift.tolist(),gravity_direction_camera=gravity.tolist(),entry_clearance_m=float(parameters['entry_clearance_m']),lateral_offset_m=float(parameters['lateral_offset_m']),insertion_distance_m=float(insertion),lift_distance_m=float(distance),feature_names=FEATURE_NAMES,features=vector.tolist(),target_relative_waypoint_xyz_camera=offsets.tolist(),phase_names=phases,absolute_start_position_is_model_input=False)


def pack_actions(actions):
    """Validate camera-independent geometry once, in batches."""
    chosen=[a for a in actions if len(a.get('waypoints_world',a.get('waypoints',[])))in (4,5)]
    if not chosen:raise ValueError('No planned actions to encode')
    paths=[]
    phases=motion_phases(chosen[0].get('waypoints_world',chosen[0].get('waypoints',[])))
    for a in chosen:
        source=a.get('waypoints_world',a.get('waypoints',[]))
        if motion_phases(source)!=phases:raise ValueError('Do not mix vertical and diagonal trajectories in one action batch')
        w={w['phase']:w for w in source}
        paths.append([w[p] for p in phases])
    xyz=np.array([[w['position_xyz'] for w in row] for row in paths],dtype=float)
    rotations=Rotation.from_quat(np.array([[w['orientation_xyzw'] for w in row] for row in paths]).reshape(-1,4)).as_matrix().reshape(-1,len(phases),3,3)
    np.testing.assert_allclose(rotations,np.repeat(rotations[:,:1],len(phases),axis=1),rtol=0,atol=1e-8)
    approach=xyz[:,2]-xyz[:,1];lift=xyz[:,-1]-xyz[:,2];insertion=np.linalg.norm(approach,axis=1);distance=np.linalg.norm(lift,axis=1)
    if not np.isfinite(xyz).all() or np.any(insertion<=0) or np.any(distance<=0):raise ValueError('Invalid geometry')
    np.testing.assert_allclose(insertion,[a['parameters']['insertion_distance_m'] for a in chosen],atol=1e-8)
    np.testing.assert_allclose(distance,[a['parameters']['lift_distance_m'] for a in chosen],atol=1e-8)
    approach/=insertion[:,None];lift/=distance[:,None]
    np.testing.assert_allclose(approach,-rotations[:,0,:,0],atol=1e-8)
    return dict(phases=phases,ids=np.array([a['candidate_id'] for a in chosen]),xyz=xyz,rot=rotations[:,0],approach=approach,lift=lift,insertion=insertion,distance=distance,clearance=np.array([a['parameters']['entry_clearance_m'] for a in chosen]),lateral=np.array([a['parameters']['lateral_offset_m'] for a in chosen]))


def batch_arrays(packed,target,pose):
    p=packed;pose=np.asarray(pose);target=np.asarray(target);rc=pose[:3,:3]
    if not np.isfinite(pose).all() or not np.isfinite(target).all():raise ValueError('Non-finite camera/target')
    np.testing.assert_allclose(rc.T@rc,np.eye(3),atol=1e-8);np.testing.assert_allclose(np.linalg.det(rc),1,atol=1e-8)
    quat=Rotation.from_matrix(rc.T@p['rot']).as_quat();quat[quat[:,3]<0]*=-1
    features=np.c_[p['approach']@rc,quat,p['clearance'],p['lateral'],p['insertion'],p['lift']@rc,p['distance']]
    if not np.isfinite(features).all():raise ValueError('Non-finite action features')
    offsets=(p['xyz']-target)@rc
    # Independently reconstruct every stored position/direction/orientation.
    error=float(np.abs(offsets@rc.T+target-p['xyz']).max())
    if error>1e-10:raise ValueError('Position reconstruction mismatch')
    np.testing.assert_allclose(features[:,:3]@rc.T,p['approach'],atol=1e-9)
    np.testing.assert_allclose(features[:,10:13]@rc.T,p['lift'],atol=1e-9)
    np.testing.assert_allclose(rc@Rotation.from_quat(quat).as_matrix(),p['rot'],atol=1e-9)
    return dict(schema=np.array(SCHEMA),candidate_ids=p['ids'],feature_names=np.array(FEATURE_NAMES),features=features,target_relative_waypoint_xyz_camera=offsets,gravity_direction_camera=rc.T@np.array([0.,0.,-1.]),target_center_camera=(target-pose[:3,3])@rc,phase_names=np.array(p['phases'])),error


def export_batch(path,actions,target,pose):
    arrays,_=batch_arrays(pack_actions(actions),target,pose)
    np.savez_compressed(path,**arrays)
    return len(arrays['candidate_ids'])
