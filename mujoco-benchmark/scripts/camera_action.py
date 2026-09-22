"""Camera-axis action encoding; target-centred positions; no rendering imports."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

SCHEMA='farmily_camera_action_v2'
PHASES=['preapproach','entry','insert','rise']
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
    points={w['phase']:w for w in waypoints}
    if any(p not in points for p in PHASES):raise ValueError('Four staged6d phases required')
    xyz=np.array([points[p]['position_xyz'] for p in PHASES],dtype=float)
    rot=Rotation.from_quat([points[p]['orientation_xyzw'] for p in PHASES]).as_matrix()
    if not np.allclose(rot,rot[0],rtol=0,atol=1e-8):raise ValueError('Compact action requires constant hook orientation')
    camera=np.asarray(world_from_camera,dtype=float);target=np.asarray(target_world,dtype=float)
    if not np.isfinite(camera).all() or not np.isfinite(xyz).all():raise ValueError('Non-finite geometry')
    rc=camera[:3,:3];np.testing.assert_allclose(rc.T@rc,np.eye(3),atol=1e-8)
    np.testing.assert_allclose(np.linalg.det(rc),1,atol=1e-8)
    approach=xyz[2]-xyz[1];insertion=np.linalg.norm(approach);lift=xyz[3]-xyz[2];distance=np.linalg.norm(lift)
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
    return dict(schema=SCHEMA,coordinate_frame='observation_color_optical',axis_convention='x right, y down, z forward',position_origin='initial target fruit center',approach_direction_camera=approach.tolist(),hook_orientation_camera_xyzw=orientation.tolist(),lift_direction_camera=lift.tolist(),gravity_direction_camera=gravity.tolist(),entry_clearance_m=float(parameters['entry_clearance_m']),lateral_offset_m=float(parameters['lateral_offset_m']),insertion_distance_m=float(insertion),lift_distance_m=float(distance),feature_names=FEATURE_NAMES,features=vector.tolist(),target_relative_waypoint_xyz_camera=offsets.tolist(),phase_names=PHASES,absolute_start_position_is_model_input=False)


def export_batch(path,actions,target,pose):
    encoded=[];ids=[]
    for a in actions:
        w=a.get('waypoints_world',a.get('waypoints',[]))
        if len(w)!=4:continue
        encoded.append(encode(w,a['parameters'],target,pose));ids.append(a['candidate_id'])
    if not encoded:raise ValueError('No planned actions to encode')
    np.savez_compressed(path,schema=np.array(SCHEMA),candidate_ids=np.array(ids),feature_names=np.array(FEATURE_NAMES),features=np.array([e['features'] for e in encoded]),target_relative_waypoint_xyz_camera=np.array([e['target_relative_waypoint_xyz_camera'] for e in encoded]),gravity_direction_camera=np.array(encoded[0]['gravity_direction_camera']),target_center_camera=(np.asarray(target)-np.asarray(pose)[:3,3])@np.asarray(pose)[:3,:3],phase_names=np.array(PHASES))
    return len(ids)
