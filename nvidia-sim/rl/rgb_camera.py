"""Calibrated tool-mounted RGB candidates; no plant state or depth inputs."""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from geometry import RING_CENTER

WIDTH, HEIGHT = 960, 720


class JointCameraModel:
    """URDF FK in robot-base coordinates, usable offline or with real joints."""
    def __init__(self, joint_names, profile):
        root = ET.parse(Path(__file__).parent.parent/'robot_usd/rb5_farmily.urdf').getroot()
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], 'tomato_gripper'
        while link in parents:
            joint = parents[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        self.chain = []
        self.count = len(joint_names)
        self.profile = profile
        for joint in reversed(chain):
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3,3] = np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                transform[:3,:3] = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            axis = joint.find('axis')
            axis = np.fromstring(axis.get('xyz'),sep=' ') if axis is not None else np.zeros(3)
            name = joint.get('name')
            self.chain.append((transform,axis,joint.get('type'),joint_names.index(name) if name in joint_names else None))

    def pose(self, joints):
        joints = np.asarray(joints)
        if joints.shape != (self.count,) or not np.isfinite(joints).all():
            raise ValueError('Invalid joint observation')
        transform = np.eye(4)
        for origin,axis,kind,index in self.chain:
            transform = transform@origin
            if index is not None:
                if kind == 'prismatic':
                    transform[:3,3] += transform[:3,:3]@(axis*joints[index])
                else:
                    transform[:3,:3] = transform[:3,:3]@Rotation.from_rotvec(axis*joints[index]).as_matrix()
        return transform@self.profile['tool_from_camera']


def optical_look_at(eye, target, up=(0., -1., 0.)):
    eye, target, up = map(lambda x: np.asarray(x, dtype=float), (eye, target, up))
    forward = target-eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((right, down, forward))
    pose[:3, 3] = eye
    return pose


def camera_profiles():
    root = ET.parse(Path(__file__).parent.parent/'robot_usd/rb5_farmily.urdf').getroot()
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}

    def transform(link):
        if link == 'tomato_gripper':
            return np.eye(4)
        joint = parents[link]
        origin = joint.find('origin')
        local = np.eye(4)
        if origin is not None:
            local[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            local[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        return transform(joint.find('parent').get('link'))@local

    center = np.asarray(RING_CENTER)
    profiles = {'current': dict(tool_from_camera=transform('d435_color_optical_frame'),
                                hfov_deg=69.4, near_m=.10)}
    for name, side in [('left', -1), ('right', 1)]:
        profiles[name] = dict(tool_from_camera=optical_look_at(
            center+[.09, -.065, side*.11], center+[-.055, -.008, 0]),
            hfov_deg=60., near_m=.01)
    return profiles


def intrinsics(profile, width=WIDTH, height=HEIGHT):
    focal = width/(2*np.tan(np.deg2rad(profile['hfov_deg']/2)))
    return np.array([[focal, 0, width/2], [0, focal, height/2], [0, 0, 1.]])


def world_from_camera(kinematics, joints, profile):
    ring, rotation = kinematics.fk(joints)
    tool = np.eye(4)
    tool[:3, :3] = rotation.as_matrix()
    tool[:3, 3] = ring-tool[:3, :3]@np.asarray(RING_CENTER)
    return tool@profile['tool_from_camera']


def project(points, pose, k):
    camera = (np.atleast_2d(points)-pose[:3, 3])@pose[:3, :3]
    pixels = camera@k.T
    return pixels[:, :2]/pixels[:, 2:], camera[:, 2]


class ToolRGBViews:
    """RGB-only annotators, mounted using FK and fixed extrinsic calibration.

    Candidate housings/brackets are not added to physics: optical placement
    screening only, not a validated mechanical mounting design.
    """
    def __init__(self, env, kinematics, names):
        import omni.replicator.core as rep
        from pxr import UsdGeom, Gf
        self.env, self.kin = env, kinematics
        self.profiles = {name: camera_profiles()[name] for name in names}
        self.views = {}
        for name, profile in self.profiles.items():
            camera = UsdGeom.Camera.Define(env.stage, '/World/ToolRGB_'+name)
            camera.CreateHorizontalApertureAttr(20.955)
            camera.CreateVerticalApertureAttr(20.955*HEIGHT/WIDTH)
            camera.CreateFocalLengthAttr(20.955/(2*np.tan(np.deg2rad(profile['hfov_deg']/2))))
            camera.CreateClippingRangeAttr(Gf.Vec2f(profile['near_m'], 20.))
            op = UsdGeom.Xformable(camera).MakeMatrixXform()
            product = rep.create.render_product(str(camera.GetPath()), (WIDTH, HEIGHT))
            rgb = rep.AnnotatorRegistry.get_annotator('rgb')
            rgb.attach(product)
            self.views[name] = (op, product, rgb)
        self.update_poses(env.robot.data.joint_pos[0].cpu().numpy())
        for _ in range(8):
            env.sim.render()

    def update_poses(self, joints):
        from pxr import Gf
        for name, (op, _, _) in self.views.items():
            pose = world_from_camera(self.kin, joints, self.profiles[name])
            pose[:3, 1:3] *= -1  # optical +Z/+Y-down -> USD -Z/+Y-up
            op.Set(Gf.Matrix4d(*pose.T.flatten().tolist()))

    def read(self, joints):
        from omni.physx import get_physx_interface
        self.update_poses(joints)
        self.env.elastic.sync_visuals()
        self.env.sim.physics_sim_view.update_articulations_kinematic()
        get_physx_interface().update_transformations(False, True, True)
        self.env.sim.render()
        result = {name: rgb.get_data()[:, :, :3].copy() for name, (_, _, rgb) in self.views.items()}
        if any(image.shape != (HEIGHT, WIDTH, 3) for image in result.values()):
            raise RuntimeError('RGB camera returned an invalid frame')
        return result

    def close(self):
        for _, product, rgb in self.views.values():
            rgb.detach(product)
        for _, product, _ in self.views.values():
            product.destroy()
