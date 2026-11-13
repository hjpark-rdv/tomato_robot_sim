"""Operating posture restrictions for V5/V6 kinematic previews.

These are additional working limits, not changes to the manufacturer's limits
or to the production physics model. The mast is its conservative CAD AABB;
clearance is measured against the existing native arm/tool collision proxies.
V6 also keeps the camera above the capture volume and excludes full yaw turns.
"""
from __future__ import annotations

import math
from itertools import product

import mujoco as mj
import numpy as np


POSTURE_POLICY = dict(
    version='v5-mast-clearance-and-open-elbow-v1',
    mast_clearance_m=.10,
    elbow_abs_working_range_deg=[20., 140.],
    wrist1_working_range_deg=[-150., 150.],
    maximum_forearm_upward_deg=10.,
    scope='All stored states and interpolated transfers, insertion, jaws and retreat',
)


def side_posture_policy(scene, camera_mount=None):
    """Bind the V6 camera rule to the actual, unchanged wrist housing CAD."""
    from newton_viser import geom_mesh
    matches = [i for i in range(scene.ngeom) if scene.geom(i).name == 's300_g0092'
               and scene.body(int(scene.geom_bodyid[i])).name == 'Robot_link5']
    if len(matches) != 1:
        raise ValueError('V6 camera housing CAD is missing or has moved to another body')
    vertices = geom_mesh(scene, matches[0]).vertices
    policy = dict(POSTURE_POLICY, version='v6-camera-up-no-yaw-turn-v3',
        camera_body='Robot_link5', camera_geom='s300_g0092',
        camera_bounds_body_m=[vertices.min(0).tolist(), vertices.max(0).tolist()],
        grip_body='Tool_fixed', grip_center_body_m=[0., .079, .037],
        grip_half_size_body_m=[.048, .079, .042],
        minimum_camera_above_grip_m=.01,
        wrist3_working_range_deg=[-170., 170.],
        grip_yaw_working_range_deg=[-30., 210.],
        minimum_grip_forward_horizontal=.25,
        yaw_rule='Unwrapped world heading of tool +Y stays in [-30, 210] degrees; right 0, parked 90, left 180. The excluded rear sector and nonvertical forward axis prohibit a full yaw turn throughout transfers and replay. Wrist3 stays within +/-170 degrees without wrapping.',
        camera_rule='Lowest point of conservative camera housing box stays at least 10 mm above highest point of capture volume, including transfers')
    if camera_mount is not None:
        policy.update(version=('v6-recessed-camera-no-yaw-turn-v5' if camera_mount.get('cover_reliefs')
                               else 'v6-central-camera-no-yaw-turn-v4'),
            camera_body=camera_mount['body'],
            camera_bounds_body_m=camera_mount['housing_bounds_body_m'],
            camera_height_reference='capture_center',
            camera_mount=camera_mount,
            camera_collision_clearance_m=.002,
            camera_rule='Central bracket mount overlaps the upper capture volume as requested; lowest camera housing point stays at least 10 mm above the capture centre throughout movement')
    return policy


class PostureGuard:
    def __init__(self, model, obstacles, *, policy=None):
        self.model = model
        self.policy = POSTURE_POLICY if policy is None else policy
        self.camera_id = None
        self.yaw_wrist = (model.joint('wrist3').id
                          if 'wrist3_working_range_deg' in self.policy else None)
        if 'minimum_camera_above_grip_m' in self.policy:
            self.camera_id = model.body(self.policy['camera_body']).id
            self.grip_id = model.body(self.policy['grip_body']).id
            low, high = self.policy['camera_bounds_body_m']
            self.camera_corners = np.array(list(product(*zip(low, high))))
            half = np.array(self.policy['grip_half_size_body_m'])
            self.grip_corners = np.array(self.policy['grip_center_body_m']) + np.array(list(product(*zip(-half, half))))
        self.columns = {
            row['gid'] for row in obstacles.rows
            if row['box']['name'].startswith('lift/')
            and row['box']['dimensions'][2] > .8
            and row['box']['dimensions'][2] > 3*max(row['box']['dimensions'][:2])
        }
        if not self.columns:
            raise ValueError('V5 posture guard requires the fixed lift mast')
        self.arm_geoms = np.array([
            model.body(int(b)).name.startswith(('Robot_link', 'Tool_'))
            for b in model.geom_bodyid
        ])
        self.elbow = model.joint('elbow').id
        self.wrist = model.joint('wrist1').id
        self.elbow_body = int(model.jnt_bodyid[self.elbow])
        self.wrist_body = int(model.jnt_bodyid[self.wrist])
        self.camera_clearance = None
        if 'camera_collision_clearance_m' in self.policy:
            from preview_gripper_camera import CameraClearance
            self.camera_clearance = CameraClearance(model, obstacles, self.policy['camera_mount'],
                                                   self.policy['camera_collision_clearance_m'])
        # Positive-distance contacts are deliberately generated on the private
        # mast proxies. A collision-free state alone does not certify clearance.
        # gap keeps these clearance contacts inactive (this preview never steps).
        for gid in self.columns:
            model.geom_margin[gid] = max(model.geom_margin[gid], .1001)
            model.geom_gap[gid] = max(model.geom_gap[gid], .1001)
        self.reset_audit()

    def tighten_bounds(self, bounds):
        result = bounds.copy()
        result[self.elbow] = np.maximum(result[self.elbow, 0], -math.radians(140.)), np.minimum(result[self.elbow, 1], math.radians(140.))
        result[self.wrist] = np.maximum(result[self.wrist, 0], -math.radians(150.)), np.minimum(result[self.wrist, 1], math.radians(150.))
        if self.yaw_wrist is not None:
            low, high = np.deg2rad(self.policy['wrist3_working_range_deg'])
            result[self.yaw_wrist] = max(result[self.yaw_wrist, 0], low), min(result[self.yaw_wrist, 1], high)
        return result

    def joint_rejection(self, q):
        bend = abs(math.degrees(float(q[self.elbow])))
        if not 20.-1e-8 <= bend <= 140.+1e-8:
            return dict(rule='elbow_working_range', measured_deg=bend, allowed_deg=[20., 140.])
        wrist = math.degrees(float(q[self.wrist]))
        if not -150.-1e-8 <= wrist <= 150.+1e-8:
            return dict(rule='wrist1_working_range', measured_deg=wrist, allowed_deg=[-150., 150.])
        if self.yaw_wrist is not None:
            angle = math.degrees(float(q[self.yaw_wrist]))
            low, high = self.policy['wrist3_working_range_deg']
            if not low-1e-8 <= angle <= high+1e-8:
                return dict(rule='wrist3_no_winding', measured_deg=angle, allowed_deg=[low, high])
        return None

    def grip_yaw(self, data):
        """One continuous heading chart, with its branch cut in the banned sector.

        Unlike wrapping a joint command, this only measures FK. Joint positions
        and interpolation are never rewritten, so a 360-degree path must cross
        the excluded heading sector and fail even if its endpoints match.
        """
        forward = data.xmat[self.grip_id].reshape(3, 3)[:, 1]
        angle = math.degrees(math.atan2(float(forward[1]), float(forward[0])))
        low, high = self.policy['grip_yaw_working_range_deg']
        midpoint = (low+high)*.5
        angle = midpoint + (angle-midpoint+180.) % 360. - 180.
        return angle, float(np.linalg.norm(forward[:2]))

    def yaw_rejection(self, data):
        if 'grip_yaw_working_range_deg' not in self.policy:
            return None
        angle, horizontal = self.grip_yaw(data)
        low, high = self.policy['grip_yaw_working_range_deg']
        if horizontal < self.policy['minimum_grip_forward_horizontal']:
            return dict(rule='grip_yaw_near_vertical', horizontal=horizontal)
        if not low-1e-8 <= angle <= high+1e-8:
            return dict(rule='grip_yaw_no_full_turn', measured_deg=angle, allowed_deg=[low, high])
        return None

    def forearm_pitch(self, data):
        arm = data.xpos[self.wrist_body]-data.xpos[self.elbow_body]
        return math.degrees(math.atan2(float(arm[2]), float(np.linalg.norm(arm[:2]))))

    def forearm_rejection(self, data):
        pitch = self.forearm_pitch(data)
        if pitch > POSTURE_POLICY['maximum_forearm_upward_deg']+1e-8:
            return dict(rule='forearm_points_up_from_low_elbow', measured_deg=pitch, maximum_deg=10.)
        return None

    def camera_gap(self, data):
        if self.camera_id is None:
            return None
        camera_z = self.camera_corners @ data.xmat[self.camera_id].reshape(3, 3)[2] + data.xpos[self.camera_id, 2]
        grip_z = self.grip_corners @ data.xmat[self.grip_id].reshape(3, 3)[2] + data.xpos[self.grip_id, 2]
        if self.policy.get('camera_height_reference') == 'capture_center':
            grip_z = np.array(self.policy['grip_center_body_m']) @ data.xmat[self.grip_id].reshape(3,3)[2] + data.xpos[self.grip_id,2]
        return float(camera_z.min() - grip_z.max())

    def geometry_rejection(self, data):
        reason = self.yaw_rejection(data)
        if reason:
            return reason
        reason = self.forearm_rejection(data)
        if reason:
            return reason
        gap = self.camera_gap(data)
        if gap is not None and gap < self.policy['minimum_camera_above_grip_m']:
            return dict(rule='camera_below_grip', clearance_m=gap,
                        minimum_m=self.policy['minimum_camera_above_grip_m'])
        return None

    def rejection(self, data, *, geometry_checked=False):
        if not geometry_checked:
            reason = self.geometry_rejection(data)
            if reason: return reason
        if self.camera_clearance is not None:
            clearance, other = self.camera_clearance.clearance(data)
            if clearance < self.camera_clearance.minimum:
                return dict(rule='camera_housing_clearance', clearance_m=clearance,
                            minimum_m=self.camera_clearance.minimum,
                            geom=self.model.geom(other).name if other is not None else None)
            self.audit['minimum_camera_housing_clearance_m'] = min(
                self.audit['minimum_camera_housing_clearance_m'], clearance)
        pitch = self.forearm_pitch(data)
        for contact in data.contact:
            a, b = int(contact.geom1), int(contact.geom2)
            arm = b if a in self.columns else a if b in self.columns else -1
            if arm >= 0 and self.arm_geoms[arm] and contact.dist < .10:
                return dict(rule='mast_clearance', clearance_m=float(contact.dist),
                    minimum_m=.10, body=self.model.body(int(self.model.geom_bodyid[arm])).name)
        self.audit['maximum_forearm_upward_deg'] = max(self.audit['maximum_forearm_upward_deg'], pitch)
        bend = abs(math.degrees(float(data.qpos[self.elbow])))
        self.audit['minimum_abs_elbow_deg'] = min(self.audit['minimum_abs_elbow_deg'], bend)
        self.audit['maximum_abs_elbow_deg'] = max(self.audit['maximum_abs_elbow_deg'], bend)
        self.audit['maximum_abs_wrist1_deg'] = max(self.audit['maximum_abs_wrist1_deg'], abs(math.degrees(float(data.qpos[self.wrist]))))
        self.audit['state_checks'] += 1
        gap = self.camera_gap(data)
        if gap is not None:
            self.audit['minimum_camera_above_grip_m'] = min(self.audit['minimum_camera_above_grip_m'], gap)
        if 'grip_yaw_working_range_deg' in self.policy:
            angle, horizontal = self.grip_yaw(data)
            self.audit['minimum_grip_yaw_deg'] = min(self.audit['minimum_grip_yaw_deg'], angle)
            self.audit['maximum_grip_yaw_deg'] = max(self.audit['maximum_grip_yaw_deg'], angle)
            self.audit['minimum_grip_forward_horizontal'] = min(self.audit['minimum_grip_forward_horizontal'], horizontal)
            self.audit['maximum_abs_wrist3_deg'] = max(self.audit['maximum_abs_wrist3_deg'], abs(math.degrees(float(data.qpos[self.yaw_wrist]))))
        return None

    def reset_audit(self):
        self.audit = dict(state_checks=0, certified_mast_clearance_lower_bound_m=.10,
            maximum_forearm_upward_deg=-180., minimum_abs_elbow_deg=180.,
            maximum_abs_elbow_deg=0., maximum_abs_wrist1_deg=0.)
        if self.camera_id is not None:
            self.audit['minimum_camera_above_grip_m'] = float('inf')
        if self.camera_clearance is not None:
            self.audit['minimum_camera_housing_clearance_m'] = float('inf')
        if 'grip_yaw_working_range_deg' in self.policy:
            self.audit.update(minimum_grip_yaw_deg=float('inf'), maximum_grip_yaw_deg=-float('inf'),
                              minimum_grip_forward_horizontal=1., maximum_abs_wrist3_deg=0.)
