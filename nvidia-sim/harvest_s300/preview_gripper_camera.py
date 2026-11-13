"""V6 camera relocation using the original CAD and optical calibration.

Only the V6 preview mount and approved jaw-cover pockets change. The source
scene, production camera calibration and V1--V5 installation remain intact.
"""
from itertools import product
from collections import OrderedDict

import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation

from newton_viser import geom_mesh, populate_scene


HOUSING = 's300_g0092'
OLD_SUPPORT = 's300_g0093'
COVERS = (('s300_g0307', 'Tool_L', .004), ('s300_g0428', 'Tool_R', -.004))


def subtract_bounds(bounds, cut):
    """Disjoint boxes covering AABB minus a rectangular relief, without filling it."""
    low, high = np.asarray(bounds, float)
    a, b = np.maximum(low, cut[0]), np.minimum(high, cut[1])
    if np.any(a >= b):
        return [[low.tolist(), high.tolist()]]
    remaining_low, remaining_high = low.copy(), high.copy()
    parts = []
    for axis in range(3):
        for edge, upper in ((a[axis], False), (b[axis], True)):
            lo, hi = remaining_low.copy(), remaining_high.copy()
            if upper: lo[axis] = edge
            else: hi[axis] = edge
            if np.all(hi-lo > 1e-10): parts.append([lo.tolist(), hi.tolist()])
        remaining_low[axis], remaining_high[axis] = a[axis], b[axis]
    return parts


def relieved_cover_mesh(scene, relief):
    """Cut only the approved camera pocket in a copy of the original CAD."""
    import trimesh
    gid = next(i for i in range(scene.ngeom) if scene.geom(i).name == relief['geom'])
    original = geom_mesh(scene, gid)
    low, high = np.asarray(relief['cut_bounds_body_m'])
    cutter = trimesh.creation.box(high-low)
    cutter.apply_translation((low+high)/2.)
    mesh = trimesh.boolean.difference([original, cutter], engine='manifold')
    if not mesh.is_watertight or not 0 < mesh.volume < original.volume:
        raise ValueError('Camera cover relief did not produce a closed, smaller solid')
    mesh.visual.vertex_colors = np.tile(original.visual.vertex_colors[0], (len(mesh.vertices), 1))
    return mesh


def camera_mount_spec(scene, source_mount):
    gid = next(i for i in range(scene.ngeom) if scene.geom(i).name == HOUSING)
    if scene.body(int(scene.geom_bodyid[gid])).name != 'Robot_link5':
        raise ValueError('Camera source CAD owner changed')
    old = np.asarray(source_mount['body_from_camera_optical'], float)
    vertices = geom_mesh(scene, gid).vertices
    optical_vertices = (vertices-old[:3, 3]) @ old[:3, :3]
    low, high = optical_vertices.min(0), optical_vertices.max(0)
    # Portrait camera, aimed forward through the rollers and slightly down.
    tilt = np.deg2rad(15.)
    rotation = np.array([[1., 0., 0.],
                         [0., -np.sin(tilt), np.cos(tilt)],
                         [0., -np.cos(tilt), -np.sin(tilt)]])
    corners = np.array(list(product(*zip(low, high)))) @ rotation.T
    optical = np.eye(4)
    optical[:3, :3] = rotation
    # Recess against the bracket nose; a small upward shift preserves the
    # camera-above-grip rule during the existing tilted approaches. The two
    # jaw covers receive only the user-approved camera pocket below.
    optical[:3, 3] = [-(corners[:, 0].min()+corners[:, 0].max())/2.,
                      -.0275-corners[:, 1].min(), .078-corners[:, 2].min()]
    corners += optical[:3, 3]
    visual_transform = optical @ np.linalg.inv(old)
    reliefs = []
    for name, body, closed in COVERS:
        i = next(i for i in range(scene.ngeom) if scene.geom(i).name == name)
        if scene.body(int(scene.geom_bodyid[i])).name != body:
            raise ValueError('Camera pocket jaw body changed')
        # Tool_L/R are X sliders relative to Tool_fixed. Define the common
        # 32mm pocket at full closure, then carry each half with its own jaw.
        cut = np.array([[-.016, -.030, .075], [.016, .024, .108]])
        cut[:, 0] -= closed
        bounds = geom_mesh(scene, i).bounds
        reliefs.append(dict(geom=name, body=body, closed_position_m=closed,
            cut_bounds_body_m=cut.tolist(),
            retained_bounds_body_m=subtract_bounds(bounds, cut)))
    return dict(schema='v6-gripper-recessed-portrait-rgbd-v2',
        body='Tool_fixed', source_body='Robot_link5', housing_geom=HOUSING,
        hidden_support_geom=OLD_SUPPORT,
        tool_from_source_camera_body=visual_transform.tolist(),
        body_from_camera_optical=optical.tolist(),
        housing_bounds_body_m=[corners.min(0).tolist(), corners.max(0).tolist()],
        housing_optical_bounds_m=[low.tolist(), high.tolist()],
        downward_tilt_deg=15.,
        support=dict(position=[0., -.029, .114], dimensions=[.024, .022, .006]),
        cover_reliefs=reliefs,
        resolution_wh=source_mount['resolution_wh'], K=source_mount['K'],
        depth_range_m=source_mount['depth_range_m'],
        mount_description='Original portrait RealSense CAD recessed 67.5mm toward the central bracket, with approved jaw-cover pockets; follows Tool_fixed including wrist3 yaw',
        source_camera_collision_enabled=False, physical_payload_changed=False,
        scope='5906 V6 concept mount; source RGB-D calibration and production physics unchanged')


def populate_camera_scene(server, scene, spec):
    """Separate camera and jaw-cover variants without rewriting source assets."""
    ids = {scene.geom(i).name: i for i in range(scene.ngeom)}
    selected = [ids[HOUSING], ids[OLD_SUPPORT]] + [ids[r['geom']] for r in spec.get('cover_reliefs', [])]
    meshes = [geom_mesh(scene, i) for i in selected]
    rgba = scene.geom_rgba[selected].copy()
    scene.geom_rgba[selected, 3] = 0.
    try:
        handles = populate_scene(server, scene, include_static=False)
    finally:
        scene.geom_rgba[selected] = rgba
    camera = server.scene.add_mesh_trimesh('/mounted_rgbd/housing', meshes[0])
    old_support = server.scene.add_mesh_trimesh('/mounted_rgbd/old_support', meshes[1])
    support = server.scene.add_box('/mounted_rgbd/central_standoff',
        dimensions=spec['support']['dimensions'], color=(158, 163, 171))
    covers = []
    for index, relief in enumerate(spec.get('cover_reliefs', [])):
        path = '/mounted_rgbd/covers/' + relief['geom']
        covers.append((relief['body'],
            server.scene.add_mesh_trimesh(path+'/original', meshes[index+2]),
            server.scene.add_mesh_trimesh(path+'/recessed', relieved_cover_mesh(scene, relief))))
    return handles, (camera, old_support, support, covers)


def update_camera(handles, poses, scene, spec, layout):
    camera, old_support, support, covers = handles
    wrist = poses[scene.body(spec['source_body']).id]
    tool = poses[scene.body(spec['body']).id]
    camera_pose = (tool @ np.asarray(spec['tool_from_source_camera_body'])
                   if layout == 'side' else wrist)
    for handle, pose in ((camera, camera_pose), (old_support, wrist)):
        handle.position = pose[:3, 3]
        handle.wxyz = Rotation.from_matrix(pose[:3, :3]).as_quat()[[3, 0, 1, 2]]
    old_support.visible = layout != 'side'
    support.visible = layout == 'side'
    support.position = tool[:3, 3] + tool[:3, :3] @ spec['support']['position']
    support.wxyz = Rotation.from_matrix(tool[:3, :3]).as_quat()[[3, 0, 1, 2]]
    for body, original, recessed in covers:
        pose = poses[scene.body(body).id]
        for handle in (original, recessed):
            handle.position = pose[:3, 3]
            handle.wxyz = Rotation.from_matrix(pose[:3, :3]).as_quat()[[3, 0, 1, 2]]
        original.visible = layout != 'side'
        recessed.visible = layout == 'side'


def optical_world(poses, scene, spec, layout, source_mount):
    body = spec['body'] if layout == 'side' else spec['source_body']
    local = spec['body_from_camera_optical'] if layout == 'side' else source_mount['body_from_camera_optical']
    return poses[scene.body(body).id] @ np.asarray(local)


class CameraClearance:
    """Explicit native distance queries for the camera's conservative OBB.

    A disabled geometry slot is used only as a query shape. No contact masks,
    body ownership or compiled collision tree is changed. Explicit queries
    also test same-weld parts which MuJoCo's contact broad phase would omit.
    """
    def __init__(self, model, obstacles, spec, minimum=.002):
        self.model, self.minimum = model, minimum
        occupied = {r['gid'] for r in obstacles.rows}
        slots = [i for i in range(model.ngeom) if i not in occupied
                 and model.geom_contype[i] == model.geom_conaffinity[i] == 0
                 and not model.body(int(model.geom_bodyid[i])).name.startswith(('Robot_', 'Tool_'))]
        reliefs = spec.get('cover_reliefs', [])
        if len(slots) < (2 if reliefs else 1):
            raise ValueError('No inactive geometry slot for camera distance checks')
        self.gid = slots[0]
        self.body = model.body(spec['body']).id
        optical = np.asarray(spec['body_from_camera_optical'])
        low, high = np.asarray(spec['housing_optical_bounds_m'])
        self.rotation = optical[:3,:3]
        self.centre = optical[:3,3]+self.rotation @ ((low+high)/2.)
        model.geom_type[self.gid] = mj.mjtGeom.mjGEOM_BOX
        model.geom_dataid[self.gid] = -1
        model.geom_size[self.gid] = (high-low)/2.
        self.radius = float(np.linalg.norm(model.geom_size[self.gid]))
        self.candidates = np.flatnonzero((model.geom_contype | model.geom_conaffinity) != 0)
        self.cover_parts, self.jaw_addresses = [], []
        self.cover_cache = OrderedDict()
        if reliefs:
            self.cover_gid = slots[1]
            model.geom_type[self.cover_gid] = mj.mjtGeom.mjGEOM_BOX
            model.geom_dataid[self.cover_gid] = -1
            for relief in reliefs:
                original = model.geom(relief['geom']).id
                body = model.body(relief['body']).id
                joint = int(model.body_jntadr[body])
                if (model.body_parentid[body] != self.body or model.body_jntnum[body] != 1
                        or model.jnt_type[joint] != mj.mjtJoint.mjJNT_SLIDE
                        or not np.allclose(model.jnt_axis[joint], [1., 0., 0.])
                        or not np.allclose(model.body_pos[body], 0.)
                        or not np.allclose(model.body_quat[body], [1., 0., 0., 0.])):
                    raise ValueError('Camera pocket requires the original Tool X sliders')
                self.jaw_addresses.append(int(model.jnt_qposadr[joint]))
                # Replace this pair's distance query, not its collision masks.
                # The uncut native hull remains a conservative obstacle for
                # every other robot part; only camera/cover uses the pocket.
                self.candidates = self.candidates[self.candidates != original]
                for low, high in np.asarray(relief['retained_bounds_body_m']):
                    self.cover_parts.append((body, original, (low+high)/2., (high-low)/2.))
        self.radii = model.geom_rbound[self.candidates].copy()
        self.planes = model.geom_type[self.candidates] == mj.mjtGeom.mjGEOM_PLANE
        self.rigid_cache = None

    def enable_rigid_cache(self):
        """Opt in only when query geometry remains immutable for the run.

        A common rigid transform cannot change a distance. Tool-weld parts
        and the two validated jaw welds therefore depend only on exact jaw
        coordinates. All other bodies still receive native distance queries
        at every checked pose, including every measured physics state.
        """
        bodies = self.model.body_weldid[self.model.geom_bodyid[self.candidates]]
        self.fixed_candidates = bodies == self.model.body_weldid[self.body]
        welds = [int(self.model.body_weldid[body]) for body, *_ in self.cover_parts]
        self.rigid_candidates = np.isin(bodies, welds) & ~self.fixed_candidates
        self.fixed_clearance = None
        self.rigid_cache = OrderedDict()

    def candidate_clearance(self, data, position, candidates, radii, planes):
        limit = .05
        close = (np.linalg.norm(data.geom_xpos[candidates]-position,axis=1)
                 < radii+self.radius+limit) | planes
        best, other = limit, None
        for gid in candidates[close]:
            distance = float(mj.mj_geomDistance(self.model,data,self.gid,int(gid),limit,None))
            if distance < best:
                best, other = distance, int(gid)
        return best, other

    def clearance(self, data):
        rotation = data.xmat[self.body].reshape(3,3)
        position = data.xpos[self.body]+rotation @ self.centre
        data.geom_xpos[self.gid] = position
        data.geom_xmat[self.gid] = (rotation @ self.rotation).ravel()
        limit = .05
        if self.rigid_cache is None:
            best, other = self.candidate_clearance(data, position, self.candidates, self.radii, self.planes)
        else:
            mask = self.rigid_candidates
            moving = ~(mask | self.fixed_candidates)
            best, other = self.candidate_clearance(data, position, self.candidates[moving], self.radii[moving], self.planes[moving])
            if self.fixed_clearance is None:
                fixed = self.fixed_candidates
                self.fixed_clearance = self.candidate_clearance(data, position, self.candidates[fixed], self.radii[fixed], self.planes[fixed])
            if self.fixed_clearance[0]-1e-10 < best:best, other = self.fixed_clearance[0]-1e-10, self.fixed_clearance[1]
            key = tuple(data.qpos[self.jaw_addresses])
            cached = self.rigid_cache.get(key)
            if cached is None:
                cached = self.candidate_clearance(data, position, self.candidates[mask], self.radii[mask], self.planes[mask])
                self.rigid_cache[key] = cached
                if len(self.rigid_cache)>2048:self.rigid_cache.popitem(last=False)
            # Conservative floating point allowance for common transforms.
            if cached[0]-1e-10 < best:best, other = cached[0]-1e-10, cached[1]
        if self.cover_parts:
            # Rigid tool-relative distance depends only on the two sliders,
            # not on the arm/lift pose. Cache exact jaw values, without rounding.
            key = tuple(data.qpos[self.jaw_addresses])
            cached = self.cover_cache.get(key)
            if cached is None:
                cover_best, cover_other = limit, None
                for body, original, centre, size in self.cover_parts:
                    r = data.xmat[body].reshape(3,3)
                    self.model.geom_size[self.cover_gid] = size
                    data.geom_xpos[self.cover_gid] = data.xpos[body]+r@centre
                    data.geom_xmat[self.cover_gid] = r.ravel()
                    d = float(mj.mj_geomDistance(self.model,data,self.gid,self.cover_gid,limit,None))
                    if d < cover_best: cover_best, cover_other = d, original
                cached = self.cover_cache[key] = (cover_best, cover_other)
                if len(self.cover_cache) > 2048: self.cover_cache.popitem(last=False)
            if cached[0] < best: best, other = cached
        return best, other
