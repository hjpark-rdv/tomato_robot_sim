"""Current S300/lift/RB5/belt-gripper USD articulation -> native MuJoCo.

This helper adds the existing articulation to the caller's MJCF. It does not
load a simulator, invent a harvesting trajectory, teleport the tool, or change
the plant. All controller coordinates are the source USD joint coordinates.
Run export-side code with nvidia-sim/.asset-venv (pxr is required).
"""
from pathlib import Path
import hashlib
import itertools
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Usd, UsdGeom, UsdPhysics


ROBOT = '/World/HarvestRobot/Robot'
TOOL = '/World/ActuatedTool'
ARM_NAMES = ['farmily_lift_height_joint', 'base', 'shoulder', 'elbow',
             'wrist1', 'wrist2', 'wrist3']


def nums(values):
    return ' '.join(format(float(x), '.12g') for x in np.asarray(values).ravel())


def quat(matrix):
    q = R.from_matrix(matrix).as_quat()
    return nums([q[3], *q[:3]])


def joint_frame(joint, index):
    """USD joint frame in its owning body; quaternions are wxyz in MJCF."""
    t = np.eye(4)
    q = getattr(joint, f'GetLocalRot{index}Attr')().Get()
    t[:3, :3] = R.from_quat([*q.GetImaginary(), q.GetReal()]).as_matrix()
    t[:3, 3] = getattr(joint, f'GetLocalPos{index}Attr')().Get()
    return t


def triangulate(mesh):
    ids = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    faces = []
    offset = 0
    for count in mesh.GetFaceVertexCountsAttr().Get():
        faces.extend((ids[offset], ids[offset+i], ids[offset+i+1])
                     for i in range(1, count-1))
        offset += count
    return np.asarray(faces, dtype=np.int64)


def add_robot(stage, manifest, *, root, world, assets, contact, meshdir,
              matrix=None, geometry_exporter=None):
    """Add physical robot/tool bodies and finite source drives to an MJCF tree.

    ``matrix(path)`` returns a column-vector USD world transform. An optional
    ``geometry_exporter(prim, node, local, attributes, assets, meshdir)`` may
    replace the per-source-mesh collision adapter (for example convex
    decomposition); return an inventory dict or list. It must preserve the
    supplied collision/visual flags. The default keeps every visible mesh and
    every authored collider; collision meshes use a MuJoCo convex hull and
    this difference from PhysX decomposition is recorded explicitly.

    Return ``nodes``, ``names`` and ``transforms`` for further caller geometry
    and portable ``metadata`` for JSON. ``initial_q`` is a joint-name mapping,
    NOT an already assembled MuJoCo qpos vector. The caller resolves addresses
    by joint name after loading the complete model.
    """
    meshdir = Path(meshdir)
    meshdir.mkdir(parents=True, exist_ok=True)
    if matrix is None:
        cache = UsdGeom.XformCache()
        def matrix(path):
            return np.asarray(cache.GetLocalToWorldTransform(
                stage.GetPrimAtPath(str(path))), dtype=float).T
    prims = [p for prefix in (ROBOT, TOOL)
             for p in Usd.PrimRange(stage.GetPrimAtPath(prefix),
                                   Usd.TraverseInstanceProxies())]
    bodies = {str(p.GetPath()): p for p in prims
              if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    joint_by_child = {}
    for prim in prims:
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for child in joint.GetBody1Rel().GetTargets():
            key = str(child)
            if key in joint_by_child:
                raise ValueError('Multiple parent joints for '+key)
            joint_by_child[key] = joint
    transforms = {path: matrix(path) for path in bodies}
    names = {path: ('Tool_' if path.startswith(TOOL+'/') else 'Robot_') +
             path.rsplit('/', 1)[1] for path in bodies}
    if len(set(names.values())) != len(names):
        raise ValueError('Nonunique robot body names')
    nodes, joints, inertia_records, exclusions = {}, [], [], set()
    actuators = root.find('actuator')
    if actuators is None:
        actuators = ET.SubElement(root, 'actuator')
    arm_q = dict(zip(ARM_NAMES, manifest.get('initial_robot_q', [])))
    arm_v = dict(zip(ARM_NAMES, manifest.get('initial_robot_qvel', [])))
    initial_q, initial_qvel, initial_ctrl, initial_ctrl_velocity = {}, {}, {}, {}

    def initial_value(name, prim, slider, velocity=False):
        if name in arm_q:
            return float((arm_v if velocity else arm_q).get(name, 0.))
        if name.startswith('jaw_'):
            field = 'initial_jaw_qvel' if velocity else 'initial_jaw_q'
            return float(manifest.get(field, {}).get(name, 0.))
        axis = 'linear' if slider else 'angular'
        value = prim.GetAttribute('state:'+axis+':physics:'+(
            'velocity' if velocity else 'position')).Get()
        value = 0. if value is None else float(value)
        return value if slider else np.deg2rad(value)

    def body(path):
        if path in nodes:
            return nodes[path]
        prim = bodies[path]
        joint = joint_by_child.get(path)
        parents = joint.GetBody0Rel().GetTargets() if joint else []
        parent = str(parents[0]) if parents else None
        if parent and parent not in bodies:
            raise ValueError('Missing S300 rigid parent '+parent)
        authored = np.linalg.inv(transforms[parent]) @ transforms[path] if parent else transforms[path]
        # The USD publishes zero-joint body transforms even at a saved nonzero
        # articulation state. Joint frames are the canonical zero-pose relation.
        local = joint_frame(joint, 0) @ np.linalg.inv(joint_frame(joint, 1)) if joint else authored
        mismatch = float(np.max(np.abs(local-authored)))
        if mismatch > 2e-5:
            raise ValueError(f'Authored robot transform vs zero joint frame mismatch {path}: {mismatch}')
        node = ET.SubElement(body(parent) if parent else world, 'body',
                             name=names[path], pos=nums(local[:3, 3]),
                             quat=quat(local[:3, :3]))
        nodes[path] = node
        mass_api = UsdPhysics.MassAPI(prim)
        mass = float(mass_api.GetMassAttr().Get())
        diagonal = np.asarray(mass_api.GetDiagonalInertiaAttr().Get(), dtype=float)
        com = np.asarray(mass_api.GetCenterOfMassAttr().Get(), dtype=float)
        principal = mass_api.GetPrincipalAxesAttr().Get()
        # USD uses (-inf,-inf,-inf) / a zero quaternion as "automatic" sentinels,
        # not physical COM/rotation. The fixed rail anchor has no geometry and
        # no dynamically integrated DOF; its explicit inertia is at its origin.
        automatic_com = bool(not np.isfinite(com).all())
        if automatic_com:
            com = np.zeros(3)
        principal_values = np.array([principal.GetReal(), *principal.GetImaginary()])
        automatic_axes = bool(np.linalg.norm(principal_values) < 1e-12)
        if automatic_axes:
            principal_values = np.array([1., 0., 0., 0.])
        inertia_attributes = dict(mass=str(mass), pos=nums(com),
                                  quat=nums(principal_values))
        if np.isfinite(diagonal).all() and np.min(diagonal) > 0:
            inertia_attributes['diaginertia'] = nums(diagonal)
            source = 'source_authored_diagonal_inertia'
        else:
            # Tool USD defers cooked inertia to PhysX. Explicit conservative
            # bounding-box estimate keeps dynamic finite-mass jaws rather than
            # claiming unavailable native cooked values were preserved.
            points = []
            for p in Usd.PrimRange(prim):
                if p.IsA(UsdGeom.Mesh):
                    owner = p
                    while owner and str(owner.GetPath()) not in bodies:
                        owner = owner.GetParent()
                    if not owner or str(owner.GetPath()) != path:
                        continue
                    local_mesh = np.linalg.inv(transforms[path]) @ matrix(p.GetPath())
                    vertices = np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(), dtype=float)
                    if vertices.size:
                        points.append(vertices @ local_mesh[:3, :3].T + local_mesh[:3, 3])
            if not points:
                raise ValueError('No geometry for inertia estimate '+path)
            vertices = np.vstack(points)
            minimum, maximum = vertices.min(axis=0), vertices.max(axis=0)
            dimensions = maximum-minimum
            inertia_attributes['pos'] = nums((maximum+minimum)/2)
            inertia_attributes['quat'] = '1 0 0 0'
            inertia_attributes['diaginertia'] = nums(mass/12 * np.array([
                dimensions[1]**2+dimensions[2]**2,
                dimensions[0]**2+dimensions[2]**2,
                dimensions[0]**2+dimensions[1]**2]))
            source = 'uncalibrated_CAD_bounding_box_estimate'
        ET.SubElement(node, 'inertial', **inertia_attributes)
        inertia_records.append(dict(body=names[path], source=source,
                                    usd_automatic_com=automatic_com,
                                    usd_automatic_axes=automatic_axes,
                                    **inertia_attributes))
        if joint:
            jp = joint.GetPrim()
            name = jp.GetName()
            slider = jp.IsA(UsdPhysics.PrismaticJoint)
            hinge = jp.IsA(UsdPhysics.RevoluteJoint)
            if slider or hinge:
                axis_index = 'XYZ'.index(jp.GetAttribute('physics:axis').Get())
                axis = joint_frame(joint, 1)[:3, :3] @ np.eye(3)[axis_index]
                limits = np.array([jp.GetAttribute('physics:lowerLimit').Get(),
                                   jp.GetAttribute('physics:upperLimit').Get()], dtype=float)
                if hinge:
                    limits = np.deg2rad(limits)
                ET.SubElement(node, 'joint', name=name,
                              type='slide' if slider else 'hinge',
                              pos=nums(joint_frame(joint, 1)[:3, 3]),
                              axis=nums(axis), limited='true', range=nums(limits))
                drive_type = 'linear' if slider else 'angular'
                drive = UsdPhysics.DriveAPI(jp, drive_type)
                kp = float(drive.GetStiffnessAttr().Get())
                kv = float(drive.GetDampingAttr().Get())
                force = float(drive.GetMaxForceAttr().Get())
                if not np.isfinite([kp, kv, force]).all() or force <= 0:
                    raise ValueError('Missing finite source drive '+name)
                ET.SubElement(actuators, 'position', name='drive_'+name, joint=name,
                              kp=str(kp), kv=str(kv), forcelimited='true',
                              forcerange=nums([-force, force]), ctrllimited='true',
                              ctrlrange=nums(limits))
                initial_q[name] = initial_value(name, jp, slider)
                initial_qvel[name] = initial_value(name, jp, slider, velocity=True)
                scale = 1. if slider else np.pi/180
                initial_ctrl[name] = float(drive.GetTargetPositionAttr().Get())*scale
                initial_ctrl_velocity[name] = float(drive.GetTargetVelocityAttr().Get())*scale
                if not limits[0]-1e-6 <= initial_q[name] <= limits[1]+1e-6:
                    raise ValueError('Source initial joint state outside limits '+name)
                joints.append(dict(name=name, body=names[path], parent=names.get(parent),
                                   type='slide' if slider else 'hinge', axis=axis.tolist(),
                                   position=joint_frame(joint, 1)[:3, 3].tolist(),
                                   limits=limits.tolist(), kp=kp, kv=kv,
                                   max_force=force, initial_q=initial_q[name],
                                   initial_qvel=initial_qvel[name],
                                   source_drive_target_position=initial_ctrl[name],
                                   source_drive_target_velocity=initial_ctrl_velocity[name],
                                   zero_frame_max_abs_error=mismatch,
                                   source_joint=str(jp.GetPath())))
            elif not jp.IsA(UsdPhysics.FixedJoint):
                raise ValueError('Unsupported robot joint '+str(jp.GetPath()))
            if parent and not bool(jp.GetAttribute('physics:collisionEnabled').Get()):
                exclusions.add(tuple(sorted((names[path], names[parent]))))
        return node

    for path in bodies:
        body(path)
    inventory = []
    for prim in prims:
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube) or
                prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Sphere) or prim.IsA(UsdGeom.Capsule)):
            continue
        owner = prim
        while owner and str(owner.GetPath()) not in bodies:
            owner = owner.GetParent()
        if not owner:
            continue
        owner_path = str(owner.GetPath())
        collision = bool(prim.HasAPI(UsdPhysics.CollisionAPI) and
                         UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
        visible = UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible'
        if not collision and not visible:
            continue
        local = np.linalg.inv(transforms[owner_path]) @ matrix(prim.GetPath())
        name = 's300_g'+str(len(inventory)).zfill(4)
        color = [.65, .68, .72]
        if prim.IsA(UsdGeom.Mesh):
            colors = UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()
            if colors:
                color = list(colors[0])
        attributes = dict(name=name, contype='2' if collision else '0',
                          conaffinity='3' if collision else '0',
                          group='0' if collision else '2', density='0',
                          rgba=nums([*color, 1 if visible else 0]))
        if geometry_exporter is not None:
            item = geometry_exporter(prim, nodes[owner_path], local, attributes,
                                     assets, meshdir)
            rows = item if isinstance(item, list) else [item]
            for row in rows:
                row.update(source=str(prim.GetPath()), owner=names[owner_path],
                           collision=collision)
                inventory.append(row)
            continue
        row = dict(name=name, source=str(prim.GetPath()), owner=names[owner_path],
                   collision=collision, visible=visible)
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            faces = triangulate(mesh)
            vertices = vertices @ local[:3, :3].T + local[:3, 3]
            # PrototypeTread is an authored eight-corner rectangular prism.
            # Detect it from the actual source vertices, not replacement dims.
            lo, hi = vertices.min(axis=0), vertices.max(axis=0)
            corners = np.array(list(itertools.product(*zip(lo, hi))))
            exact_box = len(vertices) == 8 and max(min(np.linalg.norm(v-corners, axis=1))
                                                 for v in vertices) < 1e-9
            if exact_box:
                attributes.update(type='box', pos=nums((lo+hi)/2), size=nums((hi-lo)/2))
                row['mujoco_collision'] = 'exact_source_box'
            else:
                filename = name+'.obj'
                with (meshdir/filename).open('w') as stream:
                    np.savetxt(stream, vertices, fmt='v %.9g %.9g %.9g')
                    np.savetxt(stream, faces+1, fmt='f %d %d %d')
                ET.SubElement(assets, 'mesh', name=name, file=filename)
                attributes.update(type='mesh', mesh=name)
                row['mujoco_collision'] = 'convex_hull_per_source_mesh' if collision else 'visual_triangles'
                row['mesh_sha256'] = hashlib.sha256((meshdir/filename).read_bytes()).hexdigest()
            row['source_approximation'] = str(prim.GetAttribute('physics:approximation').Get())
        else:
            scale = np.linalg.norm(local[:3, :3], axis=0)
            rotation = local[:3, :3]/scale
            attributes.update(pos=nums(local[:3, 3]), quat=quat(rotation))
            if prim.IsA(UsdGeom.Cube):
                attributes.update(type='box', size=nums(scale*UsdGeom.Cube(prim).GetSizeAttr().Get()/2))
            elif prim.IsA(UsdGeom.Sphere):
                radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get())
                attributes.update(type='ellipsoid', size=nums(radius*scale))
            else:
                shape = UsdGeom.Capsule(prim) if prim.IsA(UsdGeom.Capsule) else UsdGeom.Cylinder(prim)
                axis = 'XYZ'.index(shape.GetAxisAttr().Get())
                vector = np.eye(3)[axis]
                ends = np.array([-vector, vector])*float(shape.GetHeightAttr().Get())/2
                ends = ends @ local[:3, :3].T + local[:3, 3]
                attributes.pop('pos'); attributes.pop('quat')
                attributes.update(type='capsule' if prim.IsA(UsdGeom.Capsule) else 'cylinder',
                                  fromto=nums(ends), size=str(float(shape.GetRadiusAttr().Get())*scale[(axis+1)%3]))
            row['mujoco_collision'] = 'source_primitive'
        ET.SubElement(nodes[owner_path], 'geom', **attributes)
        inventory.append(row)
    for a, b in sorted(exclusions):
        ET.SubElement(contact, 'exclude', body1=a, body2=b)
    tool_paths = list(manifest['tool']['bodies'].values())+manifest['tool']['belts']
    metadata = dict(schema='s300-native-articulation-v1', joints=joints,
                    body_names=names, geometry=inventory, inertias=inertia_records,
                    initial_q=initial_q, initial_qvel=initial_qvel,
                    initial_ctrl=initial_ctrl,
                    initial_ctrl_velocity=initial_ctrl_velocity,
                    tool_paths=tool_paths, tool_names=[names[p] for p in tool_paths],
                    arm_joint_names=ARM_NAMES, rail_joint_name='root_joint',
                    jaw_joint_names=['jaw_L', 'jaw_R'],
                    belt_body_names=[names[p] for p in manifest['tool']['belts']],
                    exclusions=[list(pair) for pair in sorted(exclusions)],
                    zero_joint_transforms={p:t.tolist() for p,t in transforms.items()},
                    dynamic_tool=True, mocap_tool=False,
                    differences=[
                        'Source finite force-limited PD drives; response differs across engines and requires measured tracking validation.',
                        'Fixed chassis/attachment joints use MuJoCo tree welds instead of external PhysX fixed constraints.',
                        'Tool cooked PhysX inertia is unavailable; explicit uncalibrated CAD bounds estimates are used.',
                        'Default collision adapter uses one convex hull per CAD mesh; source convex decomposition equivalence is unverified.',
                        'CAD linkage transmission is visual only as in source; slider jaw actuation remains physical.'],
                    training_eligible=False, harvest_success=None)
    return dict(nodes=nodes, names=names, transforms=transforms, metadata=metadata,
                joints=joints, initial_q=initial_q, initial_qvel=initial_qvel,
                initial_ctrl=initial_ctrl,
                tool_paths=tool_paths, geometry=inventory)
