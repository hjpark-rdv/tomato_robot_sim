"""Apply measured world poses through the existing USD parent-frame mapping."""
import json
import time
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom
from scipy.spatial.transform import Rotation


def restore_recorded_display_fixture(stage, trial):
    """Restore the one known synthetic fixture's visual from recorded metadata.

    No rigid body, collider, guide, material, or physics API is authored.  The
    diagnostic sphere was transient in the producer and is absent from the
    immutable scene. Its saved native pose will subsequently drive this visual.
    Unknown missing bodies still cause an explicit mapping error.
    """
    path = Path(trial) / 'belt_fixture.json'
    if not path.is_file():
        return None
    fixture = json.loads(path.read_text())
    body = fixture['body_path']
    if body != '/World/ActuatedTool/Diagnostics/BeltControl/Sphere':
        raise ValueError('Unexpected recorded display fixture body: ' + body)
    diameter = float(fixture['diameter_m'])
    center = np.asarray(fixture['start_center_world'], dtype=float)
    rotation = np.asarray(fixture['tool_frame_world'], dtype=float)
    if (not np.isfinite(diameter) or diameter <= 0 or center.shape != (3,) or
        rotation.shape != (3,3) or not np.isfinite(center).all() or not np.isfinite(rotation).all()):
        raise ValueError('Invalid recorded fixture visual geometry')
    if stage.GetPrimAtPath(body):
        raise ValueError('Recorded transient fixture already exists in source scene')
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        parent = UsdGeom.Xform.Define(stage, body.rsplit('/',1)[0])
        parent_world = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(parent.GetPrim())).T
        world = np.eye(4)
        world[:3,:3] = rotation
        world[:3,3] = center
        sphere = UsdGeom.Sphere.Define(stage, body)
        sphere.CreateRadiusAttr(diameter/2.)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(.15,.7,.95)])
        sphere.MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent_world)@world).T.tolist()))
    return dict(body_path=body, diameter_m=diameter, fixture_only=True,
                source='recorded synthetic native belt diagnostic; display geometry only',
                physics_rerun=False)


class MeasuredPoseDisplay:
    """Shared by saved-state rendering and the separate live display process."""
    def __init__(self, stage, body_paths):
        self.stage=stage
        self.paths = list(body_paths)
        index = {path: i for i, path in enumerate(self.paths)}
        if len(index) != len(self.paths):
            raise ValueError('Display body paths are not unique')
        cache = UsdGeom.XformCache()
        rest_world = {}
        for path in self.paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                raise ValueError('Measured body missing in display scene: ' + path)
            rest_world[path] = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        self.parent_references = []
        self.ops = []
        # The recorded synthetic sphere is already defined in the stronger
        # session layer. Clearing its xformOpOrder from the root layer cannot
        # take effect and MakeMatrixXform then returns an invalid operation.
        # All display setup/updates use the session layer, including source
        # bodies, without changing the caller's edit target or source asset.
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            for path in self.paths:
                prim = stage.GetPrimAtPath(path)
                parent_world = np.asarray(cache.GetLocalToWorldTransform(prim.GetParent())).T
                ancestor = prim.GetParent()
                while ancestor and str(ancestor.GetPath()) not in index:
                    ancestor = ancestor.GetParent()
                ancestor_path = str(ancestor.GetPath()) if ancestor else None
                relative = (np.linalg.inv(rest_world[ancestor_path]) @ parent_world
                            if ancestor_path else parent_world)
                self.parent_references.append((index[ancestor_path] if ancestor_path else None, relative))
                op=UsdGeom.Xformable(prim).MakeMatrixXform()
                if not op.GetAttr().IsValid():
                    raise ValueError('Measured display matrix operation unavailable: '+path)
                self.ops.append(op)

    def update(self, poses):
        started = time.perf_counter()
        poses = np.asarray(poses)
        if poses.shape != (len(self.paths), 7) or not np.isfinite(poses).all():
            raise ValueError('Invalid measured display poses')
        worlds = np.repeat(np.eye(4)[None], len(self.paths), axis=0)
        worlds[:, :3, :3] = Rotation.from_quat(poses[:, 3:7]).as_matrix()
        worlds[:, :3, 3] = poses[:, :3]
        writes = []
        for i, op in enumerate(self.ops):
            ancestor, relative = self.parent_references[i]
            parent = relative if ancestor is None else worlds[ancestor] @ relative
            writes.append((op, Gf.Matrix4d((np.linalg.inv(parent) @ worlds[i]).T.tolist())))
        calculated = time.perf_counter()
        with Usd.EditContext(self.stage,self.stage.GetSessionLayer()):
            with Sdf.ChangeBlock():
                for op, transform in writes:
                    op.Set(transform)
        finished = time.perf_counter()
        return dict(bodies_updated=len(writes), calculation_s=calculated-started,
                    usd_write_s=finished-calculated)
