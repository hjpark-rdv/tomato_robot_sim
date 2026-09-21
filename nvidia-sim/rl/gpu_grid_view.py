"""Read-only display of co-located GPU environments as a physical-free grid.

Only geometry/materials are copied. USD physics schemas, joints, cameras and
colliders are never referenced into the display tree. Elastic meshes are skinned
from each slot's live state, not from env_0's shared visual bindings.
"""
import math
import time
import numpy as np


def grid_offsets(count, spacing=2.2):
    columns = math.ceil(math.sqrt(count))
    return np.array([(i % columns * spacing, i // columns * spacing, 0.) for i in range(count)])


class GridView:
    def __init__(self, world, app, count=16, fps=5, output=None):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
        import omni.ui as ui
        self.world, self.app, self.stage = world, app, world.scene.stage
        self.count, self.fps = min(count, len(world.slots)), fps
        self.output, self.last_frame, self.frames = output, 0., 0
        self.last_state_step = -16
        self.offsets = grid_offsets(self.count)
        self.root = '/World/DatasetDisplay'
        self.geometry = []
        self.status = ['waiting'] * len(world.slots)
        self.labels = []
        self.paused, self.finished = False, False
        self.request_capture = False
        self.base = 0
        self.focused = None
        first = world.slots[0]
        template = []
        for branch in ('Robot', 'HarvestableStem'):
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(first.root+'/'+branch), Usd.TraverseInstanceProxies()):
                if not prim.IsA(UsdGeom.Gprim):
                    continue
                if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                    continue
                if UsdGeom.Imageable(prim).ComputePurpose() in ('guide', 'proxy'):
                    continue
                template.append(prim)
        for tile in range(self.count):
            meshes = {}
            for index, source in enumerate(template):
                path = self.root+f'/Tile_{tile:02d}/Geometry_{index:04d}'
                dest = self.stage.DefinePrim(path, source.GetTypeName())
                # Copy visual attribute values only, never source references or applied APIs.
                for attr in source.GetAttributes():
                    name = attr.GetName()
                    if name.startswith(('physics:', 'physx', 'xformOp', 'collection:')) or name in ('visibility', 'purpose'):
                        continue
                    value = attr.Get()
                    if value is None:
                        continue
                    copied = dest.CreateAttribute(name, attr.GetTypeName(), attr.IsCustom())
                    copied.Set(value)
                    for key in ('interpolation', 'elementSize'):
                        if attr.HasAuthoredMetadata(key):copied.SetMetadata(key, attr.GetMetadata(key))
                material, _ = UsdShade.MaterialBindingAPI(source).ComputeBoundMaterial()
                if material:UsdShade.MaterialBindingAPI.Apply(dest).Bind(material)
                op = UsdGeom.Xformable(dest).AddTransformOp()
                relative = str(source.GetPath())[len(first.root):]
                meshes[relative] = (dest, op)
            self.geometry.append(meshes)
            # Display-only pad; no CollisionAPI.
            pad = UsdGeom.Cube.Define(self.stage, self.root+f'/Tile_{tile:02d}/Pad')
            pad.CreateSizeAttr(1.)
            pad.CreateDisplayColorAttr([(0.12, 0.16, 0.20) if tile % 2 else (0.17, 0.21, 0.25)])
            pad.AddTranslateOp().Set(Gf.Vec3d(*(self.offsets[tile]+[-.25,.3,-.03])))
            pad.AddScaleOp().Set(Gf.Vec3d(2.05,2.05,.025))
        UsdLux.DomeLight.Define(self.stage,self.root+'/Light').CreateIntensityAttr(1200.)
        # Hide the original overlapping scenes only after the dataset camera has captured.
        for slot in world.slots:UsdGeom.Imageable(self.stage.GetPrimAtPath(slot.root)).MakeInvisible()
        self.window = ui.Window('Parallel tomato tests', width=390, height=570)
        with self.window.frame:
            with ui.VStack(spacing=5):
                ui.Label('LIVE GPU PHYSICS / display-only grid', height=25)
                self.header = ui.Label('', height=30)
                with ui.HStack(height=28):
                    ui.Button('Overview', clicked_fn=self.overview)
                    ui.Button('Previous 16', clicked_fn=lambda:self.page(-1))
                    ui.Button('Next 16', clicked_fn=lambda:self.page(1))
                with ui.HStack(height=28):
                    self.pause_button = ui.Button('Pause', clicked_fn=self.toggle_pause)
                    ui.Button('Save screenshot', clicked_fn=self.capture)
                ui.Label('Click a row to inspect its hook. Overview restores grid.', height=25, word_wrap=True)
                with ui.ScrollingFrame():
                    with ui.VStack(spacing=3):
                        for i in range(self.count):
                            self.labels.append(ui.Button('', height=24, clicked_fn=lambda tile=i:self.focus(tile)))
                self.footer = ui.Label('', height=40, word_wrap=True)
        # Prove no dynamic/collision schemas entered the display tree.
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.root)):
            if any('Physics' in api or 'Physx' in api for api in prim.GetAppliedSchemas()):
                raise RuntimeError('Physics schema leaked into display: '+str(prim.GetPath()))
        self.overview()
        self.update(force=True)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.text = 'Resume' if self.paused else 'Pause'

    def page(self, direction):
        pages = math.ceil(len(self.world.slots)/self.count)
        self.base = ((self.base//self.count+direction) % pages)*self.count
        from pxr import UsdGeom
        for tile in range(self.count):
            prim = UsdGeom.Imageable(self.stage.GetPrimAtPath(self.root+f'/Tile_{tile:02d}'))
            valid = self.base+tile < len(self.world.slots)
            self.labels[tile].enabled = valid
            if valid:prim.MakeVisible()
            else:
                prim.MakeInvisible()
                self.labels[tile].text = f'{tile+1:02d} | empty'
        self.overview()
        self.last_frame = 0.
        self.last_state_step = -16

    def overview(self):
        from isaacsim.core.utils.viewports import set_camera_view
        center = self.offsets.mean(axis=0)+[-.25,.3,.65]
        extent = (max(np.ptp(self.offsets[:,0]),np.ptp(self.offsets[:,1]))+2.8)*1.2
        set_camera_view(center+np.array([.65,-.85,1.0])*extent, center)
        self.focused = None

    def focus(self, tile):
        from isaacsim.core.utils.viewports import set_camera_view
        if self.base+tile >= len(self.world.slots):return
        slot = self.world.slots[self.base+tile]
        target = slot.elastic.fruit_centers()[slot.target_index]+self.offsets[tile]
        set_camera_view(target+[.5,-.8,.35],target)
        self.focused = tile

    def set_status(self, index, text):
        self.status[index] = text
        tile = index-self.base
        if 0 <= tile < len(self.labels):
            self.labels[tile].text = f'{tile+1:02d} | env {index:04d} | {text}'

    def capture(self):
        self.request_capture = True

    def update(self, force=False, physics_step=False):
        if not force and time.monotonic()-self.last_frame < 1./self.fps:
            return
        # Commands change at 60 Hz (16 physics substeps). Skinning all visible
        # plants between every 960 Hz substep makes presentation dominate the
        # experiment. UI can still refresh without rewriting display geometry.
        step_delta = self.world.step_timings['steps']-self.last_state_step
        if not force and not self.request_capture and (step_delta == 0 or (physics_step and step_delta < 16)):
            self.world.sim.render()
            self.last_frame = time.monotonic()
            return
        from pxr import Gf, UsdGeom, Vt
        from elastic_plant import skin_points
        cache = UsdGeom.XformCache()
        first = self.world.slots[0]
        for tile, meshes in enumerate(self.geometry):
            index = self.base+tile
            if index >= len(self.world.slots):continue
            slot = self.world.slots[index]
            offset = self.offsets[tile]
            poses, rotations = slot.elastic._visual_transforms()
            skin = {str(row[0].GetPath())[len(first.root):]:row for row in slot.elastic.skin}
            rigid = {str(row[0].GetAttr().GetPrimPath())[len(first.root):]:row for row in slot.elastic.rigid_visuals}
            for relative, (prim, op) in meshes.items():
                if relative in skin:
                    _, points, _, ids, weights, normals, _ = skin[relative]
                    points = skin_points(points,slot.elastic.visual_rest[:,:3],rotations,poses[:,:3],ids,weights)
                    normal = (np.einsum('nkij,nj->nki',rotations[ids],normals)*weights[:,:,None]).sum(axis=1)
                    normal /= np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
                    mesh = UsdGeom.Mesh(prim)
                    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
                    mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normal.astype(np.float32)))
                    mesh.GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([points.min(0),points.max(0)],dtype=np.float32)))
                    matrix = np.eye(4)
                elif relative in rigid:
                    _, rest, _, body = rigid[relative]
                    delta = np.eye(4);delta[:3,:3] = rotations[body].T
                    delta[3,:3] = poses[body,:3]-slot.elastic.visual_rest[body,:3]@delta[:3,:3]
                    matrix = rest@delta
                else:
                    source = self.stage.GetPrimAtPath(slot.root+relative)
                    matrix = np.array(cache.GetLocalToWorldTransform(source))
                matrix[3,:3] += offset
                op.Set(Gf.Matrix4d(*matrix.flatten()))
            self.labels[tile].text = f'{tile+1:02d} | env {index:04d} | {self.status[index]}'
        self.header.text = f'{len(self.world.slots)} physics environments / showing {self.base}–{min(self.base+self.count,len(self.world.slots))-1}'
        self.footer.text = f'Simulated {self.world.step_timings["steps"]/960:.3f} s | '+('Finished; close window to exit.' if self.finished else 'Physics timestep unchanged; wall-clock playback may be slow.')
        self.world.sim.render()
        self.frames += 1
        self.last_state_step = self.world.step_timings['steps']
        self.last_frame = time.monotonic()
        if self.request_capture and self.output:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            from datetime import datetime
            path = self.output/('grid_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png')
            capture_viewport_to_file(get_active_viewport(),str(path))
            self.request_capture = False
            print('[DATASET VIEW] screenshot',path,flush=True)

    def pump(self, physics_step=False):
        self.update(physics_step=physics_step)
        while self.paused and self.app.is_running():
            self.update()
            time.sleep(.05)
        if not self.app.is_running():raise KeyboardInterrupt

    def close(self):
        self.window.visible = False
