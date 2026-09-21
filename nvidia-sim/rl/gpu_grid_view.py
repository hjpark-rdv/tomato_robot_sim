"""Native single-environment view or physics-free grid of co-located clones.

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


class DisplayCadence:
    """Wall-clock refresh ceiling and measured display budget; never sleeps physics."""
    def __init__(self, fps, budget=.2):
        if fps<=0 or not 0<budget<=1:raise ValueError('Invalid display cadence')
        self.period=1./fps;self.budget=budget;self.next_frame=0.

    def due(self, now, force=False):
        return force or now>=self.next_frame

    def finish(self, began, ended):
        self.next_frame=began+max(self.period,(ended-began)/self.budget)


class GridView:
    def __init__(self, world, app, count=16, fps=5, output=None):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
        import omni.ui as ui
        self.world, self.app, self.stage = world, app, world.scene.stage
        self.context=world.sim.get_physics_context()
        self.original_writeback=self.context.get_physx_update_transformations_settings()
        self.count, self.fps = min(count, len(world.slots)), fps
        self.direct=len(world.slots)==1
        self.physics_dt=world.sim.get_physics_dt()
        self.cadence=DisplayCadence(fps)
        self.timings=dict(frames=0,state_updates=0,geometry_s=0.,render_s=0.,total_s=0.)
        self.output, self.last_frame, self.frames = output, 0., 0
        self.last_state_step = -1
        self.offsets = grid_offsets(self.count)
        self.root = '/World/DatasetDisplay'
        self.geometry = []
        self.skin_cache={}
        self.status = ['waiting'] * len(world.slots)
        self.labels = []
        self.paused, self.finished = False, False
        self.request_capture = False
        self.base = 0
        self.focused = None
        first = world.slots[0]
        template = []
        for branch in (() if self.direct else ('Robot', 'HarvestableStem')):
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(first.root+'/'+branch), Usd.TraverseInstanceProxies()):
                if not prim.IsA(UsdGeom.Gprim):
                    continue
                if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                    continue
                if UsdGeom.Imageable(prim).ComputePurpose() in ('guide', 'proxy'):
                    continue
                template.append(prim)
        for tile in range(0 if self.direct else self.count):
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
        if not self.direct:
            for slot in world.slots:UsdGeom.Imageable(self.stage.GetPrimAtPath(slot.root)).MakeInvisible()
        self.window = ui.Window('Parallel tomato tests', width=390, height=570)
        with self.window.frame:
            with ui.VStack(spacing=5):
                device='CPU' if getattr(world.slots[0].cfg,'dataset_cpu_single',False) else 'GPU'
                ui.Label('LIVE '+device+' PHYSICS / '+('single environment' if self.direct else 'display-only grid'), height=25)
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
        # Physics and labels read native tensors/contact reports. USD transforms
        # are needed only for a displayed frame, not every 120/960 Hz substep.
        # Reset paths explicitly synchronize their USD attachment frames.
        self.context.set_physx_update_transformations_settings(update_to_usd=False,update_velocities_to_usd=False)
        if self.direct:self.focus(0)
        else:self.overview()
        self.update(force=True)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.text = 'Resume' if self.paused else 'Pause'

    def page(self, direction):
        if self.direct:return
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
        self.last_state_step = -1
        self.cadence.next_frame=0.

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
        # Look from the plant side used by the inspection videos, rather than
        # through the approaching wrist. Keep nearby fruit and the ring visible.
        target = slot.elastic.fruit_centers()[slot.target_index]+self.offsets[tile]+[.005,0.,.015]
        set_camera_view(target+[.13,.38,.16],target)
        self.focused = tile

    def set_status(self, index, text):
        self.status[index] = text
        tile = index-self.base
        if 0 <= tile < len(self.labels):
            self.labels[tile].text = f'{tile+1:02d} | env {index:04d} | {text}'

    def capture(self):
        self.request_capture = True

    def update(self, force=False, physics_step=False):
        began=time.monotonic()
        if not self.cadence.due(began,force=force or self.request_capture):
            return
        state_changed=force or self.request_capture or self.world.step_timings['steps']!=self.last_state_step
        if state_changed:self._update_geometry()
        geometry_done=time.monotonic()
        self.header.text = f'{len(self.world.slots)} physics environments / showing {self.base}–{min(self.base+self.count,len(self.world.slots))-1}'
        self.footer.text = f'Simulated {self.world.step_timings["steps"]*self.physics_dt:.3f} s | '+('Finished; close window to exit.' if self.finished else 'Latest physical state; refresh adapts to display cost.')
        self.world.sim.render()
        rendered=time.monotonic()
        self.frames += 1
        self.timings['frames']+=1
        self.timings['state_updates']+=int(state_changed)
        self.timings['geometry_s']+=geometry_done-began
        self.timings['render_s']+=rendered-geometry_done
        self.timings['total_s']+=rendered-began
        self.last_state_step = self.world.step_timings['steps']
        self.last_frame = rendered
        self.cadence.finish(began,rendered)
        if self.request_capture and self.output:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            from datetime import datetime
            path = self.output/('grid_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png')
            capture_viewport_to_file(get_active_viewport(),str(path))
            self.request_capture = False
            print('[화면 저장]',path,flush=True)

    def _update_geometry(self):
        from pxr import Gf, UsdGeom, Vt
        from display_skin import DisplaySkin
        from omni.physx import get_physx_interface
        get_physx_interface().update_transformations(False,True,False)
        updates=[]
        if self.direct:
            # Native USD transforms already follow robot/fruit rigid bodies.
            # Update only non-colliding elastic visual meshes, no duplicated robot.
            elastic=self.world.slots[0].elastic
            poses,rotations=elastic._visual_transforms()
            transforms=DisplaySkin.transforms(rotations,poses[:,:3])
            for op,matrix,parent_inverse,index in elastic.rigid_visuals:
                delta=np.eye(4);delta[:3,:3]=rotations[index].T
                delta[3,:3]=poses[index,:3]-elastic.visual_rest[index,:3]@delta[:3,:3]
                updates.append((op,Gf.Matrix4d(*(matrix@delta@parent_inverse).flatten())))
            for row,kernel in zip(elastic.skin,self._skin_kernels(elastic)):
                mesh,_,inverse,_,_,_,normal_inverse=row
                points,normals=kernel.evaluate(transforms)
                points=points@inverse[:3,:3]+inverse[3,:3]
                normals=normals@normal_inverse
                normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
                updates.append((mesh.GetPointsAttr(),Vt.Vec3fArray.FromNumpy(points.astype(np.float32))))
                updates.append((mesh.GetNormalsAttr(),Vt.Vec3fArray.FromNumpy(normals.astype(np.float32))))
            self._apply_geometry_updates(updates)
            return
        cache = UsdGeom.XformCache()
        first = self.world.slots[0]
        for tile, meshes in enumerate(self.geometry):
            index = self.base+tile
            if index >= len(self.world.slots):continue
            slot = self.world.slots[index]
            offset = self.offsets[tile]
            poses, rotations = slot.elastic._visual_transforms()
            transforms=DisplaySkin.transforms(rotations,poses[:,:3])
            skin = {str(row[0].GetPath())[len(first.root):]:kernel for row,kernel in zip(slot.elastic.skin,self._skin_kernels(slot.elastic))}
            rigid = {str(row[0].GetAttr().GetPrimPath())[len(first.root):]:row for row in slot.elastic.rigid_visuals}
            for relative, (prim, op) in meshes.items():
                if relative in skin:
                    points,normal=skin[relative].evaluate(transforms)
                    normal /= np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
                    mesh = UsdGeom.Mesh(prim)
                    updates.append((mesh.GetPointsAttr(),Vt.Vec3fArray.FromNumpy(points.astype(np.float32))))
                    updates.append((mesh.GetNormalsAttr(),Vt.Vec3fArray.FromNumpy(normal.astype(np.float32))))
                    updates.append((mesh.GetExtentAttr(),Vt.Vec3fArray.FromNumpy(np.array([points.min(0),points.max(0)],dtype=np.float32))))
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
                updates.append((op,Gf.Matrix4d(*matrix.flatten())))
            self.labels[tile].text = f'{tile+1:02d} | env {index:04d} | {self.status[index]}'
        self._apply_geometry_updates(updates)

    @staticmethod
    def _apply_geometry_updates(updates):
        from pxr import Sdf
        # Batch USD notices. All scene reads and attribute lookups happen before
        # entering the block; physics is never advanced while edits are pending.
        with Sdf.ChangeBlock():
            for attribute,value in updates:attribute.Set(value)

    def _skin_kernels(self, elastic):
        from display_skin import DisplaySkin
        # Co-located clones share authored vertices and rest coordinates, so
        # their display matrices are compiled only once, not once per robot.
        key=elastic.visual_rest[:,:3].tobytes()
        if key not in self.skin_cache:
            self.skin_cache[key]=[DisplaySkin(points,elastic.visual_rest[:,:3],ids,weights,normals)
                for _,points,_,ids,weights,normals,_ in elastic.skin]
        return self.skin_cache[key]

    def pump(self, physics_step=False):
        self.update(physics_step=physics_step)
        while self.paused and self.app.is_running():
            self.update()
            time.sleep(.05)
        if not self.app.is_running():raise KeyboardInterrupt

    def close(self):
        self.window.visible = False
        self.context.set_physx_update_transformations_settings(*self.original_writeback)
