"""One V6 CAD assembly shared by the live display and real GPU RGB-D sensor.

The existing packed farm and its instance identities stay intact. Old robot
geometry is hidden; the latest robot, camera pockets and conveyor replace it.
No physical model or plant placement is modified here.
"""
from pathlib import Path
import hashlib
import json
import threading

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_scene_chunks import PackedScene
from newton_viser import geom_mesh
from newton_inputs import sha256
from newton_geometric_harvest import sensor_poses
from preview_gripper_camera import relieved_cover_mesh, HOUSING, OLD_SUPPORT
from preview_robot_mount import body_poses
from side_farm import read


def visual_pose_indices(native_names,visual_names):
    by_name={str(name):i for i,name in enumerate(native_names)}
    result=[]
    for i,name in enumerate(visual_names):
        if str(name)in by_name:result.append(by_name[str(name)])
        elif str(name).startswith('Preview_'):result.append(i)
        else:raise ValueError('Unmapped sensor/display body: '+str(name))
    return np.array(result,int)


def compose_scene(farm, robot, meta):
    """Merge compiled triangles without changing any farm geometry or labels."""
    if not np.array_equal(farm.body_names,robot.body_names[:farm.nbody]):
        raise ValueError('Farm and V6 visual body identities differ')
    scene=object.__new__(PackedScene)
    fields=('geom_type','geom_size','geom_bodyid','geom_pos','geom_quat','geom_rgba','geom_matid')
    for name in fields:setattr(scene,name,np.concatenate((getattr(farm,name),getattr(robot,name))).copy())
    scene.geom_names=np.r_[farm.geom_names,np.array(['v6/'+str(n)for n in robot.geom_names])]
    scene.body_names=np.r_[robot.body_names,['Preview_conveyor','Preview_side_adapter']]
    scene.geom_dataid=np.r_[farm.geom_dataid,np.where(robot.geom_dataid>=0,robot.geom_dataid+len(farm.mesh_vertnum),-1)]
    for kind in ('vert','face'):
        setattr(scene,'mesh_'+kind,np.concatenate((getattr(farm,'mesh_'+kind),getattr(robot,'mesh_'+kind))))
        setattr(scene,'mesh_'+kind+'adr',np.r_[getattr(farm,'mesh_'+kind+'adr'),getattr(robot,'mesh_'+kind+'adr')+len(getattr(farm,'mesh_'+kind))])
        setattr(scene,'mesh_'+kind+'num',np.r_[getattr(farm,'mesh_'+kind+'num'),getattr(robot,'mesh_'+kind+'num')])
    scene.mat_rgba=np.empty((0,4));scene.geom_matid[:]=-1
    scene.vertex_color_lookup=dict(getattr(farm,'vertex_color_lookup',{}))
    scene.vertex_color_lookup.update({'v6/'+n:v for n,v in robot.vertex_color_lookup.items()})
    old_robot=[i for i in range(farm.ngeom)if farm.body(int(farm.geom_bodyid[i])).name.startswith(('Robot_','Tool_'))]
    scene.geom_rgba[old_robot,3]=0.
    scene.geom_rgba[farm.ngeom+np.flatnonzero(np.isin(robot.geom_bodyid,[meta['plate_id'],meta['motor_id']])),3]=0.
    ids={robot.geom(i).name:i for i in range(robot.ngeom)}
    scene.geom_rgba[farm.ngeom+ids[OLD_SUPPORT],3]=0.
    mount=meta['gripper_camera_mount'];tool=robot.body(mount['body']).id
    camera=geom_mesh(robot,ids[HOUSING]);camera.apply_transform(np.asarray(mount['tool_from_source_camera_body']))
    replacements=[(ids[HOUSING],tool,camera)]
    replacements += [(ids[r['geom']],robot.body(r['body']).id,relieved_cover_mesh(robot,r))for r in mount['cover_reliefs']]
    vertices=[];faces=[];nv=len(scene.mesh_vert);nf=len(scene.mesh_face);nm=len(scene.mesh_vertnum)
    for k,(gid,bid,mesh)in enumerate(replacements):
        i=farm.ngeom+gid;scene.geom_dataid[i]=nm+k;scene.geom_type[i]=mj.mjtGeom.mjGEOM_MESH
        scene.geom_bodyid[i]=bid;scene.geom_pos[i]=0.;scene.geom_quat[i]=[1.,0.,0.,0.]
        vertices.append(np.asarray(mesh.vertices));faces.append(np.asarray(mesh.faces))
        scene.vertex_color_lookup[str(scene.geom_names[i])]=np.asarray(mesh.visual.vertex_colors)
    for kind,arrays,offset in (('vert',vertices,nv),('face',faces,nf)):
        lengths=np.array([len(x)for x in arrays])
        setattr(scene,'mesh_'+kind+'adr',np.r_[getattr(scene,'mesh_'+kind+'adr'),offset+np.r_[0,np.cumsum(lengths)[:-1]]])
        setattr(scene,'mesh_'+kind+'num',np.r_[getattr(scene,'mesh_'+kind+'num'),lengths])
        setattr(scene,'mesh_'+kind,np.concatenate([getattr(scene,'mesh_'+kind),*arrays]))
    extra=[];bin_id=robot.nbody;adapter_id=bin_id+1;spec=meta['layouts']['side']['conveyor']
    def box(name,bid,position,dimensions,color):
        extra.append((name,bid,position,np.array(dimensions)/2.,[1.,0.,0.,0.],mj.mjtGeom.mjGEOM_BOX,np.r_[np.array(color)/255.,1.]))
    for p in spec['parts']:
        if not p.get('grip_only'):box('conveyor/'+p['name'],bin_id,p['position'],p['dimensions'],p['color'])
    quat=Rotation.from_euler('y',90,degrees=True).as_quat()[[3,0,1,2]]
    for i,y in enumerate(spec['roller_y_m']):
        extra.append((f'conveyor/roller_{i}',bin_id,[0.,y,spec['roller_z_m']],
            [spec['roller_radius_m'],(spec['roller_length_m']+.012)/2.,0.],quat,mj.mjtGeom.mjGEOM_CYLINDER,[.57,.62,.63,1.]))
        box(f'conveyor/roller_mark_{i}',bin_id,[-.262,y,spec['roller_z_m']+.012],[.006,.012,.004],(238,182,57))
    for i in range(36):box(f'conveyor/belt_rib_{i}',bin_id,[0.,spec['belt_start_y_m']+i*spec['belt_length_m']/36.,spec['belt_top_m']+.0007],[.510,.004,.0014],(73,81,84))
    size=meta['layouts']['side']['adapter_size_m']
    box('side_adapter',adapter_id,[0.,size[1]/2.,0.],size,(180,184,190))
    box('camera_support',tool,mount['support']['position'],mount['support']['dimensions'],(158,163,171))
    scene.geom_names=np.r_[scene.geom_names,[p[0]for p in extra]]
    for name,col in (('geom_bodyid',1),('geom_pos',2),('geom_size',3),('geom_quat',4),('geom_type',5),('geom_rgba',6)):
        setattr(scene,name,np.concatenate((getattr(scene,name),np.asarray([p[col]for p in extra]))))
    for name in ('geom_dataid','geom_matid'):setattr(scene,name,np.r_[getattr(scene,name),np.full(len(extra),-1,int)])
    scene.ngeom=len(scene.geom_names);scene.nbody=len(scene.body_names)
    scene._body_ids={str(n):i for i,n in enumerate(scene.body_names)}
    return scene


def prepared_sensor_scene(assets,manifest_path,index=0):
    """Worker-side geometry preparation; no physics models or stepping."""
    assets=Path(assets);config=read(assets/'patrol_assets.json');meta=config['side_installation']['preview_metadata']
    manifest_path=Path(manifest_path);manifest=read(manifest_path);section=manifest['chunks'][index]
    farm=PackedScene(manifest_path.parent/section['geometry'],section['geometry_sha256'])
    if sha256(manifest_path.parent/section['catalog'])!=section['catalog_sha256']:raise ValueError('Sensor catalog changed')
    catalog=read(manifest_path.parent/section['catalog'])
    preview=Path(config['side_installation']['preview']);robot=PackedScene(preview/'preview_scene.npz')
    provenance=dict(farm_geometry_sha256=section['geometry_sha256'],sensor_catalog_sha256=section['catalog_sha256'],
        robot_geometry_sha256=sha256(preview/'preview_scene.npz'),physical_model_sha256=sha256(assets/'inputs/input_model.mjb'),
        mounted_camera_sha256=hashlib.sha256(json.dumps(config['mounted_rgbd'],sort_keys=True).encode()).hexdigest(),
        compositor_sha256=sha256(__file__),layout='V6 side mount, recessed camera, approved cover pockets and conveyor')
    return compose_scene(farm,robot,meta),catalog,provenance


class SideCameraScene:
    def __init__(self,assets):
        self.assets=Path(assets);self.config=read(self.assets/'patrol_assets.json');install=self.config['side_installation']
        self.meta=install['preview_metadata'];self.lock=threading.RLock()
        old=Path(self.config['source_patrol_assets'])
        self.robot=PackedScene(Path(install['preview'])/'preview_scene.npz')
        self.visual,self.catalog,self.provenance=prepared_sensor_scene(self.assets,self.assets/'scene_stream/manifest.json')
        self.native=mj.MjModel.from_binary_path(str(self.assets/'inputs/input_model.mjb'));self.data=mj.MjData(self.native)
        self.source=mj.MjModel.from_binary_path(str(old/'inputs/input_model.mjb'))
        self.initial=dict(np.load(old/'inputs/initial_state.npz',allow_pickle=False))
        # The packed visual model predates some added fixed stem bodies. Body
        # indices after the arm therefore differ from the physics model: map
        # every fruit by its exact name, as the existing farm viewer does.
        self.pose_indices=visual_pose_indices([self.native.body(i).name for i in range(self.native.nbody)],self.visual.body_names[:-2])

    def transforms(self,q):
        with self.lock:
            self.initial['qpos'][:]=q
            poses,seat=body_poses(self.source,self.initial,self.meta,q[1],True,q[2:8],layout='side',jaw_positions=q[8:])
            self.data.qpos[:]=q;mj.mj_kinematics(self.native,self.data)
            ids=self.meta['arm_bodies']
            error=float(np.max(np.linalg.norm(poses[ids,:3,3]-self.data.xpos[ids],axis=1)))
            rotation_error=float(np.max(np.abs(poses[ids,:3,:3]-self.data.xmat[ids].reshape(-1,3,3))))
            if error>1e-8 or rotation_error>1e-8:raise ValueError('V6 sensor/display arm differs from native physics')
            poses=np.concatenate((poses[self.pose_indices],np.tile(np.eye(4),(2,1,1))))
            poses[-2,:3,3]=[0.,q[0],0.];poses[-1,:3,3]=seat
            return poses,error

    def measured_poses(self,q,harvested=()):
        with self.lock:
            poses,_=self.transforms(q)
            result={self.native.body(i).name:np.r_[self.data.xpos[i],Rotation.from_matrix(self.data.xmat[i].reshape(3,3)).as_quat()]for i in range(self.native.nbody)}
            result.update({self.visual.body(i).name:np.r_[pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat()]for i,pose in enumerate(poses)})
            return sensor_poses(result,harvested)
