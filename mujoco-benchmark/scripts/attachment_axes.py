"""Display-only world axes and plant directions at the attachment."""
import numpy as np
import mujoco as mj

def draw_axes(scene, model, data, meta, allowed_region=False, world_only=False):
    origin=np.asarray(meta['attachment_world_m'],dtype=float)
    parent=model.body(meta['attachment_parent']).id
    stem_geom=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==parent and model.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
    tip=data.geom_xmat[stem_geom].reshape(3,3)[:,2].copy()
    # All existing main stem segments progress upward in world Z.
    if tip[2]<0:tip=-tip
    branch=data.xpos[model.body('Peduncle_01').id]-data.xpos[model.body('Peduncle_00').id]
    branch/=np.linalg.norm(branch)
    vectors=[('WORLD +X',[1,0,0],[1,.12,.12,1]),('WORLD +Y',[0,1,0],[.1,1,.2,1]),('WORLD +Z',[0,0,1],[.15,.4,1,1]),('STEM TIP',tip,[1,.85,.1,1]),('BRANCH START',branch,[.1,1,1,1])]
    if world_only:vectors=vectors[:3]
    for label,axis,color in vectors:
        end=origin+(.15 if label=='STEM TIP' else .10)*np.asarray(axis)
        g=scene.geoms[scene.ngeom];scene.ngeom+=1
        mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_ARROW,np.zeros(3),origin,np.eye(3).ravel(),np.array(color,dtype=np.float32))
        mj.mjv_connector(g,mj.mjtGeom.mjGEOM_ARROW,.002,origin,end)
        marker=scene.geoms[scene.ngeom];scene.ngeom+=1
        mj.mjv_initGeom(marker,mj.mjtGeom.mjGEOM_SPHERE,np.full(3,.002),end,np.eye(3).ravel(),np.array(color,dtype=np.float32))
        marker.label=label

    if allowed_region:
        side=branch-tip*np.dot(branch,tip)
        side/=np.linalg.norm(side)
        pink=np.array([1.,.25,.75,1.],dtype=np.float32)
        def line(a,b,width=.0008):
            g=scene.geoms[scene.ngeom];scene.ngeom+=1
            mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),a,np.eye(3).ravel(),pink)
            mj.mjv_connector(g,mj.mjtGeom.mjGEOM_CAPSULE,width,a,b)
        if meta.get('preview_sector')=='world_positive_y_negative_z':
            tip=np.array([0.,1.,0.]);side=np.array([0.,0.,-1.])
            angles=np.linspace(0,np.pi/2,49)
        else:angles=np.linspace(-np.pi/2,np.pi/2,49)
        radius=meta.get('preview_sector_radius_m',.12)
        points=[origin+radius*(np.cos(t)*tip+np.sin(t)*side) for t in angles]
        for a,b in zip(points[:-1],points[1:]):line(a,b)
        line(origin,points[0]);line(origin,points[-1])
        for t in np.linspace(angles[0],angles[-1],7):
            line(origin,origin+radius*(np.cos(t)*tip+np.sin(t)*side),.00035)
