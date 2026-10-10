"""Tool-frame adapter for the existing URDF FK/least-squares IK approach.

Unlike the old ring frame, this endpoint is the actual V5.1 CAD origin. Joint
frames/limits come from the composed USD. No physical body is teleported by FK.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from pxr import UsdGeom, UsdPhysics
from motion_assets import ROBOT, TOOL


def transmission_transforms(gap):
    """CAD shape_at linkage equations, driven by measured slider gap (metres)."""
    if not .0279<=gap<=.0961:raise ValueError('Measured gap exceeds CAD mechanism travel')
    def theta(g):
        s=g/2+.018
        return np.arccos(np.clip((s*s+.025**2-.045**2)/(2*s*.025),-1,1))
    angle=theta(gap);nominal=theta(.044);pivot=np.array([0.,-.031,-.018])
    out={};t=np.eye(4);t[:3,:3]=R.from_rotvec([0,-(angle-nominal),0]).as_matrix()
    t[:3,3]=pivot-t[:3,:3]@pivot;out['crank']=t
    for side,sign in [('linkL',-1),('linkR',1)]:
        def ends(g,a):
            return np.array([sign*.025*np.cos(a),-.031,-.018+sign*.025*np.sin(a)]),np.array([sign*(g/2+.018),-.031,-.018])
        a0,b0=ends(.044,nominal);a,b=ends(gap,angle)
        change=np.arctan2((b-a)[2],(b-a)[0])-np.arctan2((b0-a0)[2],(b0-a0)[0])
        t=np.eye(4);t[:3,:3]=R.from_rotvec([0,-change,0]).as_matrix();t[:3,3]=a-t[:3,:3]@a0
        out[side]=t
    return out


class ToolKinematics:
    def __init__(self, stage):
        c=UsdGeom.XformCache()
        self.names=['farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3']
        self.chain=[]; lower=[]; upper=[]
        for name in self.names:
            p=stage.GetPrimAtPath(ROBOT+'/joints/'+name);j=UsdPhysics.Joint(p)
            bodies=[stage.GetPrimAtPath(x[0]) for x in (j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets())]
            poses=[np.asarray(c.GetLocalToWorldTransform(b)).T for b in bodies]
            if not self.chain:self.world=poses[0]
            # Authored body transforms are zero-joint poses in this robot USD.
            origin=np.linalg.inv(poses[0])@poses[1]
            axis=np.eye(3)['XYZ'.index(p.GetAttribute('physics:axis').Get())]
            q=j.GetLocalRot1Attr().Get();axis=R.from_quat([*q.GetImaginary(),q.GetReal()]).apply(axis)
            slider=p.IsA(UsdPhysics.PrismaticJoint)
            self.chain.append((origin,axis,slider,str(bodies[1].GetPath())))
            scale=1. if slider else np.pi/180
            lower.append(p.GetAttribute('physics:lowerLimit').Get()*scale)
            upper.append(p.GetAttribute('physics:upperLimit').Get()*scale)
        self.bounds=np.array([lower,upper])
        link=np.asarray(c.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT+'/link6'))).T
        tool=np.asarray(c.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/ActuatedTool/fixed'))).T
        self.offset=np.linalg.inv(link)@tool

    def frames(self,q):
        t=self.world.copy();result={}
        for value,(origin,axis,slider,path) in zip(q,self.chain):
            t=t@origin
            if slider:t[:3,3]+=t[:3,:3]@(axis*value)
            else:t[:3,:3]=t[:3,:3]@R.from_rotvec(axis*value).as_matrix()
            result[path]=t.copy()
        return result

    def fk(self,q):
        return list(self.frames(q).values())[-1]@self.offset

    def ik(self,pose,seed):
        def residual(q):
            t=self.fk(q)
            return np.r_[(t[:3,3]-pose[:3,3])*5,
                         R.from_matrix(pose[:3,:3]@t[:3,:3].T).as_rotvec(),(q-seed)*.002]
        result=least_squares(residual,np.clip(seed,*self.bounds),bounds=self.bounds,max_nfev=200)
        t=self.fk(result.x)
        return result.x,float(np.linalg.norm(t[:3,3]-pose[:3,3])),float(R.from_matrix(t[:3,:3]@pose[:3,:3].T).magnitude())


def grasp_pose(center, inward, depth=.065, pitch_deg=0.):
    """CAD +Y faces the fruit; fruit within belt height, depth measured behind nose."""
    y=np.asarray(inward,float);y[2]=0;y/=np.linalg.norm(y)
    z=np.array([0.,0.,1.]);x=np.cross(y,z)
    tilt=R.from_rotvec(x*np.deg2rad(pitch_deg))
    y=tilt.apply(y);z=tilt.apply(z)
    t=np.eye(4);t[:3,:3]=np.column_stack((x,y,z))
    # Active belt span Y=0..158 mm, vertical common interval Z=13..61 mm.
    t[:3,3]=center-t[:3,:3]@np.array([0.,.158-depth,.037])
    return t


def preparation_phases(start, qprep, preparation):
    """Continuous preparation commands; all profiles retain the physical scene.

    ``low_turn_then_lift`` delays lift travel until the base has turned. It is
    an opt-in path candidate for the known upper-tool/leaf collision during the
    original turn, not a collision-certified path or an initialization reset.
    ``tight_fold_then_turn`` reduces the turning radius before returning to the
    unchanged preparation configuration; its 44 mm gap reduces outer jaw width.
    ``tight_fold_open_approach`` retains that narrow preparation and then holds
    the staging pose for two seconds while opening to 60 mm before approach.
    """
    start, qprep = np.asarray(start, dtype=float), np.asarray(qprep, dtype=float)
    if preparation == 'direct':
        return [('prepare', 6., np.array([start, qprep]), .060, 0., 0.)]
    fold = start.copy(); fold[2:4] = qprep[2:4]
    if preparation == 'fold_then_turn':
        orient = qprep.copy(); orient[1] = start[1]
        return [('prepare_fold', 6., np.array([start, fold]), .060, 0., 0.),
                ('prepare_orient', 4., np.array([fold, orient]), .060, 0., 0.),
                ('prepare_turn', 4., np.array([orient, qprep]), .060, 0., 0.)]
    if preparation == 'low_turn_then_lift':
        orient = qprep.copy(); orient[:2] = start[:2]
        turned = qprep.copy(); turned[0] = start[0]
        return [('prepare_fold', 6., np.array([start, fold]), .060, 0., 0.),
                ('prepare_orient_low', 4., np.array([fold, orient]), .060, 0., 0.),
                ('prepare_turn_low', 4., np.array([orient, turned]), .060, 0., 0.),
                ('prepare_lift', 4., np.array([turned, qprep]), .060, 0., 0.)]
    if preparation in ('tight_fold_then_turn', 'tight_fold_open_approach'):
        tight = fold.copy(); tight[0] = qprep[0]; tight[2:4] = [1.3, -2.3]
        orient = tight.copy(); orient[4:] = qprep[4:]
        turned = orient.copy(); turned[1] = qprep[1]
        phases = [('prepare_fold', 6., np.array([start, fold]), .044, 0., 0.),
                ('prepare_tight_fold', 4., np.array([fold, tight]), .044, 0., 0.),
                ('prepare_orient_tight', 4., np.array([tight, orient]), .044, 0., 0.),
                ('prepare_turn_tight', 4., np.array([orient, turned]), .044, 0., 0.),
                ('prepare_extend', 4., np.array([turned, qprep]), .044, 0., 0.)]
        if preparation == 'tight_fold_open_approach':
            phases.append(('prepare_open', 2., np.array([qprep, qprep]), .060, 0., 0.))
        return phases
    raise ValueError('Unknown preparation profile ' + preparation)


def plan_motion(kin,center,start=None,preparation='fold_then_turn'):
    if start is None:start=np.zeros(7)
    inward=np.array(center)-np.r_[kin.world[:2,3],center[2]]
    seeds=[np.array([.05,b,s,e,w,v,0.]) for b in (-np.pi/2,np.pi/2)
           for s,e,w in [(-1.1,1.8,-.7),(.9,-1.7,.8)] for v in (-np.pi/2,np.pi/2)] + [start]
    trials=[]
    for pitch in (0.,-30.,-60.,30.):
        goal=grasp_pose(np.array(center),inward,pitch_deg=pitch)
        direction=goal[:3,1]
        prep=goal.copy();prep[:3,3]-=direction*.10
        for seed in seeds:
            q,pe,re=kin.ik(prep,seed);trials.append((pe+re*.1,q,pe,re,prep,goal,pitch))
        if min(x[0] for x in trials)<.0001:break
    _,qprep,pe,re,prep,goal,pitch=min(trials,key=lambda x:x[0])
    direction=goal[:3,1]
    if pe>.002 or re>.02:raise RuntimeError(f'preparation IK not found: position={pe} orientation={re}')
    waypoints=[];q=qprep
    for fraction in np.linspace(0,1,61):
        pose=prep.copy();pose[:3,3]+=direction*.10*fraction
        q,pe,re=kin.ik(pose,q)
        if pe>.002 or re>.02:raise RuntimeError(f'approach IK not found at {fraction}: {pe}/{re}')
        waypoints.append(q.copy())
    # Slow arm motion, followed by measured jaw closing, transport, twist, withdraw.
    prefix=preparation_phases(start,qprep,preparation)
    approach_gap=.044 if preparation=='tight_fold_then_turn' else .060
    phases=prefix+[('approach',8.,np.array(waypoints),approach_gap,0.,0.),
            ('close',3.,np.array([q,q]),.028,0.,0.),
            ('infeed',2.,np.array([q,q]),.028,-.020,-.020),
            ('twist',3.,np.array([q,q]),.028,-.015,.015),
            ('withdraw',8.,np.array(waypoints[::-1]),.028,-.005,-.005),
            ('hold',1.,np.array([qprep,qprep]),.028,0.,0.)]
    return phases,dict(prep_pose=prep.tolist(),goal_pose=goal.tolist(),start=start.tolist(),
                       prep_q=qprep.tolist(),goal_q=q.tolist(),ik_errors=[pe,re],pitch_deg=pitch,
                       preparation=preparation,approach_gap_m=approach_gap,
                       preparation_validation=('Nine offline samples per phase against source mesh proxies; no intersections with screened fixed obstacles, dynamic active-plant contact remains; no continuous/native clearance certificate'
                           if preparation in ('tight_fold_then_turn', 'tight_fold_open_approach') else 'Native collision audit required'),
                       phases=[dict(name=n,duration_s=d,waypoint_q=qs.tolist(),
                           end_gap_m=g,belt_local_y_m_s=[l,r]) for n,d,qs,g,l,r in phases],
                       scope='diagnostic IK path, contact audit during physical execution; not collision-certified planning')
