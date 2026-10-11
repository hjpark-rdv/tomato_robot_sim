"""FCL screening of the composed RB5 and actuated tool, including jaw travel.

Uses the same fixed-body/adjacent-joint exclusion convention as rl/pose_collision.
Robot convexHull colliders use hulls. Each tool part uses its convex envelope
as a conservative planning screen; native decomposition/contact validation is
still required. This does not change any simulator collision geometry.
No environment collider or contact policy is edited by this module.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def require_robot_self_collisions(stage):
    prim=stage.GetPrimAtPath('/World/HarvestRobot/Robot/root_joint')
    value=prim.GetAttribute('physxArticulation:enabledSelfCollisions').Get() if prim else None
    if value is not True:
        raise ValueError('Harvest motion requires robot self-collisions. Rebuild a checked horizontal-ready assembly; disabled/missing self-collision state is not valid.')
    return True


def excluded_body_pairs(bodies, joints):
    parent={b:b for b in bodies}
    def group(b):
        while parent[b]!=b:b=parent[b]
        return b
    for a,b,fixed,disabled in joints:
        if a in parent and b in parent and fixed:parent[group(b)]=group(a)
    adjacent={frozenset((group(a),group(b))) for a,b,_,disabled in joints
              if disabled and a in parent and b in parent}
    return {frozenset((a,b)) for i,a in enumerate(bodies) for b in bodies[:i]
            if group(a)==group(b) or frozenset((group(a),group(b))) in adjacent}


def _aabb_candidates_xyz(low, high, pairs, clearance_m):
    """Keep the original pair order using the same inclusive AABB comparisons.

    Applying each axis to the surviving pairs avoids allocating all six
    (pair_count, 3) bound arrays. This is the identical conjunction used by
    the original broadcast screen; no geometry, clearance, or FCL query is
    approximated. Nonfinite bounds retain their original comparison behavior.
    """
    candidates=pairs
    for axis in range(3):
        a,b=candidates.T
        candidates=candidates[(low[a,axis]<=high[b,axis]+clearance_m)&
                              (low[b,axis]<=high[a,axis]+clearance_m)]
    return candidates


class MotionSelfCollision:
    def __init__(self,stage,kin=None,clearance_m=0.,*,native_tool_cache=None,assembly=None):
        import fcl
        import trimesh
        from pxr import Usd,UsdGeom,UsdPhysics
        self.fcl=fcl;self.kin=kin;self.shapes=[];self.objects=[];self.rest={}
        if not np.isfinite(clearance_m) or clearance_m<0:raise ValueError('Clearance must be finite and nonnegative')
        self.clearance=float(clearance_m)
        native_pieces={};native_import=None
        if native_tool_cache is not None:
            from native_tool_hulls import load_native_tool_hulls
            native_pieces,native_import=load_native_tool_hulls(stage,native_tool_cache,assembly=assembly)
        cache=UsdGeom.XformCache();joints=[]
        for root in ('/World/HarvestRobot/Robot','/World/ActuatedTool'):
            for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.rest[str(p.GetPath())]=np.asarray(cache.GetLocalToWorldTransform(p)).T
                if p.IsA(UsdPhysics.Joint):
                    j=UsdPhysics.Joint(p);a=j.GetBody0Rel().GetTargets();b=j.GetBody1Rel().GetTargets()
                    if a and b:joints.append((str(a[0]),str(b[0]),p.IsA(UsdPhysics.FixedJoint),j.GetCollisionEnabledAttr().Get() is False))
                if not p.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:continue
                owner=p
                while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
                if not owner:raise ValueError('Missing collider body '+str(p.GetPath()))
                local=np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(owner)).T)@np.asarray(cache.GetLocalToWorldTransform(p)).T
                if str(p.GetPath()) in native_pieces:
                    # SDK vertices are already in this exact rigid-owner frame.
                    # Keep every native piece; do not apply `local` again or
                    # collapse their union into the old full-part envelope.
                    for piece_index,v in enumerate(native_pieces[str(p.GetPath())]):
                        hull=trimesh.convex.convex_hull(v)
                        geo=fcl.Convex(hull.vertices,len(hull.faces),np.column_stack((np.full(len(hull.faces),3),hull.faces)).reshape(-1))
                        low,high=v.min(0),v.max(0)
                        corners=np.array([[x,y,z] for x in (low[0],high[0]) for y in (low[1],high[1]) for z in (low[2],high[2])])
                        self.shapes.append(dict(path=str(p.GetPath()),body=str(owner.GetPath()),name=p.GetCustomDataByKey('sourceName'),
                            corners=corners,geometry_local=np.eye(4),native_piece_index=piece_index))
                        self.objects.append(fcl.CollisionObject(geo))
                    continue
                approximation=p.GetAttribute('physics:approximation').Get()
                if approximation=='convexDecomposition':approximation='convexHull'
                geo=None;geometry_local=np.eye(4)
                if p.IsA(UsdGeom.Mesh):
                    m=UsdGeom.Mesh(p);counts=np.asarray(m.GetFaceVertexCountsAttr().Get())
                    if not np.all(counts==3):raise ValueError('Self collider must be triangulated')
                    v=np.asarray(m.GetPointsAttr().Get(),float)
                    faces=np.asarray(m.GetFaceVertexIndicesAttr().Get(),np.int32).reshape(-1,3)
                elif p.IsA(UsdGeom.Cube):
                    half=float(UsdGeom.Cube(p).GetSizeAttr().Get())/2
                    v=np.array([[x,y,z] for x in (-half,half) for y in (-half,half) for z in (-half,half)])
                    approximation='convexHull'
                elif p.IsA(UsdGeom.Cylinder):
                    cylinder=UsdGeom.Cylinder(p);axis=str(cylinder.GetAxisAttr().Get())
                    align=np.eye(4)
                    if axis!='Z':align[:3,:3]=Rotation.from_euler('y' if axis=='X' else 'x',90,degrees=True).as_matrix()
                    local=local@align;scale=np.linalg.norm(local[:3,:3],axis=0)
                    if not np.isclose(scale[0],scale[1]):raise ValueError('Elliptical cylinder is unsupported')
                    radius=float(cylinder.GetRadiusAttr().Get())*scale[0];height=float(cylinder.GetHeightAttr().Get())*scale[2]
                    geometry_local=local.copy();geometry_local[:3,:3]/=scale
                    geo=fcl.Cylinder(radius,height)
                    v=np.array([[x,y,z] for x in (-radius,radius) for y in (-radius,radius) for z in (-height/2,height/2)])
                    local=geometry_local
                else:raise ValueError('Unsupported self collider '+str(p.GetPath()))
                v=v@local[:3,:3].T+local[:3,3]
                if geo is not None:pass
                elif approximation=='convexHull':
                    hull=trimesh.convex.convex_hull(v)
                    geo=fcl.Convex(hull.vertices,len(hull.faces),np.column_stack((np.full(len(hull.faces),3),hull.faces)).reshape(-1))
                else:
                    geo=fcl.BVHModel();geo.beginModel(len(v),len(faces));geo.addSubModel(v,faces);geo.endModel()
                low,high=v.min(0),v.max(0)
                corners=np.array([[x,y,z] for x in (low[0],high[0]) for y in (low[1],high[1]) for z in (low[2],high[2])])
                self.shapes.append(dict(path=str(p.GetPath()),body=str(owner.GetPath()),name=p.GetCustomDataByKey('sourceName'),corners=corners,geometry_local=geometry_local))
                self.objects.append(fcl.CollisionObject(geo))
        self.bodies=list(self.rest);self.body_ids=np.array([self.bodies.index(s['body']) for s in self.shapes])
        excluded=excluded_body_pairs(self.bodies,joints)
        self.pairs=np.array([(i,j) for i,a in enumerate(self.shapes) for j,b in enumerate(self.shapes[:i])
            if a['body']!=b['body'] and frozenset((a['body'],b['body'])) not in excluded],int)
        self.corners=np.array([s['corners'] for s in self.shapes]);self.queries=0
        self.metadata=dict(shapes=len(self.shapes),checked_shape_pairs=len(self.pairs),clearance_m=self.clearance,
            excluded_body_pairs=[sorted(p) for p in sorted(excluded,key=lambda p:sorted(p))],
            exclusions='Same rigid/fixed component; exact disabled USD joint adjacency, matching existing rl/pose_collision convention',
            method='FCL robot colliders + conservative per-part tool convex envelopes, discrete samples; native hull cooking remains separate; no physical collider changes')
        if native_import is not None:
            self.metadata['native_tool_hull_import']=native_import
            self.metadata['method']='FCL robot colliders + exact hash-bound native pieces for scoped tool meshes; remaining per-part tool envelopes unchanged; no physical collider changes'

    def poses(self,q,gap=.044,rail_position_m=0.):
        if self.kin is None:raise ValueError('Kinematics required')
        if not np.isfinite(rail_position_m):raise ValueError('Rail position must be finite')
        poses=dict(self.rest);poses.update(self.kin.frames(q));tool=self.kin.fk(q)
        for name,offset in [('fixed',0.),('L',-(gap-.044)/2),('L_belt',-(gap-.044)/2),('R',(gap-.044)/2),('R_belt',(gap-.044)/2)]:
            t=tool.copy();t[:3,3]+=t[:3,0]*offset;poses['/World/ActuatedTool/'+name]=t
        if rail_position_m:
            # One world-Y translation for the complete moving platform, including
            # base/chassis rest bodies not present in the seven-DOF FK chain.
            # Actual recorded poses are already translated and never use this.
            poses={name:t.copy() for name,t in poses.items()}
            for t in poses.values():t[1,3]+=rail_position_m
        return poses

    def check(self,q=None,gap=.044,*,poses=None,max_hits=1,rail_position_m=0.):
        self.queries+=1
        if poses is not None and rail_position_m:
            raise ValueError('Measured poses already include rail translation')
        poses=self.poses(q,gap,rail_position_m) if poses is None else poses
        transforms=np.array([poses[b] for b in self.bodies])[self.body_ids]
        world=np.einsum('nij,nkj->nki',transforms[:,:3,:3],self.corners)+transforms[:,None,:3,3]
        low,high=world.min(1),world.max(1)
        candidates=_aabb_candidates_xyz(low,high,self.pairs,self.clearance)
        updated=set();hits=[]
        for i,j in candidates:
            for n in (i,j):
                if n not in updated:
                    t=transforms[n]@self.shapes[n]['geometry_local']
                    self.objects[n].setTransform(self.fcl.Transform(t[:3,:3],t[:3,3]));updated.add(n)
            result=self.fcl.CollisionResult()
            collision=bool(self.fcl.collide(self.objects[i],self.objects[j],self.fcl.CollisionRequest(),result))
            distance=None
            if not collision and self.clearance:
                distance=self.fcl.distance(self.objects[i],self.objects[j],self.fcl.DistanceRequest(),self.fcl.DistanceResult())
            if collision or distance is not None and distance<self.clearance:
                hits.append(dict(a=self.shapes[i]['path'],b=self.shapes[j]['path'],a_name=self.shapes[i]['name'],b_name=self.shapes[j]['name'],intersection=collision,distance_m=distance,required_planning_clearance_m=self.clearance))
                if len(hits)>=max_hits:break
        return hits


def check_plan(checker,plan,hz=120,*,cache_exact_repeats=True):
    """Check every nominal control tick, including jaw interpolation and endpoints."""
    if hz<=0 or not plan.get('phases'):raise ValueError('Nonempty plan and positive Hz are required')
    from motion_continuation import initial_plan_rail_position
    previous_gap=.044;previous_rail=initial_plan_rail_position(plan);rows=[];total=0
    # This checker is an immutable geometry query. Reuse only bit-identical
    # inputs within this single plan; no rounded keys or skipped timestamps.
    # Arbitrary/stateful checker implementations retain their original calls.
    memo={} if cache_exact_repeats and type(checker) is MotionSelfCollision else None
    query_calls=0;cache_hits=0
    rail_plan=any('rail_waypoint_positions_m' in phase for phase in plan['phases'])
    for p in plan['phases']:
        qs=np.asarray(p['waypoint_q']);duration=p['duration_s'];hits=[]
        if qs.ndim!=2 or qs.shape[1]!=7 or len(qs)<2 or not np.isfinite(qs).all() or not np.isfinite([duration,p['end_gap_m']]).all() or duration<=0:
            raise ValueError('Malformed/nonfinite self-collision plan')
        rail=np.asarray(p.get('rail_waypoint_positions_m',[previous_rail]*len(qs)),dtype=float)
        if rail.shape!=(len(qs),) or not np.isfinite(rail).all() or abs(float(rail[0])-previous_rail)>1e-9:
            raise ValueError('Malformed, nonfinite or discontinuous rail plan')
        for u in np.linspace(0,1,int(np.ceil(duration*hz))+1):
            smooth=u*u*(3-2*u);f=smooth*(len(qs)-1);lo=min(int(f),len(qs)-2)
            q=qs[lo]*(1-(f-lo))+qs[lo+1]*(f-lo)
            gap=previous_gap+(p['end_gap_m']-previous_gap)*smooth
            rail_value=float(rail[lo]*(1-(f-lo))+rail[lo+1]*(f-lo))
            key=(q.tobytes(),np.float64(gap).tobytes(),
                 np.float64(rail_value).tobytes() if rail_plan else None)
            if memo is not None and key in memo:
                hits=memo[key];cache_hits+=1
            else:
                hits=checker.check(q,gap,rail_position_m=rail_value) if rail_plan else checker.check(q,gap)
                query_calls+=1
                if memo is not None:memo[key]=hits
            total+=1
            if hits:break
        rows.append(dict(phase=p['name'],clear=not hits,first_hit_fraction=float(u) if hits else None,hits=hits))
        if hits:break
        previous_gap=p['end_gap_m']
        previous_rail=float(rail[-1])
    return dict(passed=all(r['clear'] for r in rows),complete=len(rows)==len(plan['phases']) and not hits,
                samples=total,hz=hz,phases=rows,geometry=checker.metadata,training_eligible=False,
                query_cache=dict(enabled=memo is not None,exact_configuration_queries=query_calls,
                                 reused_exact_configuration_queries=cache_hits,
                                 rounded_configuration_keys=False,all_sample_times_evaluated=True))


def main():
    p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);p.add_argument('--plan',type=Path)
    p.add_argument('--states',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--hz',type=int,default=120)
    p.add_argument('--native-tool-cache',type=Path)
    a=p.parse_args()
    if bool(a.plan)==bool(a.states):p.error('Exactly one of --plan and --states is required')
    from pxr import Usd
    from motion_kinematics import ToolKinematics
    scene=a.assembly/'scene.usda';stage=Usd.Stage.Open(str(scene));checker=MotionSelfCollision(stage,ToolKinematics(stage),clearance_m=.002 if a.plan else 0.,native_tool_cache=a.native_tool_cache,assembly=a.assembly)
    manifest=json.loads((a.assembly/'motion_manifest.json').read_text())
    if manifest['scene_sha256']!=hashlib.sha256(scene.read_bytes()).hexdigest():raise ValueError('Assembly SHA mismatch')
    if a.plan:report=check_plan(checker,json.loads(a.plan.read_text()),a.hz)
    else:
        # NpzFile decompresses on every [] access: load the long history once.
        with np.load(a.states) as d:
            names=d['body_paths'].tolist();history=d['poses'];stamps=d['time'];phases=d['phase']
        indices=[names.index(b) for b in checker.bodies];rows=[]
        for i in range(len(stamps)):
            measured=history[i,indices];t=np.repeat(np.eye(4)[None],len(indices),axis=0)
            t[:,:3,:3]=Rotation.from_quat(measured[:,3:]).as_matrix();t[:,:3,3]=measured[:,:3]
            hits=checker.check(poses=dict(zip(checker.bodies,t)))
            if hits:rows.append(dict(t=float(stamps[i]),phase=str(phases[i]),hits=hits))
        report=dict(passed=not rows,complete=True,samples=len(stamps),colliding_samples=rows,geometry=checker.metadata,training_eligible=False)
    report.update(scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),input_sha256=hashlib.sha256((a.plan or a.states).read_bytes()).hexdigest(),physics_rerun=False)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('passed','complete','samples')}),flush=True)
    return 0 if report['passed'] else 2


def audit_external(assembly,output,*,plan=None,states=None,hz=120,native_tool_cache=None):
    """Use the existing asset Python for FCL; keep Isaac's Python/ABI separate."""
    import os
    import subprocess
    python=Path(__file__).resolve().parents[1]/'.asset-venv/bin/python'
    env=os.environ.copy()
    for key in ('PYTHONHOME','PYTHONPATH','PXR_PLUGINPATH_NAME'):env.pop(key,None)
    command=[str(python),str(Path(__file__).resolve()),str(assembly),'--output',str(output),'--hz',str(hz)]
    command+=['--plan',str(plan)] if plan else ['--states',str(states)]
    if native_tool_cache is not None:command+=['--native-tool-cache',str(native_tool_cache)]
    result=subprocess.run(command,env=env,capture_output=True,text=True)
    output.with_suffix('.log').write_text(result.stdout+result.stderr)
    if result.returncode not in (0,2) or not output.exists():
        raise RuntimeError('Self-collision audit could not complete: '+str(output.with_suffix('.log')))
    report=json.loads(output.read_text())
    if not report.get('complete') or not report.get('passed'):
        raise RuntimeError('Self-collision audit blocked motion: '+str(output))
    return report


if __name__=='__main__':raise SystemExit(main())
