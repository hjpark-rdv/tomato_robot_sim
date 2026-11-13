"""Compare guarded V5 starting postures on a shared, fixed workspace.

Reachability is local IK plus a complete jaw cycle. Starting-posture coverage is
the set of distinct targets connected by densely checked paths in a finite
search graph. The chosen graph centre is the best tested start, not a global
reachability proof. Existing checked routes seed the graph so a new search does
not discard the current coverage merely because a local connector times out.
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path
import time

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_scene_chunks import PackedScene
from preview_robot_mount import body_poses
from preview_workspace_motion import CAPTURE_CENTER, capture, collision_model
from preview_grip_workspace import (
    BoxObstacles, GripPlanner, OPEN_JAWS, CLOSED_JAWS, PlanningError,
    approach_path, checked_edge, finish_grip_motion, joint_connection,
    layout_boxes, nearest_joint_turns, segment_duration,
)
from preview_conveyor_bin import conveyor_grip_parts
from preview_posture import POSTURE_POLICY


def entry_rotation(nominal, side, yaw, pitch, roll):
    expected=Rotation.from_euler('z',180,degrees=True).as_matrix()@nominal if side=='left' else nominal
    forward=expected@np.array([0.,1.,0.])
    axis=np.cross(forward,[0.,0.,1.]);axis/=np.linalg.norm(axis)
    orient=(Rotation.from_euler('z',yaw,degrees=True).as_matrix()@
        Rotation.from_rotvec(axis*math.radians(pitch)).as_matrix()@expected@
        Rotation.from_euler('y',roll,degrees=True).as_matrix())
    deviation=math.degrees(math.acos(float(np.clip(np.dot(forward,orient[:,1]),-1.,1.))))
    if deviation>15.+1e-6:raise ValueError('Entry orientation exceeds the 15 degree cone')
    return orient


def coarse_edge(planner,a,b):
    """Screen only; an accepted graph edge still requires checked_edge()."""
    count=max(1,math.ceil(abs(a[0]-b[0])/.01),math.ceil(np.linalg.norm(a[1:7]-b[1:7])/.04))
    for u in np.linspace(0.,1.,count+1):
        q=planner.initial.copy();q[1:10]=a+u*(b-a)
        if not planner.valid_full(q):return False
    return True


class RigidToolProbe:
    """Prune deeply blocked tool poses independently of arm IK and lift height.

    Only private MjData geometry transforms are changed. Exact native distance
    queries bypass the collision broad phase; accepted poses still undergo full
    FK/collision and dense path validation. Six millimetres of slack avoids
    pruning a different result solely because of the IK stopping tolerance.
    """
    def __init__(self,model,q,obstacles,tool_id):
        self.model=model;self.data=mj.MjData(model)
        self.tools=np.array([g for g in range(model.ngeom)
            if model.body(int(model.geom_bodyid[g])).name.startswith(('Tool_','Robot_link6'))
            and (model.geom_contype[g]or model.geom_conaffinity[g])],dtype=int)
        self.obstacles=[]
        for row in obstacles.rows:
            box=row['box']
            if box['name'].startswith('bin/'):
                self.obstacles.append((row['gid'],box['name'],0.))
            elif box['name'].startswith('lift/')and box['dimensions'][2]>.8:
                self.obstacles.append((row['gid'],box['name'],.10))
        self.frames=[]
        for jaws in (OPEN_JAWS,CLOSED_JAWS):
            self.data.qpos[:]=q;self.data.qpos[8:10]=jaws
            mj.mj_kinematics(model,self.data)
            rot=self.data.xmat[tool_id].reshape(3,3);pos=self.data.xpos[tool_id]
            local=(self.data.geom_xpos[self.tools]-pos)@rot
            local_rot=np.einsum('ij,njk->nik',rot.T,self.data.geom_xmat[self.tools].reshape(-1,3,3))
            self.frames.append((local,local_rot))
        self.obstacle_ids=np.array([g for g,_,_ in self.obstacles],int)
        rots=self.data.geom_xmat[self.obstacle_ids].reshape(-1,3,3)
        self.centres=self.data.geom_xpos[self.obstacle_ids]+np.einsum('nij,nj->ni',rots,model.geom_aabb[self.obstacle_ids,:3])
        self.extents=np.einsum('nij,nj->ni',np.abs(rots),model.geom_aabb[self.obstacle_ids,3:])
        self.margins=np.array([margin for _,_,margin in self.obstacles])

    def rejection(self,position,rotation):
        for local,local_rot in self.frames:
            rots=np.einsum('ij,njk->nik',rotation,local_rot)
            points=local@rotation.T+position
            centres=points+np.einsum('nij,nj->ni',rots,self.model.geom_aabb[self.tools,:3])
            extents=np.einsum('nij,nj->ni',np.abs(rots),self.model.geom_aabb[self.tools,3:])
            low=np.maximum(0.,np.abs(centres[:,None]-self.centres[None])-extents[:,None]-self.extents[None])
            possible=np.linalg.norm(low,axis=2)<self.margins[None]+.001
            self.data.geom_xpos[self.tools]=points
            self.data.geom_xmat[self.tools]=rots.reshape(-1,9)
            for ti,oi in zip(*np.where(possible)):
                tool=int(self.tools[ti]);obstacle,name,margin=self.obstacles[oi]
                distance=mj.mj_geomDistance(self.model,self.data,tool,obstacle,.11,None)
                if distance<margin-.006:
                    return dict(obstacle=name,tool_geom=self.model.geom(tool).name,
                                distance_m=float(distance),required_m=margin)
        return None


class Forest:
    def __init__(self,count):
        self.parent=list(range(count))
        self.edges=[[] for _ in range(count)]

    def root(self,i):
        while self.parent[i]!=i:
            self.parent[i]=self.parent[self.parent[i]];i=self.parent[i]
        return i

    def join(self,a,b,points):
        if self.root(a)==self.root(b):return False
        self.parent[self.root(a)]=self.root(b)
        cost=segment_duration(points)
        self.edges[a].append((b,points,cost));self.edges[b].append((a,points[::-1].copy(),cost))
        return True

    def distances(self,start):
        distance={start:0.};queue=[(0.,start)]
        while queue:
            d,i=heapq.heappop(queue)
            if d!=distance[i]:continue
            for j,_,cost in self.edges[i]:
                if j not in distance or d+cost<distance[j]:
                    distance[j]=d+cost;heapq.heappush(queue,(d+cost,j))
        return distance


def rank_starts(nodes,forest,rows):
    ranks=[]
    for i,node in enumerate(nodes):
        distance=forest.distances(i)
        targets={}
        for j,d in distance.items():
            for cycle in nodes[j]['cycles']:
                k=cycle['index']
                if k not in targets or d<targets[k][0]:targets[k]=(d,j)
        counts={side:sum(rows[k]['side']==side for k in targets)for side in ('right','left')}
        if not targets:continue
        ranks.append(dict(node=i,unique_targets=len(targets),by_side=counts,
            maximum_route_s=max(d for d,_ in targets.values()),
            mean_route_s=float(np.mean([d for d,_ in targets.values()])),
            lift_m=float(node['state'][0]),elbow_branch=int(np.sign(node['state'][3])),
            arm_positions_rad=node['state'][1:7].tolist()))
    return sorted(ranks,key=lambda r:(-int(min(r['by_side'].values())>0),-r['unique_targets'],
        -min(r['by_side'].values()),r['maximum_route_s'],r['mean_route_s'],r['node']))


def neutral_seeds(initial,side,branch,roll):
    base=float(initial[2])-(math.pi if side=='left'else 0.)
    for shoulder in ([135.,110.,80.] if branch<0 else [-135.,-110.,-80.]):
        elbow=branch*100.
        wrist=(roll-90.-shoulder-elbow+180.)%360.-180.
        yield np.r_[base,np.deg2rad([shoulder,elbow,wrist,90.]),
                    base+math.pi-(math.pi if side=='left'else 0.)]


def canonical_pose(state,home,bounds,branch):
    reference=home.copy()
    # A positive-elbow family must not inherit the opposite family's shoulder
    # winding (e.g. -110 becomes +250 degrees when compared with a +113 home).
    reference[2]=abs(reference[2])*(-branch)
    reference[3]=abs(reference[3])*branch
    return nearest_joint_turns(state,reference,bounds)


def make_cycle(planner,state,orient,centre,index,side,yaw=0.,pitch=0.,roll=0.,approach=None):
    approach=approach_path(planner,state,orient,centre,.04) if approach is None else approach.copy()
    closed=approach[-1].copy();closed[7:]=CLOSED_JAWS
    if not checked_edge(planner,approach[-1],closed):raise PlanningError('Closing jaws collides')
    for a,b in zip(approach[:-1],approach[1:]):
        if not checked_edge(planner,a,b):raise PlanningError('Local approach violates working limits')
    if np.sign(state[3])!=np.sign(approach[-1,3]):raise PlanningError('Approach IK changed elbow branch')
    return dict(index=index,side=side,state=approach[-1].copy(),rotation=orient.copy(),approach=approach,
                closed=closed,yaw_deg=yaw,pitch_deg=pitch,roll_deg=roll)


def optimize_start(preview,seed_plan,output,*,resume_candidates=None):
    started=time.monotonic();preview=Path(preview);seed_plan=Path(seed_plan);output=Path(output)
    output.mkdir(parents=True,exist_ok=False)
    meta=json.loads((preview/'preview.json').read_text());old=json.loads(seed_plan.read_text())
    if old['posture_policy']!=POSTURE_POLICY:raise ValueError('Start search requires the guarded baseline')
    source=mj.MjModel.from_binary_path(str(Path(meta['assets'])/'inputs/input_model.mjb'))
    initial=dict(np.load(Path(meta['assets'])/'inputs/initial_state.npz'))
    initial['qpos'][:]=old['initial_positions'];q=initial['qpos'].copy()
    scene=PackedScene(preview/'preview_scene.npz')
    displayed,_=body_poses(source,initial,meta,q[1],True,q[2:8],.35,'conveyor')
    # The world goal frame is intentionally independent of the candidate home.
    nominal=np.array(old.get('nominal_tool_rotation_world',displayed[source.body('Tool_fixed').id,:3,:3]))
    parts=conveyor_grip_parts(meta['layouts']['conveyor']['conveyor'])
    work=output/'search_collision';work.mkdir()
    model,installation=collision_model(source,q,meta,displayed,work)
    boxes=layout_boxes(scene,meta,displayed,parts,q[1]);obstacles=BoxObstacles(model,q,boxes)
    planner=GripPlanner(model,initial,obstacles,posture_guard=True,model_sha256=installation['collision_configuration_sha256'])
    rigid=RigidToolProbe(model,q,obstacles,planner.tool_id)
    baseline=json.loads((seed_plan.parent/old['coverage_file']).read_text())['grid']
    rows=[dict(side=r['side'],target_capture_m=r['target_capture_m'],reachable=False,grip_cycle_checked=False)for r in baseline]
    regions={k:np.array(v)for k,v in old['regions'].items()}
    arrays=np.load(seed_plan.parent/old['trajectory_file']);stored=arrays['positions'];offsets=arrays['segment_offsets']
    nodes=[];node_ids={};baked=[]
    def node(state,cycle=None):
        key=tuple(np.round(state,10))
        if key not in node_ids:node_ids[key]=len(nodes);nodes.append(dict(state=np.array(state),cycles=[]))
        i=node_ids[key]
        if cycle is not None:nodes[i]['cycles'].append(cycle)
        return i
    # Keep the actual successful old routes, including their intermediate hubs.
    # No old edge is trusted: every retained edge is sampled again below.
    for k,s in enumerate(old['segments']):
        points=stored[offsets[k]:offsets[k+1]]
        if s['phase']=='transfer':
            for a,b in zip(points[:-1],points[1:]):baked.append((node(a),node(b)))
        elif s['phase']=='approach':
            index=s['coverage_index'];r=baseline[index]
            orient=entry_rotation(nominal,r['side'],r.get('yaw_deg',0.),r.get('pitch_deg',0.),r.get('roll_deg',0.))
            cycle=make_cycle(planner,points[-1],orient,np.array(r['target_capture_m']),index,r['side'],
                r.get('yaw_deg',0.),r.get('pitch_deg',0.),r.get('roll_deg',0.),approach=points)
            node(points[0],cycle);rows[index].update(r,grip_cycle_checked=False)
    home=q[1:10].copy();home[7:]=OPEN_JAWS;baseline_home=node(home)
    solved={(-1 if c['state'][3]<0 else 1,c['index'])for n in nodes for c in n['cycles']}
    if resume_candidates is not None:
        candidates=json.loads(Path(resume_candidates).read_text())['candidates']
        targets=np.array([r['target_capture_m']for r in rows])
        recovered=0
        for candidate in candidates:
            state=np.r_[candidate['lift_m'],candidate['arm_positions_rad'],OPEN_JAWS]
            full=q.copy();full[1:10]=state
            if not planner.valid_full(full):continue
            orient=planner.data.xmat[planner.tool_id].reshape(3,3).copy()
            predicted=capture(planner.data,planner.tool_id)+.04*orient[:,1]
            index=int(np.argmin(np.linalg.norm(targets-predicted,axis=1)))
            if np.linalg.norm(targets[index]-predicted)>.003:continue
            side=rows[index]['side'];branch=int(np.sign(state[3]))
            if (branch,index)in solved:continue
            options=[]
            for pitch in (0.,-14.,14.):
                for roll in (0.,180.):
                    for yaw in ((0.,-14.,14.) if pitch == 0 else (0.,)):
                        rotation=entry_rotation(nominal,side,yaw,pitch,roll)
                        options.append((np.linalg.norm(rotation-orient),rotation,yaw,pitch,roll))
            error,rotation,yaw,pitch,roll=min(options,key=lambda c:c[0])
            if error>.03:continue
            state=canonical_pose(state,home,planner.bounds,branch)
            try:
                state7=planner.ik_at_lift(dict(position=targets[index]-rotation@CAPTURE_CENTER,
                    quaternion_xyzw=Rotation.from_matrix(rotation).as_quat()),state[0],state[1:7],branch)
                cycle=make_cycle(planner,np.r_[state7,OPEN_JAWS],rotation,targets[index],index,side,yaw,pitch,roll)
                node(cycle['approach'][0],cycle);solved.add((branch,index));recovered+=1
                actual=capture(planner.data,planner.tool_id)
                rows[index].update(reachable=True,positions=cycle['state'].tolist(),
                    measured_capture_m=actual.tolist(),lift_height_m=float(state[0]),
                    yaw_deg=yaw,pitch_deg=pitch,roll_deg=roll)
                # Offer both actual bounded base turns. They are separate graph
                # nodes, never an instantaneous jump between equivalent angles.
                for turn in (-2*math.pi,2*math.pi):
                    alias=cycle['state'].copy();alias[1]+=turn
                    if not planner.bounds[2,0]<=alias[1]<=planner.bounds[2,1]:continue
                    variant=make_cycle(planner,alias,rotation,targets[index],index,side,yaw,pitch,roll)
                    node(variant['approach'][0],variant)
            except PlanningError:continue
        print(f'Recovered {recovered} local targets with canonical joint turns from {len(candidates)} cached candidates',flush=True)
    previous={}
    attempts=0;blocked={}
    for index,row in enumerate(rows if resume_candidates is None else []):
        side=row['side'];centre=np.array(row['target_capture_m'])
        for branch in (-1,1):
            if (branch,index)in solved:continue
            found=False
            # A complete cycle, not the first collision-free endpoint, wins.
            rolls=(180.,0.) if centre[2]>=1. else (0.,180.)
            orientations=[(pitch,roll,0.)for pitch in (0.,-14.,14.)for roll in rolls]
            orientations += [(0.,roll,yaw)for roll in rolls for yaw in (-14.,14.)]
            for pitch,roll,yaw in orientations:
                orient=entry_rotation(nominal,side,yaw,pitch,roll)
                key=(index,pitch,roll,yaw)
                if key not in blocked:
                    blocked[key]=rigid.rejection(centre-orient@CAPTURE_CENTER,orient)
                if blocked[key]is not None:continue
                lift_hint=float(np.clip(centre[2]-(.125 if roll==0 else .485),0.,.75))
                lifts=list(dict.fromkeys([lift_hint,.75,.60,.45,.30,.15,0.]))
                seeds=list(neutral_seeds(q,side,branch,roll))
                if (side,branch,roll)in previous:seeds.insert(0,previous[side,branch,roll].copy())
                # Two seeds at each lift keep this explicit search finite.
                for lift in lifts:
                    for seed in seeds[:2]:
                        attempts+=1
                        try:
                            state7=planner.ik_at_lift(dict(position=centre-orient@CAPTURE_CENTER,
                                quaternion_xyzw=Rotation.from_matrix(orient).as_quat()),lift,seed,branch)
                            state=np.r_[state7,OPEN_JAWS]
                            measured=capture(planner.data,planner.tool_id)
                            if np.any(measured<regions[side][0])or np.any(measured>regions[side][1]):continue
                            row.update(reachable=True,positions=state.tolist(),measured_capture_m=measured.tolist(),
                                lift_height_m=float(state[0]),yaw_deg=yaw,pitch_deg=pitch,roll_deg=roll,
                                ik_error_m=float(np.linalg.norm(measured-centre)))
                            cycle=make_cycle(planner,state,orient,centre,index,side,yaw,pitch,roll)
                            node(cycle['approach'][0],cycle);solved.add((branch,index))
                            previous[side,branch,roll]=state[1:7].copy();found=True;break
                        except PlanningError:pass
                    if found:break
                if found:break
        if (index+1)%35==0:
            print(f'Start search {index+1}/{len(rows)}: {len(solved)} local cycles, {attempts} IK probes, {sum(v is not None for v in blocked.values())} blocked orientations',flush=True)
            (output/'search_progress.json').write_text(json.dumps(dict(targets_tested=index+1,
                targets_total=len(rows),local_cycles=len(solved),ik_probes=attempts))+'\n')
    # Sparse neighbours are a search heuristic; only exact checked edges enter
    # the final forest. Coarse reach counts are never presented as verified.
    states=np.array([n['state']for n in nodes]);forest=Forest(len(nodes))
    pairs={tuple(sorted(pair))for pair in baked if pair[0]!=pair[1]}
    for i,a in enumerate(states):
        distances=np.linalg.norm((states[:,:7]-a[:7])*np.r_[4.,np.ones(6)],axis=1)
        same=np.sign(states[:,3])==np.sign(a[3]);distances[~same]=np.inf;distances[i]=np.inf
        for j in np.argsort(distances)[:10]:
            if np.isfinite(distances[j]):pairs.add(tuple(sorted((i,int(j)))))
    baked_set={tuple(sorted(p))for p in baked}
    ordered=sorted(pairs,key=lambda pair:(pair not in baked_set,
        segment_duration(states[list(pair)])))
    coarse_passes=0;fine_passes=0
    for count,(a,b)in enumerate(ordered):
        if forest.root(a)==forest.root(b):continue
        if not coarse_edge(planner,states[a],states[b]):continue
        coarse_passes+=1
        if checked_edge(planner,states[a],states[b]):forest.join(a,b,states[[a,b]]);fine_passes+=1
        if count%100==0:print(f'Start graph {count}/{len(ordered)}: {fine_passes} certified links',flush=True)
    # Limited RRT bridges may connect sparse components of the same elbow sign.
    bridge_attempts=0
    for branch in (-1,1):
        for _ in range(8):
            ranks=[r for r in rank_starts(nodes,forest,rows)if r['elbow_branch']==branch]
            if not ranks:break
            best=ranks[0]['node'];component=forest.root(best)
            other=[i for i in range(len(nodes))if forest.root(i)!=component and np.sign(states[i,3])==branch]
            inside=[i for i in range(len(nodes))if forest.root(i)==component]
            if not other:break
            candidates=sorted((float(np.linalg.norm((states[a,:7]-states[b,:7])*np.r_[4.,np.ones(6)])),a,b)
                for a in inside for b in other)
            joined=False
            for _,a,b in candidates[:3]:
                bridge_attempts+=1
                try:
                    path,_=joint_connection(planner,states[a],states[b])
                    if all(checked_edge(planner,x,y)for x,y in zip(path[:-1],path[1:])):
                        forest.join(a,b,path);joined=True;break
                except PlanningError:pass
            if not joined:break
    ranks=rank_starts(nodes,forest,rows)
    winner=ranks[0];root=winner['node'];distance=forest.distances(root)
    chosen={}
    for i,d in distance.items():
        for cycle in nodes[i]['cycles']:
            k=cycle['index']
            if k not in chosen or d<chosen[k][0]:chosen[k]=(d,i,cycle)
    if len(chosen)<sum(s['complete_grip_cycles']for s in old['summary'].values()):
        raise ValueError('Start search would regress the guarded baseline coverage')
    selected={i:[]for i in distance}
    for _,i,c in chosen.values():selected[i].append(c)
    def needed(i,parent):
        keep=bool(selected[i])
        for j,_,_ in forest.edges[i]:
            if j!=parent:
                keep=needed(j,i)or keep
        needed_nodes[i]=keep;return keep
    needed_nodes={};needed(root,None)
    # A new initial posture does not rotate the fixed world target frames.
    new_initial={k:v.copy()for k,v in initial.items()};new_initial['qpos'][1:8]=states[root,:7]
    new_q=new_initial['qpos'].copy()
    new_display,_=body_poses(source,new_initial,meta,new_q[1],True,new_q[2:8],.35,'conveyor')
    new_model,new_install=collision_model(source,new_q,meta,new_display,output)
    new_boxes=layout_boxes(scene,meta,new_display,parts,new_q[1])
    new_obstacles=BoxObstacles(new_model,new_q,new_boxes)
    audit=GripPlanner(new_model,new_initial,new_obstacles,posture_guard=True,
        model_sha256=new_install['collision_configuration_sha256'])
    segments=[]
    def segment(points,phase,side,index=None):
        measured=[]
        for state in (points[0],points[-1]):
            full=new_q.copy();full[1:10]=state
            if not audit.valid_full(full):raise ValueError('Selected start/segment endpoint violates limits')
            measured.append(capture(audit.data,audit.tool_id).tolist())
        segments.append(dict(points=np.array(points),duration_s=segment_duration(points),phase=phase,
            side=side,coverage_index=index,connector_planner='Certified start-search tree',
            entry=not segments,start_capture_m=measured[0],target_capture_m=measured[1]))
    segment(np.array([new_q[1:10],states[root]]),'open','right')
    def visit(i,parent,side):
        for c in selected[i]:
            side=c['side'];idx=c['index'];approach=c['approach']
            segment(approach,'approach',side,idx)
            segment(np.array([approach[-1],c['closed']]),'close',side,idx)
            segment(np.array([c['closed'],approach[-1]]),'open',side,idx)
            segment(approach[::-1].copy(),'retreat',side,idx)
            full=new_q.copy();full[1:10]=approach[-1]
            if not audit.valid_full(full):raise ValueError('Selected grip endpoint violates limits')
            actual=capture(audit.data,audit.tool_id)
            rows[idx].update(reachable=True,grip_cycle_checked=True,positions=approach[-1].tolist(),
                measured_capture_m=actual.tolist(),ik_error_m=float(np.linalg.norm(actual-rows[idx]['target_capture_m'])),
                lift_height_m=float(approach[-1,0]),yaw_deg=c['yaw_deg'],pitch_deg=c['pitch_deg'],roll_deg=c['roll_deg'],
                approach_distance_m=.04,closed_gap_m=.036,transfer_planner='Certified start-search tree')
        for j,path,_ in sorted(forest.edges[i],key=lambda e:e[0]):
            if j==parent or not needed_nodes[j]:continue
            target_side=selected[j][0]['side'] if selected[j]else side
            segment(path,'transfer',target_side)
            visit(j,i,target_side)
            segment(path[::-1].copy(),'transfer',side)
    visit(root,None,'right')
    comparison=dict(method='Finite multi-lift/multi-branch IK and exact collision-checked connection forest; graph centre minimises worst transfer time',
        baseline_plan=str(seed_plan.resolve()),baseline_complete_targets=sum(s['complete_grip_cycles']for s in old['summary'].values()),
        target_count=len(rows),local_cycle_count=len(solved),ik_probes=attempts,
        graph_nodes=len(nodes),candidate_edges=len(pairs),coarse_passes=coarse_passes,
        rigid_tool_orientations_pruned=sum(v is not None for v in blocked.values()),
        fine_links=fine_passes,rrt_bridge_attempts=bridge_attempts,winning_start=winner)
    comparison['baseline_start']=next((r for r in ranks if r['node']==baseline_home),None)
    comparison['global_optimum_proven']=False
    if resume_candidates is not None:comparison['resumed_candidate_search']=str(Path(resume_candidates).resolve())
    (output/'start_candidates.json').write_text(json.dumps(dict(selected=winner,candidates=ranks),indent=2)+'\n')
    (output/'parked_pose.json').write_text(json.dumps(dict(joint_names=[source.joint(i).name for i in range(2,8)],
        arm_positions_rad=new_q[2:8].tolist(),reference_lift_m=float(new_q[1]),
        nominal_tool_rotation_world=nominal.tolist(),
        scope='Best tested guarded V5 start; fixed world target orientation'),indent=2)+'\n')
    result=finish_grip_motion(audit,new_q,meta,new_install,new_boxes,regions,rows,segments,
        nominal,output,started,'near','conveyor',.04,start_search=comparison)
    print('WINNER',json.dumps(winner),flush=True)
    print('SUMMARY',json.dumps(result.report['summary']),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',type=Path,required=True)
    parser.add_argument('--seed-plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--resume-candidates',type=Path,
        help='Recover stored local approach candidates without repeating the full IK grid')
    args=parser.parse_args()
    optimize_start(args.preview,args.seed_plan,args.output,resume_candidates=args.resume_candidates)


if __name__=='__main__':main()
