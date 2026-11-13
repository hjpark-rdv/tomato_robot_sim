"""Named ROS commands -> bounded actuator targets, with no physics-state writes."""
from collections import deque
import math
import numpy as np
from numpy.polynomial import Polynomial
from newton_patrol_motion import VELOCITY,ACCELERATION


class Segment:
    def __init__(self,t0,t1,p0,p1,v0,v1,a0=None,a1=None):
        self.t0=t0;self.t1=t1;d=t1-t0
        if d<=0:raise ValueError('Trajectory times must strictly increase')
        if a0 is None:
            self.p=Polynomial([p0,v0*d,3*(p1-p0)-d*(2*v0+v1),2*(p0-p1)+d*(v0+v1)])
        else:
            c0=p0;c1=v0*d;c2=a0*d*d/2
            c3,c4,c5=np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],
                [p1-c0-c1-c2,v1*d-c1-2*c2,a1*d*d-2*c2])
            self.p=Polynomial([c0,c1,c2,c3,c4,c5])
        self.v=self.p.deriv()/d;self.a=self.v.deriv()/d

    def sample(self,t):
        u=np.clip((t-self.t0)/(self.t1-self.t0),0.,1.)
        return float(self.p(u)),float(self.v(u)),float(self.a(u))

    def validate(self,bounds,vmax,amax):
        for poly,lo,hi,label in ((self.p,*bounds,'position'),(self.v,-vmax,vmax,'velocity'),(self.a,-amax,amax,'acceleration')):
            roots=poly.deriv().roots()
            points=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
            values=poly(points)
            if not np.isfinite(values).all() or values.min()<lo-1e-8 or values.max()>hi+1e-8:
                raise ValueError('Trajectory '+label+' limit exceeded')


class Commands:
    def __init__(self,names,q,bounds,*,watchdog=.5,event_sink=None,model_sha256=None):
        self.names=list(names);self.q=np.array(q,float);self.v=np.zeros(10);self.a=np.zeros(10)
        self.bounds=np.array(bounds,float);self.watchdog=watchdog;self.event_sink=event_sink
        if len(self.names)!=10 or len(set(self.names))!=10:raise ValueError('Ten unique simulator joints required')
        if self.bounds.shape!=(10,2) or not np.isfinite(self.bounds).all() or np.any(self.bounds[:,0]>=self.bounds[:,1]):
            raise ValueError('Finite ordered position bounds required for all ten joints')
        if self.q.shape!=(10,) or not np.isfinite(self.q).all() or np.any(self.q<self.bounds[:,0]-1e-6) or np.any(self.q>self.bounds[:,1]+1e-6):
            raise ValueError('Initial commanded positions must be finite and within bounds')
        self.plans={};self.groups={};self.recent=deque(maxlen=64);self.events=0
        self.stopped=False;self.stop_reason=None;self.base_speed=0.;self.base_until=-math.inf
        self.base_sequence=None;self.last_sequence=0;self.last_time=0.
        self.tolerance=np.array([.002,.002]+[.01]*6+[.001,.001])
        self.model_sha256=model_sha256
        self.measured=self.q.copy();self.measured_velocity=np.zeros(10)

    def guarded_group(self):
        return next((g for g in self.groups.values() if g.get('guarded')),None)

    def motion_active(self):
        return bool(self.plans or self.base_sequence is not None or
                    np.any(np.abs(self.v)>1e-8) or np.any(np.abs(self.measured_velocity)>.02))

    @staticmethod
    def request_fields(group):
        return {'request_id':group['request_id']} if 'request_id' in group else {}

    def event(self,sequence,kind,status,reason='',**values):
        self.events+=1
        row=dict(event=self.events,sequence=sequence,command_type=kind,status=status,reason=reason,
                 simulation_time=self.last_time,**values)
        self.recent.append(row)
        if self.event_sink:self.event_sink(row)

    def feedback(self):
        return dict(mode='ros_control',state='stopped' if self.stopped else 'ready',stop_reason=self.stop_reason,
                    recent_events=list(self.recent),target_positions=self.q.tolist(),joint_names=self.names,
                    velocity_limits=VELOCITY.tolist(),acceleration_limits=ACCELERATION.tolist(),
                    position_limits=self.bounds.tolist(),cmd_vel_timeout_wall_s=self.watchdog,
                    motion_active=self.motion_active(),
                    active_planning_request_id=(self.guarded_group() or {}).get('request_id'))

    def cancel_axes(self,axes,reason):
        for i in axes:self.plans.pop(i,None)
        for seq,group in list(self.groups.items()):
            if set(axes)&set(group['axes']):
                # A group is one command: preempt all of its remaining axes.
                for i in group['axes']:self.plans.pop(i,None)
                self.event(seq,group['kind'],'preempted',reason,**self.request_fields(group));del self.groups[seq]
        if 0 in axes and self.base_sequence is not None:
            self.event(self.base_sequence,'cmd_vel','preempted',reason);self.base_sequence=None
            self.base_speed=0.;self.base_until=-math.inf

    def stop(self,reason='operator_stop'):
        self.cancel_axes(range(10),reason);self.stopped=True;self.stop_reason=reason

    def accept(self,c,t,wall):
        self.last_time=t;seq=c['sequence'];kind=c['command_type']
        request_fields=({'request_id':c['request_id']} if kind=='planned_arm_trajectory' and
                        isinstance(c.get('request_id'),str) else {})
        try:
            if seq<=self.last_sequence:raise ValueError('Duplicate/out-of-order command sequence')
            self.last_sequence=seq
            age=wall-float(c['received_monotonic'])
            if age<-.1 or age>max(2.,self.watchdog):raise ValueError('Command expired in transport queue')
            if kind=='stop':
                if c['value']:self.stop()
                else:self.stopped=False;self.stop_reason=None
                self.event(seq,kind,'accepted', 'holding; cancelled commands will not resume' if c['value'] else 'ready for new commands')
                return True
            if self.stopped:raise ValueError('Stop is latched; publish stop=false before a new command')
            if self.guarded_group():
                raise ValueError('Planned trajectory owns all motion axes until completion or stop')
            if kind=='planned_arm_trajectory':
                self.validate_planned_start(c)
            if kind=='cmd_vel':
                vectors=np.array(c['linear']+c['angular'],float)
                if vectors.shape!=(6,) or not np.isfinite(vectors).all():raise ValueError('Finite Twist required')
                if np.any(np.abs(vectors[1:])>1e-12):raise ValueError('Rail model accepts only linear.x; lateral/rotational motion unavailable')
                if abs(vectors[0])>VELOCITY[0]+1e-9:raise ValueError('Rail velocity exceeds 0.12m/s')
                if age>self.watchdog:raise ValueError('cmd_vel already expired')
                if 0 in self.plans:self.cancel_axes([0],'cmd_vel replaced trajectory')
                self.base_speed=float(vectors[0]);self.base_until=c['received_monotonic']+self.watchdog;self.base_sequence=seq
                self.event(seq,kind,'accepted');return True
            if kind in ('lift','gripper'):
                value=float(c['value'])
                if not np.isfinite(value):raise ValueError('Finite target required')
                ids=[1] if kind=='lift' else [8,9]
                targets=[value] if kind=='lift' else [-(value-.044)/2,(value-.044)/2]
                if np.any(np.array(targets)<self.bounds[ids,0]-1e-9) or np.any(np.array(targets)>self.bounds[ids,1]+1e-9):
                    raise ValueError(kind+' position limit exceeded')
                targets=np.clip(targets,self.bounds[ids,0],self.bounds[ids,1]).tolist()
                delta=np.abs(np.array(targets)-self.q[ids])
                duration=max(.25,float(np.max(1.875*delta/VELOCITY[ids])),float(np.sqrt(np.max(6*delta/ACCELERATION[ids]))))
                # Replacements may arrive while moving; extend until their
                # exact polynomial extrema fit the same velocity/acceleration limits.
                error=None
                for _ in range(60):
                    trial=dict(joint_names=[self.names[i] for i in ids],start_time=0.,points=[dict(
                        positions=targets,velocities=[],accelerations=[],effort=[],time_from_start=duration)])
                    try:plans,end=self.trajectory(trial,t);break
                    except ValueError as exc:error=exc;duration*=1.1
                else:raise error
            elif kind in ('arm_trajectory','joint_trajectory','planned_arm_trajectory'):
                if kind in ('arm_trajectory','planned_arm_trajectory') and not set(c['joint_names']).issubset(self.names[2:8]):
                    raise ValueError('Arm topic accepts only base, shoulder, elbow, wrist1, wrist2, wrist3')
                plans,end=self.trajectory(c,t);ids=list(plans)
            else:raise ValueError('Unknown command type')
            self.cancel_axes(ids,'replaced by command '+str(seq))
            self.plans.update(plans)
            self.groups[seq]=dict(kind=kind,axes=ids,end=end,targets=[plans[i][-1].sample(end)[0] for i in ids],
                                 guarded=kind=='planned_arm_trajectory',**request_fields)
            self.event(seq,kind,'accepted',end_simulation_time=end,**request_fields)
            return True
        except (ValueError,KeyError,TypeError,OverflowError) as error:
            self.event(seq,kind,'rejected',str(error),**request_fields);return False

    def validate_planned_start(self,c):
        """Check the planner's entire start snapshot atomically before changing any axis."""
        request=c.get('request_id')
        if not isinstance(request,str) or not request or len(request)>128:
            raise ValueError('Planned trajectory requires a nonempty request_id up to 128 characters')
        if not self.model_sha256 or c.get('source_model_sha256')!=self.model_sha256:
            raise ValueError('Planned trajectory model SHA does not match this simulator')
        expected=np.asarray(c['expected_start_positions'],float)
        if expected.shape!=(10,) or not np.isfinite(expected).all():
            raise ValueError('Planned trajectory requires ten finite expected start positions')
        if (self.plans or self.groups or self.base_sequence is not None or
                np.any(np.abs(self.v)>1e-8) or np.any(np.abs(self.a)>1e-8)):
            raise ValueError('Planned trajectory requires stationary command targets and no active motion')
        if not np.isfinite(self.measured).all() or not np.isfinite(self.measured_velocity).all():
            raise ValueError('Planned trajectory requires finite measured state')
        if np.any(np.abs(self.measured_velocity)>.02):
            raise ValueError('Planned trajectory requires the measured robot to be stationary')
        if np.any(np.abs(expected-self.q)>1e-6):
            raise ValueError('Planned trajectory start is stale: commanded positions changed')
        if np.any(np.abs(expected-self.measured)>self.tolerance):
            raise ValueError('Planned trajectory measured start exceeds tracking tolerance')

    def trajectory(self,c,t):
        names=c['joint_names'];points=c['points']
        if not names or len(names)!=len(set(names)) or not set(names).issubset(self.names):raise ValueError('Unknown, empty or duplicate joint names')
        if not 1<=len(points)<=512:raise ValueError('Trajectory needs 1..512 points')
        ids=[self.names.index(n) for n in names];start=float(c['start_time'])
        if not np.isfinite(start) or start<0:raise ValueError('Invalid trajectory header stamp')
        if start and start<t-.05:raise ValueError('Trajectory header stamp is in the past')
        if start>t+10:raise ValueError('Trajectory starts more than 10 simulated seconds ahead')
        start=max(t,start)
        if start>t+1e-8 and np.max(np.abs(self.v[ids]))>1e-5:raise ValueError('Future start requires stationary selected joints')
        count=len(names);plans={i:[] for i in ids};p=self.q[ids].copy();v=self.v[ids].copy();a=self.a[ids].copy();previous=0.
        use_velocity=bool(points[0].get('velocities'));use_acceleration=bool(points[0].get('accelerations'))
        if use_acceleration and not use_velocity:raise ValueError('Acceleration requires velocity values')
        for index,row in enumerate(points):
            if row.get('effort'):raise ValueError('Effort control is not implemented; use position trajectories')
            if bool(row.get('velocities'))!=use_velocity or bool(row.get('accelerations'))!=use_acceleration:
                raise ValueError('All points must use the same derivative fields')
            target=np.asarray(row['positions'],float)
            vel=np.asarray(row['velocities'],float) if use_velocity else np.zeros(count)
            acc=np.asarray(row['accelerations'],float) if use_acceleration else np.zeros(count)
            duration=float(row['time_from_start'])
            if any(x.shape!=(count,) or not np.isfinite(x).all() for x in (target,vel,acc)) or not np.isfinite(duration):
                raise ValueError('Finite, correctly sized point fields required')
            if not 0<=duration<=300:raise ValueError('Trajectory duration must be within 0..300s')
            if duration==0 and index==0:
                if not np.allclose(p,target,atol=1e-6,rtol=0) or not np.allclose(v,vel,atol=1e-6,rtol=0):
                    raise ValueError('Zero-time point must match current commanded position/velocity')
                if use_acceleration and not np.allclose(a,acc,atol=1e-6,rtol=0):
                    raise ValueError('Zero-time acceleration must match current commanded acceleration')
                continue
            for j,i in enumerate(ids):
                seg=Segment(start+previous,start+duration,p[j],target[j],v[j],vel[j],
                    None if use_velocity and not use_acceleration else a[j],
                    None if use_velocity and not use_acceleration else acc[j])
                seg.validate(self.bounds[i],VELOCITY[i],ACCELERATION[i]);plans[i].append(seg)
            previous=duration;p=target;v=vel;a=acc
        if any(not segments for segments in plans.values()):raise ValueError('Trajectory needs a positive-duration segment')
        if np.max(np.abs(v))>1e-8 or np.max(np.abs(a))>1e-8:raise ValueError('Final point must stop (zero velocity/acceleration)')
        return plans,start+previous

    def advance(self,t,wall,dt):
        self.last_time=t
        if self.base_sequence is not None and wall>self.base_until:
            self.event(self.base_sequence,'cmd_vel','expired','velocity watchdog: decelerating to rest')
            self.base_sequence=None;self.base_speed=0.
        for i in range(10):
            if i in self.plans:
                segments=self.plans[i]
                if t<segments[0].t0:continue
                seg=next((s for s in segments if t<=s.t1),segments[-1])
                self.q[i],self.v[i],self.a[i]=seg.sample(t)
            else:
                wanted=self.base_speed if i==0 and not self.stopped else 0.
                direction=1 if wanted>=0 else -1
                remaining=self.bounds[i,1]-self.q[i] if direction>0 else self.q[i]-self.bounds[i,0]
                wanted=direction*min(abs(wanted),math.sqrt(max(0.,2*ACCELERATION[i]*(remaining-.002))))
                old=self.v[i];self.v[i]+=float(np.clip(wanted-old,-ACCELERATION[i]*dt,ACCELERATION[i]*dt))
                self.q[i]+=dt*(old+self.v[i])/2;self.a[i]=(self.v[i]-old)/dt
                self.q[i]=float(np.clip(self.q[i],*self.bounds[i]))
        return self.q.copy()

    def observe(self,t,measured,velocity):
        self.last_time=t
        self.measured=np.asarray(measured,float).copy();self.measured_velocity=np.asarray(velocity,float).copy()
        if self.measured.shape!=(10,) or self.measured_velocity.shape!=(10,):
            raise ValueError('Ten measured positions and velocities required')
        for seq,g in list(self.groups.items()):
            if t<g['end']:continue
            ids=g['axes'];error=np.abs(np.asarray(measured)[ids]-g['targets'])
            if np.all(error<=self.tolerance[ids]) and np.all(np.abs(np.asarray(velocity)[ids])<=.02):
                self.event(seq,g['kind'],'completed',maximum_tracking_error=float(error.max()),**self.request_fields(g))
                for i in ids:self.plans.pop(i,None)
                del self.groups[seq]
            elif t>g['end']+3.:
                self.event(seq,g['kind'],'tracking_failed',maximum_tracking_error=float(error.max()),**self.request_fields(g))
                self.stop('tracking_failed');break
