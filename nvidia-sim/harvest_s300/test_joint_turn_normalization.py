"""Periodic joint identity, physical link FK, limits and default-command controls."""
import hashlib
import json
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from motion_kinematics import ToolKinematics,equivalent_joint_goal_near_start,grasp_pose,plan_motion
from test_motion_branch import RedundantCartesianRobot


def serial_robot():
    # Exercise the actual serial FK implementation on a nontrivial 7-joint
    # chain; this fixture has no simulator, synthetic collision exemptions or
    # inverse-kinematics shortcut pretending that linear coordinates wrap.
    kin=ToolKinematics.__new__(ToolKinematics);kin.world=np.eye(4);kin.world[:3,3]=[.2,-.3,.6]
    kin.names=['lift','base','shoulder','elbow','wrist1','wrist2','wrist3']
    kin.offset=np.eye(4);kin.offset[:3,3]=[.04,.16,-.02]
    kin.bounds=np.array([[0.,-2*np.pi,-2*np.pi,-2.87979,-2*np.pi,-2*np.pi,-2*np.pi],
                         [1.,2*np.pi,2*np.pi,2.87979,2*np.pi,2*np.pi,2*np.pi]])
    kin.chain=[]
    for i,axis in enumerate(([0,0,1],[0,0,1],[0,1,0],[0,1,0],[1,0,0],[0,1,0],[0,0,1])):
        origin=np.eye(4);origin[:3,3]=[.01*i,.025*(i%3),.04+.01*i]
        kin.chain.append((origin,np.asarray(axis,float),i==0,'/robot/link'+str(i)))
    return kin


class PeriodicCartesianRobot:
    """Readable physical translations plus independent periodic Euler joints."""
    world=np.eye(4)
    bounds=np.array([[-1.,-10.,-10.,-10.,-2*np.pi,-2*np.pi,-2*np.pi],
                     [1.,10.,10.,10.,2*np.pi,2*np.pi,2*np.pi]])
    chain=[(None,None,i<4,'/fixture/'+str(i)) for i in range(7)]
    def frames(self,q):
        result={};frame=np.eye(4)
        for i in range(7):
            if i<4:frame[:3,3]+={0:np.array([0,0,q[0]]),1:np.array([q[1],0,0]),
                                2:np.array([0,q[2],0]),3:np.array([0,0,q[3]])}[i]
            else:frame[:3,:3]=frame[:3,:3]@Rotation.from_euler('xyz'[i-4],q[i]).as_matrix()
            result['/fixture/'+str(i)]=frame.copy()
        return result
    def fk(self,q):return list(self.frames(q).values())[-1]
    def ik(self,pose,seed):return self.ik_at_lift(pose,seed,seed[0])
    def ik_at_lift(self,pose,seed,lift):
        angles=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')
        # Global seeds first propose the equivalent negative wrist3 branch;
        # continuation retains the nearest already selected representation.
        if abs(seed[6])<1e-12:angles[2]+=(-2*np.pi if angles[2]>=0 else 2*np.pi)
        else:
            for i in range(3):
                choices=[(abs(angles[i]+2*np.pi*n-seed[4+i]),angles[i]+2*np.pi*n)
                         for n in (-1,0,1) if -2*np.pi<=angles[i]+2*np.pi*n<=2*np.pi]
                angles[i]=min(choices)[1]
        q=np.r_[lift,pose[0,3],pose[1,3],pose[2,3]-lift,angles]
        return q,0.,0.


class JointTurnNormalizationTests(unittest.TestCase):
    def test_long_wrist_turns_shorten_without_moving_any_link_or_wrapping_elbow_lift(self):
        kin=serial_robot()
        start=np.array([.008366,1.34190,-.15202,2.34211,.95541,2.98389,3.14079])
        raw=np.array([.008366,.05532,-.16455,-2.23862,-.73834,-3.69615,-3.14152])
        saved=raw.copy();canonical,evidence=equivalent_joint_goal_near_start(kin,raw,start)
        np.testing.assert_array_equal(raw,saved)
        self.assertEqual(evidence['integer_turn_shifts'],[0,0,0,0,0,1,1])
        np.testing.assert_array_equal(canonical[:5],raw[:5])
        self.assertLess(abs(canonical[5]-start[5]),.4);self.assertLess(abs(canonical[6]-start[6]),.002)
        self.assertTrue(np.all(canonical>=kin.bounds[0]));self.assertTrue(np.all(canonical<=kin.bounds[1]))
        for path,pose in kin.frames(raw).items():np.testing.assert_allclose(pose,kin.frames(canonical)[path],atol=1e-14,rtol=0.)
        np.testing.assert_allclose(kin.fk(raw),kin.fk(canonical),atol=1e-14,rtol=0.)
        self.assertFalse(evidence['collision_certified']);self.assertFalse(evidence['physical_initial_state_changed'])

    def test_ties_preserve_original_representation_and_nonperiodic_or_unknown_joint_is_rejected(self):
        kin=serial_robot();start=np.zeros(7);goal=np.zeros(7);goal[1]=np.pi
        canonical,evidence=equivalent_joint_goal_near_start(kin,goal,start)
        self.assertEqual(canonical[1],np.pi);self.assertEqual(evidence['integer_turn_shifts'][1],0)
        class WrongLinkFK:
            bounds=kin.bounds;chain=kin.chain
            def frames(self,q):
                frame=np.eye(4);frame[0,3]=q[1]
                return {'/not-periodic':frame}
        with self.assertRaisesRegex(ValueError,'physical FK'):equivalent_joint_goal_near_start(WrongLinkFK(),np.array([0.,-3.,0,0,0,0,0]),np.array([0.,3.,0,0,0,0,0]))
        bad=serial_robot();bad.chain[5]=(*bad.chain[5][:2],None,bad.chain[5][3])
        with self.assertRaisesRegex(ValueError,'joint identity'):equivalent_joint_goal_near_start(bad,goal,start)
        bad=serial_robot();bad.bounds[0,1]=float('nan')
        with self.assertRaisesRegex(ValueError,'exact finite'):equivalent_joint_goal_near_start(bad,goal,start)

    def test_default_false_keeps_two_repetition_arrays_and_boolean_scope_is_strict(self):
        kin=RedundantCartesianRobot();center=np.array([.4,1.,1.2]);ready=grasp_pose(center,center.copy())
        ready[:3,3]-=ready[:3,1]*.3;start=kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        opts=dict(preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',
                  preparation_height_first=True,approach_direction_world=ready[:3,1])
        original,meta=plan_motion(kin,center,start,**opts)
        explicit,after=plan_motion(kin,center,start,**opts,horizontal_ready_normalize_turns=False)
        self.assertEqual(meta,after)
        self.assertEqual(hashlib.sha256(json.dumps(meta['phases'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
                         'd7f3058ba28e6ec101d5404c6f06b62da57b14c3f3e8eda1ad04d563eefcca1a')
        for a,b in zip(original,explicit):np.testing.assert_array_equal(a[2],b[2])
        for value in (1,0,'yes',None):
            with self.assertRaisesRegex(ValueError,'Turn normalization'):plan_motion(kin,center,start,**opts,horizontal_ready_normalize_turns=value)
        with self.assertRaisesRegex(ValueError,'Turn normalization'):plan_motion(kin,center,start,**opts,horizontal_ready_normalize_turns=True)

    def test_enabled_plan_is_continuous_returns_by_exact_reverse_and_keeps_fixed_entry_pose(self):
        kin=PeriodicCartesianRobot();center=np.array([.4,1.,1.2]);ready=grasp_pose(center,center.copy())
        ready[:3,3]-=ready[:3,1]*.3
        start=kin.ik_at_lift(ready,np.array([.12,0,0,0,0,0,-3.]),.12)[0]
        options=dict(preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',
                     approach_direction_world=ready[:3,1],allow_measured_ready_start=True,
                     horizontal_ready_connector='joint',horizontal_ready_seed_index=0,preparation_gap_m=.044)
        raw,old=plan_motion(kin,center,start,**options)
        phases,meta=plan_motion(kin,center,start,**options,horizontal_ready_normalize_turns=True)
        evidence=meta['joint_ready_connector']['turn_normalization']
        self.assertEqual(evidence['integer_turn_shifts'][-1],-1)
        connector=next(p for p in phases if p[0]=='prepare_joint_branch')
        reverse=next(p for p in phases if p[0]=='return_joint_branch')
        np.testing.assert_array_equal(reverse[2],connector[2][::-1])
        np.testing.assert_array_equal(phases[0][2][0],start);np.testing.assert_array_equal(phases[-1][2][-1],start)
        for a,b in zip(phases,phases[1:]):np.testing.assert_allclose(a[2][-1],b[2][0],atol=1e-14,rtol=0.)
        np.testing.assert_array_equal(meta['goal_pose'],old['goal_pose'])
        np.testing.assert_array_equal(meta['approach_waypoint_poses'],old['approach_waypoint_poses'])
        for phase in phases:
            self.assertTrue(np.all(phase[2]>=kin.bounds[0]));self.assertTrue(np.all(phase[2]<=kin.bounds[1]))
        for q in next(p for p in phases if p[0]=='approach')[2]:
            np.testing.assert_allclose(kin.fk(q)[:3,:3],np.asarray(meta['goal_pose'])[:3,:3],atol=2e-15)


if __name__=='__main__':unittest.main()
