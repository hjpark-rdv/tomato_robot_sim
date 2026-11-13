import unittest

import mujoco as mj
import numpy as np

from preview_grip_workspace import BoxObstacles, require_continuous_segments
from preview_start_search import Forest, RigidToolProbe, canonical_pose, entry_rotation, rank_starts


class StartSearchTests(unittest.TestCase):
    def test_opposite_elbow_family_keeps_a_consistent_shoulder_winding(self):
        home=np.r_[.75,np.deg2rad([-105,113,-108,-95,90,74]),[-.026,.026]]
        bounds=np.array([[0,4],[0,.75]]+[[-2*np.pi,2*np.pi]]*6+[[-.026,.008],[-.008,.026]])
        state=home.copy();state[2]=np.deg2rad(250.);state[3]=np.deg2rad(108.)
        result=canonical_pose(state,home,bounds,1)
        self.assertAlmostEqual(np.rad2deg(result[2]),-110.)
        np.testing.assert_allclose(np.exp(1j*result[1:7]),np.exp(1j*state[1:7]),atol=1e-12)

    def test_equivalent_angle_does_not_allow_a_teleport_between_segments(self):
        q=np.zeros(10);a=np.zeros(9);b=a.copy();b[1]=.2
        c=b.copy();c[1]+=2*np.pi
        with self.assertRaisesRegex(ValueError,'teleport'):
            require_continuous_segments([dict(points=np.array([a,b])),dict(points=np.array([c,c]))],q)
        require_continuous_segments([dict(points=np.array([a,b])),dict(points=np.array([b,a]))],q)

    def test_entry_cone_applies_to_both_sides_and_tool_roll(self):
        for side in ('right','left'):
            nominal=np.eye(3)
            forward=np.array([0.,1. if side=='right'else -1.,0.])
            for roll in (0.,180.):
                rotation=entry_rotation(nominal,side,5.,14.,roll)
                angle=np.degrees(np.arccos(np.clip(forward@rotation[:,1],-1.,1.)))
                self.assertLessEqual(angle,15.)
                np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
        with self.assertRaisesRegex(ValueError,'15 degree'):
            entry_rotation(np.eye(3),'right',14.,14.,0.)

    def test_start_ranking_counts_unique_connected_targets_and_selects_a_hub(self):
        states=[]
        for base in (0.,.1,.2,.3,0.):
            q=np.zeros(9);q[1]=base;q[3]=-1.;states.append(q)
        nodes=[dict(state=q,cycles=[])for q in states]
        nodes[0]['cycles']=[dict(index=0)]
        nodes[2]['cycles']=[dict(index=1)]
        nodes[3]['cycles']=[dict(index=0)]
        nodes[4]['cycles']=[dict(index=2)]
        forest=Forest(len(nodes))
        for a,b in ((0,1),(1,2),(1,3)):
            forest.join(a,b,np.array([states[a],states[b]]))
        rows=[dict(side='right'),dict(side='left'),dict(side='right')]
        ranks=rank_starts(nodes,forest,rows)
        self.assertEqual(ranks[0]['node'],1)
        self.assertEqual(ranks[0]['unique_targets'],2)
        self.assertEqual(ranks[0]['by_side'],dict(right=1,left=1))
        self.assertEqual(next(r for r in ranks if r['node']==4)['unique_targets'],1)

    def rigid_probe(self):
        chain=''.join(f'<body name="carrier{i}"><joint name="j{i}" axis="0 0 1"/>'
            '<inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>'for i in range(8))
        xml='''<mujoco><worldbody><body name="spare" pos="-3 0 0">
            <geom type="sphere" size=".01"/></body><body pos="1 0 0">'''+chain+'''
            <body name="Tool_fixed"><geom name="tool_shell" type="sphere" size=".03" contype="2" conaffinity="3"/>
              <body name="Tool_L"><joint type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/></body>
              <body name="Tool_R"><joint type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/></body>
            </body>'''+('</body>'*8)+'''</body></worldbody></mujoco>'''
        model=mj.MjModel.from_xml_string(xml)
        model.geom_contype[0]=model.geom_conaffinity[0]=0
        obstacles=BoxObstacles(model,np.zeros(10),[dict(name='bin/side',position=[0,0,0],dimensions=[.1]*3)])
        return RigidToolProbe(model,np.zeros(10),obstacles,model.body('Tool_fixed').id)

    def test_rigid_tool_probe_uses_requested_world_pose_for_native_distance(self):
        probe=self.rigid_probe()
        blocked=probe.rejection(np.array([.055,0.,0.]),np.eye(3))
        self.assertEqual(blocked['obstacle'],'bin/side')
        self.assertAlmostEqual(blocked['distance_m'],-.025,places=6)
        self.assertIsNone(probe.rejection(np.array([.2,0.,0.]),np.eye(3)))

    def test_rigid_probe_does_not_prune_an_ik_tolerance_borderline(self):
        self.assertIsNone(self.rigid_probe().rejection(np.array([.079,0.,0.]),np.eye(3)))


if __name__=='__main__':unittest.main()
