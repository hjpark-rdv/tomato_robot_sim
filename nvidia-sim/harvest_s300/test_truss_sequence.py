import unittest
import numpy as np
from truss_sequence import horizontal_truss_direction


class TrussYawTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'f{i}',path=f'/f{i}',joint=f'/j{i}') for i in range(4)]
        self.paths=[f['path'] for f in self.fruits]
        self.positions=np.array([[0,0,4],[1,1,3],[2,2,2],[3,3,1]],dtype=float)

    def test_only_xy_matters_and_sign_is_robot_side(self):
        a=horizontal_truss_direction(self.fruits,[],self.paths,self.positions,[1,1,0])
        moved=self.positions.copy();moved[:,2]=[100,-30,80,5]
        b=horizontal_truss_direction(self.fruits,[],self.paths,moved,[1,1,0])
        np.testing.assert_allclose(a['direction_world'],[2**-.5,2**-.5,0])
        np.testing.assert_array_equal(a['direction_world'],b['direction_world'])
        c=horizontal_truss_direction(self.fruits,[],self.paths,moved,[-1,-1,0])
        np.testing.assert_allclose(c['direction_world'],-np.array(a['direction_world']))

    def test_detached_fruit_outlier_is_excluded(self):
        moved=self.positions.copy();moved[3]=[100,-100,0]
        a=horizontal_truss_direction(self.fruits,[{'joint':'/j3'}],self.paths,moved,[1,1,0])
        np.testing.assert_allclose(a['direction_world'],[2**-.5,2**-.5,0])
        self.assertNotIn('f3',a['fruit_names'])

    def test_ambiguous_remainder_keeps_previous_axis_explicitly(self):
        breaks=[{'joint':f'/j{i}'} for i in (0,1,2)]
        with self.assertRaises(ValueError):
            horizontal_truss_direction(self.fruits,breaks,self.paths,self.positions,[1,0,0])
        a=horizontal_truss_direction(self.fruits,breaks,self.paths,self.positions,[1,0,0],[0,1,0])
        self.assertEqual(a['direction_world'],[0,1,0])
        self.assertEqual(a['fallback_reason'],'fewer_than_two_attached_fruits')

    def test_sparse_transverse_pair_retains_verified_longitudinal_axis_opt_in(self):
        # The whole truss extends along X; its last two fruits form a Y pair.
        positions=np.array([[-.10,0,1],[.10,0,1],[.04,-.015,1],[.04,.015,1]])
        initial=horizontal_truss_direction(self.fruits,[],self.paths,positions,[1,0,0],
                                           min_attached_fruits=3)
        previous=initial['direction_world']
        np.testing.assert_allclose(previous,[1,0,0],atol=1e-12)
        self.assertTrue(initial['axis_fit_evidence']['axis_fit_used'])
        breaks=[{'joint':'/j0'},{'joint':'/j1'}]
        legacy=horizontal_truss_direction(self.fruits,breaks,self.paths,positions,[1,0,0],previous)
        explicit_default=horizontal_truss_direction(self.fruits,breaks,self.paths,positions,[1,0,0],previous,
                                                   min_attached_fruits=2)
        self.assertEqual(legacy,explicit_default)
        self.assertAlmostEqual(abs(legacy['yaw_world_deg']),90.)
        kept=horizontal_truss_direction(self.fruits,breaks,self.paths,positions,[1,0,0],previous,
                                       min_attached_fruits=3)
        np.testing.assert_array_equal(kept['direction_world'],previous)
        self.assertEqual(kept['fruit_names'],['f2','f3'])
        self.assertEqual(kept['fallback_reason'],'insufficient_attached_fruits_for_longitudinal_axis')
        self.assertEqual(kept['axis_fit_evidence'],dict(attached_fruit_count=2,
            min_attached_fruits=3,axis_fit_used=False,retained_previous_verified_direction=True,
            retained_direction_world=previous))
        self.assertGreater(kept['xy_singular_values_m'][0],0.)

    def test_sparse_minimum_requires_verified_horizontal_fallback(self):
        breaks=[{'joint':'/j0'},{'joint':'/j1'}]
        with self.assertRaisesRegex(ValueError,'insufficient_attached_fruits'):
            horizontal_truss_direction(self.fruits,breaks,self.paths,self.positions,[1,0,0],
                                       min_attached_fruits=3)
        for fallback in ([0,0,0],[1,0,.01],[float('nan'),0,0]):
            with self.subTest(fallback=fallback),self.assertRaisesRegex(ValueError,'Fallback'):
                horizontal_truss_direction(self.fruits,breaks,self.paths,self.positions,[1,0,0],fallback,
                                           min_attached_fruits=3)

    def test_minimum_is_explicit_integer_and_does_not_change_dense_fit(self):
        for minimum in (True,False,1,0,-1,3.,float('nan'),float('inf'),'3'):
            with self.subTest(minimum=minimum),self.assertRaisesRegex(ValueError,'integer'):
                horizontal_truss_direction(self.fruits,[],self.paths,self.positions,[1,0,0],
                                           min_attached_fruits=minimum)
        legacy=horizontal_truss_direction(self.fruits,[],self.paths,self.positions,[1,1,0])
        dense=horizontal_truss_direction(self.fruits,[],self.paths,self.positions,[1,1,0],
                                         min_attached_fruits=np.int64(3))
        for key in legacy:self.assertEqual(dense[key],legacy[key])
        self.assertEqual(dense['axis_fit_evidence']['attached_fruit_count'],4)

    def test_four_fruit_remainder_can_retain_saved_dense_longitudinal_axis(self):
        # A partial truss can still have a numerically strong PCA while losing
        # the full truss's long direction. Explicit minimum five retains only
        # the supplied previously verified axis, without moving any fruit.
        observed=np.array([[-.4990014434,1.5736252069,1.1387697458],
                           [-.4883587062,1.5369555950,1.1274019480],
                           [-.4516187310,1.5582298040,1.1115759611],
                           [-.4469119906,1.5234937668,1.0999323130]])
        retained=np.array([-.9813427278227006,.1922666131950656,0.])
        old=horizontal_truss_direction(self.fruits,[],self.paths,observed,retained,
                                       fallback_direction=retained,min_attached_fruits=3)
        self.assertAlmostEqual(old['yaw_world_deg'],143.4345016,places=5)
        kept=horizontal_truss_direction(self.fruits,[],self.paths,observed,retained,
                                        fallback_direction=retained,min_attached_fruits=5)
        np.testing.assert_allclose(kept['direction_world'],retained,atol=1e-15)
        self.assertAlmostEqual(kept['yaw_world_deg'],168.9149092,places=6)
        self.assertFalse(kept['axis_fit_evidence']['axis_fit_used'])
        self.assertEqual(kept['axis_fit_evidence']['attached_fruit_count'],4)
        np.testing.assert_array_equal(kept['centers_world_m'],observed)
from truss_sequence import truss_fruits,inventory,select_group,measured_sequence_evidence


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'Truss_02_Fruit_{i:02}',path=f'/fruit/{i}',joint=f'/joint/{i}') for i in range(1,5)]
        self.paths=[f['path'] for f in self.fruits]
        self.poses=np.array([[0,0,z,0,0,0,1] for z in (.1,.08,.03,0.)])

    def test_exact_truss_and_exact_break_joint(self):
        extra=dict(name='Truss_01_Fruit_01',path='/other',joint='/other/joint')
        manifest=dict(target=self.fruits[0],plant=dict(fruits=self.fruits+[extra]))
        self.assertEqual(truss_fruits(manifest),self.fruits)
        self.assertEqual(inventory(self.fruits,[{'joint':'/joint/11'},{'joint':'/joint/1'}])['detached'],['Truss_02_Fruit_01'])

    def test_remaining_selection_uses_live_positions_and_budget(self):
        before=self.poses.copy()
        group=select_group(self.fruits,[],self.paths,self.poses,{})
        self.assertEqual(group['fruits'],['Truss_02_Fruit_04','Truss_02_Fruit_03'])
        breaks=[{'joint':'/joint/4'}]
        group=select_group(self.fruits,breaks,self.paths,self.poses,{'Truss_02_Fruit_03':1})
        self.assertEqual(group['fruits'],['Truss_02_Fruit_02','Truss_02_Fruit_01'])
        self.poses[0,2]=-.1
        group=select_group(self.fruits,breaks,self.paths,self.poses,{})
        self.assertEqual(group['fruits'],['Truss_02_Fruit_01'])
        self.assertIsNone(select_group(self.fruits,[],self.paths,before,{f['name']:2 for f in self.fruits}))
        self.assertIsNone(select_group(self.fruits,[{'joint':f['joint']} for f in self.fruits],self.paths,before,{}))

    def test_saved_sequence_rejects_clock_reset_and_stale_target_position(self):
        states=dict(time=np.arange(4)/60,poses=np.tile(self.poses,(4,1,1)),body_paths=self.paths)
        group=select_group(self.fruits,[],self.paths,self.poses,{})
        seq=dict(cycle_plans=[dict(**group,measured_frame_index=2)],inventory=inventory(self.fruits,[]))
        self.assertTrue(measured_sequence_evidence(seq,states,self.fruits,[],60)['timestamp_contiguous'])
        states['time'][3]=0
        with self.assertRaisesRegex(ValueError,'gap or reset'):measured_sequence_evidence(seq,states,self.fruits,[],60)
        states['time']=np.arange(4)/60
        states['poses'][2,3,2]+=.01
        with self.assertRaisesRegex(ValueError,'boundary state'):measured_sequence_evidence(seq,states,self.fruits,[],60)

    def test_highest_first_changes_order_without_including_detached_fruit(self):
        group=select_group(self.fruits,[],self.paths,self.poses,{},order='highest-first')
        self.assertEqual(group['fruits'],['Truss_02_Fruit_01','Truss_02_Fruit_02'])
        group=select_group(self.fruits,[{'joint':'/joint/1'},{'joint':'/joint/2'}],self.paths,self.poses,{},order='highest-first')
        self.assertEqual(group['fruits'],['Truss_02_Fruit_03','Truss_02_Fruit_04'])

    def test_first_authored_group_is_not_mislabeled_as_measured_boundary(self):
        import copy
        fruits=copy.deepcopy(self.fruits)
        for f,pose in zip(fruits,self.poses):f['center']=pose[:3].tolist()
        group=select_group(fruits,[],self.paths,self.poses,{})
        cycle=dict(**group,index=0,measured_frame_index=0)
        seq=dict(cycle_plans=[cycle],inventory=inventory(fruits,[]),
            group_policy=dict(schema='s300-harvest-group-policy-v1'),
            initial_aim_source='authored fruit positions; subsequent aims use measured boundary state')
        poses=np.tile(self.poses,(4,1,1));poses[:,:,0]+=3e-7
        states=dict(time=np.arange(4)/60,poses=poses,body_paths=self.paths)
        evidence=measured_sequence_evidence(seq,states,fruits,[],60)
        self.assertEqual(evidence['authored_initial_group_count'],1)
        self.assertAlmostEqual(evidence['authored_initial_to_native_position_max_delta_m'],3e-7)
        bad=copy.deepcopy(seq);bad['cycle_plans'][0]['measured_centers_world_m'][0][0]+=.01
        with self.assertRaisesRegex(ValueError,'authored source'):measured_sequence_evidence(bad,states,fruits,[],60)
        for change in (dict(index=1,measured_frame_index=2),dict(index=0,measured_frame_index=2)):
            bad=copy.deepcopy(seq);bad['cycle_plans'][0].update(change)
            with self.assertRaisesRegex(ValueError,'boundary state'):measured_sequence_evidence(bad,states,fruits,[],60)


class NearScopeContinuationTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'Truss_{truss:02d}_Fruit_{i:02d}',
            path=f'/active/Truss_{truss:02d}_Fruit_{i:02d}',
            joint=f'/active/joint/{truss}/{i}')for truss in (1,2)for i in range(1,11)]
        self.paths=[f['path']for f in self.fruits]
        self.poses=np.array([[.5+.015*i,1.6,1.2-.02*i,0,0,0,1]for i in range(10)]+
                           [[.5,1.6+.015*i,1.1-.02*i,0,0,0,1]for i in range(10)])
        self.initial=[f for f in self.fruits if f['name'].startswith('Truss_02_')]

    def select(self,attempts=None,breaks=(),**options):
        from truss_sequence import select_near_scope_continuation
        return select_near_scope_continuation(self.fruits,list(breaks),self.paths,self.poses,
            attempts or {},'Truss_02',**options)

    def test_original_truss_is_first_without_changing_default_selector_result(self):
        legacy=select_group(self.initial,[],self.paths,self.poses,{})
        group,evidence=self.select()
        for key in legacy:self.assertEqual(group[key],legacy[key])
        self.assertEqual(group['truss_key'],'Truss_02')
        self.assertFalse(evidence['switched_truss']);self.assertFalse(evidence['physical_state_reset'])
        # The opt-in wrapper leaves all existing default selector behavior exact.
        self.assertEqual(select_group(self.initial,[],self.paths,self.poses,{}),legacy)

    def test_exhausted_original_falls_back_without_resetting_budget_or_centers(self):
        attempts={f['name']:6 for f in self.initial};before=attempts.copy();poses=self.poses.copy()
        group,evidence=self.select(attempts=attempts,max_attempts=6)
        self.assertEqual(group['truss_key'],'Truss_01');self.assertTrue(evidence['switched_truss'])
        self.assertEqual(evidence['initial_truss_attempt_exhausted'],[f['name']for f in self.initial])
        self.assertEqual(attempts,before);np.testing.assert_array_equal(self.poses,poses)
        actual=[self.poses[self.paths.index(next(f['path']for f in self.fruits if f['name']==n)),:3].tolist()for n in group['fruits']]
        self.assertEqual(group['measured_centers_world_m'],actual)
        self.assertEqual(evidence['reason'],'initial_truss_finite_attempt_or_rejection_exhaustion')

    def test_rejected_original_and_detached_others_are_never_retried(self):
        rejected=[(f['name'],)for f in self.initial]
        rejected+=[(a['name'],b['name'])for a in self.initial for b in self.initial if a['name']<b['name']]
        one=self.fruits[0];breaks=[dict(joint=one['joint'])]
        group,evidence=self.select(breaks=breaks,rejected_groups=rejected)
        self.assertEqual(group['truss_key'],'Truss_01');self.assertNotIn(one['name'],group['fruits'])
        exhausted={f['name']:2 for f in self.fruits}
        group,evidence=self.select(attempts=exhausted)
        self.assertIsNone(group);self.assertTrue(evidence['selection_exhausted'])
        self.assertFalse(evidence['physical_state_reset'])

    def test_different_truss_gets_own_measured_axis_and_robot_side_sign(self):
        from truss_sequence import near_scope_truss_direction
        group,_=self.select(attempts={f['name']:2 for f in self.initial})
        old=horizontal_truss_direction(self.initial,[],self.paths,self.poses,[0,1,0],min_attached_fruits=10)
        axes={'Truss_02':old};before=self.poses.copy()
        actual=near_scope_truss_direction(group,self.fruits,[],self.paths,self.poses,[0,1.6,1.],axes,min_attached_fruits=10)
        np.testing.assert_allclose(actual['direction_world'],[1,0,0],atol=1e-12)
        self.assertEqual(actual['truss_key'],'Truss_01')
        self.assertTrue(actual['axis_fit_evidence']['axis_fit_used'])
        self.assertFalse(actual['near_scope_axis']['other_truss_retained_axis_used'])
        self.assertFalse(actual['near_scope_axis']['own_truss_verified_axis_available'])
        np.testing.assert_array_equal(self.poses,before)
        reverse=near_scope_truss_direction(group,self.fruits,[],self.paths,self.poses,[2,1.6,1.],axes,min_attached_fruits=10)
        np.testing.assert_allclose(reverse['direction_world'],[-1,0,0],atol=1e-12)
        with self.assertRaisesRegex(ValueError,'this exact truss'):
            near_scope_truss_direction(group,self.fruits,[],self.paths,self.poses,[0,1.6,1.],{'Truss_01':old})

    def test_sparse_new_truss_cannot_inherit_other_truss_axis(self):
        from truss_sequence import near_scope_truss_direction
        breaks=[dict(joint=f['joint'])for f in self.fruits[2:10]]
        group,_=self.select(attempts={f['name']:2 for f in self.initial},breaks=breaks)
        other=horizontal_truss_direction(self.initial,[],self.paths,self.poses,[0,1,0])
        with self.assertRaisesRegex(ValueError,'insufficient_attached_fruits'):
            near_scope_truss_direction(group,self.fruits,breaks,self.paths,self.poses,[0,1.6,1.],{'Truss_02':other})
        dense=near_scope_truss_direction(group,self.fruits,[],self.paths,self.poses,[0,1.6,1.])
        kept=near_scope_truss_direction(group,self.fruits,breaks,self.paths,self.poses,[0,1.6,1.],{'Truss_01':dense})
        np.testing.assert_array_equal(kept['direction_world'],dense['direction_world'])
        self.assertFalse(kept['axis_fit_evidence']['axis_fit_used'])

    def test_unknown_or_mixed_truss_rejection_fails_mapping(self):
        from truss_sequence import select_near_scope_continuation
        with self.assertRaisesRegex(ValueError,'Initial truss'):
            select_near_scope_continuation(self.fruits,[],self.paths,self.poses,{},'Truss_00')
        with self.assertRaisesRegex(ValueError,'crosses exact truss'):
            self.select(rejected_groups=[(self.fruits[0]['name'],self.initial[0]['name'])])


if __name__=='__main__':unittest.main()
