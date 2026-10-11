import copy
import unittest
from unittest.mock import patch

import numpy as np

from connect_s300_prefix import (prefix_indices,splice_connected_plan,timed_edges,
                                existing_ompl,reconstruct_phase_tuples,
                                MANUAL_POLICIES,manual_prefix,full_environment_trace)


def fixture():
    a=np.zeros(7);b=np.array([.01,.1,-.2,.3,.1,-.1,.05])
    def phase(name,start,end,gap,duration=1.):
        return dict(name=name,duration_s=duration,waypoint_q=[start.tolist(),end.tolist()],
                    end_gap_m=gap,belt_local_y_m_s=[0.,0.])
    return dict(phases=[phase('ready_hold',a,a,.044),phase('prepare_gap',a,a,.044),
        phase('prepare_joint_branch',a,b,.044,6.),phase('prepare_open',b,b,.096),
        phase('approach',b,b+.001,.096),phase('close',b+.001,b+.001,.028),
        phase('withdraw',b+.001,b,.096),phase('return_joint_branch',b,a,.096,6.),
        phase('finish',a,a,.044)],target_reference=dict(fruits=['Truss_01_Fruit_01','Truss_01_Fruit_02']))


class ExistingOmplPrefixTests(unittest.TestCase):
    def test_only_connector_and_exact_return_change_suffix_and_source_remain_identical(self):
        source=fixture();old=copy.deepcopy(source);begin,back,endpoints=prefix_indices(source)
        middle=(endpoints[0]+endpoints[1])/2;middle[2]+=.1
        path=np.array([endpoints[0],middle,endpoints[1]])
        revised=splice_connected_plan(source,path,[.08]+[.35]*6)
        self.assertEqual(source,old)
        original=[p for p in source['phases'] if p['name'] not in ('prepare_joint_branch','return_joint_branch')]
        preserved=[p for p in revised['phases'] if not p['name'].startswith(('prepare_ompl_','return_ompl_'))]
        self.assertEqual(original,preserved)
        self.assertEqual(source['target_reference'],revised['target_reference'])
        forward=[p for p in revised['phases'] if p['name'].startswith('prepare_ompl_')]
        reverse=[p for p in revised['phases'] if p['name'].startswith('return_ompl_')]
        self.assertEqual(len(forward),2);self.assertEqual(len(reverse),2)
        for f,r in zip(forward,reverse[::-1]):
            self.assertEqual(f['waypoint_q'],r['waypoint_q'][::-1])
            self.assertEqual(f['duration_s'],r['duration_s'])
            self.assertEqual(f['end_gap_m'],.044);self.assertEqual(r['end_gap_m'],.096)
            self.assertEqual(f['rail_waypoint_positions_m'],[0.,0.])

    def test_per_edge_duration_matches_actual_single_smoothstep_and_bounds_speed(self):
        a=np.zeros(7);b=np.array([.04,.1,.2,.3,.4,.5,.6]);c=b+np.array([0.,-.3,0.,0.,0.,0.,0.])
        caps=np.array([.08]+[.35]*6)
        phases=timed_edges([a,b,c],caps,hz=120,name='prepare_ompl',gap=.044)
        for phase in phases:
            qs=np.asarray(phase['waypoint_q']);duration=phase['duration_s']
            self.assertTrue(np.all(1.5*np.abs(qs[-1]-qs[0])/duration<=caps+1e-10))
            n=round(duration*120);u=np.linspace(0,1,n+1);fraction=u*u*(3-2*u)
            commands=qs[0]+fraction[:,None]*(qs[1]-qs[0])
            self.assertTrue(np.all(np.abs(np.diff(commands,axis=0))*120<=caps+1e-8))
            self.assertEqual(phase['belt_local_y_m_s'],[0.,0.])

    def test_retimed_belt_window_matches_actual_reconstructed_native_phase_stream(self):
        source=fixture();source['belt_cycle']=dict(phases=['close'],start_after_settle_s=10.,duration_s=1.,
            full_open_gap_m=.096,reclose_requires_measured_open=True)
        begin,_,q=prefix_indices(source);mid=(q[0]+q[1])/2;mid[2]+=.2
        revised=splice_connected_plan(source,[q[0],mid,q[1]],[.08]+[.35]*6)
        tuples=reconstruct_phase_tuples(revised)
        prefix=sum(p[1] for p in tuples[:next(i for i,p in enumerate(tuples) if p[0]=='close')])
        self.assertAlmostEqual(revised['belt_cycle']['start_after_settle_s'],prefix)
        self.assertEqual(revised['belt_cycle']['duration_s'],1.)
        self.assertEqual(source['belt_cycle']['start_after_settle_s'],10.)
        expected_delta=sum(p[1] for p in tuples if p[0].startswith('prepare_ompl_'))-source['phases'][begin]['duration_s']
        self.assertAlmostEqual(prefix-10.,expected_delta)
        for p,tuple_row in zip(revised['phases'],tuples):
            self.assertEqual(p['name'],tuple_row[0]);self.assertEqual(p['duration_s'],tuple_row[1])
            np.testing.assert_array_equal(p['waypoint_q'],tuple_row[2])
            self.assertEqual([p['end_gap_m'],*p['belt_local_y_m_s']],list(tuple_row[3:]))

    def test_no_goal_change_or_fake_return_or_rail_motion_allowed(self):
        plan=fixture();_,_,q=prefix_indices(plan)
        for path in ([q[0]+.00001,q[1]],[q[0],q[1]+.00001]):
            with self.assertRaisesRegex(ValueError,'exact source and staging'):
                splice_connected_plan(plan,path,[.08]+[.35]*6)
        for mutate in (
            lambda p:p['phases'].__setitem__(7,copy.deepcopy(p['phases'][2])),
            lambda p:p['phases'][7]['waypoint_q'][1].__setitem__(1,.01),
            lambda p:p['phases'][2].update(rail_waypoint_positions_m=[0.,.01]),
            lambda p:p['phases'][2].update(belt_local_y_m_s=[.1,0.])):
            changed=copy.deepcopy(plan);mutate(changed)
            with self.assertRaises(ValueError):prefix_indices(changed)

    def test_no_dependency_install_or_asset_dependency_replacement_for_existing_bindings(self):
        import fcl,numpy,pxr
        origins=(fcl.__file__,numpy.__file__,pxr.__file__)
        _,metadata=existing_ompl()
        self.assertFalse(metadata['installation_changes'])
        self.assertEqual(metadata['version'],'2.0.1')
        self.assertEqual(origins,(fcl.__file__,numpy.__file__,pxr.__file__))

    def test_eight_manual_orderings_preserve_one_goal_suffix_and_exact_return(self):
        source=fixture();_,_,q=prefix_indices(source);distinct=set()
        old=copy.deepcopy(source)
        for policy in MANUAL_POLICIES:
            path=manual_prefix(q[0],q[-1],policy)
            np.testing.assert_array_equal(path[0],q[0])
            np.testing.assert_array_equal(path[-1],q[-1])
            self.assertTrue(np.isfinite(path).all())
            self.assertTrue(np.all(path[:,0]>=0))
            self.assertTrue(np.all(path[:,0]<=.75))
            self.assertTrue(np.all(np.abs(path[:,1:])<2*np.pi))
            distinct.add(path.tobytes())
            revised=splice_connected_plan(source,path,[.08]+[.35]*6)
            forward=[p for p in revised['phases'] if p['name'].startswith('prepare_ompl_')]
            reverse=[p for p in revised['phases'] if p['name'].startswith('return_ompl_')]
            for a,b in zip(forward,reverse[::-1]):
                self.assertEqual(a['waypoint_q'],b['waypoint_q'][::-1])
                self.assertEqual(a['duration_s'],b['duration_s'])
                self.assertEqual(b['end_gap_m'],.096)
            self.assertEqual(source,old)
        self.assertEqual(len(distinct),8)
        with self.assertRaisesRegex(ValueError,'Unknown bounded'):
            manual_prefix(q[0],q[-1],'random_goal_search')

    def test_full_environment_wrapper_audits_entry_and_return_and_retains_hard_evidence(self):
        class Adapter:
            scope='preparation';total_hits=0;total_diagnostics=0;queries=12;filtered_pairs=3
            hits=[];diagnostics=[];hit_pairs={};diagnostic_pairs={};metadata={};current_time_s=31.
            def prepare_path_sweep(self,plan,hz):self.plan=plan;self.hz=hz
        plan=fixture();adapter=Adapter()
        with patch('motion_self_collision.check_plan',return_value=dict(complete=True,samples=10)) as sampler:
            report=full_environment_trace(adapter,plan)
        self.assertIs(adapter.plan,plan)
        self.assertEqual(adapter.hz,120)
        self.assertEqual(report['selected_phases'],[p['name'] for p in plan['phases']])
        self.assertIn('approach',report['selected_phases']);self.assertIn('withdraw',report['selected_phases'])
        sampler.assert_called_once_with(adapter,plan,hz=120,cache_exact_repeats=False)
        self.assertTrue(report['passed']);self.assertTrue(report['complete'])
        adapter.total_hits=1;adapter.hits=[{'phase':'approach','environment_geom':'foreign_stem'}]
        adapter.hit_pairs={'pair':{'phase_sample_pairs':{'approach':1}}}
        with patch('motion_self_collision.check_plan',return_value=dict(complete=True,samples=10)):
            blocked=full_environment_trace(adapter,plan)
        self.assertFalse(blocked['passed']);self.assertEqual(blocked['status'],'blocked')
        self.assertEqual(blocked['hard_hits'],adapter.hits)
        adapter.total_hits=0
        with patch('motion_self_collision.check_plan',return_value=dict(complete=False,samples=3)):
            unknown=full_environment_trace(adapter,plan)
        self.assertFalse(unknown['passed']);self.assertFalse(unknown['complete'])


if __name__=='__main__':unittest.main()
