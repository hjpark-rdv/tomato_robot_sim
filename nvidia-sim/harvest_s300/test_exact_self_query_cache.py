import unittest
from unittest.mock import patch
import numpy as np
import test_rail_self_collision as rail_fixture
from motion_self_collision import check_plan


def plan(*, gap=.044, rail=False, tiny_q=0.):
    q=np.zeros((2,7));q[1,0]=tiny_q
    phase=dict(name='hold',waypoint_q=q.tolist(),duration_s=2.,end_gap_m=gap)
    if rail:phase['rail_waypoint_positions_m']=[0.,.2]
    return dict(phases=[phase])


class ExactConfigurationCacheTests(unittest.TestCase):
    def compare(self,value):
        checker=rail_fixture.RailSelfCollisionTests().checker()
        original=check_plan(checker,value,hz=120,cache_exact_repeats=False)
        with patch.object(checker,'check',wraps=checker.check) as spy:
            cached=check_plan(checker,value,hz=120)
            self.assertEqual(spy.call_count,cached['query_cache']['exact_configuration_queries'])
        for key in original:
            if key!='query_cache':self.assertEqual(original[key],cached[key],key)
        self.assertEqual(cached['samples'],
            sum(cached['query_cache'][key] for key in
                ('exact_configuration_queries','reused_exact_configuration_queries')))
        return cached

    def test_identical_hold_checks_every_timestamp_but_queries_once(self):
        result=self.compare(plan())
        self.assertEqual(result['samples'],241)
        self.assertEqual(result['query_cache']['exact_configuration_queries'],1)

    def test_different_jaw_gaps_are_never_reused(self):
        result=self.compare(plan(gap=.045))
        self.assertEqual(result['query_cache']['reused_exact_configuration_queries'],0)

    def test_different_rail_positions_are_never_reused(self):
        result=self.compare(plan(rail=True))
        self.assertEqual(result['query_cache']['reused_exact_configuration_queries'],0)

    def test_subtolerance_q_changes_are_not_rounded(self):
        result=self.compare(plan(tiny_q=1e-15))
        self.assertEqual(result['query_cache']['reused_exact_configuration_queries'],0)

    def test_stateful_foreign_checker_keeps_every_call(self):
        class Stateful:
            metadata={}
            def __init__(self):self.calls=0
            def check(self,q,gap):
                self.calls+=1
                return [{'collision':True}] if self.calls==2 else []
        checker=Stateful();result=check_plan(checker,plan())
        self.assertEqual(checker.calls,2)
        self.assertFalse(result['passed'])
        self.assertFalse(result['query_cache']['enabled'])


if __name__=='__main__':unittest.main()
