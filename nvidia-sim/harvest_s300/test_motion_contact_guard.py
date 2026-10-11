import unittest
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

from motion_contact_buffer import BufferedContactRecorder
from motion_contact_guard import FruitContactGuard


class FruitGuardTests(unittest.TestCase):
    def setUp(self):
        self.root='/World/ElasticTarget'
        self.fruit=self.root+'/Truss_02_Fruit_09/Collider'
        self.other=self.root+'/Truss_01_Fruit_09/Collider'
        self.tool='/World/ActuatedTool/R_belt/PrototypeTread'
        fruits=[dict(name=n,path=self.root+'/'+n) for n in ('Truss_02_Fruit_09','Truss_01_Fruit_09')]
        self.manifest=dict(target=fruits[0],plant=dict(root=self.root,fruits=fruits))
        self.guard=FruitContactGuard(self.manifest,[self.tool],fruit_limit_m=.001)

    def event(self,a=None,b=None,depth=.0008,t=.1):
        return dict(a=a or self.tool,b=b or self.fruit,separation_m=-depth,force_n=1.25,t=t)

    def test_only_exact_tool_and_same_truss_fruit_pairs_relaxed_in_both_orders(self):
        self.assertFalse(self.guard.invalid(self.event()))
        self.assertFalse(self.guard.invalid(self.event(a=self.fruit,b=self.tool)))
        for a,b in [(self.tool,self.other),('/World/HarvestRobot/arm',self.fruit),
                    (self.tool,self.root+'/Truss_02_Rachis_1/Collider'),
                    ('/World/Ground',self.fruit),(self.tool,self.fruit+'_extra'),
                    (self.tool+'_extra',self.fruit)]:
            self.assertTrue(self.guard.invalid(self.event(a,b)))
        self.assertFalse(self.guard.invalid(self.event(depth=.001)))
        self.assertTrue(self.guard.invalid(self.event(depth=.001001)))

    def test_default_budget_retains_original_guard(self):
        guard=FruitContactGuard(self.manifest,[self.tool])
        self.assertTrue(guard.invalid(self.event()))
        self.assertFalse(guard.invalid(self.event(depth=.0005)))
        for value in (.0004,float('nan'),float('inf')):
            with self.assertRaises(ValueError):FruitContactGuard(self.manifest,[self.tool],fruit_limit_m=value)

    def test_fruit_fruit_budget_excludes_other_truss_and_stem(self):
        second=dict(name='Truss_02_Fruit_10',path=self.root+'/Truss_02_Fruit_10')
        self.manifest['plant']['fruits'].append(second)
        guard=FruitContactGuard(self.manifest,[self.tool],fruit_limit_m=.002,fruit_fruit_limit_m=.001)
        self.assertFalse(guard.invalid(self.event(a=self.fruit,b=second['path']+'/Collider')))
        self.assertTrue(guard.invalid(self.event(a=self.fruit,b=self.other)))
        self.assertTrue(guard.invalid(self.event(a=self.fruit,b=self.root+'/Stem_01/Collider')))

    def test_buffer_preserves_tolerated_raw_records_and_blocks_other_pairs(self):
        paths={1:self.tool,2:self.fruit,3:self.other}
        reader=BufferedContactRecorder(paths.__getitem__,60,pair_limit=self.guard.limit)
        headers=[SimpleNamespace(collider0=1,collider1=b,contact_data_offset=i,num_contact_data=1) for i,b in enumerate([2,3])]
        points=[SimpleNamespace(separation=-.0008,impulse=[0.,.01,0.]) for _ in headers]
        reader.record(headers,points,t=.1,phase='close')
        self.assertEqual(len(reader.materialize()),2)
        self.assertEqual(reader.materialize()[0]['force_n'],.6)
        self.assertEqual([e['b'] for e in reader.step_invalid_events()],[self.other])

    def test_warning_durations_count_distinct_physics_samples_and_gaps(self):
        events=[self.event(t=t) for t in (.1,.1,.2,.4)]
        evidence=self.guard.summarize(events,10)
        self.assertFalse(evidence['strict_0_5mm_passed'])
        self.assertEqual(evidence['configured_violation_contact_points'],0)
        row=evidence['exceedance_pairs'][0]
        self.assertEqual(row['contact_points'],4)
        self.assertEqual(row['physics_samples'],3)
        self.assertAlmostEqual(row['sampled_exceedance_s'],.3)
        self.assertAlmostEqual(row['longest_sampled_exceedance_s'],.2)

    def test_cli_rejects_unmapped_or_invalid_overrides_before_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            for flags in (['--fruit-contact-limit-mm','nan'],
                          ['--fruit-fruit-limit-mm','0.4'],
                          ['--contact-limit-mm','.5','--fruit-contact-limit-mm','1','--contact-backend','tensor'],
                          ['--contact-limit-mm','.5','--fruit-fruit-limit-mm','1','--num-envs','2']):
                output=Path(tmp)/'output'
                cmd=[sys.executable,str(Path(__file__).with_name('run_motion.py')),tmp,'--output',str(output),*flags]
                p=subprocess.run(cmd,capture_output=True,text=True)
                self.assertEqual(p.returncode,2,p.stderr)
                self.assertFalse(output.exists())


if __name__=='__main__':unittest.main()
