"""Mocked CPU orchestration tests; no native simulator or GPU process."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import benchmark_motion as benchmark


class ProcessTree:
    def __init__(self,pid,cpu=0,rss=0,children=()):
        self.pid=pid;self.cpu=cpu;self.rss=rss;self._children=children
    def children(self,recursive=False):return self._children
    def cpu_percent(self):return self.cpu
    def memory_info(self):return SimpleNamespace(rss=self.rss)
    def is_running(self):return True
    def status(self):return 'running'


class Clock:
    def __init__(self):self.now=100.
    def counter(self):self.now+=.01;return self.now
    def sleep(self,value):self.now+=value


class Child:
    """Fake executor obeys ready/gate contract and writes measured loop times."""
    def __init__(self,command,index,clock,initial_valid=True):
        self.pid=90000+index;self.returncode=None;self.index=index;self.clock=clock
        self.folder=Path(command[command.index('--output')+1]);self.folder.mkdir()
        self.gate=Path(command[command.index('--start-gate')+1])
        self.ready=dict(pid=self.pid,initial_valid=initial_valid,ready_monotonic_s=clock.now)
        (self.folder/'ready.json').write_text(json.dumps(self.ready))
    def poll(self):
        if self.returncode is None and self.gate.exists():
            release=json.loads(self.gate.read_text())['released_monotonic_s']
            start=release+.1*self.index;end=start+.5+.2*self.index
            self.clock.now=max(self.clock.now,end)
            summary=dict(completed=True,initialization_valid=True,stop_reason=None,
                         loop_started_monotonic_s=start,loop_finished_monotonic_s=end,
                         simulated_s=1.,aggregate_simulated_s=1.)
            (self.folder/'summary.json').write_text(json.dumps(summary));self.returncode=0
        return self.returncode
    def wait(self,timeout=None):return self.returncode


class BenchmarkTests(unittest.TestCase):
    def test_serial_tree_and_gpu_sample_keep_original_combined_schema(self):
        child=ProcessTree(2,30,200);parent=ProcessTree(1,50,100,[child]);tracked={1:parent}
        self.assertEqual(benchmark.sample_tree(parent,tracked),dict(cpu_percent=80,rss_bytes=300))
        with patch.object(benchmark,'sample_gpu',return_value=dict(gpu_percent=95,gpu_memory_mib=2000)):
            self.assertEqual(benchmark.sample(parent,tracked),
                             dict(cpu_percent=80,rss_bytes=300,gpu_percent=95,gpu_memory_mib=2000))
        self.assertIn(2,tracked)

    def test_cpu_only_and_all_paths_checked_before_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            good=[dict(name='a',arguments=['--device','cpu'])]
            benchmark.validate_parallel_cases(good,root,2)
            for jobs in (4,8,16,24):benchmark.validate_parallel_cases(good,root,jobs)
            with self.assertRaisesRegex(ValueError,'Parallel jobs'):
                benchmark.validate_parallel_cases(good,root,3)
            for arguments in (['--device','cuda:0'],['--device=cuda:0'],['--start-gate','bad']):
                with self.assertRaises(ValueError):benchmark.validate_parallel_cases([dict(name='a',arguments=arguments)],root,2)
            with self.assertRaises(ValueError):benchmark.validate_parallel_cases(good+good,root,2)
            (root/'a.log').write_text('preserve')
            with self.assertRaisesRegex(RuntimeError,'overwrite'):benchmark.validate_parallel_cases(good,root,2)
            self.assertEqual((root/'a.log').read_text(),'preserve')

    def test_parallel_throughput_uses_shared_loop_window_not_sum_rates(self):
        rows=[]
        for name,start,end in [('a',10.,12.),('b',10.5,13.)]:
            rows.append(dict(name=name,returncode=0,timeout=False,summary=dict(
                completed=True,initialization_valid=True,stop_reason=None,
                loop_started_monotonic_s=start,loop_finished_monotonic_s=end,aggregate_simulated_s=2.)))
        result=benchmark.aggregate_parallel_group(rows,group_started=1.,gate_released=9.,group_finished=14.)
        self.assertTrue(result['throughput_eligible'])
        self.assertEqual(result['aggregate_sim_s_per_wall_s'],4./3.)
        self.assertEqual(result['group_startup_wall_s'],8.)
        self.assertEqual(result['loop_start_spread_s'],.5)

    def test_one_invalid_trial_excludes_whole_group_rate(self):
        rows=[dict(name='a',returncode=0,summary=dict(completed=True,initialization_valid=True,
              stop_reason=None,loop_started_monotonic_s=10.,loop_finished_monotonic_s=11.,simulated_s=1.)),
              dict(name='b',returncode=0,summary=dict(completed=False,initialization_valid=True,
              stop_reason='penetration_over_0_5mm',loop_started_monotonic_s=10.,
              loop_finished_monotonic_s=10.1,simulated_s=.1))]
        result=benchmark.aggregate_parallel_group(rows,group_started=1.,gate_released=9.,group_finished=12.)
        self.assertFalse(result['throughput_eligible']);self.assertIsNone(result['aggregate_sim_s_per_wall_s'])
        self.assertIn('b:penetration_over_0_5mm',result['exclusion_reasons'])

    def run_group(self,directory,invalid_index=None,workers=2):
        output=Path(directory);clock=Clock();children=[]
        cases=[dict(name=chr(ord('a')+i),arguments=['--device','cpu']) for i in range(workers)]
        args=SimpleNamespace(output=output,python=Path('/mock/python.sh'),assembly=Path('/mock/assembly'),timeout=20.)
        def launch(command,**kwargs):
            child=Child(command,len(children),clock,initial_valid=len(children)!=invalid_index)
            children.append(child);return child
        def stop(child):child.returncode=-2
        with patch.object(benchmark.subprocess,'Popen',side_effect=launch),\
             patch.object(benchmark.psutil,'Process',side_effect=lambda pid:ProcessTree(pid)),\
             patch.object(benchmark.os,'getpgid',side_effect=lambda pid:pid),\
             patch.object(benchmark,'sample_tree',return_value=dict(cpu_percent=100,rss_bytes=1234)),\
             patch.object(benchmark,'sample_gpu',return_value=dict(gpu_percent=0,gpu_memory_mib=100)) as gpu,\
             patch.object(benchmark.time,'perf_counter',side_effect=clock.counter),\
             patch.object(benchmark.time,'sleep',side_effect=clock.sleep),\
             patch.object(benchmark,'_stop_child',side_effect=stop):
            rows,group=benchmark.run_parallel_group(args,cases,0)
        return rows,group,gpu.call_count

    def test_ready_native_pid_accepts_owned_python_wrapper_descendant(self):
        native=ProcessTree(101);wrapper=ProcessTree(100,children=[native])
        with patch.object(benchmark.os,'getpgid',return_value=100):
            identity=benchmark.owned_ready_process(wrapper,101)
        self.assertEqual(identity['wrapper_pid'],100)
        self.assertEqual(identity['native_pid'],101)
        self.assertTrue(identity['native_is_descendant'])

    def test_ready_native_pid_rejects_unrelated_group_or_unrelated_pid(self):
        wrapper=ProcessTree(100,children=[ProcessTree(101)])
        with patch.object(benchmark.os,'getpgid',return_value=999):
            with self.assertRaisesRegex(ValueError,'group_mismatch'):benchmark.owned_ready_process(wrapper,101)
        with patch.object(benchmark.os,'getpgid',return_value=100):
            with self.assertRaisesRegex(ValueError,'not_descendant'):benchmark.owned_ready_process(wrapper,999)

    def test_ready_native_pid_rejects_zombie_executor(self):
        native=ProcessTree(101);native.status=lambda:'zombie'
        wrapper=ProcessTree(100,children=[native])
        with patch.object(benchmark.os,'getpgid',return_value=100):
            with self.assertRaisesRegex(ValueError,'not_alive'):benchmark.owned_ready_process(wrapper,101)

    def test_orchestration_releases_after_all_ready_and_records_resources(self):
        with tempfile.TemporaryDirectory() as directory:
            rows,group,gpu_calls=self.run_group(directory)
            self.assertTrue(group['throughput_eligible'])
            self.assertEqual(len(rows),2)
            self.assertAlmostEqual(group['aggregate_sim_s_per_wall_s'],2/.8)
            gate=json.loads(Path(group['start_gate']).read_text())
            self.assertEqual(len(gate['children']),2)
            samples=json.loads(Path(group['global_resource_samples']).read_text())
            self.assertEqual(gpu_calls,len(samples))
            self.assertEqual(len(samples[0]['children']),2)
            for row in rows:
                self.assertIn('--start-gate',row['command'])
                child_samples=json.loads(Path(row['resource_samples']).read_text())
                self.assertNotIn('gpu_percent',child_samples[0])

    def test_invalid_ready_never_releases_gate_or_reports_throughput(self):
        with tempfile.TemporaryDirectory() as directory:
            rows,group,_=self.run_group(directory,invalid_index=1)
            self.assertFalse(Path(group['start_gate']).exists())
            self.assertFalse(group['throughput_eligible'])
            self.assertIsNone(group['aggregate_sim_s_per_wall_s'])
            self.assertIn('b:ready_initialization_invalid_or_unknown',group['exclusion_reasons'])
            self.assertTrue(all(row['returncode']==-2 for row in rows))

    def test_bounded_twenty_four_worker_group_waits_for_all_and_uses_one_gpu_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            rows,group,gpu_calls=self.run_group(directory,workers=24)
            self.assertEqual(group['jobs'],24)
            self.assertEqual(len(rows),24)
            self.assertTrue(group['throughput_eligible'])
            gate=json.loads(Path(group['start_gate']).read_text())
            self.assertEqual(len(gate['children']),24)
            samples=json.loads(Path(group['global_resource_samples']).read_text())
            self.assertEqual(gpu_calls,len(samples))
            self.assertEqual(len(samples[0]['children']),24)

    def test_missing_or_before_gate_timestamps_are_incomplete_evidence(self):
        rows=[dict(name='a',returncode=0,summary=dict(completed=True,initialization_valid=True,stop_reason=None,
              loop_started_monotonic_s=8.,loop_finished_monotonic_s=10.,simulated_s=1.))]
        result=benchmark.aggregate_parallel_group(rows,group_started=1.,gate_released=9.,group_finished=12.)
        self.assertFalse(result['throughput_eligible'])
        self.assertIn('a:loop_started_before_gate',result['exclusion_reasons'])
        rows[0]['summary'].pop('loop_started_monotonic_s')
        result=benchmark.aggregate_parallel_group(rows,group_started=1.,gate_released=9.,group_finished=12.)
        self.assertFalse(result['throughput_eligible'])


if __name__=='__main__':unittest.main()
