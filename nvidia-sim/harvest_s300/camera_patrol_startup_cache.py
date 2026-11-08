"""Exact, process-local reuse of decisive independent native startup probes.

This never provides a campaign checkpoint or skips initialization/contact guards
on an actual execution.  Different target trusses are NOT equivalent by name:
only identical materialized USD bytes, idle arguments, non-target manifest
context, code and parent checkpoint metadata can share a diagnostic decision.
Run this module with --self-test for its embedded offline regression tests.
"""
import copy
import hashlib
import json
from pathlib import Path


def _sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def _code_sha():
    root = Path(__file__).parent
    return hashlib.sha256(_encoded({p.name: _sha(p) for p in sorted(root.glob('*.py'))})).hexdigest()


def startup_identity(assembly, arguments, parent_metadata, *, runtime_identity=None):
    """Verify the exact dependency and bind a previously validated parent.

    ``parent_metadata`` is the actual parent checkpoint.json, not an invented
    plant/target key. The route checkpoint validator remains the caller's job.
    All arguments are retained in order; no physics option is normalized away.
    Only target/grasp reporting metadata is excluded from the manifest key:
    startup idle never commands these goals, while their physical differences
    remain covered by the exact dependency hash. Every other manifest field
    participates, conservatively preventing reuse across different guard scope.
    """
    from compact_motion_assets import verify_compact_dependencies
    assembly = Path(assembly).resolve()
    parent_metadata = Path(parent_metadata).resolve()
    if not isinstance(arguments, (list, tuple)) or any(not isinstance(a, str) for a in arguments):
        raise ValueError('Exact ordered idle argument strings required')
    arguments = list(arguments)
    if (arguments.count('--mode') != 1 or arguments.count('--seconds') != 1 or
            arguments[arguments.index('--mode') + 1:arguments.index('--mode') + 2] != ['idle'] or
            arguments[arguments.index('--seconds') + 1:arguments.index('--seconds') + 2] != ['3'] or
            any(a.startswith(('--resume-native-checkpoint', '--rail-to-y', '--start-gate')) for a in arguments)):
        raise ValueError('Cache is restricted to an independent 3-second idle probe')
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    if _sha(assembly / 'scene.usda') != manifest.get('scene_sha256'):
        raise ValueError('Startup source wrapper SHA mismatch')
    dependency = verify_compact_dependencies(assembly, manifest)
    if not dependency:
        raise ValueError('Exact materialized USDC dependency required')
    context = copy.deepcopy(manifest)
    for key in ('target', 'grasp_reference'):
        context.pop(key, None)
    identity = dict(schema='s300-exact-idle-startup-identity-v1',
        materialized_usdc_sha256=dependency['sha256'], wrapper_sha256=manifest['scene_sha256'],
        physics_arguments=arguments, route_parent_metadata_sha256=_sha(parent_metadata),
        guard_manifest_sha256=hashlib.sha256(_encoded(context)).hexdigest(),
        execution_code_sha256=_code_sha(), runtime_identity=runtime_identity or {})
    return dict(key=hashlib.sha256(_encoded(identity)).hexdigest(), identity=identity,
        assembly=str(assembly), parent_metadata=str(parent_metadata))


class StartupPreflightCache:
    """Bounded in-memory diagnostics; never rewrites source evidence files."""
    def __init__(self, max_entries=32):
        if isinstance(max_entries, bool) or not isinstance(max_entries, int) or max_entries < 1:
            raise ValueError('Positive integer cache bound required')
        self.max_entries = max_entries
        self._records = {}

    def store(self, identity, result, trial):
        """Admit only a process-complete decisive native proof, never unknown."""
        from run_row_harvest import preflight_result
        trial = Path(trial).resolve()
        current = startup_identity(identity['assembly'], identity['identity']['physics_arguments'], identity['parent_metadata'],
            runtime_identity=identity['identity']['runtime_identity'])
        if current['key'] != identity['key']:
            return False
        if (result.get('timeout') is not False or isinstance(result.get('returncode'), bool) or
                not isinstance(result.get('returncode'), int)):
            return False
        command = result.get('command', [])
        indices = [i for i, value in enumerate(command) if isinstance(value, str) and Path(value).name == 'run_motion.py']
        if len(indices) != 1:
            return False
        tail = command[indices[0] + 1:]
        expected = [identity['assembly'], '--output', str(trial)] + identity['identity']['physics_arguments']
        if tail != expected:
            return False
        proof = preflight_result(result, trial)
        decisive = ((proof['passed'] is True and proof['status'] == 'native_idle_preflight_passed') or
            (proof['passed'] is False and proof['can_defer_unused_placement'] is True and
             proof['status'] == 'placement_initialization_or_idle_physics_invalid'))
        if not decisive or proof['native_physics_initialization_observed'] is not True:
            return False
        evidence = dict(proof['evidence'])
        if not set(evidence).intersection({'initialization_validity.json', 'initialization_failure.json',
                'pre_restore_initialization.json', 'native_initialization_timing.json', 'route_start_native_validation.json'}):
            return False
        summary = trial / 'summary.json'
        if summary.is_file():
            if json.loads(summary.read_text()) != result.get('summary'):
                return False
            evidence['summary.json'] = dict(path=str(summary), sha256=_sha(summary))
        elif result.get('summary') is not None:
            return False
        # A claimed Python result without retained native file evidence is not
        # reusable. It is also not evidence of an impossible physical target.
        if not evidence or not all(Path(e['path']).is_file() and _sha(e['path']) == e['sha256'] for e in evidence.values()):
            return False
        payload = copy.deepcopy(dict(identity=identity, proof=proof, evidence=evidence))
        payload['sha256'] = hashlib.sha256(_encoded(payload)).hexdigest()
        self._records.pop(identity['key'], None)
        self._records[identity['key']] = payload
        while len(self._records) > self.max_entries:
            self._records.pop(next(iter(self._records)))
        return True

    def lookup(self, identity):
        """Return the original diagnostic only while its source proof is intact."""
        record = self._records.get(identity['key'])
        if record is None:
            return None
        current_identity = startup_identity(identity['assembly'], identity['identity']['physics_arguments'], identity['parent_metadata'],
            runtime_identity=identity['identity']['runtime_identity'])
        if current_identity['key'] != identity['key']:
            return None
        if record['identity']['identity'] != identity['identity']:
            return None
        current = {k: v for k, v in record.items() if k != 'sha256'}
        if hashlib.sha256(_encoded(current)).hexdigest() != record['sha256']:
            self._records.pop(identity['key'], None)
            return None
        for evidence in record['evidence'].values():
            path = Path(evidence['path'])
            if not path.is_file() or _sha(path) != evidence['sha256']:
                self._records.pop(identity['key'], None)
                return None
        proof = copy.deepcopy(record['proof'])
        proof['cache_reuse'] = dict(key=identity['key'], source_assembly=record['identity']['assembly'],
            requested_assembly=identity['assembly'], source_parent_metadata=record['identity']['parent_metadata'],
            same_materialized_usdc_and_arguments_and_parent=True,
            actual_new_execution=False, cached_startup_diagnostic_only=True,
            evidence=copy.deepcopy(record['evidence']),
            actual_execution_still_requires_full_initialization_and_guards=True)
        return proof


def _self_test():
    """Embedded tests keep this opt-in helper's only new source file isolated."""
    import tempfile
    import unittest

    class CacheTests(unittest.TestCase):
        def setUp(self):
            self.temp = tempfile.TemporaryDirectory()
            self.root = Path(self.temp.name)
            self.assembly = self.root / 'assembly';self.assembly.mkdir()
            (self.assembly / 'scene.usda').write_text('#usda 1.0\n')
            (self.assembly / 'composed.usdc').write_bytes(b'exact materialized bytes')
            self.manifest = dict(scene_sha256=_sha(self.assembly / 'scene.usda'),
                compact_assets=dict(materialized_dependency=dict(relative_path='composed.usdc',sha256=_sha(self.assembly / 'composed.usdc'))),
                target=dict(name='Truss_02_Fruit_01'),grasp_reference=dict(center=[0,0,1]),plant=dict(source_plant='Plant17'))
            self.save_manifest()
            self.parent = self.root / 'checkpoint.json';self.parent.write_text('{"time_s":12}')
            self.args = ['--headless','--hz','60','--contact-limit-mm','1','--mode','idle','--seconds','3']
            self.trial = self.root / 'trial';self.trial.mkdir()
            self.summary = dict(mode='idle',completed=True,stop_reason=None,initialization_valid=True,
                rail_destination=None,simulated_s=3.,native_new_break_count=0)
            self.result = dict(timeout=False,returncode=0,summary=self.summary,
                command=['python','run_motion.py',str(self.assembly),'--output',str(self.trial)]+self.args)
            (self.trial / 'summary.json').write_text(json.dumps(self.summary))
            (self.trial / 'initialization_validity.json').write_text('{"valid":true}')
            self.cache = StartupPreflightCache(2)
        def tearDown(self):self.temp.cleanup()
        def save_manifest(self):(self.assembly / 'motion_manifest.json').write_text(json.dumps(self.manifest))
        def identity(self):return startup_identity(self.assembly,self.args,self.parent,runtime_identity=dict(engine='test-only-native'))
        def test_exact_valid_and_no_new_execution_credit(self):
            identity=self.identity();self.assertTrue(self.cache.store(identity,self.result,self.trial))
            proof=self.cache.lookup(identity);self.assertTrue(proof['passed'])
            self.assertFalse(proof['cache_reuse']['actual_new_execution']);self.assertEqual(proof['diagnostic_break_harvest_credit'],0)
            proof['result']['returncode']=99;self.assertEqual(self.cache.lookup(identity)['result']['returncode'],0)
        def test_target_metadata_alone_vs_physical_change(self):
            first=self.identity();self.manifest['target']['name']='Truss_01_Fruit_01';self.save_manifest()
            self.assertEqual(first['key'],self.identity()['key'])
            self.manifest['plant']['source_plant']='Plant18';self.save_manifest()
            self.assertNotEqual(first['key'],self.identity()['key'])
            (self.assembly / 'composed.usdc').write_bytes(b'different collider masks')
            with self.assertRaises(ValueError):self.identity()
        def test_every_argument_parent_and_runtime_are_keyed(self):
            first=self.identity();self.args[2]='400'
            self.assertNotEqual(first['key'],self.identity()['key'])
            self.args[2]='60';self.parent.write_text('{"time_s":13}')
            self.assertNotEqual(first['key'],self.identity()['key'])
            with self.assertRaises(ValueError):startup_identity(self.assembly,['--mode','harvest','--seconds','3'],self.parent)
        def test_inconclusive_timeout_cli_errors_cannot_be_cached(self):
            identity=self.identity()
            for value in (None,True):
                bad=copy.deepcopy(self.result);bad['timeout']=value
                self.assertFalse(self.cache.store(identity,bad,self.trial))
            bad=copy.deepcopy(self.result);bad['summary']=None
            self.assertFalse(self.cache.store(identity,bad,self.trial))
            bad=copy.deepcopy(self.result);bad['command'][-1]='4'
            self.assertFalse(self.cache.store(identity,bad,self.trial))
        def test_actual_guard_failure_reuse_and_mutated_evidence_miss(self):
            bad=copy.deepcopy(self.result);bad['summary'].update(completed=False,initialization_valid=False,stop_reason='initialization_penetration_over_contact_policy')
            (self.trial / 'summary.json').write_text(json.dumps(bad['summary']))
            identity=self.identity();self.assertTrue(self.cache.store(identity,bad,self.trial))
            self.assertTrue(self.cache.lookup(identity)['can_defer_unused_placement'])
            (self.trial / 'summary.json').write_text('{}')
            self.assertIsNone(self.cache.lookup(identity))
        def test_bound_and_stale_input_identity(self):
            first=self.identity();self.assertTrue(self.cache.store(first,self.result,self.trial))
            self.parent.write_text('{"time_s":99}')
            self.assertIsNone(self.cache.lookup(first))
            self.assertFalse(self.cache.store(first,self.result,self.trial))
            for value in (1,2,3):
                self.parent.write_text(json.dumps(dict(time_s=value)))
                self.assertTrue(self.cache.store(self.identity(),self.result,self.trial))
            self.assertEqual(len(self.cache._records),2)
        def test_no_native_evidence_or_changed_proof_is_reused(self):
            identity=self.identity();(self.trial/'initialization_validity.json').unlink()
            self.assertFalse(self.cache.store(identity,self.result,self.trial))
            (self.trial/'initialization_validity.json').write_text('{"valid":true}')
            self.assertTrue(self.cache.store(identity,self.result,self.trial))
            self.cache._records[identity['key']]['proof']['passed']=False
            self.assertIsNone(self.cache.lookup(identity))
        def test_wrong_source_command_or_unretained_summary_misses(self):
            identity=self.identity();bad=copy.deepcopy(self.result);bad['command'][2]='/unrelated/assembly'
            self.assertFalse(self.cache.store(identity,bad,self.trial))
            (self.trial / 'summary.json').unlink()
            self.assertFalse(self.cache.store(identity,self.result,self.trial))

    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CacheTests))
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    import sys
    if sys.argv[1:] != ['--self-test']:
        raise SystemExit('Import this opt-in helper, or run --self-test; it never starts physics.')
    raise SystemExit(_self_test())
