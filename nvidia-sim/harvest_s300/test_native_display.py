"""Read-only display protocol and mocked native-launch tests; no Kit or GPU."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import benchmark_motion as benchmark
from native_display import NativeDisplayError,validated_native_display_environment


class NativeDisplayTests(unittest.TestCase):
    def probe(self,valid=(':0',),*,timeout=(),empty_reply=()):
        calls=[]
        def run(command,**kwargs):
            self.assertEqual(command[:2],['xdpyinfo','-display'])
            self.assertEqual(kwargs['timeout'],2.);self.assertFalse(kwargs['check'])
            display=command[2];calls.append((display,copy.deepcopy(kwargs)))
            self.assertEqual(kwargs['env']['DISPLAY'],display)
            if display in timeout:raise subprocess.TimeoutExpired(command,2.)
            return SimpleNamespace(returncode=0 if display in valid or display in empty_reply else 1,
                stdout='name of display: '+display+'\nversion number: 11.0\n' if display in valid else '',
                stderr='' if display in valid else 'unable to open display')
        return run,calls

    def test_stale_forwarding_display_uses_verified_existing0_without_environment_mutation(self):
        source={'DISPLAY':':194','XAUTHORITY':'/existing/auth','OMNI_KIT_ALLOW_ROOT':'1'};before=copy.deepcopy(source)
        run,calls=self.probe();env,proof=validated_native_display_environment(source,probe_runner=run)
        self.assertEqual(source,before);self.assertEqual(env['DISPLAY'],':0')
        self.assertEqual([row[0]for row in calls],[':194',':0'])
        self.assertEqual(proof['selection'],'existing_display0_fallback')
        self.assertFalse(proof['xserver_created']);self.assertFalse(proof['gpu_reset'])
        self.assertFalse(proof['vulkan_device_creation_verified'])

    def test_protocol_reply_required_even_if_socket_or_proxy_exits_successfully(self):
        run,calls=self.probe(empty_reply=(':stale',))
        env,proof=validated_native_display_environment({'DISPLAY':':stale'},probe_runner=run)
        self.assertEqual(env['DISPLAY'],':0');self.assertEqual(proof['attempts'][0]['returncode'],0)
        self.assertEqual(proof['attempts'][0]['status'],'display_query_failed')

    def test_valid_inherited_display_preserved_and_no_display_defaults_to_verified0(self):
        run,calls=self.probe(valid=(':live',':0'))
        env,proof=validated_native_display_environment({'DISPLAY':':live'},probe_runner=run)
        self.assertEqual(env['DISPLAY'],':live');self.assertEqual(len(calls),1)
        self.assertEqual(proof['selection'],'inherited_display')
        env,proof=validated_native_display_environment({},probe_runner=run)
        self.assertEqual(env['DISPLAY'],':0');self.assertEqual(calls[-1][0],':0')

    def test_explicit_override_is_validated_and_never_silently_replaced(self):
        run,calls=self.probe(valid=(':0',':chosen'))
        env,proof=validated_native_display_environment({'DISPLAY':':stale','S300_NATIVE_DISPLAY':':chosen'},probe_runner=run)
        self.assertEqual(env['DISPLAY'],':chosen');self.assertEqual(proof['selection'],'explicit_override')
        self.assertEqual([c[0]for c in calls],[':chosen'])
        calls.clear()
        with self.assertRaises(NativeDisplayError)as failure:
            validated_native_display_environment({'DISPLAY':':0','S300_NATIVE_DISPLAY':':bad'},probe_runner=run)
        self.assertEqual([c[0]for c in calls],[':bad'])
        self.assertTrue(failure.exception.proof['explicit_override'])
        with self.assertRaises(NativeDisplayError):
            validated_native_display_environment({'DISPLAY':':0','S300_NATIVE_DISPLAY':''},probe_runner=run)

    def test_each_probe_bounded_and_missing_tool_or_all_invalid_fail_closed(self):
        run,calls=self.probe(valid=(),timeout=(':stale',':0'))
        with self.assertRaises(NativeDisplayError)as failure:
            validated_native_display_environment({'DISPLAY':':stale'},probe_runner=run)
        self.assertEqual(len(calls),2)
        self.assertTrue(all(row['status']=='display_query_timeout'for row in failure.exception.proof['attempts']))
        def missing(*args,**kwargs):raise FileNotFoundError('xdpyinfo')
        with self.assertRaises(NativeDisplayError)as failure:
            validated_native_display_environment({},probe_runner=missing)
        self.assertEqual(failure.exception.proof['attempts'][0]['status'],'display_query_unavailable')
        for timeout in (0,2.1,float('nan'),True):
            with self.assertRaises(ValueError):validated_native_display_environment({},probe_runner=run,timeout_s=timeout)


class NativeDisplayLaunchTests(unittest.TestCase):
    probe=NativeDisplayTests.probe
    def test_serial_physics_only_launch_validates_display_and_records_choice(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);cases=root/'cases.json';out=root/'out';launches=[]
            cases.write_text(json.dumps([dict(name='fixture',arguments=['--physics-only','--headless'])]))
            probe,calls=self.probe()
            def validate(env):return validated_native_display_environment(env,probe_runner=probe)
            def launch(command,**kwargs):
                launches.append((command,kwargs));folder=Path(command[command.index('--output')+1]);folder.mkdir()
                (folder/'summary.json').write_text(json.dumps(dict(completed=True,physics_wall_s=1.)))
                return SimpleNamespace(pid=123,returncode=0,poll=lambda:0)
            with patch.dict(os.environ,{'DISPLAY':':194','PYTHONHOME':'old','VIRTUAL_ENV':'old'},clear=True),\
                 patch.object(benchmark,'validated_native_display_environment',side_effect=validate),\
                 patch.object(benchmark.subprocess,'Popen',side_effect=launch),\
                 patch.object(benchmark.psutil,'Process',return_value=SimpleNamespace(pid=123)),\
                 patch.object(sys,'argv',['benchmark_motion.py','/fixture',str(cases),'--output',str(out)]):
                self.assertEqual(benchmark.main(),0)
            self.assertEqual(len(launches),1);env=launches[0][1]['env'];self.assertEqual(env['DISPLAY'],':0')
            self.assertNotIn('PYTHONHOME',env);self.assertNotIn('VIRTUAL_ENV',env)
            self.assertIn('--physics-only',launches[0][0]);self.assertEqual(len(calls),2)
            proof=json.loads((out/'fixture_display_precheck.json').read_text())
            self.assertEqual(proof['selected_display'],':0')
            self.assertEqual(json.loads((out/'cases_results.json').read_text())[0]['display_precheck'],proof)

    def test_unavailable_serial_display_writes_failure_but_never_launches_native(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);cases=root/'cases.json';out=root/'out'
            cases.write_text(json.dumps([dict(name='fixture',arguments=['--headless'])]))
            probe,_=self.probe(valid=())
            with patch.dict(os.environ,{'DISPLAY':':194'},clear=True),\
                 patch.object(benchmark,'validated_native_display_environment',side_effect=lambda env:
                    validated_native_display_environment(env,probe_runner=probe)),\
                 patch.object(benchmark.subprocess,'Popen')as launch,\
                 patch.object(sys,'argv',['benchmark_motion.py','/fixture',str(cases),'--output',str(out)]):
                with self.assertRaises(NativeDisplayError):benchmark.main()
            launch.assert_not_called();self.assertFalse((out/'fixture').exists())
            self.assertEqual(json.loads((out/'fixture_display_precheck.json').read_text())['status'],'no_valid_existing_display')

    def test_parallel_group_checks_all_case_displays_before_any_native_launch(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);probe,_=self.probe()
            args=SimpleNamespace(output=root,assembly=Path('/fixture'),python=Path('/fixture/python.sh'),timeout=20.)
            cases=[dict(name='valid',arguments=['--physics-only'],environment=dict(DISPLAY=':0')),
                dict(name='invalid',arguments=['--physics-only'],environment=dict(S300_NATIVE_DISPLAY=':bad'))]
            with patch.dict(os.environ,{'DISPLAY':':194'},clear=True),\
                 patch.object(benchmark,'validated_native_display_environment',side_effect=lambda env:
                    validated_native_display_environment(env,probe_runner=probe)),\
                 patch.object(benchmark.subprocess,'Popen')as launch:
                with self.assertRaises(NativeDisplayError):benchmark.run_parallel_group(args,cases,0)
            launch.assert_not_called();self.assertTrue((root/'valid_display_precheck.json').is_file())
            self.assertEqual(json.loads((root/'invalid_display_precheck.json').read_text())['requested_display'],':bad')


if __name__=='__main__':unittest.main()
