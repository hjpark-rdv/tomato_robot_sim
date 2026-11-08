"""Run the actual camera planner/connector seams without a native simulator."""
import ast
from contextlib import redirect_stderr
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from test_run_motion_conditioning import TREE, MAIN, actual_cli, code
from test_camera_prefix_connection import fixture


CAMERA = ['--camera-harvest','--whole-truss','--near-trusses-metadata','/inventory',
          '--harvest-profile','belt_open_close_cycle',
          '--approach-yaw-mode','truss-axis','--truss-order','highest-first',
          '--horizontal-ready-connector','joint','--preparation','horizontal_ready',
          '--hz','60','--verify-full-environment-preflight',
          '--checkpoint-cycle-states',
          '--released-fruit-ground-diagnostic','--released-fruit-gripper-support-diagnostic',
          '--released-fruit-same-truss-diagnostic',
          '--native-tool-collision-cache','/cache']


class CameraPrefixIntegrationTests(unittest.TestCase):
    def test_cli_requires_camera_full_gates_native_shapes_and_bounded_settings(self):
        default = actual_cli([])
        self.assertEqual(default.camera_prefix_connection,'off')
        self.assertIsNone(default.camera_harvest_ready_lift_offset_mm)
        self.assertEqual(default.camera_prefix_proposal_resolution,'fine')
        good = actual_cli(CAMERA+['--camera-prefix-connection','lift-then-arm',
                                 '--camera-harvest-ready-lift-offset-mm','150'])
        self.assertEqual(good.camera_harvest_ready_lift_offset_mm,150.)
        for bad in (['--camera-prefix-connection','ompl'],
                    CAMERA+['--camera-prefix-solve-seconds','31'],
                    CAMERA+['--camera-harvest-ready-lift-offset-mm','nan'],
                    CAMERA+['--camera-harvest-ready-lift-offset-mm','301'],
                    CAMERA+['--camera-prefix-connection','ompl','--preparation-opening','legacy-staging'],
                    CAMERA+['--camera-prefix-proposal-resolution','coarse'],
                    CAMERA+['--camera-prefix-connection','ompl','--contact-limit-mm','.5'],
                    CAMERA+['--camera-prefix-connection','ompl','--hz','120']):
            with self.subTest(bad=bad),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                actual_cli(bad)

    def test_harvest_only_lift_branch_is_not_applied_to_the_initial_scan(self):
        definition = next(n for n in MAIN.body if isinstance(n,ast.FunctionDef) and n.name=='make_plan')
        for override,global_mm in ((150.,0.),(None,25.)):
            a=actual_cli(CAMERA+['--ready-lift-offset-mm',str(global_mm)])
            a.camera_harvest_ready_lift_offset_mm=override
            context=dict(np=np,a=a,planning_kin=object(),reference_center=np.zeros(3),
                initial_robot_q=np.zeros(7),initial_grasp_depth_mm=a.grasp_depth_mm,
                recorded_prefix=None,rail_destination=None,resume_rail_origin=None,
                plan_motion=lambda *args,**kwargs:([],{'arguments':kwargs}))
            exec(code([definition]),context)
            initial=context['make_plan']()[1]['arguments']
            suffix=context['make_plan'](camera_boundary=True)[1]['arguments']
            self.assertEqual(initial['ready_lift_offset_m'],global_mm/1000)
            self.assertEqual(suffix['ready_lift_offset_m'],global_mm/1000 if override is None else override/1000)
            for key in initial.keys()-{'ready_lift_offset_m'}:
                np.testing.assert_equal(initial[key],suffix[key])

    def test_actual_connector_seam_preserves_raw_plan_and_changes_commands_only_after_success(self):
        seam=next(n for n in ast.walk(TREE) if isinstance(n,ast.If) and
                  ast.unparse(n.test)=="a.camera_harvest and a.camera_prefix_connection != 'off'")
        with tempfile.TemporaryDirectory()as tmp:
            folder=Path(tmp);connected=folder/'verified.json';source=fixture()
            connected.write_text(json.dumps(source))
            a=actual_cli(CAMERA+['--camera-prefix-connection','lift-then-arm'])
            context=dict(a=a,folder=folder,boundary_checkpoint=folder/'checkpoint',
                group={'truss_key':'Truss_00'},new_planning=copy.deepcopy(source),
                new_phases=['unchanged until qualified'],json=json,record={})
            with patch('camera_prefix_connection.connect_external_camera_prefix',
                       side_effect=ValueError('incomplete source audit')):
                with self.assertRaisesRegex(ValueError,'incomplete source audit'):
                    exec(code([seam]),context)
            self.assertEqual(context['new_phases'],['unchanged until qualified'])
            self.assertEqual(context['record'],{})
            self.assertEqual(json.loads((folder/'plan_before_prefix_connection.json').read_text()),source)
            with patch('camera_prefix_connection.connect_external_camera_prefix',return_value=connected)as call:
                exec(code([seam]),context)
            self.assertEqual([p[0]for p in context['new_phases']],[p['name']for p in source['phases']])
            self.assertFalse(context['record']['camera_prefix_connection']['physical_arrival_proven'])
            self.assertEqual(call.call_args.kwargs['strategy'],'lift-then-arm')
            self.assertEqual(call.call_args.args[2],context['boundary_checkpoint'])

    def test_startup_probe_removes_camera_connector_and_suffix_lift_options(self):
        from run_row_harvest import startup_preflight_arguments
        args=CAMERA+['--camera-prefix-connection','ompl','--camera-prefix-solve-seconds','30',
                     '--camera-prefix-proposal-resolution','coarse','--camera-prefix-seed','2028',
                     '--camera-harvest-ready-lift-offset-mm','150']
        stripped=startup_preflight_arguments(args)
        self.assertNotIn('--camera-harvest-ready-lift-offset-mm',stripped)
        self.assertNotIn('--camera-prefix-connection',stripped)
        self.assertNotIn('--camera-prefix-solve-seconds',stripped)
        self.assertNotIn('--camera-prefix-proposal-resolution',stripped)
        self.assertNotIn('--camera-prefix-seed',stripped)
        self.assertEqual(stripped[-4:],['--mode','idle','--seconds','3'])

    def test_actual_audit_seam_reuses_only_certified_bytes_and_keeps_legacy_auditors(self):
        branch=next(n for n in ast.walk(TREE) if isinstance(n,ast.If) and
                    ast.unparse(n.test)=='connected is None')
        environment=next(n for n in ast.walk(TREE) if isinstance(n,ast.If) and
                    any(isinstance(stmt,ast.ImportFrom) and stmt.module=='rail_environment_preflight' and
                        any(alias.name=='audit_external_rail' for alias in stmt.names) for stmt in n.body) and
                    any(isinstance(child,ast.Name) and child.id=='reused_camera_audits' for child in ast.walk(n)))
        for reuse in (False,True):
            with self.subTest(reuse=reuse),tempfile.TemporaryDirectory()as tmp:
                folder=Path(tmp);source=folder/'connected.json'
                source.write_text(json.dumps(fixture(),indent=2)+'\n')
                proof={'audit_bytes_identical':True,'new_FCL_queries':0}
                context=dict(a=actual_cli(CAMERA),folder=folder,json=json,
                    connected=source if reuse else None,reused_camera_audits=None,
                    new_planning=fixture(),boundary_checkpoint=folder/'cp',
                    group={'truss_key':'Truss_00'},record={'camera_prefix_connection':{}})
                with patch('camera_prefix_connection.reuse_verified_camera_audits',return_value=proof)as cached,\
                     patch('rail_environment_preflight.audit_external_rail')as env,\
                     patch.dict(context,{'audit_external':unittest.mock.Mock()}) as _:
                    exec(code([branch,environment]),context)
                    self.assertEqual(cached.call_count,int(reuse))
                    self.assertEqual(env.call_count,int(not reuse))
                    self.assertEqual(context['audit_external'].call_count,int(not reuse))
                self.assertEqual(context['record']['environment_preflight']['status'],'passed_complete')
                if reuse:
                    self.assertEqual((folder/'plan.json').read_bytes(),source.read_bytes())
                    self.assertEqual(context['record']['camera_prefix_connection']['verified_audit_reuse'],proof)


if __name__=='__main__':unittest.main()
