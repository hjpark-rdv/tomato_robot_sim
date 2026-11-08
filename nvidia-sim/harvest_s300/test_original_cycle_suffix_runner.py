"""Real runner integration for an original-clock, no-retiming terminal suffix.

No native simulator: execute the runner's actual CLI/control AST and the same
immutable checkpoint loader used by native execution. Proof validation itself
has separate positive/negative tests in test_terminal_motion_recovery.
"""
import ast
import copy
from contextlib import redirect_stderr
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from motion_kinematics import belt_speed_fraction,cycle_opening_check
from rail_motion import rail_command
from test_run_motion_conditioning import actual_cli,code,LOOP,MAIN


class OriginalCycleSuffixRunnerTests(unittest.TestCase):
    def source_plan(self):
        names=['ready_hold','approach','close','cycle_open','cycle_reclose',
               'cycle_final_open','withdraw','return_joint_branch','ready_close','hold']
        durations=[.5]+[1.]*(len(names)-1)
        gaps=[.044,.096,.028,.096,.028,.096,.096,.096,.044,.044]
        rows=[];phases=[]
        for i,(name,duration,gap) in enumerate(zip(names,durations,gaps)):
            qs=np.array([np.arange(7)*.1+i*.02,np.arange(7)*.1+(i+1)*.02])
            speed=(.05,.05) if name in names[2:6] else (0.,0.)
            rows.append(dict(name=name,duration_s=duration,waypoint_q=qs.tolist(),
                end_gap_m=gap,belt_local_y_m_s=list(speed),
                rail_waypoint_positions_m=[-.4,-.4],rail_native_command_positions_m=[-.400001,-.400001]))
            phases.append((name,duration,qs.copy(),gap,*speed))
        plan=dict(phases=rows,belt_cycle=dict(phases=names[2:6],start_after_settle_s=1.5,
            duration_s=4.,withdraw_requires_measured_open=True,full_open_gap_m=.096,
            opening_tracking_tolerance_m=.0005))
        return phases,plan

    def phase_code(self):
        branch=next(n for n in ast.walk(LOOP) if isinstance(n,ast.If) and
            isinstance(n.test,ast.BoolOp) and "'harvest', 'lift', 'recovery'" in ast.unparse(n.test))
        return code([branch])

    def command_at(self,program,phases,plan,elapsed,*,forward=False,source_offset=0.,source_time=None):
        q=np.zeros((1,10));q[0,8:]=[-.026,.026]
        env=dict(a=SimpleNamespace(mode='recovery' if forward else 'harvest',
                resume_original_cycle_suffix=forward,belt_ramp_seconds=.25),
            planning=plan,phases=phases,rail_destination=None,route_start_context=None,
            railid=0,rids=list(range(1,8)),jawids=[8,9],
            rail_command=rail_command,cycle_start=0. if forward else 2.,
            t=elapsed-source_offset if forward else elapsed+2. if source_time is None else source_time,
            qcommand=np.zeros((1,10)),
            q=q,phase='settle',gap=.044,speed=(0.,0.),rail_arrival_feedback=None,
            preparation_open_feedback={},entry_open_feedback=None,cycle_open_feedback=None,
            cycle_final_open_feedback=None,release_relief_feedback=None,stop_reason=None,
            cycle_opening_check=cycle_opening_check,belt_speed_fraction=belt_speed_fraction)
        exec(program,env)
        return env

    def test_every_remaining_command_retains_original_curve_gap_phase_rail_and_belt_ramp(self):
        from motion_continuation import execution_duration_s
        phases,original=self.source_plan();checkpoint_tick=248;offset=checkpoint_tick/60-2.
        suffix=copy.deepcopy(original)
        suffix['terminal_forward_suffix']=dict(schema='s300-terminal-forward-suffix-clock-v1',
            initial_source_elapsed_s=offset,
            remaining_duration_s=sum(p[1]for p in phases)-offset,
            source_hz=60,source_checkpoint_tick=checkpoint_tick,source_cycle_start_s=2.)
        program=self.phase_code();duration=execution_duration_s(suffix)
        for tick in range(round(duration*60)):
            source_time=(checkpoint_tick+tick)/60;elapsed=source_time-2.
            reference=self.command_at(program,phases,original,elapsed,source_time=source_time)
            actual=self.command_at(program,phases,suffix,elapsed,forward=True,source_offset=offset)
            with self.subTest(tick=tick):
                np.testing.assert_array_equal(actual['qcommand'],reference['qcommand'])
                self.assertEqual(actual['gap'],reference['gap'])
                self.assertEqual(actual['speed'],reference['speed'])
                self.assertEqual(actual['phase'],reference['phase'])
                self.assertEqual(actual['stop_reason'],reference['stop_reason'])
        # A settled/retimed restart would begin at ready_hold and zero speed.
        initial=self.command_at(program,phases,suffix,offset,forward=True,source_offset=offset)
        self.assertEqual(initial['phase'],'close')
        self.assertGreater(initial['speed'][0],0.)
        self.assertNotEqual(initial['gap'],.044)

    def test_without_optin_existing_command_arrays_do_not_consume_suffix_clock(self):
        phases,original=self.source_plan();program=self.phase_code()
        with_metadata=copy.deepcopy(original)
        with_metadata['terminal_forward_suffix']=dict(schema='s300-terminal-forward-suffix-clock-v1',
            initial_source_elapsed_s=2.,remaining_duration_s=7.5)
        for elapsed in [0.,.3,.75,1.75,4.5,8.5]:
            normal=self.command_at(program,phases,original,elapsed)
            unchanged=self.command_at(program,phases,with_metadata,elapsed)
            np.testing.assert_array_equal(normal['qcommand'],unchanged['qcommand'])
            self.assertEqual(normal['gap'],unchanged['gap'])
            self.assertEqual(normal['speed'],unchanged['speed'])
            self.assertEqual(normal['phase'],unchanged['phase'])

    def test_forward_jaws_follow_original_curve_instead_of_reverse_recovery_fixed_targets(self):
        branch=next(n for n in LOOP.body if isinstance(n,ast.If) and
            any(isinstance(c,ast.Subscript) and isinstance(c.value,ast.Name) and
                c.value.id=='recovery_proof' for c in ast.walk(n)))
        before=np.arange(10,dtype=float)[None]
        normal_jaws=[-(.096-.044)/2,(.096-.044)/2]
        for forward,expected in [(False,[-.017,.026]),(True,normal_jaws)]:
            commands=before.copy()
            env=dict(a=SimpleNamespace(mode='recovery',resume_original_cycle_suffix=forward),
                qcommand=commands,jawids=[8,9],gap=.096,route_start_context=None,
                recovery_proof=dict(retained_jaw_position_targets_m=[-.017,.026]))
            exec(code([branch]),env)
            np.testing.assert_array_equal(commands[0,:8],before[0,:8])
            np.testing.assert_array_equal(commands[0,8:],expected)

    def test_original_endpoint_gap_is_44mm_and_96mm_is_not_a_completion(self):
        phases,planning=self.source_plan()
        branch=next(n for n in MAIN.body if isinstance(n,ast.If) and
            ast.unparse(n.test)=="a.mode == 'recovery'" and
            any(isinstance(c,ast.Call) and isinstance(c.func,ast.Name) and
                c.func.id=='cycle_opening_check' for c in ast.walk(n)))
        for actual_gap,expected_stop in [(.044,None),(.096,'original_cycle_suffix_endpoint_gap_not_reached')]:
            with tempfile.TemporaryDirectory() as temp:
                q=np.zeros((1,10));q[0,1:8]=phases[-1][2][-1]
                q[0,8:]=[-(actual_gap-.044)/2,(actual_gap-.044)/2]
                env=dict(a=SimpleNamespace(mode='recovery',resume_original_cycle_suffix=True,output=Path(temp)),
                    q=q,jawids=[8,9],rids=list(range(1,8)),planning=planning,phases=phases,
                    current_time=7.5,total=7.5,phase='hold',stop_reason=None,
                    recovery_proof=dict(source_elapsed_in_plan_s=2.,source_plan_sha256='a'*64),
                    cycle_opening_check=cycle_opening_check,np=np,__builtins__=__builtins__)
                import json
                env['json']=json
                exec(code([branch]),env)
                self.assertEqual(env['stop_reason'],expected_stop)
                self.assertEqual(env['recovery_feedback']['required_gap_m'],.044)
                self.assertFalse(env['recovery_feedback']['harvest_attempt'])
                self.assertFalse(env['recovery_feedback']['new_approach'])

    def test_cli_optin_requires_existing_strict_recovery_configuration(self):
        base=['--mode','recovery','--hz','60','--resume-native-checkpoint','/exact/route_checkpoint',
              '--save-route-checkpoint','--verify-full-environment-preflight',
              '--native-tool-collision-cache','/exact/native_tool_cooked_capture.json']
        self.assertFalse(actual_cli(base).resume_original_cycle_suffix)
        self.assertTrue(actual_cli(base+['--resume-original-cycle-suffix']).resume_original_cycle_suffix)
        for flags in [['--resume-original-cycle-suffix'],
                      base+['--resume-original-cycle-suffix','--whole-truss'],
                      base+['--resume-original-cycle-suffix','--seconds','1'],
                      base+['--resume-original-cycle-suffix','--hz','120'],
                      [v for v in base if v!='--verify-full-environment-preflight']+['--resume-original-cycle-suffix'],
                      base[:-2]+['--resume-original-cycle-suffix']]:
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(flags)

    def test_forward_has_no_general_two_second_resume_settle_overrides(self):
        branches=[n for n in ast.walk(LOOP) if isinstance(n,ast.If) and
            'resume_context is not None' in ast.unparse(n.test) and 't < 2' in ast.unparse(n.test)]
        self.assertEqual(len(branches),2)
        for forward in [False,True]:
            for branch in branches:
                desired=np.arange(10,dtype=float)[None];prior=desired[0]+10
                q0=desired+20;commands=desired.copy()
                env=dict(a=SimpleNamespace(resume_original_cycle_suffix=forward),
                    resume_context=dict(controller_state=dict(position_targets=prior)),t=0.,
                    qcommand=commands,q0=q0,names=['farmily_lift_height_joint']+list(range(9)))
                exec(code([branch]),env)
                if forward:np.testing.assert_array_equal(commands,desired)
                else:self.assertFalse(np.array_equal(commands,desired))

    def test_terminal_belt_controller_and_t0_record_restore_without_physics(self):
        branch=next(n for n in MAIN.body if isinstance(n,ast.If) and
            ast.unparse(n.test)=='a.resume_original_cycle_suffix' and
            any(isinstance(c,ast.Name) and c.id=='inherited_speed' for c in ast.walk(n)))
        class Attribute:
            def __init__(self):self.value=(0.,0.,0.)
            def Set(self,value):self.value=value
        attributes=[Attribute()for _ in range(4)]
        env=dict(a=SimpleNamespace(resume_original_cycle_suffix=True),
            recovery_proof=dict(source_terminal_belt_command_m_s=[-.05,.05]),
            belt_commands=[(0.,0.)],belt_attrs=attributes,last_speed=(0.,0.),
            Gf=SimpleNamespace(Vec3f=lambda *v:tuple(v)))
        exec(code([branch]),env)
        self.assertEqual(env['belt_commands'],[(-.05,.05)])
        self.assertEqual(env['last_speed'],(-.05,.05))
        self.assertEqual([attr.value for attr in attributes],[(0.,-.05,0),(0.,.05,0)]*2)

    def test_prepare_resume_delegates_only_explicit_forward_proof_and_preserves_source(self):
        from test_resume_native_state import ResumeNativeStateTests
        from resume_native_state import prepare_resume
        fixture=ResumeNativeStateTests();fixture.setUp()
        try:
            before=fixture.scene.read_bytes()
            proof=dict(schema='s300-terminal-forward-suffix-v1')
            with patch('terminal_motion_recovery.validate_terminal_forward_suffix',return_value=proof)as validator:
                ordinary=fixture.context()
                validator.assert_not_called()
                self.assertEqual(ordinary['checkpoint_kind'],'completed_cycle')
                context=prepare_resume(fixture.stage,fixture.assembly,fixture.manifest,fixture.root/'checkpoint',
                    allow_terminal_recovery=True,allow_original_cycle_suffix=True)
                validator.assert_called_once()
                self.assertIs(validator.call_args.kwargs['stage'],fixture.stage)
                self.assertEqual(context['checkpoint_kind'],'terminal_original_cycle_suffix')
                self.assertEqual(context['terminal_recovery_proof'],proof)
            self.assertEqual(before,fixture.scene.read_bytes())
            with self.assertRaisesRegex(ValueError,'explicit terminal recovery'):
                prepare_resume(fixture.stage,fixture.assembly,fixture.manifest,fixture.root/'checkpoint',
                    allow_original_cycle_suffix=True)
        finally:fixture.tearDown()
