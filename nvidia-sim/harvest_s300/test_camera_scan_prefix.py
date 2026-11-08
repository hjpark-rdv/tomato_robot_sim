import copy
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from camera_scan_prefix import (scan_prefix,append_scan_pan,direct_scan_prefix,camera_cycle_budget,
    retained_camera_arm_hold,verify_retained_camera_arm_hold)


class CameraScanPrefixTests(unittest.TestCase):
    def test_resumed_scan_retains_six_arm_targets_but_not_lift_bias_or_native_state(self):
        from test_native_route_checkpoint import NativeRouteCheckpointTests
        from native_route_checkpoint import load_checkpoint
        fixture=NativeRouteCheckpointTests();fixture.setUp()
        try:
            names=['root_joint','lift','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
            measured=np.array([.1,.03,1.57,-.86,2.51,-.05,3.14,.015,-.001,.002])
            targets=measured.copy();targets[1]=.036256;targets[2:8]+=.001
            fixture.save(robot_dof_names=names,robot_q=measured,robot_qvel=np.arange(10)/100,
                         robot_position_targets=targets,robot_velocity_targets=np.zeros(10))
            cp=fixture.root/'checkpoint';metadata,arrays=load_checkpoint(cp)
            kin=SimpleNamespace(names=names[1:8],bounds=np.array([[0.]+[-4.]*6,[.75]+[4.]*6]))
            context=dict(metadata=metadata,arrays=arrays,report=dict(checkpoint=str(cp)))
            start=measured[1:8].copy();before={p:p.read_bytes()for p in cp.iterdir()}
            command,proof=retained_camera_arm_hold(kin,start,context)
            self.assertEqual(command[0],.03);np.testing.assert_array_equal(command[1:],targets[2:8])
            phases,plan=direct_scan_prefix(kin,start,arm_hold_command_q=command,arm_hold_provenance=proof)
            np.testing.assert_array_equal(plan['start'],start)
            np.testing.assert_array_equal(plan['command_start_q'],command)
            for phase in phases:np.testing.assert_array_equal(phase[2],np.tile(command,(2,1)))
            self.assertTrue(proof['lift_drive_target_not_used_as_physical_height'])
            self.assertFalse(proof['physical_state_modified'])
            self.assertEqual(verify_retained_camera_arm_hold(kin,plan,cp),proof)
            wrong=copy.deepcopy(plan);wrong['phases'][0]['waypoint_q'][0][2]+=.001
            with self.assertRaisesRegex(ValueError,'exact native getter'):
                verify_retained_camera_arm_hold(kin,wrong,cp)
            self.assertEqual(before,{p:p.read_bytes()for p in before})
            np.testing.assert_array_equal(start,measured[1:8]);np.testing.assert_array_equal(arrays['robot_q'],measured)
            with self.assertRaisesRegex(ValueError,'lift bias is not a height'):
                direct_scan_prefix(kin,start,arm_hold_command_q=targets[1:8],arm_hold_provenance=proof)
            with self.assertRaisesRegex(ValueError,'exact measured'):
                retained_camera_arm_hold(kin,command,context)
            changed=copy.deepcopy(context);changed['metadata']['time_s']+=1
            with self.assertRaisesRegex(ValueError,'metadata mismatch'):
                retained_camera_arm_hold(kin,start,changed)
        finally:fixture.tearDown()

    def test_no_resume_keeps_exact_existing_scan_commands_and_camera_only_dispatch(self):
        start=np.array([.03,1.57,-.86,2.51,-.05,3.14,.015])
        command,proof=retained_camera_arm_hold(None,start,None)
        np.testing.assert_array_equal(command,start);self.assertIsNone(proof)
        old,metadata=direct_scan_prefix(object(),start)
        actual,meta=direct_scan_prefix(object(),command)
        self.assertEqual(meta,metadata)
        for a,b in zip(old,actual):np.testing.assert_array_equal(a[2],b[2])
        # Execute the actual production startup expressions only; no SDK import.
        source=Path(__file__).with_name('run_motion.py').read_text();tree=ast.parse(source)
        gate=next(n for n in ast.walk(tree)if isinstance(n,ast.If)and
            any(isinstance(child,ast.Call)and isinstance(child.func,ast.Name)and child.func.id=='direct_scan_prefix'
                for statement in n.body for child in ast.walk(statement)) and
            isinstance(n.test,ast.BoolOp) and 'a.camera_harvest' in ast.unparse(n.test))
        self.assertIsInstance(gate.body[1],ast.Assign)
        self.assertEqual(ast.unparse(gate.body[1].value.func),'retained_camera_arm_hold')
        self.assertEqual(ast.unparse(gate.body[2].value.func),'direct_scan_prefix')
        self.assertIn('initial_robot_q',ast.unparse(gate.body[2].value.args))
        self.assertIn('camera_hold_q',ast.unparse(gate.body[2]))
        self.assertIn('camera_hold_q',ast.unparse(gate.body[3]))

    def test_direct_scan_matches_existing_retained_joint_prefix_and_full_cycle_budget(self):
        from motion_kinematics import plan_motion
        from test_motion_ready import CartesianRobot
        kin=CartesianRobot();start=np.array([.05,0.,0.,0.,0.,0.,0.])
        for close,belt,ramp in [(6.,2.,.5),(3.,3.,1.2),(8.,4.,0.)]:
            full,metadata=plan_motion(kin,[.4,1.,1.2],start,'horizontal_ready',
                harvest_profile='belt_open_close_cycle',horizontal_ready_connector='joint',
                horizontal_ready_seed_index=8,allow_measured_ready_start=True,
                close_seconds=close,belt_seconds=belt,belt_ramp_seconds=ramp)
            legacy,legacy_meta=scan_prefix(full,metadata)
            direct,direct_meta=direct_scan_prefix(kin,start)
            self.assertEqual(len(direct),len(legacy))
            for a,b in zip(direct,legacy):
                self.assertEqual(a[:2],b[:2]);self.assertEqual(a[3:],b[3:])
                np.testing.assert_array_equal(a[2],b[2])
            self.assertEqual(direct_meta['phases'],legacy_meta['phases'])
            budget,evidence=camera_cycle_budget(close_seconds=close,belt_seconds=belt,belt_ramp_seconds=ramp)
            self.assertEqual(budget,sum(row[1] for row in full))
            self.assertTrue(evidence['budget_only'])
        self.assertEqual(camera_cycle_budget()[0],42.)
        self.assertFalse(direct_meta['camera_scan']['harvest_staging_ik_evaluated'])
        self.assertNotIn('goal_pose',direct_meta)

    def test_direct_prefix_never_calls_harvest_ik_and_preserves_existing_rail_and_pan(self):
        class CameraOnly:
            names=['lift','base','shoulder','elbow','wrist1','wrist2','wrist3']
            bounds=np.array([[-1.]+[-np.pi]*6,[1.]+[np.pi]*6])
            def ik(self,*args):raise AssertionError('Unused harvest staging/entry IK called')
            def frames(self,q):
                c,s=np.cos(q[1]),np.sin(q[1]);t=np.eye(4)
                t[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];t[:3,3]=[0.,1.,1.+q[0]]
                return {'/World/HarvestRobot/Robot/link1':t}
        from rail_motion import attach_rail_commands
        kin=CameraOnly();start=np.zeros(7);before=start.copy()
        phases,metadata=direct_scan_prefix(kin,start)
        phases,metadata=attach_rail_commands(phases,metadata,start,
            dict(duration_s=2.,displacement_m=.3),move_first=True)
        mount=np.eye(4);mount[:3,:3]=[[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]
        phases,metadata=append_scan_pan(phases,metadata,kin,[-.4,1.6,1.],mount)
        self.assertEqual([p[0] for p in phases],
            ['rail_move','ready_hold','prepare_open','prepare_open_hold','camera_pan','camera_pan_hold'])
        self.assertTrue(all(p[-2:]==(0.,0.) for p in phases))
        np.testing.assert_array_equal(start,before)
        for row in metadata['phases'][-2:]:self.assertTrue(all(x==.3 for x in row['rail_waypoint_positions_m']))
        self.assertIn('camera_pan',metadata['preparation_opening']['feedback_before_phases'])

    def test_direct_ready_lift_reuses_existing_builder_with_unchanged_start_and_gap(self):
        start=np.zeros(7);branch=np.array([start,start+[.02,0.,0.,0.,0.,0.,0.]])
        with patch('motion_kinematics.ready_lift_branch',return_value=(branch,dict(source='fixture'))) as builder:
            phases,metadata=direct_scan_prefix(object(),start,ready_lift_offset_m=.02)
        builder.assert_called_once()
        np.testing.assert_array_equal(builder.call_args.args[1],start)
        self.assertEqual(phases[-1][0],'prepare_lift_branch')
        np.testing.assert_array_equal(phases[-1][2],branch)
        self.assertEqual(phases[-1][3:],(.096,0.,0.))
        self.assertEqual(camera_cycle_budget(ready_lift_offset_m=.02)[0],50.)
        self.assertEqual(camera_cycle_budget(entry_duration_scale=2.,rail_duration_s=3.)[0],53.)

    def test_absolute_scan_height_moves_real_lift_after_stationary_open_before_pan(self):
        class CameraOnly:
            names=['lift','base','shoulder','elbow','wrist1','wrist2','wrist3']
            bounds=np.array([[0.]+[-np.pi]*6,[.75]+[np.pi]*6])
            def ik(self,*args):raise AssertionError('Unused harvest IK called')
            def frames(self,q):
                c,s=np.cos(q[1]),np.sin(q[1]);t=np.eye(4)
                t[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];t[:3,3]=[0.,1.,1.+q[0]]
                return {'/World/HarvestRobot/Robot/link1':t}
        from rail_motion import attach_rail_commands
        kin=CameraOnly();start=np.array([.177,0.,.1,-.2,.3,-.4,.5]);before=start.copy()
        phases,metadata=direct_scan_prefix(kin,start,scan_lift_height_m=.36)
        self.assertEqual([p[0] for p in phases],
            ['ready_hold','prepare_open','prepare_open_hold','camera_lift_vantage'])
        transfer=phases[-1]
        self.assertEqual(transfer[1],6.)
        np.testing.assert_array_equal(transfer[2][0],start)
        np.testing.assert_array_equal(transfer[2][:,1:],np.tile(start[1:],(2,1)))
        self.assertEqual(transfer[2][-1,0],.36)
        self.assertEqual(transfer[3:],(.096,0.,0.))
        self.assertFalse(metadata['camera_scan']['physical_lift']['physical_initial_state_changed'])
        phases,metadata=attach_rail_commands(phases,metadata,start,
            dict(duration_s=2.,displacement_m=.15),move_first=True)
        mount=np.eye(4);mount[:3,:3]=[[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]
        phases,metadata=append_scan_pan(phases,metadata,kin,[-.4,1.6,1.],mount)
        for phase,row in zip(phases[-3:],metadata['phases'][-3:]):
            self.assertEqual(row['rail_waypoint_positions_m'],[.15]*len(phase[2]))
            self.assertAlmostEqual(phase[2][-1,0],.36)
        np.testing.assert_array_equal(start,before)
        self.assertIn('camera_lift_vantage',metadata['preparation_opening']['feedback_before_phases'])

    def test_absolute_height_is_not_ik_redistribution_or_silent_limit_clamp(self):
        from types import SimpleNamespace
        kin=SimpleNamespace(bounds=np.array([[0.]+[-4.]*6,[.75]+[4.]*6]))
        start=np.array([.2,0.,0.,0.,0.,0.,0.])
        with self.assertRaisesRegex(ValueError,'distinct modes'):
            direct_scan_prefix(kin,start,scan_lift_height_m=.35,ready_lift_offset_m=.1)
        for height in (-.01,.751,float('nan'),float('inf'),True):
            with self.subTest(height=height),self.assertRaises(ValueError):
                direct_scan_prefix(kin,start,scan_lift_height_m=height)
        legacy,old=direct_scan_prefix(kin,start)
        disabled,new=direct_scan_prefix(kin,start,scan_lift_height_m=None)
        self.assertEqual(old,new)
        for a,b in zip(legacy,disabled):
            self.assertEqual(a[:2],b[:2]);self.assertEqual(a[3:],b[3:])
            np.testing.assert_array_equal(a[2],b[2])

    def test_direct_scan_rejects_unsupported_or_nonfinite_configuration(self):
        for kwargs in [dict(preparation='direct'),dict(horizontal_ready_connector='cartesian'),
                dict(preparation_opening='legacy-staging'),dict(ready_lift_offset_m=float('nan')),
                dict(preparation_gap_m=.1)]:
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                direct_scan_prefix(object(),np.zeros(7),**kwargs)
        for kwargs in [dict(close_seconds=0),dict(belt_ramp_seconds=3.),
                dict(rail_duration_s=-1.),dict(ready_lift_offset_m=float('inf'))]:
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):camera_cycle_budget(**kwargs)

    def test_every_opted_in_camera_plan_is_environment_checked_before_acceptance(self):
        # Regression: the old full-environment option checked only the scan
        # prefix, allowing later observation-generated paths to bypass it.
        tree=ast.parse(Path(__file__).with_name('run_motion.py').read_text())
        tries=[]
        for node in ast.walk(tree):
            if not isinstance(node,ast.Try):continue
            checks=[(i,item) for i,item in enumerate(node.body)
                    if isinstance(item,ast.If) and ast.unparse(item.test)==
                       "(a.camera_harvest or getattr(a, 'group_heading_search', False)) and a.verify_full_environment_preflight"]
            if checks:tries.append((node,checks[0]))
        self.assertEqual(len(tries),1)
        node,(index,gate)=tries[0]
        before=ast.unparse(ast.Module(body=node.body[:index],type_ignores=[]))
        after=ast.unparse(ast.Module(body=node.body[index+1:],type_ignores=[]))
        self.assertIn('audit_external(',before)
        self.assertIn('phases, planning = (new_phases, new_planning)',after)
        calls=[n for n in ast.walk(gate) if isinstance(n,ast.Call) and
               isinstance(n.func,ast.Name) and n.func.id=='audit_external_rail']
        self.assertEqual(len(calls),1)
        keywords={value.arg:ast.unparse(value.value) for value in calls[0].keywords}
        self.assertEqual(keywords['checkpoint'],'boundary_checkpoint')
        self.assertEqual(keywords['scope'],"'full'")
        self.assertEqual(keywords['hz'],'a.hz');self.assertEqual(keywords['native_hz'],'a.hz')
        self.assertEqual(keywords['allowed_truss_key'],"group['truss_key']")
        self.assertEqual(keywords['stop_on_first_hard_hit'],'True')

    def fixture(self):
        names=['rail_travel','prepare_lift','prepare_orient','prepare_open_hold',
               'approach','close','belt_finish','cycle_open','retract','return_ready']
        phases=[(name,2.+i,np.array([[float(i),0.,0.],[float(i)+1,0.,0.]]),
                 .096 if i<5 else .028,0. if i<5 else .05,0. if i<5 else .05)
                for i,name in enumerate(names)]
        metadata=dict(phases=[dict(name=name,seconds=2.+i,opaque=['keep',i]) for i,name in enumerate(names)],
                      source_sha256='a'*64,initial_command=[0.]*10)
        return phases,metadata

    def test_preparation_scan_excludes_all_entry_and_belt_harvest_phases(self):
        phases,metadata=self.fixture();selected,result=scan_prefix(phases,metadata)
        self.assertEqual([phase[0] for phase in selected],
                         ['rail_travel','prepare_lift','prepare_orient','prepare_open_hold'])
        self.assertEqual(result['phases'],metadata['phases'][:4])
        for phase in selected:
            self.assertEqual(phase[-2:],(0.,0.))
            self.assertNotIn(phase[0],('approach','close','belt_finish','cycle_open','retract'))
        self.assertFalse(result['camera_scan']['harvest_entry_executed'])
        self.assertFalse(result['camera_scan']['harvest_region_selected'])
        self.assertEqual(result['camera_scan']['removed_phase_names'],[p[0] for p in phases[4:]])

    def test_truncation_does_not_mutate_input_commands_or_metadata(self):
        phases,metadata=self.fixture();old_phases=copy.deepcopy(phases);old_metadata=copy.deepcopy(metadata)
        _,result=scan_prefix(phases,metadata)
        self.assertEqual(metadata,old_metadata)
        for old,new in zip(old_phases,phases):
            self.assertEqual(old[:2],new[:2]);np.testing.assert_array_equal(old[2],new[2])
            self.assertEqual(old[3:],new[3:])
        result['phases'][0]['opaque'][0]='changed derived metadata'
        self.assertEqual(metadata,old_metadata)

    def test_missing_duplicate_or_prefixless_approach_is_rejected(self):
        phases,metadata=self.fixture()
        for bad in (phases[:4],phases+[phases[4]],phases[4:]):
            with self.subTest(names=[p[0] for p in bad]):
                with self.assertRaisesRegex(ValueError,'One existing approach'):
                    scan_prefix(bad,metadata)


class ScanPanTests(unittest.TestCase):
    class Kinematics:
        names=['lift','base','shoulder']
        bounds=np.array([[0.,-np.pi,-np.pi],[.8,np.pi,np.pi]])
        def frames(self,q):
            c,s=np.cos(q[1]),np.sin(q[1]);matrix=np.eye(4)
            matrix[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]]
            matrix[:3,3]=[0.,1.074,1.+q[0]]
            return {'/World/HarvestRobot/Robot/link1':matrix}

    def fixture(self):
        q=np.array([.2,0.,.7])
        phases=[('prepare_open_hold',.5,np.array([q,q]),.096,0.,0.)]
        metadata=dict(phases=[dict(name='prepare_open_hold',duration_s=.5,waypoint_q=[q.tolist(),q.tolist()],
            end_gap_m=.096,belt_local_y_m_s=[0.,0.],rail_waypoint_positions_m=[-.208,-.208])],
            camera_scan=dict(harvest_region_selected=False,harvest_entry_executed=False))
        mount=np.eye(4);mount[:3,:3]=[[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]
        mount[:3,3]=[0.,.098,.0325]
        return phases,metadata,mount

    def test_physical_base_joint_pan_aligns_fixed_camera_and_preserves_other_joints(self):
        phases,metadata,mount=self.fixture();kin=self.Kinematics()
        selected,result=append_scan_pan(phases,metadata,kin,[-.5,1.55,1.3],mount)
        self.assertEqual([phase[0] for phase in selected],['prepare_open_hold','camera_pan','camera_pan_hold'])
        for phase,row in zip(selected[-2:],result['phases'][-2:]):
            np.testing.assert_array_equal(phase[2][:,[0,2]],np.tile([.2,.7],(len(phase[2]),1)))
            self.assertEqual(phase[3:],(.096,0.,0.))
            self.assertEqual(row['rail_waypoint_positions_m'],[-.208]*len(phase[2]))
        evidence=result['camera_scan']['physical_pan']
        self.assertLess(abs(evidence['horizontal_heading_error_after_rad']),1e-5)
        self.assertGreater(abs(evidence['commanded_yaw_change_rad']),.5)
        self.assertFalse(evidence['camera_transform_edited']);self.assertFalse(evidence['body_state_teleported'])
        self.assertFalse(evidence['collision_certified']);self.assertFalse(evidence['harvest_region_selected'])
        camera=np.array(evidence['camera_world_after'])
        toward=np.array([-.5,1.55])-camera[:2,3]
        np.testing.assert_allclose(camera[:2,2]/np.linalg.norm(camera[:2,2]),toward/np.linalg.norm(toward),atol=1e-6)

    def test_pan_does_not_mutate_input_phases_metadata_mount_or_kinematics(self):
        phases,metadata,mount=self.fixture()
        saved_phases=copy.deepcopy(phases);saved_metadata=copy.deepcopy(metadata);saved_mount=mount.copy()
        append_scan_pan(phases,metadata,self.Kinematics(),[-.5,1.55,1.3],mount)
        self.assertEqual(metadata,saved_metadata);np.testing.assert_array_equal(mount,saved_mount)
        self.assertEqual(phases[0][:2],saved_phases[0][:2]);np.testing.assert_array_equal(phases[0][2],saved_phases[0][2])
        self.assertEqual(phases[0][3:],saved_phases[0][3:])

    def test_row_front_keeps_heading_normal_across_longitudinal_targets_and_sides(self):
        phases,metadata,mount=self.fixture();kin=self.Kinematics()
        for side in (-1.,1.):
            for longitudinal in (-2.,1.,4.):
                _,result=append_scan_pan(phases,metadata,kin,[side,longitudinal,1.3],mount,view='row-front')
                evidence=result['camera_scan']['physical_pan'];camera=np.array(evidence['camera_world_after'])
                np.testing.assert_allclose(camera[:3,2],[side,0.,0.],atol=1e-6)
                np.testing.assert_allclose(kin.frames(evidence['goal_q'])['/World/HarvestRobot/Robot/link1']@mount,camera)
                self.assertAlmostEqual(evidence['longitudinal_target_offset_m'],longitudinal-camera[1,3])
                self.assertEqual(evidence['view'],'row-front')
                self.assertFalse(evidence['camera_transform_edited'])
        _,rotated=append_scan_pan(phases,metadata,kin,[1.,2.,1.3],mount,
                                 view='row-front',rail_axis_world=[1.,0.,0.])
        np.testing.assert_allclose(np.array(rotated['camera_scan']['physical_pan']['camera_world_after'])[:3,2],
                                   [0.,1.,0.],atol=1e-6)

    def test_row_front_rejects_unknown_view_or_invalid_rail(self):
        phases,metadata,mount=self.fixture()
        for kwargs in [dict(view='bogus'),dict(view='row-front',rail_axis_world=[0.,0.,0.]),
                       dict(view='row-front',rail_axis_world=[0.,1.,.1])]:
            with self.assertRaises(ValueError):append_scan_pan(phases,metadata,self.Kinematics(),[-.5,1.55,1.3],mount,**kwargs)

    def test_yaw_limits_and_invalid_inputs_fail_without_fake_alignment(self):
        phases,metadata,mount=self.fixture();kin=self.Kinematics()
        kin.bounds=kin.bounds.copy();kin.bounds[:,1]=[-.1,.1]
        with self.assertRaisesRegex(RuntimeError,'bounded camera-pan'):
            append_scan_pan(phases,metadata,kin,[-.5,1.55,1.3],mount)
        for target in ([np.nan,0.,0.],[0.,0.]):
            with self.assertRaisesRegex(ValueError,'Finite map-based'):
                append_scan_pan(phases,metadata,self.Kinematics(),target,mount)
        closed=copy.deepcopy(phases);closed[0]=tuple(list(closed[0][:3])+[.044,0.,0.])
        with self.assertRaisesRegex(ValueError,'open96mm'):
            append_scan_pan(closed,metadata,self.Kinematics(),[-.5,1.55,1.3],mount)


if __name__=='__main__':unittest.main()
