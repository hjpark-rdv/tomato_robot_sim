import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from compact_motion_assets import file_sha256
from resume_axis import resolve_resume_axis


def axis(yaw):
    angle=np.radians(yaw)
    return dict(direction_world=[float(np.cos(angle)),float(np.sin(angle)),0.],yaw_world_deg=yaw)


class ResumeAxisTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.trial=Path(self.temp.name)
        self.checkpoint=self.trial/'cycle_checkpoints/000';self.checkpoint.mkdir(parents=True)
        self.context=dict(metadata=dict(metadata_sha256='a'*64,scene_sha256='b'*64,sequence=dict(cycle_plans=[])))

    def tearDown(self):self.temp.cleanup()

    def legacy(self,*,truss_axis=True,direction=True):
        plan={}
        if truss_axis:plan['truss_axis']=axis(168.9149092063868)
        if direction:plan['approach_direction_world']=axis(168.9149092063868)['direction_world']
        (self.trial/'plan.json').write_text(json.dumps(plan))
        self.audit=dict(passed=True,complete=True,scene_sha256='b'*64,input_sha256=file_sha256(self.trial/'plan.json'))
        self.save_audit()

    def save_audit(self):(self.trial/'self_collision_preflight.json').write_text(json.dumps(self.audit))

    def test_rejected_axis_cannot_replace_last_physically_used_axis(self):
        self.context['metadata']['sequence']['cycle_plans']=[
            dict(index=0,status='motion_completed',truss_axis=axis(168.9)),
            dict(index=1,status='planning_rejected',truss_axis=axis(143.))]
        result,proof=resolve_resume_axis(self.context,self.checkpoint)
        self.assertEqual(result['yaw_world_deg'],168.9);self.assertEqual(proof['cycle_index'],0)
        self.assertFalse(proof['failed_proposals_used'])

    def test_initial_axis_used_when_old_cycle_record_omitted_axis(self):
        self.context['metadata']['sequence'].update(initial_axis=axis(168.9),cycle_plans=[
            dict(index=0,status='motion_completed'),dict(index=1,status='stopped',truss_axis=axis(140.))])
        result,proof=resolve_resume_axis(self.context,self.checkpoint)
        self.assertEqual(result['yaw_world_deg'],168.9);self.assertEqual(proof['source'],'checkpoint_sequence_initial_axis')

    def test_audited_legacy_plan_fallback_binds_same_scene_and_exact_input(self):
        self.legacy();result,proof=resolve_resume_axis(self.context,self.checkpoint)
        self.assertAlmostEqual(result['yaw_world_deg'],168.9149092063868)
        self.assertEqual(proof['plan_sha256'],self.audit['input_sha256']);self.assertTrue(proof['audit_complete'])
        self.legacy(truss_axis=False);result,proof=resolve_resume_axis(self.context,self.checkpoint)
        self.assertAlmostEqual(result['yaw_world_deg'],168.9149092063868)

    def test_incomplete_failed_or_unbound_audit_is_rejected(self):
        for field,value in [('passed',False),('complete',False),('scene_sha256','c'*64),('input_sha256','d'*64)]:
            with self.subTest(field=field):
                self.legacy();self.audit[field]=value;self.save_audit()
                with self.assertRaises(ValueError):resolve_resume_axis(self.context,self.checkpoint)
        self.legacy();(self.trial/'plan.json').write_text(json.dumps(dict(truss_axis=axis(143.))))
        with self.assertRaisesRegex(ValueError,'input SHA'):resolve_resume_axis(self.context,self.checkpoint)

    def test_axis_must_be_finite_unit_horizontal_and_yaw_consistent(self):
        for direction in ([0.,0.,0.],[2.,0.,0.],[0.,0.,1.],[float('nan'),1.,0.]):
            self.context['metadata']['sequence']['initial_axis']=dict(direction_world=direction)
            with self.assertRaisesRegex(ValueError,'unit XY'):resolve_resume_axis(self.context,self.checkpoint)
        self.context['metadata']['sequence']['initial_axis']=dict(axis(168.),yaw_world_deg=143.)
        with self.assertRaisesRegex(ValueError,'disagree'):resolve_resume_axis(self.context,self.checkpoint)

    def test_legacy_axis_must_match_actual_plan_entry_direction(self):
        self.legacy();p=self.trial/'plan.json';d=json.loads(p.read_text());d['approach_direction_world']=axis(143.)['direction_world']
        p.write_text(json.dumps(d));self.audit['input_sha256']=file_sha256(p);self.save_audit()
        with self.assertRaisesRegex(ValueError,'actual entry'):resolve_resume_axis(self.context,self.checkpoint)

    def test_missing_axis_is_explicit_blocker(self):
        with self.assertRaisesRegex(ValueError,'missing'):resolve_resume_axis(self.context,self.checkpoint)
        self.legacy(truss_axis=False,direction=False)
        with self.assertRaisesRegex(ValueError,'unit XY'):resolve_resume_axis(self.context,self.checkpoint)


if __name__=='__main__':unittest.main()
