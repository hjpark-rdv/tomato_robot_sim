"""CPU fixtures: strict legacy parent plus explicitly bound window transition."""
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import mujoco as mj
import numpy as np
from newton_active_window import prepare
from newton_continuation import model_identity,file_sha256,VerificationContext
from newton_window_continuation import checked_window_parent,create_transition,load_transition
from test_newton_active_window import fixture


def write(p,v):p.write_text(json.dumps(v,indent=2)+'\n')


class WindowContinuationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.common=self.root/'full';self.full,self.initial=fixture(self.common)
        self.pi=self.root/'pi';prepare(self.common,self.pi,'22')
        p=json.loads((self.pi/'inputs.json').read_text());p['same_bed_sequence']=dict(index=0,source_common=str(self.common))
        p['source_body_to_native_name']={f'/World/{self.full.body(i).name}':self.full.body(i).name for i in range(1,self.full.nbody)}
        write(self.pi/'inputs.json',p);self.meta=json.loads((self.common/'source_export.json').read_text())
        from newton_contact_policy import validate_config
        self.policy=validate_config(json.loads((Path(__file__).parent/'profiles/belt_strokes_2mm.json').read_text()))
        self.parent=self.root/'parent';self.parent.mkdir();s=dict(np.load(self.pi/'initial_state.npz'))
        self.final={k:s[k][None].copy() for k in ('qpos','qvel','eq_active','ctrl','runtime_dof_damping')}
        self.final.update(time=np.array([1.]),body_names=s['body_names'],attachment_equality_ids=np.arange(3),qacc_warmstart=np.zeros((1,len(s['qvel']))))
        np.savez_compressed(self.parent/'final_states.npz',**self.final)
        m=mj.MjModel.from_binary_path(str(self.pi/'input_model.mjb'));d=mj.MjData(m);d.qpos[:]=self.final['qpos'][0];mj.mj_kinematics(m,d)
        pose=np.column_stack((d.xpos[1:],d.xquat[1:,[1,2,3,0]]))
        self.recorded=dict(time=np.array([1.]),qpos=self.final['qpos'],qvel=self.final['qvel'],eq_active=self.final['eq_active'],poses=pose[None],body_paths=np.array(list(p['source_body_to_native_name'])))
        np.savez_compressed(self.parent/'states.npz',**self.recorded)
        np.savez_compressed(self.parent/'executed_commands.npz',time=np.arange(17)/17);write(self.parent/'cumulative_break_events.json',[])
        self.summary=dict(status='completed',stop=None,backend='gpu',worlds=1,finite_recorded_state=True,qacc_finite=True,solver_capacity_valid=True,
            device_command_buffers_match_schedule=True,controller_tick_matches_recorded_steps=True,actual_steps=17,requested_steps=17,input=str(self.pi),input_sha256=file_sha256(self.pi/'inputs.json'),
            contact_policy=dict(config=self.policy,policy_passed=True),continuation_model_identity=model_identity(s,json.loads((self.pi/'source_export.json').read_text())),
            final_physics_clock_s=1.,effective_gpu_options=dict(tolerance=[1e-8]))
        self.save()
    def save(self):
        for f,k in [('final_states.npz','final_states_sha256'),('states.npz','states_sha256'),('executed_commands.npz','executed_commands_sha256'),('cumulative_break_events.json','cumulative_break_events_sha256')]:self.summary[k]=file_sha256(self.parent/f)
        write(self.parent/'summary.json',self.summary)
    def child(self):
        bank,*_=checked_window_parent(self.parent,self.common,self.initial,self.meta,self.policy,0)
        src=self.root/'child_full';shutil.copytree(self.common,src)
        (src/'input_model.mjb').unlink();(src/'input_model.mjb').symlink_to(self.common/'input_model.mjb')
        np.savez_compressed(src/'initial_state.npz',**bank)
        p=json.loads((src/'inputs.json').read_text());p['input_files']['initial_state.npz']['sha256']=file_sha256(src/'initial_state.npz');p['same_bed_sequence']=dict(index=1,source_common=str(self.common));write(src/'inputs.json',p)
        dst=self.root/'child';prepare(src,dst,'27',bank_state=bank);return dst
    def load(self,cert,inputs):
        return load_transition(cert,inputs,dict(np.load(inputs/'initial_state.npz')),json.loads((inputs/'source_export.json').read_text()),self.policy)
    def test_exact_canonical_transition_keeps_legacy_same_sha_check(self):
        dst=self.child();cert=create_transition(dst,self.parent,self.common);restored,state,history,ev=self.load(cert,dst)
        s=dict(np.load(dst/'initial_state.npz'))
        for k in s:np.testing.assert_array_equal(restored[k],s[k])
        self.assertEqual(state['time'][0],1.);self.assertEqual(history,[]);self.assertTrue(ev['legacy_identical_mjb_resume_unchanged'])
        self.assertEqual(state['qpos'].dtype,np.dtype('float32'))
        self.assertEqual(state['runtime_dof_damping'].dtype,np.dtype('float32'))
        self.assertGreater(ev['backend_state_encoding']['maximum_abs_encoding_error_by_field']['qpos'],0)
        self.assertNotEqual(ev['physical_model_mjb_sha256'],ev['parent_physical_model_mjb_sha256'])
        self.assertEqual(ev['parent_effective_gpu_options'],self.summary['effective_gpu_options'])
    def test_call_scope_verifies_originals_once_and_reuses_only_same_read_only_models(self):
        dst=self.child();cert=create_transition(dst,self.parent,self.common)
        context=VerificationContext();initial=dict(np.load(dst/'initial_state.npz'))
        _,_,_,proof=load_transition(cert,dst,initial,json.loads((dst/'source_export.json').read_text()),self.policy,verification_context=context)
        evidence=proof['verification_scope']
        self.assertEqual(evidence['unique_file_sha_reads'],len(evidence['sources']))
        self.assertEqual(evidence['unique_read_only_model_loads'],3)
        self.assertGreaterEqual(evidence['stable_read_only_model_reuses'],2)
        self.assertGreater(evidence['stable_sha_reuses'],10)
        self.assertTrue(evidence['original_file_sha_checks_preserved'])
        self.assertFalse(evidence['global_or_cross_call_cache'])

    def test_changed_file_identity_is_rejected_even_if_bytes_and_mtime_are_restored(self):
        path=self.root/'identity.bin';path.write_bytes(b'original')
        context=VerificationContext();first=context.file_sha256(path)
        old=path.stat();replacement=self.root/'identity_replacement.bin';replacement.write_bytes(b'original')
        import os
        os.utime(replacement,ns=(old.st_atime_ns,old.st_mtime_ns));replacement.replace(path)
        with self.assertRaisesRegex(ValueError,'changed within scope'):context.file_sha256(path)
        # A new outer call independently rehashes the original bytes.
        self.assertEqual(VerificationContext().file_sha256(path),first)

    def test_same_digest_other_file_cannot_share_a_cached_mutable_model(self):
        context=VerificationContext();one=context.model(self.common/'input_model.mjb')
        self.assertIs(one,context.model(self.common/'input_model.mjb'))
        other=self.root/'other_model.mjb';shutil.copy2(self.common/'input_model.mjb',other)
        two=context.model(other)
        self.assertIsNot(one,two)
        evidence=context.evidence()
        self.assertEqual(evidence['unique_read_only_model_loads'],2)
        self.assertEqual(evidence['unique_file_sha_reads'],2)

    def test_existing_local_model_cache_is_independently_verified_after_original_hash(self):
        source=self.common/'input_model.mjb';cache=self.root/'local_cache';cache.mkdir()
        digest=file_sha256(source);cached=cache/(digest+'.mjb');shutil.copy2(source,cached)
        context=VerificationContext(local_model_cache=cache);model=context.model(source)
        self.assertEqual(model.nv,self.full.nv)
        evidence=context.evidence()
        self.assertEqual(evidence['unique_file_sha_reads'],2)
        self.assertEqual(evidence['verified_local_model_loads'],1)
        paths={row['path'] for row in evidence['sources']}
        self.assertIn(str(source),paths);self.assertIn(str(cached),paths)
        cached.write_bytes(b'corrupt cache')
        with self.assertRaisesRegex(ValueError,'cache SHA mismatch'):
            VerificationContext(local_model_cache=cache).model(source)
        # No cache files are created/replaced, and the original stays valid.
        self.assertEqual(file_sha256(source),digest)
        self.assertEqual(cached.read_bytes(),b'corrupt cache')

    def test_wrong_route_index_and_incomplete_parent_refused(self):
        with self.assertRaisesRegex(ValueError,'preceding'):checked_window_parent(self.parent,self.common,self.initial,self.meta,self.policy,1)
        self.summary['status']='stopped';self.save()
        with self.assertRaisesRegex(ValueError,'fully completed'):checked_window_parent(self.parent,self.common,self.initial,self.meta,self.policy,0)
    def test_actual_poses_must_match_joint_snapshot(self):
        self.recorded['poses'][0,0,0]+=.001;np.savez_compressed(self.parent/'states.npz',**self.recorded);self.save()
        with self.assertRaisesRegex(ValueError,'actual poses'):checked_window_parent(self.parent,self.common,self.initial,self.meta,self.policy,0)
    def test_certificate_and_bound_bank_tampering_refused(self):
        dst=self.child();cert=create_transition(dst,self.parent,self.common);c=json.loads(cert.read_text());c['initial_clock_s']=2.;write(cert,c)
        with self.assertRaisesRegex(ValueError,'digest'):self.load(cert,dst)
        cert.unlink();cert=create_transition(dst,self.parent,self.common)
        with (dst/'canonical_full_state.npz').open('ab') as f:f.write(b'changed')
        with self.assertRaisesRegex(ValueError,'SHA'):self.load(cert,dst)


if __name__=='__main__':unittest.main()
