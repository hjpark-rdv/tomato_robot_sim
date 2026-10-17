"""CPU Warp controls for pair selection, released exceptions and residual expiry."""
import unittest
import os
from types import SimpleNamespace

import mujoco as mj
import numpy as np
import warp as wp
from mujoco_warp._src.types import vec5

from newton_contact_policy import (validate_config, identify_geometries, DEFAULT_PHASES,
    evaluate_pair_policy, classify_contacts, clear_pair_depth,ContactPolicyGPU,record_contact_witness)


def arr(value,dtype=float):
    return wp.array(np.asarray(value),dtype=dtype,device='cpu')


def fixture():
    model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
    <body name="Tool_L_belt"><geom name="beltL" type="box" size=".002 .05 .02"/></body>
    <body name="Tool_R_belt"><geom name="beltR" type="box" size=".002 .05 .02"/></body>
    <body name="Tool_L"><geom name="plate" type="box" size=".02 .05 .002"/></body>
    <body name="Truss_00_Fruit_01"><freejoint/><geom name="fruit" type="sphere" size=".012" mass=".01"/></body>
    <body name="Truss_00_Pedicel_01_0"><geom name="pedicel" type="capsule" size=".001 .01"/></body>
    <body name="Truss_01_Pedicel_01_0"><geom name="neighbor_pedicel" type="capsule" size=".001 .01"/></body>
    </worldbody></mujoco>''')
    metadata={'robot':{'belt_body_names':['Tool_L_belt','Tool_R_belt'],'geometry':[
        dict(name='beltL',owner='Tool_L_belt',collision=True,source='/World/Tool/L_belt/PrototypeTread'),
        dict(name='beltR',owner='Tool_R_belt',collision=True,source='/World/Tool/R_belt/PrototypeTread')]},
        'attachments':[dict(name='Truss_00_Fruit_01')]}
    return model,metadata


class ContactPolicyTests(unittest.TestCase):
    def test_held_belt_finish_is_explicit_opt_in_and_does_not_expand_other_phases(self):
        model,metadata=fixture()
        timeline=dict(phase=np.array(['close','cycle_reclose','belt_finish','approach','cycle_open','cycle_open_final']),
            time_edges=np.arange(7,dtype=float)*.1)
        solver=SimpleNamespace(mj_model=model)
        controller=SimpleNamespace(device='cpu',steps=6)
        default=ContactPolicyGPU(solver,controller,timeline,{},metadata)
        explicit=ContactPolicyGPU(solver,controller,timeline,dict(
            relaxed_phases=['close','cycle_reclose','belt_finish']),metadata)
        self.assertEqual(DEFAULT_PHASES,('close','cycle_reclose'))
        self.assertEqual(default.config['relaxed_phases'],['close','cycle_reclose'])
        np.testing.assert_array_equal(default.phase.numpy(),[1,1,0,0,2,2])
        np.testing.assert_array_equal(explicit.phase.numpy(),[1,1,1,0,2,2])
        for phase in ('approach','belt_start_open','cycle_open_hold','cycle_open_final_hold','rail_move'):
            with self.assertRaises(ValueError):validate_config({'relaxed_phases':['close',phase]})

    def test_explicit_open_stroke_band_is_scoped_and_default_still_rejects_recontact(self):
        model,metadata=fixture()
        timeline=dict(phase=np.array(['approach','cycle_open','cycle_open_final','cycle_open_final_hold']),
            time_edges=np.arange(5,dtype=float)*.1)
        solver=SimpleNamespace(mj_model=model)
        controller=SimpleNamespace(device='cpu',steps=4)
        default=ContactPolicyGPU(solver,controller,timeline,{},metadata)
        explicit=ContactPolicyGPU(solver,controller,timeline,dict(belt_pedicel_m=.002,
            relaxed_phases=['close','cycle_reclose','belt_finish','cycle_open','cycle_open_final']),metadata)
        np.testing.assert_array_equal(default.phase.numpy(),[0,2,2,0])
        np.testing.assert_array_equal(explicit.phase.numpy(),[0,1,1,0])
        # New pedicel recontact without any previously latched compression is
        # still blocked by the default policy. Only explicit stroke bands
        # allow it, and a fully-open hold remains strict.
        event,_,_=self.policy_sequence(default.phase.numpy(),[0.,.0017,.0017,0.],
            role=2,limit=.002)
        self.assertEqual(event,1)
        event,count,residual=self.policy_sequence(explicit.phase.numpy(),[0.,.0017,.0017,0.],
            role=2,limit=.002)
        self.assertIsNone(event)
        self.assertEqual(count[0,2],2)
        self.assertEqual(residual[0,2],0)
        event,_,_=self.policy_sequence(explicit.phase.numpy(),[0.,.0017,.0017,.0017],
            role=2,limit=.002)
        self.assertEqual(event,3)
        for phase in ('prepare_open','prepare_open_hold','cycle_open_hold','cycle_open_final_hold','rail_move','approach'):
            with self.assertRaises(ValueError):validate_config({'relaxed_phases':['close',phase]})

    def test_release_updates_equality_but_audit_attachment_cache_waits_until_next_control(self):
        from newton_kernels import release_attachments
        attached=arr([[True]],bool)
        equality=arr([[True]],bool)
        break_step=arr([[-1]],int)
        wp.launch(release_attachments,(1,1),inputs=[arr([0],int),0.,arr([0],int),arr([0],int),
            arr([0],int),attached,arr([[0.]]),arr([3.]),arr([.1]),arr([3],int),
            arr([[0,0,0]],int),arr([[0,0,0]],int),arr([[4.,0.,0.]]),equality,
            arr([[1.,1.,1.]]),break_step,wp.zeros((1,1,1),float,device='cpu'),
            wp.zeros((1,1,1),float,device='cpu')],device='cpu')
        self.assertFalse(bool(equality.numpy()[0,0]))
        self.assertTrue(bool(attached.numpy()[0,0]))
        self.assertEqual(int(break_step.numpy()[0,0]),0)

    def test_current_mujoco_gap_is_inactive_outside_margin_not_allowed_penetration(self):
        model=mj.MjModel.from_xml_string('''<mujoco><option integrator="discrete" gravity="0 0 0"/>
          <default><geom margin="0" gap=".0005"/></default><worldbody>
          <geom type="sphere" size=".01"/><body pos=".0202 0 0"><freejoint/>
          <geom type="sphere" size=".01" mass=".01"/></body></worldbody></mujoco>''')
        data=mj.MjData(model)
        data.qpos[0]=.0202;mj.mj_forward(model,data)
        self.assertEqual(data.ncon,1)
        self.assertGreater(data.contact[0].dist,0)
        self.assertEqual(data.contact[0].efc_address,-1)
        self.assertEqual(data.contact[0].includemargin,0)
        data.qpos[0]=.0198;mj.mj_forward(model,data)
        self.assertEqual(data.ncon,1)
        self.assertLess(data.contact[0].dist,0)
        self.assertGreaterEqual(data.contact[0].efc_address,0)

    def test_contact_witness_reads_real_solver_force_without_mutating_sources(self):
        device='cuda:0' if os.environ.get('S300_CONTACT_CUDA')=='1' else 'cpu'
        def a(value,dtype=float):return wp.array(np.asarray(value),dtype=dtype,device=device)
        model,metadata=fixture();n=model.ngeom
        solver=SimpleNamespace(mj_model=model,mjw_data=SimpleNamespace(naconmax=4))
        controller=SimpleNamespace(device=device,steps=2)
        policy=ContactPolicyGPU(solver,controller,dict(phase=np.array(['close','cycle_open']),
            time_edges=np.array([0.,.1,.2])),{'record_contacts':True},metadata)
        belt=model.geom('beltL').id;fruit=model.geom('fruit').id
        body_fruit=np.full(model.nbody,-1,np.int32)
        body_fruit[model.body('Truss_00_Fruit_01').id]=0
        positions=np.zeros((1,n,3),np.float32);positions[0,fruit]=[.003,.004,0]
        matrices=np.tile(np.eye(3),(1,n,1,1))
        matrices[0,belt]=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
        force=a([[2.,.5,-.3]])
        geometry=a(positions,wp.vec3)
        wp.launch(record_contact_witness,1,inputs=[a([0],int),a([1],int),3,1,
            a([0],int),a([[belt,fruit]],wp.vec2i),a([-.001]),
            a([np.eye(3)],wp.mat33),a([[0.,0.,0.]],wp.vec3),a([[.002,1.]],wp.vec2),
            a([[.5,.5,0.,0.,0.]],vec5),a([3],int),a([[0,1,2]],int),a([0.]),force,
            geometry,a(matrices,wp.mat33),a(model.geom_bodyid,int),
            a(body_fruit,int),a([[True]],bool),*policy.contact_witness],device=device)
        np.testing.assert_allclose(policy.contact_witness[4].numpy()[0,0],[2.,.5,-.3,0,0,0],atol=1e-7)
        np.testing.assert_allclose(policy.contact_witness[6].numpy()[0,0],[.004,-.003,0],atol=1e-8)
        np.testing.assert_array_equal(policy.contact_witness[8].numpy()[0,0],[-1,1])
        np.testing.assert_array_equal(force.numpy(),np.asarray([[2.,.5,-.3]],np.float32))
        np.testing.assert_array_equal(geometry.numpy(),positions)

    def test_five_mm_can_hide_complete_geometric_crossing_of_thin_pedicel(self):
        # A stop tolerance does not add belt compliance or prevent a complete
        # crossing. This deliberately kinematic negative control has no claim
        # of dynamic capture success.
        model=mj.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
          <worldbody><body name="belt" mocap="true"><geom name="belt" type="box" size=".0025 .05 .02"/></body>
          <body><freejoint/><geom type="capsule" size=".0007 .01" mass=".0001"/></body>
          </worldbody></mujoco>''')
        data=mj.MjData(model);depth=[];normals=[]
        for x in np.linspace(-.005,.005,21):
            data.mocap_pos[0,0]=x
            mj.mj_forward(model,data)
            depth.extend(max(0.,-contact.dist) for contact in data.contact)
            normals.extend(float(contact.frame[0]) for contact in data.contact)
        self.assertGreater(max(depth),.0014)
        self.assertLess(max(depth),.005)
        self.assertLess(min(normals),-.9)
        self.assertGreater(max(normals),.9)

    def test_exact_belt_owner_excludes_jaw_and_top_plate(self):
        model,metadata=fixture()
        lookup,roles,rows=identify_geometries(model,metadata)
        self.assertEqual([row['geom'] for row in rows],['beltL','beltR'])
        self.assertEqual(lookup[model.geom('plate').id],-1)
        self.assertEqual(roles[model.geom('fruit').id],1)
        self.assertEqual(roles[model.geom('pedicel').id],2)
        self.assertEqual(roles[model.geom('neighbor_pedicel').id],0)
        metadata['robot']['geometry'][0]['source']='/World/Tool/L/plate'
        with self.assertRaises(ValueError):identify_geometries(model,metadata)

    def test_config_cannot_silently_relax_all_contacts_or_approach(self):
        for config in ({'default_m':.005},{'relaxed_phases':['approach']},
                       {'belt_fruit_m':float('nan')},{'residual_open_seconds':-1},
                       {'other_m':.01},{'explicit_other_pairs':[{'geoms':['x','x'],'limit_m':.002}]},
                       {'explicit_other_pairs':[{'geoms':['x','y'],'limit_m':.002,'phases':['*','close']}]},
                       {'explicit_other_pairs':[{'geoms':['x','y'],'limit_m':.002},{'geoms':['y','x'],'limit_m':.003}]}):
            with self.assertRaises(ValueError):validate_config(config)

    def test_pair_classification_and_released_fruit_exception(self):
        model,metadata=fixture();lookup,roles,_=identify_geometries(model,metadata)
        # Fruit/belt allowed candidate; plate/fruit and belt/pedicel remain
        # independently classified. The second world has released fruit.
        g=lambda name:model.geom(name).id
        pairs=[[g('beltL'),g('fruit')],[g('plate'),g('fruit')],
               [g('beltR'),g('pedicel')],[g('beltL'),g('fruit')]]
        fruit_map=np.full(model.nbody,-1,np.int32)
        fruit_map[model.body('Truss_00_Fruit_01').id]=0
        depth=wp.zeros((2,2*model.ngeom),float,device='cpu')
        maxima=wp.zeros((2,1,4),float,device='cpu')
        first=arr(np.full(2,np.iinfo(np.uint64).max,np.uint64),wp.uint64)
        release=np.zeros((1,model.ngeom),bool);release[0,g('beltL')]=True
        wp.launch(classify_contacts,4,inputs=[arr([0],int),arr([4],int),arr([0,0,0,1],int),
            arr(pairs,wp.vec2i),arr([-.003,-.002,-.0015,-.009]),arr([[0],[0],[0],[0]],int),
            arr(model.geom_bodyid,int),arr(fruit_map,int),arr(release,bool),arr([[True],[False]],bool),
            arr(lookup,int),arr(roles,int),model.ngeom,arr(np.full(model.ngeom**2,-1),int),arr([[.001]]),depth,maxima,.001,first],device='cpu')
        np.testing.assert_allclose(maxima.numpy()[0,0],[.002,.003,.0015,0.],atol=1e-9)
        np.testing.assert_array_equal(maxima.numpy()[1],0.)
        self.assertNotEqual(int(first.numpy()[0]),np.iinfo(np.uint64).max)
        self.assertEqual(int(first.numpy()[1]),np.iinfo(np.uint64).max)

    def test_exact_nonbelt_pair_limit_is_phase_scoped_and_other_pairs_stay_strict(self):
        model,metadata=fixture();lookup,roles,_=identify_geometries(model,metadata)
        plate,fruit,pedicel=[model.geom(name).id for name in ('plate','fruit','pedicel')]
        fruit_map=np.full(model.nbody,-1,np.int32)
        fruit_map[model.body('Truss_00_Fruit_01').id]=0
        indexed=np.full(model.ngeom**2,-1,np.int32)
        indexed[min(plate,fruit)*model.ngeom+max(plate,fruit)]=0
        first=arr([np.iinfo(np.uint64).max],wp.uint64)
        maximum=wp.zeros((1,2,4),float,device='cpu')
        depth=wp.zeros((1,2*model.ngeom),float,device='cpu')
        for tick,pair in enumerate(([plate,fruit],[plate,pedicel])):
            wp.launch(classify_contacts,1,inputs=[arr([tick],int),arr([1],int),arr([0],int),
                arr([pair],wp.vec2i),arr([-.0015]),arr([[0]],int),arr(model.geom_bodyid,int),
                arr(fruit_map,int),arr(np.zeros((1,model.ngeom)),bool),arr([[True]],bool),
                arr(lookup,int),arr(roles,int),model.ngeom,arr(indexed,int),arr([[.002],[.001]]),
                depth,maximum,.001,first],device='cpu')
            if tick==0:self.assertEqual(int(first.numpy()[0]),np.iinfo(np.uint64).max)
        self.assertEqual(int(first.numpy()[0])>>32,1)
        self.assertAlmostEqual(float(maximum.numpy()[0,0,3]),.0015,places=8)

    def policy_sequence(self,phases,pens,times=None,residual=.3,role=1,limit=.005):
        # One belt and one fruit: a persistent pair, plus unused belt index.
        times=np.arange(len(phases))*.1 if times is None else np.asarray(times)
        depth=wp.zeros((1,2),float,device='cpu')
        latch=wp.zeros((1,2),bool,device='cpu');start=wp.full((1,2),-1.,float,device='cpu')
        count=wp.zeros((1,3),int,device='cpu');extra=wp.zeros_like(count)
        first=arr([np.iinfo(np.uint64).max],wp.uint64)
        phase=arr(phases,int)
        for t,pen in enumerate(pens):
            depth.assign(np.array([[0.,pen]],np.float32))
            wp.launch(evaluate_pair_policy,(1,2),inputs=[arr([t],int),phase,arr([times[t]]),
                arr([.001,limit,limit]),depth,latch,start,residual,.001,
                arr([0],int),arr([0,role],int),2,count,extra,first],device='cpu')
        event=int(first.numpy()[0])
        return None if event==np.iinfo(np.uint64).max else event>>32,count.numpy(),extra.numpy()

    def test_close_only_relaxation_keeps_opening_strict_without_hysteresis(self):
        event,counts,_=self.policy_sequence([1,2],[.004,.003],residual=0)
        self.assertEqual(event,1)
        self.assertEqual(counts[0,1],1)

    def test_residual_opening_is_bounded_by_actual_timestamp(self):
        event,_,extra=self.policy_sequence([1,2,2,2],[.004,.003,.002,.002],
                                          times=[0.,.1,.25,.46],residual=.3)
        self.assertEqual(event,3)
        self.assertEqual(extra[0,1],2)

    def test_separation_clears_latch_and_does_not_allow_new_opening_contact(self):
        event,_,extra=self.policy_sequence([1,2,2],[.004,0.,.003])
        self.assertEqual(event,2)
        self.assertEqual(extra[0,1],0)

    def test_unrelated_phase_and_excess_depth_are_not_forgiven(self):
        event,_,_=self.policy_sequence([1,0,2],[.004,.002,.003])
        self.assertEqual(event,1)
        event,_,_=self.policy_sequence([1],[.006])
        self.assertEqual(event,0)


if __name__=='__main__':unittest.main()
