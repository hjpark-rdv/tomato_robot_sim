"""Material experiments must preserve geometry, commands and all other fields."""
import json
import os
from pathlib import Path
import tempfile
import unittest

import mujoco as mj
import numpy as np
import warp as wp
from mujoco_warp._src.constraint import _contact_kbimp,_efc_row
from mujoco_warp._src import constraint as constraint_module
from mujoco_warp._src.types import vec5,ConstraintType

from newton_contact_variant import direct_reference,prepare
from newton_inputs import sha256
from test_newton_contact_policy import fixture


@wp.kernel
def contact_coefficient_control(refs:wp.array[wp.vec2],discrete:wp.array[bool],out:wp.array[wp.vec3]):
    i=wp.tid()
    out[i]=_contact_kbimp(0,.0025,refs[i],vec5(.99,.999,.001,.5,2.),-.0005,discrete[i])


@wp.kernel
def contact_row_control(refs:wp.array[wp.vec2],timesteps:wp.array[float],depths:wp.array[float],
        raw:wp.array[wp.vec3],types:wp.array2d[int],ids:wp.array2d[int],positions:wp.array2d[float],
        margins:wp.array2d[float],diagonal:wp.array2d[float],velocities:wp.array2d[float],
        acceleration:wp.array2d[float],frictionloss:wp.array2d[float]):
    i=wp.tid()
    imp=vec5(.99,.999,.001,.5,2.)
    raw[i]=_contact_kbimp(0,timesteps[i],refs[i],imp,-depths[i],True)
    for j in range(2):
        _efc_row(0,0,timesteps[i],i*2+j,-.0005,-depths[i],1.,refs[i],imp,
            0.,float(j),0.,int(ConstraintType.CONTACT_ELLIPTIC),i,True,0.,
            types,ids,positions,margins,diagonal,velocities,acceleration,frictionloss)


class ContactVariantTests(unittest.TestCase):
    def test_positive_belt_variants_keep_discrete_traction_with_source_fruit_material(self):
        settings=[('.002 1',[.002,1.]),('.002 1.414213562373095',[.002,(1+1/np.sqrt(.5))/2]),
            ('.002 2',[.002,1.5]),('.004 .5',[.003,.75]),('.001 2',[.0015,1.5])]
        for belt_solref,expected_mix in settings:
            model=mj.MjModel.from_xml_string(f'''<mujoco>
              <option timestep=".0025" integrator="discrete" gravity="0 0 0"/>
              <default><geom friction=".5 0 0" solref=".002 1" solimp=".99 .999 .001"/></default>
              <worldbody><geom name="left" type="box" pos="-.014 0 0" size=".0025 .1 .025" solref="{belt_solref}"/>
              <geom name="right" type="box" pos=".014 0 0" size=".0025 .1 .025" solref="{belt_solref}"/>
              <body><freejoint/><geom type="sphere" size=".012" mass=".01"/></body>
              </worldbody></mujoco>''')
            model.flg_surfacevel=True;model.geom_surfacevel[:2,1]=.15
            data=mj.MjData(model)
            for _ in range(80):mj.mj_step(model,data)
            self.assertGreater(data.qpos[1],.025)
            self.assertAlmostEqual(data.qvel[1],.15,places=4)
            self.assertEqual(data.ncon,2)
            np.testing.assert_allclose(data.contact[0].solref,expected_mix,rtol=1e-12,atol=1e-12)

    def test_discrete_contact_row_refsafe_is_preserved_by_positive_variants(self):
        device='cuda:0' if os.environ.get('S300_CONTACT_CUDA')=='1' else 'cpu'
        configurations=[('source_positive',[.002,1.]),('negative_raw_reference',[-250000.,-1000.]),
            ('positive_stiffness05',[.002,1/np.sqrt(.5)]),
            ('positive_stiffness025',[.002,2.]),('positive_damping05',[.004,.5]),
            ('positive_damping2',[.001,2.]),
            ('mixed_pair_stiffness05',[.002,(1+1/np.sqrt(.5))/2]),
            ('mixed_pair_stiffness025',[.002,1.5]),
            ('mixed_pair_damping05',[.003,.75]),('mixed_pair_damping2',[.0015,1.5])]
        records=[(hz,depth,name,ref) for hz in (120.,240.,400.) for depth in (0.,.0005,.001,.002,.003)
                 for name,ref in configurations]
        n=len(records);rows=2*n
        def a(value,dtype=float):return wp.array(np.asarray(value),dtype=dtype,device=device)
        raw=wp.zeros(n,wp.vec3,device=device)
        ints=[wp.zeros((1,rows),int,device=device) for _ in range(2)]
        floats=[wp.zeros((1,rows),float,device=device) for _ in range(6)]
        wp.launch(contact_row_control,n,inputs=[a([r[3] for r in records],wp.vec2),
            a([1/r[0] for r in records]),a([r[1] for r in records]),raw,*ints,*floats],device=device)
        wp.synchronize_device(device)
        coefficients=raw.numpy();diagonal=floats[2].numpy()[0].reshape(n,2)
        aref=floats[4].numpy()[0].reshape(n,2)
        result=[]
        for index,(hz,depth,name,ref) in enumerate(records):
            k,b,imp=map(float,coefficients[index]);r0=(1-imp)/imp
            f=float(diagonal[index,0])*r0
            effective_k=float(aref[index,0])*f/(imp*.0005)
            effective_b=float(aref[index,0]-aref[index,1])*f-(1/hz)*effective_k*imp
            result.append(dict(hz=hz,depth_m=depth,name=name,solref=ref,
                raw_normalized_k=k,raw_normalized_b=b,impedance=imp,
                row_recovered_normalized_k=effective_k,row_recovered_normalized_b=effective_b,
                D=diagonal[index].tolist(),aref=aref[index].tolist()))
        select=lambda name:next(r for r in result if r['hz']==400 and r['depth_m']==.0005 and r['name']==name)
        source=select('source_positive');negative=select('negative_raw_reference')
        khalf=select('positive_stiffness05');ddouble=select('positive_damping2')
        self.assertLess(source['row_recovered_normalized_k'],negative['row_recovered_normalized_k']*.7)
        self.assertGreater(khalf['row_recovered_normalized_b'],source['row_recovered_normalized_b']*1.2)
        self.assertAlmostEqual(ddouble['row_recovered_normalized_k']/source['row_recovered_normalized_k'],1.,places=5)
        self.assertAlmostEqual(ddouble['row_recovered_normalized_b']/source['row_recovered_normalized_b'],2.,places=4)
        mixed=select('mixed_pair_damping2')
        self.assertLess(mixed['raw_normalized_k'],source['raw_normalized_k']*.8)
        self.assertAlmostEqual(mixed['row_recovered_normalized_b']/source['row_recovered_normalized_b'],1.5,places=4)
        destination=os.environ.get('S300_CONTACT_ROW_EVIDENCE')
        if destination:
            Path(destination).write_text(json.dumps(dict(device=device,
                scope='InstalledMuJoCoWarp actual _contact_kbimp and _efc_row; normalized solver coefficients, not measured physical N/m.',
                warp_version=wp.__version__,mujoco_version=mj.__version__,
                constraint_source_sha256=sha256(constraint_module.__file__),
                fixed_pos_aref_m=-.0005,rows=result),indent=2)+'\n')

    def test_conversion_matches_installed_warp_contact_formula(self):
        device='cuda:0' if os.environ.get('S300_CONTACT_CUDA')=='1' else 'cpu'
        refs=wp.array(np.array([[.002,1.],[-250000.,-1000.],[.002,1.],[-40000.,-400.]]),dtype=wp.vec2,device=device)
        flags=wp.array(np.array([True,True,False,False]),dtype=bool,device=device)
        output=wp.zeros(4,wp.vec3,device=device)
        wp.launch(contact_coefficient_control,4,inputs=[refs,flags,output],device=device)
        wp.synchronize_device(device)
        values=output.numpy()
        np.testing.assert_allclose(values[0],values[1],rtol=3e-7,atol=.05)
        np.testing.assert_allclose(values[2],values[3],rtol=3e-7,atol=.05)
        self.assertGreater(values[0,0],6*values[2,0])

    def test_soft_belt_still_transmits_inward_drag_to_a_fruit(self):
        # Two moving contact surfaces indent a free fruit by0.5mm each. This
        # explicitly checks traction, rather than merely reading solref arrays.
        states=[]
        for solref in ('.002 1','-40000 -400','-10000 -400','-40000 -800'):
            model=mj.MjModel.from_xml_string(f'''<mujoco>
              <option timestep=".0025" gravity="0 0 0"/>
              <default><geom friction=".5 0 0" solref="{solref}" solimp=".99 .999 .001"/></default>
              <worldbody><geom name="left" type="box" pos="-.014 0 0" size=".0025 .1 .025"/>
              <geom name="right" type="box" pos=".014 0 0" size=".0025 .1 .025"/>
              <body><freejoint/><geom type="sphere" size=".012" mass=".01"/></body>
              </worldbody></mujoco>''')
            model.flg_surfacevel=True
            model.geom_surfacevel[:2,1]=.15
            data=mj.MjData(model)
            contacts=0
            for _ in range(80):
                mj.mj_step(model,data)
                contacts+=data.ncon
            self.assertEqual(contacts,160)
            self.assertGreater(data.qpos[1],.025)
            self.assertGreater(data.qvel[1],0.)
            self.assertLess(data.qvel[1],.35)
            self.assertLess(abs(data.qpos[0]),1e-8)
            states.append(data.qpos.copy())
        np.testing.assert_allclose(states[0],states[1],atol=1e-10)

    def test_direct_reference_accounts_for_two_step_time_constant_clamp(self):
        np.testing.assert_allclose(direct_reference([.002,1.],400),[-40000.,-400.])
        np.testing.assert_allclose(direct_reference([.01,1.],400),[-10000.,-200.])
        np.testing.assert_allclose(direct_reference([.002,1.],400,discrete=True),[-250000.,-1000.])
        np.testing.assert_allclose(direct_reference([.002,1.],60,discrete=True),[-250000.,-1000.])
        np.testing.assert_allclose(direct_reference([.002,1.],400,refsafe=False),[-250000.,-1000.])
        with self.assertRaises(ValueError):direct_reference([-.002,1.],400)

    def test_only_selected_stiffness_or_damping_coordinate_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'source';source.mkdir()
            model,metadata=fixture()
            mj.mj_saveLastXML(str(source/'input_scene.xml'),model)
            mj.mj_saveModel(model,str(source/'input_model.mjb'))
            (source/'source_export.json').write_text(json.dumps(metadata))
            np.savez(source/'initial_state.npz',qpos=model.qpos0)
            np.savez(source/'commands.npz',command=np.zeros((2,1)))
            (source/'sampled_plan.json').write_text('{}')
            filenames=['input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json']
            (source/'inputs.json').write_text(json.dumps({'input_files':{name:{'sha256':sha256(source/name)} for name in filenames}}))
            reference=prepare(source,root/'ref')
            k=prepare(source,root/'k',mode='stiffness',factor=.25)
            d=prepare(source,root/'d',mode='damping',factor=2)
            ref=np.array(reference['variant_solref'])
            np.testing.assert_array_equal(np.array(k['variant_solref'])[:,1],ref[:,1])
            np.testing.assert_array_equal(np.array(d['variant_solref'])[:,0],ref[:,0])
            self.assertTrue(all(k['exact_unchanged_model_arrays'].values()))
            self.assertEqual(sha256(source/'commands.npz'),sha256(root/'k'/'commands.npz'))
            self.assertEqual(sha256(source/'initial_state.npz'),sha256(root/'d'/'initial_state.npz'))
            # Proof itself detects a mutated source on a new preparation.
            (source/'sampled_plan.json').write_text('{"changed":true}')
            with self.assertRaises(FileExistsError):prepare(source,root/'ref')


if __name__=='__main__':unittest.main()
