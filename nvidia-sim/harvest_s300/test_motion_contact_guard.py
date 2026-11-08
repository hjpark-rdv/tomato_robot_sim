import unittest
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

from motion_contact_buffer import BufferedContactRecorder
from motion_contact_guard import FruitContactGuard


class FruitGuardTests(unittest.TestCase):
    def setUp(self):
        self.root='/World/ElasticTarget'
        self.fruit=self.root+'/Truss_02_Fruit_09/Collider'
        self.other=self.root+'/Truss_01_Fruit_09/Collider'
        self.tool='/World/ActuatedTool/R_belt/PrototypeTread'
        fruits=[dict(name=n,path=self.root+'/'+n) for n in ('Truss_02_Fruit_09','Truss_01_Fruit_09')]
        self.manifest=dict(target=fruits[0],plant=dict(root=self.root,fruits=fruits))
        self.guard=FruitContactGuard(self.manifest,[self.tool],fruit_limit_m=.001)

    def event(self,a=None,b=None,depth=.0008,t=.1):
        return dict(a=a or self.tool,b=b or self.fruit,separation_m=-depth,force_n=1.25,t=t)

    def test_only_exact_tool_and_same_truss_fruit_pairs_relaxed_in_both_orders(self):
        self.assertFalse(self.guard.invalid(self.event()))
        self.assertFalse(self.guard.invalid(self.event(a=self.fruit,b=self.tool)))
        for a,b in [(self.tool,self.other),('/World/HarvestRobot/arm',self.fruit),
                    (self.tool,self.root+'/Truss_02_Rachis_1/Collider'),
                    ('/World/Ground',self.fruit),(self.tool,self.fruit+'_extra'),
                    (self.tool+'_extra',self.fruit)]:
            self.assertTrue(self.guard.invalid(self.event(a,b)))
        self.assertFalse(self.guard.invalid(self.event(depth=.001)))
        self.assertTrue(self.guard.invalid(self.event(depth=.001001)))

    def test_default_budget_retains_original_guard(self):
        guard=FruitContactGuard(self.manifest,[self.tool])
        self.assertTrue(guard.invalid(self.event()))
        self.assertFalse(guard.invalid(self.event(depth=.0005)))
        for value in (.0004,float('nan'),float('inf')):
            with self.assertRaises(ValueError):FruitContactGuard(self.manifest,[self.tool],fruit_limit_m=value)

    def test_fruit_fruit_budget_excludes_other_truss_and_stem(self):
        second=dict(name='Truss_02_Fruit_10',path=self.root+'/Truss_02_Fruit_10')
        self.manifest['plant']['fruits'].append(second)
        guard=FruitContactGuard(self.manifest,[self.tool],fruit_limit_m=.002,fruit_fruit_limit_m=.001)
        self.assertFalse(guard.invalid(self.event(a=self.fruit,b=second['path']+'/Collider')))
        self.assertTrue(guard.invalid(self.event(a=self.fruit,b=self.other)))
        self.assertTrue(guard.invalid(self.event(a=self.fruit,b=self.root+'/Stem_01/Collider')))

    def test_buffer_preserves_tolerated_raw_records_and_blocks_other_pairs(self):
        paths={1:self.tool,2:self.fruit,3:self.other}
        reader=BufferedContactRecorder(paths.__getitem__,60,pair_limit=self.guard.limit)
        headers=[SimpleNamespace(collider0=1,collider1=b,contact_data_offset=i,num_contact_data=1) for i,b in enumerate([2,3])]
        points=[SimpleNamespace(separation=-.0008,impulse=[0.,.01,0.]) for _ in headers]
        reader.record(headers,points,t=.1,phase='close')
        self.assertEqual(len(reader.materialize()),2)
        self.assertEqual(reader.materialize()[0]['force_n'],.6)
        self.assertEqual([e['b'] for e in reader.step_invalid_events()],[self.other])

    def test_warning_durations_count_distinct_physics_samples_and_gaps(self):
        events=[self.event(t=t) for t in (.1,.1,.2,.4)]
        evidence=self.guard.summarize(events,10)
        self.assertFalse(evidence['strict_0_5mm_passed'])
        self.assertEqual(evidence['configured_violation_contact_points'],0)
        row=evidence['exceedance_pairs'][0]
        self.assertEqual(row['contact_points'],4)
        self.assertEqual(row['physics_samples'],3)
        self.assertAlmostEqual(row['sampled_exceedance_s'],.3)
        self.assertAlmostEqual(row['longest_sampled_exceedance_s'],.2)

    def test_cli_rejects_unmapped_or_invalid_overrides_before_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            for flags in (['--fruit-contact-limit-mm','nan'],
                          ['--fruit-fruit-limit-mm','0.4'],
                          ['--contact-limit-mm','.5','--fruit-contact-limit-mm','1','--contact-backend','tensor'],
                          ['--contact-limit-mm','.5','--fruit-fruit-limit-mm','1','--num-envs','2']):
                output=Path(tmp)/'output'
                cmd=[sys.executable,str(Path(__file__).with_name('run_motion.py')),tmp,'--output',str(output),*flags]
                p=subprocess.run(cmd,capture_output=True,text=True)
                self.assertEqual(p.returncode,2,p.stderr)
                self.assertFalse(output.exists())


class TPUBeltIndentationTests(unittest.TestCase):
    def fixture(self):
        from pxr import Sdf,Usd,UsdGeom,UsdPhysics
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget';fruits=[];belts={}
        for side,number in [('L',163),('R',285)]:
            body='/World/ActuatedTool/'+side+'_belt';owner=UsdGeom.Xform.Define(stage,body).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(owner)
            owner.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['PhysicsRigidBodyAPI','PhysxSurfaceVelocityAPI']))
            owner.CreateAttribute('physxSurfaceVelocity:surfaceVelocityLocalSpace',Sdf.ValueTypeNames.Bool).Set(True)
            path=body+'/part_'+str(number);belts[side]=path
            mesh=UsdGeom.Mesh.Define(stage,path);mesh.GetPrim().SetCustomDataByKey('sourceName',side+'_C07_contact_belt_TPU')
            mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)])
            mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        for name in ['Truss_00_Fruit_01','Truss_02_Fruit_09']:
            body=root+'/'+name;UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,body).GetPrim())
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,body+'/Collider').GetPrim()).CreateCollisionEnabledAttr(True)
            fruits.append(dict(name=name,path=body,joint=root+'/HarvestJoints/'+name))
        return stage,dict(plant=dict(root=root,fruits=fruits)),belts

    def test_exact_pair_both_orders_only_two_closing_phases_and_two_mm(self):
        from motion_contact_guard import TPUBeltIndentationDiagnostic
        stage,m,belts=self.fixture();policy=TPUBeltIndentationDiagnostic(m,stage)
        fruit=m['plant']['fruits'][1]['path']+'/Collider'
        event=dict(a=belts['R'],b=fruit,t=2.,phase='close',separation_m=-.00107789,force_n=.609831,impulse_ns=[0,0,.01])
        for phase in ['close','cycle_reclose']:
            for a,b in [(belts['R'],fruit),(fruit,belts['R'])]:
                e=dict(event,a=a,b=b,phase=phase)
                self.assertEqual(policy.filter_invalid([e],[]),[])
        for phase in ['approach','belt_finish','cycle_open','cycle_open_final','cycle_open_hold','withdrawal','settle']:
            e=dict(event,phase=phase);self.assertEqual(policy.filter_invalid([e],[]),[e])
        e=dict(event,separation_m=-.002);self.assertEqual(policy.filter_invalid([e],[]),[])
        e=dict(event,separation_m=-.002000001);self.assertEqual(policy.filter_invalid([e],[]),[e])
        row=policy.records[0];self.assertEqual(row['raw_event'],event)
        self.assertFalse(row['physical_accuracy_verified']);self.assertFalse(row['elastic_belt_physics_implemented'])
        self.assertEqual(policy.metadata()['all_other_pairs_and_phases_limit_m'],.001)

    def test_release_changes_evidence_never_pair_phase_or_depth_permission(self):
        from motion_contact_guard import TPUBeltIndentationDiagnostic
        stage,m,belts=self.fixture();policy=TPUBeltIndentationDiagnostic(m,stage);fruit=m['plant']['fruits'][0]
        event=dict(a=belts['L'],b=fruit['path']+'/Collider',t=2.,phase='close',separation_m=-.0015,force_n=1.)
        for breaks,released in [([],False),([dict(joint=fruit['joint'],t=3.)],False),([dict(joint=fruit['joint'],t=1.)],True)]:
            self.assertEqual(policy.filter_invalid([event],breaks),[])
            self.assertEqual(policy.records[-1]['native_release_verified'],released)
            bad=dict(event,phase='cycle_open');self.assertEqual(policy.filter_invalid([bad],breaks),[bad])
        for other in ['/World/Greenhouse/Plants/Plant_21/Truss_00_Fruit_01/Collider',
                      '/World/ElasticTarget/Truss_00_Pedicel_proximal_01_0/Collider',
                      '/World/ElasticTarget/Stem_MainStem_0/Collider',
                      '/World/Greenhouse/House/Environment/Gutter_1_Base_2/Mesh']:
            e=dict(event,b=other);self.assertEqual(policy.filter_invalid([e],[]),[e])
        for other in ['/World/ActuatedTool/R/part_331','/World/ActuatedTool/L_belt/PrototypeTread','/World/HarvestRobot/Robot/link3']:
            e=dict(event,a=other);self.assertEqual(policy.filter_invalid([e],[]),[e])

    def test_raw_one_mm_buffer_never_caches_relaxed_phase_permission(self):
        from motion_contact_guard import TPUBeltIndentationDiagnostic
        stage,m,belts=self.fixture();policy=TPUBeltIndentationDiagnostic(m,stage)
        paths={1:belts['R'],2:m['plant']['fruits'][0]['path']+'/Collider'}
        recorder=BufferedContactRecorder(paths.__getitem__,60,penetration_limit_m=.001)
        header=SimpleNamespace(collider0=1,collider1=2,contact_data_offset=0,num_contact_data=1)
        point=SimpleNamespace(separation=-.0015,impulse=[0,.01,0])
        for t,phase in [(1.,'close'),(2.,'cycle_open'),(3.,'cycle_reclose'),(4.,'approach')]:
            recorder.clear_step();recorder.record([header],[point],t=t,phase=phase)
            raw=recorder.step_invalid_events();self.assertEqual(len(raw),1)
            filtered=policy.filter_invalid(raw,[])
            self.assertEqual(len(filtered),0 if phase in policy.PHASES else 1)
        self.assertEqual(len(recorder.materialize()),4)
        self.assertEqual([e['force_n'] for e in recorder.materialize()],[.6]*4)
        self.assertEqual([r['raw_event']['phase'] for r in policy.records],['close','cycle_reclose'])

    def test_bad_owner_source_collision_and_fruit_mapping_fail_closed(self):
        from pxr import UsdPhysics
        from motion_contact_guard import TPUBeltIndentationDiagnostic
        for mutation in ['source','owner','disabled','fruit']:
            stage,m,belts=self.fixture();prim=stage.GetPrimAtPath(belts['L'])
            if mutation=='source':prim.SetCustomDataByKey('sourceName','L_C08_metal_plate')
            elif mutation=='owner':UsdPhysics.RigidBodyAPI.Apply(prim)
            elif mutation=='disabled':UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
            else:m['plant']['fruits'][0]['path']='/World/Other/Truss_00_Fruit_01'
            with self.assertRaises(ValueError):TPUBeltIndentationDiagnostic(m,stage)

    def test_explicit_cli_only_default_unchanged_and_no_global_override(self):
        import ast
        script=Path(__file__).with_name('run_motion.py');tree=ast.parse(script.read_text())
        args=next(node for node in tree.body if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='a' for t in node.targets))
        parser_prefix=ast.Module(body=tree.body[:tree.body.index(args)+1],type_ignores=[])
        old=sys.argv
        try:
            sys.argv=[str(script),'/tmp/source','--output','/tmp/output'];ns={'__name__':'parser_test','__file__':str(script)}
            exec(compile(parser_prefix,str(script),'exec'),ns)
            self.assertFalse(ns['a'].experimental_tpu_fruit_close_2mm)
        finally:sys.argv=old
        with tempfile.TemporaryDirectory() as temp:
            for flags in [[],['--mode','harvest','--preparation','horizontal_ready','--harvest-profile','belt_open_close_cycle','--contact-limit-mm','.5']]:
                p=subprocess.run([sys.executable,str(script),temp,'--output',temp+'/out','--experimental-tpu-fruit-close-2mm',*flags],capture_output=True,text=True)
                self.assertEqual(p.returncode,2,p.stderr)
                self.assertIn('TPU closing diagnostic requires',p.stderr)


if __name__=='__main__':unittest.main()
