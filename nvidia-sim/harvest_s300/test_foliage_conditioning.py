import copy
import tempfile
import unittest
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from foliage_conditioning import FoliageConditioning, conditioning_inventory, validate_report, save_imported_source

ROOT='/World/ElasticTarget'
LEAF=ROOT+'/Leaf_CompoundLeaf_01'
MESH=LEAF+'/Appendage_136_0'
FRUIT=ROOT+'/Truss_00_Fruit_02'
STEM=ROOT+'/Stem_MainStem_02'
STATIC='/World/Greenhouse/Plants/Plant_41/Stem/mesh_129'
WIRE='/World/Greenhouse/House/Environment/GutterSuspensionWire_018/Mesh'


def collision(stage,path,name):
    p=UsdGeom.Mesh.Define(stage,path).GetPrim();p.SetCustomDataByKey('sourceName',name)
    UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexDecomposition')
    return p


def fixture():
    s=Usd.Stage.CreateInMemory()
    for body in (LEAF,FRUIT,STEM):
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(s,body).GetPrim()).CreateRigidBodyEnabledAttr(True)
    collision(s,MESH,'CompoundLeaf_01_214888')
    j=UsdPhysics.Joint.Define(s,ROOT+'/Joints/Leaf_CompoundLeaf_01');j.CreateBody1Rel().SetTargets([LEAF]);j.CreateBody0Rel().SetTargets([STEM])
    collision(s,FRUIT+'/Collider','Fruit_02')
    p=UsdGeom.Capsule.Define(s,STEM+'/Collider').GetPrim();UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
    collision(s,STATIC,'CompoundLeaf_01_214888');collision(s,WIRE,'source wire')
    for i,name in enumerate(('Fruit_03','Calyx_03_Sepal_1','Pedicel_distal_03','Rachis','Truss_01_Peduncle')):
        collision(s,f'/World/Greenhouse/Plants/Plant_39/Truss_00/mesh_{28+i}',name)
    collision(s,'/World/Greenhouse/Plants/Plant_39/Stem/mesh_1','MainStem')
    collision(s,'/World/Greenhouse/Plants/Plant_39/Stem/mesh_2','UnknownHardPart')
    collision(s,'/World/HarvestRobot/Robot/S300Carrier/Collision','CompoundLeaf_01')
    m=dict(plant=dict(root=ROOT,bodies=[dict(path=STEM,name='Stem_MainStem_02')],
        fruits=[dict(path=FRUIT,name='Truss_00_Fruit_02')]),compliant_foliage=dict(leaves=[dict(path=LEAF,
        source_name='CompoundLeaf_01',joint=str(j.GetPath()),meshes=[MESH])]))
    paths=[LEAF,FRUIT,STEM];poses=np.zeros((3,7));poses[:,6]=1.;velocity=np.zeros((3,6))
    return s,m,paths,poses,velocity


def contact(t=.25,a=MESH,b=STATIC):
    return dict(t=t,a=a,b=b,separation_m=-.004,force_n=12.,phase='idle',impulse_ns=[.1,.2,.3])


def started(hz=4,seconds=1):
    s,m,paths,p,v=fixture();c=FoliageConditioning(s,m,paths,seconds=seconds,hz=hz)
    c.begin(0.,p,v);return c,p,v


def valid_report():
    c,p,v=started()
    for t in np.arange(.25,2.01,.25):
        events=[contact(t)] if t<1 else []
        result=c.before_geometry(float(t),events,[],p,v)
        assert result['abort_reason'] is None
        assert c.after_geometry(float(t),result['remaining_invalid']) is None
    return c.report()


class ConditioningTest(unittest.TestCase):
    def test_exact_leaf_inventory_and_counterpart_roles_without_stage_mutation(self):
        s,m,paths,p,v=fixture();before=s.GetRootLayer().ExportToString()
        leaves,others=conditioning_inventory(s,m,paths)
        self.assertEqual(set(leaves),{MESH});self.assertIn(FRUIT+'/Collider',others)
        self.assertIn(STEM+'/Collider',others);self.assertIn(WIRE,others)
        for i in range(28,33):self.assertIn(f'/World/Greenhouse/Plants/Plant_39/Truss_00/mesh_{i}',others)
        self.assertNotIn('/World/Greenhouse/Plants/Plant_39/Stem/mesh_2',others)
        self.assertNotIn('/World/HarvestRobot/Robot/S300Carrier/Collision',others)
        self.assertEqual(before,s.GetRootLayer().ExportToString())
        m['compliant_foliage']['leaves'][0]['source_name']='CompoundLeaf_02'
        with self.assertRaisesRegex(ValueError,'declaration mismatch'):conditioning_inventory(s,m,paths)

    def test_existing_appendage_without_source_name_maps_exact_existing_visual(self):
        s,m,paths,p,v=fixture();s.GetPrimAtPath(MESH).ClearCustomDataByKey('sourceName')
        visual=UsdGeom.Mesh.Define(s,LEAF+'/Visual_136').GetPrim()
        visual.SetCustomDataByKey('sourceName','CompoundLeaf_01_214888')
        m['compliant_foliage']['leaves'][0]['meshes'].append(str(visual.GetPath()))
        leaves,_=conditioning_inventory(s,m,paths)
        self.assertEqual(leaves[MESH]['source_visual'],str(visual.GetPath()))
        visual.SetCustomDataByKey('sourceName','CompoundLeaf_02')
        with self.assertRaisesRegex(ValueError,'source mapping mismatch'):conditioning_inventory(s,m,paths)

    def test_all_raw_deferred_events_preserved_both_orders_without_clearance_claim(self):
        c,p,v=started();event=contact();original=copy.deepcopy(event)
        for a,b in ((MESH,STATIC),(STATIC,MESH),(MESH,FRUIT+'/Collider'),(MESH,STEM+'/Collider'),(MESH,WIRE),(MESH,MESH)):
            proof=c.classify(contact(a=a,b=b));self.assertIsNotNone(proof)
        result=c.before_geometry(.25,[event],[],p,v)
        self.assertEqual(result['remaining_invalid'],[]);self.assertTrue(result['query_skipped'])
        self.assertEqual(event,original);self.assertEqual(c.deferred[0]['impulse_ns'],event['impulse_ns'])
        self.assertFalse(c.deferred[0]['clearance_certificate']);self.assertFalse(c.deferred[0]['operational_physics_valid'])
        c.after_geometry(.25,[])

    def test_nonleaf_robot_unknown_or_disabled_source_cannot_be_deferred(self):
        pairs=[(FRUIT+'/Collider',STATIC),(MESH,'/World/HarvestRobot/Robot/S300Carrier/Collision'),
               (MESH,'/World/Greenhouse/Plants/Plant_39/Stem/mesh_2')]
        for a,b in pairs:
            c,p,v=started();result=c.before_geometry(.25,[contact(a=a,b=b)],[],p,v)
            self.assertEqual(result['abort_reason'],'nonleaf_or_undeclared_collision_during_conditioning')
            self.assertFalse(c.deferred)

    def test_valid_strict_window_is_separate_from_invalid_conditioning(self):
        report=valid_report();self.assertTrue(report['canonical_conditioned_start'])
        self.assertEqual(report['strict_window']['duration_s'],1.)
        self.assertEqual(report['strict_window']['sample_count'],5)
        self.assertEqual(len(report['deferred_raw_events']),3)
        self.assertFalse(report['completed_cycle']);self.assertIsNone(report['harvest_success'])
        self.assertEqual(validate_report(report)['native_step_count'],5)
        self.assertEqual(report['stability']['bodies'][0]['terminal_linear_velocity_m_s'],[0.,0.,0.])

    def test_terminal_strict_collision_is_never_deferred_or_extended(self):
        c,p,v=started()
        for t in (.25,.5,.75):
            result=c.before_geometry(t,[contact(t)],[],p,v);c.after_geometry(t,result['remaining_invalid'])
        result=c.before_geometry(1.,[contact(1.)],[],p,v)
        self.assertEqual(len(result['remaining_invalid']),1);self.assertFalse(result['query_skipped'])
        self.assertEqual(c.after_geometry(1.,result['remaining_invalid']),'strict_penetration_over_0_5mm')
        self.assertFalse(c.report()['canonical_conditioned_start'])
        with self.assertRaises(ValueError):validate_report(c.report())

    def test_finite_velocities_breaks_and_complete_timestamps_are_hard_requirements(self):
        c,p,v=started();v[1,0]=float('nan')
        self.assertEqual(c.before_geometry(.25,[],[],p,v)['abort_reason'],'Nonfinite measured body pose/velocity')
        c,p,v=started();self.assertEqual(c.before_geometry(.5,[],[],p,v)['abort_reason'],'missing_or_duplicate_native_timestamp')
        c,p,v=started();self.assertEqual(c.before_geometry(.25,[],[dict(joint='/any',t=.1)],p,v)['abort_reason'],'new_native_break_during_idle_conditioning')
        s,m,paths,p,v=fixture();c=FoliageConditioning(s,m,paths,seconds=1,hz=4)
        self.assertEqual(c.begin(0,p,v,[dict(joint='/warmup',t=0)]),'native_break_in_initializer_or_initial_state')
        c,p,v=started();self.assertEqual(c.before_geometry(.25,[contact(.1)],[],p,v)['abort_reason'],'missing_stale_or_nonfinite_contact_evidence')
        c,p,v=started();c.before_geometry(.25,[],[],p,v)
        self.assertEqual(c.before_geometry(.5,[],[],p,v)['abort_reason'],'missing_strict_after_geometry_step')

    def test_increasing_drift_fails_but_large_bounded_initial_shift_is_diagnostic(self):
        for accelerating in (False,True):
            c,p,v=started()
            for t in np.arange(.25,2.01,.25):
                pp=p.copy();pp[0,0]=.1+((t-1)**2*.001 if accelerating and t>=1 else 0.)
                result=c.before_geometry(float(t),[],[],pp,v);c.after_geometry(float(t),result['remaining_invalid'])
            report=c.report();self.assertEqual(report['canonical_conditioned_start'],not accelerating)
            self.assertGreater(report['stability']['bodies'][0]['terminal_position_delta_from_initial_m'][0],.099)
            if accelerating:self.assertFalse(report['stability']['no_increasing_drift'])
            else:validate_report(report)

    def test_validate_rejects_forged_terminal_waiver_gaps_and_wrong_threshold(self):
        original=valid_report()
        for mutate in (lambda r:r.update(strict_penetration_threshold_m=.001),
                       lambda r:r['native_step_records'][-1].update(deferred_count=1),
                       lambda r:r['native_step_records'][-1].update(t=3.),
                       lambda r:r.update(no_robot_or_nonleaf_exemptions=False),
                       lambda r:r['deferred_raw_events'][0].update(b='/World/HarvestRobot/Robot/link'),
                       lambda r:r['deferred_raw_events'][0].update(t=1.)):
            report=copy.deepcopy(original);mutate(report)
            with self.assertRaises(ValueError):validate_report(report)

    def test_opt_in_bounded_idle_and_imported_source_is_immutable(self):
        s,m,paths,p,v=fixture()
        for kwargs in (dict(seconds=0,hz=4),dict(seconds=3.1,hz=4),dict(seconds=1,hz=4,strict_seconds=.9),
                       dict(seconds=1,hz=4,mode='harvest'),dict(seconds=1,hz=4,callback_complete=False)):
            with self.assertRaises(ValueError):FoliageConditioning(s,m,paths,**kwargs)
        with tempfile.TemporaryDirectory() as t:
            proof=save_imported_source(t);self.assertEqual(proof,save_imported_source(t))
            Path(proof['path']).write_text('wrong source')
            with self.assertRaises(ValueError):save_imported_source(t)

if __name__=='__main__':unittest.main()
