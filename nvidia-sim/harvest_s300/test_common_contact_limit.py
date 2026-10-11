"""Common 1mm diagnostic acceptance, while physical settings stay unchanged."""
from contextlib import redirect_stderr
import ast
import io
from pathlib import Path
import unittest
from types import SimpleNamespace

import numpy as np
from test_run_motion_conditioning import actual_cli
from motion_contact_buffer import BufferedContactRecorder
from motion_contact_guard import FruitContactGuard
from leaf_contact_geometry import CookedLeafContactGeometry


class CommonContactLimitTests(unittest.TestCase):
    def test_default_and_explicit_reference_do_not_change_physics_settings(self):
        current=vars(actual_cli([]));reference=vars(actual_cli(['--contact-limit-mm','.5']))
        for name in ('contact_limit_mm','fruit_contact_limit_mm','fruit_fruit_limit_mm'):
            self.assertEqual(current.pop(name),1.)
            self.assertEqual(reference.pop(name),.5)
        self.assertEqual(current,reference)

    def test_invalid_common_limits_rejected_before_launch(self):
        for value in ('nan','inf','0.49','1.01'):
            with self.subTest(value=value),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                actual_cli(['--contact-limit-mm',value])

    def test_initial_contact_guard_and_stop_label_use_configured_limit(self):
        source=Path(__file__).with_name('run_motion.py')
        tree=ast.parse(source.read_text())
        selected=[]
        for node in ast.walk(tree):
            if not isinstance(node,ast.Assign) or len(node.targets)!=1:continue
            name=getattr(node.targets[0],'id',None)
            if name=='initialization_invalid' and isinstance(node.value,ast.ListComp):selected.append(node)
            if name=='stop_reason' and 'initialization_penetration' in ast.unparse(node.value):selected.append(node)
        self.assertEqual(len(selected),2)
        program=compile(ast.Module(body=sorted(selected,key=lambda n:n.lineno),type_ignores=[]),str(source),'exec')
        for limit,depth,failed in ((.5,.0008,True),(1.,.0008,False),(1.,.0012,True)):
            context=dict(a=SimpleNamespace(contact_limit_mm=limit),initialization_events=[
                dict(a='/World/HarvestRobot/arm',b='/World/Greenhouse/gutter',separation_m=-depth),
                dict(a='/World/Greenhouse/static_a',b='/World/Greenhouse/static_b',separation_m=-.003)])
            exec(program,context)
            self.assertEqual(bool(context['initialization_invalid']),failed)
            expected=('initialization_penetration_over_0_5mm' if limit==.5 else
                      'initialization_penetration_over_contact_policy') if failed else None
            self.assertEqual(context['stop_reason'],expected)

    def test_all_active_pairs_share_common_limit_and_keep_raw_evidence(self):
        paths={1:'/World/ActuatedTool/L_belt/PrototypeTread',2:'/World/ElasticTarget/fruit',
               3:'/World/HarvestRobot/arm',4:'/World/Greenhouse/gutter',
               5:'/World/ElasticTarget/stem',6:'/World/ActuatedTool/R/upper_deck'}
        reader=BufferedContactRecorder(paths.__getitem__,60,penetration_limit_m=.001)
        pairs=[(1,2),(3,4),(5,6)]
        headers=[SimpleNamespace(collider0=a,collider1=b,contact_data_offset=i,num_contact_data=1)
                 for i,(a,b) in enumerate(pairs)]
        for depth,expected in ((.0008,0),(.001,0),(.00101,3)):
            reader.clear_step()
            reader.record(headers,[SimpleNamespace(separation=-depth,impulse=[0,.01,0])]*3,t=depth,phase='close')
            self.assertEqual(len(reader.step_invalid_events()),expected)
        self.assertEqual(reader.record_count,9)
        self.assertEqual(reader.materialize()[0]['separation_m'],-.0008)

    def test_pair_override_does_not_revert_other_pairs_to_half_mm(self):
        f=dict(name='Truss_02_Fruit_01',path='/World/ElasticTarget/Truss_02_Fruit_01')
        m=dict(target=f,plant=dict(root='/World/ElasticTarget',fruits=[f]))
        g=FruitContactGuard(m,['/World/ActuatedTool/L_belt/tread'],
                           fruit_limit_m=.0005,fruit_fruit_limit_m=.001,default_limit_m=.001)
        self.assertEqual(g.limit('/World/HarvestRobot/arm','/World/ElasticTarget/stem'),.001)
        self.assertEqual(g.limit(f['path']+'/Collider','/World/ActuatedTool/L_belt/tread'),.0005)

    def test_solved_geometry_uses_same_limit_with_numerical_guard(self):
        leaf='/World/ElasticTarget/Leaf_02/Appendage_1_0';tool='/World/ActuatedTool/R/part_244'
        c=CookedLeafContactGeometry(None,dict(compliant_foliage=dict(leaves=[dict(meshes=[leaf])])),['leaf','tool'])
        v=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        c.cooked=lambda path:dict(body='leaf' if path==leaf else 'tool',hulls=[(v,np.eye(3))])
        poses=np.array([[0,0,0,0,0,0,1],[.0012,0,0,0,0,0,1.]])
        self.assertFalse(c.verify(leaf,tool,poses,1.,-.0015)['certified_within_limit'])
        relaxed=c.verify(leaf,tool,poses,1.,-.0015,limit_m=.001)
        self.assertTrue(relaxed['certified_within_limit'])
        self.assertEqual(relaxed['limit_m'],.001)
        self.assertAlmostEqual(relaxed['penetration_upper_bound_m'],.0008)
        poses[1,0]=.0008
        self.assertFalse(c.verify(leaf,tool,poses,1.,-.0015,limit_m=.001)['certified_within_limit'])


if __name__=='__main__':unittest.main()
