"""Footprint/physical-planner direction agreement for shallow stem avoidance."""
import ast
from types import SimpleNamespace
import unittest
import numpy as np
from test_run_motion_conditioning import MAIN,code,actual_cli
from truss_sequence import horizontal_truss_direction,entry_direction


class TrussEntryYawTests(unittest.TestCase):
    def test_camera_footprint_heading_is_not_rotated_again_by_physical_planner(self):
        angle=np.deg2rad(15.)
        direction=[np.cos(angle),np.sin(angle),0.]
        axis=dict(direction_world=direction,direction_offset_already_applied=True)
        function=next(n for n in ast.walk(MAIN) if isinstance(n,ast.FunctionDef) and n.name=='make_plan')
        call=next(n for n in ast.walk(function) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='plan_motion')
        expression=next(k.value for k in call.keywords if k.arg=='approach_direction_world')
        result=eval(compile(ast.Expression(expression),'<camera entry>','eval'),
            dict(a=SimpleNamespace(truss_entry_yaw_offset_deg=15.),axis=axis,entry_direction=entry_direction))
        np.testing.assert_array_equal(result,direction)
        self.assertFalse(np.allclose(entry_direction(dict(direction_world=direction),15.),result))

    def test_group_direction_uses_same_offset_without_sparse_reapplication(self):
        function=next(n for n in ast.walk(MAIN) if isinstance(n,ast.FunctionDef) and n.name=='group_direction')
        axis=dict(direction_world=[1.,0.,0.])
        context=dict(a=SimpleNamespace(truss_axis_min_fruits=10,truss_entry_yaw_offset_deg=-30.,camera_harvest=False,near_trusses_metadata=None),
            last_axis=axis,horizontal_truss_direction=horizontal_truss_direction,entry_direction=entry_direction)
        exec(code([function]),context)
        fruits=[dict(name='Truss_00_Fruit_'+f'{i+1:02d}',path='/f'+str(i),joint='/j'+str(i)) for i in range(2)]
        paths=[x['path'] for x in fruits];positions=np.array([[0,0,1],[0,.02,1]])
        for _ in range(3):
            direction=context['group_direction']('Truss_00',fruits,[],paths,positions)
            np.testing.assert_allclose(direction,entry_direction(axis,-30.),atol=1e-14)
            self.assertEqual(context['last_axis'],axis)
        # make_plan uses this same helper, with the natural axis, for actual FK.
        plan_fn=next(n for n in ast.walk(MAIN) if isinstance(n,ast.FunctionDef) and n.name=='make_plan')
        call=next(n for n in ast.walk(plan_fn) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='plan_motion')
        expr=next(k.value for k in call.keywords if k.arg=='approach_direction_world')
        compiled=compile(ast.Expression(expr),'<entry direction>','eval')
        np.testing.assert_array_equal(eval(compiled,dict(context,axis=axis)),direction)

    def test_three_truss_gt_footprint_matches_actual_own_truss_pca_and_offset(self):
        from truss_sequence import near_scope_truss_direction
        function=next(n for n in ast.walk(MAIN)if isinstance(n,ast.FunctionDef)and n.name=='group_direction')
        fruits=[];positions=[]
        for key,center,direction in [('Truss_00',[0,0,1.1],[1,0,0]),
                                     ('Truss_01',[0,1,1.],[0,1,0]),
                                     ('Truss_02',[0,2,1.2],[1,.2,0])]:
            for i in range(3):
                fruits.append(dict(name=key+'_Fruit_'+f'{i+1:02d}',path='/f/'+key+'/'+str(i),joint='/j/'+key+'/'+str(i)))
                positions.append(np.array(center)+np.array(direction)*(.02*i))
        paths=[f['path']for f in fruits];positions=np.array(positions);world=np.eye(4);world[:3,3]=[2,-1,.7]
        context=dict(a=SimpleNamespace(truss_axis_min_fruits=2,truss_entry_yaw_offset_deg=5.,
            camera_harvest=False,near_trusses_metadata=object()),last_axis=dict(direction_world=[-1,0,0]),
            horizontal_truss_direction=horizontal_truss_direction,entry_direction=entry_direction,
            planning_kin=SimpleNamespace(world=world),verified_truss_axes={})
        exec(code([function]),context)
        for key in ['Truss_00','Truss_01','Truss_02']:
            actual=near_scope_truss_direction(dict(truss_key=key,fruits=[f['name']for f in fruits if f['name'].startswith(key+'_')]),
                fruits,[],paths,positions,world[:3,3],{},min_attached_fruits=2)
            np.testing.assert_allclose(context['group_direction'](key,fruits,[],paths,positions),
                entry_direction(actual,5.),rtol=0,atol=1e-14)
        with self.assertRaisesRegex(ValueError,'exact selected truss'):
            context['group_direction']('Truss_99',fruits,[],paths,positions)


    def test_cli_opt_in_requires_horizontal_truss_mode(self):
        base=['--mode','harvest','--preparation','horizontal_ready','--approach-yaw-mode','truss-axis']
        self.assertEqual(actual_cli(base).truss_entry_yaw_offset_deg,0.)
        self.assertEqual(actual_cli(base+['--truss-entry-yaw-offset-deg','-45']).truss_entry_yaw_offset_deg,-45.)
        from contextlib import redirect_stderr
        import io
        for args in (['--truss-entry-yaw-offset-deg','10'],base+['--truss-entry-yaw-offset-deg','91'],base+['--truss-entry-yaw-offset-deg','nan']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(args)

if __name__=='__main__':unittest.main()
