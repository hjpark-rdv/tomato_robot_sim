"""Footprint/physical-planner direction agreement for shallow stem avoidance."""
import ast
from types import SimpleNamespace
import unittest
import numpy as np
from test_run_motion_conditioning import MAIN,code,actual_cli
from truss_sequence import horizontal_truss_direction,entry_direction


class TrussEntryYawTests(unittest.TestCase):
    def test_group_direction_uses_same_offset_without_sparse_reapplication(self):
        function=next(n for n in ast.walk(MAIN) if isinstance(n,ast.FunctionDef) and n.name=='group_direction')
        axis=dict(direction_world=[1.,0.,0.])
        context=dict(a=SimpleNamespace(truss_axis_min_fruits=10,truss_entry_yaw_offset_deg=-30.),
            last_axis=axis,horizontal_truss_direction=horizontal_truss_direction,entry_direction=entry_direction)
        exec(code([function]),context)
        fruits=[dict(name=str(i),path='/f'+str(i),joint='/j'+str(i)) for i in range(2)]
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

    def test_cli_opt_in_requires_horizontal_truss_mode(self):
        base=['--mode','harvest','--preparation','horizontal_ready','--approach-yaw-mode','truss-axis']
        self.assertEqual(actual_cli(base).truss_entry_yaw_offset_deg,0.)
        self.assertEqual(actual_cli(base+['--truss-entry-yaw-offset-deg','-45']).truss_entry_yaw_offset_deg,-45.)
        from contextlib import redirect_stderr
        import io
        for args in (['--truss-entry-yaw-offset-deg','10'],base+['--truss-entry-yaw-offset-deg','91'],base+['--truss-entry-yaw-offset-deg','nan']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(args)

if __name__=='__main__':unittest.main()
