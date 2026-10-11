"""Exercise the actual native loop transition guard without loading Isaac."""
import ast
import unittest
import numpy as np
from types import SimpleNamespace
from rail_motion import station_kinematics
from truss_sequence import horizontal_truss_direction
from test_run_motion_conditioning import MAIN,code

GUARD=next(n for n in ast.walk(MAIN) if isinstance(n,ast.If) and
    ast.unparse(n.test)=="rail_destination is not None and label != 'rail_move' and (rail_arrival_feedback is None)")

class RailArrivalWiringTests(unittest.TestCase):
    def test_entry_axis_sign_uses_destination_not_departure_station(self):
        assignment=next(n for n in ast.walk(MAIN) if isinstance(n,ast.Assign)
            and ast.unparse(n.targets[0])=='axis_hint'
            and isinstance(n.value,ast.BinOp))
        world=np.eye(4);world[1,3]=3.222
        kin=SimpleNamespace(world=world)
        dest=station_kinematics(kin,dict(axis_world=[0,1,0],displacement_m=.652))
        center=np.array([-.560,3.808,.889])
        # The truss's unsigned PCA points diagonally along the row. From the
        # departure station its sign is opposite to the actual arrival side.
        axis=np.array([.712,.702,0.]);axis/=np.linalg.norm(axis)
        positions=np.array([center-.1*axis,center+.1*axis])
        fruits=[dict(name=str(i),joint='/j'+str(i),path='/f'+str(i)) for i in range(2)]
        paths=[f['path'] for f in fruits]
        for planning in (kin,dest):
            context=dict(reference_center=center,kin=kin,planning_kin=planning)
            exec(code([assignment]),context)
            result=horizontal_truss_direction(fruits,[],paths,positions,context['axis_hint'])
            expected=horizontal_truss_direction(fruits,[],paths,positions,center-planning.world[:3,3])
            self.assertEqual(result,expected)
            if planning is dest:
                self.assertLess(result['direction_world'][0],0.)
            else:
                self.assertGreater(result['direction_world'][0],0.)
        np.testing.assert_array_equal(kin.world,world)
    def test_actual_loop_stops_before_manipulation_command_if_cart_is_stuck(self):
        context=dict(rail_destination={},label='ready_hold',rail_arrival_feedback=None,
            q=np.array([[.798033]]),qcommand=np.array([[1.622384]]),railid=0,t=34.,stop_reason=None)
        exec(code([GUARD]),context)
        self.assertEqual(context['stop_reason'],'rail_arrival_not_reached')
        self.assertFalse(context['rail_arrival_feedback']['reached'])
        text=ast.unparse(MAIN)
        guard=text.index("stop_reason = 'rail_arrival_not_reached'")
        stop=text.index("if stop_reason in ('rail_arrival_not_reached'",guard)
        self.assertLess(stop,text.index('robot.set_dof_position_targets(',stop))
    def test_actual_arrival_proceeds_and_feedback_is_recorded(self):
        context=dict(rail_destination={},label='ready_hold',rail_arrival_feedback=None,
            q=np.array([[1.6223]]),qcommand=np.array([[1.622384]]),railid=0,t=34.,stop_reason=None)
        exec(code([GUARD]),context)
        self.assertIsNone(context['stop_reason']);self.assertTrue(context['rail_arrival_feedback']['reached'])

if __name__=='__main__':unittest.main()
