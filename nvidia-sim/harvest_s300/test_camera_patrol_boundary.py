"""Exercise the native runner's scan-only decision boundary, without Kit."""
import ast
import copy
from contextlib import redirect_stderr
import io
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from test_run_motion_conditioning import actual_cli
from test_camera_prefix_integration import CAMERA


class CameraPatrolBoundaryTests(unittest.TestCase):
    def test_cli_requires_actual_camera_mode_and_exact_target_identity(self):
        for args in (['--camera-scan-only'],['--camera-target-truss','Truss_02'],
                     CAMERA+['--camera-target-truss','any']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(args)
        args=actual_cli(CAMERA+['--camera-scan-only','--camera-target-truss','Truss_02'])
        self.assertTrue(args.camera_scan_only);self.assertEqual(args.camera_target_truss,'Truss_02')

    def test_scan_boundary_observes_current_state_and_exits_before_harvest_planning(self):
        tree=ast.parse(Path(__file__).with_name('run_motion.py').read_text())
        boundary=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and
                      ast.unparse(n.test)=='a.camera_scan_only')
        poses=np.array([[1.,2.,3.,0.,0.,0.,1.]])
        sequence={'cycle_plans':[{'status':'motion_completed','fruits':[]}],
                  'attempt_counts':{'Truss_02_Fruit_01':1}}
        before=copy.deepcopy(sequence);observed=[]
        context=dict(a=SimpleNamespace(camera_scan_only=True),sequence=sequence,
                     body_paths=['body'],poses=poses,
                     camera_observe=lambda paths,state:observed.append((list(paths),state.copy())))
        wrapped=ast.Module(body=[ast.While(test=ast.Constant(True),body=[boundary],orelse=[])],type_ignores=[])
        exec(compile(ast.fix_missing_locations(wrapped),'<actual scan boundary>','exec'),context)
        self.assertEqual(len(observed),1);np.testing.assert_array_equal(observed[0][1],poses)
        self.assertEqual(sequence['termination'],'camera_scan_complete_patrol_decision_required')
        self.assertEqual(sequence['attempt_counts'],before['attempt_counts'])
        self.assertEqual(sequence['cycle_plans'],before['cycle_plans'])
        self.assertEqual(sequence['camera_scan_only']['harvest_attempts'],0)


if __name__=='__main__':unittest.main()
