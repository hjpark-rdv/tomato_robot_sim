from types import SimpleNamespace as NS
import copy
from gpu_initialization import same_calibration


def slot():
    return NS(dataset_origin=[0,0,0],ready_joints={'shoulder':.3},
              cfg=NS(robot=NS(init_state=NS(pos=[1,2,3],rot=[1,0,0,0])),lift_start_below=.4,lift_height_reference='mount'),
              target_spec={'name':'Tomato_05'},fruit_specs=[{'pose':[1,2,3,1,0,0,0]}])


def test_calibration_is_shared_only_for_identical_scene_and_start_pose():
    a,b=slot(),slot();assert same_calibration(a,b)
    changes=[lambda s:s.dataset_origin.__setitem__(0,1),lambda s:s.ready_joints.update(shoulder=.4),
             lambda s:setattr(s.cfg,'lift_start_below',.3),lambda s:s.target_spec.update(name='Tomato_06'),
             lambda s:s.fruit_specs[0]['pose'].__setitem__(1,3)]
    for change in changes:
        b=copy.deepcopy(a);change(b);assert not same_calibration(a,b)
