import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation as R
from robot_engine import RobotEngine,forward

def test_robot_kinematics_and_dynamic_hook():
 e=RobotEngine();assert e.model.nmocap==0;assert e.model.nu==7
 rng=np.random.default_rng(42)
 for _ in range(20):
  q=e.initial+rng.uniform(-.03,.03,7);e.data.qpos[e.qids]=q;mj.mj_forward(e.model,e.data);expected=forward(e.ref,q)
  assert np.linalg.norm(expected[:3,3]-e.data.xpos[e.hook])<1e-6
  assert (R.from_matrix(expected[:3,:3]).inv()*R.from_matrix(e.data.xmat[e.hook].reshape(3,3))).magnitude()<1e-6

def test_replay_reset_and_collision_masks():
 e=RobotEngine();initial=e.data.qpos.copy();a=e.rollout(seconds=.25);e.reset();np.testing.assert_array_equal(e.data.qpos,initial)
 b=e.rollout(seconds=.25);assert a['hook_final_xyz']==b['hook_final_xyz'];assert not b['unstable']
 robot=[i for i in range(e.model.ngeom) if e.model.geom(i).name.startswith('robot_geom_') and e.model.geom_contype[i]]
 assert len(robot)==9
 for i in robot:
  for j in robot:
   assert not (e.model.geom_contype[i]&e.model.geom_conaffinity[j] or e.model.geom_contype[j]&e.model.geom_conaffinity[i])
  assert e.model.geom_conaffinity[i]&1 # plant contact retained
