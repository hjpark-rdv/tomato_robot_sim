"""Target-centred free camera; mouse orbit/pan/zoom remain enabled."""
import mujoco as mj
import numpy as np


def target_camera(model,data,camera):
    mj.mj_forward(model,data)
    fruit=model.body('Tomato_05').id
    camera.type=mj.mjtCamera.mjCAMERA_FREE
    camera.fixedcamid=-1
    camera.lookat[:]=data.xpos[fruit]+np.array([0,0,.015])
    camera.distance=.32
    camera.azimuth=-45
    camera.elevation=-15
