"""Shared CAD linkage geometry; pure NumPy/SciPy, no simulator imports."""
import numpy as np
from scipy.spatial.transform import Rotation as R


def transmission_transforms(gap):
    """CAD shape_at linkage equations, driven by measured slider gap (metres)."""
    if not .0279<=gap<=.0961:raise ValueError('Measured gap exceeds CAD mechanism travel')
    def theta(g):
        s=g/2+.018
        return np.arccos(np.clip((s*s+.025**2-.045**2)/(2*s*.025),-1,1))
    angle=theta(gap);nominal=theta(.044);pivot=np.array([0.,-.031,-.018])
    out={};t=np.eye(4);t[:3,:3]=R.from_rotvec([0,-(angle-nominal),0]).as_matrix()
    t[:3,3]=pivot-t[:3,:3]@pivot;out['crank']=t
    for side,sign in [('linkL',-1),('linkR',1)]:
        def ends(g,a):
            return np.array([sign*.025*np.cos(a),-.031,-.018+sign*.025*np.sin(a)]),np.array([sign*(g/2+.018),-.031,-.018])
        a0,b0=ends(.044,nominal);a,b=ends(gap,angle)
        change=np.arctan2((b-a)[2],(b-a)[0])-np.arctan2((b0-a0)[2],(b0-a0)[0])
        t=np.eye(4);t[:3,:3]=R.from_rotvec([0,-change,0]).as_matrix();t[:3,3]=a-t[:3,:3]@a0
        out[side]=t
    return out

