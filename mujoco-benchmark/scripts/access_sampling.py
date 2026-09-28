"""Paired low-tilt proposal schedule; no physics or broad simultaneous jitter.

Pass each returned `parameters` to the EXISTING motion_family_search.make_candidate.
Tier 0 changes heading only. Tier 1 varies one tilt/elevation factor around one
selected parent, under an explicit count. Does not choose from test scenes.
"""
from __future__ import annotations
import copy
import math


def low_tilt_schedule():
    from motion_family_search import RANGES
    base={k:float((lo+hi)/2) for k,(lo,hi) in RANGES.items()}
    base.update(roll_deg=0.,pitch_deg=0.,elevation_deg=0.,entry_twist_deg=0.,
                target_selector=.25,wire_selector=.5)
    return [dict(tier=0,changed_factor='azimuth_deg',parameters=dict(base,azimuth_deg=float(a)))
            for a in (-45.,0.,45.)]


def one_factor_tilts(parent):
    from motion_family_search import RANGES
    rows=[]
    for factor in ('roll_deg','pitch_deg','elevation_deg','entry_twist_deg'):
        for sign in (-1.,1.):
            p=copy.deepcopy(parent);value=p[factor]+sign*15.
            if not math.isfinite(value) or not RANGES[factor][0]<=value<=RANGES[factor][1]:continue
            p[factor]=value
            rows.append(dict(tier=1,changed_factor=factor,parameters=p))
    return rows
