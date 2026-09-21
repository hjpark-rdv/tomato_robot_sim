"""Bounded startup optimization for explicit USD asset paths.

Isaac Lab's regex helper scans the entire stage even when given an exact path.
Keep regex behavior, default traversal predicates, and all physics unchanged.
The override exists only while our single-threaded scene constructor runs.
"""
from contextlib import contextmanager
import re

_LITERAL = re.compile(r'(?:/[A-Za-z_][A-Za-z_0-9]*)+')


def explicit_path_lookup(path, stage, fallback):
    if not _LITERAL.fullmatch(path):
        return fallback(path,stage)
    prim=stage.GetPrimAtPath(path)
    if prim and prim.IsActive() and prim.IsDefined() and prim.IsLoaded() and not prim.IsAbstract() and not prim.IsInstanceProxy():
        return prim
    return None


def explicit_path_matches(path,stage,fallback):
    if not _LITERAL.fullmatch(path):
        return fallback(path,stage)
    prim=stage.GetPrimAtPath(path)
    # find_matching_prims uses GetAllChildren (including inactive/abstract
    # prims), unlike find_first_matching_prim's default traversal predicate.
    return [prim] if prim and not prim.IsInstanceProxy() else []


@contextmanager
def fast_explicit_asset_lookup():
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    original=sim_utils.find_first_matching_prim
    original_matches=sim_utils.find_matching_prims
    def lookup(path,stage=None):
        return explicit_path_lookup(path,get_current_stage() if stage is None else stage,original)
    def matches(path,stage=None):
        return explicit_path_matches(path,get_current_stage() if stage is None else stage,original_matches)
    sim_utils.find_first_matching_prim=lookup
    sim_utils.find_matching_prims=matches
    try:
        yield
    finally:
        sim_utils.find_first_matching_prim=original
        sim_utils.find_matching_prims=original_matches
