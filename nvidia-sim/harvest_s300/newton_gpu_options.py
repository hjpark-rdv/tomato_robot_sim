"""Small explicit GPU option overrides after the standard model upload."""
from __future__ import annotations

import math


def solver_tolerance_value(value):
    if value is None:
        return None
    if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0:
        raise ValueError('Solver tolerance must be finite and positive')
    return float(value)


def apply_explicit_solver_tolerance(options, requested):
    """Honor an opt-in value that put_model otherwise clamps to its floor.

    None leaves the existing upload and its numerical defaults untouched.
    This runs before graph capture and changes no integration state.
    """
    value = solver_tolerance_value(requested)
    if value is not None:
        options.tolerance.fill_(value)
    return value
