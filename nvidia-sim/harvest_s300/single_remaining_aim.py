"""Optional robot aim for the last attached fruit of the selected exact truss.

This changes only existing waypoint-planner parameters. Native joint inventory,
source fruit positions, gripper travel, physics and attempt limits are untouched.
"""
import math


def selected_aim(fruits, breaks, group, *, height_mm, depth_mm,
                 last_height_mm=None, last_depth_mm=None):
    from row_harvest_targets import truss_key
    from truss_sequence import inventory
    for value in (height_mm, last_height_mm):
        if value is not None and (isinstance(value, bool) or
                not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > 50):
            raise ValueError('Existing finite aim height range is +/-50mm')
    for value in (depth_mm, last_depth_mm):
        if value is not None and (isinstance(value, bool) or
                not isinstance(value, (int, float)) or not math.isfinite(value) or not 15 <= value <= 100):
            raise ValueError('Existing finite aim depth range is15..100mm')
    if height_mm is None or depth_mm is None:
        raise ValueError('Explicit baseline height and depth required')
    selected = [] if group is None else group['fruits']
    names = {fruit['name'] for fruit in fruits}
    if selected and (len(set(selected)) != len(selected) or not set(selected) <= names):
        raise ValueError('Exact existing selected fruit identities required')
    keys = {truss_key(name) for name in selected}
    if len(keys) > 1:
        raise ValueError('A last-fruit aim cannot cross trusses')
    key = next(iter(keys), None)
    subset = [fruit for fruit in fruits if truss_key(fruit['name']) == key]
    remaining = inventory(subset, breaks)['remaining']
    active = len(selected) == 1 and remaining == selected
    effective_height = last_height_mm if active and last_height_mm is not None else height_mm
    effective_depth = last_depth_mm if active and last_depth_mm is not None else depth_mm
    return float(effective_height), float(effective_depth), dict(
        selected_truss_key=key, remaining_exact_truss_fruits=remaining,
        selected_fruits=list(selected), last_remaining_fruit=active,
        baseline_height_mm=float(height_mm), baseline_depth_mm=float(depth_mm),
        last_height_mm=last_height_mm, last_depth_mm=last_depth_mm,
        effective_height_mm=float(effective_height), effective_depth_mm=float(effective_depth),
        source='Measured native joint inventory; absolute robot aim only',
        physics_unchanged=True, native_detachment_required=True, attempt_budget_unchanged=True)
