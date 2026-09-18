"""CAD-derived tool geometry and task constants (metres, radians, kilograms)."""

import math

# Ver.6 distal half-ring, measured from assy_gripper_ver_6.stl, gripper frame.
RING_CENTER = (-0.10640287, 0.00616804, 0.00006305)
RING_RADIUS = 0.0275
WIRE_RADIUS = 0.001
ARC_SEGMENTS = 32
FRUIT_RADIUS = 0.0115
STEM_RADIUS = 0.0015
STEM_LENGTH = 0.030
FRUIT_MASS = 0.008  # Initial cherry-tomato approximation; calibrate experimentally.
BREAK_FORCE = 3.0
BREAK_TORQUE = 0.08


def arc_points():
    """Preserve the CAD half-ring; do not close its open side."""
    return [
        (RING_CENTER[0] + RING_RADIUS * math.cos(a), RING_CENTER[1],
         RING_CENTER[2] + RING_RADIUS * math.sin(a))
        for a in [math.pi / 2 + math.pi * i / ARC_SEGMENTS for i in range(ARC_SEGMENTS + 1)]
    ]
