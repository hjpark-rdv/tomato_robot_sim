import math
from typing import Iterable

import numpy as np


def quaternion_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.array(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            float((matrix[2, 1] - matrix[1, 2]) / s),
            float((matrix[0, 2] - matrix[2, 0]) / s),
            float((matrix[1, 0] - matrix[0, 1]) / s),
            float(0.25 * s),
        )

    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return (
            float(0.25 * s),
            float((matrix[0, 1] + matrix[1, 0]) / s),
            float((matrix[0, 2] + matrix[2, 0]) / s),
            float((matrix[2, 1] - matrix[1, 2]) / s),
        )
    if matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return (
            float((matrix[0, 1] + matrix[1, 0]) / s),
            float(0.25 * s),
            float((matrix[1, 2] + matrix[2, 1]) / s),
            float((matrix[0, 2] - matrix[2, 0]) / s),
        )

    s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return (
        float((matrix[0, 2] + matrix[2, 0]) / s),
        float((matrix[1, 2] + matrix[2, 1]) / s),
        float(0.25 * s),
        float((matrix[1, 0] - matrix[0, 1]) / s),
    )


def orientation_from_z_axis(z_axis: Iterable[float]) -> tuple[float, float, float, float]:
    z = np.array(tuple(z_axis), dtype=float)
    z /= np.linalg.norm(z)
    x_hint = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(z, x_hint))) > 0.95:
        x_hint = np.array([0.0, 1.0, 0.0])
    y = np.cross(z, x_hint)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return quaternion_from_matrix(np.column_stack((x, y, z)))


def target_stem_axis(object_position: Iterable[float]) -> np.ndarray:
    tomato_pos = np.array(tuple(object_position), dtype=float)
    robot_direction_xy = -tomato_pos[:2]
    norm = np.linalg.norm(robot_direction_xy)
    if norm < 1e-6:
        robot_direction_xy = np.array([-1.0, 0.0])
    else:
        robot_direction_xy /= norm

    calyx_up_angle = math.radians(45.0)
    stem_axis = np.array(
        [
            robot_direction_xy[0] * math.cos(calyx_up_angle),
            robot_direction_xy[1] * math.cos(calyx_up_angle),
            math.sin(calyx_up_angle),
        ],
        dtype=float,
    )
    stem_axis /= np.linalg.norm(stem_axis)
    return stem_axis
