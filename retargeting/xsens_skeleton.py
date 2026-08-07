"""Dependency-free geometry helpers for displaying the Xsens body skeleton."""

from __future__ import annotations

import math
from typing import Sequence


SEGMENT_EDGES = (
    ("Pelvis", "L5"),
    ("L5", "L3"),
    ("L3", "T12"),
    ("T12", "T8"),
    ("T8", "Neck"),
    ("Neck", "Head"),
    ("T8", "RightShoulder"),
    ("RightShoulder", "RightUpperArm"),
    ("RightUpperArm", "RightForeArm"),
    ("RightForeArm", "RightHand"),
    ("T8", "LeftShoulder"),
    ("LeftShoulder", "LeftUpperArm"),
    ("LeftUpperArm", "LeftForeArm"),
    ("LeftForeArm", "LeftHand"),
    ("Pelvis", "RightUpperLeg"),
    ("RightUpperLeg", "RightLowerLeg"),
    ("RightLowerLeg", "RightFoot"),
    ("RightFoot", "RightToe"),
    ("Pelvis", "LeftUpperLeg"),
    ("LeftUpperLeg", "LeftLowerLeg"),
    ("LeftLowerLeg", "LeftFoot"),
    ("LeftFoot", "LeftToe"),
)

IK_SOURCE_SEGMENTS = frozenset({"Pelvis", "T8", "LeftHand", "RightHand", "LeftFoot", "RightFoot"})


def _inverse_rotate_wxyz(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a world vector into the local frame of a wxyz quaternion."""
    if len(quaternion) != 4 or len(vector) != 3:
        raise ValueError("expected a four-value quaternion and three-value vector")
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    w, x, y, z = w / norm, -x / norm, -y / norm, -z / norm
    vx, vy, vz = (float(value) for value in vector)
    dot = x * vx + y * vy + z * vz
    cross_x = y * vz - z * vy
    cross_y = z * vx - x * vz
    cross_z = x * vy - y * vx
    scale = w * w - x * x - y * y - z * z
    return (
        2.0 * dot * x + scale * vx + 2.0 * w * cross_x,
        2.0 * dot * y + scale * vy + 2.0 * w * cross_y,
        2.0 * dot * z + scale * vz + 2.0 * w * cross_z,
    )


def display_positions(
    segment_positions: Sequence[Sequence[float]],
    initial_pelvis_position: Sequence[float],
    initial_pelvis_quaternion_wxyz: Sequence[float],
    display_origin: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Express Xsens world positions in the initial pelvis frame for display."""
    if len(initial_pelvis_position) != 3 or len(display_origin) != 3:
        raise ValueError("pelvis position and display origin must have three values")
    origin = tuple(float(value) for value in display_origin)
    pelvis = tuple(float(value) for value in initial_pelvis_position)
    output = []
    for position in segment_positions:
        if len(position) != 3:
            raise ValueError("segment position must have three values")
        delta = tuple(float(position[index]) - pelvis[index] for index in range(3))
        local = _inverse_rotate_wxyz(initial_pelvis_quaternion_wxyz, delta)
        output.append(tuple(origin[index] + local[index] for index in range(3)))
    return tuple(output)


def edge_indices(segment_names: Sequence[str]) -> tuple[tuple[int, int], ...]:
    """Map the standard Xsens body tree to indexes in an MVNX segment array."""
    indexes = {name: index for index, name in enumerate(segment_names)}
    missing = sorted({name for edge in SEGMENT_EDGES for name in edge} - indexes.keys())
    if missing:
        raise ValueError(f"Xsens skeleton is missing segments: {', '.join(missing)}")
    return tuple((indexes[parent], indexes[child]) for parent, child in SEGMENT_EDGES)
