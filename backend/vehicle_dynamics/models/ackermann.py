"""Port of the existing ideal Ackermann benchmark, not the linkage solver.

Derivation, source evidence, units, signs and limits are recorded in
docs/ACKERMANN_BACKEND.md. Positive headings turn right about SAE +Z.
"""

from math import isfinite
from typing import TypedDict

import numpy as np


MIN_LENGTH_MM = 1e-6
MAX_LENGTH_MM = 1e7
MAX_ABS_INSIDE_ANGLE_DEG = 89.0
MAX_SAMPLES = 721


class AckermannSample(TypedDict):
    inside_angle_deg: float
    outside_angle_deg: float
    left_heading_deg: float
    right_heading_deg: float
    turn_direction: str


def _finite_number(value: object, label: str) -> float:
    # JSON numbers only; bool is an int subclass and must not become a dimension.
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be a finite number without coercion.")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{label} must be a finite number.") from exc
    if not isfinite(number):
        raise ValueError(f"{label} must be a finite number.")
    return number


def calculate_ackermann(
    wheelbase_mm: float,
    front_track_mm: float,
    inside_angles_deg: list[float],
) -> list[AckermannSample]:
    """Return ideal signed wheel headings in degrees, preserving sample order.

    Invalid inputs raise ValueError. Input dimensions are positive millimeters,
    headings are positive rightward, and no vehicle measurements are inferred.
    """
    wheelbase = _finite_number(wheelbase_mm, "wheelbase_mm")
    track = _finite_number(front_track_mm, "front_track_mm")
    for label, value in (("wheelbase_mm", wheelbase), ("front_track_mm", track)):
        if not MIN_LENGTH_MM <= value <= MAX_LENGTH_MM:
            raise ValueError(f"{label} must be at least {MIN_LENGTH_MM:g} and at most {MAX_LENGTH_MM:g} mm.")
    if not isinstance(inside_angles_deg, list) or not 1 <= len(inside_angles_deg) <= MAX_SAMPLES:
        raise ValueError(f"inside_angles_deg must be a list of 1 through {MAX_SAMPLES} angles.")
    angles = []
    for index, value in enumerate(inside_angles_deg):
        angle = _finite_number(value, f"inside_angles_deg[{index}]")
        if abs(angle) >= MAX_ABS_INSIDE_ANGLE_DEG:
            raise ValueError(f"inside_angles_deg[{index}] must be strictly between -89 and 89 degrees.")
        angles.append(angle)

    magnitudes = np.abs(np.asarray(angles, dtype=np.float64))
    radians = np.deg2rad(magnitudes)
    sine, cosine = np.sin(radians), np.cos(radians)
    outside_magnitudes = np.rad2deg(np.arctan2(sine, cosine + (track / wheelbase) * sine))
    # Below the normal radian range, conversion loses relative precision or
    # becomes zero. With t/L <= 1e13, the physical angle difference is then far
    # below representable degree precision, so retain the input magnitude.
    subnormal_radians = radians < np.finfo(np.float64).tiny
    outside_magnitudes = np.where(subnormal_radians, magnitudes, outside_magnitudes)

    samples: list[AckermannSample] = []
    for inside, outside_magnitude in zip(angles, outside_magnitudes):
        if inside == 0:
            inside = outside = left = right = 0.0
            turn = "straight"
        else:
            outside = float(outside_magnitude) * (1 if inside > 0 else -1)
            if outside == 0:
                outside = 0.0
            if inside > 0:
                left, right, turn = outside, inside, "right"
            else:
                left, right, turn = inside, outside, "left"
        samples.append({
            "inside_angle_deg": inside,
            "outside_angle_deg": outside,
            "left_heading_deg": left,
            "right_heading_deg": right,
            "turn_direction": turn,
        })
    return samples
