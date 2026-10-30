"""Steering-arm construction port; these intersections are not vehicle ICRs.

See suspension-studio/docs/STEERING.md for the derivation and verified source
passages. The spatial included angle uses the perpendicular axis foot C; the
plan angle and infinite construction lines use the point K on the axis at the
outer ball-joint height (Milliken Figure 19.3, printed p.714 / PDF p.377).
Both are measured at the outer joint. All positions are SAE J670 axle-local mm.
"""

from __future__ import annotations

import math
import sys

from .steering import _cross, _dot, _finite, _norm, _record, _sub

DEG = 180 / math.pi
LENGTH_RELATIVE_TOLERANCE = 1e-12
PARALLEL_RELATIVE_TOLERANCE = 1e-10
AXIS_VERTICAL_COMPONENT_TOLERANCE = 1e-10
AXIS_RADIUS_ROUNDOFF_FACTOR = 32
STEERING_CONSTRUCTION_NUMERICS = {
    "lengthRelativeTolerance": LENGTH_RELATIVE_TOLERANCE,
    "parallelRelativeTolerance": PARALLEL_RELATIVE_TOLERANCE,
    "axisVerticalComponentTolerance": AXIS_VERTICAL_COMPONENT_TOLERANCE,
    "axisRadiusRoundoffFactor": AXIS_RADIUS_ROUNDOFF_FACTOR,
}


def _get(record, key):
    return record.get(key) if _record(record) else None


def _point(value):
    return isinstance(value, list) and len(value) == 3 and all(_finite(item) for item in value)


def _copy_point(value):
    # Match the JS binary64 number domain even for Python integer inputs.
    return [float(item) for item in value] if _point(value) else None


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _angle_between(a, b):
    length_a, length_b = _norm(a), _norm(b)
    if not _finite(length_a) or not _finite(length_b) or length_a == 0 or length_b == 0:
        return None
    unit_a, unit_b = [value / length_a for value in a], [value / length_b for value in b]
    sine = abs(_cross2(unit_a, unit_b)) if len(a) == 2 else _norm(_cross(unit_a, unit_b))
    angle = math.atan2(sine, _dot(unit_a, unit_b)) * DEG
    return angle if _finite(angle) else None


def _construction_side(design_side, solved_side, use_solved):
    outer = _get(solved_side, "outer") if use_solved else _get(design_side, "tieOuter")
    inner = _get(solved_side, "inner") if use_solved else _get(design_side, "tieInner")
    result = {
        "axisFoot": None, "planAxisPoint": None,
        "outer": _copy_point(outer), "inner": _copy_point(inner),
        "spatialAngleDeg": None, "planAngleDeg": None,
    }
    upper = _copy_point(_get(design_side, "axisUpper"))
    lower = _copy_point(_get(design_side, "axisLower"))
    if upper is None or lower is None or result["outer"] is None:
        return result
    axis = _sub(lower, upper)
    axis_length = _norm(axis)
    if not _finite(axis_length) or axis_length == 0:
        return result
    k = [value / axis_length for value in axis]
    from_upper, from_lower = _sub(result["outer"], upper), _sub(result["outer"], lower)
    use_upper = _norm(from_upper) <= _norm(from_lower)
    anchor, r = (upper, from_upper) if use_upper else (lower, from_lower)
    # Equivalent perpendicular projection k×(r×k), using the nearer endpoint.
    # Its cancellation guard follows component product sums, not axis span.
    radial_cross = _cross(r, k)
    radial = _cross(k, radial_cross)
    radius = _norm(radial)
    cross_product_scale = math.hypot(
        abs(r[1] * k[2]) + abs(r[2] * k[1]),
        abs(r[2] * k[0]) + abs(r[0] * k[2]),
        abs(r[0] * k[1]) + abs(r[1] * k[0]),
    )
    radius_roundoff = AXIS_RADIUS_ROUNDOFF_FACTOR * sys.float_info.epsilon * cross_product_scale
    foot = [value - radial[i] for i, value in enumerate(result["outer"])]
    if not _point(foot):
        return result
    result["axisFoot"] = foot
    if not _finite(radius) or not _finite(radius_roundoff) or radius <= radius_roundoff or _norm(_sub(foot, result["outer"])) == 0:
        return result
    if abs(k[2]) > AXIS_VERTICAL_COMPONENT_TOLERANCE:
        height_parameter = (result["outer"][2] - anchor[2]) / k[2]
        at_height = [anchor[0] + k[0] * height_parameter, anchor[1] + k[1] * height_parameter, result["outer"][2]]
        if _point(at_height):
            result["planAxisPoint"] = at_height
    if result["inner"] is None:
        return result
    arm, tie = _sub(foot, result["outer"]), _sub(result["inner"], result["outer"])
    arm_length, tie_length = _norm(arm), _norm(tie)
    local_scale = max(arm_length, tie_length)
    if not _finite(local_scale) or arm_length <= LENGTH_RELATIVE_TOLERANCE * local_scale or tie_length <= LENGTH_RELATIVE_TOLERANCE * local_scale:
        return result
    result["spatialAngleDeg"] = _angle_between(arm, tie)
    if result["planAxisPoint"] is None:
        return result
    plan_arm = _sub(result["planAxisPoint"], result["outer"])[:2]
    arm_plan_length, tie_plan_length = _norm(plan_arm), math.hypot(tie[0], tie[1])
    if not _finite(arm_plan_length) or arm_plan_length <= LENGTH_RELATIVE_TOLERANCE * local_scale or tie_plan_length <= LENGTH_RELATIVE_TOLERANCE * tie_length:
        return result
    result["planAngleDeg"] = _angle_between(plan_arm, tie[:2])
    return result


def _arm_line(side):
    if side["planAxisPoint"] is None or side["outer"] is None:
        return None
    arm = _sub(side["outer"], side["planAxisPoint"])
    spatial_length, plan_length = _norm(arm), math.hypot(arm[0], arm[1])
    if not _finite(spatial_length) or spatial_length == 0 or not _finite(plan_length) or plan_length <= LENGTH_RELATIVE_TOLERANCE * spatial_length:
        return None
    return {"origin": side["planAxisPoint"][:2], "direction": [arm[0] / plan_length, arm[1] / plan_length], "length": plan_length}


def _intersect_arm_lines(left, right, wheelbase):
    def unavailable(status):
        return {"status": status, "point": None, "rearAxleOffsetMm": None}

    a, b = _arm_line(left), _arm_line(right)
    if a is None or b is None:
        return unavailable("undefined")
    between = _sub(b["origin"], a["origin"])
    separation = _norm(between)
    if not _finite(separation):
        return unavailable("undefined")
    denominator = _cross2(a["direction"], b["direction"])
    if not _finite(denominator):
        return unavailable("undefined")
    if abs(denominator) <= PARALLEL_RELATIVE_TOLERANCE:
        perpendicular_separation = abs(_cross2(between, a["direction"]))
        position_scale = max(a["length"], b["length"], separation)
        return unavailable("coincident" if perpendicular_separation <= PARALLEL_RELATIVE_TOLERANCE * position_scale else "parallel")
    distance = _cross2(between, b["direction"]) / denominator
    intersection = [a["origin"][0] + distance * a["direction"][0], a["origin"][1] + distance * a["direction"][1], 0]
    if not _point(intersection):
        return unavailable("undefined")
    rear_offset = intersection[0] + float(wheelbase) if _finite(wheelbase) else None
    if rear_offset is not None and not _finite(rear_offset):
        return unavailable("undefined")
    return {"status": "finite", "point": intersection, "rearAxleOffsetMm": rear_offset}


def steering_construction(design: object, result: object = None) -> dict:
    """Return included angles and infinite plan-arm-line intersection.

    Only result.ok=True selects solved joints. Failed poses fall back to the
    entered static assembly; malformed successful poses remain unavailable.
    Unresolved values are null, never inferred, NaN, or Infinity. There is no
    tire-slip, real-arm-shape, joint-articulation or wheel-normal ICR inference.
    """
    use_solved = _get(result, "ok") is True
    left = _construction_side(_get(design, "left"), _get(result, "left"), use_solved)
    right = _construction_side(_get(design, "right"), _get(result, "right"), use_solved)
    return {"left": left, "right": right,
            "intersection": _intersect_arm_lines(left, right, _get(design, "wheelbase")),
            "pose": "solved" if use_solved else "static"}
