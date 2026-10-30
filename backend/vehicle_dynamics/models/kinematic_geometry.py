"""Pure SAE Z-down kinematic constructions, with explicit degeneracy reporting.

Derivations/source comparison: suspension-studio/docs/KINEMATICS-REFERENCE-REVIEW.md.
Coordinates are mm; angular rates are rad per the caller's tangent parameter.
These functions do not infer body motion, tire forces or undeclared contact points.
"""

import math
import numpy as np

_DOWN = np.array([0.0, 0.0, 1.0])


def _vector(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain three finite components.")
    return array


def _norm(vector):
    return float(np.hypot.reduce(vector))


def _unit(vector, name):
    length = _norm(vector)
    if not math.isfinite(length) or length == 0:
        raise ValueError(f"{name} must have finite nonzero length.")
    return vector / length


def _unavailable(reason, **metadata):
    return {"available": False, "reason": reason, **metadata}


def circle_contact(center, spindle, radius):
    """Lowest point on an ideal circular wheel center plane (not a tire patch)."""
    try:
        c = _vector(center, "Wheel center")
        n = _unit(_vector(spindle, "Spindle"), "Spindle")
        radius = float(radius)
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("Tire radius must be positive and finite in mm.")
        projection = _DOWN - n[2] * n
        length = _norm(projection)
        if length <= 1e-12:
            return _unavailable("A horizontal wheel circle has no unique lowest point.")
        point = c + radius * projection / length
        if not np.all(np.isfinite(point)):
            return _unavailable("Circular support point exceeds finite numerical range.")
        return {"available": True, "point": point.tolist(),
                "definition": "ideal-rigid-circular-center-plane-support",
                "radiusMm": radius}
    except (TypeError, ValueError, OverflowError) as error:
        return _unavailable(str(error))


def instantaneous_screw_axis(center, linear, angular):
    """Axis of the spatial twist v(P)=linear+angular cross (P-center).

    Nonzero screw pitch is retained. No finite axis is invented for translation.
    Axis direction changes sign if the tangent parameter is reversed; its line
    and pitch do not. All values are plain JSON-compatible scalars/lists.
    """
    try:
        c = _vector(center, "Twist reference point")
        velocity = _vector(linear, "Linear velocity")
        omega = _vector(angular, "Angular velocity")
        magnitude = _norm(omega)
        if magnitude == 0:
            return _unavailable("Pure translation or zero motion has no unique finite instantaneous screw axis.",
                                angularMagnitude=0.0, linearMagnitude=_norm(velocity))
        direction = omega / magnitude
        offset = np.cross(direction, velocity) / magnitude
        pitch = float(np.dot(direction, velocity) / magnitude)
        parallel = float(np.dot(direction, velocity)) * direction
        point = c + offset
        if not np.all(np.isfinite(point)) or not math.isfinite(pitch):
            return _unavailable("Instantaneous axis exceeds finite numerical range.")
        return {"available": True, "point": point.tolist(),
                "direction": direction.tolist(), "pitchMmPerRad": pitch,
                "parallelVelocity": parallel.tolist(), "angularMagnitude": magnitude,
                "construction": "spatial-instantaneous-screw-axis",
                "interpretation": "An axis of zero point velocity only when screw pitch is zero."}
    except (TypeError, ValueError, OverflowError) as error:
        return _unavailable(str(error))


def axis_plane_intersection(axis, plane_point, plane_normal, *, parallel_tolerance=1e-10):
    """Intersection of a declared finite axis and plane, including parallel failure."""
    if not isinstance(axis, dict) or not axis.get("available"):
        return _unavailable("A finite instantaneous axis is unavailable.")
    try:
        point = _vector(axis["point"], "Axis point")
        direction = _unit(_vector(axis["direction"], "Axis direction"), "Axis direction")
        reference = _vector(plane_point, "Plane point")
        normal = _unit(_vector(plane_normal, "Plane normal"), "Plane normal")
        denominator = float(np.dot(normal, direction))
        if abs(denominator) <= parallel_tolerance:
            return _unavailable("Axis is parallel to or lies in the reference plane; no unique intersection.")
        distance = float(np.dot(normal, reference - point) / denominator)
        result = point + distance * direction
        if not np.all(np.isfinite(result)):
            return _unavailable("Instant-center intersection exceeds finite numerical range.")
        return {"available": True, "point": result.tolist(),
                "planeResidualMm": abs(float(np.dot(normal, result - reference))),
                "axisPlaneCosine": abs(denominator),
                "construction": "instantaneous-axis-plane-intersection"}
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        return _unavailable(str(error))


def front_view_instant_center(axis, wheel_center):
    """Axis intersection with the transverse plane X=wheel-center X."""
    return axis_plane_intersection(axis, wheel_center, [1.0, 0.0, 0.0])


def side_view_instant_center(axis, wheel_center):
    """Axis intersection with the longitudinal plane Y=wheel-center Y."""
    return axis_plane_intersection(axis, wheel_center, [0.0, 1.0, 0.0])


def _contact_plane(axis, contact):
    if not isinstance(axis, dict) or not axis.get("available"):
        reason = axis.get("reason", "") if isinstance(axis, dict) else ""
        raise ValueError(f"Finite corner instantaneous axis unavailable. {reason}".strip())
    point = _vector(axis["point"], "Axis point")
    direction = _unit(_vector(axis["direction"], "Axis direction"), "Axis direction")
    delta = contact - point
    normal = np.cross(direction, delta)
    length = _norm(normal)
    if length <= 1e-10 * max(1.0, _norm(delta)):
        raise ValueError("Support point lies on or too near the instantaneous axis; its contact plane is undefined.")
    return normal / length


def _kinematic_center(axis_a, contact_a, axis_b, contact_b, construction,
                      *, condition_limit=1e10):
    try:
        ca = _vector(contact_a, "First support point")
        cb = _vector(contact_b, "Second support point")
        normal_a = _contact_plane(axis_a, ca)
        normal_b = _contact_plane(axis_b, cb)
        chord = cb - ca
        vertical_normal = np.cross(chord, _DOWN)
        if _norm(vertical_normal) <= 1e-10 * max(1.0, _norm(chord)):
            return _unavailable("Support points do not define a unique vertical reference plane.")
        vertical_normal = _unit(vertical_normal, "Vertical-plane normal")
        matrix = np.array([normal_a, normal_b, vertical_normal])
        # Use a nearby origin so a common large global translation does not
        # magnify cancellation in plane right-hand sides.
        origin = ca + (cb - ca) / 2.0
        rhs = np.array([np.dot(normal_a, ca - origin),
                        np.dot(normal_b, cb - origin),
                        np.dot(vertical_normal, ca - origin)])
        singular = np.linalg.svd(matrix, compute_uv=False)
        condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else math.inf
        if not math.isfinite(condition) or condition > condition_limit:
            return _unavailable("The three construction planes are dependent or their intersection is ill-conditioned.",
                                conditionNumber=condition if math.isfinite(condition) else None)
        relative = np.linalg.solve(matrix, rhs)
        point = origin + relative
        residual = float(np.max(np.abs(matrix @ relative - rhs)))
        if not np.all(np.isfinite(point)):
            return _unavailable("Kinematic center exceeds finite numerical range.")
        return {"available": True, "point": point.tolist(),
                "planeResidualMm": residual, "conditionNumber": condition,
                "construction": construction,
                "screwPitchesMmPerRad": [axis_a.get("pitchMmPerRad"), axis_b.get("pitchMmPerRad")],
                "interpretation": "Geometric instantaneous-axis/support-plane construction; not a force-based center or proof of actual body rotation."}
    except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as error:
        return _unavailable(str(error))


def kinematic_roll_center(left_axis, left_contact, right_axis, right_contact, **kwargs):
    """Three-plane center at one axle, with caller-supplied support points."""
    return _kinematic_center(left_axis, left_contact, right_axis, right_contact,
                             "kinematic-roll-center-three-plane", **kwargs)


def kinematic_pitch_center(front_axis, front_contact, rear_axis, rear_contact, **kwargs):
    """Three-plane center at one vehicle side, with caller-supplied support points."""
    return _kinematic_center(front_axis, front_contact, rear_axis, rear_contact,
                             "kinematic-pitch-center-three-plane", **kwargs)


def steering_geometry(lower, upper, contact, spindle, side):
    """Physical/declared steering line geometry at a circular support-point height.

    Caller must identify a real physical line or explicitly label a virtual one.
    Positive caster=top rearward, KPI=top inboard; trail forward, scrub outward.
    """
    try:
        lo = _vector(lower, "Lower steering-axis point")
        hi = _vector(upper, "Upper steering-axis point")
        q = _vector(contact, "Support point")
        n = _unit(_vector(spindle, "Spindle"), "Spindle")
        if side not in ("left", "right"):
            raise ValueError("Side must be left or right.")
        sign = -1.0 if side == "left" else 1.0
        direction = _unit(hi - lo, "Steering axis")
        # The line is unoriented geometrically; normalize its sense toward up.
        if direction[2] > 0:
            direction = -direction
        angles = {"casterDeg": math.degrees(math.atan2(-direction[0], -direction[2])),
                  "kingpinInclinationDeg": math.degrees(math.atan2(-sign * direction[1], -direction[2]))}
        if abs(direction[2]) <= 1e-10:
            return _unavailable("Steering axis is horizontal; no unique support-height intersection.", **angles)
        forward = sign * np.cross(n, _DOWN)
        if _norm(forward) <= 1e-12:
            return _unavailable("Wheel heading is undefined for a horizontal wheel circle.", **angles)
        forward = _unit(forward, "Wheel heading")
        outward = sign * np.cross(_DOWN, forward)
        intersection = lo + ((q[2] - lo[2]) / direction[2]) * direction
        if not np.all(np.isfinite(intersection)):
            return _unavailable("Steering intersection exceeds finite numerical range.", **angles)
        return {"available": True, **angles,
                "axisGroundPoint": intersection.tolist(),
                "mechanicalTrailMm": float(np.dot(intersection - q, forward)),
                "scrubRadiusMm": float(np.dot(q - intersection, outward)),
                "referencePlaneZMm": float(q[2]),
                "definition": "steering-line-at-circular-support-height"}
    except (TypeError, ValueError, OverflowError) as error:
        return _unavailable(str(error))
