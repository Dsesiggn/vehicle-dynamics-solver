"""Rigid suspension corner closure; see docs/KINEMATICS-SOLVER.md.

SAE +X forward, +Y right, +Z down, axle-local mm. This module does not
silently equate wheel-center bump with vehicle pitch/roll or infer a rocker
axis. Proper rotations and analytic tangent Jacobians enforce rigid geometry.
"""
from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np
from scipy.spatial.transform import Rotation

MIN_LENGTH = 1e-7
TARGET_MM = 1e-10
ACCEPT_MM = 1e-6
MAX_CONDITION = 1e10
MAX_ROTATION = math.pi / 2
SCALE_MM = 300.0
SIDES = {"left": -1, "right": 1}


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _point(value):
    return isinstance(value, (list, tuple)) and len(value) == 3 and all(_finite(x) for x in value)


def _failed(reason, error=None):
    return {"ok": False, "reason": reason, "error": error}


def _pairs(topology):
    tie = [("tie_rod_inner", "tie_rod_outer")]
    if topology == "multi-link":
        return [(name + "_inner", name + "_outer") for name in
                ("lower_front", "lower_rear", "upper_front", "upper_rear")] + tie
    lower = [("lca_front", "lca_outer"), ("lca_rear", "lca_outer")]
    if topology == "double-wishbone":
        return lower + [("uca_front", "uca_outer"), ("uca_rear", "uca_outer")] + tie
    return lower + tie


def _model(axle, side, static_alignment=None):
    if side not in SIDES:
        raise ValueError("Corner side must be left or right.")
    if not isinstance(axle, dict) or axle.get("topology") not in ("double-wishbone", "macpherson", "multi-link"):
        raise ValueError("Unsupported suspension topology.")
    raw = axle.get("hardpoints")
    if not isinstance(raw, dict) or not raw or any(not _point(p) for p in raw.values()):
        raise ValueError("Hardpoints must contain finite three-component axle-local millimeter vectors.")
    reflection = np.array([1.0, -1.0 if side == "right" else 1.0, 1.0])
    points = {key: np.asarray(value, dtype=float) * reflection for key, value in raw.items()}
    pairs = _pairs(axle["topology"])
    required = {"wheel_center", *(key for pair in pairs for key in pair)}
    if axle["topology"] == "macpherson":
        required.update(("strut_top", "strut_bottom"))
    if not required.issubset(points):
        raise ValueError("Missing suspension constraint hardpoints: " + ", ".join(sorted(required - points.keys())))
    lengths = [float(np.linalg.norm(points[outer] - points[inner])) for inner, outer in pairs]
    if any(length <= MIN_LENGTH for length in lengths):
        raise ValueError("A suspension link has coincident or unresolved endpoints.")
    normals = []
    if axle["topology"] == "macpherson":
        axis = points["strut_top"] - points["strut_bottom"]
        length = float(np.linalg.norm(axis))
        if length <= MIN_LENGTH:
            raise ValueError("Strut top and bottom coincide or are too close to resolve their line.")
        axis /= length
        helper = np.eye(3)[int(np.argmin(np.abs(axis)))]
        n1 = np.cross(axis, helper)
        n1 /= np.linalg.norm(n1)
        normals = [n1, np.cross(axis, n1)]
    if static_alignment is None:
        gamma = tau = 0.0
        alignment_reference = "nominal-zero-static-alignment"
    else:
        if not isinstance(static_alignment, dict) or any(not _finite(static_alignment.get(key)) or
                abs(static_alignment[key]) >= 90 for key in ("camberDeg", "toeInDeg")):
            raise ValueError("Static alignment requires finite camberDeg and toeInDeg strictly between -90 and +90 degrees.")
        gamma, tau = (math.radians(static_alignment[key]) for key in ("camberDeg", "toeInDeg"))
        alignment_reference = "entered-static-alignment"
    s = SIDES[side]
    delta = -s * tau
    spindle = np.array([-s * math.sin(delta) * math.cos(gamma),
                        s * math.cos(delta) * math.cos(gamma), math.sin(gamma)])
    attachment = axle.get("actuationAttachment", "upright")
    if attachment not in ("upright", "lower-wishbone", "upper-wishbone") or (
            attachment != "upright" and axle["topology"] != "double-wishbone"):
        raise ValueError("Unsupported actuation attachment for this topology.")
    return {"points": points, "pairs": pairs, "lengths": lengths, "normals": normals,
            "center": points["wheel_center"], "spindle": spindle, "side": side,
            "alignmentReference": alignment_reference, "attachment": attachment,
            "reflection": reflection, "topology": axle["topology"]}


def _constraints(model, center, rotation, bump, rack, last_constraint=None):
    points, c0 = model["points"], model["center"]
    residual, rows = [], []
    for (inner, outer), length in zip(model["pairs"], model["lengths"]):
        r = rotation @ (points[outer] - c0)
        anchor = points[inner] + (np.array([0.0, rack, 0.0]) if inner == "tie_rod_inner" else 0)
        d = center + r - anchor
        current = float(np.linalg.norm(d))
        if current <= MIN_LENGTH:
            raise ValueError("A trial suspension link collapsed to zero length.")
        e = d / current
        residual.append(current - length)
        rows.append(np.r_[e, np.cross(r, e)])
    for normal in model["normals"]:
        n = rotation @ normal
        bottom = center + rotation @ (points["strut_bottom"] - c0)
        residual.append(float((points["strut_top"] - bottom) @ n))
        rows.append(np.r_[-n, np.cross(n, points["strut_top"] - center)])
    if last_constraint is None:
        residual.append(float(center[2] - c0[2] + bump))
        rows.append(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    else:
        value, row = last_constraint(center, rotation, model["spindle"])
        if not math.isfinite(value) or np.shape(row) != (6,) or not np.isfinite(row).all():
            raise ValueError("The imposed wheel constraint returned nonfinite residuals or Jacobian.")
        residual.append(float(value))
        rows.append(row)
    return np.asarray(residual), np.asarray(rows)


def _condition(jacobian):
    scaled = jacobian.copy()
    scaled[:, 3:] /= SCALE_MM
    sv = np.linalg.svd(scaled, compute_uv=False)
    condition = float(sv[0] / sv[-1]) if sv[-1] > 0 else math.inf
    return scaled, condition


def _pose(model, bump, rack, center, rotation, last_constraint=None):
    for iteration in range(55):
        f, jacobian = _constraints(model, center, rotation, bump, rack, last_constraint)
        scaled, condition = _condition(jacobian)
        if not math.isfinite(condition) or condition > MAX_CONDITION:
            return _failed("Constraint Jacobian is singular or too ill-conditioned to identify a unique nearby upright pose.", float(max(abs(f))))
        error = float(max(abs(f)))
        if error <= TARGET_MM:
            break
        delta = np.linalg.solve(scaled, -f)
        accepted = False
        factor = 1.0
        for _ in range(14):
            trial_c = center + delta[:3] * factor
            trial_r = Rotation.from_rotvec(delta[3:] * factor / SCALE_MM).as_matrix() @ rotation
            tf, _ = _constraints(model, trial_c, trial_r, bump, rack, last_constraint)
            if np.linalg.norm(tf) < np.linalg.norm(f):
                center, rotation = trial_c, trial_r
                accepted = True
                break
            factor /= 2
        if not accepted:
            break
    f, jacobian = _constraints(model, center, rotation, bump, rack, last_constraint)
    _, condition = _condition(jacobian)
    error = float(max(abs(f)))
    rotation_vector = Rotation.from_matrix(rotation).as_rotvec()
    if error > ACCEPT_MM or not math.isfinite(condition) or condition > MAX_CONDITION or np.linalg.norm(rotation_vector) > MAX_ROTATION:
        return _failed("No sufficiently resolved nearby assembly solution; reduce motion or review hardpoints.", error)
    return {"ok": True, "center": center, "matrix": rotation, "error": error,
            "condition": condition, "iterations": iteration + 1, "jacobian": jacobian}


def _wishbone_pickup(model, moved):
    prefix = "lca" if model["attachment"] == "lower-wishbone" else "uca"
    points = model["points"]
    fore, aft = points[prefix + "_front"], points[prefix + "_rear"]
    ball0, ball1 = points[prefix + "_outer"], moved[prefix + "_outer"]
    axis = aft - fore
    if np.linalg.norm(axis) <= MIN_LENGTH:
        raise ValueError("Wishbone pivots cannot define a resolved actuator attachment axis.")
    axis /= np.linalg.norm(axis)
    r0, r1 = ball0 - fore, ball1 - fore
    v0, v1 = r0 - axis * (axis @ r0), r1 - axis * (axis @ r1)
    if min(np.linalg.norm(v0), np.linalg.norm(v1)) <= MIN_LENGTH:
        raise ValueError("Wishbone ball joint lies on its pivot axis; pickup rotation is undefined.")
    v0, v1 = v0 / np.linalg.norm(v0), v1 / np.linalg.norm(v1)
    angle = math.atan2(float(axis @ np.cross(v0, v1)), float(v0 @ v1))
    r = Rotation.from_rotvec(axis * angle).as_matrix()
    reconstruction = float(np.linalg.norm(fore + r @ (ball0 - fore) - ball1))
    if reconstruction > ACCEPT_MM * 10:
        raise ValueError("Wishbone ball does not reconstruct from a rigid rotation about its inboard pivots.")
    pickup = fore + r @ (points["actuation_outer"] - fore)
    return pickup, reconstruction


def _moved_points(model, pose, rack):
    p, c0 = model["points"], model["center"]
    moved = {}
    for key, value in p.items():
        moving = key.endswith("_outer") or key in ("wheel_center", "strut_bottom")
        if key == "actuation_outer" and model["attachment"] != "upright":
            moving = False
        moved[key] = pose["center"] + pose["matrix"] @ (value - c0) if moving else value.copy()
    moved["tie_rod_inner"][1] += rack
    attachment_error = 0.0
    if model["attachment"] != "upright" and "actuation_outer" in moved:
        moved["actuation_outer"], attachment_error = _wishbone_pickup(model, moved)
    return moved, attachment_error


def _actuator(axle, model, moved, rocker_axis, previous_angle=0.0):
    p = model["points"]
    unavailable = lambda reason: {"ok": False, "compression": None, "angleRad": None, "reason": reason}
    if axle["topology"] == "macpherson":
        compression = np.linalg.norm(p["strut_top"] - p["strut_bottom"]) - np.linalg.norm(moved["strut_top"] - moved["strut_bottom"])
        return {"ok": True, "compression": float(compression), "angleRad": None, "residualMm": 0.0}
    required = {"actuation_outer", "damper_top"}
    if not required.issubset(p):
        return unavailable("Actuator endpoint coordinates are incomplete.")
    if axle.get("actuation") == "direct":
        compression = np.linalg.norm(p["damper_top"] - p["actuation_outer"]) - np.linalg.norm(moved["damper_top"] - moved["actuation_outer"])
        return {"ok": True, "compression": float(compression), "angleRad": None, "residualMm": 0.0}
    if rocker_axis is None:
        return unavailable("An explicit rocker axis direction is required; no longitudinal axis is inferred.")
    if not _point(rocker_axis) or np.linalg.norm(rocker_axis) <= MIN_LENGTH:
        return unavailable("Rocker axis direction must be a finite nonzero three-component vector.")
    if not {"rocker_pivot", "rocker_rod", "rocker_damper"}.issubset(p):
        return unavailable("Rocker endpoint coordinates are incomplete.")
    k = np.asarray(rocker_axis, dtype=float) * model["reflection"]
    k /= np.linalg.norm(k)
    o = p["rocker_pivot"]
    rod = p["rocker_rod"] - o
    parallel = k * (k @ rod)
    v = rod - parallel
    w = np.cross(k, v)
    if np.linalg.norm(v) <= MIN_LENGTH:
        return unavailable("Rocker rod pickup lies on the rocker axis; its motion cannot close the actuator.")
    length = float(np.linalg.norm(p["rocker_rod"] - p["actuation_outer"]))
    d0 = o + parallel - p["actuation_outer"]
    neutral_derivative = float(d0 @ w)
    if abs(neutral_derivative) <= 1e-12 * max(float(np.linalg.norm(d0) * np.linalg.norm(w)), 1.0):
        return unavailable("The neutral rocker is at an unresolved toggle; its assembly branch is not unique.")
    d = o + parallel - moved["actuation_outer"]
    a, b = float(d @ v), float(d @ w)
    c = (length * length - float(d @ d) - float(v @ v)) / 2
    radius = math.hypot(a, b)
    if radius <= MIN_LENGTH * MIN_LENGTH or radius * radius - c * c <= 1e-12 * radius * radius:
        return unavailable("The rocker cannot reach this pickup or is at a toggle singularity.")
    offset, phase = math.acos(max(-1.0, min(1.0, c / radius))), math.atan2(b, a)
    candidates = [phase - offset, phase + offset]
    candidates = [angle for angle in candidates if (-a * math.sin(angle) + b * math.cos(angle)) * neutral_derivative > 0]
    if len(candidates) != 1:
        return unavailable("Rocker assembly branch is unresolved.")
    angle = candidates[0]
    angle += 2 * math.pi * round((previous_angle - angle) / (2 * math.pi))
    if abs(angle) > MAX_ROTATION or abs(angle - previous_angle) > math.pi / 4:
        return unavailable("Rocker motion leaves the supported nearby assembly branch.")
    rotation = Rotation.from_rotvec(k * angle).as_matrix()
    moved["rocker_rod"] = o + rotation @ rod
    moved["rocker_damper"] = o + rotation @ (p["rocker_damper"] - o)
    error = abs(float(np.linalg.norm(moved["rocker_rod"] - moved["actuation_outer"])) - length)
    if error > ACCEPT_MM:
        return unavailable("Rocker rod closure exceeds the numerical length tolerance.")
    compression = np.linalg.norm(p["damper_top"] - p["rocker_damper"]) - np.linalg.norm(moved["damper_top"] - moved["rocker_damper"])
    return {"ok": True, "compression": float(compression), "angleRad": float(angle), "residualMm": float(error)}


def circular_tire_constraint(normal, offset_mm, radius_mm):
    """Return exact rigid circular centerline contact residual/Jacobian.

    Plane n.C = offset, +n toward the road; no finite tread/loaded-radius
    model is inferred. Caller must establish this selected contact model and
    static compatibility, and transform the plane into this corner's frame.
    """
    if not _point(normal) or not _finite(offset_mm) or not _finite(radius_mm) or radius_mm <= 0:
        raise ValueError("Tire plane needs a finite normal, offset and positive circle radius.")
    m = np.asarray(normal, dtype=float)
    length = float(np.linalg.norm(m))
    if length <= MIN_LENGTH:
        raise ValueError("Road-plane normal cannot be zero.")
    m /= length
    offset = offset_mm / length

    def constraint(center, rotation, static_spindle):
        spindle = rotation @ static_spindle
        mu = float(m @ spindle)
        radial = math.sqrt(max(0.0, 1.0 - mu * mu))
        if radial <= 1e-8:
            raise ValueError("The circular tire contact normal is unresolved with the spindle normal to the road.")
        value = float(m @ center) + radius_mm * radial - offset
        row = np.r_[m, -radius_mm * mu / radial * np.cross(spindle, m)]
        return value, row

    return constraint


def solve_corner(axle, bump_mm=0.0, rack_mm=0.0, *, side="left", static_alignment=None,
                 rocker_axis=None, constraint_factory: Callable | None = None,
                 continuation_steps=None, trace=False):
    """Solve one reflected corner, preserving the assembly connected to static.

    Optional constraint_factory(fraction) returns a last-constraint callback
    (center, rotation_matrix, static_spindle) -> (residual_mm, tangent_row).
    Its fraction=0 must represent the entered neutral assembly. This supports
    declared full-body motion without replacing geometry by guessed bump.
    """
    try:
        if not _finite(bump_mm) or not _finite(rack_mm) or abs(bump_mm) > 100:
            return _failed("Wheel bump must be finite and within +/-100 mm; rack must be finite.")
        model = _model(axle, side, static_alignment)
        rack_travel = axle.get("rackTravel")
        if not _finite(rack_travel) or rack_travel <= 0 or abs(rack_mm) > rack_travel / 2 + 1e-8:
            return _failed("Rack displacement exceeds the declared total rack travel.")
        rack = float(rack_mm if axle.get("steered") is True else 0.0)
        center, rotation = model["center"].copy(), np.eye(3)
        steps = max(1, math.ceil(max(abs(bump_mm), abs(rack)) / 2))
        if continuation_steps is not None:
            if type(continuation_steps) is not int or not 1 <= continuation_steps <= 1000:
                return _failed("Continuation steps must be an integer from 1 through 1000.")
            steps = max(steps, continuation_steps)
        initial_constraint = constraint_factory(0.0) if constraint_factory else None
        f0, _ = _constraints(model, center, rotation, 0.0, 0.0, initial_constraint)
        if max(abs(f0)) > ACCEPT_MM:
            return _failed("The imposed neutral wheel/road constraint is inconsistent with the entered static geometry.", float(max(abs(f0))))
        max_condition, max_error, total_iterations = 0.0, 0.0, 0
        actuator = None
        actuator_path_failure = None
        previous_angle = 0.0
        path = []
        for step in range(steps + 1):
            fraction = step / steps
            last_constraint = constraint_factory(fraction) if constraint_factory else None
            pose = _pose(model, bump_mm * fraction, rack * fraction, center, rotation, last_constraint)
            if not pose["ok"]:
                return pose
            center, rotation = pose["center"], pose["matrix"]
            max_condition = max(max_condition, pose["condition"])
            max_error = max(max_error, pose["error"])
            total_iterations += pose["iterations"]
            moved, attachment_error = _moved_points(model, pose, rack * fraction)
            actuator = _actuator(axle, model, moved, rocker_axis, previous_angle)
            if not actuator["ok"]:
                actuator_path_failure = actuator_path_failure or actuator
            elif actuator["angleRad"] is not None:
                previous_angle = actuator["angleRad"]
            if trace:
                path.append({"ok":True,"side":side,"points":{key:point.tolist() for key,point in moved.items()},
                             "rotation":Rotation.from_matrix(rotation).as_rotvec().tolist(),"actuator":dict(actuator)})
        if actuator_path_failure is not None:
            actuator = actuator_path_failure
            # Do not show solved rocker positions after an unresolved path.
            for key in ("rocker_rod", "rocker_damper"):
                if key in model["points"]:
                    moved[key] = model["points"][key].copy()
        spindle = rotation @ model["spindle"]
        s = SIDES[side]
        camber = math.degrees(math.atan2(float(spindle[2]), math.hypot(float(spindle[0]), float(spindle[1]))))
        steer = math.degrees(math.atan2(float(-s * spindle[0]), float(s * spindle[1])))
        rotvec = Rotation.from_matrix(rotation).as_rotvec()
        return {"ok": True, "error": float(max(pose["error"], attachment_error)),
                **({"_path":path} if trace else {}),
                "points": {key: value.tolist() for key, value in moved.items()},
                "rotation": rotvec.tolist(), "q": np.r_[center - model["center"], rotvec * 300.0].tolist(),
                "spindle": spindle.tolist(), "camber": camber, "toe": -s * steer, "steer": steer,
                "alignmentReference": model["alignmentReference"], "actuator": actuator,
                "compression": actuator["compression"], "side": side,
                "bumpMm": float(model["center"][2] - center[2]), "rackMm": rack,
                "constraintJacobian": pose["jacobian"][-1].tolist(),
                "diagnostics": {"constraintResidualMm": pose["error"], "pathMaximumResidualMm": max_error,
                                "conditionNumber": pose["condition"], "pathMaximumConditionNumber": max_condition,
                                "iterations": total_iterations, "continuationSteps": steps,
                                "attachmentResidualMm": attachment_error,
                                "rotationOrthogonalityError": float(np.linalg.norm(rotation.T @ rotation - np.eye(3))),
                                "rotationDeterminant": float(np.linalg.det(rotation))}}
    except (ValueError, TypeError, KeyError, OverflowError, np.linalg.LinAlgError) as error:
        return _failed(str(error))


def corner_velocity(axle, pose, *, bump_rate=1.0, rack_rate=0.0, constraint_rate=None):
    """Differentiate the five physical constraints plus the solved last row.

    Rates are per caller's parameter unit. constraint_rate is the explicit
    partial derivative of the last residual, with pose held fixed; default
    is bump_rate. omega is a spatial angular-rate vector, rad/parameter.
    """
    try:
        if not isinstance(pose, dict) or pose.get("ok") is not True:
            return _failed("An accepted corner pose is required for a constraint tangent.")
        if not _finite(bump_rate) or not _finite(rack_rate) or (constraint_rate is not None and not _finite(constraint_rate)):
            return _failed("Constraint rates must be finite.")
        model = _model(axle, pose.get("side", "left"))
        center = np.asarray(pose["points"]["wheel_center"])
        rotation = Rotation.from_rotvec(pose["rotation"]).as_matrix()
        _, jacobian = _constraints(model, center, rotation, pose["bumpMm"], pose["rackMm"])
        jacobian[-1] = np.asarray(pose["constraintJacobian"])
        scaled, condition = _condition(jacobian)
        if not math.isfinite(condition) or condition > MAX_CONDITION:
            return _failed("Constraint tangent is singular or numerically unresolved.")
        rhs = np.zeros(6)
        if axle.get("steered") is True:
            tie_index = next(index for index, pair in enumerate(model["pairs"]) if pair[0] == "tie_rod_inner")
            rhs[tie_index] = jacobian[tie_index, 1] * rack_rate
        rhs[-1] = -(bump_rate if constraint_rate is None else constraint_rate)
        velocity = np.linalg.solve(scaled, rhs)
        velocity[3:] /= SCALE_MM
        residual = float(np.max(np.abs(jacobian @ velocity - rhs)))
        if not np.isfinite(velocity).all():
            return _failed("Constraint tangent produced nonfinite motion.")
        linear, omega = velocity[:3], velocity[3:]
        return {"ok": True, "linear": linear.tolist(), "angular": omega.tolist(),
                "spindle": np.cross(omega, np.asarray(pose["spindle"])).tolist(),
                "conditionNumber": condition, "residual": residual}
    except (ValueError, TypeError, KeyError, OverflowError, np.linalg.LinAlgError) as error:
        return _failed(str(error))


def actuator_velocity(axle, pose, velocity, *, rocker_axis=None):
    """Differentiate the selected body attachment and actuator closure.

    compressionRate has mm/parameter units. With the default unit bump-rate
    corner tangent it is damper compression per wheel bump (mm/mm). This is
    an instantaneous derivative, not a total-displacement quotient.
    """
    try:
        if pose.get("ok") is not True or velocity.get("ok") is not True:
            return _failed("A resolved corner pose and tangent are required for actuator motion ratio.")
        if pose.get("actuator", {}).get("ok") is not True:
            return _failed(pose.get("actuator", {}).get("reason", "Actuator geometry is unavailable."))
        model = _model(axle, pose["side"])
        p = {key: np.asarray(value) for key, value in pose["points"].items()}
        center = p["wheel_center"]
        linear, omega = np.asarray(velocity["linear"]), np.asarray(velocity["angular"])
        upright_velocity = lambda point: linear + np.cross(omega, point - center)
        pickup_key = "strut_bottom" if axle["topology"] == "macpherson" else "actuation_outer"
        pickup = p[pickup_key]
        pickup_velocity = upright_velocity(pickup)
        if model["attachment"] != "upright":
            prefix = "lca" if model["attachment"] == "lower-wishbone" else "uca"
            fore, aft = p[prefix + "_front"], p[prefix + "_rear"]
            k = aft - fore
            if np.linalg.norm(k) <= MIN_LENGTH:
                return _failed("Wishbone pivot axis is unresolved for actuator velocity.")
            k /= np.linalg.norm(k)
            ball = p[prefix + "_outer"]
            lever = np.cross(k, ball - fore)
            if np.linalg.norm(lever) <= MIN_LENGTH:
                return _failed("Wishbone ball has no resolved angular velocity lever.")
            ball_velocity = upright_velocity(ball)
            arm_rate = float(lever @ ball_velocity) / float(lever @ lever)
            if np.linalg.norm(lever * arm_rate - ball_velocity) > 1e-7 * max(1.0, np.linalg.norm(ball_velocity)):
                return _failed("Ball velocity is inconsistent with rigid wishbone rotation.")
            pickup_velocity = np.cross(k, pickup - fore) * arm_rate
        if axle["topology"] == "macpherson" or axle.get("actuation") == "direct":
            top = p["strut_top"] if axle["topology"] == "macpherson" else p["damper_top"]
            direction = top - pickup
            if np.linalg.norm(direction) <= MIN_LENGTH:
                return _failed("Damper endpoints coincide; its length derivative is unresolved.")
            direction /= np.linalg.norm(direction)
            return {"ok": True, "compressionRate": float(direction @ pickup_velocity),
                    "rockerAngleRateRad": None, "pickupVelocity": pickup_velocity.tolist(),
                    "residual": 0.0}
        if not _point(rocker_axis) or np.linalg.norm(rocker_axis) <= MIN_LENGTH:
            return _failed("An explicit nonzero rocker axis is required for actuator velocity.")
        k = np.asarray(rocker_axis, dtype=float) * model["reflection"]
        k /= np.linalg.norm(k)
        rod, pivot = p["rocker_rod"], p["rocker_pivot"]
        d = rod - pickup
        lever = np.cross(k, rod - pivot)
        denominator = float(d @ lever)
        if abs(denominator) <= 1e-10 * max(1.0, float(np.linalg.norm(d) * np.linalg.norm(lever))):
            return _failed("Rocker is at or too close to a toggle for a resolved motion ratio.")
        angle_rate = float(d @ pickup_velocity) / denominator
        damper_velocity = np.cross(k, p["rocker_damper"] - pivot) * angle_rate
        direction = p["damper_top"] - p["rocker_damper"]
        if np.linalg.norm(direction) <= MIN_LENGTH:
            return _failed("Damper endpoints coincide; its length derivative is unresolved.")
        direction /= np.linalg.norm(direction)
        return {"ok": True, "compressionRate": float(direction @ damper_velocity),
                "rockerAngleRateRad": angle_rate, "pickupVelocity": pickup_velocity.tolist(),
                "residual": abs(float(d @ (lever * angle_rate - pickup_velocity))) / float(np.linalg.norm(d))}
    except (ValueError, TypeError, KeyError, OverflowError, np.linalg.LinAlgError) as error:
        return _failed(str(error))
