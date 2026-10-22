"""Fixed-ride spatial rack-and-pinion kinematics, ported from steering.js.

Read suspension-studio/docs/STEERING.md for the first-principles derivation,
source passages, SAE J670 +X forward/+Y right/+Z down signs, and limitations.
All lengths are front-axle-local millimeters; angles exposed here are degrees.

The upright's outer joint follows C+v*cos(phi)+w*sin(phi). Constant tie length
reduces to a*cos(phi)+b*sin(phi)=c. The quartic a(t)^2+b(t)^2-c(t)^2 must remain
positive over the entire neutral-to-requested rack path; every derivative root
and both endpoints are checked. The neutral derivative sign chooses the same
assembly branch. This is a language port, not a new or simplified physical law.
"""

from __future__ import annotations

import json
import math

RAD = math.pi / 180
DEG = 180 / math.pi
SIDES = ("left", "right")
MAX_MM = 1e7
MIN_LENGTH_MM = 1e-7
PATH_MARGIN = 1e-12
MAX_SW_DEG = 3600
COT_MIN_ANGLE_DEG = 0.01
META = {
    "version": 1,
    "kind": "fixed-axis-steering",
    "coordinateSystem": "SAE_J670_Z_DOWN",
    "hardpointFrame": "AXLE_LOCAL",
    "lengthUnit": "mm",
}
STEERING_NUMERICS = {
    "maxCoordinateOrLengthMm": MAX_MM,
    "minimumLinkOrAxisMm": MIN_LENGTH_MM,
    "pathDiscriminantRelativeMargin": PATH_MARGIN,
    "closureAbsoluteToleranceMm": 1e-7,
    "closureRelativeTolerance": 1e-10,
    "maximumSteeringWheelLimitDeg": MAX_SW_DEG,
    "minimumCotangentMetricAngleDeg": COT_MIN_ANGLE_DEG,
}
_MISSING = object()
# ECMAScript String.trim whitespace (Python str.strip has a different set).
_JS_WHITESPACE = "\u0009\u000b\u000c\u0020\u00a0\ufeff\u000a\u000d\u2028\u2029\u1680\u2000\u2001\u2002\u2003\u2004\u2005\u2006\u2007\u2008\u2009\u200a\u202f\u205f\u3000"


def _finite(value: object) -> bool:
    """JSON numbers only: bool must not become an entered dimension or angle."""
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _record(value: object) -> bool:
    return isinstance(value, dict)


def _finite_outputs(value):
    """All computed numbers must be representable, never silently nulled."""
    if type(value) in (int, float):
        return _finite(value)
    if isinstance(value, list):
        return all(_finite_outputs(item) for item in value)
    if isinstance(value, dict):
        return all(_finite_outputs(item) for item in value.values())
    return True


def _add(a, b):
    return [value + b[i] for i, value in enumerate(a)]


def _sub(a, b):
    return [value - b[i] for i, value in enumerate(a)]


def _mul(a, scale):
    return [value * scale for value in a]


def _dot(a, b):
    # Explicit accumulation preserves the JS operation order (Python's sum
    # may use compensated summation in newer interpreters).
    total = 0
    for i, value in enumerate(a):
        total += value * b[i]
    return total


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _norm(a):
    return math.hypot(*a)


def _clean_zero(value):
    return 0 if abs(value) < 1e-12 else value


def _sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def validate_steering_design(design: object, allow_incomplete: bool = False) -> list[str]:
    """Validate the existing JS JSON schema without coercion or inferred inputs.

    Only an explicitly present None is allowed as an incomplete numeric value;
    omitted keys remain malformed, matching undefined versus null in JS.
    """
    errors: list[str] = []
    if not _record(design):
        return ["Steering design must be an object."]
    for key, expected in META.items():
        actual = design.get(key, _MISSING)
        matches = (_finite(actual) and actual == expected) if type(expected) is int else (type(actual) is str and actual == expected)
        if not matches:
            errors.append(f"{key} must be {json.dumps(expected)}; no coordinate or unit conversion is inferred.")
    name = design.get("name")
    if not isinstance(name, str) or not name.strip(_JS_WHITESPACE) or len(name.encode("utf-16-le", errors="surrogatepass")) // 2 > 200:
        errors.append("Name must contain 1–200 characters.")

    def number(value, label, *, positive=False, nonzero=False, limit=MAX_MM):
        if value is None and allow_incomplete:
            return
        if not _finite(value):
            errors.append(f"{label} must be a finite number{' or null' if allow_incomplete else ''}.")
            return
        if abs(value) > limit:
            errors.append(f"{label} magnitude must not exceed {limit:g}.")
        if positive and value <= 0:
            errors.append(f"{label} must be positive.")
        if nonzero and value == 0:
            errors.append(f"{label} must be nonzero; its sign defines rack direction.")

    for key in ("wheelbase", "frontTrack", "rearTrack", "rackTravel"):
        number(design.get(key, _MISSING), key, positive=True)
    number(design.get("rackMmPerRevolution", _MISSING), "rackMmPerRevolution", nonzero=True)
    limit = design.get("steeringLimitDeg", _MISSING)
    number(limit, "steeringLimitDeg", positive=True, limit=MAX_SW_DEG)
    if _finite(limit) and limit % 5 != 0:
        errors.append("steeringLimitDeg must be a multiple of 5 degrees.")
    for side in SIDES:
        geometry = design.get(side)
        if not _record(geometry):
            errors.append(f"{side} geometry must be an object.")
            continue
        for key in ("axisUpper", "axisLower", "tieInner", "tieOuter"):
            point = geometry.get(key)
            if not isinstance(point, list) or len(point) != 3:
                errors.append(f"{side}.{key} must contain exactly three axle-local coordinates.")
                continue
            for i, label in enumerate(("X", "Y", "Z")):
                number(point[i], f"{side}.{key}.{label}")
        number(geometry.get("wheelCenterZ", _MISSING), f"{side}.wheelCenterZ")
        for key in ("staticToeDeg", "staticCamberDeg"):
            value = geometry.get(key, _MISSING)
            number(value, f"{side}.{key}", limit=90)
            if _finite(value) and abs(value) >= 90:
                errors.append(f"{side}.{key} magnitude must be less than 90 degrees.")
    return errors


def _evaluate(polynomial, x):
    value = 0
    for coefficient in reversed(polynomial):
        value = value * x + coefficient
    return value


def _derivative(polynomial):
    return [coefficient * (i + 1) for i, coefficient in enumerate(polynomial[1:])]


def _roots_on_unit_interval(coefficients):
    """Find all real roots by derivative-partition isolation, then bisection.

    Used only for degree at most three. Repeated roots are included by testing
    the derivative partition points. No finite rack-travel stepping is used.
    """
    polynomial = list(coefficients)
    magnitude = max(abs(value) for value in polynomial)
    if not magnitude:
        return []
    while len(polynomial) > 1 and abs(polynomial[-1]) <= magnitude * 1e-14:
        polynomial.pop()
    if len(polynomial) == 1:
        return []
    if len(polynomial) == 2:
        root = -polynomial[0] / polynomial[1]
        return [root] if 0 <= root <= 1 else []
    divisions = sorted([0, *_roots_on_unit_interval(_derivative(polynomial)), 1])
    roots = []
    zero_tolerance = magnitude * 1e-14
    for x in divisions:
        if abs(_evaluate(polynomial, x)) <= zero_tolerance:
            roots.append(x)
    for i in range(1, len(divisions)):
        low, high = divisions[i - 1], divisions[i]
        f_low, f_high = _evaluate(polynomial, low), _evaluate(polynomial, high)
        if f_low == 0 or f_high == 0 or _sign(f_low) == _sign(f_high):
            continue
        for _ in range(70):
            middle = (low + high) / 2
            if middle == low or middle == high:
                break
            f_middle = _evaluate(polynomial, middle)
            if f_middle == 0:
                low = high = middle
                break
            if _sign(f_low) == _sign(f_middle):
                low, f_low = middle, f_middle
            else:
                high, f_high = middle, f_middle
        roots.append((low + high) / 2)
    ordered = sorted(roots)
    return [x for i, x in enumerate(ordered) if i == 0 or abs(x - ordered[i - 1]) > 1e-12]


def _rotate(vector, axis, phi):
    cosine, sine = math.cos(phi), math.sin(phi)
    return _add(_add(_mul(vector, cosine), _mul(_cross(axis, vector), sine)), _mul(axis, _dot(axis, vector) * (1 - cosine)))


def _solve_side(design, side, q):
    geometry = design[side]
    side_sign = -1 if side == "left" else 1
    axis = _sub(geometry["axisLower"], geometry["axisUpper"])
    axis_length = _norm(axis)
    if axis_length <= MIN_LENGTH_MM:
        return {"ok": False, "reason": f"{side}: steering axis points coincide or are too close for numerical resolution."}
    k = _mul(axis, 1 / axis_length)
    r = _sub(geometry["tieOuter"], geometry["axisUpper"])
    v = _sub(r, _mul(k, _dot(k, r)))
    w, rho = _cross(k, v), _norm(v)
    if rho <= MIN_LENGTH_MM:
        return {"ok": False, "reason": f"{side}: outer tie joint lies on the steering axis; it cannot define steering rotation."}
    center = _add(geometry["axisUpper"], _mul(k, _dot(k, r)))
    d0 = _sub(center, geometry["tieInner"])
    tie_length = _norm(_sub(geometry["tieOuter"], geometry["tieInner"]))
    if tie_length <= MIN_LENGTH_MM:
        return {"ok": False, "reason": f"{side}: tie-rod reference length is zero or below numerical resolution."}
    a0, b0 = _dot(d0, v), _dot(d0, w)
    a1, b1, c1, c2 = -v[1] * q, -w[1] * q, d0[1] * q, -q * q / 2
    # c(0)=a(0) exactly because reference length is assembled from input joints.
    scale = max(abs(value) for value in (a0, b0, a1, b1, c1, c2))
    if not _finite(scale) or scale == 0:
        return {"ok": False, "reason": f"{side}: degenerate tie-rod/steering-axis geometry."}
    a, b, aa, bb, cc, ee = (value / scale for value in (a0, b0, a1, b1, c1, c2))
    polynomial = [b * b, 2 * (a * aa + b * bb - a * cc), aa * aa + bb * bb - cc * cc - 2 * a * ee, -2 * cc * ee, -ee * ee]
    coefficient_scale = 0
    for coefficient in polynomial:
        coefficient_scale += abs(coefficient)
    candidates = [0, 1, *_roots_on_unit_interval(_derivative(polynomial))]
    minimum = min(_evaluate(polynomial, t) for t in candidates)
    threshold = PATH_MARGIN * max(coefficient_scale, a * a + b * b, (a + aa) ** 2 + (b + bb) ** 2)
    if not _finite(minimum) or minimum <= threshold:
        return {"ok": False, "reason": f"{side}: neutral-to-requested rack path reaches an inaccessible position, toggle, or numerically unresolved near-toggle; assembly-branch switching is not allowed."}
    aq, bq, cq = a + aa, b + bb, a + cc + ee
    r0, rq = math.hypot(a, b), math.hypot(aq, bq)
    theta0 = math.acos(max(-1, min(1, a / r0)))
    theta = math.acos(max(-1, min(1, cq / rq)))
    alpha_change = math.atan2(a * bq - b * aq, a * aq + b * bq)
    phi = 0 if q == 0 else alpha_change - _sign(b0) * (theta - theta0)
    outer = _add(geometry["axisUpper"], _rotate(r, k, phi))
    inner = _add(geometry["tieInner"], [0, q, 0])
    residual = _norm(_sub(outer, inner)) - tie_length
    closure_tolerance = STEERING_NUMERICS["closureAbsoluteToleranceMm"] + STEERING_NUMERICS["closureRelativeTolerance"] * max(tie_length, rho)
    if not _finite(residual) or abs(residual) > closure_tolerance:
        return {"ok": False, "reason": f"{side}: linkage closure residual exceeds numerical tolerance."}
    reference_heading = -side_sign * geometry["staticToeDeg"] * RAD
    camber = geometry["staticCamberDeg"] * RAD
    reference_spindle = [-side_sign * math.sin(reference_heading) * math.cos(camber), side_sign * math.cos(reference_heading) * math.cos(camber), math.sin(camber)]
    spindle = _rotate(reference_spindle, k, phi)
    if math.hypot(spindle[0], spindle[1]) <= 1e-10:
        return {"ok": False, "reason": f"{side}: wheel plane is horizontal; road-plane heading is undefined."}
    heading = _clean_zero(math.atan2(-side_sign * spindle[0], side_sign * spindle[1]) * DEG)
    toe_in = _clean_zero(-side_sign * heading)
    return {
        "ok": True, "headingDeg": heading, "toeInDeg": toe_in,
        "deltaToeInDeg": _clean_zero(toe_in - geometry["staticToeDeg"]),
        "camberDeg": _clean_zero(math.atan2(spindle[2], math.hypot(spindle[0], spindle[1])) * DEG),
        "rotationDeg": _clean_zero(phi * DEG), "outer": outer, "inner": inner,
        "wheelCenter": _add(geometry["axisUpper"], _rotate(_sub([0, side_sign * design["frontTrack"] / 2, geometry["wheelCenterZ"]], geometry["axisUpper"]), k, phi)),
        "spindle": spindle, "tieLength": tie_length, "residual": residual,
    }


def ideal_outside_angle(inside_deg, track, wheelbase):
    """Exact nominal no-slip reference, not a linkage or migrating-contact ICR."""
    if not _finite(inside_deg) or inside_deg < 0 or inside_deg >= 90 or not _finite(track) or track <= 0 or not _finite(wheelbase) or wheelbase <= 0:
        return None
    angle = inside_deg * RAD
    value = math.atan2(wheelbase * math.sin(angle), wheelbase * math.cos(angle) + track * math.sin(angle)) * DEG
    return value if _finite(value) else None


def _metrics(design, left, right):
    left_heading, right_heading = left["headingDeg"], right["headingDeg"]
    total_toe = _clean_zero(left["toeInDeg"] + right["toeInDeg"])
    result = {
        "turn": "mixed", "insideSide": None, "insideDeg": None, "outsideDeg": None,
        "totalToeInDeg": total_toe,
        "deltaTotalToeInDeg": _clean_zero(total_toe - design["left"]["staticToeDeg"] - design["right"]["staticToeDeg"]),
        "idealOutsideDeg": None, "ackermannErrorDeg": None, "ackermannPercent": None, "cotResidual": None,
    }
    if abs(left_heading) < 1e-10 and abs(right_heading) < 1e-10:
        result["turn"] = "straight"
        return result
    if left_heading * right_heading <= 0:
        return result
    result["turn"] = "right" if left_heading > 0 else "left"
    result["insideSide"] = result["turn"]
    inside = abs(left_heading if result["insideSide"] == "left" else right_heading)
    outside = abs(right_heading if result["insideSide"] == "left" else left_heading)
    result["insideDeg"], result["outsideDeg"] = inside, outside
    if inside >= 89 or outside >= 89:
        return result
    ideal = ideal_outside_angle(inside, design["frontTrack"], design["wheelbase"])
    result["idealOutsideDeg"] = ideal
    result["ackermannErrorDeg"] = outside - ideal
    if min(inside, outside) < COT_MIN_ANGLE_DEG:
        return result
    cot_difference = 1 / math.tan(outside * RAD) - 1 / math.tan(inside * RAD)
    result["ackermannPercent"] = 100 * design["wheelbase"] / design["frontTrack"] * cot_difference
    result["cotResidual"] = cot_difference - design["frontTrack"] / design["wheelbase"]
    return result


def solve_steering(design: object, steering_wheel_deg: object) -> dict:
    """Return the JS-compatible pose or an explicit failure, without mutation.

    Invalid requested angle values are reported as null in the failure payload,
    never NaN/Infinity or arbitrary nonnumeric objects. If a calculated output
    exceeds the finite numerical range, the entire reported row fails explicitly;
    null metrics are reserved for the defined unavailable geometric cases.
    """
    def fail(reason, rack=None):
        return {"ok": False, "reason": reason, "steeringWheelDeg": steering_wheel_deg if _finite(steering_wheel_deg) else None,
                "rack": rack, "left": None, "right": None, "metrics": None}

    errors = validate_steering_design(design)
    if errors:
        return fail(" ".join(errors))
    if not _finite(steering_wheel_deg):
        return fail("Steering-wheel angle must be a finite number.")
    if abs(steering_wheel_deg) > MAX_SW_DEG:
        return fail(f"Steering-wheel angle magnitude exceeds the {MAX_SW_DEG} degree numerical/UI limit.")
    rack = design["rackMmPerRevolution"] * steering_wheel_deg / 360
    if abs(rack) > design["rackTravel"] / 2 + 1e-10:
        return fail("Requested rack displacement exceeds the entered half-travel limit.", rack)
    left = _solve_side(design, "left", rack)
    if not left["ok"]:
        return fail(left["reason"], rack)
    right = _solve_side(design, "right", rack)
    if not right["ok"]:
        return fail(right["reason"], rack)
    metrics = _metrics(design, left, right)
    unrepresentable = [key for key, value in metrics.items() if type(value) in (int, float) and not _finite(value)]
    if unrepresentable:
        return fail(f"Derived steering metrics exceed the finite numerical range ({', '.join(unrepresentable)}); the entered wheelbase/track magnitudes cannot support a finite reported result.", rack)
    output = {"ok": True, "steeringWheelDeg": steering_wheel_deg, "rack": _clean_zero(rack),
              "left": left, "right": right, "metrics": metrics}
    if not _finite_outputs(output):
        return fail("Calculated steering output exceeds the finite numerical range; this pose cannot be reported reliably.", rack)
    return output


def sweep_steering(design: object) -> list[dict]:
    """Both signs and neutral, every 5 degrees, preserving failed output rows."""
    if validate_steering_design(design):
        return []
    limit = int(design["steeringLimitDeg"])
    return [solve_steering(design, angle) for angle in range(-limit, limit + 1, 5)]
