/**
 * Steering-arm construction only; this is not a rolling-wheel turn center.
 * Derivation and source limitations: ../docs/STEERING.md.
 * All points are front-axle-local SAE J670 Z-down millimeters.
 */
const DEG = 180 / Math.PI;
const LENGTH_RELATIVE_TOLERANCE = 1e-12;
const PARALLEL_RELATIVE_TOLERANCE = 1e-10;
const AXIS_VERTICAL_COMPONENT_TOLERANCE = 1e-10;
const AXIS_RADIUS_ROUNDOFF_FACTOR = 32;
const finite = value => typeof value === 'number' && Number.isFinite(value);
const point = value => Array.isArray(value) && value.length === 3 && [0, 1, 2].every(i => finite(value[i]));
const sub = (a, b) => a.map((value, i) => value - b[i]);
const norm = v => Math.hypot(...v);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross2 = (a, b) => a[0] * b[1] - a[1] * b[0];
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

export const steeringConstructionNumerics = Object.freeze({
  lengthRelativeTolerance: LENGTH_RELATIVE_TOLERANCE,
  parallelRelativeTolerance: PARALLEL_RELATIVE_TOLERANCE,
  axisVerticalComponentTolerance: AXIS_VERTICAL_COMPONENT_TOLERANCE,
  axisRadiusRoundoffFactor: AXIS_RADIUS_ROUNDOFF_FACTOR,
});

function angleBetween(a, b) {
  const lengthA = norm(a), lengthB = norm(b);
  if (!finite(lengthA) || !finite(lengthB) || lengthA === 0 || lengthB === 0) return null;
  const unitA = a.map(value => value / lengthA), unitB = b.map(value => value / lengthB);
  const sine = a.length === 2 ? Math.abs(cross2(unitA, unitB)) : norm(cross3(unitA, unitB));
  const angle = Math.atan2(sine, dot(unitA, unitB)) * DEG;
  return finite(angle) ? angle : null;
}

function constructionSide(designSide, solvedSide, useSolved) {
  const selectedOuter = useSolved ? solvedSide?.outer : designSide?.tieOuter;
  const selectedInner = useSolved ? solvedSide?.inner : designSide?.tieInner;
  const result = {
    axisFoot: null,
    planAxisPoint: null,
    outer: point(selectedOuter) ? [...selectedOuter] : null,
    inner: point(selectedInner) ? [...selectedInner] : null,
    spatialAngleDeg: null,
    planAngleDeg: null,
  };
  if (!point(designSide?.axisUpper) || !point(designSide?.axisLower) || !result.outer) return result;
  const axis = sub(designSide.axisLower, designSide.axisUpper), axisLength = norm(axis);
  if (!finite(axisLength) || axisLength === 0) return result;
  const k = axis.map(value => value / axisLength);
  const fromUpper = sub(result.outer, designSide.axisUpper), fromLower = sub(result.outer, designSide.axisLower);
  const useUpper = norm(fromUpper) <= norm(fromLower);
  const anchor = useUpper ? designSide.axisUpper : designSide.axisLower;
  const r = useUpper ? fromUpper : fromLower;
  // k×(r×k) is the same perpendicular projection as r−k(k·r), but avoids
  // subtracting a large parallel component from each coordinate. The error
  // scale follows the products in r×k, not an arbitrary axis endpoint span.
  const radialCross = cross3(r, k), radial = cross3(k, radialCross);
  const radius = norm(radial);
  const crossProductScale = Math.hypot(
    Math.abs(r[1] * k[2]) + Math.abs(r[2] * k[1]),
    Math.abs(r[2] * k[0]) + Math.abs(r[0] * k[2]),
    Math.abs(r[0] * k[1]) + Math.abs(r[1] * k[0]),
  );
  const radiusRoundoff = AXIS_RADIUS_ROUNDOFF_FACTOR * Number.EPSILON * crossProductScale;
  const foot = result.outer.map((value, i) => value - radial[i]);
  if (!point(foot)) return result;
  result.axisFoot = foot;
  if (!finite(radius) || !finite(radiusRoundoff) || radius <= radiusRoundoff || norm(sub(foot, result.outer)) === 0) return result;
  // Milliken Fig.19.3 uses the axis AT the outer joint's height, not the
  // perpendicular foot. These points differ for an inclined steering axis.
  if (Math.abs(k[2]) > AXIS_VERTICAL_COMPONENT_TOLERANCE) {
    const heightParameter = (result.outer[2] - anchor[2]) / k[2];
    const atHeight = [
      anchor[0] + k[0] * heightParameter,
      anchor[1] + k[1] * heightParameter,
      result.outer[2],
    ];
    if (point(atHeight)) result.planAxisPoint = atHeight;
  }
  if (!result.inner) return result;
  const arm = sub(foot, result.outer), tie = sub(result.inner, result.outer);
  const armLength = norm(arm), tieLength = norm(tie);
  const localScale = Math.max(armLength, tieLength);
  if (!finite(localScale) || armLength <= LENGTH_RELATIVE_TOLERANCE * localScale || tieLength <= LENGTH_RELATIVE_TOLERANCE * localScale) return result;
  result.spatialAngleDeg = angleBetween(arm, tie);
  if (!result.planAxisPoint) return result;
  const planArm = sub(result.planAxisPoint, result.outer).slice(0, 2);
  const armPlanLength = norm(planArm), tiePlanLength = Math.hypot(tie[0], tie[1]);
  if (!finite(armPlanLength) || armPlanLength <= LENGTH_RELATIVE_TOLERANCE * localScale || tiePlanLength <= LENGTH_RELATIVE_TOLERANCE * tieLength) return result;
  result.planAngleDeg = angleBetween(planArm, tie.slice(0, 2));
  return result;
}

function armLine(side) {
  if (!side.planAxisPoint || !side.outer) return null;
  const arm = sub(side.outer, side.planAxisPoint), spatialLength = norm(arm), planLength = Math.hypot(arm[0], arm[1]);
  if (!finite(spatialLength) || spatialLength === 0 || !finite(planLength) || planLength <= LENGTH_RELATIVE_TOLERANCE * spatialLength) return null;
  return { origin: side.planAxisPoint.slice(0, 2), direction: [arm[0] / planLength, arm[1] / planLength], length: planLength };
}

function intersectArmLines(left, right, wheelbase) {
  const unavailable = status => ({ status, point: null, rearAxleOffsetMm: null });
  const a = armLine(left), b = armLine(right);
  if (!a || !b) return unavailable('undefined');
  const between = sub(b.origin, a.origin), separation = norm(between);
  if (!finite(separation)) return unavailable('undefined');
  const denominator = cross2(a.direction, b.direction);
  if (!finite(denominator)) return unavailable('undefined');
  if (Math.abs(denominator) <= PARALLEL_RELATIVE_TOLERANCE) {
    const perpendicularSeparation = Math.abs(cross2(between, a.direction));
    const positionScale = Math.max(a.length, b.length, separation);
    return unavailable(perpendicularSeparation <= PARALLEL_RELATIVE_TOLERANCE * positionScale ? 'coincident' : 'parallel');
  }
  const distance = cross2(between, b.direction) / denominator;
  const intersection = [a.origin[0] + distance * a.direction[0], a.origin[1] + distance * a.direction[1], 0];
  if (!point(intersection)) return unavailable('undefined');
  const rearOffset = finite(wheelbase) ? intersection[0] + wheelbase : null;
  if (rearOffset !== null && !finite(rearOffset)) return unavailable('undefined');
  return { status: 'finite', point: intersection, rearAxleOffsetMm: rearOffset };
}

/**
 * Included angles are measured AT the outer joint. Effective spatial angle
 * uses outer→perpendicular-axis-foot and outer→inner-joint. The conventional
 * plan angle instead uses the axis point at outer-joint height (Milliken
 * Fig.19.3), with both vectors projected into XY. Infinite plan-arm-line
 * intersections are construction points, never true ICRs.
 */
export function steeringConstruction(design, result = null) {
  const useSolved = result?.ok === true;
  const left = constructionSide(design?.left, result?.left, useSolved);
  const right = constructionSide(design?.right, result?.right, useSolved);
  return {
    left, right,
    intersection: intersectArmLines(left, right, design?.wheelbase),
    pose: useSolved ? 'solved' : 'static',
  };
}
