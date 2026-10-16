/**
 * Fixed-ride rack-and-pinion steering. Derivation, signs, source evidence and
 * applicability: ../docs/STEERING.md. All positions/lengths are axle-local mm.
 * This is rigid kinematics; it does not establish measured vehicle accuracy.
 */
const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;
const SIDES = ['left', 'right'];
const MAX_MM = 1e7;
const MIN_LENGTH_MM = 1e-7;
const PATH_MARGIN = 1e-12;
const MAX_SW_DEG = 3600;
const COT_MIN_ANGLE_DEG = 0.01;
const META = {
  version: 1,
  kind: 'fixed-axis-steering',
  coordinateSystem: 'SAE_J670_Z_DOWN',
  hardpointFrame: 'AXLE_LOCAL',
  lengthUnit: 'mm',
};
const add = (a, b) => a.map((x, i) => x + b[i]);
const sub = (a, b) => a.map((x, i) => x - b[i]);
const mul = (a, s) => a.map(x => x * s);
const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);
const finite = n => typeof n === 'number' && Number.isFinite(n);
const record = x => x !== null && typeof x === 'object' && !Array.isArray(x);
const cleanZero = n => Math.abs(n) < 1e-12 ? 0 : n;

export const steeringNumerics = Object.freeze({
  maxCoordinateOrLengthMm: MAX_MM,
  minimumLinkOrAxisMm: MIN_LENGTH_MM,
  pathDiscriminantRelativeMargin: PATH_MARGIN,
  closureAbsoluteToleranceMm: 1e-7,
  closureRelativeTolerance: 1e-10,
  maximumSteeringWheelLimitDeg: MAX_SW_DEG,
  minimumCotangentMetricAngleDeg: COT_MIN_ANGLE_DEG,
});

export function newSteeringDesign() {
  const side = () => ({
    axisUpper: [null, null, null], axisLower: [null, null, null],
    tieInner: [null, null, null], tieOuter: [null, null, null],
    wheelCenterZ: null, staticToeDeg: null, staticCamberDeg: null,
  });
  return {
    ...META, name: 'Untitled steering geometry',
    wheelbase: null, frontTrack: null, rearTrack: null, rackTravel: null,
    rackMmPerRevolution: null, steeringLimitDeg: null,
    left: side(), right: side(),
  };
}

/** Deliberately synthetic dimensions, never an inferred vehicle geometry. */
export function exampleSteeringDesign() {
  return {
    ...newSteeringDesign(), name: 'Synthetic symmetric rack example',
    wheelbase: 1600, frontTrack: 1200, rearTrack: 1180,
    rackTravel: 120, rackMmPerRevolution: -60, steeringLimitDeg: 180,
    left: {
      axisUpper: [0, -550, -450], axisLower: [0, -550, -150],
      tieInner: [-100, -250, -250], tieOuter: [-100, -520, -250],
      wheelCenterZ: -250, staticToeDeg: 0, staticCamberDeg: 0,
    },
    right: {
      axisUpper: [0, 550, -450], axisLower: [0, 550, -150],
      tieInner: [-100, 250, -250], tieOuter: [-100, 520, -250],
      wheelCenterZ: -250, staticToeDeg: 0, staticCamberDeg: 0,
    },
  };
}

/** Strict JSON-compatible schema validation; null means explicitly incomplete. */
export function validateSteeringDesign(d, { allowIncomplete = false } = {}) {
  const errors = [];
  if (!record(d)) return ['Steering design must be an object.'];
  for (const [key, expected] of Object.entries(META)) {
    if (d[key] !== expected) errors.push(`${key} must be ${JSON.stringify(expected)}; no coordinate or unit conversion is inferred.`);
  }
  if (typeof d.name !== 'string' || !d.name.trim() || d.name.length > 200) errors.push('Name must contain 1–200 characters.');
  function number(value, label, { positive = false, nonzero = false, limit = MAX_MM } = {}) {
    if (value === null && allowIncomplete) return;
    if (!finite(value)) { errors.push(`${label} must be a finite number${allowIncomplete ? ' or null' : ''}.`); return; }
    if (Math.abs(value) > limit) errors.push(`${label} magnitude must not exceed ${limit}.`);
    if (positive && value <= 0) errors.push(`${label} must be positive.`);
    if (nonzero && value === 0) errors.push(`${label} must be nonzero; its sign defines rack direction.`);
  }
  for (const key of ['wheelbase', 'frontTrack', 'rearTrack', 'rackTravel']) number(d[key], key, { positive: true });
  number(d.rackMmPerRevolution, 'rackMmPerRevolution', { nonzero: true });
  number(d.steeringLimitDeg, 'steeringLimitDeg', { positive: true, limit: MAX_SW_DEG });
  if (finite(d.steeringLimitDeg) && d.steeringLimitDeg % 5 !== 0) errors.push('steeringLimitDeg must be a multiple of 5 degrees.');
  for (const side of SIDES) {
    const s = d[side];
    if (!record(s)) { errors.push(`${side} geometry must be an object.`); continue; }
    for (const key of ['axisUpper', 'axisLower', 'tieInner', 'tieOuter']) {
      if (!Array.isArray(s[key]) || s[key].length !== 3) { errors.push(`${side}.${key} must contain exactly three axle-local coordinates.`); continue; }
      for (let i = 0; i < 3; i++) number(s[key][i], `${side}.${key}.${['X', 'Y', 'Z'][i]}`);
    }
    number(s.wheelCenterZ, `${side}.wheelCenterZ`);
    for (const key of ['staticToeDeg', 'staticCamberDeg']) {
      number(s[key], `${side}.${key}`, { limit: 90 });
      if (finite(s[key]) && Math.abs(s[key]) >= 90) errors.push(`${side}.${key} magnitude must be less than 90 degrees.`);
    }
  }
  return errors;
}

const evaluate = (p, x) => p.reduceRight((v, coefficient) => v * x + coefficient, 0);
const derivative = p => p.slice(1).map((coefficient, i) => coefficient * (i + 1));

/**
 * Isolate all real polynomial roots on [0,1]. Derivative roots partition the
 * interval into monotone pieces; bisection handles crossings and evaluating
 * the partition points includes repeated roots. Degree is at most three here.
 */
function rootsOnUnitInterval(coefficients) {
  const p = coefficients.slice();
  const magnitude = Math.max(...p.map(Math.abs));
  if (!magnitude) return [];
  // Discard terms below round-off significance; PATH_MARGIN is 100× larger.
  while (p.length > 1 && Math.abs(p.at(-1)) <= magnitude * 1e-14) p.pop();
  if (p.length === 1) return [];
  if (p.length === 2) {
    const root = -p[0] / p[1];
    return root >= 0 && root <= 1 ? [root] : [];
  }
  const divisions = [0, ...rootsOnUnitInterval(derivative(p)), 1].sort((a, b) => a - b);
  const roots = [];
  const zeroTolerance = magnitude * 1e-14;
  for (const x of divisions) if (Math.abs(evaluate(p, x)) <= zeroTolerance) roots.push(x);
  for (let i = 1; i < divisions.length; i++) {
    let low = divisions[i - 1], high = divisions[i];
    let fLow = evaluate(p, low), fHigh = evaluate(p, high);
    if (fLow === 0 || fHigh === 0 || Math.sign(fLow) === Math.sign(fHigh)) continue;
    for (let iteration = 0; iteration < 70; iteration++) {
      const mid = (low + high) / 2;
      if (mid === low || mid === high) break;
      const fMid = evaluate(p, mid);
      if (fMid === 0) { low = high = mid; break; }
      if (Math.sign(fLow) === Math.sign(fMid)) { low = mid; fLow = fMid; }
      else { high = mid; fHigh = fMid; }
    }
    roots.push((low + high) / 2);
  }
  return roots.sort((a, b) => a - b).filter((x, i, all) => i === 0 || Math.abs(x - all[i - 1]) > 1e-12);
}

function rotate(vector, k, phi) {
  const cosine = Math.cos(phi), sine = Math.sin(phi);
  return add(add(mul(vector, cosine), mul(cross(k, vector), sine)), mul(k, dot(k, vector) * (1 - cosine)));
}

function solveSide(d, side, q) {
  const s = d[side], sideSign = side === 'left' ? -1 : 1;
  const axis = sub(s.axisLower, s.axisUpper), axisLength = norm(axis);
  if (axisLength <= MIN_LENGTH_MM) return { ok: false, reason: `${side}: steering axis points coincide or are too close for numerical resolution.` };
  const k = mul(axis, 1 / axisLength), r = sub(s.tieOuter, s.axisUpper);
  const v = sub(r, mul(k, dot(k, r))), w = cross(k, v), rho = norm(v);
  if (rho <= MIN_LENGTH_MM) return { ok: false, reason: `${side}: outer tie joint lies on the steering axis; it cannot define steering rotation.` };
  const center = add(s.axisUpper, mul(k, dot(k, r))), D0 = sub(center, s.tieInner);
  const tieLength = norm(sub(s.tieOuter, s.tieInner));
  if (tieLength <= MIN_LENGTH_MM) return { ok: false, reason: `${side}: tie-rod reference length is zero or below numerical resolution.` };
  const a0 = dot(D0, v), b0 = dot(D0, w);
  const a1 = -v[1] * q, b1 = -w[1] * q, c1 = D0[1] * q, c2 = -q * q / 2;
  // c(0)=a(0) exactly because tie length comes from the entered assembly.
  const scale = Math.max(Math.abs(a0), Math.abs(b0), Math.abs(a1), Math.abs(b1), Math.abs(c1), Math.abs(c2));
  if (!finite(scale) || scale === 0) return { ok: false, reason: `${side}: degenerate tie-rod/steering-axis geometry.` };
  const a = a0 / scale, b = b0 / scale, A = a1 / scale, B = b1 / scale, C = c1 / scale, E = c2 / scale;
  const G = [b * b, 2 * (a * A + b * B - a * C), A * A + B * B - C * C - 2 * a * E, -2 * C * E, -E * E];
  const gScale = G.reduce((sum, x) => sum + Math.abs(x), 0);
  const candidates = [0, 1, ...rootsOnUnitInterval(derivative(G))];
  const minimum = Math.min(...candidates.map(t => evaluate(G, t)));
  // Using the full coefficient scale also guards near cancellation in G.
  const threshold = PATH_MARGIN * Math.max(gScale, a * a + b * b, (a + A) ** 2 + (b + B) ** 2);
  if (!finite(minimum) || minimum <= threshold) {
    return { ok: false, reason: `${side}: neutral-to-requested rack path reaches an inaccessible position, toggle, or numerically unresolved near-toggle; assembly-branch switching is not allowed.` };
  }
  const aq = a + A, bq = b + B, cq = a + C + E;
  const R0 = Math.hypot(a, b), Rq = Math.hypot(aq, bq);
  const theta0 = Math.acos(Math.max(-1, Math.min(1, a / R0)));
  const theta = Math.acos(Math.max(-1, Math.min(1, cq / Rq)));
  const alphaChange = Math.atan2(a * bq - b * aq, a * aq + b * bq);
  const phi = q === 0 ? 0 : alphaChange - Math.sign(b0) * (theta - theta0);
  const outer = add(s.axisUpper, rotate(r, k, phi));
  const inner = add(s.tieInner, [0, q, 0]);
  const residual = norm(sub(outer, inner)) - tieLength;
  const closureTolerance = steeringNumerics.closureAbsoluteToleranceMm + steeringNumerics.closureRelativeTolerance * Math.max(tieLength, rho);
  if (!finite(residual) || Math.abs(residual) > closureTolerance) return { ok: false, reason: `${side}: linkage closure residual exceeds numerical tolerance.` };
  const referenceHeading = -sideSign * s.staticToeDeg * RAD, camber = s.staticCamberDeg * RAD;
  const referenceSpindle = [-sideSign * Math.sin(referenceHeading) * Math.cos(camber), sideSign * Math.cos(referenceHeading) * Math.cos(camber), Math.sin(camber)];
  const spindle = rotate(referenceSpindle, k, phi);
  if (Math.hypot(spindle[0], spindle[1]) <= 1e-10) return { ok: false, reason: `${side}: wheel plane is horizontal; road-plane heading is undefined.` };
  const headingDeg = cleanZero(Math.atan2(-sideSign * spindle[0], sideSign * spindle[1]) * DEG);
  const toeInDeg = cleanZero(-sideSign * headingDeg);
  return {
    ok: true, headingDeg, toeInDeg,
    deltaToeInDeg: cleanZero(toeInDeg - s.staticToeDeg),
    camberDeg: cleanZero(Math.atan2(spindle[2], Math.hypot(spindle[0], spindle[1])) * DEG),
    rotationDeg: cleanZero(phi * DEG), outer, inner,
    wheelCenter: add(s.axisUpper, rotate(sub([0, sideSign * d.frontTrack / 2, s.wheelCenterZ], s.axisUpper), k, phi)),
    spindle, tieLength, residual,
  };
}

/** Exact no-slip planar reference target, with positive inside angle in degrees. */
export function idealOutsideAngle(insideDeg, track, wheelbase) {
  if (!finite(insideDeg) || insideDeg < 0 || insideDeg >= 90 || !finite(track) || track <= 0 || !finite(wheelbase) || wheelbase <= 0) return null;
  const angle = insideDeg * RAD;
  return Math.atan2(wheelbase * Math.sin(angle), wheelbase * Math.cos(angle) + track * Math.sin(angle)) * DEG;
}

function metrics(d, left, right) {
  const lh = left.headingDeg, rh = right.headingDeg;
  const totalToeInDeg = cleanZero(left.toeInDeg + right.toeInDeg);
  const result = {
    turn: 'mixed', insideSide: null, insideDeg: null, outsideDeg: null,
    totalToeInDeg, deltaTotalToeInDeg: cleanZero(totalToeInDeg - d.left.staticToeDeg - d.right.staticToeDeg),
    idealOutsideDeg: null, ackermannErrorDeg: null, ackermannPercent: null, cotResidual: null,
  };
  if (Math.abs(lh) < 1e-10 && Math.abs(rh) < 1e-10) { result.turn = 'straight'; return result; }
  if (lh * rh <= 0) return result;
  result.turn = lh > 0 ? 'right' : 'left';
  result.insideSide = result.turn;
  result.insideDeg = Math.abs(result.insideSide === 'left' ? lh : rh);
  result.outsideDeg = Math.abs(result.insideSide === 'left' ? rh : lh);
  if (result.insideDeg >= 89 || result.outsideDeg >= 89) return result;
  result.idealOutsideDeg = idealOutsideAngle(result.insideDeg, d.frontTrack, d.wheelbase);
  result.ackermannErrorDeg = result.outsideDeg - result.idealOutsideDeg;
  if (Math.min(result.insideDeg, result.outsideDeg) < COT_MIN_ANGLE_DEG) return result;
  const cotDifference = 1 / Math.tan(result.outsideDeg * RAD) - 1 / Math.tan(result.insideDeg * RAD);
  result.ackermannPercent = 100 * d.wheelbase / d.frontTrack * cotDifference;
  result.cotResidual = cotDifference - d.frontTrack / d.wheelbase;
  return result;
}

export function solveSteering(d, steeringWheelDeg) {
  const fail = (reason, rack = null) => ({ ok: false, reason, steeringWheelDeg, rack, left: null, right: null, metrics: null });
  const errors = validateSteeringDesign(d);
  if (errors.length) return fail(errors.join(' '));
  if (!finite(steeringWheelDeg)) return fail('Steering-wheel angle must be a finite number.');
  if (Math.abs(steeringWheelDeg) > MAX_SW_DEG) return fail(`Steering-wheel angle magnitude exceeds the ${MAX_SW_DEG} degree numerical/UI limit.`);
  const rack = d.rackMmPerRevolution * steeringWheelDeg / 360;
  if (Math.abs(rack) > d.rackTravel / 2 + 1e-10) return fail('Requested rack displacement exceeds the entered half-travel limit.', rack);
  const left = solveSide(d, 'left', rack);
  if (!left.ok) return fail(left.reason, rack);
  const right = solveSide(d, 'right', rack);
  if (!right.ok) return fail(right.reason, rack);
  return { ok: true, steeringWheelDeg, rack: cleanZero(rack), left, right, metrics: metrics(d, left, right) };
}

/** Both directions are computed from the entered neutral assembly, in 5° steps. */
export function sweepSteering(d) {
  if (validateSteeringDesign(d).length) return [];
  const rows = [];
  for (let angle = -d.steeringLimitDeg; angle <= d.steeringLimitDeg; angle += 5) rows.push(solveSteering(d, angle));
  return rows;
}
