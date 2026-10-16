import test from 'node:test';
import assert from 'node:assert/strict';
import {
  newSteeringDesign, exampleSteeringDesign, validateSteeringDesign,
  solveSteering, sweepSteering, idealOutsideAngle, steeringNumerics,
} from '../src/steering.js';

const rad = Math.PI / 180;
const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b} (tolerance ${tolerance})`);
const vectorClose = (a, b, tolerance = 1e-9) => a.forEach((x, i) => close(x, b[i], tolerance));
const sub = (a, b) => a.map((x, i) => x - b[i]);
const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);
const norm = a => Math.hypot(...a);
const solved = (d, angle) => { const result = solveSteering(d, angle); assert.equal(result.ok, true, result.reason); return result; };

test('blank input never infers dimensions, alignment, rack gain or geometry', () => {
  const d = newSteeringDesign();
  assert.deepEqual(validateSteeringDesign(d, { allowIncomplete: true }), []);
  assert.ok(validateSteeringDesign(d).length > 20);
  for (const key of ['wheelbase', 'frontTrack', 'rearTrack', 'rackTravel', 'rackMmPerRevolution', 'steeringLimitDeg']) assert.equal(d[key], null);
  assert.equal(d.left.staticToeDeg, null);
  assert.equal(d.left.staticCamberDeg, null);
  assert.equal(solveSteering(d, 0).ok, false);
  assert.deepEqual(sweepSteering(d), []);
  d.left.axisUpper[0] = 50;
  assert.equal(d.right.axisUpper[0], null, 'blank arrays are independent');
});

test('JSON round trip preserves entered data and strict coordinate metadata', () => {
  const d = exampleSteeringDesign();
  const imported = JSON.parse(JSON.stringify(d));
  assert.deepEqual(imported, d);
  assert.deepEqual(validateSteeringDesign(imported), []);
  const frozen = JSON.stringify(imported);
  solved(imported, 145);
  assert.equal(JSON.stringify(imported), frozen, 'solver must not mutate imported coordinates');
  for (const [key, bad] of [['version', 2], ['kind', 'suspension'], ['coordinateSystem', 'Z_UP'], ['hardpointFrame', 'GLOBAL'], ['lengthUnit', 'm']]) {
    const invalid = { ...d, [key]: bad };
    assert.ok(validateSteeringDesign(invalid).some(error => error.includes(key)));
    assert.equal(solveSteering(invalid, 0).ok, false);
  }
});

test('schema rejects coercion, malformed points, missing metadata and invalid table limits', () => {
  assert.ok(validateSteeringDesign(null).length);
  assert.ok(validateSteeringDesign([]).length);
  for (const value of ['1600', NaN, Infinity, undefined, 0, -1, 1e8]) {
    const d = exampleSteeringDesign(); d.wheelbase = value;
    assert.ok(validateSteeringDesign(d).length, String(value));
  }
  for (const value of [null, {}, [0, 0], [0, 0, 0, 0], [0, NaN, 0], [0, '0', 0], Array(3)]) {
    const d = exampleSteeringDesign(); d.left.axisUpper = value;
    assert.ok(validateSteeringDesign(d).length, JSON.stringify(value));
  }
  for (const value of [0, -5, 6, 3605, NaN]) {
    const d = exampleSteeringDesign(); d.steeringLimitDeg = value;
    assert.ok(validateSteeringDesign(d).length);
    assert.deepEqual(sweepSteering(d), []);
  }
  const d = exampleSteeringDesign(); d.rackMmPerRevolution = 0;
  assert.ok(validateSteeringDesign(d).some(error => error.includes('nonzero')));
  d.rackMmPerRevolution = -60; d.left.staticCamberDeg = 90;
  assert.ok(validateSteeringDesign(d).length);
  d.left.staticCamberDeg = 0; delete d.coordinateSystem;
  assert.ok(validateSteeringDesign(d, { allowIncomplete: true }).length);
});

test('independent prescribed upright rotation recovers the road-wheel angle', () => {
  const d = exampleSteeringDesign();
  // Prescribe left upright rotation directly in planar geometry, then obtain
  // rack displacement by intersecting its horizontal rack line with the rod.
  const angle = 10 * rad;
  const x = -100 * Math.cos(angle) - 30 * Math.sin(angle);
  const y = -550 - 100 * Math.sin(angle) + 30 * Math.cos(angle);
  const dx = x - d.left.tieInner[0], dy = y - d.left.tieInner[1];
  const q = dy + Math.sqrt(270 ** 2 - dx ** 2);
  const r = solved(d, q * 360 / d.rackMmPerRevolution);
  close(r.rack, q);
  close(r.left.rotationDeg, 10);
  close(r.left.headingDeg, 10);
  vectorClose(r.left.outer, [x, y, -250]);
  close(r.left.tieLength, 270);
  close(r.left.residual, 0, 1e-10);
});

test('neutral pose retains separately entered static toe and camber exactly', () => {
  const d = exampleSteeringDesign();
  d.left.staticToeDeg = 0.4; d.right.staticToeDeg = 0.7;
  d.left.staticCamberDeg = -2.2; d.right.staticCamberDeg = 1.3;
  const r = solved(d, 0);
  close(r.left.headingDeg, 0.4); close(r.right.headingDeg, -0.7);
  close(r.left.camberDeg, -2.2); close(r.right.camberDeg, 1.3);
  close(r.metrics.totalToeInDeg, 1.1); close(r.metrics.deltaTotalToeInDeg, 0);
  close(r.left.deltaToeInDeg, 0); close(r.right.deltaToeInDeg, 0);
  assert.equal(r.metrics.turn, 'mixed');
  assert.equal(r.metrics.ackermannPercent, null);
  vectorClose(r.left.outer, d.left.tieOuter);
  vectorClose(r.left.wheelCenter, [0, -600, -250]);
});

test('inclined-axis steering preserves rigid upright invariants and tie length', () => {
  const d = exampleSteeringDesign();
  d.left.axisUpper = [30, -530, -450]; d.left.axisLower = [-10, -560, -150];
  d.left.staticToeDeg = 0.5; d.left.staticCamberDeg = -2;
  const r = solved(d, 120).left, s = d.left;
  const axis = sub(s.axisLower, s.axisUpper), k = axis.map(x => x / norm(axis));
  const outer0 = sub(s.tieOuter, s.axisUpper), outer1 = sub(r.outer, s.axisUpper);
  const wc0 = sub([0, -d.frontTrack / 2, s.wheelCenterZ], s.axisUpper), wc1 = sub(r.wheelCenter, s.axisUpper);
  close(norm(outer0), norm(outer1)); close(dot(k, outer0), dot(k, outer1));
  close(norm(wc0), norm(wc1)); close(dot(k, wc0), dot(k, wc1));
  close(dot(outer0, wc0), dot(outer1, wc1), 1e-8);
  close(norm(sub(r.outer, r.wheelCenter)), norm(sub(s.tieOuter, [0, -600, s.wheelCenterZ])));
  close(norm(r.spindle), 1, 1e-12);
  close(norm(sub(r.outer, r.inner)), r.tieLength);
  assert.ok(Math.abs(r.camberDeg - s.staticCamberDeg) > 0.01, 'inclination must affect camber');
  assert.ok(Math.abs(r.wheelCenter[2] - s.wheelCenterZ) > 0.01, 'geometric vertical motion is reported');
  assert.ok(Math.abs(r.headingDeg - r.rotationDeg - s.staticToeDeg) > 0.01, 'spatial heading is not assumed equal to axis rotation');
});

test('axis endpoint ordering changes only the rotation-parameter sign', () => {
  const d = exampleSteeringDesign();
  d.left.axisUpper = [30, -530, -450]; d.left.axisLower = [-10, -560, -150];
  const a = solved(d, 120);
  [d.left.axisUpper, d.left.axisLower] = [d.left.axisLower, d.left.axisUpper];
  const b = solved(d, 120);
  vectorClose(a.left.outer, b.left.outer); vectorClose(a.left.wheelCenter, b.left.wheelCenter);
  vectorClose(a.left.spindle, b.left.spindle);
  close(a.left.headingDeg, b.left.headingDeg);
  close(a.left.rotationDeg, -b.left.rotationDeg);
});

test('mirror symmetry, steering sign and side-dependent toe are consistent', () => {
  const d = exampleSteeringDesign();
  for (let angle = 5; angle <= 180; angle += 5) {
    const right = solved(d, angle), left = solved(d, -angle);
    assert.equal(right.metrics.turn, 'right'); assert.equal(left.metrics.turn, 'left');
    close(right.left.headingDeg, -left.right.headingDeg);
    close(right.right.headingDeg, -left.left.headingDeg);
    close(right.left.toeInDeg, right.left.headingDeg);
    close(right.right.toeInDeg, -right.right.headingDeg);
    close(right.metrics.totalToeInDeg, left.metrics.totalToeInDeg);
    close(right.metrics.insideDeg, left.metrics.insideDeg);
    assert.ok(right.metrics.insideDeg > right.metrics.outsideDeg);
    close(right.metrics.ackermannPercent, left.metrics.ackermannPercent, 1e-7);
    for (const side of ['left', 'right']) close(norm(sub(right[side].outer, right[side].inner)), right[side].tieLength);
  }
  const negativeGain = solved(d, 100);
  d.rackMmPerRevolution *= -1;
  const positiveGain = solved(d, -100);
  close(positiveGain.left.headingDeg, negativeGain.left.headingDeg);
  assert.equal(solved(d, 100).metrics.turn, 'left', 'gain sign is honored rather than silently corrected');
});

test('rear track affects neither front linkage solve nor nominal Ackermann target', () => {
  const d = exampleSteeringDesign(), a = solved(d, 100);
  d.rearTrack = 2200;
  const b = solved(d, 100);
  assert.deepEqual(a, b);
});

test('rack half-travel and invalid requested angles are rejected', () => {
  const d = exampleSteeringDesign(); d.rackTravel = 60;
  solved(d, 180);
  assert.match(solveSteering(d, 180.001).reason, /half-travel/);
  for (const angle of [null, undefined, '0', NaN, Infinity, 3601]) assert.equal(solveSteering(d, angle).ok, false);
});

test('zero axis, on-axis outer joint, zero rod and neutral toggle are rejected', () => {
  const changes = [
    s => { s.axisLower = [...s.axisUpper]; },
    s => { s.tieOuter = [...s.axisLower]; },
    s => { s.tieInner = [...s.tieOuter]; },
    s => { s.tieInner = [0, -550, -250]; },
  ];
  for (const change of changes) {
    const d = exampleSteeringDesign(); change(d.left);
    const r = solveSteering(d, 0);
    assert.equal(r.ok, false); assert.match(r.reason, /left:/);
  }
});

function pathFixture(innerX) {
  const d = exampleSteeringDesign();
  d.rackMmPerRevolution = -360;
  d.left.axisUpper = [0, 0, -1]; d.left.axisLower = [0, 0, 1];
  d.left.tieOuter = [0, 2, 0]; d.left.tieInner = [innerX, 2, 0];
  return d;
}

test('an endpoint with algebraic roots cannot jump an intervening infeasible interval', () => {
  const d = pathFixture(0.5);
  // Radius=2, rod=.5. Both endpoint rack radii sqrt(.5²+2²)
  // lie in [1.5,2.5], but the intermediate radius .5 does not.
  const endpointG = 4 ** 2 + 1 ** 2 - (-4) ** 2;
  assert.ok(endpointG > 0, 'endpoint alone has two algebraic roots');
  assert.equal(solveSteering(d, 0).ok, true);
  const result = solveSteering(d, 4);
  assert.equal(result.ok, false); assert.match(result.reason, /path.*toggle/);
});

test('path test detects tangent toggles and narrow infeasible intervals between samples', () => {
  assert.equal(solveSteering(pathFixture(1), 0).ok, true);
  assert.match(solveSteering(pathFixture(1), 4).reason, /toggle/);
  const d = pathFixture(0.9999999);
  // Sub-millimeter forbidden interval centered at q=-2; endpoint is -3.753.
  // The analytical quartic-extremum test cannot step over this interval.
  const endpointRadius = Math.hypot(0.9999999, 2 - 3.753);
  assert.ok(endpointRadius > 2 - 0.9999999 && endpointRadius < 2 + 0.9999999);
  assert.match(solveSteering(d, 3.753).reason, /path.*toggle/);
});

test('Seward Table 6.2 target angles agree with the independent right-triangle relation', () => {
  // Seward Ch6 §6.3, printed p.163 / PDF p.174: L=1600 mm, t=1200 mm.
  // Stored inner values are unrounded evaluations; book table rounds to 0.1°.
  const rows = [[5, 5.349151385], [10, 11.486068272], [15, 18.538346781], [20, 26.593987016], [25, 35.644282431]];
  for (const [outer, inner] of rows) {
    const radius = 1600 / Math.tan(outer * rad) - 1200 / 2;
    const fromTriangle = Math.atan(1600 / (radius - 1200 / 2)) / rad;
    close(inner, fromTriangle, 1e-9);
    close(idealOutsideAngle(inner, 1200, 1600), outer, 1e-9);
  }
  close(idealOutsideAngle(0, 1200, 1600), 0);
  close(idealOutsideAngle(1e-10, 1200, 1600), 1e-10, 1e-18);
  for (const angle of [-1, 90, NaN]) assert.equal(idealOutsideAngle(angle, 1200, 1600), null);
  assert.equal(idealOutsideAngle(10, 0, 1600), null);
});

test('cotangent percentage uses its declared formula and is suppressed near straight', () => {
  const d = exampleSteeringDesign();
  const inside = 20, outside = idealOutsideAngle(inside, d.frontTrack, d.wheelbase);
  d.left.staticToeDeg = outside; d.right.staticToeDeg = -inside;
  const r = solved(d, 0);
  close(r.metrics.idealOutsideDeg, outside);
  close(r.metrics.ackermannErrorDeg, 0);
  close(r.metrics.ackermannPercent, 100);
  close(r.metrics.cotResidual, 0);
  const straight = solved(exampleSteeringDesign(), 0);
  assert.equal(straight.metrics.turn, 'straight');
  assert.equal(straight.metrics.ackermannPercent, null);
  assert.equal(straight.metrics.idealOutsideDeg, null);
  const tiny = solved(exampleSteeringDesign(), 0.001);
  assert.equal(tiny.metrics.ackermannPercent, null);
  assert.notEqual(tiny.metrics.idealOutsideDeg, null);
  d.left.staticToeDeg = 89; d.right.staticToeDeg = -89;
  assert.equal(solved(d, 0).metrics.idealOutsideDeg, null);
});

test('5-degree sweep includes both endpoints and neutral, retaining infeasible rows', () => {
  const d = exampleSteeringDesign(); d.steeringLimitDeg = 20;
  const rows = sweepSteering(d);
  assert.deepEqual(rows.map(row => row.steeringWheelDeg), [-20, -15, -10, -5, 0, 5, 10, 15, 20]);
  assert.ok(rows.every(row => row.ok));
  d.rackTravel = 2;
  const limited = sweepSteering(d);
  assert.equal(limited.length, 9);
  assert.equal(limited[0].ok, false);
  assert.equal(limited[4].ok, true);
  assert.equal(limited.at(-1).ok, false);
  assert.equal(steeringNumerics.pathDiscriminantRelativeMargin, 1e-12);
});
