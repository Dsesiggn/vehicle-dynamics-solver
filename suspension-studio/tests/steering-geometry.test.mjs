import test from 'node:test';
import assert from 'node:assert/strict';
import { steeringConstruction, steeringConstructionNumerics } from '../src/steering-geometry.js';
import { exampleSteeringDesign, newSteeringDesign, solveSteering } from '../src/steering.js';

const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b} (tolerance ${tolerance})`);
const vectorClose = (a, b, tolerance = 1e-9) => a.forEach((x, i) => close(x, b[i], tolerance));
const side = (center, outer, inner) => ({ axisUpper: [center[0], center[1], -10], axisLower: [center[0], center[1], 10], tieOuter: outer, tieInner: inner });
const symmetricDesign = () => ({
  wheelbase: 10,
  left: side([0, -2], [-1, -1, 0], [-1, 1, 0]),
  right: side([0, 2], [-1, 1, 0], [-1, -1, 0]),
});

test('included arm/tie angle is measured at the outer joint in spatial and plan views', () => {
  const result = steeringConstruction(symmetricDesign());
  assert.equal(result.pose, 'static');
  vectorClose(result.left.axisFoot, [0, -2, 0]);
  close(result.left.spatialAngleDeg, 135);
  close(result.left.planAngleDeg, 135);
  close(result.right.spatialAngleDeg, 135);
  close(result.right.planAngleDeg, 135);
  const d = symmetricDesign();
  d.left.tieInner = [-1, -1, 2];
  const verticalTie = steeringConstruction(d).left;
  close(verticalTie.spatialAngleDeg, 90);
  assert.equal(verticalTie.planAngleDeg, null, 'vertical tie has no plan direction');
});

test('inclined-axis plan construction uses ball-joint height, independently of perpendicular foot', () => {
  const d = symmetricDesign();
  d.left = { axisUpper: [0, 0, 0], axisLower: [0, 1, 1], tieOuter: [1, 1, 0], tieInner: [1, 3, 2] };
  const result = steeringConstruction(d).left;
  vectorClose(result.axisFoot, [0, 0.5, 0.5]);
  // Arm=(-1,-.5,.5), tie=(0,2,2): dot=0, hence spatial90°.
  close(result.spatialAngleDeg, 90);
  vectorClose(result.planAxisPoint, [0, 0, 0]);
  // Conventional XY vectors=(-1,-1),(0,2), so plan angle=135°.
  close(result.planAngleDeg, 135);
  const projectedFootAngle = Math.acos(-1 / Math.sqrt(5)) * 180 / Math.PI;
  assert.ok(Math.abs(result.planAngleDeg - projectedFootAngle) > 10, 'the foot projection is not the source-defined plan arm');
});

test('known infinite-line intersection and rear-axle offset are signed axle-local quantities', () => {
  const result = steeringConstruction(symmetricDesign());
  assert.equal(result.intersection.status, 'finite');
  vectorClose(result.intersection.point, [-2, 0, 0]);
  close(result.intersection.rearAxleOffsetMm, 8);
  const d = symmetricDesign(); d.wheelbase = null;
  const noWheelbase = steeringConstruction(d).intersection;
  assert.equal(noWheelbase.status, 'finite');
  vectorClose(noWheelbase.point, [-2, 0, 0]);
  assert.equal(noWheelbase.rearAxleOffsetMm, null);
});

test('parallel, coincident and zero-projection arm constructions are distinguished', () => {
  const d = symmetricDesign();
  d.left = side([0, -2], [-1, -2, 0], [-1, 0, 0]);
  d.right = side([0, 2], [-1, 2, 0], [-1, 0, 0]);
  assert.equal(steeringConstruction(d).intersection.status, 'parallel');
  d.right = side([4, -2], [3, -2, 0], [3, 0, 0]);
  assert.equal(steeringConstruction(d).intersection.status, 'coincident');
  d.left = { axisUpper: [-1, 0, 0], axisLower: [1, 0, 0], tieOuter: [0, 0, 1], tieInner: [0, 1, 1] };
  const verticalArm = steeringConstruction(d);
  close(verticalArm.left.spatialAngleDeg, 90);
  assert.equal(verticalArm.left.planAngleDeg, null);
  assert.equal(verticalArm.left.planAxisPoint, null, 'horizontal axis cannot select a unique point at joint height');
  assert.equal(verticalArm.intersection.status, 'undefined');
  assert.equal(verticalArm.intersection.point, null);
});

test('relative parallel tolerance suppresses unresolved extremely distant intersections', () => {
  const d = symmetricDesign();
  d.left = side([0, -2], [-1, -2, 0], [-1, 0, 0]);
  d.right = side([0, 2], [-1, 2 + 1e-11, 0], [-1, 0, 0]);
  assert.equal(steeringConstruction(d).intersection.status, 'parallel');
  d.right.tieOuter[1] = 2 + 1e-8;
  const distinct = steeringConstruction(d).intersection;
  assert.equal(distinct.status, 'finite');
  assert.ok(Math.abs(distinct.point[0]) > 1e8, 'well-resolved far constructions remain finite');
  assert.equal(steeringConstructionNumerics.parallelRelativeTolerance, 1e-10);
  assert.equal(steeringConstructionNumerics.axisVerticalComponentTolerance, 1e-10);
});

test('blank, malformed and degenerate geometry produce nulls without guessed points', () => {
  for (const d of [null, undefined, {}, newSteeringDesign()]) {
    const result = steeringConstruction(d);
    assert.equal(result.pose, 'static');
    assert.equal(result.left.axisFoot, null);
    assert.equal(result.left.planAngleDeg, null);
    assert.equal(result.intersection.status, 'undefined');
  }
  for (const invalid of [[0, 0], [NaN, 0, 0], [Infinity, 0, 0], ['0', 0, 0], Array(3)]) {
    const d = symmetricDesign(); d.left.axisUpper = invalid;
    assert.equal(steeringConstruction(d).left.axisFoot, null);
  }
  const d = symmetricDesign(); d.left.axisLower = [...d.left.axisUpper];
  assert.equal(steeringConstruction(d).left.axisFoot, null);
  d.left = side([0, 0], [0, 0, 0], [1, 0, 0]);
  const onAxis = steeringConstruction(d);
  assert.equal(onAxis.left.spatialAngleDeg, null);
  assert.equal(onAxis.left.planAngleDeg, null);
  assert.equal(onAxis.intersection.status, 'undefined');
  d.left = side([0, 0], [1, 0, 0], [1, 0, 0]);
  assert.equal(steeringConstruction(d).left.spatialAngleDeg, null);
});

test('mirror reflection exchanges sides and preserves included angles', () => {
  const d = symmetricDesign();
  d.left.tieOuter[0] = -1.3; d.left.tieInner[2] = 0.7;
  const a = steeringConstruction(d), reflected = structuredClone(d);
  const mirror = p => [p[0], -p[1], p[2]];
  for (const name of ['left', 'right']) {
    const source = d[name === 'left' ? 'right' : 'left'];
    for (const key of ['axisUpper', 'axisLower', 'tieOuter', 'tieInner']) reflected[name][key] = mirror(source[key]);
  }
  const b = steeringConstruction(reflected);
  close(a.left.spatialAngleDeg, b.right.spatialAngleDeg);
  close(a.left.planAngleDeg, b.right.planAngleDeg);
  vectorClose(b.intersection.point, mirror(a.intersection.point));
  close(a.intersection.rearAxleOffsetMm, b.intersection.rearAxleOffsetMm);
});

test('scale and translation preserve angles and transform construction points', () => {
  const d = symmetricDesign(), original = steeringConstruction(d);
  for (const scale of [0.001, 1000]) {
    const transformed = structuredClone(d), translation = [30000, -40000, 50000];
    for (const name of ['left', 'right']) for (const key of ['axisUpper', 'axisLower', 'tieOuter', 'tieInner']) {
      transformed[name][key] = d[name][key].map((value, i) => scale * value + translation[i]);
    }
    transformed.wheelbase *= scale;
    const result = steeringConstruction(transformed);
    close(result.left.spatialAngleDeg, original.left.spatialAngleDeg, 1e-6);
    close(result.left.planAngleDeg, original.left.planAngleDeg, 1e-6);
    vectorClose(result.intersection.point, [original.intersection.point[0] * scale + translation[0], original.intersection.point[1] * scale + translation[1], 0], 1e-6);
    close(result.intersection.rearAxleOffsetMm, result.intersection.point[0] + transformed.wheelbase);
  }
});

test('neutral and moving solved poses use solver joint coordinates without mutating data', () => {
  const d = exampleSteeringDesign(), before = JSON.stringify(d);
  const staticConstruction = steeringConstruction(d), neutral = solveSteering(d, 0);
  assert.equal(neutral.ok, true);
  const neutralConstruction = steeringConstruction(d, neutral);
  assert.equal(neutralConstruction.pose, 'solved');
  assert.deepEqual(neutralConstruction.left, staticConstruction.left);
  assert.deepEqual(neutralConstruction.intersection, staticConstruction.intersection);
  close(staticConstruction.intersection.point[0], -550 * 100 / 30);
  const moving = solveSteering(d, 180);
  assert.equal(moving.ok, true);
  const dynamic = steeringConstruction(d, moving);
  vectorClose(dynamic.left.outer, moving.left.outer);
  vectorClose(dynamic.left.inner, moving.left.inner);
  vectorClose(dynamic.left.axisFoot, staticConstruction.left.axisFoot);
  assert.ok(Math.abs(dynamic.left.planAngleDeg - staticConstruction.left.planAngleDeg) > 1);
  assert.ok(Math.abs(dynamic.intersection.point[1]) > 1, 'dynamic arm lines need not meet at centerline');
  assert.equal(JSON.stringify(d), before);
  dynamic.left.outer[0] = 999;
  assert.notEqual(moving.left.outer[0], 999, 'returned points do not alias solver arrays');
});

test('failed solve uses reference construction and malformed successful solve stays unavailable', () => {
  const d = exampleSteeringDesign();
  const fallback = steeringConstruction(d, { ok: false, left: { outer: [999, 999, 999] } });
  assert.deepEqual(fallback, steeringConstruction(d));
  const malformed = steeringConstruction(d, { ok: true });
  assert.equal(malformed.pose, 'solved');
  assert.equal(malformed.left.outer, null);
  assert.equal(malformed.intersection.status, 'undefined');
});

test('inclined arm-line intersection uses joint-height axis points, not projected feet', () => {
  const d = symmetricDesign();
  d.left.axisUpper = [0, -2, 0]; d.left.axisLower = [0, -1, 1];
  d.right.axisUpper = [0, 2, 0]; d.right.axisLower = [0, 1, 1];
  const result = steeringConstruction(d);
  vectorClose(result.left.axisFoot, [0, -1.5, 0.5]);
  vectorClose(result.left.planAxisPoint, [0, -2, 0]);
  vectorClose(result.right.axisFoot, [0, 1.5, 0.5]);
  vectorClose(result.right.planAxisPoint, [0, 2, 0]);
  vectorClose(result.intersection.point, [-2, 0, 0]);
  assert.notEqual(result.intersection.point[0], -3, 'projected perpendicular-foot lines would incorrectly meet at X=-3');
});

test('nearly horizontal steering axes leave conventional plan construction unavailable', () => {
  const d = symmetricDesign();
  d.left.axisUpper = [-1, 0, 0]; d.left.axisLower = [1, 0, 1e-12];
  d.left.tieOuter = [0, 0, 1]; d.left.tieInner = [0, 1, 1];
  const result = steeringConstruction(d);
  assert.notEqual(result.left.axisFoot, null);
  close(result.left.spatialAngleDeg, 90);
  assert.equal(result.left.planAxisPoint, null);
  assert.equal(result.left.planAngleDeg, null);
  assert.equal(result.intersection.status, 'undefined');
});


test('axis endpoint span does not create an arbitrary arm-angle cutoff', () => {
  const d = symmetricDesign();
  d.left.tieOuter = [1e-8, -2, 0]; d.left.tieInner = [1e-8, -1, 0];
  const original = steeringConstruction(d);
  assert.notEqual(original.left.spatialAngleDeg, null);
  assert.notEqual(original.left.planAngleDeg, null);
  d.left.axisUpper[2] *= 1e6; d.left.axisLower[2] *= 1e6;
  const extended = steeringConstruction(d);
  close(extended.left.spatialAngleDeg, original.left.spatialAngleDeg);
  close(extended.left.planAngleDeg, original.left.planAngleDeg);
  vectorClose(extended.left.axisFoot, original.left.axisFoot);
  vectorClose(extended.left.planAxisPoint, original.left.planAxisPoint);
  d.left = { axisUpper: [0, 0, 0], axisLower: [0, 1, 1], tieOuter: [1, 1, 0], tieInner: [1, 3, 2] };
  const inclined = steeringConstruction(d).left;
  d.left.axisUpper = [0, -1e6, -1e6]; d.left.axisLower = [0, 1e6, 1e6];
  const extendedInclined = steeringConstruction(d).left;
  close(extendedInclined.spatialAngleDeg, inclined.spatialAngleDeg, 1e-7);
  close(extendedInclined.planAngleDeg, inclined.planAngleDeg, 1e-7);
});

test('on-axis floating-point residual cannot fabricate a finite plan-arm intersection', () => {
  const d = symmetricDesign();
  d.left.axisUpper = [4.426767549011856, 7.615198688581586, 0.9577909926883876];
  d.left.axisLower = [13.839551915880293, 14.874140101019293, 2.773008595686406];
  d.left.tieOuter = [11.431112197229188, 13.016802024242264, 2.3085506836956355];
  d.left.tieInner = [5, 6, 7];
  const result = steeringConstruction(d);
  assert.equal(result.left.spatialAngleDeg, null);
  assert.equal(result.left.planAxisPoint, null);
  assert.equal(result.left.planAngleDeg, null);
  assert.equal(result.intersection.status, 'undefined');
  d.left.tieInner = null;
  assert.equal(steeringConstruction(d).intersection.status, 'undefined', 'degenerate arm-line rejection is independent of tie-joint availability');
});
