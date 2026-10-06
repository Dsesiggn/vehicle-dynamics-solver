import test from 'node:test';
import assert from 'node:assert/strict';
import { tireTreadGeometry } from '../src/tire-geometry.js';
import { TIRE_TREAD_WIDTH_MAX } from '../src/model.js';

const near = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
const vectorNear = (actual, expected, tolerance) => actual.forEach((value, i) => near(value, expected[i], tolerance));
const subtract = (a, b) => a.map((value, i) => value - b[i]);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const normalized = vector => vector.map(value => value / Math.hypot(...vector));

test('axis-aligned tread band agrees with hand-calculated edge positions and bounds in millimeters', () => {
  const geometry = tireTreadGeometry([100, -400, -250], [0, -2, 0], 500, 200);
  assert.equal(geometry.widthKnown, true);
  assert.equal(geometry.radius, 250);
  assert.deepEqual(geometry.edgeCenters, [[100, -300, -250], [100, -500, -250]]);
  assert.deepEqual(geometry.bounds, { min: [-150, -500, -500], max: [350, -300, 0] });
});

test('rotated tread band has edge separation W, midpoint C, and radial diameter D', () => {
  const center = [37, -610, -230], geometry = tireTreadGeometry(center, [2, -3, 6], 508, 203.2);
  vectorNear(geometry.axis, [2/7, -3/7, 6/7]);
  near(Math.hypot(...geometry.axis), 1);
  const [first, second] = geometry.edgeCenters;
  near(Math.hypot(...subtract(second, first)), 203.2);
  vectorNear(first.map((value, i) => (value + second[i]) / 2), center);
  const radial = normalized(cross(geometry.axis, [1, 0, 0]));
  const outerPoint = first.map((value, i) => value + geometry.radius * radial[i]);
  near(Math.hypot(...subtract(outerPoint, first)), 508/2);
  near(dot(subtract(outerPoint, first), geometry.axis), 0);
});

test('exact rotated bounds enclose dense independently parameterized circumference samples', () => {
  const geometry = tireTreadGeometry([37, -610, -230], [2, -3, 6], 508, 203.2);
  // Construct an independent orthonormal basis and sample both band edges.
  const u = normalized(cross(geometry.axis, [1, 0, 0])), v = cross(geometry.axis, u);
  const minimum = [Infinity, Infinity, Infinity], maximum = [-Infinity, -Infinity, -Infinity];
  for (const edgeCenter of geometry.edgeCenters) {
    for (let sample = 0; sample < 16384; sample++) {
      const angle = sample * 2 * Math.PI / 16384;
      const point = edgeCenter.map((value, i) => value + geometry.radius * (u[i] * Math.cos(angle) + v[i] * Math.sin(angle)));
      for (let i = 0; i < 3; i++) {
        assert.ok(point[i] >= geometry.bounds.min[i] - 1e-9 && point[i] <= geometry.bounds.max[i] + 1e-9);
        minimum[i] = Math.min(minimum[i], point[i]);
        maximum[i] = Math.max(maximum[i], point[i]);
      }
    }
  }
  vectorNear(minimum, geometry.bounds.min, 0.00001);
  vectorNear(maximum, geometry.bounds.max, 0.00001);
});

test('unknown tread width retains only the diameter circle and never guesses band edges', () => {
  for (const geometry of [tireTreadGeometry([100, -400, -250], [0, -2, 0], 500), tireTreadGeometry([100, -400, -250], [0, -2, 0], 500, null)]) {
    assert.equal(geometry.widthKnown, false);
    assert.deepEqual(geometry.edgeCenters, []);
    assert.deepEqual(geometry.bounds, { min: [-150, -400, -500], max: [350, -400, 0] });
  }
});

test('width validation shares the application edit bound, which is not a physical tire limit', () => {
  assert.ok(tireTreadGeometry([0, 0, 0], [0, 1, 0], 500, TIRE_TREAD_WIDTH_MAX));
  for (const width of [0, -1, NaN, Infinity, -Infinity, TIRE_TREAD_WIDTH_MAX+1, '200', true, [], {}]) {
    assert.equal(tireTreadGeometry([0, 0, 0], [0, 1, 0], 500, width), null, `Accepted width ${String(width)}`);
  }
});

test('invalid centers, axes, and diameters return no preview geometry', () => {
  for (const center of [null, [], [0, 0], [0, 0, 0, 0], [0, NaN, 0], [0, Infinity, 0], ['0', 0, 0]]) {
    assert.equal(tireTreadGeometry(center, [0, 1, 0], 500, 200), null);
  }
  for (const axis of [null, [], [0, 1], [0, 0, 0], [0, Infinity, 0], [0, NaN, 1], [0, '1', 0]]) {
    assert.equal(tireTreadGeometry([0, 0, 0], axis, 500, 200), null);
  }
  for (const diameter of [0, -1, NaN, Infinity, -Infinity, '500', null, undefined]) {
    assert.equal(tireTreadGeometry([0, 0, 0], [0, 1, 0], diameter, 200), null);
  }
});

test('finite very large and very small axis vectors normalize without overflow or underflow', () => {
  for (const scale of [1e308, 1e-308]) {
    const geometry = tireTreadGeometry([0, 0, 0], [scale, -scale, scale], 500, 200);
    vectorNear(geometry.axis, [1/Math.sqrt(3), -1/Math.sqrt(3), 1/Math.sqrt(3)]);
  }
  assert.equal(tireTreadGeometry([Number.MAX_VALUE, 0, 0], [0, 1, 0], Number.MAX_VALUE, 200), null);
});

test('left-right reflection preserves tread width and mirrors center, axis, edges, and bounds', () => {
  const reflect = vector => [vector[0], -vector[1], vector[2]];
  const center = [37, -610, -230], axis = [2, -3, 6];
  const original = tireTreadGeometry(center, axis, 508, 203.2);
  const mirrored = tireTreadGeometry(reflect(center), reflect(axis), 508, 203.2);
  vectorNear(mirrored.center, reflect(original.center));
  vectorNear(mirrored.axis, reflect(original.axis));
  original.edgeCenters.forEach((edge, i) => vectorNear(mirrored.edgeCenters[i], reflect(edge)));
  vectorNear(mirrored.bounds.min, [original.bounds.min[0], -original.bounds.max[1], original.bounds.min[2]]);
  vectorNear(mirrored.bounds.max, [original.bounds.max[0], -original.bounds.min[1], original.bounds.max[2]]);
  near(Math.hypot(...subtract(...mirrored.edgeCenters)), 203.2);
});

test('geometry does not mutate input vectors and returns independently owned vectors', () => {
  const center = Object.freeze([37, -610, -230]), axis = Object.freeze([2, -3, 6]);
  const geometry = tireTreadGeometry(center, axis, 508, 203.2);
  assert.deepEqual(center, [37, -610, -230]);
  assert.deepEqual(axis, [2, -3, 6]);
  geometry.center[0] = 999;
  geometry.axis[0] = 999;
  geometry.edgeCenters[0][0] = 999;
  assert.equal(center[0], 37);
  assert.equal(axis[0], 2);
});
