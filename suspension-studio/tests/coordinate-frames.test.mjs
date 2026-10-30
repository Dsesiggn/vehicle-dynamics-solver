import test from 'node:test';
import assert from 'node:assert/strict';
import { axlePointToFront, frontPointToAxle } from '../src/coordinate-frames.js';

const close = (actual, expected, tolerance = 1e-10) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
const distance = (a, b) => Math.hypot(...a.map((value, index) => value - b[index]));

test('rear datum and hardpoints use the verified negative-wheelbase translation', () => {
  assert.deepEqual(axlePointToFront([0, 0, 0], 'rear', 1530), [-1530, 0, 0]);
  assert.deepEqual(axlePointToFront([0, -608.05, -203.2], 'rear', 1530), [-1530, -608.05, -203.2]);
  assert.deepEqual(axlePointToFront([82.2, -279.4, -269.75], 'rear', 1530), [-1447.8, -279.4, -269.75]);
  assert.deepEqual(frontPointToAxle([-1530, -608.05, -203.2], 'rear', 1530), [0, -608.05, -203.2]);
  const fore = frontPointToAxle([-1447.8, -279.4, -269.75], 'rear', 1530);
  close(fore[0], 82.2);
  assert.deepEqual(fore.slice(1), [-279.4, -269.75]);
});

test('front coordinates are copied exactly without aliasing or mutating inputs', () => {
  const point = Object.freeze([127.123456789, -616.8, -203.2]);
  for (const convert of [axlePointToFront, frontPointToAxle]) {
    const result = convert(point, 'front', 1530);
    assert.deepEqual(result, point);
    assert.notStrictEqual(result, point);
    result[0] = 0;
    assert.equal(point[0], 127.123456789);
  }
});

test('front/rear round trips preserve coordinates within floating-point translation precision', () => {
  for (const wheelbase of [500, 1530, 10000]) {
    for (const axle of ['front', 'rear']) {
      for (const point of [[0, 0, 0], [127.123456789, -616.8, -203.2], [-9845.6789, 10000, -10000]]) {
        const snapshot = [...point];
        const fromLocal = frontPointToAxle(axlePointToFront(point, axle, wheelbase), axle, wheelbase);
        const fromFront = axlePointToFront(frontPointToAxle(point, axle, wheelbase), axle, wheelbase);
        point.forEach((value, index) => { close(fromLocal[index], value); close(fromFront[index], value); });
        assert.deepEqual(point, snapshot);
      }
    }
  }
});

test('a common rear translation preserves link vectors, distances and angles', () => {
  const local = [[82.2, -279.4, -269.75], [-128.95, -247.78, -271.48], [-19.4, -488.95, -280.75]];
  const front = local.map(point => axlePointToFront(point, 'rear', 1530));
  const vector = (points, index) => points[index].map((value, axis) => value - points[0][axis]);
  const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
  const cosine = points => dot(vector(points, 1), vector(points, 2)) / (Math.hypot(...vector(points, 1)) * Math.hypot(...vector(points, 2)));
  for (const index of [1, 2]) {
    close(distance(front[0], front[index]), distance(local[0], local[index]));
    vector(local, index).forEach((value, axis) => close(vector(front, index)[axis], value));
  }
  close(cosine(front), cosine(local), 1e-14);
});

test('unknown ARB and obstacle coordinates remain null through both adapters', () => {
  for (const convert of [axlePointToFront, frontPointToAxle]) {
    for (const axle of ['front', 'rear']) {
      assert.deepEqual(convert([null, null, null], axle, 1530), [null, null, null]);
      assert.deepEqual(convert([null, -238.33, null], axle, 1530), [null, -238.33, null]);
    }
  }
  assert.deepEqual(axlePointToFront([3.18, null, -557.79], 'rear', 1530), [-1526.82, null, -557.79]);
  assert.deepEqual(frontPointToAxle([-1530, null, null], 'rear', 1530), [0, null, null]);
});

test('frame adapters reject malformed or nonfinite inputs without coercing or overflowing', () => {
  for (const convert of [axlePointToFront, frontPointToAxle]) {
    for (const axle of ['', 'left', 'FRONT', null, undefined]) assert.throws(() => convert([0, 0, 0], axle, 1530), TypeError);
    for (const wheelbase of [0, -1530, '1530', null, undefined, NaN, Infinity, -Infinity]) assert.throws(() => convert([0, 0, 0], 'front', wheelbase), RangeError);
    for (const point of [null, {}, [], [0, 0], [0, 0, 0, 0], Array(3), ['0', 0, 0], [false, 0, 0], [0, undefined, 0], [0, 0, NaN], [Infinity, 0, 0]]) {
      assert.throws(() => convert(point, 'rear', 1530), TypeError);
    }
  }
  assert.throws(() => frontPointToAxle([Number.MAX_VALUE, 0, 0], 'rear', Number.MAX_VALUE), RangeError);
  assert.throws(() => axlePointToFront([-Number.MAX_VALUE, 0, 0], 'rear', Number.MAX_VALUE), RangeError);
});
