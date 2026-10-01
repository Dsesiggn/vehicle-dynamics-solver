import test from 'node:test';
import assert from 'node:assert/strict';
import { segmentMeshDistance, segmentSegmentDistance } from '../src/geometry-distance.js';
import { newObstacles, obstacleMesh } from '../src/obstacles.js';

const near = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);

function envelope(kind) {
  const obstacle = newObstacles()[kind];
  obstacle.visible = true;
  obstacle.center = [120, -35, -280];
  obstacle.dimensions = kind === 'cockpit'
    ? { depth: 900, topWidth: 520, bottomWidth: 700, height: 600 }
    : { length: 400, width: 500, height: 300 };
  return obstacleMesh(obstacle, kind, 'front', 1530);
}

function independentSegmentAabbDistance(p, q, bounds) {
  const direction = q.map((value, i) => value - p[i]), breaks = [0, 1];
  for (let axis = 0; axis < 3; axis++) if (direction[axis] !== 0) {
    for (const bound of bounds[axis]) {
      const t = (bound - p[axis]) / direction[axis];
      if (t > 0 && t < 1) breaks.push(t);
    }
  }
  breaks.sort((a, b) => a - b);
  const distanceSquared = t => p.reduce((sum, value, axis) => {
    const coordinate = value + direction[axis] * t, [minimum, maximum] = bounds[axis];
    const delta = coordinate < minimum ? coordinate - minimum : coordinate > maximum ? coordinate - maximum : 0;
    return sum + delta * delta;
  }, 0);
  let best = Math.min(...breaks.map(distanceSquared));
  for (let index = 0; index < breaks.length - 1; index++) {
    const low = breaks[index], high = breaks[index + 1], middle = (low + high) / 2, numerator = [], denominator = [];
    for (let axis = 0; axis < 3; axis++) {
      const coordinate = p[axis] + direction[axis] * middle, [minimum, maximum] = bounds[axis];
      const activeBound = coordinate < minimum ? minimum : coordinate > maximum ? maximum : null;
      if (activeBound !== null) { numerator.push(direction[axis] * (activeBound - p[axis])); denominator.push(direction[axis] ** 2); }
    }
    const stationary = denominator.reduce((sum, value) => sum + value, 0) ?
      numerator.reduce((sum, value) => sum + value, 0) / denominator.reduce((sum, value) => sum + value, 0) : middle;
    best = Math.min(best, distanceSquared(Math.max(low, Math.min(high, stationary))));
  }
  return Math.sqrt(best);
}

test('segment-to-box distance detects a centerline passing through the solid', () => {
  near(segmentMeshDistance([-500, -35, -280], [500, -35, -280], envelope('engine')), 0);
});

test('parallel link over a box face uses face-interior distance, not edge distance', () => {
  // The engine top is at Z=-130; the projected link lies inside that face.
  near(segmentMeshDistance([-50, -100, -120], [250, -100, -120], envelope('engine')), 10);
});

test('parallel link beside a box side has the correct lateral clearance', () => {
  // The engine right-side Y boundary is 215 mm.
  near(segmentMeshDistance([-50, 225, -280], [250, 225, -280], envelope('engine')), 10);
});

test('point-like segment outside a box corner returns the 3D Euclidean distance', () => {
  const closestCorner = [320, 215, -430];
  const point = [323, 219, -442];
  assert.deepEqual(closestCorner.map((v, i) => point[i] - v), [3, 4, -12]);
  near(segmentMeshDistance(point, point, envelope('engine')), 13);
});

test('segment fully inside the cockpit solid has zero centerline-to-solid distance', () => {
  const inside = [120, -35, -280];
  near(segmentMeshDistance(inside, [inside[0] + 10, inside[1], inside[2]], envelope('cockpit')), 0);
});

test('segment distance is invariant to endpoint ordering and reports invalid input', () => {
  const mesh = envelope('differential'), a = [-200, 500, -200], b = [300, 500, -200];
  near(segmentMeshDistance(a, b, mesh), segmentMeshDistance(b, a, mesh));
  assert.ok(Number.isNaN(segmentMeshDistance([0, 0], [0, 0, 0], mesh)));
});

test('finite link-axis distance handles parallel, crossing, and separated endpoint cases', () => {
  near(segmentSegmentDistance([0, 0, 0], [10, 0, 0], [0, 10, 0], [10, 10, 0]), 10);
  near(segmentSegmentDistance([0, 0, 0], [10, 0, 0], [5, -2, 0], [5, 2, 0]), 0);
  near(segmentSegmentDistance([0, 0, 0], [1, 0, 0], [4, 4, 0], [4, 5, 0]), 5);
});

test('box-mesh distances agree with an independent piecewise segment-to-AABB minimization', () => {
  const mesh = envelope('engine');
  const bounds = [0, 1, 2].map(axis => [Math.min(...mesh.vertices.map(point => point[axis])), Math.max(...mesh.vertices.map(point => point[axis]))]);
  let seed = 1729;
  const random = () => { seed = (1664525 * seed + 1013904223) >>> 0; return seed / 0x100000000; };
  for (let index = 0; index < 100; index++) {
    const p = [0, 1, 2].map(() => -700 + random() * 1400), q = [0, 1, 2].map(() => -700 + random() * 1400);
    const independent = independentSegmentAabbDistance(p, q, bounds);
    near(segmentMeshDistance(p, q, mesh), independent, 1e-7);
  }
});
