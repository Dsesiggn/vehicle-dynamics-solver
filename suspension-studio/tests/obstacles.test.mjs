import test from 'node:test';
import assert from 'node:assert/strict';
import { newDesign, clone, validateDesign } from '../src/model.js';
import { importDesign } from '../src/migration.js';
import { newObstacles, OBSTACLE_TYPES, validateObstacles, obstacleComplete, obstacleMesh } from '../src/obstacles.js';
import { solveAxle } from '../src/solver.js';
import { readFileSync } from 'node:fs';

const near = (actual, expected, tolerance = 1e-10) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
const configured = (kind, values = {}) => {
  const obstacles = newObstacles(), item = obstacles[kind];
  item.visible = true;
  item.center = [120, -35, -280];
  if (kind === 'cockpit') item.dimensions = { depth: 900, topWidth: 520, bottomWidth: 700, height: 600 };
  else item.dimensions = { length: 400, width: 500, height: 300 };
  Object.assign(item, values);
  return item;
};

test('new obstacle drafts are blank, hidden, and valid; old v2 designs may omit them', () => {
  const obstacles = newObstacles();
  assert.deepEqual(Object.keys(obstacles), OBSTACLE_TYPES.map(({ key }) => key));
  assert.deepEqual(validateObstacles(obstacles), []);
  assert.deepEqual(validateObstacles(undefined), []);
  for (const obstacle of Object.values(obstacles)) {
    assert.equal(obstacle.visible, false);
    assert.deepEqual(obstacle.center, [null, null, null]);
    assert.ok(Object.values(obstacle.dimensions).every(value => value === null));
  }
  const legacyV2 = newDesign();
  delete legacyV2.obstacles;
  assert.deepEqual(validateDesign(legacyV2), []);
  assert.equal(Object.hasOwn(importDesign(legacyV2).design, 'obstacles'), false);
});

test('box envelopes use center plus or minus half of each SAE dimension', () => {
  const item = configured('engine'), mesh = obstacleMesh(item, 'engine', 'front', 1530);
  assert.equal(mesh.vertices.length, 8);
  assert.equal(mesh.faces.length, 6);
  assert.equal(mesh.edges.length, 12);
  const bounds = [0, 1, 2].map(axis => [Math.min(...mesh.vertices.map(p => p[axis])), Math.max(...mesh.vertices.map(p => p[axis]))]);
  assert.deepEqual(bounds, [[-80, 320], [-285, 215], [-430, -130]]);
  for (const [from, to] of mesh.edges) assert.ok([from, to].every(i => i >= 0 && i < mesh.vertices.length));
});

test('cockpit has a top-narrow trapezoid in the Y-Z section, extruded along X', () => {
  const mesh = obstacleMesh(configured('cockpit'), 'cockpit', 'front', 1530);
  const expected = [
    [-330, -295, -580], [-330, 225, -580], [-330, 315, 20], [-330, -385, 20],
    [570, -295, -580], [570, 225, -580], [570, 315, 20], [570, -385, 20],
  ];
  mesh.vertices.forEach((point, index) => point.forEach((value, axis) => near(value, expected[index][axis])));
  assert.equal(mesh.faces.length, 6);
  assert.equal(mesh.edges.length, 12);
});

test('front and rear preview conversion preserves a shared physical X coordinate', () => {
  const wheelbase = 1530;
  const frontReferenced = configured('engine', { center: [350, -20, -250], referenceAxle: 'front' });
  const fromFront = obstacleMesh(frontReferenced, 'engine', 'front', wheelbase);
  const fromRear = obstacleMesh(frontReferenced, 'engine', 'rear', wheelbase);
  for (let i = 0; i < fromFront.vertices.length; i++) {
    near(fromRear.vertices[i][0], fromFront.vertices[i][0] + wheelbase);
    near(fromRear.vertices[i][1], fromFront.vertices[i][1]);
    near(fromRear.vertices[i][2], fromFront.vertices[i][2]);
  }
  const rearReferenced = configured('differential', { center: [-120, 20, -200], referenceAxle: 'rear' });
  const fromRearReference = obstacleMesh(rearReferenced, 'differential', 'rear', wheelbase);
  const fromFrontView = obstacleMesh(rearReferenced, 'differential', 'front', wheelbase);
  for (let i = 0; i < fromRearReference.vertices.length; i++) near(fromFrontView.vertices[i][0], fromRearReference.vertices[i][0] - wheelbase);
});

test('hidden and incomplete envelopes produce no mesh without substituting zeroes', () => {
  const incomplete = newObstacles().cockpit;
  incomplete.visible = true;
  assert.equal(obstacleComplete(incomplete, 'cockpit'), false);
  assert.equal(obstacleMesh(incomplete, 'cockpit', 'front', 1530), null);
  const hidden = configured('engine');
  hidden.visible = false;
  assert.equal(obstacleComplete(hidden, 'engine'), true);
  assert.equal(obstacleMesh(hidden, 'engine', 'front', 1530), null);
  assert.equal(obstacleMesh(configured('engine'), 'engine', 'front', null), null);
});

test('validator rejects bad coordinates, nonpositive dimensions, unknown properties, and partial collections', () => {
  const obstacles = newObstacles();
  obstacles.engine.center[0] = Infinity;
  assert.ok(validateObstacles(obstacles).some(error => error.includes('center coordinates')));
  obstacles.engine.center[0] = null;
  obstacles.engine.dimensions.length = 0;
  assert.ok(validateObstacles(obstacles).some(error => error.includes('0.001')));
  obstacles.engine.dimensions.length = null;
  obstacles.extra = {};
  assert.ok(validateObstacles(obstacles).some(error => error.includes('Unexpected obstacle')));
  assert.ok(validateObstacles({ engine: newObstacles().engine }).some(error => error.includes('Differential is missing')));
});

test('obstacle draft survives JSON import/export and remains outside the kinematic solver', () => {
  const design = newDesign();
  design.obstacles.engine = configured('engine', { center: [250, -15, -300] });
  const serialized = JSON.parse(JSON.stringify(design));
  const imported = importDesign(serialized).design;
  assert.deepEqual(imported.obstacles, serialized.obstacles);
  assert.deepEqual(validateDesign(imported), []);
  const plain = clone(design);
  delete plain.obstacles;
  const before = solveAxle(plain.axles.front, 12, 4);
  const after = solveAxle(imported.axles.front, 12, 4);
  assert.deepEqual(after, before);
});

test('schema v1 obstacle coordinates are rejected instead of assigned an unrecorded frame', () => {
  const legacy = JSON.parse(readFileSync(new URL('./fixtures/design-v1.json', import.meta.url), 'utf8'));
  legacy.obstacles = newObstacles();
  assert.throws(() => importDesign(legacy), /Version 1 files do not support obstacle geometry/);
});
