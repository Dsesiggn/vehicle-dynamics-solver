import test from 'node:test';
import assert from 'node:assert/strict';
import { newDesign, validateDesign } from '../src/model.js';
import { importDesign } from '../src/migration.js';
import { solveAxle } from '../src/solver.js';
import { readFileSync } from 'node:fs';

const legacyV1 = () => JSON.parse(readFileSync(new URL('./fixtures/design-v1.json', import.meta.url), 'utf8'));

test('new designs default to independent editable 25.4 mm heave magnitudes', () => {
  const design = newDesign();
  assert.deepEqual(design.heaveTravel, { compression: 25.4, rebound: 25.4 });
  assert.deepEqual(design.interference, { linkOD: null });
  design.heaveTravel.compression = 30;
  assert.deepEqual(validateDesign(design), []);
});

test('older version 1 and version 2 designs receive defaults at import, without mutating input', () => {
  const v1 = legacyV1(), beforeV1 = JSON.stringify(v1);
  assert.deepEqual(importDesign(v1).design.heaveTravel, { compression: 25.4, rebound: 25.4 });
  assert.deepEqual(importDesign(v1).design.interference, { linkOD: null });
  assert.equal(JSON.stringify(v1), beforeV1);

  const v2 = newDesign();
  delete v2.heaveTravel;
  assert.deepEqual(importDesign(v2).design.heaveTravel, { compression: 25.4, rebound: 25.4 });
  assert.deepEqual(importDesign(v2).design.interference, { linkOD: null });
  assert.equal(Object.hasOwn(v2, 'heaveTravel'), false);
});

test('heave magnitudes are validated against solver support and reject unknown settings', () => {
  const design = newDesign();
  design.heaveTravel.compression = 100.1;
  assert.ok(validateDesign(design).some(error => error.includes('Heave compression')));
  design.heaveTravel.compression = 25.4;
  design.heaveTravel.extra = 1;
  assert.ok(validateDesign(design).some(error => error.includes('Heave travel')));
  delete design.heaveTravel.extra;
  design.heaveTravel.rebound = null;
  assert.ok(validateDesign(design).some(error => error.includes('Heave rebound')));
  design.heaveTravel.rebound = 25.4;
  design.interference.linkOD = 0;
  assert.ok(validateDesign(design).some(error => error.includes('link OD')));
  design.interference.linkOD = null;
  design.interference.extra = 1;
  assert.ok(validateDesign(design).some(error => error.includes('Interference settings')));
});

test('positive bump is compression upward in negative Z; negative bump is rebound downward', () => {
  const axle = newDesign().axles.front, staticZ = axle.hardpoints.wheel_center[2];
  const compression = solveAxle(axle, 25.4, 0), rebound = solveAxle(axle, -25.4, 0);
  assert.ok(compression.ok, compression.left?.reason || compression.right?.reason);
  assert.ok(rebound.ok, rebound.left?.reason || rebound.right?.reason);
  assert.ok(Math.abs(compression.left.points.wheel_center[2] - (staticZ - 25.4)) < 1e-6);
  assert.ok(Math.abs(rebound.left.points.wheel_center[2] - (staticZ + 25.4)) < 1e-6);
});
