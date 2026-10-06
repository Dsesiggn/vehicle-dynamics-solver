import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { axle, changeDimensions, clone, newDesign, preset, TIRE_TREAD_WIDTH_MAX, validateDesign } from '../src/model.js';
import { importDesign } from '../src/migration.js';
import { solveAxle } from '../src/solver.js';

test('new designs and all presets leave each tire tread width unknown', () => {
  for (const d of [newDesign(), preset('formula'), preset('road'), preset('cr26')]) {
    assert.equal(d.axles.front.tireTreadWidth, null);
    assert.equal(d.axles.rear.tireTreadWidth, null);
    assert.deepEqual(validateDesign(d), []);
  }
});

test('tire tread width accepts null and positive finite millimeter values up to the edit bound', () => {
  assert.equal(TIRE_TREAD_WIDTH_MAX, 10000);
  const d = newDesign();
  for (const width of [null, 0.1, 100, 254, 300, TIRE_TREAD_WIDTH_MAX]) {
    d.axles.front.tireTreadWidth = width;
    assert.deepEqual(validateDesign(d), [], `Rejected valid width ${width}`);
  }
});

test('tire tread width rejects nonpositive, nonfinite, out-of-bound, and nonnumeric values', () => {
  const d = newDesign();
  for (const width of [0, -1, NaN, Infinity, -Infinity, TIRE_TREAD_WIDTH_MAX + 0.1, '', '254', true, false, [], [254], {}, undefined]) {
    d.axles.front.tireTreadWidth = width;
    assert.ok(validateDesign(d).some(error => error.includes('front tire tread width')), `Accepted ${String(width)}`);
    assert.throws(() => importDesign(d), /front tire tread width/);
  }
});

test('front and rear tire tread widths are independent and editing them preserves hardpoints', () => {
  const d = newDesign(), before = clone(d.axles);
  changeDimensions(d.axles.front, 'tireTreadWidth', 254);
  assert.equal(d.axles.front.tireTreadWidth, 254);
  assert.equal(d.axles.rear.tireTreadWidth, null);
  assert.deepEqual(d.axles.front.hardpoints, before.front.hardpoints);
  assert.deepEqual(d.axles.rear, before.rear);
  changeDimensions(d.axles.rear, 'tireTreadWidth', 200);
  assert.equal(d.axles.front.tireTreadWidth, 254);
  assert.equal(d.axles.rear.tireTreadWidth, 200);
  assert.deepEqual(d.axles.rear.hardpoints, before.rear.hardpoints);
});

test('older v1 and v2 files receive null widths without mutation or inferred measurements', () => {
  const v1 = JSON.parse(readFileSync(new URL('./fixtures/design-v1.json', import.meta.url), 'utf8'));
  const v2 = newDesign();
  delete v2.axles.front.tireTreadWidth;
  delete v2.axles.rear.tireTreadWidth;
  assert.deepEqual(validateDesign(v2), []);
  for (const original of [v1, v2]) {
    const before = clone(original), result = importDesign(original).design;
    assert.equal(result.axles.front.tireTreadWidth, null);
    assert.equal(result.axles.rear.tireTreadWidth, null);
    assert.deepEqual(original, before);
    assert.deepEqual(importDesign(result).design, result);
  }
  assert.deepEqual(importDesign(v2).design.axles.front.hardpoints, v2.axles.front.hardpoints);
  assert.deepEqual(importDesign(v2).design.axles.rear.hardpoints, v2.axles.rear.hardpoints);
});

test('JSON export and re-import retain distinct widths and null widths', () => {
  const d = newDesign();
  d.axles.front.tireTreadWidth = 254;
  d.axles.rear.tireTreadWidth = 203.2;
  assert.deepEqual(importDesign(JSON.parse(JSON.stringify(d))).design, d);
  d.axles.rear.tireTreadWidth = null;
  assert.deepEqual(importDesign(JSON.parse(JSON.stringify(d))).design, d);
});

for (const topology of ['double-wishbone', 'macpherson', 'multi-link']) {
  test(`${topology}: tread width does not alter hardpoints or existing kinematic solutions`, () => {
    const a = axle(topology), points = clone(a.hardpoints);
    const motions = [[0, 0], [25.4, 12], [-25.4, 0]];
    const expected = motions.map(([bump, rack]) => solveAxle(a, bump, rack));
    for (const result of expected) assert.ok(result.ok, result.left.reason || result.right.reason);
    for (const width of [100, 300, null]) {
      changeDimensions(a, 'tireTreadWidth', width);
      assert.deepEqual(a.hardpoints, points);
      motions.forEach(([bump, rack], i) => assert.deepEqual(solveAxle(a, bump, rack), expected[i]));
    }
  });
}
