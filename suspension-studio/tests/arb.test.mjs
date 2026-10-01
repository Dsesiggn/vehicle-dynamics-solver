import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { ARB_POINTS, newAntiRollBar, validateAntiRollBar, arbComplete } from '../src/arb.js';
import { axle as makeAxle, newDesign, validateDesign } from '../src/model.js';
import { importDesign, readLibrary, STORAGE } from '../src/migration.js';
import { solveAxle } from '../src/solver.js';

// Deliberately synthetic input geometry; it is not a physically validated bar design.
const completeBar = () => ({ type: 'u-bar', enabled: true, points: {
  bearing: [80, -200, -180], bend: [80, -400, -180],
  arm_tip: [-80, -400, -180], link_pickup: [-80, -450, -100],
} });

test('New ARB drafts are disabled, unentered, and independent for each axle', () => {
  const front = newAntiRollBar(), rear = newAntiRollBar();
  assert.equal(front.type, 'u-bar');
  assert.equal(front.enabled, false);
  assert.deepEqual(Object.keys(front.points), ['bearing', 'bend', 'arm_tip', 'link_pickup']);
  assert.equal(ARB_POINTS.length, 4);
  assert.ok(Object.values(front.points).every(point => point.length === 3 && point.every(value => value === null)));
  assert.deepEqual(validateAntiRollBar(front), []);
  assert.equal(arbComplete(front), false);
  front.points.bearing[0] = 100;
  assert.equal(front.points.bend[0], null);
  assert.equal(rear.points.bearing[0], null);
});

test('Enabled partial ARB drafts round-trip through JSON and the design import boundary', () => {
  const design = newDesign();
  design.axles.front.antiRollBar = newAntiRollBar();
  Object.assign(design.axles.front.antiRollBar, { enabled: true });
  design.axles.front.antiRollBar.points.bearing = [0, -220, null];
  const restored = importDesign(JSON.parse(JSON.stringify(design))).design;
  assert.deepEqual(restored.axles.front.antiRollBar, design.axles.front.antiRollBar);
  assert.deepEqual(validateDesign(restored), []);
  assert.equal(arbComplete(restored.axles.front.antiRollBar), false);
  assert.equal(restored.axles.front.antiRollBar.points.bearing[2], null);
});

test('Disabling a complete ARB preserves its points through import and saved-library reload', () => {
  const design = newDesign();
  design.axles.front.antiRollBar = completeBar();
  design.axles.rear.antiRollBar = completeBar();
  design.axles.rear.antiRollBar.points.bearing[0] = 160;
  design.axles.front.antiRollBar.enabled = false;
  const imported = importDesign(JSON.parse(JSON.stringify(design))).design;
  assert.deepEqual(imported.axles.front.antiRollBar, design.axles.front.antiRollBar);
  assert.deepEqual(imported.axles.rear.antiRollBar, design.axles.rear.antiRollBar);
  const serialized = JSON.stringify([imported]);
  const library = readLibrary({ getItem: key => key === STORAGE ? serialized : null });
  assert.equal(library.saved.length, 1);
  assert.equal(library.migrated, 0);
  assert.deepEqual(library.rejected, []);
  const restored = library.saved[0];
  assert.equal(restored.axles.front.antiRollBar.enabled, false);
  assert.equal(restored.axles.rear.antiRollBar.enabled, true);
  assert.deepEqual(restored.axles.front.antiRollBar.points, design.axles.front.antiRollBar.points);
  restored.axles.front.antiRollBar.enabled = true;
  assert.equal(arbComplete(restored.axles.front.antiRollBar), true);
  assert.deepEqual(restored.axles.rear.antiRollBar, design.axles.rear.antiRollBar);
});

test('ARB completeness accepts valid boundary values and remains separate from enabled state', () => {
  const bar = completeBar();
  assert.deepEqual(validateAntiRollBar(bar), []);
  assert.equal(arbComplete(bar), true);
  bar.enabled = false;
  bar.points.bearing = [10000, -10000, -10000];
  bar.points.bend = [0, 0, 0];
  assert.deepEqual(validateAntiRollBar(bar), []);
  assert.equal(arbComplete(bar), true);
  bar.points.arm_tip[2] = null;
  assert.equal(arbComplete(bar), false);
  assert.deepEqual(validateAntiRollBar(bar), []);
});

test('Malformed ARB inputs are rejected without throwing, even while disabled', () => {
  const variants = [null, false, 12, 'u-bar', [], {},
    { ...completeBar(), type: 'blade' }, { ...completeBar(), enabled: 1 },
    { ...completeBar(), points: null }, { ...completeBar(), points: [] },
  ];
  const change = update => { const bar = completeBar(); bar.enabled = false; update(bar); variants.push(bar); };
  change(bar => { delete bar.points.bearing; });
  change(bar => { bar.points.extra = [1, -1, 1]; });
  change(bar => { bar.points.bearing = [0, 0]; });
  change(bar => { bar.points.bearing = [0, 0, 0, 0]; });
  change(bar => { bar.points.bearing = new Array(3); });
  for (const value of ['1', true, undefined, Infinity, NaN, 10000.1, -10000.1, {}, []]) {
    change(bar => { bar.points.bearing[0] = value; });
  }
  change(bar => { bar.points.arm_tip[1] = 1; });
  change(bar => { bar.points = JSON.parse('{"__proto__":[0,0,0]}'); });
  for (const value of variants) {
    assert.doesNotThrow(() => validateAntiRollBar(value));
    assert.ok(validateAntiRollBar(value).length > 0);
    assert.equal(arbComplete(value), false);
  }
});

test('Design import rejects malformed ARB data rather than silently accepting it', () => {
  const design = newDesign();
  design.axles.front.antiRollBar = completeBar();
  design.axles.front.antiRollBar.points.link_pickup[1] = 450;
  assert.ok(validateDesign(design).some(message => /anti-roll bar/i.test(message)));
  assert.throws(() => importDesign(design), /left-side Y/i);
  design.axles.front.antiRollBar = null;
  assert.throws(() => importDesign(design), /anti-roll bar/i);
});

test('Existing v1 and v2 designs without ARB data remain importable', () => {
  assert.deepEqual(validateAntiRollBar(undefined), []);
  assert.equal(arbComplete(undefined), false);
  const v2 = newDesign();
  for (const axle of Object.values(v2.axles)) delete axle.antiRollBar;
  const importedV2 = importDesign(v2);
  assert.equal(importedV2.migrated, false);
  assert.deepEqual(importedV2.design.axles.front.hardpoints, v2.axles.front.hardpoints);
  const v1 = JSON.parse(readFileSync(new URL('./fixtures/design-v1.json', import.meta.url)));
  const importedV1 = importDesign(v1);
  assert.equal(importedV1.migrated, true);
  assert.deepEqual(validateDesign(importedV1.design), []);
  assert.equal(importedV1.design.axles.front.antiRollBar?.enabled ?? false, false);
});

test('Unsupported ARB extensions in v1 files are rejected without inferring their coordinate axes', () => {
  for (const axleName of ['front', 'rear']) {
    const v1 = JSON.parse(readFileSync(new URL('./fixtures/design-v1.json', import.meta.url)));
    v1.axles[axleName].antiRollBar = completeBar();
    const original = structuredClone(v1);
    assert.throws(() => importDesign(v1), /Version 1 files do not support ARB points/);
    assert.deepEqual(v1, original, 'Rejected import must leave its source untouched');
    const library = readLibrary({ getItem: key => key === STORAGE ? JSON.stringify([v1]) : null });
    assert.deepEqual(library.saved, []);
    assert.deepEqual(library.rejected, [original]);
  }
});

for (const topology of ['double-wishbone', 'macpherson', 'multi-link']) {
  test(`${topology}: ARB input geometry does not change solved suspension motion or compression`, () => {
    const axle = makeAxle(topology);
    const baseline = solveAxle(axle, 12, 7);
    assert.equal(baseline.ok, true);
    axle.antiRollBar = completeBar();
    assert.deepEqual(solveAxle(axle, 12, 7), baseline);
    axle.antiRollBar.points.arm_tip[0] -= 60;
    assert.deepEqual(solveAxle(axle, 12, 7), baseline);
    axle.antiRollBar.enabled = false;
    assert.deepEqual(solveAxle(axle, 12, 7), baseline);
  });
}
