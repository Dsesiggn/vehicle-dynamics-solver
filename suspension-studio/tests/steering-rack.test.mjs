import test from 'node:test';
import assert from 'node:assert/strict';
import { rackLongitudinalOffset, setRackLongitudinalOffset } from '../src/steering-rack.js';
import { exampleSteeringDesign, newSteeringDesign, validateSteeringDesign, solveSteering } from '../src/steering.js';

const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b} (tolerance ${tolerance})`);
const apply = (d, offset, options) => { const result = setRackLongitudinalOffset(d, offset, options); assert.equal(result.ok, true, result.reason); return result; };
const deepFreeze = value => {
  if (value && typeof value === 'object') {
    Object.values(value).forEach(deepFreeze);
    Object.freeze(value);
  }
  return value;
};

test('rack offset derives from entered inner-joint X coordinates and positive X is forward', () => {
  const d = exampleSteeringDesign();
  assert.equal(rackLongitudinalOffset(d), -100);
  d.left.tieInner[0] = -120; d.right.tieInner[0] = -80;
  assert.equal(rackLongitudinalOffset(d), -100);
  for (const target of [-250, 0, 175]) {
    const result = apply(d, target);
    assert.equal(result.mode, 'translate');
    assert.equal(result.offset, target);
    close(rackLongitudinalOffset(result.design), target);
  }
});

test('common X translation preserves asymmetry, every Y/Z and all unrelated fields', () => {
  const d = exampleSteeringDesign();
  d.left.tieInner = [-133.25, -246.5, -271];
  d.right.tieInner = [-82.75, 254, -249.125];
  d.notes = { measurement: 'Entered asymmetric rack', ridePosition: [1, 2, 3] };
  const before = JSON.stringify(d), offset = 55.5;
  const result = apply(d, offset), changed = result.design;
  const delta = offset - (-133.25 - 82.75) / 2;
  close(changed.left.tieInner[0], d.left.tieInner[0] + delta);
  close(changed.right.tieInner[0], d.right.tieInner[0] + delta);
  close(changed.right.tieInner[0] - changed.left.tieInner[0], d.right.tieInner[0] - d.left.tieInner[0]);
  assert.deepEqual(changed.left.tieInner.slice(1), d.left.tieInner.slice(1));
  assert.deepEqual(changed.right.tieInner.slice(1), d.right.tieInner.slice(1));
  const restored = structuredClone(changed);
  restored.left.tieInner[0] = d.left.tieInner[0]; restored.right.tieInner[0] = d.right.tieInner[0];
  assert.deepEqual(restored, d);
  assert.equal(JSON.stringify(d), before);
  assert.notEqual(changed, d); assert.notEqual(changed.left, d.left); assert.notEqual(changed.right, d.right);
  assert.notEqual(changed.left.tieInner, d.left.tieInner); assert.notEqual(changed.right.tieInner, d.right.tieInner);
});

test('zero offset is explicit and translation leaves frozen input intact', () => {
  const d = deepFreeze(exampleSteeringDesign());
  const result = apply(d, 0);
  assert.equal(result.design.left.tieInner[0], 0);
  assert.equal(result.design.right.tieInner[0], 0);
  assert.equal(d.left.tieInner[0], -100);
  const unchanged = apply(d, -100);
  assert.deepEqual(unchanged.design, d);
});

test('unknown X requires explicit set-both mode without filling unknown Y or Z', () => {
  const d = newSteeringDesign();
  assert.equal(rackLongitudinalOffset(d), null);
  assert.equal(setRackLongitudinalOffset(d, -80).ok, false);
  const result = apply(d, -80, { mode: 'set-both' });
  assert.equal(result.mode, 'set-both');
  assert.deepEqual(result.design.left.tieInner, [-80, null, null]);
  assert.deepEqual(result.design.right.tieInner, [-80, null, null]);
  assert.deepEqual(d.left.tieInner, [null, null, null]);
  assert.deepEqual(validateSteeringDesign(result.design, { allowIncomplete: true }), []);
  d.left.tieInner = [-120, -250, -240];
  const partial = apply(d, 0, { mode: 'set-both' });
  assert.deepEqual(partial.design.left.tieInner, [0, -250, -240]);
  assert.deepEqual(partial.design.right.tieInner, [0, null, null]);
});

test('set-both deliberately removes existing longitudinal X difference', () => {
  const d = exampleSteeringDesign();
  d.left.tieInner[0] = -140; d.right.tieInner[0] = -60;
  const translated = apply(d, 20).design, assigned = apply(d, 20, { mode: 'set-both' }).design;
  assert.equal(translated.left.tieInner[0], -20);
  assert.equal(translated.right.tieInner[0], 60);
  assert.equal(assigned.left.tieInner[0], 20);
  assert.equal(assigned.right.tieInner[0], 20);
  assert.equal(rackLongitudinalOffset(translated), rackLongitudinalOffset(assigned));
});

test('blank, nonnumeric, nonfinite and out-of-range offsets are rejected without coercion', () => {
  const d = exampleSteeringDesign(), before = JSON.stringify(d);
  for (const value of [null, undefined, '', ' ', '-100', NaN, Infinity, -Infinity, true, [], {}, 1e7 + 1, -1e7 - 1]) {
    const result = setRackLongitudinalOffset(d, value);
    assert.equal(result.ok, false, String(value));
    assert.equal(typeof result.reason, 'string');
    assert.equal(JSON.stringify(d), before);
  }
  for (const mode of ['move', '', null, 1]) assert.equal(setRackLongitudinalOffset(d, 0, { mode }).ok, false);
});

test('malformed point arrays are rejected in both modes and are never invented', () => {
  const invalidPoints = [null, undefined, {}, [], [0, 0], [0, 0, 0, 0], Array(3), [0, undefined, 0], [0, '0', 0], [0, NaN, 0], [0, Infinity, 0], ['0', 0, 0]];
  for (const value of invalidPoints) {
    const d = exampleSteeringDesign(); d.left.tieInner = value;
    assert.equal(rackLongitudinalOffset(d), null);
    for (const mode of ['translate', 'set-both']) assert.equal(setRackLongitudinalOffset(d, 10, { mode }).ok, false);
  }
  for (const d of [null, undefined, [], {}, { left: null, right: {} }]) {
    assert.equal(rackLongitudinalOffset(d), null);
    assert.equal(setRackLongitudinalOffset(d, 10, { mode: 'set-both' }).ok, false);
  }
});

test('out-of-range translated endpoints fail atomically even when requested mean is valid', () => {
  const d = exampleSteeringDesign();
  d.left.tieInner[0] = -9e6; d.right.tieInner[0] = 9e6;
  const before = JSON.stringify(d);
  const result = setRackLongitudinalOffset(d, 2e6);
  assert.equal(result.ok, false);
  assert.match(result.reason, /neither joint was changed/);
  assert.equal(JSON.stringify(d), before);
  const boundary = apply(d, 1e6).design;
  assert.equal(boundary.left.tieInner[0], -8e6);
  assert.equal(boundary.right.tieInner[0], 1e7);
  const both = apply(d, -1e7, { mode: 'set-both' }).design;
  assert.equal(both.left.tieInner[0], -1e7); assert.equal(both.right.tieInner[0], -1e7);
});

test('existing unbounded X values cannot pass through unreliable translate arithmetic', () => {
  const d = exampleSteeringDesign();
  d.left.tieInner[0] = Number.MAX_VALUE; d.right.tieInner[0] = Number.MAX_VALUE;
  assert.equal(rackLongitudinalOffset(d), Number.MAX_VALUE);
  assert.equal(setRackLongitudinalOffset(d, 1).ok, false);
  assert.equal(apply(d, 1, { mode: 'set-both' }).design.left.tieInner[0], 1, 'explicit replacement may repair finite out-of-range X');
});

test('existing steering JSON round trips without adding a redundant rack-offset field', () => {
  const oldDesign = JSON.parse(JSON.stringify(exampleSteeringDesign()));
  assert.equal(Object.hasOwn(oldDesign, 'rackLongitudinalOffset'), false);
  const edited = apply(oldDesign, -85).design;
  assert.deepEqual(Object.keys(edited).sort(), Object.keys(oldDesign).sort());
  const imported = JSON.parse(JSON.stringify(edited));
  assert.deepEqual(validateSteeringDesign(imported), []);
  assert.equal(rackLongitudinalOffset(imported), -85);
  assert.deepEqual(imported, edited);
});

test('edited rack geometry solves identically to directly editing joint X coordinates', () => {
  const d = exampleSteeringDesign(); d.left.tieInner[0] = -105; d.right.tieInner[0] = -95;
  const edited = apply(d, -90).design, direct = structuredClone(d);
  direct.left.tieInner[0] += 10; direct.right.tieInner[0] += 10;
  assert.deepEqual(edited, direct);
  for (const sw of [-120, -5, 0, 5, 120]) {
    const result = solveSteering(edited, sw), reference = solveSteering(direct, sw);
    assert.equal(result.ok, true, result.reason);
    assert.deepEqual(result, reference);
    close(result.rack, edited.rackMmPerRevolution * sw / 360);
    close(result.left.inner[0], edited.left.tieInner[0]);
    close(result.right.inner[0], edited.right.tieInner[0]);
    close(result.left.inner[1] - edited.left.tieInner[1], result.rack);
    close(result.right.inner[1] - edited.right.tieInner[1], result.rack);
    close(result.left.inner[2], edited.left.tieInner[2]);
    close(result.right.inner[2], edited.right.tieInner[2]);
  }
});
