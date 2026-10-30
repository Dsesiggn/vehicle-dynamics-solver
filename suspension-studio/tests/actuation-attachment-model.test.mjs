import test from 'node:test';
import assert from 'node:assert/strict';
import { ACTUATION_ATTACHMENTS, getActuationAttachment, axle, clone, generatePoints, newDesign, normalizeAxle, preset, validateDesign } from '../src/model.js';
import { importDesign } from '../src/migration.js';
import { solveAxle } from '../src/solver.js';

test('generated axles and presets explicitly retain the original upright attachment', () => {
  assert.deepEqual(Object.keys(ACTUATION_ATTACHMENTS), ['upright', 'lower-wishbone', 'upper-wishbone']);
  for (const topology of ['double-wishbone', 'multi-link', 'macpherson']) assert.equal(axle(topology).actuationAttachment, 'upright');
  for (const id of ['formula', 'road', 'cr26']) {
    const design = preset(id);
    for (const a of Object.values(design.axles)) assert.equal(a.actuationAttachment, 'upright');
    assert.deepEqual(validateDesign(design), []);
  }
});

test('attachment selection is per axle and survives JSON import without changing static geometry', () => {
  const design = newDesign(), before = clone(design);
  design.axles.front.actuationAttachment = 'lower-wishbone';
  design.axles.rear.actuationAttachment = 'upper-wishbone';
  const snapshot = clone(design);
  assert.deepEqual(validateDesign(design), []);
  const imported = importDesign(JSON.parse(JSON.stringify(design)));
  assert.deepEqual(imported.design, design);
  assert.equal(imported.migrated, false);
  assert.deepEqual(design, snapshot);
  for (const side of ['front', 'rear']) assert.deepEqual(imported.design.axles[side].hardpoints, before.axles[side].hardpoints);
  imported.design.axles.front.actuationAttachment = 'upright';
  assert.equal(imported.design.axles.rear.actuationAttachment, 'upper-wishbone');
  assert.equal(design.axles.front.actuationAttachment, 'lower-wishbone');
});

test('missing legacy attachment imports as upright and preserves exact accepted poses', () => {
  for (const actuation of ['pushrod', 'pullrod', 'direct']) {
    const original = newDesign();
    for (const a of Object.values(original.axles)) {
      a.actuation = actuation;
      normalizeAxle(a);
      a.hardpoints = generatePoints(a, original.wheelDiameter);
      delete a.actuationAttachment;
    }
    const before = clone(original), imported = importDesign(original).design;
    assert.deepEqual(validateDesign(original), []);
    assert.deepEqual(original, before);
    for (const side of ['front', 'rear']) {
      const legacy = original.axles[side], explicit = imported.axles[side];
      assert.equal(getActuationAttachment(legacy), 'upright');
      assert.equal(explicit.actuationAttachment, 'upright');
      assert.deepEqual(explicit.hardpoints, legacy.hardpoints);
      for (const [bump, rack] of [[0, 0], [15, 12], [-10, -8]]) {
        const pose = solveAxle(legacy, bump, rack);
        assert.ok(pose.ok, `${actuation} ${side} at ${bump},${rack}: ${pose.left?.reason || pose.right?.reason}`);
        assert.deepEqual(solveAxle(explicit, bump, rack), pose);
      }
    }
  }
});

test('only a missing own property invokes compatibility; supplied malformed attachments are rejected', () => {
  assert.equal(getActuationAttachment({}), 'upright');
  assert.equal(getActuationAttachment(Object.create({ actuationAttachment: 'lower-wishbone' })), 'upright');
  for (const value of [null, undefined, '', 'lower', 'UPRIGHT', '__proto__', 0, false, [], ['upright'], {}, { toString: () => 'upright' }]) {
    const design = newDesign();
    design.axles.front.actuationAttachment = value;
    assert.strictEqual(getActuationAttachment(design.axles.front), value);
    assert.ok(validateDesign(design).some(message => message.includes('front: invalid actuation attachment')));
    // structuredClone correctly rejects function-bearing input independently.
    assert.throws(() => importDesign(design));
  }
});

test('wishbone attachments require double-wishbone topology without import fallback', () => {
  for (const topology of ['multi-link', 'macpherson']) {
    for (const attachment of ['lower-wishbone', 'upper-wishbone']) {
      const design = newDesign();
      design.axles.rear = axle(topology, 1180, false);
      design.axles.rear.actuationAttachment = attachment;
      const before = clone(design);
      assert.ok(validateDesign(design).some(message => message.includes('rear: wishbone actuation attachment requires')));
      assert.throws(() => importDesign(design), /wishbone actuation attachment requires/);
      assert.deepEqual(design, before);
    }
  }
});

test('topology normalization resets unsupported wishbone attachment without regenerating hardpoints', () => {
  for (const topology of ['double-wishbone', 'multi-link', 'macpherson']) {
    for (const attachment of ['upright', 'lower-wishbone', 'upper-wishbone']) {
      const a = axle();
      a.actuationAttachment = attachment;
      const points = a.hardpoints, snapshot = clone(points);
      a.topology = topology;
      normalizeAxle(a);
      assert.equal(a.actuationAttachment, topology === 'double-wishbone' ? attachment : 'upright');
      assert.strictEqual(a.hardpoints, points);
      assert.deepEqual(a.hardpoints, snapshot);
    }
  }
  const malformed = axle('multi-link');
  malformed.actuationAttachment = null;
  normalizeAxle(malformed);
  assert.equal(malformed.actuationAttachment, null, 'Normalization must not hide malformed imported values');
});
