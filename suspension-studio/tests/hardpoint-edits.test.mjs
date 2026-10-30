import test from 'node:test';
import assert from 'node:assert/strict';
import { clone, newDesign, validateDesign } from '../src/model.js';
import { importDesign } from '../src/migration.js';
import { evaluateHardpointEdits } from '../src/hardpoint-edits.js';
import { axlePointToFront } from '../src/coordinate-frames.js';
import { solveAxle } from '../src/solver.js';

test('front-origin rear edits convert once, preserve the static assembly, and survive v2 import', () => {
  const design=newDesign();design.wheelbase=1530;
  const before=clone(design), frame={frame:'FRONT_AXLE'};
  const edits=Object.fromEntries(Object.entries(design.axles.rear.hardpoints).map(([key,p])=>[key,Object.fromEntries(axlePointToFront(p,'rear',1530).map((value,axis)=>[axis,String(value)]))]));
  const accepted=evaluateHardpointEdits(design,'rear',edits,frame);
  assert.deepEqual(accepted.errors,[]);
  assert.deepEqual(accepted.axle,before.axles.rear);
  design.axles.rear=accepted.axle;
  assert.deepEqual(importDesign(JSON.parse(JSON.stringify(design))).design,before);
  for(const travel of [-25.4,0,25.4]) assert.deepEqual(solveAxle(design.axles.rear,travel,0),solveAxle(before.axles.rear,travel,0));
  assert.deepEqual(design.axles.front,before.axles.front);
});

test('front-origin X edits preserve Y/Z and convert the axle-local input bounds', () => {
  const d=newDesign();d.wheelbase=1530;const before=clone(d),frame={frame:'FRONT_AXLE'};
  const rear=evaluateHardpointEdits(d,'rear',{uca_front:{0:'-1402.87655'}},frame);
  assert.deepEqual(rear.errors,[]);
  assert.ok(Math.abs(rear.axle.hardpoints.uca_front[0]-127.12345)<1e-10);
  assert.deepEqual(rear.axle.hardpoints.uca_front.slice(1),d.axles.rear.hardpoints.uca_front.slice(1));
  for(const [entered,local] of [['-11530',-10000],['8470',10000]]) {
    const r=evaluateHardpointEdits(d,'rear',{uca_front:{0:entered}},frame);
    assert.deepEqual(r.errors,[]);assert.equal(r.axle.hardpoints.uca_front[0],local);
  }
  for(const entered of ['', '-', '-1530.', '8470.01', '-11530.01', '1e309']) assert.equal(evaluateHardpointEdits(d,'rear',{uca_front:{0:entered}},frame).axle,null);
  assert.equal(evaluateHardpointEdits(d,'front',{uca_front:{0:'127.12345'}},frame).axle.hardpoints.uca_front[0],127.12345);
  assert.equal(evaluateHardpointEdits(d,'rear',{}, {frame:'UNKNOWN'}).axle,null);
  assert.deepEqual(d,before);
});

test('pending front-origin entries keep their meaning across wheelbase changes', () => {
  const d=newDesign(),edits={wheel_center:{0:'-1530',1:'-6'}},frame={frame:'FRONT_AXLE'};
  assert.equal(evaluateHardpointEdits(d,'rear',edits,frame).axle,null);
  d.wheelbase=1530;edits.wheel_center[1]='-608.05';
  const r=evaluateHardpointEdits(d,'rear',edits,frame);
  assert.deepEqual(r.errors,[]);assert.deepEqual(r.axle.hardpoints.wheel_center,[0,-608.05,-230]);
  assert.equal(r.axle.track,1216.1);
});

test('wheel-center Y drafts remain invalid until their derived track meets the existing schema', () => {
  const design = newDesign(), before = clone(design);
  for (const raw of ['', '-', '-6', '-61', '-616.']) {
    const edits = { wheel_center: { 1: raw } }, snapshot = clone(edits);
    const result = evaluateHardpointEdits(design, 'front', edits);
    assert.equal(result.axle, null, `Accepted incomplete or invalid draft ${raw}`);
    assert.ok(result.errors.length);
    assert.deepEqual(edits, snapshot);
    assert.deepEqual(design, before);
  }
  const result = evaluateHardpointEdits(design, 'front', { wheel_center: { 1: '-616.8' } });
  assert.deepEqual(result.errors, []);
  assert.equal(result.axle.track, 1233.6);
  assert.deepEqual(result.axle.hardpoints.wheel_center, [0, -616.8, -230]);
  for (const [key, point] of Object.entries(before.axles.front.hardpoints)) {
    if (key !== 'wheel_center') assert.deepEqual(result.axle.hardpoints[key], point);
  }
  assert.deepEqual(design, before);
});

test('rack-inner Y drafts recover without changing other chassis points', () => {
  const design = newDesign(), before = clone(design);
  for (const raw of ['-', '-2', '-21']) {
    const result = evaluateHardpointEdits(design, 'front', { tie_rod_inner: { 1: raw } });
    assert.equal(result.axle, null);
    assert.ok(result.errors.length);
  }
  const result = evaluateHardpointEdits(design, 'front', { tie_rod_inner: { 1: '-219.08' } });
  assert.deepEqual(result.errors, []);
  assert.equal(result.axle.rackLength, 438.16);
  assert.deepEqual(result.axle.hardpoints.tie_rod_inner, [95, -219.08, -195]);
  assert.equal(result.axle.track, before.axles.front.track);
  for (const [key, point] of Object.entries(before.axles.front.hardpoints)) {
    if (key !== 'tie_rod_inner') assert.deepEqual(result.axle.hardpoints[key], point);
  }
  assert.deepEqual(design, before);
});

test('coordinate precision survives an accepted edit and JSON export/import without rounding', () => {
  const design = newDesign();
  const result = evaluateHardpointEdits(design, 'front', {
    uca_front: { 0: '127.123456789', 2: '-3.30123456789e2' },
  });
  assert.deepEqual(result.errors, []);
  assert.equal(result.axle.hardpoints.uca_front[0], 127.123456789);
  assert.equal(result.axle.hardpoints.uca_front[2], -330.123456789);
  design.axles.front = result.axle;
  const imported = importDesign(JSON.parse(JSON.stringify(design))).design;
  assert.deepEqual(imported, design);
});

test('multiple pending cells can resolve a temporarily degenerate link in a single transaction', () => {
  const design = newDesign(), outer = design.axles.front.hardpoints.lca_outer;
  const degenerate = { lca_front: { 0: String(outer[0]), 1: String(outer[1]), 2: String(outer[2]) } };
  const rejected = evaluateHardpointEdits(design, 'front', degenerate);
  assert.equal(rejected.axle, null);
  assert.ok(rejected.errors.some(error => error.includes('degenerate')));
  const corrected = { ...degenerate, lca_outer: { 0: String(outer[0] + 20) } };
  const accepted = evaluateHardpointEdits(design, 'front', corrected);
  assert.deepEqual(accepted.errors, []);
  assert.deepEqual(accepted.axle.hardpoints.lca_front, outer);
  assert.deepEqual(accepted.axle.hardpoints.lca_outer, [outer[0] + 20, outer[1], outer[2]]);
});

test('strict numeric drafts reject blank, partial, nonfinite, hexadecimal and out-of-range inputs with cell labels', () => {
  const design = newDesign(), before = clone(design);
  for (const raw of ['', ' ', '+', '-', '.', '+.', '1.', '1e', '1e+', '1e-', '12mm', '12 3', '0x10', '0b10', 'Infinity', '-Infinity', 'NaN', '1e309', '10000.0001', '-10000.0001', null, 25]) {
    const result = evaluateHardpointEdits(design, 'front', { uca_front: { 0: raw } });
    assert.equal(result.axle, null, `Unexpectedly accepted ${String(raw)}`);
    assert.ok(result.errors.some(error => error.includes('UCA fore X')), `Missing cell label for ${String(raw)}`);
    assert.deepEqual(design, before);
  }
  for (const [raw, expected] of [['.5', .5], ['+.5', .5], ['-1.25e+2', -125], ['1E-3', .001], ['10000', 10000], ['-10000', -10000]]) {
    const result = evaluateHardpointEdits(design, 'front', { uca_front: { 0: raw } });
    assert.deepEqual(result.errors, [], `Rejected complete numeric input ${raw}`);
    assert.equal(result.axle.hardpoints.uca_front[0], expected);
  }
});

test('front and rear edits are independent and overlay touched cells on the current design', () => {
  const design = newDesign();
  design.axles.front.hardpoints.uca_front[2] = -350.12345;
  design.axles.front.hardpoints.lca_rear[0] = -137.12345;
  const before = clone(design), edits = { uca_front: { 0: '131.45678' } }, beforeEdits = clone(edits);
  const front = evaluateHardpointEdits(design, 'front', edits);
  assert.deepEqual(front.errors, []);
  assert.deepEqual(front.axle.hardpoints.uca_front, [131.45678, before.axles.front.hardpoints.uca_front[1], -350.12345]);
  assert.deepEqual(front.axle.hardpoints.lca_rear, before.axles.front.hardpoints.lca_rear);
  const rear = evaluateHardpointEdits(design, 'rear', { wheel_center: { 1: '-608.05' } });
  assert.deepEqual(rear.errors, []);
  assert.equal(rear.axle.track, 1216.1);
  assert.deepEqual(design, before);
  assert.deepEqual(edits, beforeEdits);
  front.axle.hardpoints.lca_rear[0] = 999;
  assert.deepEqual(design, before, 'Accepted result must own independent point arrays');
});

test('unrelated edits preserve existing model-valid dimension tolerances without normalization', () => {
  const design = newDesign();
  design.axles.front.track = 1200.05;
  design.axles.front.rackLength = 440.05;
  assert.deepEqual(validateDesign(design), []);
  const untouched = evaluateHardpointEdits(design, 'front', {
    wheel_center: { 0: '1.23456' }, tie_rod_inner: { 2: '-196.23456' },
  });
  assert.deepEqual(untouched.errors, []);
  assert.equal(untouched.axle.track, 1200.05);
  assert.equal(untouched.axle.rackLength, 440.05);

  const wheel = evaluateHardpointEdits(design, 'front', { wheel_center: { 1: '-616.8' } });
  assert.deepEqual(wheel.errors, []);
  assert.equal(wheel.axle.track, 1233.6);
  assert.equal(wheel.axle.rackLength, 440.05);
  const rack = evaluateHardpointEdits(design, 'front', { tie_rod_inner: { 1: '-219.08' } });
  assert.deepEqual(rack.errors, []);
  assert.equal(rack.axle.track, 1200.05);
  assert.equal(rack.axle.rackLength, 438.16);
});

test('unknown point, axis, axle and malformed edit containers are rejected', () => {
  const design = newDesign();
  for (const edits of [null, [], { unknown: { 0: '1' } }, { uca_front: [] }, { uca_front: null }, { uca_front: { 3: '1' } }, { uca_front: { '01': '1' } }]) {
    const result = evaluateHardpointEdits(design, 'front', edits);
    assert.equal(result.axle, null);
    assert.ok(result.errors.length);
  }
  assert.equal(evaluateHardpointEdits(design, 'left', {}).axle, null);
  assert.equal(evaluateHardpointEdits(null, 'front', {}).axle, null);
});
