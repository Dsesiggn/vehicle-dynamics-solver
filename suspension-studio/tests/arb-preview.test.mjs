import test from 'node:test';
import assert from 'node:assert/strict';
import { ARB_POINTS, newAntiRollBar, arbComplete, arbPreviewGeometry } from '../src/arb.js';

// Synthetic coordinates test the stated static representation, not a vehicle design.
const completeBar = () => ({ type: 'u-bar', enabled: true, points: {
  bearing: [80, -200, -180], bend: [80, -400, -180],
  arm_tip: [-80, -400, -180], link_pickup: [-80, -450, -100],
} });
const distance = (a, b) => Math.hypot(...a.map((value, index) => value - b[index]));

test('Disabled, unsupported, or malformed ARB containers produce no preview', () => {
  for (const arb of [undefined, null, false, {}, newAntiRollBar(),
    { ...completeBar(), enabled: false }, { ...completeBar(), enabled: 1 },
    { ...completeBar(), type: 'blade' }, { ...completeBar(), points: null },
    { ...completeBar(), points: [] }]) {
    assert.deepEqual(arbPreviewGeometry(arb), {
      points: [], segments: [], enteredPointCount: 0,
      missingLabels: ARB_POINTS.map(({ label }) => label),
      layoutComplete: false,
    });
  }
});

test('Enabled but blank ARB inputs never infer marker locations or segments', () => {
  const arb = newAntiRollBar(); arb.enabled = true;
  const preview = arbPreviewGeometry(arb);
  assert.equal(preview.enteredPointCount, 0);
  assert.deepEqual(preview.points, []);
  assert.deepEqual(preview.segments, []);
  assert.equal(preview.missingLabels.length, 4);
  assert.equal(preview.layoutComplete, false);
});

test('Each complete point previews immediately; a bend alone defines the mirrored transverse centerline', () => {
  const arb = newAntiRollBar(); arb.enabled = true;
  arb.points.bend = [80, -400, -180];
  const preview = arbPreviewGeometry(arb);
  assert.equal(preview.enteredPointCount, 1);
  assert.deepEqual(preview.points, [
    { key: 'bend', label: 'Bar bend', side: 'Left', position: [80, -400, -180] },
    { key: 'bend', label: 'Bar bend', side: 'Right', position: [80, 400, -180] },
  ]);
  assert.deepEqual(preview.segments, [{ kind: 'bar', start: [80, -400, -180], end: [80, 400, -180] }]);
  assert.deepEqual(preview.missingLabels, ['Chassis bearing', 'Lever-arm tip', 'Suspension pickup']);
  assert.equal(arbComplete(arb), false);
  assert.equal(preview.layoutComplete, false);
});

test('Arm tip and pickup draw both drop links without inventing bearing or bend coordinates', () => {
  const arb = newAntiRollBar(); arb.enabled = true;
  arb.points.arm_tip = [-80, -400, -180];
  arb.points.link_pickup = [-80, -450, -100];
  const preview = arbPreviewGeometry(arb);
  assert.equal(preview.enteredPointCount, 2);
  assert.equal(preview.points.length, 4);
  assert.deepEqual(preview.segments, [
    { kind: 'drop-link', start: [-80, -400, -180], end: [-80, -450, -100] },
    { kind: 'drop-link', start: [-80, 400, -180], end: [-80, 450, -100] },
  ]);
});

test('Missing bearing no longer suppresses complete bar and drop-link centerlines', () => {
  const arb = completeBar(); arb.points.bearing = [null, null, null];
  const preview = arbPreviewGeometry(arb);
  assert.equal(preview.enteredPointCount, 3);
  assert.equal(preview.points.length, 6);
  assert.equal(preview.segments.filter(segment => segment.kind === 'bar').length, 3);
  assert.equal(preview.segments.filter(segment => segment.kind === 'drop-link').length, 2);
  assert.deepEqual(preview.missingLabels, ['Chassis bearing']);
  assert.equal(arbComplete(arb), false);
  assert.equal(preview.layoutComplete, true);
});

test('Partial or invalid point affects only its own marker and dependent segments', () => {
  for (const value of [null, [0, 0], [0, 0, null], [0, 1, 0], [10000.1, 0, 0],
    [-10000.1, 0, 0], [Infinity, 0, 0], [NaN, 0, 0], ['0', 0, 0], new Array(3)]) {
    const arb = completeBar(); arb.points.arm_tip = value;
    const preview = arbPreviewGeometry(arb);
    assert.equal(preview.enteredPointCount, 3);
    assert.equal(preview.points.length, 6);
    assert.deepEqual(preview.missingLabels, ['Lever-arm tip']);
    assert.equal(preview.layoutComplete, false);
    assert.deepEqual(preview.segments, [{ kind: 'bar', start: [80, -400, -180], end: [80, 400, -180] }]);
  }
  const arb = completeBar(); delete arb.points.bearing;
  assert.equal(arbPreviewGeometry(arb).enteredPointCount, 3);
});

test('Zero and valid range endpoints remain literal coordinates; null never substitutes zero', () => {
  const arb = newAntiRollBar(); arb.enabled = true;
  arb.points.bearing = [0, 0, 0];
  arb.points.bend = [10000, -10000, -10000];
  let preview = arbPreviewGeometry(arb);
  assert.equal(preview.enteredPointCount, 2);
  assert.deepEqual(preview.points.find(point => point.side === 'Left' && point.key === 'bearing').position, [0, 0, 0]);
  assert.deepEqual(preview.points.find(point => point.side === 'Right' && point.key === 'bend').position, [10000, 10000, -10000]);
  arb.points.bearing[2] = null;
  preview = arbPreviewGeometry(arb);
  assert.equal(preview.enteredPointCount, 1);
  assert.ok(!preview.points.some(point => point.key === 'bearing'));
});

test('Preview reflection preserves all point distances and corresponding segment lengths', () => {
  const preview = arbPreviewGeometry(completeBar());
  assert.equal(preview.layoutComplete, true);
  const left = preview.points.filter(point => point.side === 'Left');
  const right = preview.points.filter(point => point.side === 'Right');
  for (let i = 0; i < left.length; i++) {
    assert.deepEqual(right[i].position, [left[i].position[0], -left[i].position[1], left[i].position[2]]);
    for (let j = 0; j < left.length; j++) {
      assert.equal(distance(left[i].position, left[j].position), distance(right[i].position, right[j].position));
    }
  }
  const bars = preview.segments.filter(segment => segment.kind === 'bar');
  const drops = preview.segments.filter(segment => segment.kind === 'drop-link');
  assert.equal(distance(bars[0].start, bars[0].end), 160);
  assert.equal(distance(bars[1].start, bars[1].end), 800);
  assert.equal(distance(bars[0].start, bars[0].end), distance(bars[2].start, bars[2].end));
  assert.equal(distance(drops[0].start, drops[0].end), Math.hypot(50, 80));
  assert.equal(distance(drops[0].start, drops[0].end), distance(drops[1].start, drops[1].end));
});

test('Preview leaves input untouched and returns independent mutable coordinate arrays', () => {
  const arb = completeBar(), original = structuredClone(arb);
  const preview = arbPreviewGeometry(arb);
  assert.deepEqual(arb, original);
  const firstSegmentBefore = [...preview.segments[0].start];
  preview.points.find(point => point.key === 'arm_tip' && point.side === 'Left').position[0] = 999;
  assert.deepEqual(preview.segments[0].start, firstSegmentBefore);
  preview.segments[0].start[1] = 999;
  preview.segments[0].end[2] = 999;
  assert.deepEqual(arb, original);
  assert.notDeepEqual(preview.segments[0].end, preview.segments[1].start);
});
