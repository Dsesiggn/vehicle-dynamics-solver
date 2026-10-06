import test from 'node:test';
import assert from 'node:assert/strict';
import { axle, linkPairs, newDesign } from '../src/model.js';
import { solveAxle, sub, norm } from '../src/solver.js';
import { buildSuspensionMembers, evaluateHeavePosition, scanHeaveTravel, HEAVE_SWEEP_STEP_MM, HEAVE_BOUNDARY_TOLERANCE_MM } from '../src/interference.js';

function midpoint(a, b) { return a.map((value, i) => (value + b[i]) / 2); }

test('all modeled hardpoint links and each topology damper produce finite-diameter members', () => {
  const design = newDesign();
  const topologies = [
    ['double-wishbone', design.axles.front, 20, 2],
    ['macpherson', axle('macpherson'), 8, 2],
    ['multi-link', axle('multi-link'), 20, 2],
  ];
  for (const [name, suspension, expectedMembers, expectedDampers] of topologies) {
    const solved = solveAxle(suspension, 0, 0), members = buildSuspensionMembers(suspension, solved, 20);
    assert.ok(solved.ok, `${name}: ${solved.left?.reason || solved.right?.reason}`);
    assert.equal(members.length, expectedMembers);
    assert.equal(members.filter(member => member.kind === 'damper').length, expectedDampers);
    assert.ok(members.every(member => member.radius > 0 && member.p1.length === 3 && member.p2.length === 3));
  }
});

test('heave leaves inboard pivots fixed and preserves every link length while moving outboards', () => {
  const suspension = newDesign().axles.front, atStatic = solveAxle(suspension, 0, 0), atHeave = solveAxle(suspension, 25.4, 0);
  assert.ok(atStatic.ok && atHeave.ok);
  for (const [inner, outer] of linkPairs(suspension)) {
    assert.deepEqual(atHeave.left.points[inner], suspension.hardpoints[inner]);
    assert.notDeepEqual(atHeave.left.points[outer], suspension.hardpoints[outer]);
    const originalLength = norm(sub(suspension.hardpoints[outer], suspension.hardpoints[inner]));
    const solvedLength = norm(sub(atHeave.left.points[outer], atHeave.left.points[inner]));
    assert.ok(Math.abs(solvedLength - originalLength) < 1e-5, `${inner} rod length changed`);
  }
});

test('members joined at the same modeled hardpoint are excluded from self-contact pairs', () => {
  const design = newDesign(); design.interference.linkOD = 10;
  const result = evaluateHeavePosition({ axle: design.axles.front, design, activeAxle: 'front', travel: 0 });
  assert.ok(result.ok);
  assert.ok(!result.measurements.some(item => item.target === 'Left LCA fore link ↔ Left LCA aft link'));
  assert.ok(!result.measurements.some(item => item.target === 'Left UCA fore link ↔ Left UCA aft link'));
});

test('link-to-link contact uses the sum of the shared link radii', () => {
  const design = newDesign(); design.interference.linkOD = 40;
  const result = evaluateHeavePosition({ axle: design.axles.front, design, activeAxle: 'front', travel: 0 });
  assert.ok(result.ok);
  const pair = result.contacts.find(item => item.target === 'Left LCA fore link ↔ Left pushrod');
  assert.ok(pair, 'expected intentionally oversized shared OD to expose a link-to-link overlap');
  assert.ok(Math.abs(pair.clearanceMm - (pair.distanceMm - 40)) < 1e-9);
});

test('pairwise checks also include link-to-damper combinations', () => {
  const design = newDesign(); design.interference.linkOD = 100;
  const result = evaluateHeavePosition({ axle: design.axles.front, design, activeAxle: 'front', travel: 0 });
  assert.ok(result.ok);
  assert.ok(result.contacts.some(item => item.kind === 'member' && item.memberNames.some(name => name.includes('damper')) && item.memberNames.some(name => name.includes('pushrod'))));
});

test('visible packaging boxes are checked against suspension links', () => {
  const design = newDesign(); design.interference.linkOD = 20;
  const suspension = design.axles.front, [start, end] = linkPairs(suspension)[0];
  design.obstacles.engine.visible = true;
  design.obstacles.engine.center = midpoint(suspension.hardpoints[start], suspension.hardpoints[end]);
  design.obstacles.engine.dimensions = { length: 4, width: 4, height: 4 };
  const result = evaluateHeavePosition({ axle: suspension, design, activeAxle: 'front', travel: 0 });
  assert.ok(result.ok);
  const contact = result.contacts.find(item => item.target === 'Left LCA fore link ↔ Engine');
  assert.ok(contact);
  assert.equal(contact.distanceMm, 0);
  assert.equal(contact.clearanceMm, -10);
});

test('existing damper OD is used for MacPherson strut body versus packaging', () => {
  const design = newDesign(), suspension = axle('macpherson');
  design.interference.linkOD = 5;
  design.obstacles.differential.visible = true;
  design.obstacles.differential.center = midpoint(suspension.hardpoints.strut_top, suspension.hardpoints.strut_bottom);
  design.obstacles.differential.dimensions = { length: 4, width: 4, height: 4 };
  const result = evaluateHeavePosition({ axle: suspension, design, activeAxle: 'front', travel: 0 });
  const damperContact = result.contacts.find(item => item.target === 'Left MacPherson strut ↔ Differential');
  assert.ok(damperContact);
  assert.equal(damperContact.clearanceMm, -suspension.damperOD / 2);
});

test('sweep brackets a packaging interference interval and refines it for the travel slider', () => {
  const design = newDesign(), suspension = design.axles.front, linkOD = 10, contactTravel = 10;
  design.interference.linkOD = linkOD;
  design.obstacles.engine.visible = true;
  const pose = solveAxle(suspension, contactTravel, 0);
  const member = buildSuspensionMembers(suspension, pose, linkOD).find(item => item.id === 'left:link:lca_front>lca_outer');
  design.obstacles.engine.center = midpoint(member.p1, member.p2);
  design.obstacles.engine.dimensions = { length: 4, width: 4, height: 4 };

  const scan = scanHeaveTravel({ axle: suspension, design, activeAxle: 'front' });
  assert.ok(scan.ok, scan.reason);
  assert.equal(scan.sampleStepMm, HEAVE_SWEEP_STEP_MM);
  assert.equal(scan.boundaryToleranceMm, HEAVE_BOUNDARY_TOLERANCE_MM);
  assert.equal(scan.sampleCount, 205);
  const event = scan.events.find(item => item.target === 'Left LCA fore link ↔ Engine');
  assert.ok(event);
  assert.ok(event.startTravel < contactTravel && event.endTravel > contactTravel);
  assert.ok(event.endTravel - event.startTravel > 0);
  assert.ok(event.minimumClearanceMm < 0);
  const sliderPose = evaluateHeavePosition({ axle: suspension, design, activeAxle: 'front', travel: event.firstContactTravel });
  assert.ok(sliderPose.ok && sliderPose.contacts.some(item => item.key === event.key));
  assert.ok(!evaluateHeavePosition({ axle: suspension, design, activeAxle: 'front', travel: 25.5 }).ok);
});

test('invalid link OD stops the checker instead of inventing a member diameter', () => {
  const design = newDesign();
  assert.match(evaluateHeavePosition({ axle: design.axles.front, design, activeAxle: 'front', travel: 0 }).reason, /link OD/);
  assert.match(scanHeaveTravel({ axle: design.axles.front, design, activeAxle: 'front' }).reason, /link OD/);
});

test('full 25.4 mm compression and rebound scans solve each default topology', () => {
  const design = newDesign(); design.interference.linkOD = 10;
  for (const [topology, suspension] of [
    ['double-wishbone', design.axles.front],
    ['macpherson', axle('macpherson', 1200, false)],
    ['multi-link', axle('multi-link', 1200, false)],
  ]) {
    const scan = scanHeaveTravel({ axle: suspension, design, activeAxle: 'front' });
    assert.ok(scan.ok, `${topology}: ${scan.reason}`);
    assert.equal(scan.sampleCount, 205);
    assert.equal(scan.events.length, 0);
    assert.ok(scan.maximumConstraintResidualMm < 1e-6, `${topology}: ${scan.maximumConstraintResidualMm} mm residual`);
  }
});
