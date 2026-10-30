import test from 'node:test';
import assert from 'node:assert/strict';
import { axle, clone, generatePoints } from '../src/model.js';
import { solveAxle, solveCorner } from '../src/solver.js';
import { suspensionPreviewGeometry } from '../src/viewer.js';

// Prescribed spatial rotation, independent of the production rotate() helper.
// Rational inclined unit direction has both caster and kingpin inclination.
// Positions: axle-local SAE +X forward, +Y right, +Z down, millimeters.
const kingpin = [3 / 13, 4 / 13, -12 / 13];
const subtract = (a, b) => a.map((value, index) => value - b[index]);
const add = (a, b) => a.map((value, index) => value + b[index]);
const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
const distance = (a, b) => Math.hypot(...subtract(a, b));
const reflected = point => [point[0], -point[1], point[2]];
const near = (actual, expected, tolerance, label = '') => assert.ok(Math.abs(actual - expected) <= tolerance,
  `${label}: ${actual} != ${expected}; tolerance ${tolerance}`);
const vectorNear = (actual, expected, tolerance, label = '') => actual.forEach((value, index) => near(value, expected[index], tolerance, `${label}[${index}]`));

function prescribedRotation(degrees) {
  const theta = degrees * Math.PI / 180, c = Math.cos(theta), s = Math.sin(theta), v = 1 - c;
  const [x, y, z] = kingpin;
  // Explicit matrix expansion is independent of solver.js Rodrigues routine.
  const matrix = [
    [c + x*x*v, x*y*v - z*s, x*z*v + y*s],
    [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
    [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
  ];
  return { theta, direction: point => matrix.map(row => dot(row, point)) };
}

function fixture(topology, centerOnAxis = false) {
  const a = axle(topology, centerOnAxis ? 1100 : 1240);
  a.tireTreadWidth = 203.2;
  if (topology === 'double-wishbone') {
    a.actuation = 'direct'; a.mounting = 'direct';
    a.hardpoints = generatePoints(a, 460);
  }
  const p = a.hardpoints;
  Object.assign(p, {
    lca_outer: [-30, -590, -110],
    wheel_center: centerOnAxis ? [0, -550, -230] : [-15, -620, -220],
    tie_rod_outer: [85, -570, -195],
  });
  if (topology === 'double-wishbone') {
    p.uca_outer = [30, -510, -350];
    p.actuation_outer = [0, -510, -180];
  } else {
    p.strut_top = [30, -510, -350];
    // The shaft top-bottom line intentionally differs from the kingpin line.
    p.strut_bottom = [-10, -540, -190];
  }
  return a;
}

function prescribedPose(a, degrees) {
  const p = a.hardpoints, rotation = prescribedRotation(degrees), lower = p.lca_outer;
  const position = point => add(lower, rotation.direction(subtract(point, lower)));
  const center = position(p.wheel_center), outer = position(p.tie_rod_outer), inner = p.tie_rod_inner;
  const tieLength = distance(p.tie_rod_outer, inner);
  const square = tieLength**2 - (outer[0] - inner[0])**2 - (outer[2] - inner[2])**2;
  assert.ok(square > 0, 'Prescribed rack construction must have a real square root');
  const rack = outer[1] - inner[1] - Math.sign(p.tie_rod_outer[1] - inner[1]) * Math.sqrt(square);
  assert.ok(Math.abs(rack) <= a.rackTravel / 2, 'Prescribed rack must be within the declared travel');
  return {
    ...rotation, position, center, rack,
    bump: p.wheel_center[2] - center[2],
    spindle: rotation.direction([0, -1, 0]),
  };
}

for (const topology of ['double-wishbone', 'macpherson']) {
  test(`${topology}: an independently prescribed inclined steering axis controls full wheel pose in both directions`, () => {
    const a = fixture(topology), before = clone(a);
    for (const degrees of [-12, 12]) {
      const expected = prescribedPose(a, degrees), solved = solveCorner(a, expected.bump, expected.rack);
      assert.equal(solved.ok, true, solved.reason);
      assert.ok(solved.error < 1e-5, 'Constraint residual must meet the solver target in this fixture');
      vectorNear(solved.rotation, kingpin.map(value => value * expected.theta), 1e-6, 'inclined rotation vector');
      vectorNear(solved.points.lca_outer, a.hardpoints.lca_outer, 1e-4, 'fixed lower ball');
      if (topology === 'double-wishbone') vectorNear(solved.points.uca_outer, a.hardpoints.uca_outer, 1e-4, 'fixed upper ball');
      else {
        vectorNear(solved.points.strut_top, a.hardpoints.strut_top, 1e-10, 'fixed strut mount');
        vectorNear(solved.points.strut_bottom, expected.position(a.hardpoints.strut_bottom), 1e-4, 'offset shaft bottom');
        near(distance(solved.points.strut_top, solved.points.strut_bottom), distance(a.hardpoints.strut_top, a.hardpoints.strut_bottom), 1e-4, 'pure-steer strut length');
      }
      vectorNear(solved.points.wheel_center, expected.center, 1e-4, 'off-axis wheel center');
      // A yaw-only construction cannot produce this prescribed spindle Z.
      assert.ok(Math.abs(expected.spindle[2]) > 0.03);
      near(solved.camber, Math.atan2(expected.spindle[2], Math.hypot(...expected.spindle.slice(0, 2))) * 180 / Math.PI, 1e-4, 'steering-induced camber');
      assert.ok(distance(expected.center, a.hardpoints.wheel_center) > 10, 'Fixture must expose wheel-center orbit');
      assert.ok(Math.abs(expected.bump) > 1, 'Offset center must expose steering-induced vertical motion');
    }
    assert.deepEqual(a, before, 'Pose verification must not alter supplied geometry');
  });

  test(`${topology}: preview spindle, tire plane and center reproduce prescribed inclined steering on either corner and axle frame`, () => {
    const a = fixture(topology), before = clone(a);
    for (const degrees of [-12, 12]) for (const side of [-1, 1]) {
      const expected = prescribedPose(a, degrees);
      // Right solves the reflected left template at the opposite rack input.
      const solved = solveAxle(a, expected.bump, -side * expected.rack);
      assert.equal(solved.ok, true, solved.left?.reason || solved.right?.reason);
      for (const activeAxle of ['front', 'rear']) {
        const scene = suspensionPreviewGeometry(a, 460, solved, {activeAxle, wheelbase: 1530, obstacles: {}});
        const corner = scene.corners.find(value => value.side === side);
        const physicalCenter = side === -1 ? expected.center : reflected(expected.center);
        const expectedCenter = physicalCenter.map((value, index) => value - (index === 0 && activeAxle === 'rear' ? 1530 : 0));
        const expectedSpindle = side === -1 ? expected.spindle : reflected(expected.spindle);
        vectorNear(corner.points.wheel_center, expectedCenter, 1e-4, 'preview center');
        vectorNear(corner.wheelAxis, expectedSpindle, 1e-6, 'full spatial spindle');
        vectorNear(corner.tire.center, expectedCenter, 1e-4, 'tire center');
        vectorNear(corner.tire.axis, expectedSpindle, 1e-6, 'tire plane normal');
        const edgeDirection = subtract(corner.tire.edgeCenters[1], corner.tire.edgeCenters[0]);
        vectorNear(edgeDirection, expectedSpindle.map(value => value * a.tireTreadWidth), 1e-4, 'tread edge separation');
        near(Math.hypot(...edgeDirection), a.tireTreadWidth, 1e-8, 'tread width');
      }
    }
    assert.deepEqual(a, before);
  });

  test(`${topology}: a center on the inclined steering axis stays fixed at zero bump while wheel orientation changes`, () => {
    const a = fixture(topology, true);
    for (const degrees of [-12, 12]) {
      const expected = prescribedPose(a, degrees), solved = solveCorner(a, 0, expected.rack);
      assert.equal(solved.ok, true, solved.reason);
      near(expected.bump, 0, 1e-12, 'center-on-axis prescribed bump');
      vectorNear(solved.points.wheel_center, a.hardpoints.wheel_center, 1e-4, 'fixed center');
      const solvedAxle = solveAxle(a, 0, expected.rack);
      assert.equal(solvedAxle.ok, true, solvedAxle.left?.reason || solvedAxle.right?.reason);
      const scene = suspensionPreviewGeometry(a, 460, solvedAxle, {activeAxle: 'front', wheelbase: 1530, obstacles: {}});
      vectorNear(scene.corners[0].wheelAxis, expected.spindle, 1e-6, 'inclined wheel orientation');
      assert.ok(Math.abs(solved.camber) > 2, 'Pure steering about the inclined axis must change camber');
    }
  });
}

test('MacPherson steering follows the lower-ball/top-bearing axis when its strut shaft is deliberately noncollinear', () => {
  const a = fixture('macpherson'), p = a.hardpoints;
  const shaft = subtract(p.strut_top, p.strut_bottom), shaftLength = Math.hypot(...shaft);
  const shaftAxis = shaft.map(value => value / shaftLength);
  assert.ok(distance(shaftAxis, kingpin) > 0.1, 'Shaft must not approximate the steering axis in this fixture');
  for (const degrees of [-12, 12]) {
    const expected = prescribedPose(a, degrees), solved = solveCorner(a, expected.bump, expected.rack);
    assert.equal(solved.ok, true, solved.reason);
    const rotationLength = Math.hypot(...solved.rotation);
    const measuredAxis = solved.rotation.map(value => Math.sign(expected.theta) * value / rotationLength);
    vectorNear(measuredAxis, kingpin, 1e-6, 'actual steering rotation axis');
    assert.ok(distance(measuredAxis, shaftAxis) > 0.1, 'Steering axis must remain distinct from the offset shaft');
  }
});
