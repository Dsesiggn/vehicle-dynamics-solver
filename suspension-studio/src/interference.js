import { linkPairs, hardpointLabel } from './model.js?v=0.2.5';
import { solveAxle, sub } from './solver.js?v=0.2.3';
import { OBSTACLE_TYPES, obstacleComplete, obstacleMesh } from './obstacles.js?v=0.2.5';
import { segmentMeshDistance, segmentSegmentDistance } from './geometry-distance.js?v=0.2.5';

export const HEAVE_SWEEP_STEP_MM = 0.25;
export const HEAVE_BOUNDARY_TOLERANCE_MM = 0.01;
const CONTACT_EPSILON_MM = 1e-8;
const length = vector => Math.hypot(...vector);
const sideName = side => side === 'left' ? 'Left' : 'Right';

export function memberSegmentId(side, kind, start, end) {
  return `${side}:${kind}:${start}>${end}`;
}

function addSegment(segments, points, side, kind, start, end, name, diameterKey, diameter) {
  if (!points[start] || !points[end] || !Number.isFinite(diameter) || diameter <= 0) return;
  segments.push({
    id: memberSegmentId(side, kind, start, end), side, kind, start, end, name,
    p1: points[start], p2: points[end], radius: diameter / 2, diameterKey,
  });
}

/**
 * Builds the rod/damper centerlines at one solved heave position. Chassis-side
 * points come from the unchanged axle hardpoints; upright-side and rocker
 * points come from the solver pose.
 */
export function buildSuspensionMembers(axle, solved, linkOD) {
  if (!solved?.ok || !Number.isFinite(linkOD) || linkOD <= 0) return [];
  const segments = [];
  for (const side of ['left', 'right']) {
    const points = solved[side].points, prefix = sideName(side);
    for (const [start, end] of linkPairs(axle)) {
      const label = hardpointLabel(start).replace(/\s+inner$/i, '');
      addSegment(segments, points, side, 'link', start, end, `${prefix} ${label} link`, 'linkOD', linkOD);
    }

    if (axle.topology === 'macpherson') {
      addSegment(segments, points, side, 'damper', 'strut_top', 'strut_bottom', `${prefix} MacPherson strut`, 'damperOD', axle.damperOD);
    } else if (axle.actuation === 'direct') {
      addSegment(segments, points, side, 'damper', 'damper_top', 'actuation_outer', `${prefix} direct damper`, 'damperOD', axle.damperOD);
    } else {
      const rodName = axle.actuation === 'pullrod' ? 'pullrod' : 'pushrod';
      addSegment(segments, points, side, 'link', 'actuation_outer', 'rocker_rod', `${prefix} ${rodName}`, 'linkOD', linkOD);
      // The app's bell-crank visualization is a three-arm centerline. Each arm
      // uses the shared conservative link OD until rocker plate geometry exists.
      addSegment(segments, points, side, 'link', 'rocker_pivot', 'rocker_rod', `${prefix} bell-crank arm`, 'linkOD', linkOD);
      addSegment(segments, points, side, 'link', 'rocker_rod', 'rocker_damper', `${prefix} bell-crank cross-arm`, 'linkOD', linkOD);
      addSegment(segments, points, side, 'link', 'rocker_damper', 'rocker_pivot', `${prefix} bell-crank damper arm`, 'linkOD', linkOD);
      addSegment(segments, points, side, 'damper', 'damper_top', 'rocker_damper', `${prefix} bell-crank damper`, 'damperOD', axle.damperOD);
    }
  }
  return segments;
}

function obstacleShapes(design, activeAxle) {
  return OBSTACLE_TYPES.flatMap(({ key, label }) => {
    const obstacle = design.obstacles?.[key];
    if (!obstacle?.visible || !obstacleComplete(obstacle, key)) return [];
    const mesh = obstacleMesh(obstacle, key, activeAxle, design.wheelbase);
    return mesh ? [{ key, label, mesh }] : [];
  });
}

function sameJoint(a, b) {
  return [a.p1, a.p2].some(p => [b.p1, b.p2].some(q => length(sub(p, q)) <= 1e-7));
}

function obstacleMeasurement(member, obstacle) {
  const axisDistance = segmentMeshDistance(member.p1, member.p2, obstacle.mesh);
  return {
    key: `obstacle:${member.id}:${obstacle.key}`,
    memberIds: [member.id], memberNames: [member.name], target: `${member.name} ↔ ${obstacle.label}`,
    kind: 'obstacle', distanceMm: axisDistance,
    clearanceMm: axisDistance - member.radius,
  };
}

function memberMeasurement(a, b) {
  const axisDistance = segmentSegmentDistance(a.p1, a.p2, b.p1, b.p2);
  return {
    key: `members:${[a.id, b.id].sort().join('|')}`,
    memberIds: [a.id, b.id], memberNames: [a.name, b.name], target: `${a.name} ↔ ${b.name}`,
    kind: 'member', distanceMm: axisDistance,
    clearanceMm: axisDistance - a.radius - b.radius,
  };
}

function measureMembers(members, obstacles) {
  const measurements = [];
  for (const member of members) for (const obstacle of obstacles) measurements.push(obstacleMeasurement(member, obstacle));
  for (let i = 0; i < members.length; i++) for (let j = i + 1; j < members.length; j++) {
    // The centerline model has no ball-joint, bushing, or bracket solids. Two
    // members sharing a pivot therefore have no defined clearance at that joint.
    if (!sameJoint(members[i], members[j])) measurements.push(memberMeasurement(members[i], members[j]));
  }
  return measurements.filter(item => Number.isFinite(item.clearanceMm)).sort((a, b) => a.clearanceMm - b.clearanceMm);
}

function preparedInputs({ axle, design, activeAxle }) {
  const linkOD = design.interference?.linkOD;
  if (!Number.isFinite(linkOD) || linkOD <= 0) return { error: 'Enter a shared suspension link OD greater than 0 mm.' };
  if (!Number.isFinite(design.heaveTravel?.compression) || !Number.isFinite(design.heaveTravel?.rebound)) return { error: 'Enter valid compression and rebound travel.' };
  const obstacles = obstacleShapes(design, activeAxle);
  return { axle, design, activeAxle, linkOD, obstacles };
}

/** Evaluate all requested member/envelope and member/member pairs at one bump position. */
export function evaluateHeavePosition(options) {
  const prepared = preparedInputs(options);
  if (prepared.error) return { ok: false, reason: prepared.error };
  const { axle, design, activeAxle, linkOD, obstacles } = prepared;
  const travel = options.travel;
  if (!Number.isFinite(travel) || Math.abs(travel) > 100) return { ok: false, reason: 'Requested heave position exceeds the solver travel limit of ±100 mm.' };
  if (travel < -design.heaveTravel.rebound || travel > design.heaveTravel.compression) return { ok: false, reason: 'Requested heave position is outside the configured rebound/compression range.' };
  const solved = solveAxle(axle, travel, 0);
  if (!solved.ok) return { ok: false, reason: solved.left?.reason || solved.right?.reason || 'The kinematic solver could not solve this heave position.', travel, solved };
  const members = buildSuspensionMembers(axle, solved, linkOD);
  const measurements = measureMembers(members, obstacles).map(item => ({ ...item, contact: item.clearanceMm <= CONTACT_EPSILON_MM, travel }));
  return {
    ok: true, travel, solved, members, obstacles, measurements,
    contacts: measurements.filter(item => item.contact),
    minimum: measurements[0] || null,
    maximumConstraintResidualMm: Math.max(solved.left.error, solved.right.error),
  };
}

function valuesAcrossTravel(compression, rebound) {
  const values = new Set([0]);
  for (const [start, end] of [[0, -rebound], [0, compression]]) {
    const intervals = Math.ceil(Math.abs(end - start) / HEAVE_SWEEP_STEP_MM);
    for (let index = 0; index <= intervals; index++) values.add(start + (end - start) * index / Math.max(1, intervals));
  }
  return [...values].sort((a, b) => a - b);
}

/** Sample both heave directions and refine each detected contact boundary. */
export function scanHeaveTravel(options) {
  const prepared = preparedInputs(options);
  if (prepared.error) return { ok: false, reason: prepared.error };
  const { design } = prepared, { compression, rebound } = design.heaveTravel;
  const travels = valuesAcrossTravel(compression, rebound), cache = new Map();
  const at = travel => {
    const key = travel.toFixed(10);
    if (!cache.has(key)) cache.set(key, evaluateHeavePosition({ ...options, travel }));
    return cache.get(key);
  };
  const states = travels.map(at);
  const failed = states.find(state => !state.ok);
  if (failed) return { ok: false, reason: `Sweep stopped at ${failed.travel.toFixed(2)} mm: ${failed.reason}`, failedAt: failed.travel, sampleCount: states.length };

  const keys = [...new Set(states.flatMap(state => state.measurements.map(item => item.key)))];
  const template = new Map(states[0].measurements.map(item => [item.key, item]));
  const minima = keys.map(key => {
    let best = null;
    for (const state of states) {
      const value = state.measurements.find(item => item.key === key);
      if (value && (!best || value.clearanceMm < best.clearanceMm)) best = value;
    }
    return best;
  }).filter(Boolean).sort((a, b) => a.clearanceMm - b.clearanceMm);

  const refine = (key, outside, inside) => {
    let safe = outside, contact = inside;
    while (Math.abs(contact - safe) > HEAVE_BOUNDARY_TOLERANCE_MM) {
      const middle = (safe + contact) / 2, state = at(middle);
      if (!state.ok) return { ok: false, reason: `could not solve while refining a contact boundary near ${middle.toFixed(3)} mm` };
      const value = state.measurements.find(item => item.key === key);
      if (!value) return { ok: false, reason: 'member geometry changed while refining a contact boundary' };
      if (value.clearanceMm <= CONTACT_EPSILON_MM) contact = middle;
      else safe = middle;
    }
    return { ok: true, travel: contact };
  };

  const events = [];
  for (const key of keys) {
    let index = 0;
    while (index < states.length) {
      const value = states[index].measurements.find(item => item.key === key);
      if (!value || value.clearanceMm > CONTACT_EPSILON_MM) { index++; continue; }
      const first = index;
      while (index + 1 < states.length) {
        const next = states[index + 1].measurements.find(item => item.key === key);
        if (!next || next.clearanceMm > CONTACT_EPSILON_MM) break;
        index++;
      }
      const last = index, firstTravel = states[first].travel, lastTravel = states[last].travel;
      const entry = first === 0 ? { ok: true, travel: firstTravel } : refine(key, states[first - 1].travel, firstTravel);
      const exit = last === states.length - 1 ? { ok: true, travel: lastTravel } : refine(key, states[last + 1].travel, lastTravel);
      if (!entry.ok || !exit.ok) return { ok: false, reason: `Sweep stopped: ${entry.reason || exit.reason}.`, failedAt: firstTravel, sampleCount: states.length };
      const definition = template.get(key) || states[first].measurements.find(item => item.key === key);
      let minimum = definition;
      for (let i = first; i <= last; i++) {
        const item = states[i].measurements.find(candidate => candidate.key === key);
        if (item && item.clearanceMm < minimum.clearanceMm) minimum = item;
      }
      events.push({ ...definition, startTravel: Math.min(entry.travel, exit.travel), endTravel: Math.max(entry.travel, exit.travel), firstContactTravel: firstTravel, minimumClearanceMm: minimum.clearanceMm, minimumTravel: minimum.travel });
      index++;
    }
  }
  events.sort((a, b) => a.startTravel - b.startTravel || a.minimumClearanceMm - b.minimumClearanceMm);
  return {
    ok: true, compression, rebound, sampleStepMm: HEAVE_SWEEP_STEP_MM,
    boundaryToleranceMm: HEAVE_BOUNDARY_TOLERANCE_MM, sampleCount: states.length,
    obstacleCount: states[0].obstacles.length, events, minima,
    maximumConstraintResidualMm: Math.max(...states.map(state => state.maximumConstraintResidualMm)),
  };
}
