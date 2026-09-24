/** Canonical schema v2: SAE J670 Z-down; axle-local hardpoints in mm. */
import { COORDINATE_SYSTEM, HARDPOINT_FRAME, LENGTH_UNIT, legacyToSAE } from './coordinates.js';
export const TOPOLOGIES = {
  'double-wishbone': { name: 'Double wishbone', solver: 'Wishbone · 5 links' },
  macpherson: { name: 'MacPherson', solver: 'Strut constraint' },
  'multi-link': { name: 'Multi-link', solver: '5-link constraint' },
};
export const clone = value => structuredClone(value);
export const title = value => value.replaceAll('-', ' ').replace(/^./, c => c.toUpperCase());
export function axle(topology = 'double-wishbone', track = 1200, steered = true) {
  const a = { topology, track, steered, actuation: topology === 'macpherson' ? 'direct' : 'pushrod', mounting: topology === 'macpherson' ? 'direct' : 'bell-crank', rackLength: 440, rackTravel: 60, springOD: 65, damperOD: 40, hardpoints: {} };
  a.hardpoints = generatePoints(a, 460); return a;
}
export function newDesign() {
  return { version: 2, coordinateSystem: COORDINATE_SYSTEM, hardpointFrame: HARDPOINT_FRAME, lengthUnit: LENGTH_UNIT, name: 'Untitled suspension', wheelbase: 1600, wheelDiameter: 460, axles: { front: axle(), rear: axle('double-wishbone', 1180, false) } };
}
export function normalizeAxle(a) {
  if (a.topology === 'macpherson') a.actuation = 'direct';
  a.mounting = a.actuation === 'direct' ? 'direct' : 'bell-crank';
}
export function generatePoints(a, diameter) {
  const y = -a.track / 2, z = -diameter / 2;
  const p = {
    lca_front: [125, y * .4, z + 105], lca_rear: [-125, y * .4, z + 105], lca_outer: [0, y + 35, z + 95],
    tie_rod_inner: [95, -a.rackLength / 2, z + 35], tie_rod_outer: [90, y + 25, z + 35], wheel_center: [0, y, z],
  };
  if (a.topology === 'double-wishbone') Object.assign(p, { uca_front: [110, y * .43, z - 100], uca_rear: [-110, y * .43, z - 100], uca_outer: [0, y + 60, z - 85] });
  if (a.topology === 'macpherson') Object.assign(p, { strut_top: [0, y * .66, z - 320], strut_bottom: [0, y + 55, z - 60] });
  if (a.topology === 'multi-link') {
    delete p.lca_front; delete p.lca_rear; delete p.lca_outer;
    Object.assign(p, { lower_front_inner: [170, y * .4, z + 105], lower_front_outer: [40, y + 35, z + 95], lower_rear_inner: [-180, y * .45, z + 95], lower_rear_outer: [-35, y + 30, z + 90], upper_front_inner: [150, y * .43, z - 100], upper_front_outer: [30, y + 65, z - 85], upper_rear_inner: [-160, y * .47, z - 85], upper_rear_outer: [-35, y + 60, z - 75] });
  }
  if (a.topology !== 'macpherson') {
    const pull = a.actuation === 'pullrod';
    p.actuation_outer = [0, y + 80, pull ? z - 65 : z + 65];
    if (a.actuation === 'direct') p.damper_top = [0, y * .6, z - 285];
    else Object.assign(p, { rocker_pivot: [0, y * .42, pull ? z + 110 : z - 180], rocker_rod: [0, y * .42, pull ? z + 35 : z - 105], rocker_damper: [0, y * .42 + 65, pull ? z + 110 : z - 180], damper_top: [0, y * .18, pull ? z - 140 : z - 300] });
  }
  return p;
}
export function linkPairs(a) {
  const tie = [['tie_rod_inner', 'tie_rod_outer']];
  if (a.topology === 'multi-link') return ['lower_front','lower_rear','upper_front','upper_rear'].map(k => [k + '_inner', k + '_outer']).concat(tie);
  const lower = [['lca_front','lca_outer'], ['lca_rear','lca_outer']];
  return (a.topology === 'double-wishbone' ? lower.concat([['uca_front','uca_outer'],['uca_rear','uca_outer']]) : lower).concat(tie);
}
export const isMoving = key => key.endsWith('_outer') || ['wheel_center','strut_bottom'].includes(key);
export function changeDimensions(a, key, value) {
  const old = a[key]; a[key] = value;
  if (key === 'track') for (const p of Object.values(a.hardpoints)) p[1] *= value / old;
  if (key === 'track') a.rackLength = -a.hardpoints.tie_rod_inner[1] * 2;
  if (key === 'rackLength') a.hardpoints.tie_rod_inner[1] = -value / 2;
}
export function validateDesign(d) {
  const errors = [], number = (n, min, max, label) => { if (!Number.isFinite(n) || n < min || n > max) errors.push(`${label} must be between ${min} and ${max}.`); };
  if (!d || d.version !== 2 || !d.axles) return ['Unsupported design file. Expected Suspension Studio schema version 2.'];
  if (d.coordinateSystem !== COORDINATE_SYSTEM || d.hardpointFrame !== HARDPOINT_FRAME || d.lengthUnit !== LENGTH_UNIT) errors.push('Expected SAE_J670_Z_DOWN coordinates, AXLE_LOCAL frame, and mm units.');
  if (typeof d.name !== 'string' || !d.name.trim() || d.name.length > 80) errors.push('Design name must contain 1–80 characters.');
  number(d.wheelbase,500,10000,'Wheelbase'); number(d.wheelDiameter,200,1500,'Wheel diameter');
  for (const key of ['front','rear']) {
    const a = d.axles[key]; if (!a || !Object.hasOwn(TOPOLOGIES,a.topology)) { errors.push(`${key}: invalid topology.`); continue; }
    number(a.track,600,3000,`${key} track`); number(a.rackLength,100,2500,`${key} rack length`); number(a.rackTravel,1,300,`${key} rack travel`); number(a.springOD,20,250,`${key} spring OD`); number(a.damperOD,10,200,`${key} damper OD`);
    if (typeof a.steered !== 'boolean') errors.push(`${key}: steering must be true or false.`);
    if (!['pushrod','pullrod','direct'].includes(a.actuation) || a.mounting !== (a.actuation === 'direct' ? 'direct' : 'bell-crank') || a.topology === 'macpherson' && a.actuation !== 'direct') errors.push(`${key}: incompatible actuation and mounting.`);
    if (a.springOD <= a.damperOD) errors.push(`${key}: spring OD must be greater than damper OD.`);
    if (a.rackLength >= a.track) errors.push(`${key}: rack length must be less than track width.`);
    const required = generatePoints(a,d.wheelDiameter);
    if (!a.hardpoints || typeof a.hardpoints !== 'object') { errors.push(`${key}: missing hardpoints.`); continue; }
    let invalidCoordinates = false;
    for (const k of Object.keys(required)) {
      const p = a.hardpoints[k]; if (!Array.isArray(p) || p.length !== 3 || !p.every(n => Number.isFinite(n) && Math.abs(n) <= 10000)) { errors.push(`${key}: invalid ${k} coordinates.`); invalidCoordinates = true; }
    }
    if (Object.keys(a.hardpoints).some(k => !Object.hasOwn(required,k))) errors.push(`${key}: unexpected hardpoints for topology.`);
    if (invalidCoordinates) continue;
    const wc = a.hardpoints.wheel_center, rack = a.hardpoints.tie_rod_inner;
    if (wc && Math.abs(-wc[1] * 2 - a.track) > .1) errors.push(`${key}: left wheel center Y must equal negative half the track width.`);
    if (rack && Math.abs(-rack[1] * 2 - a.rackLength) > .1) errors.push(`${key}: left rack inner Y must equal negative half the rack length.`);
    for (const [i,o] of linkPairs(a)) if (a.hardpoints[i] && a.hardpoints[o] && Math.hypot(...a.hardpoints[i].map((v,j)=>v-a.hardpoints[o][j])) < 1) errors.push(`${key}: ${i} link is degenerate.`);
  }
  return errors;
}
export function preset(id) {
  const d = newDesign();
  if (id === 'formula') { d.name = 'Formula / double wishbone'; return d; }
  if (id === 'road') { d.name = 'Road car / strut + multi-link'; d.wheelbase = 2700; d.wheelDiameter = 640; d.axles = { front: axle('macpherson',1550), rear: axle('multi-link',1540,false) }; for (const a of Object.values(d.axles)) a.hardpoints = generatePoints(a,d.wheelDiameter); return d; }
  d.name = 'CR26 / original front geometry'; d.wheelDiameter = 406.4;
  const a = d.axles.front; a.track = 1117.6; a.rackLength = 438.16;
  // Preserve the original measurements here; convert once at the source boundary.
  const original = { uca_front:[263.53,-127,263.53],uca_rear:[232.43,127,248.77],uca_outer:[482.6,9.12,285.98],lca_front:[215.9,-117.48,120.65],lca_rear:[215.9,123.42,127],lca_outer:[533.4,-3.18,119.13],tie_rod_inner:[219.08,-69.85,151.99],tie_rod_outer:[542.93,-73.03,171.45],wheel_center:[558.8,0,203.2]};
  a.hardpoints = { ...generatePoints(a,d.wheelDiameter), ...Object.fromEntries(Object.entries(original).map(([key,point])=>[key,legacyToSAE(point)])) };
  d.axles.rear.hardpoints = generatePoints(d.axles.rear,d.wheelDiameter); return d;
}
