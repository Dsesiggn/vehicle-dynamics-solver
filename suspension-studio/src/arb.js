/** Optional U-bar geometry: LEFT-side, axle-local SAE J670 Z-down coordinates in mm. */
export const ARB_POINTS = [
  { key: 'bearing', label: 'Chassis bearing', description: 'Center of the left chassis bearing supporting the transverse bar.' },
  { key: 'bend', label: 'Bar bend', description: 'Centerline bend where the transverse bar meets the left lever arm.' },
  { key: 'arm_tip', label: 'Lever-arm tip', description: 'Drop-link attachment on the left bar lever arm.' },
  { key: 'link_pickup', label: 'Suspension pickup', description: 'Left suspension attachment for the drop link.' },
];

export function newAntiRollBar() {
  return { type: 'u-bar', enabled: false, points: Object.fromEntries(ARB_POINTS.map(({ key }) => [key, [null, null, null]])) };
}

const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);
const coordinate = value => Number.isFinite(value) && Math.abs(value) <= 10000;

/** Null coordinates are deliberately unentered draft values, never zero substitutes. */
export function validateAntiRollBar(arb) {
  if (arb === undefined) return []; // Existing v2 designs have no ARB field.
  if (!record(arb)) return ['Anti-roll bar must be an object.'];
  const errors = [];
  if (arb.type !== 'u-bar') errors.push('Anti-roll bar type must be u-bar.');
  if (typeof arb.enabled !== 'boolean') errors.push('Anti-roll bar enabled must be true or false.');
  if (!record(arb.points)) return [...errors, 'Anti-roll bar points must be an object.'];
  const keys = ARB_POINTS.map(({ key }) => key);
  if (Object.keys(arb.points).some(key => !keys.includes(key))) errors.push('Unexpected anti-roll bar point.');
  for (const { key, label } of ARB_POINTS) {
    const point = arb.points[key];
    if (!Object.hasOwn(arb.points, key) || !Array.isArray(point) || point.length !== 3 ||
      ![0, 1, 2].every(index => point[index] === null || coordinate(point[index]))) {
      errors.push(`Anti-roll bar ${label}: use three coordinates, each blank or between -10000 and 10000 mm.`);
      continue;
    }
    if (point[1] !== null && point[1] > 0) errors.push(`Anti-roll bar ${label}: left-side Y must be zero or negative.`);
  }
  return errors;
}

/** Completeness concerns geometry only; a complete bar can still be disabled. */
export function arbComplete(arb) {
  return arb !== undefined && validateAntiRollBar(arb).length === 0 &&
    ARB_POINTS.every(({ key }) => [0, 1, 2].every(index => coordinate(arb.points[key][index])));
}
