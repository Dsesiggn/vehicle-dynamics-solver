import { clone, hardpointLabel, validateDesign } from './model.js?v=0.2.8';

// Complete decimal values, with an optional complete exponent. In particular,
// a lone sign, trailing decimal point, partial exponent and hexadecimal input
// remain drafts; Number()/parseFloat() must not silently reinterpret them.
const DECIMAL = /^[+-]?(?:\d+(?:\.\d+)?|\.\d+)(?:[eE][+-]?\d+)?$/;
const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);

/**
 * Evaluate a transaction of user-entered, axle-local SAE coordinates in mm.
 * Only the touched cells are overlaid on the current design. Invalid drafts
 * return errors without replacing any previously accepted design values.
 * This is an editor boundary; it introduces no kinematic constraints.
 */
export function evaluateHardpointEdits(design, axleName, edits) {
  if (!['front', 'rear'].includes(axleName) || !record(design?.axles?.[axleName]?.hardpoints)) {
    return { axle: null, errors: ['Select a front or rear axle with existing hardpoints.'] };
  }
  if (!record(edits)) return { axle: null, errors: ['Hardpoint edits must contain named coordinate cells.'] };

  const candidate = clone(design), axle = candidate.axles[axleName], errors = [];
  for (const [key, coordinates] of Object.entries(edits)) {
    if (!Object.hasOwn(axle.hardpoints, key)) {
      errors.push(`Unknown hardpoint: ${key}.`);
      continue;
    }
    const point = axle.hardpoints[key], label = hardpointLabel(key);
    if (!record(coordinates) || !Array.isArray(point) || point.length !== 3) {
      errors.push(`${label}: edits must identify X, Y or Z coordinates.`);
      continue;
    }
    for (const [axis, raw] of Object.entries(coordinates)) {
      if (!['0', '1', '2'].includes(axis)) {
        errors.push(`${label}: unknown coordinate axis ${axis}.`);
        continue;
      }
      const coordinate = `${label} ${'XYZ'[Number(axis)]}`, text = typeof raw === 'string' ? raw.trim() : '';
      if (!text || !DECIMAL.test(text)) {
        errors.push(`${coordinate}: enter a complete decimal number in mm.`);
        continue;
      }
      const value = Number(text);
      if (!Number.isFinite(value) || Math.abs(value) > 10000) {
        errors.push(`${coordinate}: enter a finite value from −10000 to 10000 mm.`);
        continue;
      }
      point[Number(axis)] = value;
    }
  }
  if (errors.length) return { axle: null, errors };

  // The existing schema stores the left corner and defines each width by its
  // symmetric pair. Only an edit to the corresponding Y coordinate changes a
  // width; tolerated preexisting differences remain untouched otherwise.
  if (Object.hasOwn(edits, 'wheel_center') && Object.hasOwn(edits.wheel_center, '1')) axle.track = -2 * axle.hardpoints.wheel_center[1];
  if (Object.hasOwn(edits, 'tie_rod_inner') && Object.hasOwn(edits.tie_rod_inner, '1')) axle.rackLength = -2 * axle.hardpoints.tie_rod_inner[1];
  errors.push(...validateDesign(candidate));
  return { axle: errors.length ? null : axle, errors };
}
