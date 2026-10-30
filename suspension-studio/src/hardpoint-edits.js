import { clone, hardpointLabel, validateDesign } from './model.js?v=0.2.12';
import { frontPointToAxle } from './coordinate-frames.js?v=0.2.11';

// Complete decimal values, with an optional complete exponent. In particular,
// a lone sign, trailing decimal point, partial exponent and hexadecimal input
// remain drafts; Number()/parseFloat() must not silently reinterpret them.
const DECIMAL = /^[+-]?(?:\d+(?:\.\d+)?|\.\d+)(?:[eE][+-]?\d+)?$/;
const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);

/**
 * Evaluate SAE coordinates in the declared input frame, returning axle-local mm.
 * Only the touched cells are overlaid on the current design. Invalid drafts
 * return errors without replacing any previously accepted design values.
 * This is an editor boundary; it introduces no kinematic constraints.
 */
export function evaluateHardpointEdits(design, axleName, edits, {frame='AXLE_LOCAL'}={}) {
  if (!['front', 'rear'].includes(axleName) || !record(design?.axles?.[axleName]?.hardpoints)) {
    return { axle: null, errors: ['Select a front or rear axle with existing hardpoints.'] };
  }
  if (!record(edits)) return { axle: null, errors: ['Hardpoint edits must contain named coordinate cells.'] };
  if (!['AXLE_LOCAL','FRONT_AXLE'].includes(frame)) return { axle: null, errors: ['Unknown hardpoint input frame.'] };
  if (frame==='FRONT_AXLE'&&(!Number.isFinite(design.wheelbase)||design.wheelbase<=0)) return {axle:null,errors:['A finite positive wheelbase is required for front-origin inputs.']};

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
      const entered = Number(text), offset=frame==='FRONT_AXLE'&&axleName==='rear'&&axis==='0'?design.wheelbase:0;
      if (!Number.isFinite(entered) || entered < -10000-offset || entered > 10000-offset) {
        errors.push(`${coordinate}: enter a finite value from ${-10000-offset} to ${10000-offset} mm${frame==='FRONT_AXLE'?' from the front origin':''}.`);
        continue;
      }
      const value=frame==='FRONT_AXLE'&&axis==='0'?frontPointToAxle([entered,0,0],axleName,design.wheelbase)[0]:entered;
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
