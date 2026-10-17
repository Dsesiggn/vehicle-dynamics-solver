/**
 * Longitudinal rack placement editor in front-axle-local SAE J670 millimeters.
 * xRack=(left.inner.X+right.inner.X)/2. A common X translation preserves the
 * joint-to-joint X difference; rack actuation remains purely along +Y.
 * This derives an editor value from existing hardpoints, adding no schema or
 * steering solver law. The entered offset is positive forward of the axle.
 */
import { steeringNumerics } from './steering.js';

const MAX_COORDINATE_MM = steeringNumerics.maxCoordinateOrLengthMm;
const finite = value => typeof value === 'number' && Number.isFinite(value);
const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);
const innerPoint = value => Array.isArray(value) && value.length === 3 && [0, 1, 2].every(i => value[i] === null || finite(value[i]));

function innerJoints(design) {
  if (!record(design) || !record(design.left) || !record(design.right)) return null;
  if (!innerPoint(design.left.tieInner) || !innerPoint(design.right.tieInner)) return null;
  return { left: design.left.tieInner, right: design.right.tieInner };
}

/** Unknown or malformed joint coordinates never imply an inferred offset. */
export function rackLongitudinalOffset(design) {
  const joints = innerJoints(design);
  if (!joints || !finite(joints.left[0]) || !finite(joints.right[0])) return null;
  // Halving first also avoids overflow for otherwise finite imported numbers.
  const offset = joints.left[0] / 2 + joints.right[0] / 2;
  return finite(offset) ? offset : null;
}

/**
 * translate: ΔX=offset−xRack, then X_left/right += ΔX. Both X must be known.
 * set-both: explicitly assign both X=offset, including when either was null.
 * Unchanged fields use structural sharing; neither input object nor arrays
 * are modified. The two edited point arrays and side objects are new copies.
 */
export function setRackLongitudinalOffset(design, offset, { mode = 'translate' } = {}) {
  const fail = reason => ({ ok: false, reason });
  if (!finite(offset)) return fail('Rack longitudinal offset must be a finite number in millimeters; blank or nonnumeric input is not converted to zero.');
  if (Math.abs(offset) > MAX_COORDINATE_MM) return fail(`Rack longitudinal offset must lie within ±${MAX_COORDINATE_MM} mm.`);
  if (mode !== 'translate' && mode !== 'set-both') return fail('Rack offset mode must be translate or set-both.');
  const joints = innerJoints(design);
  if (!joints) return fail('Both inner-joint points must already contain exactly three finite or explicitly null coordinates; missing or malformed arrays are not inferred.');
  let leftX, rightX;
  if (mode === 'translate') {
    if (!finite(joints.left[0]) || !finite(joints.right[0])) return fail('Both inner-joint X coordinates must be known before translating the rack. Use Set both rack X values explicitly to assign unknown X coordinates.');
    if (Math.abs(joints.left[0]) > MAX_COORDINATE_MM || Math.abs(joints.right[0]) > MAX_COORDINATE_MM) return fail(`Existing inner-joint X coordinates must lie within ±${MAX_COORDINATE_MM} mm before translation.`);
    const delta = offset - rackLongitudinalOffset(design);
    leftX = joints.left[0] + delta;
    rightX = joints.right[0] + delta;
  } else {
    leftX = offset;
    rightX = offset;
  }
  if (![leftX, rightX].every(value => finite(value) && Math.abs(value) <= MAX_COORDINATE_MM)) return fail(`Translating this rack would place an inner-joint X coordinate outside ±${MAX_COORDINATE_MM} mm; neither joint was changed.`);
  const nextDesign = {
    ...design,
    left: { ...design.left, tieInner: [leftX, joints.left[1], joints.left[2]] },
    right: { ...design.right, tieInner: [rightX, joints.right[1], joints.right[2]] },
  };
  return { ok: true, design: nextDesign, mode, offset };
}
