/**
 * Position adapters between the persisted axle-local frame and the editor's
 * front-axle-origin frame, in SAE J670 Z-down millimeters.
 *
 * The rear datum is [-wheelbase, 0, 0] in the front frame. Axes are parallel;
 * this translation applies to positions only, never directions or rotations.
 * Null components remain unknown rather than being coerced to zero.
 */
function translatePoint(point, axle, wheelbase, direction) {
  if (axle !== 'front' && axle !== 'rear') throw new TypeError('Select a front or rear axle frame.');
  if (!Number.isFinite(wheelbase) || wheelbase <= 0) throw new RangeError('Wheelbase must be a finite positive number in mm.');
  if (!Array.isArray(point) || point.length !== 3 ||
      [0, 1, 2].some(index => point[index] !== null && !Number.isFinite(point[index]))) {
    throw new TypeError('A position must contain exactly three finite numeric or null coordinates.');
  }
  const result = [...point];
  if (axle === 'rear' && result[0] !== null) result[0] += direction * wheelbase;
  if (result[0] !== null && !Number.isFinite(result[0])) throw new RangeError('Translated X coordinate exceeds finite numeric range.');
  return result;
}

/** P_front = P_axle + [-L, 0, 0] for rear; front is an identity copy. */
export function axlePointToFront(point, axle, wheelbase) {
  return translatePoint(point, axle, wheelbase, -1);
}

/** P_axle = P_front + [L, 0, 0] for rear; front is an identity copy. */
export function frontPointToAxle(point, axle, wheelbase) {
  return translatePoint(point, axle, wheelbase, 1);
}
