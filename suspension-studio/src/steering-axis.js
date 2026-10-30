/** Numerical display resolution in mm, not a physical suspension design limit. */
export const STEERING_AXIS_MIN_LENGTH_MM = 1e-7;

const finitePoint = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);

/**
 * Current physical steering-pivot reference in the points' existing frame.
 * This adds no constraint or material member. The conventional MacPherson
 * kingpin joins its lower ball joint and upper strut bearing, not necessarily
 * the strut shaft endpoints. See docs/STEERING-AXIS-PREVIEW.md.
 */
export function steeringAxisGeometry(topology, points) {
  let upperKey, definition;
  const lowerKey = 'lca_outer';
  if (topology === 'double-wishbone') {
    upperKey = 'uca_outer';
    definition = 'Lower outer ball joint to upper outer ball joint';
  } else if (topology === 'macpherson') {
    upperKey = 'strut_top';
    definition = 'Lower outer ball joint to upper strut mounting/bearing center';
  } else {
    return {available:false, reason:topology === 'multi-link'
      ? 'A physical two-pivot steering axis is not defined for this multi-link model; a virtual-axis calculation is not implemented.'
      : 'This topology has no supported steering-axis definition.'};
  }
  const metadata = {definition, lowerKey, upperKey};
  const lower = points?.[lowerKey], upper = points?.[upperKey];
  if (!finitePoint(lower) || !finitePoint(upper)) {
    return {available:false, ...metadata, reason:'Steering-axis endpoints must each contain three finite coordinates in mm.'};
  }
  const difference = upper.map((value,index) => value-lower[index]);
  const length = Math.hypot(...difference);
  if (!Number.isFinite(length) || length <= STEERING_AXIS_MIN_LENGTH_MM) {
    return {available:false, ...metadata, reason:'Steering-axis endpoints coincide or are too close to resolve a direction.'};
  }
  return {available:true, ...metadata, lower:[...lower], upper:[...upper], direction:difference.map(value => value/length)};
}
