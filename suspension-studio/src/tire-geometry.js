import { TIRE_TREAD_WIDTH_MAX } from './model.js?v=0.2.8';

const finiteVector = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);

/**
 * Geometry for a circular tread-band preview, in axle-local millimeters.
 * This does not describe the tire section profile, loaded radius, or contact patch.
 * With unit wheel axis n, band points are C + s*n + R*(u*cos(theta)+v*sin(theta)),
 * where u,v span the plane perpendicular to n and |s| <= W/2. Thus coordinate i
 * has exact half-extent |n_i|*W/2 + R*sqrt(1-n_i^2). With W missing, bounds frame
 * only the existing center-plane diameter circle; no tread width is supplied.
 */
export function tireTreadGeometry(center, axis, diameter, width = null) {
  if (!finiteVector(center) || !finiteVector(axis) || !Number.isFinite(diameter) || diameter <= 0 ||
      (width !== null && (!Number.isFinite(width) || width <= 0 || width > TIRE_TREAD_WIDTH_MAX))) return null;

  // Scaling before normalization also handles finite very large/small axis vectors.
  const scale = Math.max(...axis.map(Math.abs));
  if (scale === 0) return null;
  const scaled = axis.map(value => value / scale), magnitude = Math.hypot(...scaled);
  const normal = scaled.map(value => value / magnitude), radius = diameter / 2;
  const widthKnown = width !== null, halfWidth = widthKnown ? width / 2 : 0;
  const halfExtent = normal.map(value => Math.abs(value) * halfWidth + radius * Math.sqrt(Math.max(0, 1 - value * value)));
  const edgeCenters = widthKnown
    ? [-1, 1].map(sign => center.map((value, i) => value + sign * normal[i] * halfWidth))
    : [];
  const bounds = {
    min: center.map((value, i) => value - halfExtent[i]),
    max: center.map((value, i) => value + halfExtent[i])
  };
  // An overflowing coordinate cannot be rendered as finite preview geometry.
  if (!finiteVector(bounds.min) || !finiteVector(bounds.max) || edgeCenters.some(point => !finiteVector(point))) return null;
  return { center: [...center], axis: normal, radius, widthKnown, edgeCenters, bounds };
}
