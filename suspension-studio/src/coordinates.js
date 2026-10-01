/** SAE J670 Z-down orientation. See docs/COORDINATES.md for origins and signs. */
export const COORDINATE_SYSTEM = 'SAE_J670_Z_DOWN';
export const HARDPOINT_FRAME = 'AXLE_LOCAL';
export const LENGTH_UNIT = 'mm';

// Studio v1: lateral-left, longitudinal-rearward, vertical-up. Proper rotation,
// determinant +1: applies to positions, free vectors, and rotation vectors.
const negate = value => value === 0 ? 0 : -value;
export const legacyToSAE = ([lateral, longitudinal, vertical]) => [negate(longitudinal), negate(lateral), negate(vertical)];
export const mirrorPoint = ([x, y, z]) => [x, -y, z];
// Rotation vectors are axial vectors: reflection uses det(M) M, not M.
export const mirrorRotation = ([rx, ry, rz]) => [-rx, ry, -rz];
// Rendering basis only. Never persist screen coordinates or use them in physics.
export const toDisplay = ([x, y, z]) => [-y, -x, -z];

/** Outward wheel spindle, SAE components; side -1 left, +1 right. Degrees. */
export function alignmentFromAxis(outward, side) {
  const [x, y, z] = outward, degrees = 180 / Math.PI;
  const steer = Math.atan2(-side * x, side * y) * degrees;
  return {
    camber: Math.atan2(z, Math.hypot(x, y)) * degrees,
    toe: -side * steer, // Positive toe-in on either side, not a yaw component.
    steer,
  };
}
