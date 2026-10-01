/** Visual packaging envelopes in SAE J670 Z-down millimeters; no collision or force model. */
export const OBSTACLE_TYPES = [
  { key: 'engine', label: 'Engine', dimensions: [
    { key: 'length', label: 'Length', axis: 'X' },
    { key: 'width', label: 'Width', axis: 'Y' },
    { key: 'height', label: 'Height', axis: 'Z' },
  ] },
  { key: 'differential', label: 'Differential', dimensions: [
    { key: 'length', label: 'Length', axis: 'X' },
    { key: 'width', label: 'Width', axis: 'Y' },
    { key: 'height', label: 'Height', axis: 'Z' },
  ] },
  { key: 'cockpit', label: 'Cockpit cross-section', dimensions: [
    { key: 'depth', label: 'Depth', axis: 'X' },
    { key: 'topWidth', label: 'Top width', axis: 'Y' },
    { key: 'bottomWidth', label: 'Bottom width', axis: 'Y' },
    { key: 'height', label: 'Height', axis: 'Z' },
  ] },
];

/** Blank values represent unentered measurements, never inferred vehicle dimensions. */
export function newObstacles() {
  return Object.fromEntries(OBSTACLE_TYPES.map(({ key, dimensions }) => [key, {
    visible: false, referenceAxle: 'front', center: [null, null, null],
    dimensions: Object.fromEntries(dimensions.map(dimension => [dimension.key, null])),
  }]));
}

const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);
const coordinate = value => Number.isFinite(value) && Math.abs(value) <= 10000;
const dimension = value => Number.isFinite(value) && value >= 0.001 && value <= 10000;
const axles = ['front', 'rear'];
const definition = kind => OBSTACLE_TYPES.find(({ key }) => key === kind);

function validateObstacle(obstacle, type) {
  if (!type) return ['Unknown obstacle geometry type.'];
  const prefix = `Obstacle ${type.label}`;
  if (!record(obstacle)) return [`${prefix} must be an object.`];
  const errors = [];
  const fields = ['visible', 'referenceAxle', 'center', 'dimensions'];
  if (Object.keys(obstacle).some(key => !fields.includes(key))) errors.push(`${prefix}: unexpected property.`);
  if (!Object.hasOwn(obstacle, 'visible') || typeof obstacle.visible !== 'boolean') errors.push(`${prefix}: visible must be true or false.`);
  if (!Object.hasOwn(obstacle, 'referenceAxle') || !axles.includes(obstacle.referenceAxle)) errors.push(`${prefix}: reference axle must be front or rear.`);
  if (!Object.hasOwn(obstacle, 'center') || !Array.isArray(obstacle.center) || obstacle.center.length !== 3 ||
    ![0, 1, 2].every(index => obstacle.center[index] === null || coordinate(obstacle.center[index]))) {
    errors.push(`${prefix}: use three center coordinates, each blank or between -10000 and 10000 mm.`);
  }
  if (!Object.hasOwn(obstacle, 'dimensions') || !record(obstacle.dimensions)) return [...errors, `${prefix}: dimensions must be an object.`];
  const expected = type.dimensions.map(({ key }) => key);
  if (Object.keys(obstacle.dimensions).some(key => !expected.includes(key))) errors.push(`${prefix}: unexpected dimension.`);
  for (const { key, label } of type.dimensions) {
    if (!Object.hasOwn(obstacle.dimensions, key) || !(obstacle.dimensions[key] === null || dimension(obstacle.dimensions[key]))) {
      errors.push(`${prefix}: ${label} must be blank or between 0.001 and 10000 mm.`);
    }
  }
  return errors;
}

export function validateObstacles(collection) {
  if (collection === undefined) return []; // Existing v2 designs may omit the entire extension.
  if (!record(collection)) return ['Obstacle geometry must be an object.'];
  const errors = [];
  const expected = OBSTACLE_TYPES.map(({ key }) => key);
  if (Object.keys(collection).some(key => !expected.includes(key))) errors.push('Unexpected obstacle geometry type.');
  for (const type of OBSTACLE_TYPES) {
    if (!Object.hasOwn(collection, type.key)) errors.push(`Obstacle ${type.label} is missing.`);
    else errors.push(...validateObstacle(collection[type.key], type));
  }
  return errors;
}

/** Completeness is independent of visibility; unchecking an envelope preserves all inputs. */
export function obstacleComplete(obstacle, kind) {
  const type = definition(kind);
  return Boolean(type && validateObstacle(obstacle, type).length === 0 &&
    [0, 1, 2].every(index => coordinate(obstacle.center[index])) &&
    type.dimensions.every(({ key }) => dimension(obstacle.dimensions[key])));
}

/**
 * Axis-aligned, chassis-fixed envelopes, never mirrored with the suspension corners.
 * The rear axle origin is [-wheelbase, 0, 0] in the front frame, so:
 * X_active = X_reference + (activeRear - referenceRear) * wheelbase.
 * Cockpit top is -Z: its centered YZ trapezoid is extruded along X.
 */
export function obstacleMesh(obstacle, kind, activeAxle, wheelbase) {
  if (!obstacleComplete(obstacle, kind) || !obstacle.visible || !axles.includes(activeAxle) || !dimension(wheelbase)) return null;
  const d = obstacle.dimensions;
  const halfLength = (kind === 'cockpit' ? d.depth : d.length) / 2;
  const topHalfWidth = (kind === 'cockpit' ? d.topWidth : d.width) / 2;
  const bottomHalfWidth = (kind === 'cockpit' ? d.bottomWidth : d.width) / 2;
  const halfHeight = d.height / 2;
  const shift = (activeAxle === 'rear' ? wheelbase : 0) - (obstacle.referenceAxle === 'rear' ? wheelbase : 0);
  const [cx, cy, cz] = obstacle.center;
  const section = [[-topHalfWidth, -halfHeight], [topHalfWidth, -halfHeight], [bottomHalfWidth, halfHeight], [-bottomHalfWidth, halfHeight]];
  const vertices = [-halfLength, halfLength].flatMap(x => section.map(([y, z]) => [cx + shift + x, cy + y, cz + z]));
  return {
    vertices,
    // Consistent outward winding, including the tapered cockpit sides.
    faces: [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]],
    edges: [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]],
  };
}
