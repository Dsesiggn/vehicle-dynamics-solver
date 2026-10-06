/** Euclidean distance between a line segment and a closed convex triangle mesh.
 * The mesh must use outward-wound faces and describe a convex solid.
 */
const sub = (a, b) => a.map((value, i) => value - b[i]);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm2 = a => dot(a, a);
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

function pointTriangleDistanceSquared(p, a, b, c) {
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return norm2(ap);
  const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return norm2(bp);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return norm2(sub(p, a.map((v, i) => v + ab[i] * d1 / (d1 - d3))));
  const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return norm2(cp);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return norm2(sub(p, a.map((v, i) => v + ac[i] * d2 / (d2 - d6))));
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const bc = sub(c, b), t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return norm2(sub(p, b.map((v, i) => v + bc[i] * t)));
  }
  const denominator = 1 / (va + vb + vc), v = vb * denominator, w = vc * denominator;
  const closest = a.map((value, i) => value + ab[i] * v + ac[i] * w);
  return norm2(sub(p, closest));
}

function segmentSegmentDistanceSquared(p1, q1, p2, q2) {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const aa = dot(d1, d1), ee = dot(d2, d2), ff = dot(d2, r);
  let s, t;
  if (aa <= 1e-24 && ee <= 1e-24) return norm2(r);
  if (aa <= 1e-24) { s = 0; t = clamp(ff / ee, 0, 1); }
  else {
    const cc = dot(d1, r);
    if (ee <= 1e-24) { t = 0; s = clamp(-cc / aa, 0, 1); }
    else {
      const bb = dot(d1, d2), denominator = aa * ee - bb * bb;
      s = denominator > 1e-24 ? clamp((bb * ff - cc * ee) / denominator, 0, 1) : 0;
      t = (bb * s + ff) / ee;
      if (t < 0) { t = 0; s = clamp(-cc / aa, 0, 1); }
      else if (t > 1) { t = 1; s = clamp((bb - cc) / aa, 0, 1); }
    }
  }
  const delta = r.map((value, i) => value + d1[i] * s - d2[i] * t);
  return norm2(delta);
}

/** Exact minimum centerline distance between two finite line segments. */
export function segmentSegmentDistance(p1, q1, p2, q2) {
  if (![p1, q1, p2, q2].every(point => Array.isArray(point) && point.length === 3 && point.every(Number.isFinite))) return NaN;
  return Math.sqrt(segmentSegmentDistanceSquared(p1, q1, p2, q2));
}

function projectedPoint(point, dropAxis) {
  return point.filter((_, i) => i !== dropAxis);
}

function orient2(a, b, c) { return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]); }

function pointInTriangle2(p, a, b, c, tolerance = 1e-9) {
  const ab = orient2(a, b, p), bc = orient2(b, c, p), ca = orient2(c, a, p);
  return (ab >= -tolerance && bc >= -tolerance && ca >= -tolerance) ||
    (ab <= tolerance && bc <= tolerance && ca <= tolerance);
}

function segmentsIntersect2(p1, p2, q1, q2, tolerance = 1e-9) {
  const a = orient2(p1, p2, q1), b = orient2(p1, p2, q2), c = orient2(q1, q2, p1), d = orient2(q1, q2, p2);
  if (((a > tolerance && b < -tolerance) || (a < -tolerance && b > tolerance)) &&
      ((c > tolerance && d < -tolerance) || (c < -tolerance && d > tolerance))) return true;
  const on = (p, q, r) => Math.abs(orient2(p, q, r)) <= tolerance &&
    r[0] >= Math.min(p[0], q[0]) - tolerance && r[0] <= Math.max(p[0], q[0]) + tolerance &&
    r[1] >= Math.min(p[1], q[1]) - tolerance && r[1] <= Math.max(p[1], q[1]) + tolerance;
  return on(p1, p2, q1) || on(p1, p2, q2) || on(q1, q2, p1) || on(q1, q2, p2);
}

function segmentIntersectsTriangle(p, q, a, b, c) {
  const direction = sub(q, p), edge1 = sub(b, a), edge2 = sub(c, a), h = cross(direction, edge2), determinant = dot(edge1, h);
  if (Math.abs(determinant) < 1e-12) return false;
  const inverse = 1 / determinant, s = sub(p, a), u = inverse * dot(s, h);
  if (u < -1e-10 || u > 1 + 1e-10) return false;
  const r = cross(s, edge1), v = inverse * dot(direction, r);
  if (v < -1e-10 || u + v > 1 + 1e-10) return false;
  const t = inverse * dot(edge2, r);
  return t >= -1e-10 && t <= 1 + 1e-10;
}

function segmentTriangleDistanceSquared(p, q, a, b, c) {
  if (segmentIntersectsTriangle(p, q, a, b, c)) return 0;
  let best = Math.min(pointTriangleDistanceSquared(p, a, b, c), pointTriangleDistanceSquared(q, a, b, c));
  best = Math.min(best,
    segmentSegmentDistanceSquared(p, q, a, b),
    segmentSegmentDistanceSquared(p, q, b, c),
    segmentSegmentDistanceSquared(p, q, c, a));

  // A segment parallel to a face can project through the face interior even
  // when its endpoints and the nearest boundary edges are far away.
  const normal = cross(sub(b, a), sub(c, a)), normalLength = Math.sqrt(norm2(normal));
  const direction = sub(q, p), normalMotion = dot(normal, direction);
  if (normalLength > 1e-15 && Math.abs(normalMotion) <= 1e-12 * normalLength * Math.max(1, Math.sqrt(norm2(direction)))) {
    const dropAxis = normal.map(Math.abs).indexOf(Math.max(...normal.map(Math.abs)));
    const pp = projectedPoint(p, dropAxis), qp = projectedPoint(q, dropAxis);
    const aa = projectedPoint(a, dropAxis), bb = projectedPoint(b, dropAxis), cc = projectedPoint(c, dropAxis);
    const projectedHit = pointInTriangle2(pp, aa, bb, cc) || pointInTriangle2(qp, aa, bb, cc) ||
      segmentsIntersect2(pp, qp, aa, bb) || segmentsIntersect2(pp, qp, bb, cc) || segmentsIntersect2(pp, qp, cc, aa);
    if (projectedHit) {
      const signed = dot(normal, sub(p, a)) / normalLength;
      best = Math.min(best, signed * signed);
    }
  }
  return best;
}

function pointInsideConvexMesh(point, mesh) {
  for (const face of mesh.faces) {
    const a = mesh.vertices[face[0]], b = mesh.vertices[face[1]], c = mesh.vertices[face[2]];
    const normal = cross(sub(b, a), sub(c, a)), length = Math.sqrt(norm2(normal));
    if (length < 1e-15) continue;
    if (dot(normal, sub(point, a)) > 1e-9 * length) return false;
  }
  return true;
}

/** Exact centerline-to-solid distance for a segment and the app's convex obstacle meshes. */
export function segmentMeshDistance(p, q, mesh) {
  if (!Array.isArray(p) || !Array.isArray(q) || p.length !== 3 || q.length !== 3 ||
      !p.every(Number.isFinite) || !q.every(Number.isFinite) ||
      !mesh || !Array.isArray(mesh.vertices) || !Array.isArray(mesh.faces) || !mesh.faces.length) return NaN;
  if (pointInsideConvexMesh(p, mesh) || pointInsideConvexMesh(q, mesh)) return 0;
  let minimumSquared = Infinity;
  for (const face of mesh.faces) {
    if (face.length < 3) continue;
    const a = mesh.vertices[face[0]];
    for (let i = 1; i < face.length - 1; i++) {
      const b = mesh.vertices[face[i]], c = mesh.vertices[face[i + 1]];
      if (segmentIntersectsTriangle(p, q, a, b, c)) return 0;
      minimumSquared = Math.min(minimumSquared, segmentTriangleDistanceSquared(p, q, a, b, c));
    }
  }
  return Math.sqrt(minimumSquared);
}
