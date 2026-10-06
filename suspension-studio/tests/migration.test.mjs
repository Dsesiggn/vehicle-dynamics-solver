import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { axle, generatePoints, normalizeAxle, linkPairs, preset } from '../src/model.js';
import { importDesign } from '../src/migration.js';
import { legacyToSAE } from '../src/coordinates.js';
import { solveCorner, norm, sub } from '../src/solver.js';

const fixture = name => JSON.parse(readFileSync(new URL(`./fixtures/${name}`, import.meta.url)));
const near = (a, b, tolerance = 1e-4) => assert.ok(Math.abs(a - b) < tolerance, `${a} differs from ${b}`);

test('Original v1 CR26 file imports without changing link lengths or measured geometry', () => {
  const old = fixture('design-v1.json'), { design, migrated } = importDesign(old);
  assert.ok(migrated);
  assert.deepEqual(design.axles, preset('cr26').axles);
  for (const side of ['front', 'rear']) {
    for (const [i, o] of linkPairs(design.axles[side])) {
      const before = old.axles[side].hardpoints, after = design.axles[side].hardpoints;
      near(norm(sub(before[i], before[o])), norm(sub(after[i], after[o])), 1e-10);
    }
  }
});

// Captured from v1 commit afb5a078 before conversion, at bump +15 mm and
// legacy rack -12 mm (= SAE rack +12 mm). These are numerical regressions,
// not experimental validation of the physical suspension model.
for (const expected of fixture('poses-v1.json')) {
  test(`${expected.topology} / ${expected.actuation}: SAE solve matches original physical pose`, () => {
    const a = axle(expected.topology);
    a.actuation = expected.actuation;
    normalizeAxle(a);
    a.hardpoints = generatePoints(a, 460);
    const result = solveCorner(a, expected.bump, expected.rack);
    assert.ok(result.ok, result.reason);
    for (const [key, point] of Object.entries(expected.points)) {
      legacyToSAE(point).forEach((n, i) => near(result.points[key][i], n));
    }
    legacyToSAE(expected.rotation).forEach((n, i) => near(result.rotation[i], n, 1e-6));
    near(result.compression, expected.compression);
  });
}
