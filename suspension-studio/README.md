# Suspension Studio

A new, dependency-free suspension design workspace. All original repository files remain unchanged. This app is a foundation for a vehicle dynamics application, not a complete vehicle dynamics simulator.

## Run

From the repository root:

```sh
python3 -m http.server 8765 --bind 127.0.0.1
```

Open http://127.0.0.1:8765/suspension-studio/ in a modern browser. ES modules require HTTP; opening the HTML directly as a file is not supported. No package installation or build step is required.

## Workflow

1. Define a new topology or open a preset/saved design.
2. Set vehicle dimensions, then independently configure the front and rear axles.
3. Choose double wishbone, MacPherson, or five-link suspension. Pushrod and pullrod require a bell crank; direct actuation uses direct mounting. MacPherson requires a direct strut.
4. Open **Define hardpoints** to edit the generated geometry. Changes to topology or actuation regenerate that axle, with confirmation for customized geometry.
5. Open **Explore kinematics** to solve wheel bump and rack displacement. The preview shows the solved axle, or the static geometry with an explicit failure message when no solution is found.
6. Save named designs in browser storage or export/import versioned JSON. Saving an existing name updates that saved design. Data does not leave the browser.

## Coordinates and parameter behavior

- **SAE J670 Z-down:** +X forward, +Y right, +Z down. Dimensions and hardpoints are millimeters. Each axle has a chassis-fixed origin at its nominal static axle station, center plane, and ground datum; it is not the vehicle CG. See the complete [coordinate and sign contract](docs/COORDINATES.md).
- Enter left-side hardpoints (negative Y). Right geometry is mirrored about Y = 0. Both solved corners are returned in the same SAE frame.
- Track is wheel-center to wheel-center. Changing track scales Y coordinates, including rack length. Editing wheel-center or rack-inner Y updates the corresponding dimension; these left coordinates equal negative half the dimension.
- Increasing wheel diameter moves all points toward negative Z by the radius difference. Wheelbase is stored for future vehicle assembly; the rear datum will be at X = −wheelbase relative to the front datum. It does not alter this isolated axle preview.
- Bump is positive upward (negative Z). Rack displacement is positive rightward (+Y); rack travel is total lock-to-lock stroke. Fixed toe links remain constraints when steering is disabled.
- Version 2 JSON declares axes, frame and units. Original version 1 files and saved libraries are converted automatically; original v1 browser storage is retained. Unknown coordinate metadata is rejected. Save or export to retain a converted v2 design.
- Spring/damper OD controls the rendered packaging envelopes. The fixed 130 mm wheel width is illustrative. No clearance/collision check is implied.

## Architecture

- `src/coordinates.js`: SAE frame definitions, reflection and alignment signs; `src/migration.js`: versioned import and saved-library conversion.
- `src/model.js`: versioned design schema, topology definitions, default geometry, validation, presets. This is the single source for geometry and configuration.
- `src/solver.js`: numerical rigid-body position solver, independent from DOM/rendering.
- `src/viewer.js`: interactive canvas rendering of projected 3D geometry, orbit/zoom, orthographic camera views. No CDN/WebGL dependency.
- `src/app.js`: editor, state transitions, local persistence, imports/exports.
- `tests/*.test.mjs`: numerical, migration, physical-pose and schema regressions, run using Node 20+ with `node --test suspension-studio/tests/*.test.mjs` from the repository root.
- `tests/index.html`: browser-run version of the 40 pure model/solver checks (eight additional fixture regressions run in Node) for environments without Node. Open `/suspension-studio/tests/` through the local server.

The application does not execute or reinterpret the legacy dashboard, tire scripts, or Python renderer.

## Kinematic formulation

The upright has six pose unknowns: three translations and three rotation-vector components. Rodrigues' formula rotates rigidly attached hardpoints. Rotation parameters are scaled to a 300 mm characteristic length to improve conditioning.

- Double wishbone: four fixed arm-length constraints plus tie-rod length and prescribed wheel-center Z.
- Five-link: five independent fixed-length constraints plus prescribed wheel-center Z.
- MacPherson: two lower-arm constraints, tie-rod length, two perpendicular constraints keeping the chassis strut top on the rotating upright strut axis, and prescribed wheel-center Z. The strut telescopes freely.
- Bell crank: a separate scalar solve rotates a rocker around its longitudinal X axis while preserving pushrod/pullrod length. Damper compression is the change in eye-to-eye length; direct mount and strut compression use their corresponding eye distances.

Newton iteration with numerical Jacobian, partial-pivot elimination, line search, and 4 mm continuation steps searches near the static assembly. A maximum constraint residual above 0.001 mm rejects the result. Motion is limited to ±40 mm in the UI; an impossible pose, singular linkage, or unreachable rocker is reported without inventing a result. The preview returns to static geometry on a failed solve.

Camber and toe outputs are **changes from the static pose**, not absolute alignment angles. The static wheel axis is assumed lateral because the input schema does not yet specify wheel/upright orientation. Camber is positive top-outward; toe is positive inward on either side. Separate SAE steer angles are positive rightward about +Z. Forces, masses, compliance, tire contact, springs rates, damper curves, anti-roll bars, hard stops, collision checks, motion-ratio curves, and dynamic integration are outside this first milestone. These generated hardpoints are illustrative design seeds, not optimized or validated vehicle geometry.

## CR26 provenance

The nine original front-left coordinates from `FS-CR26.py` (repository commit `7b70957b0562d037a01f8f7537b256f46f93f857`) are transformed to SAE coordinates in the CR26 preset with distances and physical geometry preserved. Front track is 1117.6 mm and rack length is 438.16 mm, derived from those coordinates. The original source script remains unchanged; the new preset applies `[X, Y, Z] = [-oldY, -oldX, -oldZ]` once at the source boundary. Front actuation, the rear axle, and the 1600 mm wheelbase are generated/assumed starting data; they are not claimed as CR26 measurements.

## Technical references

The six supplied vehicle dynamics books are cataloged in [the reference library](docs/REFERENCES.md), with review status and a workflow for documenting future equations, sign conversions and validation. Gillespie Chapter 1 and Milliken Chapter 4 support the present coordinate contract. Source PDFs remain outside the repository.

## Next development milestones

Add absolute alignment and wheel orientation; replace the illustrative upright actuation attachment with selectable arm/upright mounting; support arbitrary rocker axes and multi-body joints; introduce full-vehicle assembly and validated travel sweeps; then connect tire, spring, damper, mass/inertia and load models through explicit module contracts.
