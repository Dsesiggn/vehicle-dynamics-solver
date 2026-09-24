# Suspension Studio

A new, dependency-free suspension design workspace. All original repository files remain unchanged. This app is a foundation for a vehicle dynamics application, not a complete vehicle dynamics simulator.

## Run

From the repository root:

```sh
python3 -m http.server 8765 --bind 127.0.0.1
```

Open http://localhost:8765/suspension-studio/ in a modern browser. ES modules require HTTP; opening the HTML directly as a file is not supported. No package installation or build step is required.

## Workflow

1. Define a new topology or open a preset/saved design.
2. Set vehicle dimensions, then independently configure the front and rear axles.
3. Choose double wishbone, MacPherson, or five-link suspension. Pushrod and pullrod require a bell crank; direct actuation uses direct mounting. MacPherson requires a direct strut.
4. Open **Define hardpoints** to edit the generated geometry. Changes to topology or actuation regenerate that axle, with confirmation for customized geometry.
5. Open **Explore kinematics** to solve wheel bump and rack displacement. The preview shows the solved axle, or the static geometry with an explicit failure message when no solution is found.
6. Save named designs in browser storage or export/import versioned JSON. Saving an existing name updates that saved design. Data does not leave the browser.

## Coordinates and parameter behavior

- All dimensions and hardpoints use millimeters. X points laterally left; Y is longitudinal; Z points up. Each axle has its own local origin at ground level. Front/rear installation transforms can place the rear axle at Y = wheelbase in a future full-vehicle assembly.
- Enter the left side. Right geometry is mirrored about X = 0, including the sign of the rack motion in the local corner solve.
- Track is wheel-center to wheel-center. Changing track scales the axle's lateral coordinates, including rack length. Changing rack length updates the inner tie-rod X coordinate. Editing wheel-center or rack-inner X updates the corresponding dimension.
- Wheel diameter shifts all Z coordinates by the radius difference, preserving geometry relative to the wheel center. Wheelbase is stored for vehicle assembly; it does not alter this isolated axle preview.
- Total rack travel is lock-to-lock, so the steering slider runs from negative half travel to positive half travel. Fixed rear toe links remain kinematic constraints when rear steering is disabled.
- Spring/damper OD controls the rendered packaging envelopes. The fixed 130 mm wheel width is illustrative. No clearance/collision check is implied.

## Architecture

- `src/model.js`: versioned design schema, topology definitions, default geometry, validation, presets. This is the single source for geometry and configuration.
- `src/solver.js`: numerical rigid-body position solver, independent from DOM/rendering.
- `src/viewer.js`: interactive canvas rendering of projected 3D geometry, orbit/zoom, orthographic camera views. No CDN/WebGL dependency.
- `src/app.js`: editor, state transitions, local persistence, imports/exports.
- `tests/model.test.mjs`: numerical and schema regressions, run using Node 20+ with `node --test suspension-studio/tests/model.test.mjs` from the repository root.
- `tests/index.html`: browser-run version of the same pure model/solver checks for environments without Node. Open `/suspension-studio/tests/` through the local server.

The application does not execute or reinterpret the legacy dashboard, tire scripts, or Python renderer.

## Kinematic formulation

The upright has six pose unknowns: three translations and three rotation-vector components. Rodrigues' formula rotates rigidly attached hardpoints. Rotation parameters are scaled to a 300 mm characteristic length to improve conditioning.

- Double wishbone: four fixed arm-length constraints plus tie-rod length and prescribed wheel-center Z.
- Five-link: five independent fixed-length constraints plus prescribed wheel-center Z.
- MacPherson: two lower-arm constraints, tie-rod length, two perpendicular constraints keeping the chassis strut top on the rotating upright strut axis, and prescribed wheel-center Z. The strut telescopes freely.
- Bell crank: a separate scalar solve rotates a rocker around its local Y axis while preserving pushrod/pullrod length. Damper compression is the change in eye-to-eye length; direct mount and strut compression use their corresponding eye distances.

Newton iteration with numerical Jacobian, partial-pivot elimination, line search, and 4 mm continuation steps searches near the static assembly. A maximum constraint residual above 0.001 mm rejects the result. Motion is limited to ±40 mm in the UI; an impossible pose, singular linkage, or unreachable rocker is reported without inventing a result. The preview returns to static geometry on a failed solve.

Camber and toe outputs are **changes from the static pose**, not absolute alignment angles. The static wheel axis is assumed lateral because the input schema does not yet specify wheel/upright orientation. Right toe is expressed in its mirrored local coordinate system. Forces, masses, compliance, tire contact, springs rates, damper curves, anti-roll bars, hard stops, collision checks, motion-ratio curves, and dynamic integration are outside this first milestone. These generated hardpoints are illustrative design seeds, not optimized or validated vehicle geometry.

## CR26 provenance

The nine original front-left coordinates from `FS-CR26.py` (repository commit `7b70957b0562d037a01f8f7537b256f46f93f857`) are retained exactly in the CR26 preset. Front track is 1117.6 mm and rack length is 438.16 mm, derived from those coordinates. The original script's axis convention is preserved. Front actuation, the rear axle, and the 1600 mm wheelbase are generated/assumed starting data; they are not claimed as CR26 measurements.

## Next development milestones

Add absolute alignment and wheel orientation; replace the illustrative upright actuation attachment with selectable arm/upright mounting; support arbitrary rocker axes and multi-body joints; introduce full-vehicle assembly and validated travel sweeps; then connect tire, spring, damper, mass/inertia and load models through explicit module contracts.
