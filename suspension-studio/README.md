# Suspension Studio

A new, dependency-free suspension design workspace. All original repository files remain unchanged. This app is a foundation for a vehicle dynamics application, not a complete vehicle dynamics simulator.

## Run

From the repository root:

```sh
python3 -m http.server 8766 --bind 127.0.0.1
```

Open http://127.0.0.1:8766/suspension-studio/ in a modern browser and keep Terminal running. Or open [the hosted app](https://dsesiggn.github.io/vehicle-dynamics-solver/suspension-studio/), which needs no local server. No package installation or build step is required.

ES modules require HTTP/HTTPS; opening `index.html` directly with a `file://` address is not supported. The startup screen now explains this and links to the hosted app and local setup. It prevents editing an uninitialized workspace. HTTP startup also reports missing required browser features or module-load errors instead of leaving a silent blank viewer. Browser requirements include Canvas 2D, native dialogs, ResizeObserver, structuredClone, and Object.hasOwn. The source of the file-origin restriction is [MDN’s JavaScript modules guide](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Modules#other_differences_between_modules_and_classic_scripts).

Saved libraries belong to the browser and web origin. The hosted app and a local HTTP address do not share their saved library. Export a design to JSON, then use **Start from an existing design → Import design JSON** at the other address. Reload the local page after editing source files; this static server does not provide automatic refresh. If the selected port is occupied, choose another port and use that port in the browser URL.

## Workflow

1. Define a new topology or open a preset/saved design.
2. Set vehicle dimensions, then independently configure the front and rear axles.
3. Choose double wishbone, MacPherson, or five-link suspension. Pushrod and pullrod require a bell crank; direct actuation uses direct mounting. MacPherson requires a direct strut.
4. Open **Define hardpoints** to edit the generated geometry. Changes to topology or actuation regenerate that axle, with confirmation for customized geometry.
5. Open **Explore kinematics** to solve wheel bump and rack displacement. The preview shows the solved axle, or the static geometry with an explicit failure message when no solution is found.
6. Enter compression/rebound magnitudes and the shared link OD in **Heave interference check**. The default travel is 25.4 mm in each direction; the shared OD is blank until entered. Run the full sweep, then use the travel slider or a listed event to inspect the solved pose and colliding members.
7. Optionally enter engine, differential, and cockpit packaging envelopes. Check **Show in preview** to draw a translucent static shape; complete visible shapes also enter the heave clearance check. Use **Fit all** to frame the suspension and visible envelopes.
8. Save named designs in browser storage or export/import versioned JSON. Saving an existing name updates that saved design. Data does not leave the browser.

## Appearance

Use the **Dark mode** switch in the top bar to change the full workspace, including the 3D viewer. The switch supports keyboard activation and exposes its state to assistive technology. The app starts in light mode and remembers your explicit choice in this browser on this site. Theme preferences are separate from saved vehicle designs. If browser storage is unavailable, switching still works for the current page.

## U-bar anti-roll bar inputs

Enable **U-bar anti-roll bar** under the selected axle's configuration to enter four left-side XYZ points: chassis bearing center, bar bend, lever-arm tip/drop-link joint, and suspension drop-link pickup. All coordinates use the same axle-local SAE frame and millimeter units as the suspension. No coordinates are generated: blank components remain `null`, and incomplete drafts can be saved/exported. Disabling the checkbox hides the inputs and preview while retaining the points. Front and rear are independent.

The first layout is symmetric: each right-side point is `[X, -Y, Z]`. Once all coordinates are entered, the viewer draws a straight transverse section between the mirrored bends, arms from bends to lever tips, and links from tips to suspension pickups. Bearing centers are markers; for a straight transverse shaft, their X and Z should match the bends. This is a static packaging reference, including during bump/steering. It does not establish mechanical feasibility, assume which suspension body carries the pickup, or calculate ARB rotation, twist, stiffness, forces, bend radii, or clearances. The four-point representation is an implementation choice; reference passages are recorded in [the reference library](docs/REFERENCES.md#u-bar-input-and-static-layout-reference).

ARB coordinates are retained when changing track, topology, actuation, or resetting suspension hardpoints; review the ARB layout after such changes. A wheel-diameter change shifts every entered ARB Z by `−Δdiameter/2`, consistently with the suspension's ground datum adjustment; blank Z values stay blank. JSON schema v2 accepts an optional `axles.front/rear.antiRollBar` object (`type`, `enabled`, `points`). Existing files without it remain compatible. ARB extensions on v1 files are rejected because that historical format did not define ARB coordinate semantics.

## Obstacle geometry

The shared vehicle-level panel accepts an axis-aligned box for engine and differential envelopes (center X/Y/Z plus length/width/height) and a cockpit trapezoid (center X/Y/Z plus depth, top width, bottom width, and height). The cockpit cross-section lies in Y–Z and is extruded along X; equal top and bottom widths form a rectangle. All dimensions are in millimeters. Coordinate centers use SAE J670 Z-down: +X forward, +Y right, +Z down. The controls start blank; no engine, differential, or cockpit dimensions are inferred.

Each shape has a **Coordinate reference axle** selector. Switching between axle previews converts X using the documented flat vehicle-frame assumption: `X_active = X_entered + (I_active,rear − I_reference,rear) × wheelbase`, with `I=1` for rear and `I=0` for front. Thus, a front-referenced center's local X increases by `wheelbase` when shown from the rear datum; a rear-referenced center shifts by the opposite amount in the front view. Y and Z do not change because current assembly axes are parallel and have no vertical offset. This is a rendering transform, not a suspension calculation.

Unchecking an envelope retains its inputs. Blank fields represent unknown draft values and can be saved, but a visible incomplete envelope is not drawn or checked. Changing wheel diameter shifts each entered obstacle center Z by `−Δdiameter/2`, matching the existing ground-datum adjustment for suspension hardpoints; dimensions do not scale when track width changes. These simple shapes stay fixed during bump and steering. Their shape and field definitions are application choices, not equations sourced from the vehicle dynamics references. Existing v2 JSON may omit the optional top-level `obstacles` collection; once present it contains engine, differential, and cockpit records. Version 1 obstacle data is rejected because v1 does not identify a coordinate frame for it.

## Heave interference check

Each new design starts with editable compression and rebound magnitudes of **25.4 mm**. Compression is upward wheel-center motion: `bump = +compression`, so `Z_solved = Z_static − compression`. Rebound is downward: `bump = −rebound`, so `Z_solved = Z_static + rebound`. The left and right wheels receive the same heave input, rack motion is zero for this check, and each axle is evaluated independently. The existing six-pose solver leaves chassis-side hardpoints anchored and moves the upright-side hardpoints while preserving each fixed link length; the bell-crank solve pivots its rocker about the declared X axis.

The shared link OD is an editable positive value and has no invented default. It is the maximum link OD supplied by the user, so applying it to every modeled link gives a conservative packaging envelope. Suspension arms, toe links, pushrods/pullrods, and the three displayed bell-crank arms use radius `r_link = linkOD/2`. Dampers use each axle's existing damper OD with `r_damper = damperOD/2`; this covers a direct damper, a bell-crank damper, or the MacPherson strut. The current rocker is drawn as three centerline arms; those use the shared link OD until plate geometry is defined. Members are approximated as capsules: straight centerlines with a circular section and rounded ends. This is a geometry approximation, not the measured solid or joint shape.

For member–member contact, `clearance = d(segment₁, segment₂) − r₁ − r₂`, where `d` is the minimum Euclidean distance between the two finite centerline segments. For member–obstacle contact, `clearance = d(segment, solid) − r_member`; the obstacle distance is the minimum Euclidean distance from the centerline segment to the closed engine/differential box or cockpit prism. Clearance at or below zero indicates overlap of these idealized envelopes. The overlap flag is geometrically meaningful; a negative clearance is not a physical penetration-depth prediction when an axis lies inside an obstacle.

The scan samples both directions with no more than **0.25 mm** wheel-center spacing. A detected transition is bracket-refined until the travel interval is at most **0.01 mm**. The slider directly solves the selected heave position, recomputes clearance there, and highlights contacting members in the viewer. Because the sweep is sampled, a very short contact interval between samples can be missed; the slider's current-position result is not interpolated. Only complete packaging envelopes enabled by **Show in preview** are tested. Members that share a modeled pivot are excluded from member–member clearance because no bearing, ball-joint, or bracket solids are defined. Springs, tires, uprights, anti-roll bars, and chassis brackets are not included.

The geometric equations are Euclidean segment/solid distances, not equations attributed to the six vehicle-dynamics books; no measured vehicle or CAD solid has been used to validate this packaging model. Regression tests check segment and mesh distances, the SAE heave signs, link-length preservation, direct damper use, synthetic contacts, and a refined travel interval. See [the full method and limits](docs/INTERFERENCE.md). Older v1/v2 designs receive the heave defaults and a blank shared link OD when imported.

## Coordinates and parameter behavior

- **SAE J670 Z-down:** +X forward, +Y right, +Z down. Dimensions and hardpoints are millimeters. Each axle has a chassis-fixed origin at its nominal static axle station, center plane, and ground datum; it is not the vehicle CG. See the complete [coordinate and sign contract](docs/COORDINATES.md).
- Control-arm hardpoints use **fore** and **aft** in the editor, viewer labels, and validation messages, including multi-link arms. Fore means toward +X (forward); aft means toward −X (rearward), relative to the other arm attachment. These names do not require either point to have a particular coordinate sign. Existing v1/v2 JSON identifiers retain `_front`/`_rear` for saved-design compatibility.
- Enter left-side hardpoints (negative Y). Right geometry is mirrored about Y = 0. Both solved corners are returned in the same SAE frame.
- Track is wheel-center to wheel-center. Changing track scales Y coordinates, including rack length. Editing wheel-center or rack-inner Y updates the corresponding dimension; these left coordinates equal negative half the dimension.
- Increasing wheel diameter moves all points toward negative Z by the radius difference. Wheelbase is stored for future vehicle assembly; the rear datum will be at X = −wheelbase relative to the front datum. It does not alter this isolated axle preview.
- Bump is positive upward (negative Z). Rack displacement is positive rightward (+Y); rack travel is total lock-to-lock stroke. Fixed toe links remain constraints when steering is disabled.
- Version 2 JSON declares axes, frame and units. Original version 1 files and saved libraries are converted automatically; original v1 browser storage is retained. Unknown coordinate metadata is rejected. Save or export to retain a converted v2 design.
- Spring/damper OD controls the rendered packaging envelopes; the damper OD is also used in heave interference checks. The fixed 130 mm wheel width is illustrative. See the declared check scope and limitations above.

## Architecture

- `src/coordinates.js`: SAE frame definitions, reflection and alignment signs; `src/migration.js`: versioned import and saved-library conversion.
- `src/model.js`: versioned design schema, topology definitions, default geometry, validation, presets. This is the single source for geometry and configuration.
- `src/arb.js`: optional U-bar point definitions, nullable draft coordinates, validation, and completeness checks. ARB points stay separate from the suspension solver's hardpoints.
- `src/obstacles.js`: optional packaging-envelope dimensions, nullable drafts, axle-datum conversion, and simple box/trapezoid mesh vertices. These remain independent of the suspension solver.
- `src/interference.js` and `src/geometry-distance.js`: heave pose sampling, rigid member/damper centerlines, capsule clearances to convex obstacle solids and other segments, and contact-boundary refinement.
- `src/solver.js`: numerical rigid-body position solver, independent from DOM/rendering.
- `src/viewer.js`: interactive canvas rendering of projected 3D geometry, orbit/zoom, orthographic camera views. No CDN/WebGL dependency.
- `src/app.js`: editor, state transitions, local persistence, imports/exports.
- `src/theme.js`: applies the saved appearance before first paint and handles the accessible theme switch.
- `tests/*.test.mjs`: numerical, migration, physical-pose and schema regressions, run using Node 20+ with `node --test suspension-studio/tests/*.test.mjs` from the repository root.
- `tests/index.html`: browser-run version of the 40 pure model/solver checks (eight additional fixture regressions and 11 ARB input checks run in Node) for environments without Node. Open `/suspension-studio/tests/` through the local server.

The application does not execute or reinterpret the legacy dashboard, tire scripts, or Python renderer.

## Kinematic formulation

The upright has six pose unknowns: three translations and three rotation-vector components. Rodrigues' formula rotates rigidly attached hardpoints. Rotation parameters are scaled to a 300 mm characteristic length to improve conditioning.

- Double wishbone: four fixed arm-length constraints plus tie-rod length and prescribed wheel-center Z.
- Five-link: five independent fixed-length constraints plus prescribed wheel-center Z.
- MacPherson: two lower-arm constraints, tie-rod length, two perpendicular constraints keeping the chassis strut top on the rotating upright strut axis, and prescribed wheel-center Z. The strut telescopes freely.
- Bell crank: a separate scalar solve rotates a rocker around its longitudinal X axis while preserving pushrod/pullrod length. Damper compression is the change in eye-to-eye length; direct mount and strut compression use their corresponding eye distances.

Newton iteration with numerical Jacobian, partial-pivot elimination, line search, and 4 mm continuation steps searches near the static assembly. A maximum constraint residual above 0.001 mm rejects the result. Heave travel controls accept magnitudes through 100 mm, the solver's stated bound; an impossible pose, singular linkage, or unreachable rocker is reported without inventing a result. The preview returns to static geometry on a failed solve.

Camber and toe outputs are **changes from the static pose**, not absolute alignment angles. The static wheel axis is assumed lateral because the input schema does not yet specify wheel/upright orientation. Camber is positive top-outward; toe is positive inward on either side. Separate SAE steer angles are positive rightward about +Z. Forces, masses, compliance, tire contact, springs rates, damper curves, anti-roll-bar dynamics, hard stops, collision checks, motion-ratio curves, and dynamic integration are outside this first milestone. These generated hardpoints are illustrative design seeds, not optimized or validated vehicle geometry.

## CR26 provenance

The nine original front-left coordinates from `FS-CR26.py` (repository commit `7b70957b0562d037a01f8f7537b256f46f93f857`) are transformed to SAE coordinates in the CR26 preset with distances and physical geometry preserved. Front track is 1117.6 mm and rack length is 438.16 mm, derived from those coordinates. The original source script remains unchanged; the new preset applies `[X, Y, Z] = [-oldY, -oldX, -oldZ]` once at the source boundary. Front actuation, the rear axle, and the 1600 mm wheelbase are generated/assumed starting data; they are not claimed as CR26 measurements.

## Technical references

The six supplied vehicle dynamics books are cataloged in [the reference library](docs/REFERENCES.md), with review status and a workflow for documenting future equations, sign conversions and validation. Gillespie Chapter 1 and Milliken Chapter 4 support the present coordinate contract. Source PDFs remain outside the repository.

## Next development milestones

Add absolute alignment and wheel orientation; replace the illustrative upright actuation attachment with selectable arm/upright mounting; support arbitrary rocker axes and multi-body joints; introduce full-vehicle assembly and validated travel sweeps; then connect tire, spring, damper, mass/inertia and load models through explicit module contracts.
