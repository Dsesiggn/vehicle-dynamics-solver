# Suspension Studio

A suspension design workspace with a Python steering engine. The original root applications are preserved. This app is a foundation for a vehicle dynamics application, not a complete vehicle dynamics simulator.

## Run

For a computer where Python cannot be installed, open the hosted
[Suspension Studio](https://vehicle-dynamics-studio.onrender.com/suspension-studio/)
or [Steering Studio](https://vehicle-dynamics-studio.onrender.com/suspension-studio/steering.html).
Both use the same deployed application; steering calculations run on the server.
See [browser-only deployment](../docs/WEB_DEPLOYMENT.md) for updates, version checks,
and transferring designs. GitHub Pages alone cannot run the steering backend.

For the steering linkage tool and engineering workbench, run the Python service from the repository root:

```sh
python3 setup_hybrid.py
.venv-hybrid/bin/python run_hybrid.py --open
```

Setup is needed once. Open `http://127.0.0.1:8767/suspension-studio/steering.html` for steering. The service calculates the wheel poses, 5° sweep and construction geometry; no JavaScript calculation fallback runs if it is unavailable. See [the hybrid quickstart](../docs/HYBRID_QUICKSTART.md) and [steering backend contract](../docs/STEERING_BACKEND.md).

The suspension editor still supports a static server:

```sh
python3 -m http.server 8766 --bind 127.0.0.1
```

Open http://127.0.0.1:8766/suspension-studio/ in a modern browser and keep Terminal running. Or open [the hosted suspension editor](https://dsesiggn.github.io/vehicle-dynamics-solver/suspension-studio/), which needs no local server. The static suspension editor needs no package installation or build step; static hosting cannot execute the Python steering engine.

ES modules require HTTP/HTTPS; opening `index.html` directly with a `file://` address is not supported. The startup screen now explains this and links to the hosted app and local setup. It prevents editing an uninitialized workspace. HTTP startup also reports missing required browser features or module-load errors instead of leaving a silent blank viewer. Browser requirements include Canvas 2D, native dialogs, ResizeObserver, structuredClone, and Object.hasOwn. The source of the file-origin restriction is [MDN’s JavaScript modules guide](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Modules#other_differences_between_modules_and_classic_scripts).

Saved libraries belong to the browser and web origin. The hosted app and a local HTTP address do not share their saved library. Export a design to JSON, then use **Start from an existing design → Import design JSON** at the other address. Reload the local page after editing source files; this static server does not provide automatic refresh. If the selected port is occupied, choose another port and use that port in the browser URL.

## Workflow

1. Define a new topology or open a preset/saved design.
2. Set vehicle dimensions, then independently configure the front and rear axles.
3. Choose double wishbone, MacPherson, or five-link suspension. Pushrod and pullrod require a bell crank; direct actuation uses direct mounting. For each double-wishbone axle, choose whether the outer actuation pickup belongs to the upright, lower wishbone or upper wishbone. MacPherson requires a direct strut.
4. Open **Define hardpoints** to edit the generated geometry. Changes to topology or actuation regenerate that axle, with confirmation for customized geometry.
5. Open **Explore kinematics** to solve wheel bump and rack displacement. The preview shows the solved axle, or the static geometry with an explicit failure message when no solution is found.
6. Enter compression/rebound magnitudes and the shared link OD in **Heave interference check**. The default travel is 25.4 mm in each direction; the shared OD is blank until entered. Run the full sweep, then use the travel slider or a listed event to inspect the solved pose and colliding members.
7. Optionally enter engine, differential, and cockpit packaging envelopes. Check **Show in preview** to draw a translucent static shape; complete visible shapes also enter the heave clearance check. Use **Fit all** to frame the suspension and visible envelopes.
8. Save named designs in browser storage or export/import versioned JSON. Saving an existing name updates that saved design. Suspension calculations remain in the browser; steering designs are sent to the same-origin Python service for calculation and are not stored there.

## Hardpoint editing and live preview

**Define hardpoints** places the coordinate table beside the live preview on desktop. Use **Front hardpoints** and **Rear hardpoints** to select the axle, and **Configure vehicle** to return to dimensions and topology. Accepted coordinate changes update and reframe the preview immediately. Coordinates are not rounded to two decimal places by the editor.

**All coordinate inputs use the front-axle origin.** The front datum is `[0,0,0]`; the rear datum is `[-wheelbase,0,0]`. With a 1530 mm wheelbase, enter X = −1530 mm for a rear point at the nominal rear-axle station. The preview shows the selected axle at its assembly position and marks both datums. Y remains positive right and Z positive down.

Saved v2 JSON and the suspension solver retain their existing axle-local representation. The editor converts rear X with `X_local = X_entered + wheelbase` and displays it with the inverse conversion. Loading an existing design therefore preserves its geometry while showing front-origin rear X values. This conversion also applies to the rear ARB coordinate editor. Use the declared `AXLE_LOCAL` frame when editing raw JSON; the interface converts front-origin values automatically.

Incomplete or invalid entries remain editable. A persistent message explains why they have not been applied, and the preview displays the last accepted static geometry. Saving, exporting and solving stay disabled while either axle has unaccepted edits. Correct the entries or use **Discard unaccepted edits** to restore the accepted coordinates. Pending edits are not saved in the design library.

This is an input-validation and rendering change, not a new kinematic model. The existing symmetric-left-corner convention is unchanged: editing wheel-center Y sets track to `−2Y`, and editing rack-inner Y sets rack length to `−2Y`, in mm. Other coordinate edits preserve the existing widths. The editor evaluates all pending coordinate changes together against the existing design validation before replacing the accepted axle; blanks never become zero. See [the coordinate contract](docs/COORDINATES.md).

## Outer actuation pickup attachment

Each double-wishbone axle independently identifies the body carrying **Actuation outer**: **Upright**, **Lower wishbone** or **Upper wishbone**. This applies to pushrod, pullrod and direct-damper arrangements. Changing the attachment preserves every static XYZ hardpoint; it does not move the pickup onto a guessed arm centerline. Enter the actual pickup position, including any bracket offset, in the same front-origin coordinates as the other inputs.

During a solve, an upright-mounted pickup follows the upright pose. A wishbone-mounted pickup follows that arm's rotation about its entered fore–aft inboard pivot line. The resulting endpoint is used before pushrod/pullrod rocker closure or direct-damper compression is calculated, and the preview and interference checker consume those same solved endpoints. The pickup therefore does not inherit upright steering rotation when it belongs to a wishbone. This does not mean that it is fixed for every rack input: the existing coupled constraints can move an arm while maintaining the prescribed wheel-center height.

Five-link suspension currently supports upright attachment only; an off-axis pickup on an independent two-ended link needs additional orientation/joint information. MacPherson uses its existing strut endpoints and does not use this attachment setting. Saved v2 designs store the optional per-axle `actuationAttachment` value. Missing values preserve the earlier upright behavior, and generated layouts explicitly start with upright attachment; these defaults are compatibility/starting behavior, not inferred real-vehicle geometry.

The rigid-body derivation, supplied-reference page comparisons, degeneracy checks, applicability limits and verification status are recorded in [ACTUATION-ATTACHMENT.md](docs/ACTUATION-ATTACHMENT.md). Attachment selection retains the existing front-origin editor adapter and axle-local storage/solver frame.

## Steering axis in the suspension preview

Use **Show steering axis** in the live preview to display the finite dashed segment between the current steering pivots. For double wishbones these are **LCA outer** and **UCA outer**. For the modeled conventional MacPherson arrangement they are **LCA outer** and **Strut top**, interpreted as the upper strut mounting/bearing center. The MacPherson steering axis can differ from the physical shaft line between **Strut bottom** and **Strut top**. The reference follows the solved points during rack, bump and heave motion. Five-link suspension has no defined two-pivot steering axis in the current schema, so no physical axis line is inferred. Degenerate or unavailable endpoints are omitted with an explanation.

The wheel preview already uses the full solved three-dimensional upright rotation: its spindle is `n_solved = R(rotationVector) n_static`, and its tire plane is perpendicular to that spindle at the solved wheel center. It does not substitute a rotation about the vehicle's vertical Z axis. The new reference makes the actual pivot line visible without changing the mathematical solver, hardpoints or collision geometry. The positive-right steer readout is a heading projected onto the level road plane; it is not the upright's rotation-axis direction or one component of its rotation vector.

When suspension travel and steering occur together, the pivot endpoints can move too. Their current connecting line is a steering-axis reference, not necessarily the single finite rotation axis of the entire displacement from the static pose. The derivation, source comparison and audit status are recorded in [STEERING-AXIS-PREVIEW.md](docs/STEERING-AXIS-PREVIEW.md).

## Appearance

Use the **Dark mode** switch in the top bar to change the full workspace, including the 3D viewer. The switch supports keyboard activation and exposes its state to assistive technology. The app starts in light mode and remembers your explicit choice in this browser on this site. Theme preferences are separate from saved vehicle designs. If browser storage is unavailable, switching still works for the current page.

## Tire tread width

Enter **Tire tread width** under the selected axle's configuration, in millimeters. Front and rear values are independent. This is the user's confirmed tread-width definition, not overall mounted section width. New designs and presets start blank because their tread widths are unknown. Clearing the input restores that unknown state. Values must be greater than zero and no greater than 10000 mm; the upper bound is an editor limit, not a physical suitability criterion.

The preview draws an ideal circular tread band centered on each solved wheel center. Its radius is half the existing vehicle-level **Wheel diameter**, interpreted as tire outer diameter. Its two edge centers are `C − aW/2` and `C + aW/2`, where `C` is the wheel center in axle-local SAE millimeters, `a` is the unit wheel axis, and `W` is the entered tread width in millimeters. The existing static wheel axis is lateral; solved upright rotation carries that axis during motion. Width and diameter alone do not determine sidewall shape, shoulder curvature, loaded deflection, section width, or rim dimensions. The concentric inner circle remains an illustrative rim cue with no assigned rim width.

When width is unknown, only the center-plane outer-diameter circle is drawn; no fixed width is substituted. **Fit all** includes the tread edges when known. Editing width preserves hardpoints and the current solved pose. Tire geometry remains outside the heave interference checker. Saved/exported v2 JSON stores the optional `axles.front/rear.tireTreadWidth`; imports of older v1/v2 designs without it receive `null`, not a measured value.

This rendering construction uses Euclidean geometry, not a tire-force or contact model from the supplied books. Edge separation, midpoint, reflection, bounds and unchanged solver output are checked in software. No measured tire or CAD profile has been used for physical validation. See [the geometric method](docs/TIRE-TREAD.md) for the assumptions and cross-checks.

## U-bar anti-roll bar inputs

Enable **U-bar anti-roll bar** under the selected axle's configuration to enter four left-side XYZ points: chassis bearing center, bar bend, lever-arm tip/drop-link joint, and suspension drop-link pickup. Inputs use the same front-origin SAE frame and millimeter units as the suspension editor; rear ARB coordinates are converted to axle-local values for v2 storage. No coordinates are generated: blank components remain `null`, and incomplete drafts can be saved/exported. Disabling the checkbox hides the inputs and preview while retaining the points. Front and rear are independent.

The preview uses the declared symmetric layout: each right-side point is `[X, -Y, Z]`. **Three complete left-side points—bar bend, lever-arm tip, and suspension pickup—draw the U-shaped centerline and both drop links.** The chassis bearing location is optional for rendering its reference marker; leaving it blank does not hide the bar and does not infer a bearing position. This does not imply that actual chassis bearings are mechanically optional.

Each valid, wholly entered XYZ point is shown immediately, along with its mirrored counterpart. A segment is drawn only when its endpoints are known: bend-to-mirrored-bend (transverse section), bend-to-tip (each arm), and tip-to-pickup (each drop link). Null coordinates remain unknown, including when other components are zero. Partial layouts show an explicit status. Preview markers use the ARB color and are larger than suspension hardpoint markers; when entered ARB geometry changes, the camera fits the visible suspension, ARB, and obstacle geometry so the ARB is not left outside the initial view.

Bearing centers are markers; for a straight transverse shaft, their X and Z should match the bends. This is a static packaging reference, including during bump/steering. It does not establish mechanical feasibility, assume which suspension body carries the pickup, or calculate ARB rotation, twist, stiffness, forces, bend radii, or clearances. The four-point input schema is unchanged and incomplete drafts remain saveable. Reference passages supporting the component names are recorded in [the reference library](docs/REFERENCES.md#u-bar-input-and-static-layout-reference).

ARB coordinates are retained when changing track, topology, actuation, or resetting suspension hardpoints; review the ARB layout after such changes. A wheel-diameter change shifts every entered ARB Z by `−Δdiameter/2`, consistently with the suspension's ground datum adjustment; blank Z values stay blank. JSON schema v2 accepts an optional `axles.front/rear.antiRollBar` object (`type`, `enabled`, `points`). Existing files without it remain compatible. ARB extensions on v1 files are rejected because that historical format did not define ARB coordinate semantics.

## Obstacle geometry

The shared vehicle-level panel accepts an axis-aligned box for engine and differential envelopes (center X/Y/Z plus length/width/height) and a cockpit trapezoid (center X/Y/Z plus depth, top width, bottom width, and height). The cockpit cross-section lies in Y–Z and is extruded along X; equal top and bottom widths form a rectangle. All dimensions are in millimeters. Coordinate centers use SAE J670 Z-down: +X forward, +Y right, +Z down. The controls start blank; no engine, differential, or cockpit dimensions are inferred.

Center inputs and the preview always use the front-axle origin. **Moves with wheelbase changes** selects whether the shape follows the front or rear datum when wheelbase changes; changing this selector preserves the shape's current position. Existing obstacle records retain their storage reference axle; the editor displays `X_front = X_stored − I_reference,rear × wheelbase`, with `I=1` for rear and `I=0` for front, and reverses that translation on input. The local interference check separately transforms the same stored envelope into the active axle's frame. Y and Z do not change because the assembly axes are parallel and have no vertical offset. Changing wheelbase moves rear-referenced envelopes with the rear datum and keeps front-referenced envelopes at their existing front-frame X. These are frame conversions, not suspension calculations.

Unchecking an envelope retains its inputs. Blank fields represent unknown draft values and can be saved, but a visible incomplete envelope is not drawn or checked. Changing wheel diameter shifts each entered obstacle center Z by `−Δdiameter/2`, matching the existing ground-datum adjustment for suspension hardpoints; dimensions do not scale when track width changes. These simple shapes stay fixed during bump and steering. Their shape and field definitions are application choices, not equations sourced from the vehicle dynamics references. Existing v2 JSON may omit the optional top-level `obstacles` collection; once present it contains engine, differential, and cockpit records. Version 1 obstacle data is rejected because v1 does not identify a coordinate frame for it.

## Heave interference check

Each new design starts with editable compression and rebound magnitudes of **25.4 mm**. Compression is upward wheel-center motion: `bump = +compression`, so `Z_solved = Z_static − compression`. Rebound is downward: `bump = −rebound`, so `Z_solved = Z_static + rebound`. The left and right wheels receive the same heave input, rack motion is zero for this check, and each axle is evaluated independently. The existing six-pose solver leaves chassis-side hardpoints anchored and moves the upright-side hardpoints while preserving each fixed link length. The outer actuation pickup follows its selected upright or wishbone attachment before the bell-crank solve pivots its rocker about the declared X axis or direct-damper length is evaluated.

The shared link OD is an editable positive value and has no invented default. It is the maximum link OD supplied by the user, so applying it to every modeled link gives a conservative packaging envelope. Suspension arms, toe links, pushrods/pullrods, and the three displayed bell-crank arms use radius `r_link = linkOD/2`. Dampers use each axle's existing damper OD with `r_damper = damperOD/2`; this covers a direct damper, a bell-crank damper, or the MacPherson strut. The current rocker is drawn as three centerline arms; those use the shared link OD until plate geometry is defined. Members are approximated as capsules: straight centerlines with a circular section and rounded ends. This is a geometry approximation, not the measured solid or joint shape.

For member–member contact, `clearance = d(segment₁, segment₂) − r₁ − r₂`, where `d` is the minimum Euclidean distance between the two finite centerline segments. For member–obstacle contact, `clearance = d(segment, solid) − r_member`; the obstacle distance is the minimum Euclidean distance from the centerline segment to the closed engine/differential box or cockpit prism. Clearance at or below zero indicates overlap of these idealized envelopes. The overlap flag is geometrically meaningful; a negative clearance is not a physical penetration-depth prediction when an axis lies inside an obstacle.

The scan samples both directions with no more than **0.25 mm** wheel-center spacing. A detected transition is bracket-refined until the travel interval is at most **0.01 mm**. The slider directly solves the selected heave position, recomputes clearance there, and highlights contacting members in the viewer. Because the sweep is sampled, a very short contact interval between samples can be missed; the slider's current-position result is not interpolated. Only complete packaging envelopes enabled by **Show in preview** are tested. Members that share a modeled pivot are excluded from member–member clearance because no bearing, ball-joint, or bracket solids are defined. Springs, tires, uprights, anti-roll bars, and chassis brackets are not included.

The geometric equations are Euclidean segment/solid distances, not equations attributed to the six vehicle-dynamics books; no measured vehicle or CAD solid has been used to validate this packaging model. Regression tests check segment and mesh distances, the SAE heave signs, link-length preservation, direct damper use, synthetic contacts, and a refined travel interval. See [the full method and limits](docs/INTERFERENCE.md). Older v1/v2 designs receive the heave defaults and a blank shared link OD when imported.

## Coordinates and parameter behavior

- **SAE J670 Z-down:** +X forward, +Y right, +Z down. Dimensions and hardpoints are millimeters. Coordinate inputs and the assembly preview use the front-axle ground datum; the rear datum is at X = −wheelbase. The solver and saved v2 design retain explicitly converted axle-local coordinates. Neither datum is the vehicle CG. See the complete [coordinate and sign contract](docs/COORDINATES.md).
- Control-arm hardpoints use **fore** and **aft** in the editor, viewer labels, and validation messages, including multi-link arms. Fore means toward +X (forward); aft means toward −X (rearward), relative to the other arm attachment. These names do not require either point to have a particular coordinate sign. Existing v1/v2 JSON identifiers retain `_front`/`_rear` for saved-design compatibility.
- Enter left-side hardpoints (negative Y). Right geometry is mirrored about Y = 0. Both solved corners are returned in the active axle's local SAE frame, then placed in the front-origin assembly preview.
- Track is wheel-center to wheel-center. Changing track scales Y coordinates, including rack length. Editing wheel-center or rack-inner Y updates the corresponding dimension; these left coordinates equal negative half the dimension.
- Increasing wheel diameter moves all points toward negative Z by the radius difference. Changing wheelbase keeps axle-local suspension and ARB geometry fixed while moving the rear assembly and its displayed X coordinates by −Δwheelbase. Front-axle geometry remains at the same X.
- Bump is positive upward (negative Z). Rack displacement is positive rightward (+Y); rack travel is total lock-to-lock stroke. Fixed toe links remain constraints when steering is disabled.
- Version 2 JSON declares axes, frame and units. Original version 1 files and saved libraries are converted automatically; original v1 browser storage is retained. Unknown coordinate metadata is rejected. Save or export to retain a converted v2 design.
- Spring/damper OD controls the rendered packaging envelopes; the damper OD is also used in heave interference checks. Tire tread width controls the optional tread-band preview and is excluded from interference checks. See the declared scopes and limitations above.

## Steering geometry tool

The default **3D geometry** view displays the same XYZ hardpoints, moving joints, wheel centers and spindle orientations returned by Python. Drag to orbit, Shift-drag to pan, scroll to zoom, or choose Isometric / Top / Front / Side. Top is an orthographic camera view of the 3D scene. **2D construction** retains the earlier XY diagram and heading-only wheel icons. Plan arm projections stay visibly separate from the spatial linkage. Ring sizes and line thicknesses are symbolic; real tire dimensions, rear-wheel height and bent-arm shapes are not inferred.

Nonhorizontal tie rods and effective steering arms are already supported by the full spatial rod-length constraint. The rack still moves in +Y and suspension ride position remains fixed. See [the governing model and inclined-geometry checks](docs/STEERING.md#nonhorizontal-linkage-and-the-3d-viewer).

Open [Steering geometry](steering.html) from the workspace navigation. This separate tool solves rack-and-pinion steering with suspension held at a defined ride position. Enter wheelbase, independent front/rear tracks, signed rack travel per steering-wheel revolution, total rack stroke, sweep range, both steering-axis points and both tie-rod joints per side, wheel-center Z, static toe and static camber. Dimensions and angles begin unknown; the explicitly selected synthetic example is not vehicle data. Left/right geometry is independent, with an explicit mirror action if appropriate.

The tool uses spatial rigid-axis rotation and constant tie-rod length to calculate left/right wheel heading, inside/outside turn angles and steering-induced toe at every 5° of steering-wheel input. Its top view and curves compare actual linkage behavior with the nominal zero-slip Ackermann target. CSV exports the sweep; versioned JSON transfers geometry; browser save retains a separate steering draft without changing suspension designs. Complete derivation, source-page evidence, numerical safeguards and validation limits are in [STEERING.md](docs/STEERING.md).

The model excludes suspension travel, compliance, backlash, tire slip and forces. Tilted-axis wheel-center movement is geometric only: no road-contact or chassis-jacking solve is imposed. A geometry result is not an optimized setup or measured-vehicle validation. Rear track controls the rear layout; it does not enter the ideal front-angle relation for unsteered rear wheels.

The steering preview also displays included tie-rod/arm angles and extended steering-arm construction lines. Plan angle α and the intersection use the steering axis at outer-ball height (Milliken Fig. 19.3); effective spatial angle β uses the perpendicular steering radius. The readouts distinguish them, report the intersection's axle-local coordinates and rear-axle offset, and provide a fit-to-intersection control. These construction lines are not a solved rolling turn center. Both angles are included in the 5° table and CSV.

The rack-offset editor sets the rack midpoint's longitudinal X relative to the front axle (+ ahead, − behind, in mm). Apply moves both existing inner-joint X coordinates equally, preserving any left/right X difference and all Y/Z inputs. If an X is unknown, the explicitly labeled Set both rack X values action initializes both at that offset. Changes are staged until Apply/Enter; Reset entry discards them. This edits neutral layout and recalculates the reference tie-rod lengths; steering stroke still acts in +Y. Offset is derived from the saved hardpoints, so existing JSON designs remain compatible.

## Architecture

- `src/coordinates.js`: SAE frame definitions, reflection and alignment signs; `src/migration.js`: versioned import and saved-library conversion.
- `src/coordinate-frames.js`: explicit position translations between axle-local storage/solver frames and front-origin editor/preview coordinates.
- `src/model.js`: versioned design schema, topology definitions, default geometry, validation, presets. This is the single source for geometry and configuration.
- `src/arb.js`: optional U-bar point definitions, nullable draft coordinates, validation, and completeness checks. ARB points stay separate from the suspension solver's hardpoints.
- `src/obstacles.js`: optional packaging-envelope dimensions, nullable drafts, axle-datum conversion, and simple box/trapezoid mesh vertices. These remain independent of the suspension solver.
- `src/interference.js` and `src/geometry-distance.js`: heave pose sampling, rigid member/damper centerlines, capsule clearances to convex obstacle solids and other segments, and contact-boundary refinement.
- `src/solver.js`: numerical rigid-body position solver, independent from DOM/rendering.
- `src/viewer.js`: interactive canvas rendering of projected 3D geometry, orbit/zoom, orthographic camera views. No CDN/WebGL dependency.
- `src/tire-geometry.js`: optional circular tread-band edge centers and exact axis-aligned bounds, independent of the suspension solver.
- `src/app.js`: editor, state transitions, local persistence, imports/exports.
- `src/theme.js`: applies the saved appearance before first paint and handles the accessible theme switch.
- `tests/*.test.mjs`: numerical, migration, physical-pose and schema regressions, run using Node 20+ with `node --test suspension-studio/tests/*.test.mjs` from the repository root.
- `tests/index.html`: browser-run version of the 40 pure model/solver checks (eight additional fixture regressions and 11 ARB input checks run in Node) for environments without Node. Open `/suspension-studio/tests/` through the local server.

The application does not execute or reinterpret the legacy dashboard, tire scripts, or Python renderer.

## Kinematic formulation

The upright has six pose unknowns: three translations and three rotation-vector components. Rodrigues' formula rotates points rigidly attached to that upright. A wishbone-mounted actuation pickup is transported separately with its selected arm, using the actual fore–aft pivot line and solved outer ball joint. Rotation parameters in the upright solve are scaled to a 300 mm characteristic length to improve conditioning.

- Double wishbone: four fixed arm-length constraints plus tie-rod length and prescribed wheel-center Z.
- Five-link: five independent fixed-length constraints plus prescribed wheel-center Z.
- MacPherson: two lower-arm constraints, tie-rod length, two perpendicular constraints keeping the chassis strut top on the rotating upright strut axis, and prescribed wheel-center Z. The strut telescopes freely.
- Bell crank: after resolving the selected outer pickup attachment, a separate scalar solve rotates a rocker around its longitudinal X axis while preserving pushrod/pullrod length. Damper compression is the change in eye-to-eye length; a direct damper uses the transported pickup, while a MacPherson strut uses its existing strut endpoints.

Newton iteration with numerical Jacobian, partial-pivot elimination, line search, and 4 mm continuation steps searches near the static assembly. A maximum constraint residual above 0.001 mm rejects the result. Heave travel controls accept magnitudes through 100 mm, the solver's stated bound; an impossible pose, singular linkage, or unreachable rocker is reported without inventing a result. The preview returns to static geometry on a failed solve.

Camber and toe outputs are **changes from the static pose**, not absolute alignment angles. The static wheel axis is assumed lateral because the input schema does not yet specify wheel/upright orientation. Camber is positive top-outward; toe is positive inward on either side. Separate SAE steer angles are positive-right wheel-plane headings projected onto the level road plane. Their sign agrees with positive yaw about +Z, but they do not impose a vertical steering axis. Forces, masses, compliance, tire contact, springs rates, damper curves, anti-roll-bar dynamics, hard stops, collision checks, motion-ratio curves, and dynamic integration are outside this first milestone. These generated hardpoints are illustrative design seeds, not optimized or validated vehicle geometry.

## CR26 provenance

The nine original front-left coordinates from `FS-CR26.py` (repository commit `7b70957b0562d037a01f8f7537b256f46f93f857`) are transformed to SAE coordinates in the CR26 preset with distances and physical geometry preserved. Front track is 1117.6 mm and rack length is 438.16 mm, derived from those coordinates. The original source script remains unchanged; the new preset applies `[X, Y, Z] = [-oldY, -oldX, -oldZ]` once at the source boundary. Front actuation, the rear axle, and the 1600 mm wheelbase are generated/assumed starting data; they are not claimed as CR26 measurements.

## Technical references

The six supplied vehicle dynamics books are cataloged in [the reference library](docs/REFERENCES.md), with review status and a workflow for documenting future equations, sign conversions and validation. Gillespie Chapter 1 and Milliken Chapter 4 support the present coordinate contract. Source PDFs remain outside the repository.

## Next development milestones

Add absolute alignment and wheel orientation; extend attachment support to explicitly oriented independent links; support arbitrary rocker axes and multi-body joints; introduce full-vehicle assembly and validated travel sweeps; then connect tire, spring, damper, mass/inertia and load models through explicit module contracts.
