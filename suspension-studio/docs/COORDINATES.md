# Coordinate and sign contract

Suspension Studio uses **SAE J670 Z-down: +X forward, +Y right, +Z down**, a right-handed orientation. Here, right and left mean the occupant's right and left when facing forward. Modern J670 allows both Z-up and Z-down; this project explicitly selects the traditional Z-down option used by Gillespie and Milliken. This is an orientation and interface contract, not a claim that every model implements the complete standard.

## Frames and units

Hardpoints are in **millimeters**. The solver uses radians internally for rotation vectors and reports alignment in degrees. Uppercase X/Y/Z in the editor are component labels; they do not identify an earth-fixed frame.

The coordinate editors and assembly preview use one **front-axle origin**: the intersection of the vehicle center plane, the nominal static front-axle station, and the static ground plane. The rear-axle origin is `[-L, 0, 0]` in this frame, where `L` is the entered wheelbase in millimeters. Thus a point directly above the nominal rear-axle station has editor X = `−L`, not zero. Both axle frames have parallel axes and the same ground height; no reference pitch or relative datum-height offset is modeled. These chassis-fixed datums do not follow the wheels during a sweep and are not CG origins.

The existing solver and version 2 saved-design format retain a separate **axle-local frame** for each axle. The editor converts at its boundary; it does not relabel local data as front-referenced data. For rear points:

```text
X_editor = X_rear_local − L
X_rear_local = X_editor + L
Y_editor = Y_rear_local
Z_editor = Z_rear_local
```

Front-axle coordinates are identical in both frames. For example, with `L = 1530 mm`, a rear hardpoint at local `[80, −250, −300]` is entered and displayed as `[-1450, −250, −300]`. The rear wheel center at local X = 0 appears at editor X = −1530 mm. Unknown ARB or obstacle coordinate components remain unknown; translation never substitutes zero for a blank.

The following generated wheel-center definitions are **axle-local**; apply the rear translation above for the editor and assembly preview:

- Left wheel center: `[0, -track/2, -wheelRadius]` in generated geometry.
- Right wheel center: `[0, +track/2, -wheelRadius]`.
- Positive physical height above the datum: `height = -Z`.
- Left-to-right reflection of a position/free vector: `[X, -Y, Z]`.
- Reflection of an axial vector (rotation, angular velocity, moment): `[-Rx, Ry, -Rz]`.

The preview places the selected axle in the front-origin assembly frame and marks both axle datums. Static and solved suspension points, ARB points, wheels and packaging envelopes use that same frame. Changing wheelbase keeps each axle's local geometry fixed and moves the rear assembly along X; displayed rear X values change by `−ΔL`. Distances within a translated assembly and all direction vectors remain unchanged. A CG-based dynamics frame will need an explicit translation and any required rotation from this assembly frame.

Obstacle centers are also entered and displayed relative to the front origin. Existing obstacle records retain their `referenceAxle` and local center coordinates for compatibility. A stored obstacle point in reference frame `r` is displayed at `X_editor = X_r − I_r,rear × L`, where `I` is 1 for rear and 0 for front. For the existing axle-local interference check it is transformed instead as `X_active = X_r + (I_active,rear − I_r,rear) × L`. Y and Z do not change. Each path applies the translation once. The **Moves with wheelbase changes** selector chooses the storage anchor; changing it converts the stored center through the front frame, preserving its current position and unknown components. On a subsequent wheelbase change, a rear-referenced stored envelope follows the rear datum; a front-referenced envelope stays at its existing front-frame X. These are coordinate transformations, not suspension calculations.

Optional engine and differential envelopes are axis-aligned boxes described by center `[X,Y,Z]` and length, width and height. The optional cockpit envelope is a trapezoid in the Y–Z cross-section, extruded along X by its depth. For cockpit height `H` and center Z `Zc`, the top and bottom are at `Zc − H/2` and `Zc + H/2`; their half-widths are `topWidth/2` and `bottomWidth/2`. This follows the project sign convention that top is toward negative Z. These are user-entered visualization and packaging envelopes, not vehicle-specific measured geometry. Complete visible shapes are used in the heave interference check.

## Inputs and outputs

| Quantity | Positive direction / definition |
| --- | --- |
| Wheel bump (jounce) | Upward wheel-center motion relative to the chassis: `Z_solved = Z_static - bump` |
| Rack displacement | Translation toward the right, +Y, for **both** rack ends |
| Steering-tool rack longitudinal offset | Midpoint X of the two neutral rack inner joints, relative to the front axle: positive ahead, negative behind; distinct from +Y steering stroke |
| Rack travel | Total lock-to-lock stroke; slider limits are ±half travel |
| Steer angle | Wheel-plane heading projected onto the level road plane, positive toward the right; its sign agrees with positive yaw about +Z |
| Toe-in | Front of wheel points toward the center plane; positive on either side |
| Camber | Top of wheel tilts outward; positive on either side |
| Damper compression | Static eye-to-eye length minus solved length; shortening is positive |
| Positive roll / pitch / yaw | Right-hand rotations: right side down / nose up / turn right |

Toe-in is a side-dependent alignment scalar, not a Z rotation component: `toeLeft = steerLeft`, `toeRight = -steerRight`. Identical steer directions therefore produce opposite toe-in signs. Rack direction does not guarantee steer direction; that depends on the linkage geometry and front/behind-axle rack placement.

The suspension solver's alignment outputs are changes from a nominal zero-camber,
zero-toe static wheel. The fixed-axis steering module instead accepts explicit static
toe and camber, reports the resulting absolute alignment, and reports toe changes
separately. Both use the rotated outward unit spindle `n` and side `s` (-1 left,
+1 right):

```text
camber = atan2(nZ, hypot(nX, nY))
steer  = atan2(-s*nX, s*nY)
toeIn  = -s*steer
```

Heading is taken from the intersection of the wheel plane with the level reference road plane. A positive-right steer readout does not impose rotation about a vertical axis and is not a component of the upright's rotation vector. Static wheel orientation is not yet an input to the suspension solver; the steering module's `staticToeDeg` and `staticCamberDeg` define it as documented in [STEERING.md](STEERING.md). The suspension six-pose solver uses a Rodrigues rotation vector; its three components must not be interpreted as sequential Euler angles. The live wheel preview applies this complete rotation to the static outward spindle `[0, side, 0]` and centers the tire plane on the solved wheel center. The bell crank is currently constrained to the longitudinal X axis.

The **Show steering axis** reference joins the solved lower/upper outer ball joints for double wishbones. For the modeled conventional MacPherson arrangement it joins the solved lower outer ball joint and chassis-fixed upper strut mounting/bearing center (`strut_top`); this line need not coincide with the physical strut shaft through `strut_bottom`. Five-link geometry does not identify a physical two-pivot axis, so no such line is inferred. The display uses the finite joint-to-joint segment in millimeters, omitting unavailable, nonfinite or coincident definitions. It adds no physical member or collision constraint. The two positions receive the same front-origin translation as all other preview points, while free directions and rotation vectors receive none. During simultaneous suspension travel and steering the endpoints can move, so their connecting line need not be the single finite rotation axis of the entire displacement from static geometry. See [the derivation and supplied-reference comparison](STEERING-AXIS-PREVIEW.md).

The optional per-axle outer actuation attachment identifies which rigid body transports `actuation_outer`. For double wishbones, upright attachment uses the solved upright transform; upper/lower wishbone attachment uses rotation about that arm's entered fore–aft inboard pivot line. The pivot line need not be parallel to a vehicle coordinate axis. Attachment selection preserves all static XYZ coordinates, including any rigid bracket offset. It does not change the front-origin editor or axle-local solver/storage conventions. The transported endpoint enters the existing rocker or direct-damper calculation before the viewer and interference checker consume the result. See [the derivation and source comparison](ACTUATION-ATTACHMENT.md).

For future force models, signed vehicle force components follow these axes: weight acts in +Z; a ground reaction on the vehicle acts in -Z on a level road. A positive tire-load magnitude is a separate scalar and must be converted explicitly. Tire slip, inclination, force-on-road versus force-on-vehicle, and tire-local axes require a declared adapter for each data source.

## Schema and migration

Saved and exported JSON retains the existing axle-local representation:

```json
{
  "version": 2,
  "coordinateSystem": "SAE_J670_Z_DOWN",
  "hardpointFrame": "AXLE_LOCAL",
  "lengthUnit": "mm"
}
```

The front-origin coordinate editor is an interface adapter, not a schema migration. Suspension hardpoints and ARB points in the rear axle's v2 JSON remain rear-local. The browser library retains the same `.v2` storage key. Existing v2 designs load with the same local values and physical assembly; their rear X values appear one wheelbase lower in the editor because the displayed reference origin has changed. JSON imports must follow their declared `AXLE_LOCAL` metadata: do not paste front-origin rear X values directly into an axle-local JSON file. Front-origin values entered through the editor are converted before saving or solving.

Schema v2 may omit the optional top-level `obstacles` field for older designs. If present, it stores optional static engine, differential, and cockpit envelopes in SAE J670 Z-down axle-local millimeters. Incomplete centers/dimensions are `null` until entered; v1 obstacle fields are rejected because their coordinate meaning is unspecified.

Schema v2 also accepts optional `axles.front/rear.actuationAttachment`: `upright`, `lower-wishbone` or `upper-wishbone`. The choice is independent for each axle. Missing values preserve the previous upright-attached behavior, and generated layouts explicitly use upright until changed. Double-wishbone suspension supports all three choices; multi-link supports upright only. MacPherson uses its strut endpoints and does not use attachment metadata to transport a separate pickup. This metadata changes body ownership during motion, not the coordinate frame or static point values.

New designs also store top-level `heaveTravel: { "compression": 25.4, "rebound": 25.4 }` in millimeters. These are positive editable magnitudes for wheel motion; the solver receives `bump = +compression` for upward compression toward −Z, and `bump = −rebound` for downward rebound toward +Z. The current pose solver supports magnitudes through 100 mm. Older v1/v2 designs that omit the extension receive these explicit defaults on import; malformed supplied values are rejected.

New designs also store `interference: { "linkOD": null }`. The null value means the shared maximum suspension-link OD is not yet known; the checker remains unavailable until the user enters a positive millimeter value. Older v1/v2 designs receive this blank extension on import. For the geometric check, each axle is swept with equal left/right wheel-center travel and zero rack motion. Chassis-side hardpoints remain fixed; solver outputs move upright-side points, the pickup on its selected body, and solved rocker/damper points. The user-entered positive magnitudes keep the SAE direction contract: compression uses positive bump and rebound negative bump.

Modeled member radius is `linkOD/2` for suspension and actuation links; damper radius is existing per-axle `damperOD/2`. The software represents each member as a straight capsule around the centerline between its two points. For two capsules, `clearance = d(segment₁, segment₂) − r₁ − r₂`. For a capsule and convex obstacle, `clearance = d(segment, solid) − r_member`. Here `d` is a Euclidean minimum distance in axle-local millimeters. Clearance `≤ 0` indicates overlap of the idealized shapes. A negative value does not report true penetration depth when a centerline lies inside a solid. Shared-pivot members are excluded from pairwise member checks because joint and bracket shapes are not defined. The full travel scan samples at no more than 0.25 mm and refines sampled contact transitions to a 0.01 mm wheel-travel bracket; it cannot guarantee detection of a contact interval narrower than its scan spacing. Springs, tires, uprights, ARBs and chassis brackets are outside this check. The full method and software evidence are in [INTERFERENCE.md](INTERFERENCE.md).

These metadata are required. Unknown coordinate systems, frames, units, or schema versions are rejected instead of guessed. Importing an existing v2 file never converts its coordinates again.

Original Studio v1 JSON is recognized by version 1 with no coordinate metadata. It used lateral-left X, longitudinal-rearward Y (fore pivot Y < aft pivot Y), and Z-up. Every hardpoint is converted once:

```text
[X_SAE, Y_SAE, Z_SAE] = [-Y_v1, -X_v1, -Z_v1]
```

This proper rotation has determinant +1 and preserves distances and physical geometry. The original input object is unchanged. All point attachments, including actuator geometry, are converted. Legacy scripts remain unchanged; the CR26 preset applies this transform at its input boundary.

Browser storage is read from `suspension-studio.designs.v2`, falling back to `.v1` only when v2 is absent. Compatible v1 designs are converted in memory and written to v2 on Save. The v1 key remains untouched. Unreadable array entries are retained when saving the library; if the library itself cannot be read, Save is disabled and JSON export remains available. Local storage is scoped to the same browser and origin (host/port).

## Evidence and limits

- [SAE J670_202206 official record](https://saemobilus.sae.org/standards/j670_202206-vehicle-dynamics-terminology): current referenced edition, reaffirmed 2022; revised 2008.
- [SAE J670 JAN2008 public sample](https://www.normsplash.com/Samples/SAE/911431649/SAE-J-670-2008-en.pdf): rationale, introduction, and Section 3 / Figure 1 distinguish Z-up and Z-down orientations.
- Gillespie, *Fundamentals of Vehicle Dynamics*, Chapter 1, printed pp. 8–10 (supplied PDF pages 24–26), especially Figure 1.4. These pages were visually inspected.
- Milliken & Milliken, *Race Car Vehicle Dynamics*, Chapter 4, Sections 4.1–4.3, printed pp. 114–120 (supplied two-page scan PDF pages 76–79). The text was reviewed; origin choices vary with the model, so Studio declares its own local datum explicitly.

Regression checks cover the coordinate transform, migration, left/right steering and camber signs, link-length invariance, and physical poses captured before conversion for seven topology/actuation combinations. These establish software consistency, not experimental validation of a real vehicle.

The front-origin editor translation follows the Euclidean identity `P_front = P_local + O_axle`, with `O_front = [0,0,0]` and `O_rear = [−L,0,0]`. For any two positions in the same translated frame, `(P₂ + O) − (P₁ + O) = P₂ − P₁`; therefore link lengths, angles and the existing local solver constraints are unchanged. This is a project frame-adapter derivation, not a new vehicle-dynamics equation attributed to a reference book. Relevant verification checks are front identity, rear forward/inverse conversion, distance preservation, unchanged unknown components, import/export compatibility, and consistent placement of solved points, ARBs and obstacles. A mathematical translation identity does not establish surveyed-vehicle accuracy.

Verification on 2026-10-27: all 201 Node tests passed, including 14 additional frame/editor/viewer checks. Browser checks covered front X = 0 and rear X = −1530 at a 1530 mm wheelbase, rear decimal entry, a wheelbase change to 1600 mm, nullable ARB inputs, obstacle anchor changes that preserve front-frame position, invalid-entry retention, saved-design reload, an older v2 design, and a solved rear bump pose. Downloaded JSON retained `AXLE_LOCAL` metadata and the expected local hardpoint, ARB and obstacle coordinates. These checks establish software and coordinate consistency for the checked cases, not measured suspension accuracy.
