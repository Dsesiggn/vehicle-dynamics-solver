# Coordinate and sign contract

Suspension Studio uses **SAE J670 Z-down: +X forward, +Y right, +Z down**, a right-handed orientation. Here, right and left mean the occupant's right and left when facing forward. Modern J670 allows both Z-up and Z-down; this project explicitly selects the traditional Z-down option used by Gillespie and Milliken. This is an orientation and interface contract, not a claim that every model implements the complete standard.

## Frames and units

Hardpoints are in **millimeters**. The solver uses radians internally for rotation vectors and reports alignment in degrees. Uppercase X/Y/Z in the editor are component labels; they do not identify an earth-fixed frame.

Each axle has a chassis-fixed design frame. Its origin is the intersection of the vehicle center plane, the nominal static axle station, and the static ground plane. Axes are parallel to the vehicle's reference axes. This datum remains fixed during a suspension sweep; it does not follow the wheel or road. It is not a CG origin. The current app has no body mass or CG model.

- Left wheel center: `[0, -track/2, -wheelRadius]` in generated geometry.
- Right wheel center: `[0, +track/2, -wheelRadius]`.
- Positive physical height above the datum: `height = -Z`.
- Left-to-right reflection of a position/free vector: `[X, -Y, Z]`.
- Reflection of an axial vector (rotation, angular velocity, moment): `[-Rx, Ry, -Rz]`.

A future vehicle assembly anchored at the front datum will place the rear datum at `[-wheelbase, 0, 0]`, assuming no reference pitch or relative datum height. A CG-based dynamics frame will need an explicit translation and any required rotation from that assembly frame. Wheelbase is currently stored but the viewer shows one axle at a time.

## Inputs and outputs

| Quantity | Positive direction / definition |
| --- | --- |
| Wheel bump (jounce) | Upward wheel-center motion relative to the chassis: `Z_solved = Z_static - bump` |
| Rack displacement | Translation toward the right, +Y, for **both** rack ends |
| Rack travel | Total lock-to-lock stroke; slider limits are ±half travel |
| Steer angle | Wheel-plane heading toward the right, positive about +Z |
| Toe-in | Front of wheel points toward the center plane; positive on either side |
| Camber | Top of wheel tilts outward; positive on either side |
| Damper compression | Static eye-to-eye length minus solved length; shortening is positive |
| Positive roll / pitch / yaw | Right-hand rotations: right side down / nose up / turn right |

Toe-in is a side-dependent alignment scalar, not a Z rotation component: `toeLeft = steerLeft`, `toeRight = -steerRight`. Identical steer directions therefore produce opposite toe-in signs. Rack direction does not guarantee steer direction; that depends on the linkage geometry and front/behind-axle rack placement.

All current alignment outputs are changes from a nominal zero-camber, zero-toe static wheel. Given the rotated outward unit spindle `n` and side `s` (-1 left, +1 right):

```text
camber = atan2(nZ, hypot(nX, nY))
steer  = atan2(-s*nX, s*nY)
toeIn  = -s*steer
```

Heading is taken from the intersection of the wheel plane with the level reference road plane. Wheel orientation at static is not yet an input. The six-pose solver uses a Rodrigues rotation vector; its three components must not be interpreted as sequential Euler angles. The bell crank is currently constrained to the longitudinal X axis.

For future force models, signed vehicle force components follow these axes: weight acts in +Z; a ground reaction on the vehicle acts in -Z on a level road. A positive tire-load magnitude is a separate scalar and must be converted explicitly. Tire slip, inclination, force-on-road versus force-on-vehicle, and tire-local axes require a declared adapter for each data source.

## Schema and migration

New JSON includes:

```json
{
  "version": 2,
  "coordinateSystem": "SAE_J670_Z_DOWN",
  "hardpointFrame": "AXLE_LOCAL",
  "lengthUnit": "mm"
}
```

These metadata are required. Unknown coordinate systems, frames, units, or schema versions are rejected instead of guessed. Importing an existing v2 file never converts its coordinates again.

Original Studio v1 JSON is recognized by version 1 with no coordinate metadata. It used lateral-left X, longitudinal-rearward Y (front pivot Y < rear pivot Y), and Z-up. Every hardpoint is converted once:

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
