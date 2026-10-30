# Prescribed-travel kinematics dashboard

## Boundary conditions agreed with the user

On 2026-10-29 the user selected prescribed wheel-center travel, explicit missing
alignment/rocker-axis/reference inputs, damper compression divided by wheel bump,
and an ideal rigid circular tire contact construction. These are modeling choices,
not measurements of CR26. The underlying engineering references and differences
are reviewed in KINEMATICS-REFERENCE-REVIEW.md and KINEMATICS-CHANNELS.md.

The chassis is fixed. For each corner's **static** front-origin station `(X,Y)`,
the commanded upward bump in millimeters is

`b = h − (X − Xr) tan(theta) + (Y − Yr) tan(phi)`.

The three inputs are swept separately. `h` is common wheel bump; `theta` is a
pitch travel angle and `phi` a roll travel angle. Trigonometric arguments are
radians internally; inputs/plots use degrees. Positive pitch commands front
rebound and rear bump about `Xr`; positive roll commands right bump and left
rebound about `Yr`. This follows the existing nose-up/right-side-down signs as
**body-equivalent rig commands**. It is the exact slope definition of an imposed
height plane at the nominal wheel stations, not a small-angle approximation.
No actual chassis rotation, CG position, flat-road constraint, or force balance
is implied. Solved wheel X/Y remain free under suspension constraints. Reference
Z is deliberately not requested: it does not enter this rig prescription.

`db/dh = 1`; `db/dtheta_deg = −(X−Xr) sec²(theta) pi/180`;
`db/dphi_deg = (Y−Yr) sec²(phi) pi/180`. Units are mm/mm or mm/deg.
Nominal rear stations receive the existing `−wheelbase` translation once.
Reference X/Y are entered in the shared front-origin frame. Unsupported travel
outside ±100 mm is a failed corner, not a clipped command or fabricated curve.

## Pose, orientation and derivatives

See KINEMATICS-SOLVER.md for the six-coordinate rigid-pose closure, analytic
Jacobian, branch continuation, wishbone pickup transport and arbitrary rocker
axis. Hardpoints use SAE +X forward, +Y right, +Z down and axle-local mm.
Wheel camber and toe are **chassis-relative**, not ground camber. Static alignment
must be entered before orientation-dependent graphs are returned. For mirrored
geometry the entered per-axle scalar alignment applies to both sides; spindle
vectors are reflected with the documented side convention. Unknown alignment
may be visualized using a clearly labeled nominal spindle, never reported as
a calculated physical alignment channel.

For outward unit spindle `n` and side `s = −1` left, `+1` right:
`gamma = atan2(nz, hypot(nx,ny))`, `delta = atan2(−s nx,s ny)`,
`toe = −s delta`. Positive camber is top outward; positive toe is inward.
With spatial angular velocity `omega`, `n' = omega × n`.
Differentiating the unit-vector formulas gives
`gamma' = nz'/sqrt(1−nz²)` and
`delta' = (nx ny' − ny nx')/(nx²+ny²)` in rad per independent-variable unit.
Multiply by `180/pi` for degree outputs. A degenerate projected spindle has no
defined heading; derivatives are unavailable. These formulas follow directly
from vector rotation and differentiation, not an Euler-angle approximation.

Let damper length be `ell = ||D2−D1||`, shortening `c = ell0−ell`, and unit
damper direction `u = (D2−D1)/ell`. Then
`c' = −u·(D2'−D1')`. The requested instantaneous motion ratio is `dc/db`,
not `c/b`. For mode coordinate `q`, `dc/dq` is a separate sensitivity in mm/mm
or mm/deg. A rocker/link singularity makes actuator outputs unavailable while
independent suspension pose outputs can remain valid. Zero motion ratio is a
valid derivative; a zero denominator is never silently replaced by zero.

The entered tire diameter defines the radius of the agreed ideal center-plane
circle. Its lowest support point and the spatial instantaneous-axis center
construction are derived in KINEMATICS-REFERENCE-REVIEW.md. During rig travel,
the support points need not share a horizontal ground plane. Reported center
height is relative to the mean Z of the appropriate pair of support points;
chassis/front-origin Z is reported separately. This is not an equilibrium
force-based roll center or a deformable tire/contact-patch model.

## Presentation and scope

Each channel carries a name, sign definition, units, axle/side, and frame.
All graphs share the actual sweep coordinate rather than an arbitrary step
number. Missing inputs, singular construction and unsolved positions are null
with reasons and gaps. CSV exports preserve those gaps and units. Changing any
design input invalidates old responses and preview positions; request-generation
checks reject out-of-order results. Python is the source of dashboard results.

The implemented optional anti-effects and straight transverse U-bar models have
separate derivations in KINEMATICS-ANTI-EFFECTS.md and KINEMATICS-ARB.md. They
require explicit force-reference/torque-path or geometry/carrier inputs. Joint
forces, compliance and five-link virtual steering-axis outputs remain unavailable.
This build is not an exhaustive/current OptimumKinematics equivalent. The
screenshots are visual requirements; their curves are not numerical reference
data for validating this solver.

## Verification evidence

Software and numerical checks are recorded after execution below. Passing those
checks is not experimental validation. Vehicle-level validation still requires
measured geometry/alignments and independent reference or rig measurements over
the requested travel range.

### Executed integration checks

The complete local suites passed: **129 Python tests and 257 Node tests, no
skips**. This includes independent prescribed finite-rotation fixtures, rigid
link/upright invariants, all three topologies, symmetry and translation,
analytical Jacobians/tangents versus successively halved central differences,
known planar roll-center examples, signed anti-effect limiting cases, and
independently prescribed ARB arm rotation. Full-solver ARB tests additionally
exercise all four pickup carriers, drop-link/lever lengths, path continuation,
mirrored heave twist and analytical sensitivities in heave/roll/steering. The
actual Python response is passed through the browser's boundary validator.
Malformed optional ARB data returns a client error, not a server exception.

In the local browser, explicit synthetic fixtures exercised 41-sample heave,
pitch, roll and steering sweeps, reference requirements, per-axle alignment and
rocker inputs, selected-pose sliders, graph-family/axle filters, roll-center
cross-plots, explicit ARB motion without a bearing marker, missing-channel
reasons, dark appearance and a narrow viewport without horizontal overflow.
The exported roll CSV contained 41 samples, 360 channels, units, unrounded
numbers, blank unavailable values and the complete design/sweep snapshots.
Invalid physical entries remained pending across axle switches and blocked
save/run until corrected or discarded. An unfinished hardpoint edit cleared
old results and disabled export; restoring the point restored readiness.
No console errors were observed in these checks. This is coverage of the
checked browser, not a claim that every browser/version has been tested.

The formula and source comparisons establish the stated model/definition
contract. The tests establish agreement with the independent constructions and
numerical tolerances for the tested cases. **Physical accuracy is not 100%
certain:** no measured CR26 motion/force data or matched OptimumKinematics
numerical export was supplied. A real-vehicle correlation must use surveyed
hardpoints, bearing axes, static alignment and measured trajectories throughout
the intended range. Compliance, tire deflection and mechanical stops require
additional models and measurements.
