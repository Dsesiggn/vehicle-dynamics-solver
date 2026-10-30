# Python rigid suspension corner model

## Derivation recorded before implementation

This model solves an ideal rigid upright located by the entered suspension links.
It uses SAE J670 Z-down (+X forward, +Y right, +Z down), axle-local millimeters,
and right-hand rotation angles in radians internally. The front-origin editor
adapter remains separate. Positive wheel bump is upward relative to the chassis.
This corner calculation does not itself prescribe body pitch/roll, tire contact,
load transfer, or a force-based roll center.

Let C0 be the static wheel center, t its displacement, R a proper orthogonal
rotation, and P0 any upright-carried static point. Rigidity requires

```
C = C0 + t
P = C + R(P0 - C0)
R^T R = I, det(R) = +1
```

There are six independent pose coordinates: three translations and three
rotations. A spherical-ended link between a chassis point A and an upright
point P preserves its entered static length L:

```
f = ||P - A|| - L = 0,       L = ||P0 - A0||
A_tie = A_tie,0 + [0, rack, 0]
```

A double wishbone supplies four such equations (both legs of each arm) and a
tie rod supplies the fifth. A modeled five-link suspension supplies four
independent links and the tie rod. The sixth equation for the initial corner
interface is `C_z - C0_z + bump = 0`. Its sign is fixed by the project contract;
it is not an approximation to arbitrary body roll or pitch.

The conventional MacPherson model instead supplies two lower-arm leg equations,
one tie equation and two telescopic strut-line equations. Let B0 be the static
upright-carried strut bottom, T its fixed top mount and N10/N20 orthonormal
vectors perpendicular to T-B0. Then

```
B = C + R(B0 - C0),       Nj = R Nj0
fj = (T-B) dot Nj = 0,    j = 1, 2
```

The shaft can telescope and turn; it is not assigned a fixed length. This
constraint permits the MacPherson steering axis through the lower ball joint
and top mount to differ from the shaft line. The perpendicular basis is formed
with the coordinate direction least aligned with the strut, so a valid strut
parallel to X is not accidentally converted to two zero constraints.

## Exact local Jacobian and instantaneous motion

At the current pose define an infinitesimal wheel-center translation dc and a
spatial infinitesimal rotation dtheta, updating R by left multiplication with
`exp([dtheta]x)`. For r=P-C,

```
dP = dc + dtheta cross r
e = (P-A)/||P-A||
df_link = e dot dc + (r cross e) dot dtheta - e dot dA
```

Thus the link Jacobian row is `[e, r cross e]`. Its translation entries are
dimensionless and its rotation entries have units mm/rad. For a strut normal N,
differentiate both the bottom and the rotating normal:

```
df_strut = -N dot dc + [N cross (T-C)] dot dtheta
```

The prescribed wheel-center row is `[0,0,1,0,0,0]`. Setting a characteristic
length s converts the rotation increments to distance-like coordinates
`u=[dc, s*dtheta]`; the last three Jacobian columns are divided by s. The chosen
scaling changes conditioning, not the physical equations. Newton correction
solves `J_scaled du = -f`, updates C and R, and accepts only a residual-reducing
step. A singular-value check rejects rank-deficient or numerically unresolved
constraint sets even when the initial residual happens to be zero.

For a motion parameter q, the same differentiated constraints give
`J [dC/dq, omega] = -partial(f)/partial(q)`. A unit upward wheel-bump rate makes
the sixth right-hand entry -1. A rack rate contributes `e_y * rack_rate` to the
tie equation's right-hand side. This returns the upright twist without
subtracting nearly identical poses. Velocity of an upright point is
`dP/dq = dC/dq + omega cross (P-C)`. These rates describe the selected kinematic
path. They do not alone identify a unique vehicle roll center; that requires a
declared instantaneous-center definition and wheel/road constraint.

## Wheel orientation and actuation

An explicitly entered static camber gamma and toe-in tau define the static
outward unit spindle for side s=-1 left/+1 right. Let heading delta=-s*tau:

```
n0 = [-s*sin(delta)*cos(gamma), s*cos(delta)*cos(gamma), sin(gamma)]
n = R n0
camber = atan2(n_z, hypot(n_x,n_y))
heading = atan2(-s*n_x, s*n_y)
toeIn = -s*heading
```

Positive camber is outward top tilt; positive toe is inward on either side. If
static alignment is absent, outputs are explicitly marked relative to a
nominal zero-alignment spindle, preserving the earlier model's meaning without
claiming measured absolute alignment.

An upright-attached actuator pickup follows P above. A selected wishbone
attachment rotates about its actual entered fore/aft pivot line, using the
signed ball-joint arc and independent reconstruction/rigidity checks documented
in [ACTUATION-ATTACHMENT.md](ACTUATION-ATTACHMENT.md). Direct and strut compression
are static length minus current length, in mm.

For a separate bell crank, the pivot alone does not define its axis. An explicit
nonzero axis direction k through the entered pivot O is required for new
backend actuator results. Missing axis data makes the actuator unavailable;
the independently constrained upright pose can still be reported. For static
rod point B0, pickup P and static rod length l:

```
B(theta) = O + R(k,theta)(B0-O)
||B(theta)-P|| = l
```

Decomposing B0-O into parallel and perpendicular parts reduces squared closure
to `a*cos(theta)+b*sin(theta)=c`. The branch continuous from theta=0 is followed
through the same pose continuation. Tangency, no real roots, an unresolved
neutral lever, excessive angular change or failed length closure is reported,
not replaced by a guessed longitudinal axis or a different assembly branch.
The damper's rocker endpoint receives the same rotation. This is geometric
compression; no force, spring rate, damping or compliance is inferred.

Instantaneous motion ratio is obtained by differentiating these same rigid
constraints, rather than dividing total compression by total travel. For an
arm-carried pickup, with pivot-axis unit k, ball radial vector rB and solved
ball velocity vB, arm rate is `(k cross rB) dot vB / ||k cross rB||²`. Its
reconstructed ball velocity must agree independently; pickup velocity is
`vP = arm_rate * k cross (P-F)`. For the rocker let d=B-P and
`wB=k cross (B-O)`. Constant rod length gives
`theta_rate = d dot vP / (d dot wB)`. A small/unresolved denominator identifies
a toggle and makes the derivative unavailable. The damper endpoint velocity
is `vD=theta_rate*k cross (D-O)`. With fixed damper top T, compression rate is
`((T-D)/||T-D||) dot vD`. A direct damper uses vP in place of vD. Applying a unit
upward wheel-bump rate yields the user's selected MR `dc/db` in mm/mm. The
same equations yield mm/deg when the caller supplies rates per degree of a
declared pitch/roll rig motion; those units must not be relabeled mm/mm.

## Numerical and physical scope

The implementation uses finite proper rotations, exact local Jacobians,
continuation from the entered assembly, scaled singular-value diagnostics and
explicit residual limits. The length scale is 300 mm; target closure is 1e-10 mm,
accepted closure at most 1e-6 mm; scaled condition numbers above 1e10 fail. The
neutral assembly is checked even if its residual is already zero. Travel is
continued in steps no larger than 2 mm in bump or rack; the supported finite
rotation is at most 90 degrees. These are numerical/implementation bounds.
Continuation and rank checks sample a path: they do not analytically exclude a
narrow singular interval between samples or prove global uniqueness among all
possible assemblies. The neutral rocker derivative selects its local branch,
and each sampled pose checks reachability/toggle margin. An actuator failure
anywhere on that sampled path makes its final result unavailable even if a
later sample could close again. No inference of an assembly beyond a detected
singularity is justified. Numerical thresholds are algorithm resolution limits,
not manufacturing tolerances. Solver failure leaves unavailable outputs with
a reason. Input dictionaries are not mutated. Left/right reflection acts on
positions as `[X,-Y,Z]`, on axial vectors as `[-Rx,Ry,-Rz]`; global rack motion
remains +Y on both corners.

Assumptions are rigid members and upright, ideal stated joints, fixed chassis
hardpoints, no joint clearance, compliance, tire deformation or mechanical
stops. A detached/degenerate constraint or alternate linkage architecture is
outside the model. A successful residual check proves only satisfaction of the
implemented equations near the selected assembly; it is not experimental
validation or proof of physical suitability for a real car.

## Supplied-reference evidence and comparison

The following passages were visually verified in the earlier focused reviews
recorded in [ACTUATION-ATTACHMENT.md](ACTUATION-ATTACHMENT.md) and
[STEERING-AXIS-PREVIEW.md](STEERING-AXIS-PREVIEW.md). Those reviews inspected
complete pages; this document does not claim a new page inspection. The
finite-pose and Jacobian equations here are an explicit project derivation.

| Reference | Evidence used; limits |
| --- | --- |
| Milliken & Milliken, Ch.17 §17.1, printed pp.608–609 / supplied PDF p.324, Figs.17.1–17.2 | A-arm represented by two fixed-length links at the outer ball joint; suspension path and steering distinguished. Supports the five link closures, not a particular numerical algorithm. |
| Milliken, Ch.17 §17.5, printed pp.630–635 / PDF pp.335–337, Figs.17.20–17.22 | Actual inner-pivot arm axes; conventional strut top lies on steering axis while shaft need not. Supports real pivot geometry and telescopic line constraints. |
| Gillespie, Ch.8, printed pp.279–280 / PDF pp.295–296, Fig.8.4 | Outer arm points follow pivot arcs; motion-derived centers distinguished from anti-dive force constructions. Supports rigid transport and prohibits relabeling a motion-derived point as a force center. |
| Gillespie, Ch.7, printed pp.243–244 / PDF pp.259–260 | Conventional upper-mounted telescopic strut and lower links. Compatible mechanism description; does not establish a shaft/steering-axis identity. |
| Staniforth, Ch.3, printed pp.78–80 / PDF pp.68–70; Smith Ch.6 pp.65–66 / PDF pp.64–65; Seward Ch.4 §4.1 pp.90–91 / PDF pp.101–102 | Pickup attachment can be upright or wishbone and actuation type alone does not determine body ownership. |

No reviewed passage contradicts these ideal rigid-joint constraints within
their stated architecture. Simplified planar constructions and force-based
anti-dive diagrams are different models, not substitutes for the spatial
closure. The source figures identify points/axes; all algebra is applied to
already converted SAE project coordinates. No source diagram is silently
relabeled. The [Modern Robotics rotation derivation](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-2-3-exponential-coordinates-of-rotation-part-2-of-2/)
provides an independent rotation reference already checked in the prior review.

## Executed numerical verification (2026-10-29)

The 18 focused Python corner-model tests passed with no skips in 2.95 seconds,
including the Node/browser comparison. They check:

- Independently expanded finite rotation matrices about [3/13,4/13,-12/13],
  at positive/negative 12 degrees for double wishbone and MacPherson, both
  corners, with rack and bump calculated independently from the prescribed pose.
- Fixed link lengths, upright point distances, proper rotation determinant and
  orthogonality; reflection and common-translation invariance; static alignment
  signs; preserved inputs and finite JSON output.
- Analytic Jacobians against central differences, all three topologies,
  including the rotating strut-normal terms. Maximum permitted comparison error
  is 8e-8 in the corresponding column units. A strut parallel to X retains two
  unit perpendicular constraints instead of disappearing numerically.
- Implicit wheel/upright velocity against successively halved pose differences
  (0.1, 0.05, 0.025 mm); rack differentiation on both sides; a locked rack limit.
- A separately prescribed 0.3-rad rocker rotation about [0.6,0,0.8], independently
  predicting both rod and damper endpoints. Direct, strut, pushrod and pullrod
  compression derivatives, and all supported pickup body attachments, agree
  with central differences at 0.02/0.01 mm steps within 2e-7 mm/mm.
- Singular constraints at zero residual, incomplete alignment, invalid axis,
  absent rocker axis, and an inconsistent neutral optional road constraint are
  rejected or explicitly made unavailable. Missing actuator information does
  not destroy an independently resolved upright pose.
- Existing browser pose/actuator parity for 18 corner cases, with the legacy
  longitudinal rocker axis supplied explicitly. Tolerances are 1e-4 mm for
  points/compression and 1e-6 rad for rotation; this is regression evidence,
  separate from the independent constructions.

An additional 246-corner audit sampled 41 bump positions from -25.4 to +25.4 mm
for all three topologies and both sides, at +10 mm rack, with the explicit
rocker direction [0.8,0.2,0.4]. It took 4.02 seconds on the development machine
including sampled derivative cross-checks. Maximum accepted constraint error
was 1.71e-13 mm; determinant error 2.45e-15; orthogonality error 2.95e-15. The
maximum sampled analytic MR discrepancy against a +/-0.005 mm central
difference was 1.28e-8 mm/mm. These are observed floating-point errors for
synthetic fixtures, not claimed hardpoint accuracy or service-time guarantees.

The circular-centerline road helper has an independently differentiated
Jacobian and neutral-compatibility test. Its existence does not select that
model for vehicle motion; the caller must establish the declared tire/road
contract separately.

These are numerical/software checks, not measured-vehicle validation.
Real-car validation still needs surveyed geometry, actual joint/body ownership,
initial wheel orientation and measured wheel/damper travel under matching
imposed constraints. The rigid constraints and their derivatives follow the
stated ideal model; 100% physical accuracy or global numerical reliability for
arbitrary geometry is not established.
