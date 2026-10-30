# Kinematics dashboard: channel coverage and comparison evidence

## Scope of the comparison

This is a requirements and definition review, not a claim that Studio reproduces
OptimumKinematics. OptimumG's current [product page](https://optimumg.com/product/optimumkinematics/)
advertises more than 400 channels, including derivatives in heave, pitch, roll and
steering. It does not enumerate a complete versioned channel registry. The official
[documentation page](https://optimumg.com/learn/optimumkinematics-documentation/)
links a 101-page [manual](https://optimumg.com/wp-content/uploads/2019/08/OptimumKinematics_Documentation.pdf)
under a 2019 upload path; that path alone is not evidence of software release or
document revision. No current exhaustive channel-equivalence claim is justified.

The manual was downloaded on 2026-10-29. Complete pages 31, 52, 62, 64, 80,
84-89 and 91-99 were visually inspected; their page numbers match the PDF pages.
The PDF, text extraction and rendered images remain outside this repository.
User screenshots are examples of desired analysis, not numerical validation data.

## Verified definition differences

The manual defines positive camber outward (p.88), positive toe outward (p.89),
spring motion ratio as wheel/spring displacement, and ARB ratio as roll/ARB angle
(p.89). Studio toe is inward-positive. Its existing suspension solver instead
reports compression, not a declared motion ratio.

The manual's kinematic centers use wheel perturbations including steering-link
constraints (pp.62,95). Its roll center intersects three planes: each corner's
instantaneous axis/contact point plane, and the axle's vertical contact-point
plane (p.97). Front/side instant centers and swing-arm measures appear on pp.93-94;
pitch and roll axes on pp.96-98. Alignment, steering geometry, anti-effects and
Ackermann are covered on pp.84-99; point positions and spring/damper displacement
on p.52. Positive heave moves the body along positive vertical (p.80).

These definitions require explicit adapters; a screenshot's axis label is not a
safe substitute. The screenshots' heave gain units `in/deg`, angular-mode motion
ratio units `in/deg`, and the older dashboard's percentage title with a degree
axis do not establish unique formulas. The manual does not resolve all of those
channel-specific meanings.

## Initial channel-family requirements

This table is a project design proposal based on the requested plots and the
available inputs. It is not a transcription of OptimumG's channel registry and
does not assert that these calculations have been implemented or validated.
All channels must identify frame, units, reference pose, side and axle.
The implemented scope and differences are recorded below the table.

| Family | Proposed quantities | Inputs or model decisions that must be resolved |
| --- | --- | --- |
| Motion | Heave mm, pitch/roll deg; wheel travel per corner; rack mm; path progress | Body-pose datum, exact positive directions, ground/contact constraint and rotation order for combined motions |
| Wheel alignment | Camber, toe-in and steer heading in deg; change from reference; separate body-relative and ground-relative values | Static spindle orientation per axle/corner, or an explicitly selected zero-alignment reference; upright rotation and body attitude |
| Alignment sensitivity | Camber/toe derivative versus wheel travel in deg/mm; versus pitch/roll in deg/deg | Differentiation variable must be explicit; no hidden reciprocal of a plotted gain |
| Actuation | Damper eye-to-eye length and compression mm; rocker rotation deg; spring displacement if spring shares damper axis | Rigid carrier of each pickup, actual rocker bearing axis, complete rod/rocker/damper geometry; separate spring model if not coaxial |
| Local motion ratio | `dc/db` and separately `db/dc`, both mm/mm | `b` is upward wheel-center travel relative to body; `c` is positive damper shortening; denominator-zero state is unavailable, never zero-filled |
| Vehicle-mode actuation sensitivity | `dc/dh` mm/mm; `dc/dtheta` and `dc/dphi` mm/deg | `h`, `theta`, `phi` must refer to the declared body-motion coordinates; these are different quantities from `db/dc` |
| Roll-center migration | Front/rear coordinates mm; height above road; Y-versus-height path; changes from static; roll-axis inclination | Declared kinematic construction, contact representation, full corner constraints, singular-plane handling and body/road frames |
| Instantaneous geometry | Per-corner instantaneous axis; front/side plane intersection coordinates; swing-arm lengths and angles | Differential rigid-body motion; explicit treatment of nonzero screw pitch and parallel/infinite intersections |
| Pitch-center migration | Left/right pitch-center coordinates and connecting pitch axis | Complete front/rear vehicle geometry and a declared spatial construction |
| Steering geometry | Caster, kingpin inclination, mechanical trail, scrub radius; camber versus steer | Physical or virtual steering-axis definition per topology, road plane and contact point; multi-link needs a virtual-axis method |
| Differential steer | Inner/outer wheel headings, Ackermann angle difference, ideal steer relationship, explicitly named percentage conventions | Wheelbase/track definitions, straight-ahead limit and treatment of static toe; existing standalone tool's percentage must retain its existing meaning |
| Dimensions and trajectories | Wheel-center XYZ, contact-point XYZ, track and wheelbase changes, individual hardpoint trajectories | Contact geometry; axle-local versus front-reference conversions applied exactly once |
| Anti-effects | Front anti-dive/rear anti-lift under braking, rear anti-squat/front anti-lift under drive | CG height, force split, brake location, drive torque reactions and side-view force construction; these are not determined by hardpoints alone |
| Anti-roll bar | Left/right arm rotation, relative twist, drop-link displacement and explicitly named roll sensitivities | Actual pivot axis/bearing support, pickup body, lever/drop-link closure and twist definition; three visible points alone do not define the full model |
| Force reactions | Joint forces/moments and resultants | Separate equilibrium/force-input model; kinematics alone cannot provide these |

## First-principles checks on ratio labels

Let `b` and `c` both be measured in mm, with signs as above. A differentiable
geometry map `c(b)` gives a dimensionless local displacement ratio
`m = dc/db`. Where `m != 0`, the inverse is `r = db/dc = 1/m`. At a toggle or
zero slope, the inverse is unbounded or locally undefined; chart gaps must carry
a reason. A central difference `(c(b+eps)-c(b-eps))/(2*eps)` approximates `m`,
but convergence with smaller `eps` and solver residuals must be checked. It is
not the finite secant `c/b`, which is undefined at the static origin and differs
on nonlinear motion curves.

For a declared angular input `q` in degrees, `dc/dq` has units mm/deg. For an
orientation `gamma` in degrees, `dgamma/dq` has units deg/deg. If the independent
coordinate is heave in mm, `dgamma/dh` has units deg/mm. These dimensional
identities explain why mixed units on an unlabeled common vertical scale cannot
establish formula equivalence. Conversion of an explicitly length/angle channel
from inch/deg to mm/deg multiplies by 25.4; it does not invert the channel.

## Ackermann comparison guard

The formula rendered on manual p.99 uses the ratio of actual inside angle to an
ideal inside angle calculated from outside angle, wheelbase and track. Studio's
standalone steering documentation defines a cotangent percentage. They are
different measures: neither may silently replace the other or be labeled merely
"Ackermann %" in comparison exports. The screenshot titled "Ackermann %" but
with a degree-valued axis is insufficient to select either convention.

## Baseline gaps found in source inspection

At the start of this task, `model.js` stores one mirrored left-corner geometry
per axle. It has no static suspension-wheel alignment fields, no body-motion
reference point, no CG/brake/drive inputs, and no explicit rocker-axis input.
`solver.js` solves equal left/right bump in `solveAxle`; it transports the upright
rigidly and solves rocker rotation about axle X. `arb.js` explicitly describes a
static preview, not an ARB kinematic model. These are verified implementation
facts, not measured properties of CR26.

The correct response to missing physical inputs is an editable, explicit model
choice or an unavailable channel with its reason. A plausible default must not
be presented as vehicle data. Existing stored geometry and coordinate meanings
must remain intact. Numerical tests can establish implementation consistency;
they cannot establish agreement with the user's car or with an untested
OptimumKinematics release.

## Implemented v0.3 coverage

The Python registry is `backend/vehicle_dynamics/models/kinematics.py::channel_registry`.
Channel count depends on topology and its hardpoint set; a count alone is not
proof of vendor equivalence. Each numerical value has explicit units and each
unavailable value has a reason. The UI offers 18 performance plot groups plus
family filters and arbitrary channel cross-plots; CSV includes every channel.

- Heave, pitch, roll and rack sweeps use the agreed **prescribed wheel-center**
  boundary, with explicit X/Y travel reference. No combined chassis-attitude or
  flat-road solve is implemented.
- Absolute chassis-relative camber/toe, changes from static, wheel heading,
  local bump gains and per-mode sensitivities require entered static alignment.
  Ground-relative alignment is not invented for a chassis-fixed rig.
- Damper length/compression, signed rocker angle, `dc/db`, and `dc/dq` use
  analytical constraint tangents and the explicit rocker axis/attachment.
  A separate spring displacement or inverse `db/dc` is not a separate channel.
- Spatial screw pitch, front/side instant centers, roll/pitch-center coordinates,
  roll-center support-reference height and roll-axis inclination are implemented.
  Infinite/singular centers remain gaps. Centers are geometric constructions,
  not an imposed chassis axis or a force-equilibrium prediction.
- DW/MacPherson physical-axis caster, KPI, trail and scrub are implemented.
  Five-link virtual-axis versions are explicitly unavailable pending a separately
  verified virtual-axis definition; its wheel poses/alignment remain supported.
- Front inside/outside heading, ideal outside heading, angle error and the existing
  **cotangent Ackermann percentage** are implemented, with the nonsteered-rear
  nominal reference and straight-ahead exclusions stated. They are not relabeled
  as the manual's inside-angle percentage.
- Wheel/support-point/hardpoint XYZ, track/change, projected side wheelbases,
  closure residual and condition number are implemented. Rear positions receive
  the front-frame translation exactly once.
- Braking anti-dive/rear anti-lift and drive front anti-lift/rear anti-squat use
  the separately derived spatial virtual-work coefficient, explicit CG height,
  axle force fractions and spin-torque reactions. No joint force result follows.
- The explicitly selected straight transverse U-bar supports left/right arm
  rotation, tip arc displacement, relative twist and per-mode derivatives.
  Missing carrier/axis definition is not inferred from static rendering.
- Joint forces/moments, material compliance, tire forces/deflection, and transient
  dynamics require other models and inputs and are not returned by this solver.

The differences above are deliberate limits, not unreported substitutions.
Software regression and independent geometric examples do not establish physical
accuracy or a numerical match to an untested OptimumKinematics release.
