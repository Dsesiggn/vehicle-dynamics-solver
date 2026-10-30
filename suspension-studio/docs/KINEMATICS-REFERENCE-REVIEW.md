# Kinematics reference review and geometric derivation

Reviewed 2026-10-29. This is a focused review of the supplied books, not a claim that every chapter or every OptimumKinematics calculation has been verified. Source PDFs, extracted text and rendered pages remain outside the repository. The commercial output-channel review is separate: [KINEMATICS-CHANNELS.md](KINEMATICS-CHANNELS.md).

## Source evidence actually inspected

The complete pages listed here were visually inspected, including their figures and neighboring explanations. Printed pages and one-based PDF pages are distinguished because the Milliken scan has two printed pages per PDF page.

| Source | Passage inspected | What it establishes; limits |
| --- | --- | --- |
| Milliken & Milliken, *Race Car Vehicle Dynamics*, Chapter 16, section 16.3 | Printed pp.595-600, PDF pp.317-320, Figs.16.2-16.3 | Installation ratio is spring compression per vertical wheel displacement; it can vary with travel. Computer analysis still requires accurate measured geometry. Torsional/anti-roll installation ratios need their own displacement and angular definitions. No universal rocker axis is supplied. |
| Milliken, Chapter 17, sections 17.2-17.3 | Printed pp.610-615, PDF pp.325-327, Figs.17.3-17.9 | Independent wheel location has five constraints and one ride freedom. Instant centers are local, moving constructions, not finite rotation centers for an entire sweep. Three-dimensional motion requires an instant axis. The front-view contact-to-instant-center lines intersect at a roll center that can migrate laterally. |
| Milliken, Chapter 17, section 17.5 | Printed pp.630-633, PDF pp.335-336, Figs.17.20-17.22 | Wishbones rotate around their actual inner-pivot axes. The MacPherson strut shaft need not coincide with the upper-mount/lower-ball steering axis. The diagrams are useful constructions, not a fully specified arbitrary-spatial numerical algorithm. |
| Gillespie, *Fundamentals of Vehicle Dynamics*, Chapter 7 | Printed pp.257-258 and 263-265, PDF pp.273-274 and 279-281, Figs.7.14, 7.19-7.20 | Defines a force-coupling roll center and discusses each wheel's center-plane reaction height. With roll and unequal lateral tire forces, those heights differ and the effective force reaction changes. This does not justify treating a geometric center as a complete force/load-transfer model. |
| Gillespie, Appendix terminology note 16 | Printed p.412, PDF p.428 | The idealized roll-center definition does not necessarily represent the true instantaneous center of sprung-mass rotation. |
| Gillespie, Chapter 8 | Printed pp.279-280, PDF pp.295-296, Fig.8.4 | A knuckle-point motion center need not equal the virtual center used for anti-dive/anti-squat force analysis. Steering linkage closure must participate in the wheel-motion solution. |
| Seward, *Race Car Design*, Chapter 3, sections 3.1-3.3 | Printed pp.62-70, PDF pp.73-81, Figs.3.1-3.7 | Equal/opposed wheel motion, positive camber with the tire top outward, front-view instantaneous-center/roll-center constructions and migration. Equal parallel wishbones are a useful limiting case: zero bump-camber change and ground camber following body roll in the illustrated body-roll experiment. The latter is not a commanded wheel-travel rig with a stationary chassis. |
| Seward, Chapter 4, section 4.2.4 | Printed pp.100-102, PDF pp.111-113, Figs.4.5-4.6 | Motion ratio is wheel travel divided by spring travel; installation ratio is its inverse. A bell-crank ratio varies with actual geometry. Coplanarity/right-angle guidance is design advice, not permission to assume an unknown axis or attachment. |
| Smith, *Tune to Win*, Chapter 3 | Printed pp.29-30, PDF pp.28-29, Figs.12-13 | Geometric roll center and front/rear connecting roll axis; the accompanying text acknowledges vertical and lateral migration. |
| Smith, Chapter 4 | Printed pp.47-48, PDF pp.46-47, Fig.22 | Explicitly distinguishes tire-to-road camber from tire-to-chassis camber and constructs a laterally displaced center in roll. Equal parallel links have an infinite front-view instant center. |
| Smith, Chapter 6 | Printed pp.65-66, PDF pp.64-65, Figs.34-36 | Wheel/spring motion-ratio convention and changing spring-axis geometry. |

Staniforth's previously verified attachment and steering-axis passages remain applicable as recorded in [REFERENCES.md](REFERENCES.md). No additional Staniforth equation is claimed in this review. Pacejka's tire-force equations are not used: the selected circular tire below is an explicitly idealized geometry model, not a pneumatic tire model or measured contact-patch reconstruction.

## Agreement, differences and unresolved applicability

- **Ratio naming is resolved:** Milliken's installation ratio and Seward/Smith's motion ratio use reciprocal conventions. They describe the same mechanism. The user explicitly selected `d(damper compression) / d(wheel bump)` for this dashboard. The plot must display that numerator and denominator; it must not imply it is OptimumG's reciprocal channel without conversion.
- **Kinematic and force centers differ:** the migrating contact/instant-center intersection in Milliken/Seward/Smith is a geometric construction. Gillespie's discussion of separate left/right force-reaction heights and unequal lateral forces is a different question. The dashboard may report the declared **kinematic roll-center construction**, not an effective force-based center or proof of the actual sprung-mass rotation axis. This resolves the apparent disagreement by scope; the inspected sources do not establish that the two quantities coincide for arbitrary three-dimensional geometry.
- **Spatial extension requires a declared method:** the books support instantaneous geometry but do not fully specify a numerical treatment of a nonzero-pitch screw motion. The implementation below uses the independently derived instantaneous screw axis and the three-plane geometric construction described in the commercial channel review. It reports the screw pitch and does not silently assume pure rotation.
- **No body attitude is inferred from a rig command:** a wheel-travel pitch/roll command is not a force response, a freely rolling chassis, or a body constrained by four flat-road tire contacts. Ground-referenced camber cannot be obtained by simply adding the commanded travel angle to a chassis-relative result.
- **Input geometry remains essential:** exact rocker-axis data, attachment-body ownership and static wheel alignment must be supplied for outputs depending on them. Missing data must remain unavailable. Generated geometry is an illustration, not evidence about CR26.

## Coordinate, unit and modeling contract

Use [COORDINATES.md](COORDINATES.md): SAE J670 Z-down, X forward, Y right, Z down; millimeters; internal angles in radians; displayed angles in degrees. Stored/solver geometry is axle-local. Move rear positions into the common front-origin frame by subtracting wheelbase from X exactly once. Directions and angular velocities receive no translation. Physical height above a zero-Z reference is `-Z`.

The user confirmed:

1. **Prescribed wheel-center travel**, with a stationary chassis, rather than a body-attitude/flat-road solve.
2. Explicit inputs for missing alignment, rocker axis and motion-reference position.
3. Motion ratio as damper compression divided by wheel bump.
4. An **ideal rigid circular tire center-plane support point**, derived from entered diameter and solved spindle orientation. The support points follow the wheels during commanded travel; four common flat-road contacts are not imposed.

The supplied books use physical height-above-ground drawings and a mixture of inch and millimeter examples. Here a positive physical height becomes negative project Z; imperial lengths would be multiplied by 25.4 before entering geometry. The reviewed ratio definitions are dimensionless and require only the declared reciprocal conversion. Positive-outward camber agrees with Seward and Smith. Roll figures are used as geometric limiting cases, not copied as signed angular inputs. No source force equation or force-center sign convention is imported into these geometric metrics.

Rigid links, ideal joints, rigid upright, fixed chassis hardpoints, and a specified steering-rack position are the model assumptions. Compliance, bushing friction, tire deformation, tire force, sprung-mass equilibrium and dynamics are outside this kinematic solve.

### Travel modes

Let the static front-origin wheel-center station of corner i be `(X_i^0,Y_i^0)`, the entered reference station be `(X_ref,Y_ref)`, and h be common upward wheel bump in mm. Define pitch travel angle theta and roll travel angle phi by the commanded plane slopes:

    b_i = h - (X_i^0 - X_ref) tan(theta)
            + (Y_i^0 - Y_ref) tan(phi)
    C_i,z = C_i,z^0 - b_i

This is the exact **definition of the rig's commanded height differences**, not a finite rigid-body rotation law. The nominal stations stay fixed in the command calculation while solved wheel-center X and Y are free to follow the suspension. Positive roll travel compresses the right side relative to the left; positive pitch travel gives the front more rebound than the rear. The definition does not depend on a reference Z. Pure heave uses theta=phi=0. Singular tangent angles and travel outside the solver's supported range must be rejected explicitly.

### Rigid closure and tangent velocity

For a static upright point p0 and static wheel center C0, describe the current point by

    p = C + R (p0 - C0),   R in SO(3).

Each spherical-ended locating link to a fixed chassis/rack point a imposes

    f = ||p - a|| - ell = 0,

where ell is its static length in mm. Four wishbone-leg constraints plus the tie rod, or four independent locating links plus the tie rod, provide five scalar constraints. A conventional telescoping strut supplies two independent constraints keeping its chassis top point on the shaft line fixed in the upright; with two lower-arm legs and a tie rod this again gives five. The commanded wheel-center Z is the sixth equation. A rank-deficient geometry is not repaired by prescribing an arbitrary orientation.

The instantaneous velocity of any point fixed to the upright is

    v(p) = v_C + omega cross (p - C).

Differentiating a link constraint, with `u=(p-a)/||p-a||`, gives

    u dot (v_C + omega cross (p-C) - v_a) = 0.

Its Jacobian row for `[v_C,omega]` is `[u, (p-C) cross u]`. Holding rack position fixed sets `v_a=0`; a rack sweep supplies its actual +Y velocity. This shows why the tie rod participates in the instantaneous motion and why the two wishbone-plane intersection alone must not replace the complete constrained tangent calculation. The angular velocity is a spatial angular velocity, not the componentwise derivative of a finite rotation vector.

### Alignment

Let n be the outward unit spindle, s=-1 on the left and +1 on the right. In the stationary chassis frame:

    camber = atan2(n_z, hypot(n_x,n_y))
    steer  = atan2(-s*n_x, s*n_y)
    toe_in = -s*steer

These reproduce the existing positive-outward camber and positive-inward toe conventions. Their frame and static alignment must be explicit. OptimumG channels using toe-out require sign reversal, not relabeling. Upright rotation transports the entered static spindle; it must not replace it with a vehicle-Z yaw.

### Actuation and motion ratio

For damper endpoints D1 and D2, length `l=||D2-D1||` and compression `c=l_static-l`. The actual endpoints follow their assigned rigid bodies. A separate bell crank rotates about its entered unit axis k through pivot A:

    P(theta_r) = A + R(k,theta_r)(P0-A).

A constant pushrod/pullrod length determines theta_r on the connected assembly branch. No universal longitudinal axis is assumed. A direct damper uses its actual transported endpoint without this extra crank solve.

For the user-selected ratio, `MR=dc/db`, where b is upward wheel-center travel. This is dimensionless mm/mm. A derivative with respect to the rig's pitch or roll angle is instead `dc/dtheta` or `dc/dphi`, in mm/deg when plotted in degrees. It is a different channel. Do not divide c by b at zero travel or use a whole-sweep secant as an instantaneous ratio. Since `dl/du = unit(D2-D1) dot (dD2/du-dD1/du)`, the tangent mechanism gives a local derivative directly. Finite-difference agreement and step refinement are appropriate independent checks. A reciprocal is undefined at a zero slope; it must not be clipped to a plausible finite value.

## Pure geometry implemented by kinematic_geometry.py

These are independent Euclidean derivations applied to the solver's current points and tangent velocities. They do not implement force equilibrium.

### Circular tire support point

Given wheel center C, outward unit spindle n, radius r>0 and chassis-down unit vector d=(0,0,1), the circle is the set `C+r*u` with `u dot n=0` and `||u||=1`. Maximize its Z coordinate. Projection of d into the wheel plane gives

    a = d - (d dot n)n
    Q = C + r*a/||a||.

Q is the lowest point of that ideal circular center plane. It is not the centroid of a finite-width, cambered pneumatic contact patch. If n is parallel to d, the circle is horizontal and there is no unique lowest point; report unavailable. The point follows the wheel and can have a different Z at each corner under the selected rig motion. Tire tread width is not used to invent a contact profile.

### Instantaneous screw axis

For nonzero omega, let `w2=omega dot omega`. Define

    A = C + (omega cross v_C)/w2
    k = omega/||omega||
    h_screw = (omega dot v_C)/w2
    v_parallel = h_screw*omega.

A+k*t is the instantaneous screw axis. Substitution into the rigid velocity equation gives `v(A+k*t)=v_parallel`; thus it is a zero-velocity axis only when the screw pitch is zero. The pitch has units mm/radian; velocities retain the tangent parameter's units. Reversing/rescaling that parameter does not change the axis as an unoriented line or its pitch. Shifting the common coordinate origin translates A and C together.

At exactly zero angular velocity there is no unique finite axis. A pure translation can be approached by zero-pitch rotations with axes at infinity, but it can also be approached by divergent-pitch screws with other limiting contact planes. The implementation reports the finite-axis construction unavailable rather than inferring a unique limiting axis from translation alone. A stationary pose derivative is likewise not an arbitrary origin-centered axis.

### Kinematic roll and pitch centers

For each corner, form the plane containing its instantaneous screw axis and the declared circular support point Q:

    N = k cross (Q-A)
    N dot (P-Q) = 0.

The plane is undefined if Q lies on the axis. Normalize N before assessing distances. For an axle, take the vertical plane through the left/right support points; with `g=Q_R-Q_L`, its normal is `g cross d`. Intersect that plane with the two corner planes using a three-by-three linear system. This is the **kinematic roll-center construction**. For one vehicle side, the same construction using front/rear support points defines the geometric **pitch center**. A missing axis, coincident contacts in plan, dependent planes, or ill-conditioned intersection produces an unavailable result with a reason, not zero or infinity.

With planar, pure-rotation front-view motion this reduces to the familiar contact-to-instant-center line intersection in the supplied books. For arbitrary spatial screw motion it remains a declared geometric construction; a nonzero `h_screw` means the axis carries translation and the resulting center cannot be advertised as a zero-velocity rotation point. Report both screw pitches and numerical conditioning with the result. Unequal support heights in the rig do not assert common flat-road contact.

### Steering-axis geometry

For a defined physical steering line, use lower-to-upper direction k. Double wishbones use lower and upper outer ball joints; the modeled MacPherson arrangement uses lower ball joint and chassis upper strut bearing. A multi-link virtual axis requires its separately derived instantaneous steering motion and must not be fabricated from two arbitrary points.

With k pointing upward and side s:

    caster = atan2(-k_x,-k_z)
    kingpin_inclination = atan2(-s*k_y,-k_z).

These are vehicle side/front-view angle definitions, positive for top rearward/inboard. Intersect the line with the horizontal plane through the declared support point Q to obtain G. The wheel-plane forward direction projected onto this plane is `f = s*(n cross d)/||n cross d||`, and its outward lateral direction is `l=s*(d cross f)`. Then

    mechanical_trail = (G-Q) dot f
    scrub_radius      = (Q-G) dot l.

Lengths are mm. Positive trail places the steering-axis intersection ahead of the support point; positive scrub places Q outward of G. These are declared geometric metrics at each corner's support height, not pneumatic trail or a deformed contact-patch measurement. A horizontal steering axis or undefined wheel heading makes the dependent intersection/metric unavailable.

## Verification and certainty

Source verification above is complete only for the listed passages and stated claims. It does not establish equivalence with a proprietary solver, a real car, or all commercial output channels.

The accompanying pure-geometry suite was run on 2026-10-29: **16 tests passed**. It checks independently calculable cases: circular support extrema; spindle sign invariance; finite inclined-axis rigid velocity; screw pitch/parallel velocity; invariance to origin translation and tangent-parameter scaling; symmetric and asymmetric planar roll centers; pitch-center plane intersections; singular and nearly singular geometry; and independently constructed caster/KPI/trail/scrub. A zero-angular-velocity case must explicitly remain undefined rather than claim a finite instant axis. These checks verify mathematical/software behavior, not experimental validation.

Additional integration checks are required on the actual model: rigid distances and strut constraints, tangent residuals, finite-difference convergence, mirror symmetry, all required editor/viewer behavior, and comparison with surveyed hardpoints and measured wheel/damper motion. Passing software tests cannot make the real-vehicle predictions 100% certain. No measured-vehicle validation is claimed here.
