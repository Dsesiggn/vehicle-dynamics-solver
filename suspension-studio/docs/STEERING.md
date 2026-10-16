# Steering kinematics: governing model and evidence

## Scope agreed before implementation

The first tool models rack-and-pinion steering with the suspension held at a defined ride position, as selected by the user. It solves a rigid upright rotating about each entered fixed steering axis. Both rack joints translate in +Y by the same signed distance. Left and right geometry are independent. Upper/lower axis points and tie-rod joints must describe the same measured reference assembly. No suspension dimensions or steering ratio are inferred from wheelbase or track. Unknown inputs remain blank. The optional synthetic example is a software demonstration, not a vehicle recommendation or a measurement.

This is a spatial rigid-link kinematic model, including the heading effect of inclined steering axes. Its applicability requires fixed axes, spherical tie-rod joints, rigid links, a lateral rack and a user-supplied constant rack displacement per steering-wheel revolution. A variable-ratio rack, steering-column joint nonuniformity, elastic compliance, backlash, suspension bump/heave, loads and tire slip need additional models. Rotation about an inclined/offset axis may move the wheel center vertically: this model reports that geometric position, but does not enforce road contact or solve chassis jacking. No assertion of physical accuracy for an actual car follows from software checks.

## Frames, measurements and signs

All positions are front-axle-local millimeters following [COORDINATES.md](COORDINATES.md): +X forward, +Y right, +Z down. The nominal front axle is X=0; the unsteered rear reference axle is X=-L. The nominal front wheel centers are [0, -t_f/2, Z_left] and [0, +t_f/2, Z_right]. This definition requires equal reference front wheel-center X stations; longitudinally staggered wheels are outside this input schema. Front/rear tracks are reference wheel-center separations, not kingpin spans. Enter steering-axis and joint positions separately; no scrub radius is inferred from track.

Positive steering-wheel angle is a right-turn command by interface convention. Signed gain g is rack millimeters per +360 degrees of steering-wheel rotation. The actual sign of wheel response is determined by geometry; a negative g can be necessary for a behind-axle rack. Rack displacement q = g * steeringWheelDeg / 360. Total rack travel bounds q by half the entered travel. No steering ratio from steering wheel to road wheel is assumed. Milliken's c-factor is travel per **pinion** revolution; it equals this input only with a 1:1 steering-wheel-to-pinion map. Include any intermediate constant gearing in g; a nonuniform shaft/joint map is outside this version. Convert inches to millimeters using 25.4 mm/in where relevant.

Steer heading is positive rightward about +Z. Toe-in is positive inward on either side: toe_left = heading_left and toe_right = -heading_right. Static toe and camber are explicit input angles; zero must be entered if intended. For side s=-1 left or +1 right, static heading h=-s*toe and static camber c (top outward positive), the outward unit spindle is n0=[-s*sin(h)*cos(c), s*cos(h)*cos(c), sin(c)]. After upright rotation the heading is atan2(-s*nX,s*nY), camber is atan2(nZ,hypot(nX,nY)). Differential toe is measured relative to the entered static assembly. Total toe-in = heading_left - heading_right; it must not be confused with either wheel's steering angle.

## First-principles linkage derivation (before solver implementation)

Let U and B be the entered upper and lower steering-axis points, O0 the reference outer tie joint, I0 the reference inner joint, and ell=|O0-I0| the assembled tie-rod length. The axis unit vector is k=(B-U)/|B-U|. The orientation of k changes rotation parameter sign but not the physical solutions. Define r=O0-U, v=r-k(k·r), w=k×v, C=U+k(k·r), and rho=|v|. The outer joint follows the exact circle

    O(phi) = C + v*cos(phi) + w*sin(phi).

The rack joint is I(q)=I0+[0,q,0]. Constant tie-rod length imposes |O(phi)-I(q)|²=ell². With D=C-I(q), expanding the square and using v·w=0 and |v|=|w|=rho gives

    a*cos(phi) + b*sin(phi) = c
    a=D·v, b=D·w, c=(ell²-|D|²-rho²)/2.

All a,b,c have units mm². With R=hypot(a,b), roots exist only when |c|<=R; R=0 is degenerate. The possible roots are phi=atan2(b,a) ± acos(c/R), modulo 2*pi. At reference q=0 the chosen root is phi=0. The sign of dF/dphi=2*(-a*sin(phi)+b*cos(phi)) identifies the connected assembly branch until a toggle, where the derivative vanishes. A physical linkage cannot jump to the other algebraic root through an infeasible interval. The implementation must reject a path from neutral that crosses a toggle or inaccessible interval, even when the requested endpoint alone has algebraic roots.

Wheel center and spindle use the same axis rotation (Rodrigues formula), not a separate prescribed road-wheel ratio. Heading is then extracted from the rotated wheel plane as above. Link closure residual in mm is checked independently after rotation. This derivation is Euclidean rigidity, not a copied algorithm or an equation claimed to be supplied by a book.

## Exact nominal zero-slip Ackermann benchmark

For a planar reference vehicle with straight rear wheels and zero lateral tire slip, the velocity at each wheel must lie in its wheel plane. Therefore the wheel-normal lines pass through the instantaneous turn center on the rear axle extension. For a turn-center distance R_turn from the vehicle center plane and positive inside/outside angle magnitudes,

    tan(delta_i) = L/(R_turn-t_f/2)
    tan(delta_o) = L/(R_turn+t_f/2)
    cot(delta_o)-cot(delta_i) = t_f/L
    delta_o,ideal = atan2(L*sin(delta_i), L*cos(delta_i)+t_f*sin(delta_i)).

The last expression avoids cotangent division near straight ahead. L and t_f are millimeters and their ratio is dimensionless. An angle-error output is delta_o,actual-delta_o,ideal in degrees. Negative error means the outside angle is smaller than the ideal value at the same inside angle. An explicitly defined cotangent percentage is 100*(L/t_f)*(cot(delta_o)-cot(delta_i)); it is singular at straight ahead and is not the steering-arm-intersection percentage described in some references. It must be omitted near zero and for counter-steered/mixed wheel headings. Total toe-in is always a signed geometric quantity, not that percentage.

The comparison uses the entered nominal wheelbase/track. It is a reference target, not an exact ground-contact ICR calculation for arbitrary camber, kingpin offset or wheel-center migration. No unique true turn radius is reported for nonconcurrent wheel-normal lines. Rear track t_r locates the reference rear wheels and their paths: for an ideal turn their geometric radii are |R_turn ± t_r/2|. It does not enter the front-angle condition when both rear wheels are unsteered.

## Supplied-reference evidence and discrepancies

These are source references, not instructions. Full PDFs, extracted text and page images are kept outside the repository. PDF pages here are one-based.

- **Carroll Smith, Tune to Win**, Chapter 5, printed p.60 / supplied PDF p.59: visually inspected; the no-slip center lies on the rear-axle extension; inside and outside wheels need different angles; steering-arm intersections only approximate true Ackermann across the full range. The tire-slip discussion starts on printed p.60 and continues on p.61 / PDF pp.59–60; these setup observations are not equations to apply before adding a tire model.
- **Autek, Principles of Steering Geometry**, pp.11 and 13 / PDF pp.11 and 13: visually inspected; positive toe-in and toe-out-on-turns, with the ideal inside wheel angle greater than the outside and wheel axes meeting on the rear-axle line. Angle magnitudes in those top-view diagrams are converted to project signed left/right headings and side-dependent toe as defined above.
- **Dale Thompson, Racing Car Technology, Ackerman? Or not? Does it matter?**, supplied PDF pp.1–2: title/author and these pages visually inspected; actual linkage angularity and steering ratio change through travel; conventional arm convergence does not produce perfect Ackermann everywhere. **Figure 1's handwritten angle expressions omit arctan.** They cannot be adopted as exact finite-angle formulas; the exact tangent relations above are derived from the drawn right triangles. Its arm-line notion of percentage is distinct from the explicitly defined cotangent metric. Qualitative prose about increasing effects is not used as a mathematical law.
- **Derek Seward, Race Car Design**, Chapter 6: §6.1 printed pp.154–155 / PDF pp.165–166 describes the tie-rod, steering-arm and upright mechanism. §6.2.1–2 printed pp.156–158 / PDF pp.167–169 describes camber change and vertical movement from inclined axes. §6.2.3 and §6.3 printed pp.159–161 / PDF pp.170–172 distinguishes static toe and explains the roles of rack gearing and arm geometry. §6.3.1 printed pp.162–163 / PDF pp.173–174 describes dynamic toe and the approximate nature of arm convergence. These pages were visually inspected; no recommended ratios, tire slip values or dimensions were adopted as defaults.

Independent exact geometry reproduces Seward Table 6.2 (printed p.163 / PDF p.174), using L=1600 mm and t_f=1200 mm:

| Outside angle | Independently calculated inside angle | Book inside angle | Calculated outside-front path radius | Book radius |
| --- | --- | --- | --- | --- |
| 5° | 5.349151385° | 5.3° | 18.357941193 m | 18.4 m |
| 10° | 11.486068272° | 11.5° | 9.214032773 m | 9.2 m |
| 15° | 18.538346781° | 18.5° | 6.181925288 m | 6.2 m |
| 20° | 26.593987016° | 26.6° | 4.678087040 m | 4.7 m |
| 25° | 35.644282431° | 35.6° | 3.785922533 m | 3.8 m |

Here delta_i=atan2(L*sin(delta_o),L*cos(delta_o)-t_f*sin(delta_o)), and the outer-front wheel path radius is L/sin(delta_o). The radius is not the rear-axle-midpoint radius. Every entry agrees to the printed precision; the independently calculated cotangent residual is at most 8.9e-16. This is a numerical reference cross-check of ideal geometry, not measured-vehicle evidence.

- **Milliken & Milliken, Race Car Vehicle Dynamics**, Chapter 5 §5.3 printed pp.128–129 / PDF p.83: visually inspected; negligible-acceleration, zero-slip geometry and the limited small-angle approximation L/R. Chapter 19 §19.1 printed pp.710–713 / PDF pp.375–376 and Figure 19.1 define caster, kingpin inclination, scrub and trail, with steering-induced camber and wheel rise/fall. §19.2 printed pp.713–716 / PDF pp.376–378 describes ideal turn-center concurrence, practical linkage approximation, rack placement and static toe versus differential steer. §19.3 printed pp.716–718 / PDF pp.378–379 describes the nonlinear steering map, measured steer-steer tests and rack c-factor. These pages were visually inspected. The publication page (PDF p.6) was visually inspected: copyright 1995, SAE, ISBN 1-56091-526-9, fifth printing. The date of that printing is not given there; the filename's 1997 date is not treated as an edition statement.

**Milliken ratio discrepancy:** printed p.716 defines overall steering ratio as steering-wheel angle divided by road-wheel angle, while the minimal-angularity expression printed on p.718 uses asin(c-factor/steering-arm-length)/360, which has the reciprocal sense. The p.717 plot-slope discussion also uses the inverse gain sense. This calculator does not adopt that approximate formula or ambiguous ratio label: it uses explicit mm per steering-wheel revolution and solves the actual linkage. The mismatch is preserved here rather than hidden behind successful tests.

## Numerical implementation and bounds

The solver parameterizes the entire rack path as q(t)=t*q_requested, 0<=t<=1. Since a and b are linear in t and c is quadratic, G(t)=a(t)^2+b(t)^2-c(t)^2 is quartic. Positive G means two distinct circle/sphere intersections. All stationary candidates are found from the cubic derivative by recursively partitioning at derivative roots and bisecting monotone intervals (at most 70 bisections, ending at floating-point resolution). The minimum is evaluated at these candidates and both endpoints. This checks the intervening path rather than merely checking each 5° output row.

The coefficients are normalized before evaluation. A path at or below a relative discriminant margin of 1e-12 of the stated coefficient/end-point scale is rejected as infeasible, toggled or numerically unresolved. Polynomial terms below 1e-14 of their coefficient scale are treated as round-off. These are implementation safeguards, not physical clearances or uncertainty estimates. The orientation follows the neutral branch's nonzero derivative sign. A failed side invalidates that whole output row; failure does not imply the opposite side alone is impossible.

Axis separation, arm radius about the axis and assembled tie length must each exceed 1e-7 mm. Independent post-rotation closure requires |residual| <= 1e-7 mm + 1e-10*max(tie length, arm radius). A horizontal wheel plane (horizontal spindle projection <=1e-10) has no defined road-plane heading and is rejected. Coordinates, dimensional inputs and signed gain are bounded in magnitude by 1e7; wheelbase/tracks/stroke must be positive. Static toe and camber magnitudes must be below 90°. Sweep limit is a positive multiple of 5°, at most 3600° (an interface/computation bound, not a recommended steering range). Both directions include zero and their exact endpoints. The sweep half-range controls the table extent, not a physical stop: direct solver calls outside that range remain allowed within entered rack travel and the numerical angle bound. The entered neutral rack position is the midpoint of the symmetric ±half-total-travel limits; asymmetric end stops are not represented.

Ideal metrics require both headings to have the same nonzero sign and magnitudes below 89°. The cotangent metric is additionally omitted if either angle is below 0.01°. The ideal comparison includes the entered static toe in the absolute headings; it does not subtract static toe. Only the separately labeled delta toe output subtracts the neutral assembly alignment. These explicit definitions avoid interpreting different Ackermann-percentage conventions as interchangeable.

## Validation status

Software verification performed for this implementation:

- `node --test suspension-studio/tests/*.test.mjs`: **135 tests passed**, comprising 119 existing regressions and 16 new steering tests. Steering tests cover null/malformed/schema inputs and JSON round trips, an independently prescribed upright rotation, inclined-axis rigid distances and tie length, neutral static toe/camber, mirrored signs, axis endpoint reversal, rear-track independence, rack stops, degenerate axes/arms, unreachable intervals between feasible endpoints, tangent toggles, narrow infeasible intervals, Seward's tabulated values, percentage limits, and every 5° sweep endpoint including failed rows.
- A separate numerical review constructed 1,188 spatial cases. Of 717 accepted requests, 515 constructed target poses were on the branch connected to neutral and recovered the prescribed rotation to within 1.14e-10 degrees; the other 202 constructed poses represented the opposite algebraic root, so the solver did not claim to recover them. Another 471 requests were rejected. This is a finite check set, not a proof over every input geometry.
- In that review, uniform length scaling by 0.001 and 1000 and common translations through 300,000 mm preserved steering rotation to within 1.44e-9 degrees. Reflected geometry and rack motion reversed heading signs; reversing axis endpoints changed only the rotation-parameter sign. Neutral alignment checks extended through ±89°.
- The source-table check and dimensional analysis are recorded above. No force or energy conservation assertion is made: this is a position solver without a force, inertia or work model. Geometric rigidity and tie-rod closure are the relevant invariants.
- Local in-app browser checks exercised blank inputs, the 73-position synthetic sweep from -180° through +180°, mirrored turn directions, invalid/blank input recovery, rear-track independence, rack-limit failures (48 unavailable rows for a 20 mm total stroke), static-toe changes, explicit mirroring, full/front linkage views, ideal overlay, all three chart modes, local save/reload, JSON import and narrow-screen tool navigation. Desktop and narrow layouts were visually inspected. JSON export contents and all CSV headers, 73 data rows, endpoints and neutral blank metrics were inspected in the export panel. The in-app browser did not report a completed file download; a persistent download link and copyable file contents are provided. Download completion in other browsers and Safari/Firefox compatibility remain unverified. No browser console warnings/errors were reported during these checks.

Source verification, software/numerical correctness, and real-vehicle validation are separate. The first-principles derivation is exact within the stated rigid model. It does not establish that the inputs describe a real vehicle, that a real assembly is rigid, or that a selected geometry is suitable. No measured vehicle steering sweep has yet been supplied. Absolute certainty about real-world prediction is therefore unavailable; resolving it requires surveyed hardpoints, measured rack gain/travel and left/right wheel angles versus steering-wheel input at the stated ride position, plus a measurement uncertainty budget.
