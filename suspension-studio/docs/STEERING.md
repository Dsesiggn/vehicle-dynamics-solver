# Steering kinematics: governing model and evidence

## Python calculation engine

The steering page now calculates through the same-origin Python service. The exact fixed-axis closure, neutral-connected branch check, signs and numerical limits derived below are preserved in `backend/vehicle_dynamics/models/steering.py`; construction angles and intersections are in `steering_geometry.py`. Python returns every 5° pose and its construction data. The browser renders those results and retains editing/draft storage; the JavaScript solver remains a regression reference, not a runtime fallback. A missing service leaves calculations unavailable with startup/retry guidance. The API, port methodology and verification are recorded in `docs/STEERING_BACKEND.md` and `docs/HYBRID_VERIFICATION.md` at the repository root. Changing language does not itself improve or validate physical accuracy.

## Scope agreed before implementation

The first tool models rack-and-pinion steering with the suspension held at a defined ride position, as selected by the user. It solves a rigid upright rotating about each entered fixed steering axis. Both rack joints translate in +Y by the same signed distance. Left and right geometry are independent. Upper/lower axis points and tie-rod joints must describe the same measured reference assembly. No suspension dimensions or steering ratio are inferred from wheelbase or track. Unknown inputs remain blank. The optional synthetic example is a software demonstration, not a vehicle recommendation or a measurement.

This is a spatial rigid-link kinematic model, including the heading effect of inclined steering axes. Its applicability requires fixed axes, spherical tie-rod joints, rigid links, a lateral rack and a user-supplied constant rack displacement per steering-wheel revolution. A variable-ratio rack, steering-column joint nonuniformity, elastic compliance, backlash, suspension bump/heave, loads and tire slip need additional models. Rotation about an inclined/offset axis may move the wheel center vertically: this model reports that geometric position, but does not enforce road contact or solve chassis jacking. No assertion of physical accuracy for an actual car follows from software checks.

## Frames, measurements and signs

All positions are front-axle-local millimeters following [COORDINATES.md](COORDINATES.md): +X forward, +Y right, +Z down. The nominal front axle is X=0; the unsteered rear reference axle is X=-L. The nominal front wheel centers are [0, -t_f/2, Z_left] and [0, +t_f/2, Z_right]. This definition requires equal reference front wheel-center X stations; longitudinally staggered wheels are outside this input schema. Front/rear tracks are reference wheel-center separations, not kingpin spans. Enter steering-axis and joint positions separately; no scrub radius is inferred from track.

Positive steering-wheel angle is a right-turn command by interface convention. Signed gain g is rack millimeters per +360 degrees of steering-wheel rotation. The actual sign of wheel response is determined by geometry; a negative g can be necessary for a behind-axle rack. Rack displacement q = g * steeringWheelDeg / 360. Total rack travel bounds q by half the entered travel. No steering ratio from steering wheel to road wheel is assumed. Milliken's c-factor is travel per **pinion** revolution; it equals this input only with a 1:1 steering-wheel-to-pinion map. Include any intermediate constant gearing in g; a nonuniform shaft/joint map is outside this version. Convert inches to millimeters using 25.4 mm/in where relevant.

Steer heading is positive rightward about +Z. Toe-in is positive inward on either side: toe_left = heading_left and toe_right = -heading_right. Static toe and camber are explicit input angles; zero must be entered if intended. For side s=-1 left or +1 right, static heading h=-s*toe and static camber c (top outward positive), the outward unit spindle is n0=[-s*sin(h)*cos(c), s*cos(h)*cos(c), sin(c)]. After upright rotation the heading is atan2(-s*nX,s*nY), camber is atan2(nZ,hypot(nX,nY)). Differential toe is measured relative to the entered static assembly. Total toe-in = heading_left - heading_right; it must not be confused with either wheel's steering angle.

## First-principles linkage derivation (before solver implementation)

### Longitudinal rack placement editor

The rack-offset input is a layout editor for the existing inner-joint coordinates, not an additional kinematic degree of freedom. For the entered neutral inner joints I_L and I_R, define x_rack=(I_LX+I_RX)/2. The front-axle datum is X=0, so a positive x_rack is ahead of that axle and a negative value is behind it, in millimeters. This midpoint definition is an explicit interface choice that supports independently entered, longitudinally staggered joints; it does not assert that both X coordinates are equal.

For requested offset s, a rigid translation of both rack-joint reference positions gives

    ΔX = s - x_rack
    I_L' = I_L + [ΔX,0,0]
    I_R' = I_R + [ΔX,0,0].

Thus (I_LX'+I_RX')/2=s and I_RX'-I_LX'=I_RX-I_LX; all Y/Z values remain unchanged. These are direct coordinate identities, independent of tire slip or a steering approximation. Unknown X values cannot supply a midpoint: a separately labeled **Set both rack X values** action assigns I_LX=I_RX=s explicitly. It does not infer Y, Z, joint spacing, or any other missing input. Typing is staged until Apply or Enter; clearing the proposed value does not delete joint coordinates. Reset entry discards the proposed edit. The current layout and calculations remain visible while an edit is pending, and Save/JSON/CSV are disabled until it is applied or reset.

The solver continues to use I(q)=I0+[0,q,0], q=g*steeringWheelDeg/360; longitudinal placement and lateral steering stroke are distinct. Applying a rack layout edit leaves outer joints, steering axes and entered static toe/camber unchanged. Because the model defines each neutral tie-rod length as |O0-I0|, that reference length is recalculated after this edit. It is a design reassembly with the tie rods adjusted to the entered static geometry, not a simulation of moving a built rack fore/aft with fixed-length rods. The JSON schema stores only the existing joint coordinates; offset is derived on reload/import and cannot become a conflicting second geometry source. Invalid values and edits whose resulting X coordinates exceed the existing ±1e7 mm numerical bound are rejected atomically; that bound is not a vehicle recommendation.

The relevant physical cross-check is Milliken §19.2, printed p.715 / PDF p.377, Figure 19.4, and Seward §6.3.1, printed pp.162–163 / PDF pp.173–174: fore/aft rack placement changes tie-rod/arm angularity and the practical differential-steer relationship. This supports recomputing actual rigid-link closure after editing the hardpoints; it does not justify a universal percentage correction or a fixed sign of change for every linkage. The coordinate transformation above is derived here, not attributed to a book equation. Further source comparisons and any unresolved differences are recorded in [REFERENCES.md](REFERENCES.md).

Rack-editor checks performed: 11 tests cover signed/zero offsets, asymmetric X preservation, all Y/Z and unrelated inputs, frozen-input nonmutation, explicit unknown-X assignment, malformed values, coordinate-bound rejection without partial edits, existing JSON round trips without a duplicate scalar, and identical solved results to equivalent direct hardpoint edits at five steering inputs. Browser checks confirmed that clearing/retyping a staged offset leaves joint positions intact; translating X=[-120,-80] mm to midpoint -60 mm produces [-80,-40] mm and retains the selected +180° steering-wheel position. These are coordinate/editor and software checks, not physical rack-relocation measurements.

### Fixed-axis closure

Let U and B be the entered upper and lower steering-axis points, O0 the reference outer tie joint, I0 the reference inner joint, and ell=|O0-I0| the assembled tie-rod length. The axis unit vector is k=(B-U)/|B-U|. The orientation of k changes rotation parameter sign but not the physical solutions. Define r=O0-U, v=r-k(k·r), w=k×v, C=U+k(k·r), and rho=|v|. The outer joint follows the exact circle

    O(phi) = C + v*cos(phi) + w*sin(phi).

The rack joint is I(q)=I0+[0,q,0]. Constant tie-rod length imposes |O(phi)-I(q)|²=ell². With D=C-I(q), expanding the square and using v·w=0 and |v|=|w|=rho gives

    a*cos(phi) + b*sin(phi) = c
    a=D·v, b=D·w, c=(ell²-|D|²-rho²)/2.

All a,b,c have units mm². With R=hypot(a,b), roots exist only when |c|<=R; R=0 is degenerate. The possible roots are phi=atan2(b,a) ± acos(c/R), modulo 2*pi. At reference q=0 the chosen root is phi=0. The sign of dF/dphi=2*(-a*sin(phi)+b*cos(phi)) identifies the connected assembly branch until a toggle, where the derivative vanishes. A physical linkage cannot jump to the other algebraic root through an infeasible interval. The implementation must reject a path from neutral that crosses a toggle or inaccessible interval, even when the requested endpoint alone has algebraic roots.

Wheel center and spindle use the same axis rotation (Rodrigues formula), not a separate prescribed road-wheel ratio. Heading is then extracted from the rotated wheel plane as above. Link closure residual in mm is checked independently after rotation. This derivation is Euclidean rigidity, not a copied algorithm or an equation claimed to be supplied by a book.

## Tie-rod / effective steering-arm construction (derived before implementation)

At each selected pose let O be the outer tie-rod ball center and I the inner center. The effective steering arm is the perpendicular radius from the steering axis to O, not the shape of a cast/bent arm and not a segment to an arbitrarily chosen upper or lower joint. With the same axis unit vector k as above, its axis foot is C=U+k[k·(O-U)]. This is the center of the outer joint's rotation circle; (O-C)·k=0. Its position is invariant under a different choice of points on the same axis.

The included angle **at O** uses rays a=C-O (toward the axis) and b=I-O (toward the rack). From Euclidean dot and cross products,

    angle_3D = atan2(|a × b|, a · b) × 180/pi.

Both arguments have units mm²; the displayed unsigned angle is in [0,180] degrees. The spatial result is labeled effective 3D angle β. The separate conventional plan angle α uses the steering-axis point at the outer ball height, defined below; it is not generally the projection of β. Canvas arcs represent only α; readouts explicitly distinguish both constructions. Neither angle is a spherical-joint articulation limit, and 90 degrees in 3D alone is not a general criterion for maximum steering leverage about an inclined axis.

For the conventional top-view construction, Milliken Figure 19.3 (printed p.714 / PDF p.377) locates the kingpin axis at the outer tie-rod ball height. Define K=U+k*(O_Z-U_Z)/k_Z. K and O share Z. The displayed plan angle α is the included angle between XY(K-O) and XY(I-O), using the same dot/cross formula. For inclined axes K generally differs from perpendicular foot C; these are deliberately separate definitions. A horizontal axis does not supply a unique K, so this construction is unavailable.

For v_L=O_L-K_L, v_R=O_R-K_R (XY components), solve the infinite construction lines K_L+lambda*v_L=K_R+mu*v_R with the scalar 2D cross product:

    lambda = cross2(K_R-K_L,v_R)/cross2(v_L,v_R)
    P = K_L + lambda*v_L
    rear_axle_offset = P_X + L.

P_X and P_Y are front-axle-local mm (+X forward, +Y right). Positive rear-axle offset means ahead of X=-L. This is a **projection intersection**; its placeholder Z=0 is not a spatial intersection or ground contact. Finite intersection coordinates are reported even outside the current view; an explicit Fit intersection control includes them in the view. Parallel, coincident and degenerate projections have no unique reported intersection. Failed steering poses explicitly show static construction only; CSV leaves their construction outputs blank.

Source basis: Seward §6.3.1, printed pp.162–163 / PDF pp.173–174, Smith Chapter 5 printed p.60 / PDF p.59, and Milliken §19.2 printed pp.713–716 / PDF pp.376–378 describe conventional steering-arm convergence as an approximation. The overlay uses the ball-height construction explicitly labeled in Milliken Figure 19.3. The separate spatial perpendicular-foot definition and dot/cross derivation above are explicit project definitions, not equations attributed to those sources. Steering-arm intersection is not the wheel-normal instantaneous turn center, does not by itself establish ideal Ackermann, and does not define the tool's cotangent Ackermann percentage. No tire-slip model or real-arm shape is inferred.

Construction numerical choices (not physical limits): use normalized vectors for angle and line-cross calculations. A plan construction is unavailable when |k_Z|≤1e-10. Lines are parallel within numerical tolerance when the cross product of their XY unit directions has magnitude≤1e-10; coincident additionally requires perpendicular separation≤1e-10 times the maximum of their projected arm lengths and axis-point separation. Angle ray lengths≤1e-12 times the larger radius/tie length are unresolved; a projected rod length≤1e-12 of its spatial length is unresolved. Nonfinite values remain unavailable. The perpendicular radius is evaluated by the equivalent k×(r×k), using the nearer axis endpoint. A radius≤32 times machine epsilon times the norm of the component product-sums in r×k is rejected as unresolved roundoff before forming any arm line. This prevents an on-axis outer joint from producing an artificial intersection. These guards are numerical resolution choices, not a vehicle geometry recommendation; no finite physical intersection is clamped to the viewport.

Construction checks performed: 14 automated cases include known 90°/135° angles, independent inclined-axis geometry, finite/parallel/coincident/undefined lines, scale/translation and reflection, extended steering-axis endpoints, on-axis roundoff, and neutral/moving/failed solver poses. A separate hand calculation for U=[30,-530,-450], B=[-10,-560,-150], O=[-100,-520,-250], I=[-100,-250,-250] mm gives α=106.189206257027° and β=106.856448253359°; the browser shows 106.19° and 106.86°. The synthetic symmetric example has neutral intersection X=-1833.333333 mm, Y=0, 233.333333 mm behind the nominal rear axle. At +180° wheel input its plan angles are left 89.085304° / right 128.505304° and its intersection is [-1549.802999,-549.467024] mm; reversing input swaps the angles and reflects Y. These are mathematical/software checks, not physical vehicle measurements.

## Nonhorizontal linkage and the 3D viewer

The Python linkage model already evaluates the full XYZ coordinates. Neither the tie
rod nor the effective steering arm is constrained to be horizontal. In particular,
the rod constraint is

    (O_X−I_X)² + (O_Y−I_Y)² + (O_Z−I_Z)² = ell².

The 3D rotation about the entered U–B axis changes all applicable components of O,
the wheel center and the spindle. No projection is used in solving this constraint.
Different inner-joint heights, inclined axes and nonhorizontal effective arms are
supported within the fixed-ride, rigid-link, spherical-joint assumptions. The rack's
motion remains I(q)=I0+[0,q,0]: different rack-end heights do not define a tilted
actuation axis. Rack travel along another direction needs an explicit additional model.

A useful limiting case prevents an incorrect expectation: for a vertical steering
axis at fixed ride, both O_Z and I_Z are constant throughout the turn. Their squared
height difference occurs in both the current constraint and the assembled reference
length and cancels. Changing only that difference, while reassembling rod length from
the new reference hardpoints, then leaves the same XY steering relation. For an inclined
axis O_Z can change with rotation, and the height difference couples into the solution.

An independent synthetic verification uses k=[0,0.6,0.8], C=[0,−550,−250],
O0=[−100,−526,−268] and I0=[−110,−250,−180] mm. A prescribed 12° upright rotation gives
q=−13.264892238736 mm by intersecting the rack travel line with the sphere of radius ell
about the independently rotated outer joint. At g=−60 mm/revolution this is a
79.589353432416° steering-wheel input. The backend recovers heading 9.574999677828°
and camber −0.600995140912°. Changing only I0_Z to −268 mm, keeping XY and q fixed
and reassembling the reference rod length, gives heading 7.400205119311°.
Additional positive/negative prescribed rotations, independent distance-root checks,
rigidity and the vertical-axis cancellation case pass. These are synthetic mathematical
checks, not measurements of the user's assembly.

The original 2D display projects solved joint coordinates as screen-horizontal ∝ Y
and screen-vertical ∝ −X. Its joints are therefore the actual orthographic XY projection
of the spatial solution; its fixed-size wheel icons show heading only and omit camber
and physical tire shape. Its arm is the conventional K–O plan construction, not the
spatial perpendicular radius C–O. It was not a rendered three-dimensional scene.

The default **3D geometry** view now builds a Three.js scene from those same Python
results. Actual axis points, inner/outer joints, wheel centers and spindle orientations
retain all three coordinates. Dashed, side-colored C–O radius references show effective
steering arms, not an assumed bent-arm casting shape. Wheel-plane orientation rings are perpendicular to the
returned spindle. Static geometry provides a separate reference. Changing the camera
does not recalculate kinematics or substitute a planar steering solution.

The display adapter maps SAE coordinates to the renderer by

    [x_render, y_render, z_render] = [Y_SAE, −Z_SAE, −X_SAE].

This dimensionless orthogonal matrix has determinant +1, preserving distances, dot
products, cross-product handedness and the physical orientation. It changes only display
basis; saved hardpoints and reported values remain SAE axle-local millimeters. Isometric,
top, front and side views use an orthographic camera on the same scene; orbit, pan, zoom
and fit affect the camera only. The separate **2D construction** mode retains the original
schematic and is labeled accordingly. WebGL unavailability is reported explicitly and
does not disable Python calculations.

Plan construction overlays are deliberately projected onto the labeled Z=0 reference
plane. Their intersection is still a planar construction and is not presented as the
meeting point of spatial steering arms. The spatial β arc uses C−O and I−O; α uses the
separately defined projected K−O and I−O rays. The displayed radii, link thicknesses,
markers and wheel-plane ring sizes are visual aids, not measured component dimensions.
No tire diameter/width, rear-wheel center height, contact patch, real arm contour,
collision envelope or joint articulation limit is inferred. Rear markers show only
the nominal rear-axle datum at X=−L on the Z=0 reference plane.

The physical interpretation remains consistent with the previously inspected Seward
§6.2.1–2 pp.156–158 / PDF pp.167–169 and Milliken §19.1 pp.710–713 / PDF pp.375–376
on inclined-axis camber and wheel movement, and Milliken Figure 19.3 p.714 / PDF p.377
for the distinct plan construction. The cross-source comparison above remains applicable;
no conflicting supplied-source statement changes the Euclidean rigid-link derivation.
Three.js is a renderer only; its [orthographic camera documentation](https://threejs.org/docs/#OrthographicCamera)
describes the display projection, not vehicle dynamics.

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

- **Thomas D. Gillespie, Fundamentals of Vehicle Dynamics**, Chapter 6 printed pp.196–198 / PDF pp.212–214 and Chapter 8 printed pp.276–280 / PDF pp.292–296: visually inspected in the follow-up review. Equations 8-1/8-2 on printed p.277 distinguish exact arctangent geometry from small-angle approximations; practical linkage approximation, rack translation and suspension-induced toe are separately scoped. Figures 6.1/8.2 use a radius drawing whose datum differs from that required by the exact denominators; the app retains its explicitly derived rear-axle-midpoint transverse radius. The cross-source comparison and unresolved source definitions are recorded in [REFERENCES.md](REFERENCES.md). Edition/year of this supplied scan remain unverified.

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
