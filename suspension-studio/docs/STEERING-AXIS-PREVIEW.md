# Steering-axis preview and wheel-pose audit

## Decisions and first principles before implementation

The wheel geometry must follow the complete solved rigid-upright pose, not a separately imposed rotation about vehicle Z. The current suspension solver returns a rotation vector r and solved wheel center C1. The current preview already uses `n1 = R(r) n0`, with static outward spindle `n0 = [0, side, 0]` and side −1 left / +1 right. The tire plane is perpendicular to n1 and centered on C1. R is the finite Rodrigues rotation, not Euler yaw. The audit has found no vertical-axis substitution in that rendering path. Initial independent inclined-axis checks also reproduce the complete wheel pose; formal regression results are recorded after execution below.

A rigid upright with two fixed steering pivots L and U can turn only about their connecting line. For double wishbone suspension these pivots are the lower and upper outer ball joints. For the modeled conventional MacPherson suspension, use L = lower outer ball joint and U = upper strut mounting/bearing center (`strut_top`). The strut shaft between `strut_bottom` and `strut_top` need not coincide with this steering axis. Both definitions follow the supplied-reference evidence below. A five-link suspension can have a moving virtual steering axis; no physical two-pivot line will be invented for it.

The preview will therefore add a clearly identified dashed steering-axis reference through the current two defining points, independently of the wheel spindle and physical strut shaft. It will update with solved suspension points during rack, bump and heave motion. A display checkbox will control its visibility. Unavailable or degenerate definitions will be explained and omitted. No change to the existing suspension pose solver is justified by the audit so far.

## Geometry and conventions

All position calculations use axle-local millimeters and SAE +X forward, +Y right, +Z down. Only the final position display is translated into the existing front-origin assembly frame. Free directions receive no translation. Rotation angles are radians and obey the right-hand rule; displayed alignment changes are degrees.

For distinct steering pivots L and U:

```text
k = (U − L) / ||U − L||
P1 = L + R(k, theta)(P0 − L)
n1 = R(k, theta) n0
R(k,theta)v = v cos(theta) + (k × v) sin(theta)
             + k(k·v)(1 − cos(theta))
axis(t) = L + t(U − L)
```

P0/P1 are static/rotated upright-attached positions; n0/n1 are spindle directions. The fixed-pivot rotation is a test construction and conditional physical result, not an additional constraint imposed during simultaneous suspension travel. In the actual six-pose solve the arm ball joints may move while the rack moves at prescribed wheel-center height. The current steering-axis reference may then move too; it is not generally the single finite rotation axis of the entire combined bump/steer displacement.

For a MacPherson strut with static bottom B0 and rotated bottom B1, a rotation fixing U gives `R(U − B0) = U − B1`. Thus the rotated strut direction still passes through the top mount while the steering-axis line U–L differs from the shaft line U–B. This is consistent with the existing telescoping strut-line constraints. It does not require forcing the wheel to rotate about the shaft itself.

The display line uses the actual finite segment between its defining endpoints; it introduces no physical member or collision geometry. A line with coincident or nonfinite endpoints is omitted rather than replaced by a vertical line. Its numerical minimum endpoint separation is 1e−7 mm, a rendering resolution choice, not a physical design limit. No caster/KPI, ground-intercept, scrub-radius or jacking-force calculation is added here.

The static wheel spindle is still assumed lateral because the schema does not contain static wheel orientation. This audit verifies changes from that entered-model pose, not absolute alignment on a surveyed car. Ideal rigid links/joints, no compliance, and the existing rocker-axis limitations remain applicable.

## Supplied-reference evidence and comparison

Complete relevant pages were visually inspected on 2026-10-29; PDF page numbers are one-based. PDFs, page images and extracted text remain outside the repository.

- Milliken & Milliken, *Race Car Vehicle Dynamics*, Chapter 17 §17.5, printed pp.632–635 / PDF pp.336–337, especially Fig.17.22 and printed p.633: the upper strut mount lies on the kingpin/steering axis, while the strut shaft need not coincide with that axis. This is the governing distinction for the MacPherson reference line.
- Milliken, Chapter 19 §19.1, printed pp.709–711 / PDF pp.374–375: the upper/lower ball joints define the double-wishbone steering axis; inclined steering affects camber and other wheel geometry during steering.
- Gillespie, *Fundamentals of Vehicle Dynamics*, Chapter 7, printed pp.243–244 / PDF pp.259–260: the telescopic strut maintains wheel camber, has an upper chassis mount, and works with lower links. This mechanism description agrees with the top-mount/lower-ball definition and does not establish that shaft and steering axes coincide.

- Seward, *Race Car Design*, Chapter 6 §§6.2.1–6.2.2, printed pp.156–157 / PDF pp.167–168, Figs.6.2–6.3: defines the double-wishbone steering axis through upper/lower outer ball joints in front and side views, with steering-induced camber and jacking.
- Staniforth, *Competition Car Suspension*, “King Pin Inclination” and “Caster Angle,” printed pp.107–108 / PDF pp.97–98: upper/lower upright pivots replace a physical kingpin; their connecting line defines the axis. Its “MacPherson Strut” discussion, printed p.66 / PDF p.56, describes the sliding architecture but does not assert that shaft and steering axes coincide.
- Smith, *Tune to Win*, Chapter 4, printed pp.41–42 / PDF pp.40–41: discusses strut packaging and compliance; no opposing precise steering-axis definition was found there. This passage is not used as proof of the axis equation.

The explicit pivot definitions in Seward, Staniforth and Milliken agree for the modeled double-wishbone layout. Milliken Fig.17.22 supplies the precise MacPherson distinction; the other reviewed strut descriptions are compatible but less specific. No governing contradiction was found in these reviewed passages. Pacejka is not used: this is rigid suspension geometry, not a tire-force model. This comparison is a focused relevant-page review, not a claim that all six books were exhaustively checked. No sourced angle equation is imported: the finite rigid-body rotation and line construction are the project derivation above. Rodrigues rotation was independently cross-checked with the authors’ [Modern Robotics §3.2.3](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-2-3-exponential-coordinates-of-rotation-part-2-of-2/). Source diagrams establish the geometric entities; their printed axis/sign conventions are not relabeled as the project SAE frame. The construction uses the project's already converted XYZ data, so no new source-to-project scalar sign conversion is required.

## Independent audit methodology

Use a rational inclined unit axis k = [3/13, 4/13, −12/13]. Prescribe a finite rotation by independently expanding its 3×3 matrix rather than calling the production rotation helper. Expected upright points and spindle follow the equations above. For wheel center C0/C1, request `bump = C0_z − C1_z` so the test follows the existing upward-positive bump contract. This is essential when the wheel center is offset from an inclined steering axis; setting bump to zero would prescribe a different problem.

Choose static inner tie point I, static outer tie point S0, and constant tie length ell = ||S0 − I||. For prescribed rotated S1 and rack translation d along +Y, independently recover:

```text
d = S1_y − I_y − sign(S0_y − I_y)
    * sqrt(ell² − (S1_x − I_x)² − (S1_z − I_z)²)
```

Use the branch connected to the neutral assembly and verify a real square root and supported rack travel. Every term in d has units mm; the radicand has units mm². The expected solved wheel center and spindle can then be checked without deriving the expectation from the solver output. Check both signs, reflected corners, off-axis centers, a center-on-axis zero-bump limit, and a deliberately noncollinear MacPherson shaft/kingpin. A nonzero steering-induced camber change rejects a yaw-only wheel orientation. Also check exact endpoint ownership and display-frame translations of the new axis geometry.

Source verification, numerical/software checks and measured-vehicle validation are separate. Passing these checks cannot establish correctness for every real suspension or every possible input.

## Executed validation (2026-10-29)

- All 231 JavaScript regression tests passed without skips, including seven new independent wheel-pose tests and six new steering-axis geometry tests. All 58 existing Python tests passed without skips. No suspension solver equations or constraints were changed for this display update.
- Across eight inclined-axis reference corner cases (double wishbone/MacPherson, positive/negative 12° prescribed rotation, left/right), maximum constraint residual was 4.0081e−6 mm, wheel-center error 4.2452e−6 mm, rotation-vector error 2.2772e−8 rad, and spindle-direction error 1.7174e−8. These are floating-point verification errors for synthetic fixtures, not measured-vehicle tolerances.
- The prescribed inclined-axis fixtures produce camber changes +3.106163° and −2.394115°. A global-Z/yaw-only wheel rotation would leave the starting lateral spindle without this Z component. Off-axis wheel centers require bump inputs −2.870941 mm and +1.927021 mm for the prescribed −12° and +12° cases; the tests use these values rather than accidentally prescribing fixed height.
- Tests cover both corners, both display axle frames, true tire-plane normals and tread edge centers, the center-on-axis zero-bump limit, and a MacPherson shaft deliberately noncollinear with the steering axis. The new reference-line tests check actual endpoint ownership, normalized 3D directions, solved positions, rear translation, reflection, missing/degenerate endpoints, multi-link unavailability and input immutability.
- An additional independent renderer-only 90° construction about [0.6, 0, 0.8] predicts wheel center [−30, −400, −140] mm and spindle [0.8, 0, −0.6]. The preview center matched exactly and the spindle error was 6.13e−17. Rear wheelbase translation changed positions and preserved directions.
- Local browser verification exercised double-wishbone +15 mm rack/+10 mm bump, MacPherson ±15 mm rack and +10 mm bump, a 200 mm tread band, front camera, light/dark rendering, axis visibility and legend/note synchronization. Toggling the axis left solver results unchanged. Switching to multi-link disabled the axis control and displayed the unsupported-definition explanation. A rear upper-ball X edit to −1570 mm in the front-origin editor was accepted and refreshed the preview. No console errors were observed in the checked sessions.
- The normal generated layouts were also audited without altering them: at +15 mm rack and zero prescribed bump, default double wishbone produces a 3D rotation [−4.30e−5, −0.0228441, 0.1645033] rad and wheel-center displacement [7.891724, 0.649345, 0] mm. Default MacPherson likewise has nonzero spatial rotation components. These illustrative defaults are not claimed to represent the user's measured car.

The verified conclusion is that the checked suspension solver/preview path uses the full spatial upright pose, with no vertical-axis substitution found. The new overlay makes its steering pivots visible and distinguishes the MacPherson shaft. Physical accuracy for the user's real suspension is not 100% established: it requires surveyed hardpoints, the actual steering/joint architecture, initial wheel orientation and measured steer/camber/wheel-center travel under matching constraints. There is no experimental-validation claim.
