# Outer actuation pickup attachment

## Scope and decisions before implementation

The user requested an axle-specific choice of the rigid body carrying `actuation_outer`. A pushrod, pullrod or directly mounted damper does not, by its name, identify that body. For double wishbones the choices are upright, lower wishbone and upper wishbone. The chosen pickup's static XYZ coordinates are supplied by the user; the app does not move them to a guessed point on an arm. An offset bracket is represented as a rigid part of the selected body, with its entered offset preserved.

Multi-link retains upright attachment: a two-ended independent link does not define its axial orientation, so an off-axis attachment would require additional orientation/joint information. MacPherson uses the existing strut endpoints and has no separate `actuation_outer` pickup. This change introduces neither compliance nor joint articulation limits. The existing suspension pose solve and longitudinal rocker-axis assumption remain separate limitations.

All calculations below use axle-local millimeters, SAE +X forward, +Y right, +Z down. The editor uses front-origin coordinates through the existing explicit adapter. Rotation angles in the derivation are radians and follow the right-hand rule.

## First-principles derivation

A rigid wishbone with two distinct chassis-fixed pivots can rotate only about the line through those pivots. Let F be its fore inboard pivot, A its aft inboard pivot, B0 its static outer ball joint, B1 the outer ball joint returned by the suspension solve, and P0 the entered actuation pickup.

```text
k  = (A − F) / ||A − F||
r0 = B0 − F;                 r1 = B1 − F
v0 = r0 − k(k·r0);           v1 = r1 − k(k·r1)
theta = atan2(k·(v0 × v1), v0·v1)
R(k,theta)v = v cos(theta) + (k × v) sin(theta)
             + k(k·v)(1 − cos(theta))
P1 = F + R(k,theta)(P0 − F)
Bcheck = F + R(k,theta)(B0 − F)
```

Fixed distances from the ball joint to F and A place B1 on the intersection of two spheres, a circle about the pivot line. Therefore its axial coordinate and radial distance are invariant. The signed angle above recovers that circle rotation. The rotation applies to the whole wishbone and an attached bracket, including a pickup outside the plane of its three defining points. This preserves distances from the pickup to both pivots and the rigid arm's ball joint; it does not rotate the pickup with the steering upright.

The atan2 arguments both have units mm², so the resulting angle is dimensionless. Reversing the pivot order reverses both axis and angle, preserving the physical result. A common position translation cancels in the differences. Right-side reflection uses the existing proper axial-vector reflection contract; positions are reflected only after solving the corresponding left template.

The model does **not** force zero pickup movement whenever rack displacement changes at zero bump. Prescribed bump constrains wheel-center Z, not wishbone angle. Inclined/offset steering geometry can require an arm movement to satisfy all existing constraints. The pickup follows that solved arm movement. A synthetic pure-steering case with fixed arm ball joints must instead leave an arm-mounted pickup fixed.

After attachment transport, existing direct-damper compression or pushrod/pullrod constant-length rocker closure uses P1. The viewer and interference checker consume these same solved endpoints. Changing only the drawing would leave the actuator calculation incorrect.

## Numerical checks and limits

Before accepting transport, reject a pivot-axis separation or static/moved radial ball-joint lever at or below 1e−7 mm. These are numerical resolution bounds, not physical design limits. Independently require `||Bcheck − B1|| <= 1e−3 mm`, matching the existing pose solver's maximum accepted millimeter constraint residual. An off-circle solved point outside that tolerance fails explicitly; its coordinates are not projected onto a circle or silently overwritten. Zero travel returns the entered pickup. Degenerate arms cannot define an attachment rotation.

The existing pose solver targets 1e−5 mm residual and accepts at most 1e−3 mm. The reconstruction check is separately necessary: small individual link-length errors can be amplified by a poorly conditioned pivot triangle. These tolerances are software choices, not surveyed-hardpoint or manufacturing accuracy claims.

## Supplied-reference comparison

Relevant complete pages were newly inspected on 2026-10-29. PDF pages here are one-based; source files and extracted material remain outside the repository.

| Source | Verified passage / figure | Consequence and limit |
| --- | --- | --- |
| Seward, *Race Car Design*, Chapter 4 §4.1, printed pp.90–91 / PDF pp.101–102, Fig.4.2 | Distinguishes external spring/damper, pullrod with top wishbone, and upper rocker-arm arrangements. | Actuation type alone does not select the attached body. It does not supply the finite 3D rotation algorithm above. |
| Staniforth, *Competition Car Suspension*, third edition (1999), Chapter 3, printed pp.78–80 / PDF pp.68–70 | Direct mounting can act on axle/upright/wishbone; p.80 describes rod attachments to hub/upright lugs or brackets near the ends of top/bottom wishbones. Printed p.13 / PDF p.3 also contrasts upright and lower-wishbone direct mounts. | Explicitly supports retaining upright as well as upper/lower wishbone choices. Rod names cannot identify their attachment. |
| Milliken & Milliken, *Race Car Vehicle Dynamics*, Chapter 17 §17.1 printed pp.608–609 / PDF p.324, Figs.17.1–17.2; §17.5 printed pp.630–631 / PDF p.335, Fig.17.20 | Separates suspension path from commanded steering; represents an A-arm by two fixed-length links at an outer ball joint; actual inner pivots define arm pivot axes. | Use the entered fore–aft pivot line, not an assumed global X axis. The exact transport algebra is a project derivation. |
| Smith, *Tune to Win*, Chapter 6 printed pp.65–66 / PDF pp.64–65, Figs.34–36 | Describes lower spring attachment on a lower wishbone or hub carrier, and an inboard rocker commonly formed by an upper wishbone. | Agrees that the attachment body varies. A rocker wishbone carrying the spring itself is a different topology from the app's separate bell crank. |
| Gillespie, *Fundamentals of Vehicle Dynamics*, Chapter 8 printed pp.279–280 / PDF pp.295–296, Fig.8.4 | Control-arm outboard points follow pivot arcs; knuckle-ball paths and motion-derived centers are distinguished from anti-dive force constructions. | Supports tracking arm and knuckle motion separately. Its planar construction is not substituted for finite spatial rotation. |

Seward §3.4.1 printed pp.71–73 / PDF pp.82–84 concerns projected anti-dive geometry. Its simplified wheel-motion statements are not adopted as universal 3D constraints. No reviewed passage contradicts rigid rotation of a point attached to a fixed-pivot rigid wishbone within the stated assumptions. Agreement does not establish applicability to compliant or differently jointed mechanisms.

Rodrigues rotation was additionally cross-checked against the authors' [Modern Robotics §3.2.3](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-2-3-exponential-coordinates-of-rotation-part-2-of-2/). No supplied vehicle-dynamics book is claimed to provide this implementation's algorithm.

## Persistence and validation

The optional v2 axle field `actuationAttachment` records `upright`, `lower-wishbone` or `upper-wishbone`. Missing fields preserve the previous upright behavior, explicitly displayed in the editor; this is compatibility behavior, not an inference about a real car. Generated layouts also declare upright until the user chooses another member. Changing attachment preserves all static hardpoints. Unsupported topology/attachment combinations must fail instead of falling back during a solve.

Executed on 2026-10-29:

- All 218 Node regression tests passed without skips, including 11 focused attachment tests and six schema/persistence tests. The independent inclined-axis +90° fixture gives pickup `[156, -240, -182]` mm exactly at the displayed precision; ball reconstruction error is 2.84e−14 mm. Its expected coordinates use an independently prescribed orthonormal basis, not the production rotation helper.
- Across 36 combined corner cases (upper/lower attachment × pushrod/pullrod/direct × three bump/rack inputs × left/right), maximum attachment residual was 9.89e−7 mm, maximum pickup-to-arm distance error 8.05e−7 mm, and maximum rod-length closure error 2.26e−6 mm. Left/right reflection discrepancy was zero at the checked floating-point values. These bounds describe synthetic fixtures, not vehicle measurement accuracy.
- Pure steering with the steering axis through the wheel center preserves both wishbone ball joints and either arm-mounted pickup within 1e−4 mm while an upright-mounted pickup moves. A separate inclined/offset case confirms that fixed wheel-center height can coexist with arm and pickup movement.
- Off-plane brackets, pivot reversal, common translation, both corner reflections, direct-damper length differences, unchanged non-actuator pose points, legacy upright equivalence, singular arms, independent ball-circle reconstruction and tolerance sensitivity were checked. Inputs outside the declared numerical bounds fail explicitly.
- Heave-check integration tests cover front/rear axles, both corners, either wishbone and all three actuation methods: checked pickup/damper endpoints match the corrected solved geometry and differ from the old upright-carried endpoints in the prescribed moving cases.
- Local browser checks exercised front lower-wishbone and rear upper-wishbone selections, unchanged entered XYZ, +15 mm rack, +20 mm bump with +15 mm rack, rear −20 mm rebound, and +15 mm heave with 20 mm shared link OD. Solves completed and the preview/checker updated. No console errors were observed in these sessions. Saved choices survived reload and JSON export, with AXLE_LOCAL storage retained. Unsupported-topology controls and unfinished-number retention were also checked. Browser automation encountered timeouts around the native topology confirmation; the resulting multi-link reset and disabled selector were observed, but confirmation-dialog presentation itself is not claimed as verified.
- The existing 58 Python model/API/deployment tests passed without skips after including Node on PATH. The suspension attachment calculation remains in the existing browser solver; no new backend suspension model is claimed.

Software regressions and reference review are not measured-vehicle validation. Physical validation still requires the actual attachment, surveyed geometry, rigid-body/joint suitability and measured pickup/actuator travel. The reviewed first-principles result is certain for an ideal rigid wishbone with two fixed distinct pivots and a nondegenerate outer ball-joint radius; correctness for every real suspension or every possible input is not established by these checks.
