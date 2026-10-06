# Heave interference check

## Contract

The check evaluates one axle at a time with a pure heave input: the left and right wheel centers receive the same travel, and rack displacement is zero. All coordinates and distances are in that axle's SAE J670 Z-down frame (`+X` forward, `+Y` right, `+Z` down), in millimeters.

Let `c ≥ 0` be compression and `r ≥ 0` be rebound. The position solver uses positive bump for upward wheel motion, so:

```text
compression endpoint: bump = +c      Z_wheel = Z_static - c
rebound endpoint:     bump = -r      Z_wheel = Z_static + r
```

Both magnitudes default to 25.4 mm and can be edited independently up to the current solver limit of 100 mm. The travel slider selects a wheel-center bump directly. At each selected value, the same kinematic solver is run, then clearances are evaluated from that solved pose; the slider result is not interpolated from scan samples.

The solver treats the axle inboard hardpoints as chassis-fixed. Upright-side points move with the solved rigid upright. Fixed links maintain their input lengths, and the existing rocker solve pivots its rocker around its X axis before damper and actuator-link endpoints are checked. This uses the model's kinematic constraints; it does not model bushing motion, flex, loads, or chassis motion.

## Members and shapes

The shared **link OD** has no default because the user has supplied no link size. It represents the largest link OD in the design and is applied uniformly to control-arm legs, tie/toe links, pushrods/pullrods, and the three centerline arms currently drawn for each bell crank. This intentionally uses the maximum specified OD as a conservative clearance envelope. The damper bodies use the existing per-axle **damper OD**; the MacPherson strut is checked as a damper body.

Each member is modeled as a capsule: a finite line segment between its two hardpoints, expanded by a sphere of radius `OD/2`. The resulting envelope is a straight circular cylinder with rounded ends. This lets the checker use one shared diameter for rods whose CAD details and rod ends have not been entered. It does not infer rod-end, bearing, weld, bracket, or rocker-plate solids. The three-arm bell-crank profile is an app representation, not verified plate geometry.

Two types of pairs are checked:

- Every modeled member against each complete packaging envelope whose **Show in preview** box is on: engine and differential axis-aligned boxes, or the cockpit's convex extruded trapezoid.
- Every member against every other modeled member on both sides of that axle. This includes link–link, link–damper, damper–damper, and opposite-side pairs.

Pairs that share a hardpoint at the evaluated pose are excluded from member-to-member contact. Their intended joint fit cannot be assessed without ball joints, bearings, or mounting brackets. Incomplete or hidden obstacle envelopes do not enter the check. Springs, tires, uprights, ARB parts, chassis members, and brackets are also outside the current scope.

## Clearance equations

For finite centerline segments `A(t)=A₀+t(A₁−A₀)` and `B(u)=B₀+u(B₁−B₀)`, where `t,u ∈ [0,1]`, their minimum centerline distance is:

```text
d_seg-seg = min ||A(t) - B(u)||₂
clearance_links = d_seg-seg - (OD_A + OD_B)/2
```

For a member segment and a closed convex obstacle solid `S`, the minimum centerline-to-solid distance is:

```text
d_seg-solid = min distance(A(t), S)
clearance_obstacle = d_seg-solid - OD_member/2
```

The implemented finite-segment distance uses the clamped closest-points solution. The segment-to-solid distance checks whether an endpoint lies inside the convex mesh, whether the segment intersects its triangulated faces, and the minimum segment-to-triangle distance across the surface (including face-interior, edge, and endpoint cases). The app's box and cockpit meshes are convex and have outward-wound faces. Distances are Euclidean and retain the SAE axes; no sign conversion is applied to scalar clearance.

`clearance ≤ 0 mm` means the two idealized envelopes touch or overlap. For two capsules, the signed axis-distance expression gives a useful separation metric. For a capsule whose axis lies within an obstacle solid, centerline-to-solid distance is zero, so the reported clearance is `−OD/2`; that negative number signals overlap but is **not** the actual penetration depth into the obstacle. A positive value is the modeled surface-to-surface gap.

## Travel sweep and slider

The full scan covers `[-rebound, +compression]`, includes static travel at zero, and samples wheel-center travel at intervals no greater than 0.25 mm. For each sample it solves both corners and evaluates all applicable member pairs and selected obstacle shapes. When a clearance changes from positive to nonpositive (or back), binary refinement narrows that sampled transition to a travel bracket no wider than 0.01 mm. Reported event endpoints are therefore numerical brackets, not analytical roots.

The scan can miss a very narrow contact interval that falls completely between 0.25 mm samples, or a contact feature smaller than the idealized geometry captures. When the user moves the slider, the app solves that exact requested bump position (the existing pose solver's residual tolerance is 0.001 mm) and recalculates the clearances there. Interference members are highlighted in the live viewer. Selecting a reported event moves the slider to a sampled contacting position within that event.

## Verification and limits

The software tests include analytic segment distances (10 mm face/side gaps, a 13 mm three-axis corner gap, 10 mm parallel segment separation and segment crossing), inside-solid/intersection cases for box and cockpit meshes, and a synthetic link-envelope sweep whose contact event is then rechecked at the selected slider position. Separate solver checks confirm that a 25.4 mm heave keeps inboard points unchanged and preserves link lengths while moving outboards. All topologies are tested for default compression/rebound convergence in the main solver tests.

These checks establish implementation consistency for the declared geometric shapes. They are not a comparison against CAD solids, fabricated hardware, measured vehicle clearances, or physical tests. The supplied vehicle-dynamics books do not provide the equations used for finite-segment/convex-solid distances, so the code documents this as Euclidean computational geometry rather than attributing it to those books. No vehicle-specific packaging conclusion should be inferred from generated demo hardpoints or placeholder obstacle dimensions.
