# Straight transverse U-bar kinematics

This optional model must be explicitly selected per axle. Existing three-point
static previews do not silently become a fully defined mechanism. The user must
identify the rigid body carrying the suspension pickup. Unknown attachment
leaves ARB motion channels unavailable.

## Source review and model declaration

Seward Chapter4 §4.4.1, printed pp.111–112 / supplied PDF122–123, Fig4.11,
previously visually inspected, identifies the supported transverse bar, lever
arms and links. Smith Chapter6 printed pp.67–68 / PDF66–67, previously visually
inspected, illustrates mounting/link arrangements; these do not justify
inferring an unknown pickup carrier. Milliken Chapter16 §16.3 printed pp.595–600
/ PDF317–320, reviewed for installation ratios in this change, distinguishes
linear and torsional displacement ratios. None supplies CR26 bearing or carrier
measurements. The following is an original rigid-geometry derivation for the
explicitly selected straight transverse layout, not an ARB stiffness model.

The shaft axis is the line through the entered left bar bend and its mirrored
right bend. Its direction is global +Y. In this declared model the bend lies on
the shaft centerline. A complete optional bearing must lie on that line; an
absent bearing is not manufactured. Each arm rotates independently about the
same axis. A rigid, spherical-ended drop link retains its static length.
Shaft twist is reported; stiffness, forces, arm bending and bearing compliance
are not inferred. Nontransverse or otherwise different supports require a
different model, not guessed geometry.

## Closure and derivative

For bend B, static arm tip T0 and static pickup P0, let
`l = ||T0−P0||`. Given the solved pickup P, the arm satisfies

`T = B + R(k,alpha)(T0−B)` and `||T−P|| = l`.

Resolve the arm into parallel component u and perpendicular v with
`w = k cross v`, and let `d=B+u−P`. Squaring the length yields
`a cos(alpha)+b sin(alpha)=c`, where
`a=d dot v`, `b=d dot w`, `c=(l²−d dot d−v dot v)/2`.
The branch retains the neutral derivative sign. Both neutral and continuation
poses are checked; inaccessible/toggle states and rotations beyond ±90° are
unavailable. Sampling the path does not prove global uniqueness between samples.

Differentiate the drop-link closure, with `s=T−P`:

`alpha' = (s dot P') / (s dot [k cross (T−B)])`.

A zero denominator is a toggle, never a zero motion ratio. Pickup motion is
transported by its explicitly chosen upright, upper/lower wishbone, or rocker.
The same rigid transforms and tangents used for actuation apply. The shaft is
oriented +Y for **both** sides, so twist is `alpha_right−alpha_left`;
common heave rotation cancels for a symmetric assembly. Angular outputs use
degrees. Tip arc displacement is `radius*alpha` in mm; its mode derivative has
mm/mm or mm/deg units. Angular mode sensitivities are deg/mm or deg/deg.
These explicitly named quantities do not silently adopt a reciprocal vendor
ratio convention.

Numerical verification covers an independently known arm angle, exact link
length, heave symmetry, analytical derivatives versus finite differences,
missing carrier, incompatible bearings and toggle failures. Physical validation
requires surveyed ARB geometry and measured arm/drop-link motion.
