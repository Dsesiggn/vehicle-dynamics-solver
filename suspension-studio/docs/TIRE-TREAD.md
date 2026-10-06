# Tire tread-width preview

## Input, frame and scope

The user explicitly defined the requested width as **tread width only**. Each axle stores `tireTreadWidth` in millimeters, either `null` (unknown) or a finite positive number through the application's 10000 mm input limit. No preset provides a measured tread width. Existing files without this property import as unknown. Width does not change hardpoints, kinematic constraints, alignment outputs or heave clearances.

Coordinates follow the axle-local SAE J670 contract: +X forward, +Y right, +Z down. The renderer uses the existing static lateral wheel-axis assumption and rotates it with the solved upright. This assumption does not establish absolute static alignment. No source-coordinate conversion is introduced.

## Geometric construction

Let `C` be the wheel center [mm], `a` a normalized wheel-axis vector [dimensionless], `D` the existing tire outer-diameter input [mm], and `W` the entered tread width [mm]. Set `R = D/2`. An ideal constant-radius tread surface consists of points

`P(t, θ) = C + t a + R (u cos θ + v sin θ)`,

where `−W/2 ≤ t ≤ W/2`, `0 ≤ θ < 2π`, and unit vectors `u` and `v` span the plane perpendicular to `a`. The edge-circle centers are therefore `C ± aW/2`. Their midpoint is `C` and their separation is `W` because `|a| = 1`. Each term in `P` has units of millimeters. The viewer draws sampled circles and axial guide lines as a wireframe representation of this construction.

For coordinate component `i`, projection of a radius vector in the plane normal to `a` has maximum magnitude `R sqrt(1 − a_i²)`. Adding the maximum axial contribution gives the exact axis-aligned half-extent

`e_i = |a_i| W/2 + R sqrt(max(0, 1 − a_i²))`.

Bounds are `C_i − e_i` through `C_i + e_i`; the tiny roundoff clamp prevents a negative square-root argument. With an axis parallel to Y, half-extents are `[R, W/2, R]`. When the width is unknown, the displayed object is only the center-plane circumference; its bounds use no axial contribution and its returned edge-center list is empty. This does not assign a zero tread width to the design.

## Applicability limits and source status

The constant-radius band is an explicit visualization choice. Tread width and outer diameter do not define actual shoulders, sidewalls, mounted section width, loaded radius, deformation or rim dimensions. Tire contact/force/stiffness models and tire interference checks are outside this change. Actual packaging clearance requires appropriate measured/CAD tire envelopes and a collision model that includes them.

The construction is original Euclidean rendering geometry. No equation is attributed to the six supplied books, and no vehicle-dynamics source verification is claimed for it. Their catalog remains the required reference library for future simulation models. The existing coordinate contract and solver are retained, not re-derived or physically validated by this addition.

## Cross-checks

Automated geometry checks compare an axis-aligned case with hand-calculated bounds, check that the edge midpoint is `C` and separation is `W`, and independently sample 32768 circumference points at the two edges of a rotated band. Those samples must lie inside the analytic bounds; sampled extrema agree with them within 0.00001 mm in the synthetic fixture. Reflection across Y = 0, immutable source vectors, unknown widths, invalid inputs, and normalization of finite nonzero axes are checked separately.

Schema checks cover independent front/rear widths, malformed values, JSON round trips, unknown older imports, and preservation of source data. Kinematic regression checks compare complete solve results and hardpoints for unknown, 100 mm and 300 mm widths across all three topologies at static, compression/steering and rebound inputs. Browser checks exercise the editor and rendered preview. These are software/numerical checks of the stated construction; they are not experimental evidence of a real tire profile or fit.
