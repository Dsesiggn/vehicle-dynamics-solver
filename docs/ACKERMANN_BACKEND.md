# Python ideal Ackermann reference

## Purpose and scope

This is a Python port of the existing `idealOutsideAngle` reference function in
[`suspension-studio/src/steering.js`](../suspension-studio/src/steering.js), with an
explicit signed-angle interface. It establishes a small, reviewable calculation
across the frontend/backend boundary. It does not replace the existing suspension
or fixed-axis rack-and-pinion linkage solvers. Those solvers continue to run in the
existing frontend.

The benchmark assumes planar rigid vehicle geometry, an unsteered rear axle,
equal front wheel-center X stations, and zero lateral tire slip. Under these
assumptions it is the exact geometric target for wheel headings. It is adequate
for verifying the new architecture because the same target is already implemented
and independently checked in the project. It is not a prediction of actual
linkage response, suspension movement, tire forces, compliance, or road contact.

## First-principles derivation, recorded before implementation

For zero lateral slip, each wheel's instantaneous velocity lies in its wheel
plane. Its perpendicular line therefore passes through the instantaneous center
of rotation. Straight rear wheels place that center on the rear-axle extension.
Let `L` be wheelbase in mm, `t` front wheel-center track in mm, and `R` the positive
lateral distance from the vehicle center plane to the turn center. For positive
inside/outside heading magnitudes `delta_i` and `delta_o`, the right triangles give

```text
tan(delta_i) = L / (R - t/2)
tan(delta_o) = L / (R + t/2)
cot(delta_o) - cot(delta_i) = t/L

delta_o = atan2(L*sin(delta_i), L*cos(delta_i) + t*sin(delta_i))
```

The final expression eliminates `R` and avoids cotangent division near straight
ahead. Both `atan2` arguments have units mm; dividing both by `L` gives the
dimensionless form used in code:

```text
delta_o = atan2(sin(delta_i), cos(delta_i) + (t/L)*sin(delta_i))
```

This removes an avoidable multiplication of two tiny values. Supported dimensions
bound `t/L` between `1e-13` and `1e13`, so the ratio remains representable.
Trigonometry uses radians internally and the API uses degrees. For angles whose
radian representation is subnormal, the outside angle retains the input
magnitude: even the largest supported `t/L` makes its relative geometric
correction less than `3e-295`, far below double-precision resolution. This also
avoids losing a representable degree input during radian conversion. At zero input
both headings are exactly zero. No radius is returned.

## Coordinates, signs and input contract

The project contract remains [SAE J670 Z-down, axle-local millimeters](../suspension-studio/docs/COORDINATES.md):
`+X` forward, `+Y` right, `+Z` down. Positive heading is a right turn about `+Z`.
For positive input, the right wheel is inside and the left is outside. For negative
input, the left wheel is inside; both returned headings are negative. The signed
outside angle uses the same sign as the supplied inside angle. A zero input is
reported as `straight`, with both headings zero.

`calculate_ackermann(wheelbase_mm, front_track_mm, inside_angles_deg)` accepts:

- Finite numeric wheelbase and front track, each at least `1e-6` mm and at most
  `1e7` mm. The upper bound follows the existing steering schema's numerical
  bound. The lower bound explicitly limits numerical scale; neither bound is a
  recommended vehicle dimension. Smaller positive dimensions are rejected.
- A list of 1 through 721 finite numeric inside angles, each strictly between
  `-89` and `89` degrees. This intentionally uses the existing linkage metrics'
  comparison range, a subset of the scalar helper's `[0, 90)` magnitude range.
- Numeric inputs without coercion: strings, booleans, nulls, NaN and infinities
  are rejected. Invalid input raises `ValueError` for the API layer to translate.

Each returned sample contains `inside_angle_deg`, `outside_angle_deg`,
`left_heading_deg`, `right_heading_deg`, and `turn_direction`. Values are ordinary
Python numbers/strings suitable for JSON; input order is preserved and input data
is not mutated. Negative zero is normalized to zero. No static toe subtraction,
steering ratio, default vehicle dimensions, or new Ackermann-percentage definition
is inferred. Rear track is not an input because it does not enter this relation.

## Supplied-reference evidence

This port relies on the project's existing recorded source review in
[`STEERING.md`](../suspension-studio/docs/STEERING.md#supplied-reference-evidence-and-discrepancies).
The original PDFs were not newly inspected for this architectural port. That
document records visually inspected passages, source-to-project sign conversions,
and the independent geometric derivation; its evidence is more than a catalog entry.

- Seward, *Race Car Design*, Chapter 6, especially Table 6.2, printed p.163 /
  supplied PDF p.174: the tabulated ideal inside angles and front-wheel path
  radii for `L=1600 mm`, `t=1200 mm` provide a numerical reference.
- Smith, *Tune to Win*, Chapter 5, printed pp.60–61 / supplied PDF pp.59–60:
  the zero-slip turn center and limitations of steering-arm convergence.
- Milliken & Milliken, *Race Car Vehicle Dynamics*, §5.3 printed pp.128–129 /
  supplied PDF p.83 and §19.2 printed pp.713–716 / supplied PDF pp.376–378:
  ideal geometry and practical linkage approximation.

The existing review records two discrepancies that this port does not conceal:
Thompson's Figure 1 omits arctangent in its angle expressions, and Milliken's
approximate rack-ratio formula uses a reciprocal sense relative to its earlier
ratio definition. This port uses neither expression. The signed adaptation is
the project's convention; the source diagrams use positive angle magnitudes.

## Verification and certainty

The standalone Python suite passed all **15 tests**, covering the five Seward
table rows using independent right-triangle geometry; independent turn-center
concurrence for both turn directions; left/right mirror symmetry; exact zero and
small angles; near-boundary behavior; dimension scaling; strict invalid inputs;
sample ordering and JSON compatibility; and parity with the unchanged JavaScript
reference. It also checks both dimension bounds, the maximum supported `t/L=1e13`
against independent right triangles, and positive/negative tiny degree inputs
through the smallest representable positive float. The source-table outputs agree
within `1e-12` degrees, independent
wheel-normal intersections within `2e-10` mm for the chosen fixtures, and all
**728 Python/JavaScript comparisons** within `3e-13` degrees. These are test
tolerances, not measurement uncertainties or bounds on every possible input.

The dimensionless expression avoids underflow from multiplying a small normalized
wheelbase by a small angle. The reported underflow case `L=1e-200 mm`, `t=1e7 mm`,
`delta_i=1e-200 degrees` is explicitly rejected by a regression test under the
supported dimension contract. At the maximum supported ratio, the independent
right-triangle checks agree within `1e-26` degrees for the selected small and
large headings. Subnormal-radian inputs retain their degree values exactly.

Run the model tests from the project root with:

```sh
python -m unittest discover -s backend/tests -p 'test_ackermann.py' -v
```

Set `NODE` to a Node.js executable when Node is not on `PATH`; otherwise the
JavaScript parity test reports an explicit skip. It was run without skips during
this implementation. Full frontend regressions and browser/API integration are
separate checks performed by the enclosing architecture change.

The geometry is exact within the stated assumptions. Floating-point calculations
and tests cover a finite set of inputs, not a proof for every accepted value. At
extreme dimension ratios or subnormal angles, differences smaller than machine
resolution cannot be represented. No measured vehicle steering sweep has been
supplied; software correctness and the book-table comparison do not establish
real-vehicle accuracy. Resolving that uncertainty requires measured geometry and
left/right wheel angles versus steering input, with measurement uncertainties.
