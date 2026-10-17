# Python steering linkage service

The Steering geometry page now sends complete steering designs to the same-origin
FastAPI service. Python owns the solved poses, 5° sweep, dynamic toe, nominal Ackermann
metrics, tie-rod/arm angles and arm-line intersections. JavaScript retains input editing,
draft storage, static reference drawing and rendering of returned results. The original
JavaScript solver remains as a numerical regression reference; it is not a runtime fallback.

## Governing model and evidence

The language port uses the existing first-principles derivation in
[STEERING.md](../suspension-studio/docs/STEERING.md). With fixed axis unit vector k,
the outer tie joint follows O(φ)=C+v cosφ+w sinφ, where C is its perpendicular axis foot,
v its neutral radius and w=k×v. Lateral rack displacement is q=g·θ/360, with θ in
steering-wheel degrees and signed g in millimeters per steering-wheel revolution.
Holding the assembled tie-rod length ℓ constant gives |O(φ)−I(q)|²=ℓ², or
a cosφ+b sinφ=c. Coefficients a,b,c have units mm². The solver follows the root
connected to neutral and tests the entire neutral-to-commanded path using the quartic
discriminant a²+b²−c², including its stationary points. It rejects toggles and inaccessible
paths rather than selecting a disconnected endpoint root. Rodrigues rotation carries
the wheel center and spindle with the same upright rotation.

The project uses front-axle-local millimeters, SAE +X forward, +Y right, +Z down.
Signed wheel heading is positive right; toe-in is heading_left and −heading_right.
The nominal zero-slip comparison is cot(δo)−cot(δi)=frontTrack/wheelbase, evaluated
with the nonsingular atan2 formulation for the ideal outside angle. Rear track draws
the rear reference layout and does not enter this front-angle identity.

Supplied-source evidence was reviewed in the recorded page-specific notes before the
port: Seward Chapter 6, pp.154–163; Milliken Chapter 19, pp.710–718 and Figure 19.3;
Smith Chapter 5, pp.60–61; Autek pp.11/13; Thompson pp.1–2; and Gillespie Chapter 8,
pp.276–280, with Chapter 6 pp.196–198 for zero-slip applicability. These sources support
the mechanism, reference geometry and distinction from tire-slip behavior. The exact
rigid-link algebra and numerical branch algorithm are project derivations, not attributed
to a book algorithm. Original PDFs were not newly inspected for this language port.
The source comparisons and unresolved radius/ratio notation differences remain in
[REFERENCES.md](../suspension-studio/docs/REFERENCES.md); the explicit project definitions
avoid depending on those ambiguities. This port adds no new physical approximation.

The fixed ride position, rigid links, spherical joints, lateral rack and constant g are
explicit modeling assumptions. Suspension travel, compliance, backlash, tire slip and
road-contact/chassis-jacking constraints remain outside this model. Mathematical consistency
within these assumptions does not establish measured-vehicle accuracy.

## API

Start the existing service using [HYBRID_QUICKSTART.md](HYBRID_QUICKSTART.md).

| Route | JSON request | Response |
| --- | --- | --- |
| `POST /api/v1/steering/solve` | `{"design": <complete v1 steering design>, "steeringWheelDeg": 30}` | One `result`, including `construction` |
| `POST /api/v1/steering/sweep` | `{"design": <complete v1 steering design>}` | `rows` from −steeringLimitDeg through +steeringLimitDeg, inclusive, `step_deg: 5` |
| `GET /api/v1/health` | None | Advertises `fixed_axis_steering_v1` alongside `ideal_ackermann_v1` |

Both calculation responses declare `model: fixed_axis_steering_v1`,
`coordinate_system: SAE_J670_Z_DOWN`, `hardpoint_frame: AXLE_LOCAL`,
`length_unit: mm`, and `angle_unit: deg`. Design JSON keeps its existing camelCase
field names and schema version. Design/side extension fields are accepted for saved-file
compatibility and have no effect on this model; unknown request-envelope fields are rejected.

Complete inputs must be finite JSON numbers without bool/string coercion. Null remains
an unknown draft value and is rejected for calculations. Coordinates, dimensions and gain
retain the existing magnitude bound of 1e7; positive dimensions and nonzero gain are
required. Static toe/camber magnitudes are below 90°. Sweep half-range must be a positive
multiple of 5° through 3600° (at most 1441 rows). A single steering input is bounded by
±3600° but need not be on the 5° grid. These are numerical/interface limits, not recommended
vehicle dimensions. All existing closure, degeneracy and discriminant tolerances remain
documented in STEERING.md and exposed in the pure model's numerical constants.

Malformed, incomplete or out-of-domain requests return HTTP 422. A well-formed but
unreachable pose returns HTTP 200 with `ok: false`, a reason, and null wheel/metric
outputs. Its construction is explicitly marked `pose: static`; it does not represent
the requested moving pose. Successful rows carry `pose: solved`. Parallel, coincident
and undefined arm-line intersections remain distinct from a finite intersection.
Responses never encode NaN or Infinity as a number. No request is stored in a backend database.

One deliberate boundary improvement over the reference JavaScript is explicit rejection
of unrepresentable calculated metrics. For example, a positive subnormal front track can
overflow the cotangent percentage despite a finite linkage pose. Python returns a failed
row naming the numerical-range problem, rather than allowing JSON serialization to turn
Infinity into an unexplained null. Defined geometric omissions (straight/mixed headings,
near-zero percentage, degenerate construction) retain their documented null/status behavior.

## Browser behavior

Edits invalidate the displayed sweep immediately. A short debounce batches typing;
superseded requests are aborted and late responses ignored. The slider uses the returned
5° samples directly, with no interpolation. Charts, tables, construction readouts and
CSV use the same returned samples. Pending and failed requests clear calculated values
and disable pose/CSV controls; draft save and JSON export remain available for valid
drafts, subject to the existing staged rack-offset rules. Retry explicitly requests a new sweep.

The rack longitudinal offset remains a layout edit of the neutral inner-joint X values.
It preserves their X separation and Y/Z values; Python recomputes the reference tie-rod
lengths from the edited assembly. It is not a fore/aft actuator with fixed rods.

A static HTTP server or GitHub Pages cannot run Python. The page shows startup guidance
when the API is absent and never substitutes JavaScript calculations. Run the Python
service and open the page on that service's origin. Reuse port 8766 after stopping the
old static server to retain its browser draft, or export/import JSON when changing ports.

## Verification

Run both suites from the repository root:

```sh
PYTHONPATH=backend NODE=/path/to/node .venv-hybrid/bin/python -m unittest discover -s backend/tests -v
node --test suspension-studio/tests/*.test.mjs
```

The added checks compare Python with the original JavaScript over complete sweeps and
adversarial geometries, and separately verify geometric invariants and known cases.
Transport checks cover request validation, units/metadata, unreachable rows, and public
asset boundaries. Client checks cover response validation and stale-request handling.
Actual results and browser checks are recorded in HYBRID_VERIFICATION.md after execution.
Cross-language agreement is not an independent proof of the governing physics; neither
it nor software tests replace measured steer-steer data from the actual assembly.

The implementation uses the existing pinned FastAPI/Pydantic stack; no new dependency
or external calculation service is added. See the official [FastAPI request-body
documentation](https://fastapi.tiangolo.com/tutorial/body/) and [Pydantic strict
validation documentation](https://pydantic.dev/docs/validation/latest/concepts/strict_mode/)
for the HTTP validation mechanism.
