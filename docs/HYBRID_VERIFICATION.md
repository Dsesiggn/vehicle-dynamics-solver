# Hybrid foundation verification — 2026-10-16

## Steering linkage port — 2026-10-16

The steering page now uses the Python service for its actual spatial rack linkage,
not only the ideal benchmark. All earlier foundation checks below are historical;
the port intentionally updates the steering frontend and shared service.

- **50 Python tests passed, no skips:** 20 existing benchmark/API tests plus 30 steering
  model/API tests. Command: `PYTHONPATH=backend NODE=/path/to/node .venv-hybrid/bin/python
  -m unittest discover -s backend/tests -v`.
- **171 Node tests passed, no skips:** the 160 existing tests plus 11 transport/controller
  tests. Command: `node --test suspension-studio/tests/*.test.mjs`.
- Steering parity covers **463 solve/construction cases** against JavaScript and a
  complete **73-row sweep**. Separate checks cover prescribed rotations, tie-length and
  rigid-body invariants, tilted axes, reflection, axis reversal, scale, static alignment,
  known plan/spatial angles, the recorded Seward table, whole-path unreachable gaps and
  tangencies, and the maximum 1441-row sweep. Matching implementations alone are not
  independent physical evidence.
- API checks distinguish malformed/incomplete inputs (422) from unreachable or numerically
  unresolved calculations (200 with `ok: false`). Booleans, numeric strings, nonfinite
  values, malformed vectors and incompatible metadata are rejected. Positive subnormal
  dimensions that overflow derived metrics explicitly fail instead of silently becoming
  null. Output metadata, construction pose, health/OpenAPI and extension compatibility pass.
- Client tests check model/units/frame, row order and completeness, finite pose/construction
  data, network/HTTP/timeout failures, 150 ms edit batching, cancellation and stale responses.
- In-app browser: blank state; 73-row synthetic example; neutral and ±180° inputs; rack
  offset staging/application while retaining selected input; incomplete-input clearing;
  saved draft/reload; actual service shutdown and retry recovery; and failure rows when
  rack travel is reduced to 10 mm. Failed-position construction is labeled static.
  CSV contents have 31 columns and 74 lines including the header; failed-position angle
  and construction columns are blank. A completed download was not asserted.
- Visual checks at 1280 px and 390 px confirm the Python status/help, editor, construction
  and results remain usable without horizontal page overflow. The known synthetic case
  at +180° displays 16.68° left and 18.65° right headings, with 89.09°/128.51° plan angles.
- Independent integration review found no actionable defect. For a synthetic gain of
  −1 mm/revolution with all 1441 positions solved, local TestClient requests took
  264–319 ms and returned about 2.72 MB. These are local measurements, not hosting guarantees.

The existing derivation and recorded supplied-reference evidence were reviewed before
porting. No new original-PDF inspection, tire-slip model, suspension migration or measured
steer-steer validation is claimed. The Python service was checked locally; publishing its
source to GitHub does not deploy a publicly running backend. Browser engine coverage
remains limited to the in-app browser.

## Completed checks

- Existing Studio baseline: **149/149 Node regression tests passed** before additions.
  Existing source, test, schema and HTML files were not edited.
- New Python suite: **20/20 tests passed**: 15 model tests and 5 API/asset-boundary tests.
  The parity test executed **728 comparisons** against the unchanged JavaScript helper,
  with a 3e-13 degree absolute tolerance and no skipped tests.
- Model checks include independent wheel-normal intersections, the recorded Seward
  Table 6.2 values, mirror signs, zero, small angles, scale invariance, monotonicity,
  dimension/angle boundaries, malformed types and extreme supported ratios.
- JSON/API checks include nonfinite input rejection, response model/units, sample limits,
  frontend and method-document availability, and rejecting access to Python, original
  root artifacts, unselected documentation, tests and traversal paths.
- New JavaScript modules passed Node syntax checks.
- In-app browser: initial unknown inputs, explicit reference selection, 241-sample sweep,
  +30°/−30° and zero readouts, Plotly chart, Three.js schematic, input clearing and invalid
  input recovery, lost-backend error with old results cleared, reload/recovery, light/dark
  appearance and desktop/narrow layouts. A desktop heading-spacing defect found visually
  was corrected. The 1600/1200 mm example shows 30.000° inside and 21.944° outside.
- The unchanged suspension editor and steering tool both initialize under FastAPI;
  their original navigation and input controls are present.
- Runtime assets for the new workbench are local files. The server log confirms requests
  for local Three.js module/core and Plotly bundles. Library versions, licenses and hashes
  are recorded in `suspension-studio/vendor/`.

## Interpretation and remaining work

The formula is exact within its stated planar, rigid, zero-slip geometric assumptions;
these finite software checks establish consistency over the tested cases. They do not
prove every input case or establish physical vehicle accuracy. Existing source-reference
evidence was reviewed in project documents; original PDFs were not re-inspected this turn.
No measured vehicle data, force validation, suspension port or tire model was added.

The Python model supports dimensions from 1e-6 to 1e7 mm and signed inside angles strictly
between −89° and +89°. These are explicit numerical bounds, not engineering recommendations.
The formula avoids the earlier underflow of multiplying tiny angle sines by normalized
wheelbase. Subnormal angles are covered by direct tests.

SciPy is available for future solvers; this closed-form benchmark uses NumPy. The shared
service and local browser assets support offline runtime after setup. An operating-system
network-disconnected cold start on a clean machine and other-browser compatibility have
not been tested. The legacy dashboard still uses CDN scripts and is not served by this API.

No standalone executable was built. No PyInstaller bundle, Docker build, public hosting,
TLS, C++ adapter or MATLAB integration was tested or deployed. Those remain documented
next stages using the same frontend/service boundary. The HTTP test client currently
emits a Starlette deprecation notice for HTTPX; the tests complete successfully.

## Installation verification

Installed 30 additive source/document/library files in the actual project and created
its isolated `.venv-hybrid` environment using the supplied setup script. All 20 Python
tests passed again in that fresh environment, including JavaScript parity with no skips.
The actual project service starts on 127.0.0.1:8767. The previous service on 8766 was
left running. SHA-256 checks confirm all 92 pre-existing project files are unchanged.
The generated Python environment and bytecode caches are not source deliverables in the
addition manifest; recreate the environment with setup_hybrid.py when needed.
