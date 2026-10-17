# Hybrid foundation verification — 2026-10-16

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
