# Hybrid foundation verification — 2026-10-16

## Hosted deployment preparation — 2026-10-18

The intended client is an administrator-managed computer without a local Python
installation. A same-origin Python web service hosts the existing application and
steering API; no numerical solver, geometry schema, units or coordinate convention
changes are introduced by deployment.

- Downloaded the published commit `637af57a8d8e1a8afe97032506c70e8912cfc67f` as a
  GitHub source ZIP. All 91 audited application/backend/documentation files matched
  the working copy before this change. No virtual environment was in the archive.
- Ran the project's setup script in that separate extracted directory. The clean
  Python 3.14.5 environment installed the pinned dependencies successfully; its
  **53 Python tests and 179 Node tests passed without skips**.
- Added a Render Free Python-service Blueprint, linked to the actual complete-app
  branch, with health checks and automatic deployment after GitHub CI checks pass.
  Added a Linux CI workflow for the regression suites and real HTTP startup. These
  configuration files do not themselves create a hosting account or live service.
- With the deployment changes, **58 Python tests passed without skips**. Five new
  tests cover environment-only commit identity, explicit unknown revisions,
  cache headers on pages/API/errors, and fresh module responses even if old file
  validators match a new equal-size/equal-timestamp module. Vendor caching remains
  unchanged. These checks verify software behavior, not measured vehicle accuracy.
- In-app browser through the local FastAPI service on a fresh origin: suspension
  startup and navigation to steering; synthetic 73-row sweep; +180-degree selection;
  visible 3D geometry and no recorded console errors/warnings. The selected example
  retains +16.68/+18.65-degree left/right headings. This is local browser verification,
  not a remote deployment or a test on the user's managed computer.
- Hosted timeout/loading-page guidance now explains wakeup and retry without
  requiring Python installation on the visiting computer. Inputs and explicit
  failure handling remain unchanged; no fallback solver is introduced.

Subsequent hosted verification:

- [GitHub Actions run 39194452253](https://github.com/Dsesiggn/vehicle-dynamics-solver/actions/runs/39194452253)
  passed on Linux with Python 3.14.5 and Node 22: all **58 Python** and **179 Node**
  tests, no skips, plus the actual launcher/HTTP/version checks.
- Render created the **Free Python 3** service `vehicle-dynamics-studio`. Its first
  deployment succeeded; public HTTPS health reported exact source commit
  `f6c2494ed22c409d27ac4a61e174ffc3b0eff903` from `RENDER_GIT_COMMIT`. The selected branch
  is `codex/hybrid-engineering-backend`, health path `/api/v1/health`, and auto-deploy
  setting **After CI Checks Pass**.
- Public HTTPS browser checks in Chrome: suspension `studioState=ready`, same-origin
  steering navigation, 73-position synthetic sweep, +180-degree input, 3D scene,
  +16.68/+18.65-degree left/right headings and no recorded console warnings/errors.
- Public API comparison against the local Python service model covered two full
  73-row sweeps: the planar synthetic case (73 solved) and an inclined-linkage case
  (70 solved, three explicit infeasible rows). All response values and failure
  states agreed. Across 11,530 numeric comparisons the largest absolute difference
  was 9.55e-12, within the check's 1e-9 absolute / 1e-12 relative tolerances.

The live URLs and service dashboard are recorded in [WEB_DEPLOYMENT.md](WEB_DEPLOYMENT.md).
The managed computer's own network/WebGL policy and a full hosted sleep/wake cycle
have not been exercised. These are deployment/software checks, not physical validation.

## Spatial steering viewer and inclined-linkage checks — 2026-10-18

The Python fixed-axis model already used full XYZ geometry; its equations, schema and
API remain unchanged. The steering page now renders an actual Three.js scene from the
returned joints, wheel centers, spindle directions and constructions. The original 2D
construction remains explicitly selectable. Camera choices do not change solved results.

- **53 Python tests passed, no skips.** Three new independent tests prescribe rotations
  of a nonhorizontal arm and tie rod about a known inclined axis, independently recover
  the rack displacement, check wheel-center/spindle motion and rod closure, change only
  inner-joint height, and check the vertical-axis constant-height cancellation limit.
  The existing 463-case JavaScript parity and 73-row sweep checks still pass.
- **179 Node tests passed, no skips.** Eight new scene tests check the proper SAE-to-Three
  basis rotation, exact preservation of supplied XYZ data, spindle-normal wheel rings
  and camber signs, actual spatial beta versus projected alpha, static/pending state
  isolation, rear-datum semantics, distant intersections and nearly horizontal axes.
- Independent implementation review found no remaining actionable code defect. A direct
  camera check exercised isometric/top/front/side at 900×600, 300×600 and 600×200 canvas
  sizes; all selected geometry bounds stayed inside the clip volume. A contradictory
  coordinate-document statement was corrected: zero static alignment applies to the
  suspension solver, while steering accepts explicit static toe/camber inputs.
- In-app browser: inclined synthetic hardpoints; neutral and positive/negative input;
  isometric/top/front/side, keyboard orbit/pan/zoom and fit; full-vehicle/intersection fit;
  3D/2D switching with unchanged readings; out-of-travel static references; clearing an
  essential Z input; and recovery after restoring inputs. At +80° the synthetic left
  side reads heading +9.62°, camber −0.61°, alpha 97.01° and beta 94.03°.
- Light and dark rendering and 1280 px / 390 px layouts were inspected. Narrow layout
  has no horizontal page overflow. Label overlaps found during inspection were corrected.
  No console errors or warnings were recorded in the completed verification tab. Touch
  gestures, physical-device interaction and other browser engines were not exercised.

The derivation and existing supplied-reference evidence in `suspension-studio/docs/STEERING.md`
were reviewed. No new original-PDF inspection or measured-vehicle validation is claimed.
Joint articulation limits, real arm/tire contours, collisions, compliance, slip, suspension
travel and chassis jacking remain outside the model. Rack translation remains along +Y;
nonhorizontal tie rods do not imply an arbitrary rack actuation direction. Software checks
establish consistency for the tested cases, not certainty for an unmeasured physical assembly.

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
