# Vehicle Dynamics: hybrid architecture

Decision recorded 2026-10-16: adopt a shared local-first engineering frontend and Python
calculation service, with later desktop packaging and public-web deployment.

## Adopted direction

Develop locally first, using one HTML/CSS/JavaScript frontend and one Python engineering
engine. FastAPI connects them on the same origin. The engine uses NumPy, with SciPy
available for later numerical solvers. Three.js supplies new 3D views and Plotly.js supplies
new charts. Ship their assets locally so runtime does not require an internet connection.

Package a standalone desktop/offline application after the local workflow is stable;
deploy the same service and frontend to a public web host later. C++ and MATLAB remain
optional adapters behind the Python model boundary. No purchased API, API key, AI service,
or third-party calculation service is required. Hosting and optional proprietary MATLAB
components can have separate costs; no hosting budget or license entitlement is assumed.

## What was found and preserved

The active app is `suspension-studio/`, as directed by the existing `AGENTS.md`. It already
uses separate native JavaScript modules for models, solvers, persistence, and rendering.
Its current viewers and charts use Canvas 2D. All pre-existing project files are preserved.
The root `Dynamics_Dashboard.html`, MATLAB scripts, tire calibration data, archives,
existing Studio pages, source modules, schemas, tests, and reference notes are unchanged.

The legacy root dashboard still loads Tailwind and Plotly from CDNs. Its cold-start offline
support is **not** established by this change. Studio itself has local runtime assets.
The new page uses locally bundled Three.js 0.180.0 and Plotly.js 3.1.0, with license files.

## Implemented structure

```text
Vehicle Dynamics Simulation Tool/
  HYBRID_ARCHITECTURE.md       architectural decision and migration boundary
  Start Hybrid.command        macOS launcher after one-time setup
  setup_hybrid.py              isolated Python environment setup
  run_hybrid.py                localhost by default; optional web-host bind
  backend/
    requirements*.txt          pinned Python dependencies
    Dockerfile                optional web container recipe
    vehicle_dynamics/
      app.py                  HTTP routes and selected public assets
      schemas.py              strict inputs, explicit units and versioned outputs
      models/ackermann.py     pure NumPy ideal-Ackermann benchmark
    tests/                    numerical and API regression checks
  docs/
    HYBRID_QUICKSTART.md       run, test, packaging and deployment instructions
    ACKERMANN_BACKEND.md       derivation, evidence, assumptions and verification
    HYBRID_VERIFICATION.md     actual checks and remaining limitations
  suspension-studio/          preserved frontend in its current location
    engineering.html         additive Python-backed engineering workbench
    src/api-client.js         same-origin API transport
    src/engineering.*         new page controller and style
    vendor/                   pinned Three.js and Plotly.js files + licenses
```

FastAPI serves `/suspension-studio/` at the same path as before and `/api/v1/` beside it.
The root address opens the engineering workbench, with links to the existing suspension
editor and steering tool. Only the three HTML entry points, the Studio README, `src/`, `vendor/`, and four
explicitly selected existing method documents are public (STEERING, COORDINATES,
INTERFERENCE, TIRE-TREAD). The project root, Python code, other documents, tests, tire datasets, and MATLAB files are not
exposed by the new server. Existing static-hosted Studio use remains available.

## Calculation boundary

The first backend model is the existing **nominal ideal zero-slip Ackermann benchmark**.
The new page explicitly requests its outputs; its charts and schematic consume those
results. No silent JavaScript substitute is used if the backend is unavailable.

The existing fixed-axis rack linkage, suspension pose, and interference calculations
continue in JavaScript. Port them individually only after source evidence, independent
numerical checks, fixture parity, failure handling, and browser checks establish the
replacement's behavior. A Python endpoint is not evidence of better physical accuracy.
SciPy is installed and its version is reported by the health endpoint; the closed-form
Ackermann benchmark does not need an iterative SciPy solver.

Preserve the project's coordinate contract: SAE J670 Z-down, +X forward, +Y right,
+Z down; axle-local millimeters and reported degrees. Signed heading is positive right.
Keep suspension design schema v2 and steering design schema v1 distinct. Unknown user
inputs remain blank; demonstration geometry is explicitly selected and labeled.

Ideal Ackermann is a low-speed, zero-slip geometric target. It does **not** guarantee equal
tire slip angles at racing speeds or optimize front lateral force. Future tire and load
models need measured data, their own sign/unit adapters, and independent validation.
See `docs/ACKERMANN_BACKEND.md` and the existing `suspension-studio/docs/STEERING.md`.

## Local, desktop, and web stages

1. **Local service, implemented:** shared Python API, preserved editor, new benchmark page,
   local visualization assets, strict contracts, and regression checks. Initial dependency
   installation requires internet; starting and using the installed app does not.
2. **Solver migration, future:** establish source-backed equations and deterministic fixtures
   for each suspension/vehicle model, port one at a time, and compare residuals, signs,
   limits and branch/failure behavior before changing existing UI calls.
3. **Standalone desktop distribution, future:** bundle Python, native dependencies and
   frontend with PyInstaller; use a browser launcher first, then an optional desktop window.
   Build and test on each target OS. The supplied `.command` launcher requires the installed
   Python environment and is not a Python-free packaged application.
4. **Public hosting, future:** run the same FastAPI service behind HTTPS. The included
   Dockerfile is a starting recipe, not a tested deployment. Apply request/compute limits
   appropriate to later expensive solvers, decide access/data-retention policy, and verify
   the deployment. No external site or cloud resource is created by this change.
5. **Optional native adapters:** prefer a narrow C++ binding with an identical model
   contract and parity fixtures. MATLAB integration requires explicit runtime/licensing
   decisions and an adapter for units, signs and errors. Never accept arbitrary uploaded
   executable code through the public API.

## Saved designs and origins

The original static workflow uses `http://127.0.0.1:8766`. The hybrid launcher defaults to
`http://127.0.0.1:8767` so it can coexist with that running service. To reuse the original
origin instead, stop the static server and explicitly pass `--port 8766`. Browser storage belongs to its
browser and exact origin. `localhost`, another port, and a hosted URL have separate
libraries. Export/import design JSON to transfer data. Stop an existing static server
before starting the hybrid service on the same port. The new benchmark page does not
read or modify saved Studio designs. The backend has no database or user-data persistence.

## Technical sources

- [FastAPI static files](https://fastapi.tiangolo.com/tutorial/static-files/): mounting local assets.
- [FastAPI testing](https://fastapi.tiangolo.com/tutorial/testing/): in-process HTTP contract checks.
- [Plotly JavaScript setup](https://plotly.com/javascript/getting-started/): locally distributed scripts.
- [Three.js source](https://github.com/mrdoob/three.js/tree/r180): pinned local renderer release.

The supplied vehicle-dynamics reference evidence is retained in the existing project
documents. This architecture work reviewed those recorded passages and numerical
benchmarks; it does not claim a new inspection of the original PDFs or measured-vehicle validation.

## Legacy dashboard access

The preserved Studio navigation still contains its original Legacy dashboard link.
That root dashboard and its tire-calibration files are intentionally outside the new
service public allowlist; this link returns 404 under the hybrid server. Use the original
static-server workflow separately to open it. This does not change or remove those files.
Those legacy research/calibration artifacts are outside the hybrid service scope and
are not included in the web container.
