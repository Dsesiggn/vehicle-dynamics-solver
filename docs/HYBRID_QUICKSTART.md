# Run the hybrid application

**Using a computer that cannot install Python?** Open the application on a Python
web host instead. [Browser-only deployment](WEB_DEPLOYMENT.md) includes the Render
template, GitHub update setup, and the checks required before calling a hosted URL
ready. The visiting computer only needs a compatible browser and network access.

From the **Vehicle Dynamics Simulation Tool** folder, one time with internet access:

```sh
python3 setup_hybrid.py
```

Python 3.11 or newer is required. Setup creates `.venv-hybrid` inside this project and
installs the pinned runtime and test dependencies. It does not change system Python.

Then double-click **Start Hybrid.command** on macOS, or run:

```sh
.venv-hybrid/bin/python run_hybrid.py --open
```

Open **http://127.0.0.1:8767/** if a browser does not open automatically. Keep the launcher
window open while using the app; Ctrl+C stops the server. It binds to this computer only.
The launcher never downloads dependencies. After setup, local operation needs no internet.
On Windows, use `.venv-hybrid\Scripts\python.exe run_hybrid.py --open` instead.

The new workbench opens with unknown dimensions blank. **Load reference example** selects
an explicitly synthetic/reference case. **Calculate reference** sends dimensions and signed
inside-wheel angles to Python; Plotly charts and a Three.js schematic show returned values.
The suspension editor and steering linkage tool are available through navigation links.
The steering page at `/suspension-studio/steering.html` now sends complete geometry to
Python automatically and returns all 5° samples, toe, arm angles and intersections.
Incomplete drafts remain editable. Pending/failed requests clear calculated results;
**Retry calculation** retries the current complete geometry. No local JS solver fallback is used.

The hybrid app defaults to port 8767 because the existing static server uses 8766.
If port 8767 is in use, stop the prior hybrid launcher or pass `--port 8768`. A different origin has
a different saved-design library; use export/import JSON if needed. Open the old static
server with its original address to recover saved data. Backend errors do not trigger a
fallback calculation. A missing WebGL capability may disable only the schematic.

## Using an existing Studio library

After cloning the repository, run the one-time setup above. The Python environment is
local to your machine and is not included in Git. Existing browser-saved designs on port
8766 stay at that origin. Export/import JSON to transfer them, or stop the old static
server and launch `run_hybrid.py --port 8766` to reuse its origin.

## Verification

From this project folder:

```sh
PYTHONPATH=backend .venv-hybrid/bin/python -m unittest discover -s backend/tests -v
node --test suspension-studio/tests/*.test.mjs
```

Use Node 20+ for the JavaScript suite. Set `NODE=/absolute/path/to/node` when Node is not
on PATH; the Python parity test uses it. Review `docs/HYBRID_VERIFICATION.md` for actual
results and any skipped checks. Passing software tests does not validate a real vehicle.

## API example

POST `/api/v1/ackermann` with JSON:

```json
{"wheelbase_mm":1600,"front_track_mm":1200,"inside_angles_deg":[-30,0,30]}
```

Inputs are finite JSON numbers; dimensions are between 0.000001 and 10,000,000 mm, inclusive.
Each signed inside angle must be strictly between -89 and +89 degrees; requests contain
1–721 angles. These are numerical/interface bounds, not recommended vehicle dimensions.
Positive heading means right turn. The response declares model, units and coordinate
system and returns signed ideal outside and left/right headings per sample.

GET `/api/v1/health` reports readiness and numerical library versions. The JSON contract
is `/api/v1/openapi.json`. CDN-backed Swagger/ReDoc pages are intentionally not enabled;
normal frontend use has no runtime CDN dependency.

## Later desktop packaging

The launcher uses the environment created by the one-time setup. A distributable
application is a separate release step. Use PyInstaller to bundle `run_hybrid.py`, the
`backend/vehicle_dynamics` package, the selected Studio HTML/src/vendor assets, NumPy and
SciPy. The server understands PyInstaller's extracted resource directory. Validate cold
start with networking disabled on a clean target machine before calling that build offline
or standalone. Add a desktop window wrapper only if it improves the workflow; no wrapper
or PyInstaller installation is required for the current local app.

## Web deployment

For the configured Render deployment, follow [WEB_DEPLOYMENT.md](WEB_DEPLOYMENT.md).
The same-origin service below also supports another Python-capable host.

For a host that runs Python, use the same code and installed requirements:

```sh
.venv-hybrid/bin/python run_hybrid.py --host 0.0.0.0 --port 8000
```

This explicitly exposes the service on the host network. A public deployment should place
it behind HTTPS and configure resource/request limits. No browser API base-URL change is
needed when serving at the host root: the client uses relative same-origin requests.
Subpath/proxy-root hosting needs an explicit path-prefix integration test before release.

Optional container, from this project root:

```sh
docker build -f backend/Dockerfile -t vehicle-dynamics .
docker run --rm -p 127.0.0.1:8000:8000 vehicle-dynamics
```

The recipe copies only the backend, selected frontend assets, and launcher. It has not
been built or deployed in this implementation. Static-only hosts can continue to serve
the suspension editor and steering draft controls, but cannot execute steering or the
Python-backed benchmark. See [STEERING_BACKEND.md](STEERING_BACKEND.md) for the linkage API.

The existing Studio **Legacy dashboard** link is outside this service and returns 404.
To use the unchanged original dashboard, start the original static workflow separately
from the project root: `python3 -m http.server 8768 --bind 127.0.0.1`, then open
`http://127.0.0.1:8768/Dynamics_Dashboard.html`. That legacy page still needs its CDN
libraries; offline verification applies to Studio and the new engineering workbench.
