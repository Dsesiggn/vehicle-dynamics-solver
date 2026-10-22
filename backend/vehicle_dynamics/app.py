"""Same-origin local/web server. Only selected frontend assets are public."""
from pathlib import Path
import os
import re
import sys

import numpy
import scipy
from fastapi import FastAPI, HTTPException
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from .models.ackermann import calculate_ackermann
from .schemas import AckermannRequest, AckermannResponse
from .models.steering import solve_steering, sweep_steering
from .models.steering_geometry import steering_construction
from .steering_schemas import (
    SteeringSolveRequest, SteeringSolveResponse,
    SteeringSweepRequest, SteeringSweepResponse,
)

PROJECT_ROOT = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[2]))
FRONTEND = PROJECT_ROOT / "suspension-studio"


class CurrentSourceFiles(StaticFiles):
    """Application modules are mutable URLs, unlike the bundled vendor assets."""

    def is_not_modified(self, response_headers, request_headers):
        # Starlette's file ETag uses size and mtime, not file contents. Even a
        # deploy preserving both must return current code, not a stale 304.
        return False


def deployment_source():
    """Report only a full commit identity explicitly supplied by the host."""
    for name in ("RENDER_GIT_COMMIT", "APP_COMMIT_SHA"):
        value = os.environ.get(name, "").strip()
        if re.fullmatch(r"(?:[0-9a-fA-F]{40}|[0-9a-fA-F]{64})", value):
            return value.lower(), name
    return None, None


def create_app(frontend: Path = FRONTEND) -> FastAPI:
    source_commit, source_commit_source = deployment_source()
    app = FastAPI(
        title="Vehicle Dynamics Engineering API", version="0.2.0",
        docs_url=None, redoc_url=None, openapi_url="/api/v1/openapi.json",
        description="Fixed-axis rack steering kinematics and nominal ideal zero-slip Ackermann geometry.",
    )

    @app.middleware("http")
    async def current_application_responses(request, call_next):
        response = await call_next(request)
        path = request.url.path
        studio = path == "/suspension-studio" or path.startswith("/suspension-studio/")
        vendor = path == "/suspension-studio/vendor" or path.startswith("/suspension-studio/vendor/")
        if path == "/" or path.startswith("/api/") or (studio and not vendor):
            # Keep HTML, modules, methods, redirects and API/error responses
            # current on reload. Versioned bundled libraries retain normal caching.
            response.headers["Cache-Control"] = "no-store"
            for validator in ("etag", "last-modified"):
                if validator in response.headers:
                    del response.headers[validator]
        return response

    @app.exception_handler(RequestValidationError)
    async def validation_error(_request, error):
        # Omit echoed input: NaN/Infinity and arbitrary malformed payloads are not JSON-safe.
        return JSONResponse(status_code=422, content={"detail": [
            {"loc": list(item["loc"]), "msg": item["msg"], "type": item["type"]}
            for item in error.errors()
        ]})

    @app.get("/api/v1/health")
    def health():
        return {"status": "ok", "models": ["ideal_ackermann_v1", "fixed_axis_steering_v1"],
                "numpy": numpy.__version__, "scipy": scipy.__version__,
                "source_commit": source_commit, "source_commit_source": source_commit_source}

    @app.post("/api/v1/ackermann", response_model=AckermannResponse)
    def ackermann(request: AckermannRequest):
        samples = calculate_ackermann(request.wheelbase_mm, request.front_track_mm,
                                      request.inside_angles_deg)
        return AckermannResponse(wheelbase_mm=request.wheelbase_mm,
                                 front_track_mm=request.front_track_mm, samples=samples)

    def with_construction(design, result):
        return {**result, "construction": steering_construction(design, result)}

    @app.post("/api/v1/steering/solve", response_model=SteeringSolveResponse)
    def steering_pose(request: SteeringSolveRequest):
        design = request.design.model_dump()
        result = solve_steering(design, request.steeringWheelDeg)
        return SteeringSolveResponse(result=with_construction(design, result))

    @app.post("/api/v1/steering/sweep", response_model=SteeringSweepResponse)
    def steering_sweep(request: SteeringSweepRequest):
        design = request.design.model_dump()
        return SteeringSweepResponse(rows=[with_construction(design, result)
                                            for result in sweep_steering(design)])

    @app.get("/", include_in_schema=False)
    def home():
        return RedirectResponse("/suspension-studio/engineering.html")

    @app.get("/suspension-studio", include_in_schema=False)
    def studio_redirect():
        return RedirectResponse("/suspension-studio/")

    @app.get("/suspension-studio/", include_in_schema=False)
    def studio():
        return FileResponse(frontend / "index.html")

    @app.get("/suspension-studio/{page}", include_in_schema=False)
    def page_file(page: str):
        if page not in {"index.html", "steering.html", "engineering.html", "README.md"}:
            raise HTTPException(status_code=404)
        if page == "README.md":
            return FileResponse(frontend / page, media_type="text/plain; charset=utf-8")
        return FileResponse(frontend / page)

    @app.get("/suspension-studio/docs/{document}", include_in_schema=False)
    def method_document(document: str):
        if document not in {"STEERING.md", "COORDINATES.md", "INTERFERENCE.md", "TIRE-TREAD.md"}:
            raise HTTPException(status_code=404)
        return FileResponse(frontend / "docs" / document, media_type="text/plain; charset=utf-8")

    app.mount("/suspension-studio/src", CurrentSourceFiles(directory=frontend / "src"), name="source")
    app.mount("/suspension-studio/vendor", StaticFiles(directory=frontend / "vendor"), name="vendor")
    return app


app = create_app()
