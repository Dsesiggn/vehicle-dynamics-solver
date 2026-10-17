"""Same-origin local/web server. Only selected frontend assets are public."""
from pathlib import Path
import sys

import numpy
import scipy
from fastapi import FastAPI, HTTPException
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from .models.ackermann import calculate_ackermann
from .schemas import AckermannRequest, AckermannResponse

PROJECT_ROOT = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[2]))
FRONTEND = PROJECT_ROOT / "suspension-studio"


def create_app(frontend: Path = FRONTEND) -> FastAPI:
    app = FastAPI(
        title="Vehicle Dynamics Engineering API", version="0.1.0",
        docs_url=None, redoc_url=None, openapi_url="/api/v1/openapi.json",
        description="Ideal zero-slip Ackermann benchmark. Existing linkage solvers remain in Studio.",
    )

    @app.exception_handler(RequestValidationError)
    async def validation_error(_request, error):
        # Omit echoed input: NaN/Infinity and arbitrary malformed payloads are not JSON-safe.
        return JSONResponse(status_code=422, content={"detail": [
            {"loc": list(item["loc"]), "msg": item["msg"], "type": item["type"]}
            for item in error.errors()
        ]})

    @app.get("/api/v1/health")
    def health():
        return {"status": "ok", "models": ["ideal_ackermann_v1"],
                "numpy": numpy.__version__, "scipy": scipy.__version__}

    @app.post("/api/v1/ackermann", response_model=AckermannResponse)
    def ackermann(request: AckermannRequest):
        samples = calculate_ackermann(request.wheelbase_mm, request.front_track_mm,
                                      request.inside_angles_deg)
        return AckermannResponse(wheelbase_mm=request.wheelbase_mm,
                                 front_track_mm=request.front_track_mm, samples=samples)

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

    app.mount("/suspension-studio/src", StaticFiles(directory=frontend / "src"), name="source")
    app.mount("/suspension-studio/vendor", StaticFiles(directory=frontend / "vendor"), name="vendor")
    return app


app = create_app()
