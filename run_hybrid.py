#!/usr/bin/env python3
"""Launch the shared local service without downloading anything at startup."""
import argparse
from pathlib import Path
import sys
import threading
import time
import webbrowser

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "backend"))


def main():
    parser = argparse.ArgumentParser(description="Run Vehicle Dynamics locally or on a web host.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--open", action="store_true", help="Open a browser after startup")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    try:
        import uvicorn
        from vehicle_dynamics.app import app
    except ImportError as exc:
        parser.exit(1, f"Missing Python dependency: {exc}\nRun python3 setup_hybrid.py once, then use Start Hybrid.command.\n")
    server = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, log_level="info"))
    address = f"http://127.0.0.1:{args.port}/"

    def open_when_ready():
        # Only open for this process after its server binds, not an unrelated occupied port.
        for _ in range(200):
            if server.should_exit:
                return
            if server.started:
                webbrowser.open(address)
                return
            time.sleep(0.05)

    if args.open and args.host in {"127.0.0.1", "0.0.0.0", "localhost"}:
        threading.Thread(target=open_when_ready, daemon=True).start()
    try:
        server.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
