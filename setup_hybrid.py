#!/usr/bin/env python3
"""One-time explicit dependency setup, isolated from other Python projects."""
from pathlib import Path
import subprocess
import sys
import venv

root = Path(__file__).resolve().parent
environment = root / ".venv-hybrid"
if sys.version_info < (3, 11):
    raise SystemExit("Python 3.11 or newer is required; Python 3.12 or 3.14 is recommended.")
if not (environment / "pyvenv.cfg").exists():
    venv.EnvBuilder(with_pip=True).create(environment)
python = environment / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
subprocess.run([str(python), "-m", "pip", "install", "--no-cache-dir", "-r",
                str(root / "backend/requirements-dev.txt")], check=True)
print("Setup complete. Run Start Hybrid.command on macOS, or .venv-hybrid/bin/python run_hybrid.py --open.")
