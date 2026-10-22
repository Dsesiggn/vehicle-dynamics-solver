#!/bin/zsh
set -eu
cd -- "${0:A:h}"
if [[ ! -x .venv-hybrid/bin/python ]]; then
  print 'One-time setup is needed. In this folder run: python3 setup_hybrid.py'
  read '?Press Return to close.'
  exit 1
fi
exec .venv-hybrid/bin/python run_hybrid.py --open
