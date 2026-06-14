#!/usr/bin/env bash
# Preview or build the URLab documentation site locally.
#
# Usage:
#   Scripts/docs.sh           # live preview at http://localhost:8000 (auto-reload)
#   Scripts/docs.sh build     # static strict build into ./site
#
# On first run this creates an isolated .venv-docs and installs the docs
# dependencies (mkdocs-material, matching CI). It is kept separate from any
# global mkdocs so a broken global install can't interfere.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv-docs"
PY="$VENV/bin/python"

if [[ ! -x "$PY" ]]; then
    echo "Creating docs venv at $VENV ..."
    python3 -m venv "$VENV"
    "$PY" -m pip install --quiet --upgrade pip
    "$PY" -m pip install --quiet -r "$ROOT/requirements-docs.txt"
fi

cd "$ROOT"
if [[ "${1:-serve}" == "build" ]]; then
    "$PY" -m mkdocs build --strict
else
    echo "Serving docs at http://localhost:8000 (Ctrl+C to stop)"
    "$PY" -m mkdocs serve
fi
