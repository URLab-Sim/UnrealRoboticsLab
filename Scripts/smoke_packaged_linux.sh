#!/usr/bin/env bash
# Fast typed smoke for a freshly staged executable. Usage:
#   Scripts/smoke_packaged_linux.sh /abs/fresh/StagedBuilds/Linux/URLabTest.sh
# The full matrix is URLab_Bridge/tests/packaged/run_matrix.sh with the same path.
set -euo pipefail
BINARY="${1:-}"
[[ -n "$BINARY" ]] || { echo "usage: $0 /abs/path/to/URLabTest.sh" >&2; exit 2; }
[[ "$BINARY" = /* ]] || { echo "packaged launcher must be an absolute path: $BINARY" >&2; exit 2; }
[[ -x "$BINARY" ]] || { echo "packaged launcher is not executable: $BINARY" >&2; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$HERE/../.." && pwd)"
BRIDGE="$WORKSPACE/URLab_Bridge"
PY="${PYTHON:-$BRIDGE/.venv/bin/python}"
[[ -x "$PY" ]] || { echo "python not found: $PY" >&2; exit 2; }
export URLAB_RENDER_SERVER="$BINARY" URLAB_RENDER_TARGET=packaged URLAB_LIVE=1
export URLAB_CAMERA_MONITOR="${URLAB_CAMERA_MONITOR:-1}"

# One current typed ZMQ/network render cell gives the quickest proof that the
# packaged process boots, accepts an explicit owner/source, returns real pixels,
# omits the editor service, and rejects an editor operation.
"$PY" -m pytest -q -s \
  "$BRIDGE/tests/integration/test_typed_render_smoke.py" \
  -k 'zmq and network'

echo "SMOKE PASS: typed packaged ZMQ/network render and packaged boundary"
