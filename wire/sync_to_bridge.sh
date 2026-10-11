#!/usr/bin/env bash
# Vendor the wire schema and the toolchain lock into URLab_Bridge, and record
# their digests so the bridge can detect a stale or edited copy on its own.
#
# The schema's single source of truth is this file's directory, in the plugin
# repo. The bridge carries a read-only copy plus `wire/SCHEMA.lock`; its test
# suite fails if the copy no longer matches the digest, which is what turns
# "someone edited one side" from a silent drift into a red test.
#
#   ./wire/sync_to_bridge.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGIN="$(dirname "$HERE")"
BRIDGE="$(dirname "$PLUGIN")/URLab_Bridge"
DEST="$BRIDGE/wire"

[[ -d "$BRIDGE" ]] || { echo "URLab_Bridge not found beside the plugin ($BRIDGE)"; exit 2; }
mkdir -p "$DEST"

for f in urlab_wire.yaml toolchain.lock request_envelope_vectors.json; do
  cp "$HERE/$f" "$DEST/$f"
  echo "vendored $f"
done

sha() { sha256sum "$1" | awk '{print $1}'; }

cat > "$DEST/SCHEMA.lock" <<EOF
# Digests of the vendored wire artefacts. Written by the plugin's
# wire/sync_to_bridge.sh; asserted by tests/test_wire_schema.py.
#
# A mismatch means the vendored copy was edited here, or the plugin's schema
# moved and this copy was not re-synced. Fix by re-running the sync script in
# the plugin repo, never by editing the copy.
source_repo   = UnrealRoboticsLab
source_path   = wire/urlab_wire.yaml
synced_at     = $(date -u +%Y-%m-%dT%H:%M:%SZ)
urlab_wire.yaml.sha256 = $(sha "$DEST/urlab_wire.yaml")
toolchain.lock.sha256  = $(sha "$DEST/toolchain.lock")
EOF

echo "wrote $DEST/SCHEMA.lock"
echo
python3 "$HERE/check_descriptors.py"
