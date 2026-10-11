#!/usr/bin/env bash
# Regenerate the protobuf/gRPC code on BOTH sides from the one set of .proto
# files, using the toolchains pinned in wire/toolchain.lock.
#
# Before this script existed there was no regeneration path at all on either
# side: the generated code was checked in and the commands lived in a docstring
# and a setup doc. That is why the two sides drifted to protobuf 4 and 7.
#
#   ./wire/regen_proto.sh            # regenerate both, then verify
#   ./wire/regen_proto.sh --check    # verify only, generate nothing
#
# The Python side needs grpcio-tools at the pinned version; the C++ side needs
# the protoc and grpc_cpp_plugin built by third_party/grpc/build.sh. Either half
# is skipped with a clear message when its toolchain is absent, so the --check
# path works on a machine with neither.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGIN="$(dirname "$HERE")"
BRIDGE="$(dirname "$PLUGIN")/URLab_Bridge"
PROTO_DIR="$PLUGIN/Source/URLabDmEnvRpc/Private/Proto"
GEN_CPP="$PLUGIN/Source/URLabDmEnvRpc/Private/GenProto"
GEN_PY="$BRIDGE/src/urlab_client/transports/_dmenv"

lock() { sed -n "s/^[[:space:]]*$1[[:space:]]*=[[:space:]]*//p" "$HERE/toolchain.lock" | head -1 | tr -d '[:space:]'; }
PY_GRPCIO_TOOLS="$(lock py.grpcio_tools)"
UE_PROTOBUF="$(lock ue.protobuf)"

CHECK_ONLY=0
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=1

if [[ $CHECK_ONLY -eq 0 ]]; then
  # ---- Python ---------------------------------------------------------------
  PY="${PYTHON:-$BRIDGE/.venv/bin/python}"
  if [[ -x "$PY" ]] && "$PY" -c "import grpc_tools" 2>/dev/null; then
    have="$("$PY" -c 'import grpc_tools,importlib.metadata as m;print(m.version("grpcio-tools"))' 2>/dev/null || echo unknown)"
    if [[ "$have" != "$PY_GRPCIO_TOOLS" ]]; then
      echo "WARNING: grpcio-tools $have installed, toolchain.lock pins $PY_GRPCIO_TOOLS."
      echo "         Generating anyway will move the Python side off the pin."
      read -r -p "         Continue? [y/N] " reply
      [[ "$reply" == "y" || "$reply" == "Y" ]] || { echo "aborted"; exit 1; }
    fi
    echo "==> regenerating Python stubs into $GEN_PY"
    "$PY" -m grpc_tools.protoc \
      -I "$PROTO_DIR" \
      --python_out="$GEN_PY" \
      --grpc_python_out="$GEN_PY" \
      "$PROTO_DIR/urlab_dm_env_rpc.proto" "$PROTO_DIR/dm_env_rpc.proto"
    # generated imports are absolute; the package vendors them, so make them relative
    sed -i 's/^import dm_env_rpc_pb2/from . import dm_env_rpc_pb2/' "$GEN_PY"/*_pb2_grpc.py 2>/dev/null || true
    sed -i 's/^import dm_env_rpc_pb2/from . import dm_env_rpc_pb2/' "$GEN_PY"/*_pb2.py 2>/dev/null || true
  else
    echo "SKIP python: grpcio-tools not importable (set PYTHON= or pip install grpcio-tools==$PY_GRPCIO_TOOLS)"
  fi

  # ---- C++ ------------------------------------------------------------------
  PROTOC="${PROTOC:-$PLUGIN/third_party/install/bin/protoc}"
  PLUGIN_BIN="${GRPC_CPP_PLUGIN:-$PLUGIN/third_party/install/bin/grpc_cpp_plugin}"
  if [[ -x "$PROTOC" && -x "$PLUGIN_BIN" ]]; then
    have="$("$PROTOC" --version | awk '{print $2}')"
    [[ "$have" == "$UE_PROTOBUF" ]] || echo "WARNING: protoc $have, toolchain.lock pins $UE_PROTOBUF."
    echo "==> regenerating C++ stubs into $GEN_CPP"
    "$PROTOC" -I "$PROTO_DIR" \
      --cpp_out="$GEN_CPP" \
      --grpc_out="$GEN_CPP" --plugin=protoc-gen-grpc="$PLUGIN_BIN" \
      "$PROTO_DIR/urlab_dm_env_rpc.proto" "$PROTO_DIR/dm_env_rpc.proto" \
      "$PROTO_DIR/google/rpc/status.proto"
  else
    echo "SKIP c++: protoc / grpc_cpp_plugin not built (see third_party/grpc/build.sh)"
  fi
fi

# ---- Verify -----------------------------------------------------------------
echo
echo "==> verifying descriptors agree across the two repos"
python3 "$HERE/check_descriptors.py"
