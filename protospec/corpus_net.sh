#!/usr/bin/env bash
# The corpus net: build the round-trip differential's tools and run it.
# The twin of corpus_net.ps1; see it for the rationale. The verdict is shared:
# both scripts end in tools/corpus_net.py.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MUJOCO_ROOT="${1:-$SCRIPT_DIR/../third_party/install/MuJoCo}"
CORPUS="${2:-$SCRIPT_DIR/../third_party/MuJoCo/src}"
BUILD_TYPE="${3:-Release}"

if [ ! -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ]; then
    echo "No MuJoCo headers under $MUJOCO_ROOT. Run third_party/build_all.sh first, or pass it as the first argument." >&2
    exit 2
fi
MUJOCO_ROOT="$(cd "$MUJOCO_ROOT" && pwd)"

if [ ! -d "$CORPUS" ]; then
    echo "No MuJoCo corpus at $CORPUS. Pass it as the second argument." >&2
    exit 2
fi
CORPUS="$(cd "$CORPUS" && pwd)"

SRC="$SCRIPT_DIR/lib"
BUILD="$SRC/build-urlab"

# The same build directory and the same configure line as build.sh, so the two
# scripts share a cache instead of invalidating each other's.
echo "Configuring ProtoSpec from $SRC..."
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DMUJOCO_ROOT="$MUJOCO_ROOT"
if [ $? -ne 0 ]; then echo "CMake configuration failed for ProtoSpec" >&2; exit 2; fi

echo "Building the differential tools..."
cmake --build "$BUILD" --config "$BUILD_TYPE" --target ps_roundtrip mj_model_diff
if [ $? -ne 0 ]; then echo "Build failed for ps_roundtrip / mj_model_diff" >&2; exit 2; fi

# The harness CMake copies the MuJoCo runtime next to mj_model_diff; an ELF
# loader does not look there on its own, so say so. Both the build tree's copy
# and the install's lib/ are named, because a single-config generator and a
# multi-config one put the executable in different places.
DIFF_DIR="$(dirname "$(find "$BUILD" -name mj_model_diff -type f -print -quit)")"
export LD_LIBRARY_PATH="${DIFF_DIR}:${MUJOCO_ROOT}/lib:${LD_LIBRARY_PATH:-}"

echo "Running the corpus net over $CORPUS..."
export PROTOSPEC_CORPUS="$CORPUS"
cd "$SCRIPT_DIR" || exit 2
uv run python tools/corpus_net.py
CODE=$?

if [ $CODE -ne 0 ]; then
    echo "Corpus net FAILED (exit $CODE)" >&2
else
    echo "Corpus net passed"
fi
exit $CODE
