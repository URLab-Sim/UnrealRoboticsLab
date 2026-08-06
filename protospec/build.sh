#!/usr/bin/env bash
# Build ProtoSpec's static libraries and stage them where URLab.Build.cs looks.
# The twin of build.ps1; see it for the rationale.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

INSTALL_DIR="${1:-$SCRIPT_DIR/../third_party/install}"
BUILD_TYPE="${2:-Release}"
MUJOCO_ROOT="${3:-$SCRIPT_DIR/../third_party/install/MuJoCo}"

mkdir -p "$INSTALL_DIR"
INSTALL_ROOT="$(cd "$INSTALL_DIR" && pwd)"
PROTOSPEC_INSTALL_DIR="$INSTALL_ROOT/protospec"

if [ ! -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ]; then
    echo "No MuJoCo headers under $MUJOCO_ROOT. Run third_party/build_all.sh first, or pass it as the third argument." >&2
    exit 1
fi
MUJOCO_ROOT="$(cd "$MUJOCO_ROOT" && pwd)"

SRC="$SCRIPT_DIR/lib"
if [ ! -f "$SRC/CMakeLists.txt" ]; then
    echo "No ProtoSpec sources at $SRC." >&2
    exit 1
fi
SRC="$(cd "$SRC" && pwd)"
BUILD="$SRC/build-urlab"

echo "Resolved install: $PROTOSPEC_INSTALL_DIR"
echo "Configuring ProtoSpec from $SRC..."
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DMUJOCO_ROOT="$MUJOCO_ROOT"
if [ $? -ne 0 ]; then echo "CMake configuration failed for ProtoSpec" >&2; exit 1; fi

echo "Building ProtoSpec..."
cmake --build "$BUILD" --config "$BUILD_TYPE" \
      --target protospec protospec_core protospec_io protospec_harness
if [ $? -ne 0 ]; then echo "Build failed for ProtoSpec" >&2; exit 1; fi

# Staging is explicit because ProtoSpec's CMake declares no install() rules. The
# lib/ header layout is mirrored rather than flattened: the umbrella headers
# reach the generated tables through relative paths that only resolve in the
# original shape. URLab.Build.cs adds each of these directories to the include
# path.
#
# The previous install is removed only once the build has produced libraries, so
# a failure leaves what was there rather than nothing. Depth 2 is what reaches
# the harness, which is its own CMake subdirectory and archives one level down.
if [ -z "$(find "$BUILD" -maxdepth 2 -name '*.a' -print -quit)" ]; then
    echo "ProtoSpec built no static libraries under $BUILD" >&2
    exit 1
fi

echo "Staging ProtoSpec into $PROTOSPEC_INSTALL_DIR..."
rm -rf "$PROTOSPEC_INSTALL_DIR"
mkdir -p "$PROTOSPEC_INSTALL_DIR/lib" "$PROTOSPEC_INSTALL_DIR/third_party/tinyxml2"

for dir in include sdk generated core io compile validate harness; do
    [ -d "$SRC/$dir" ] || continue
    (cd "$SRC/$dir" && find . \( -name "*.h" -o -name "*.inc" \) -print0 |
        while IFS= read -r -d '' f; do
            mkdir -p "$PROTOSPEC_INSTALL_DIR/$dir/$(dirname "$f")"
            cp -f "$f" "$PROTOSPEC_INSTALL_DIR/$dir/$f"
        done)
done
cp -f "$SRC/third_party/tinyxml2/tinyxml2.h" "$PROTOSPEC_INSTALL_DIR/third_party/tinyxml2/"

LIB_COUNT=$(find "$BUILD" -maxdepth 2 -name "*.a" -exec cp -f {} "$PROTOSPEC_INSTALL_DIR/lib/" \; -print | wc -l)
echo "ProtoSpec staged: $LIB_COUNT libraries, headers under $PROTOSPEC_INSTALL_DIR"
