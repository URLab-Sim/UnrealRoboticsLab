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
      --target protospec_core protospec_mjcf protospec_harness
if [ $? -ne 0 ]; then echo "Build failed for ProtoSpec" >&2; exit 1; fi

# The staged MuJoCo shared library uses UE's libc++ on Linux. Unlike static
# ProtoSpec archives, this standalone helper must link that ABI explicitly.
HELPER_BUILD="$SRC/build-urlab-deps"
HELPER_CACHE=()
MUJOCO_SOURCE="$SCRIPT_DIR/../third_party/MuJoCo/src"
if [ -f "$MUJOCO_SOURCE/build/CMakeCache.txt" ] &&
   grep -q -- '-stdlib=libc++' "$MUJOCO_SOURCE/build/CMakeCache.txt"; then
    CACHE_FILE="$HELPER_BUILD/ue-toolchain-cache.txt"
    mkdir -p "$HELPER_BUILD"
    python3 "$SCRIPT_DIR/tools/make_probe_cmake_cache.py" \
        --source "$MUJOCO_SOURCE" --mujoco "$MUJOCO_ROOT" \
        --source-cache "$MUJOCO_SOURCE/build/CMakeCache.txt" --out "$CACHE_FILE"
    if [ $? -ne 0 ]; then echo "Cannot configure standalone MuJoCo dependency helper" >&2; exit 1; fi
    # This is CMakeCache.txt syntax, not a -C initial-cache script.
    while IFS= read -r cache_row; do
        case "$cache_row" in
            CMAKE_CXX_COMPILER:*|CMAKE_CXX_FLAGS:*|CMAKE_EXE_LINKER_FLAGS:*)
                cache_key="${cache_row%%:*}"
                HELPER_CACHE+=("-D$cache_key=${cache_row#*=}")
                ;;
        esac
    done < "$CACHE_FILE"
fi
cmake -S "$SRC" -B "$HELPER_BUILD" "${HELPER_CACHE[@]}" \
      -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DMUJOCO_ROOT="$MUJOCO_ROOT"
if [ $? -ne 0 ]; then echo "Dependency-helper configuration failed" >&2; exit 1; fi
cmake --build "$HELPER_BUILD" --config "$BUILD_TYPE" --target urlab_mj_deps
if [ $? -ne 0 ]; then echo "Dependency-helper build failed" >&2; exit 1; fi
HELPER_EXE="$HELPER_BUILD/urlab_mj_deps"
if [ ! -x "$HELPER_EXE" ]; then echo "Dependency helper was not built at $HELPER_EXE" >&2; exit 1; fi

# Stage named narrow helper archives. The comparison helper is test-only and
# lives in a CMake subdirectory. A failed build leaves the prior install intact.
STAGED_LIBS=(protospec_core protospec_mjcf tinyxml2 protospec_harness)
FOUND_LIBS=()
for name in "${STAGED_LIBS[@]}"; do
    lib="$(find "$BUILD" -maxdepth 2 -name "lib$name.a" -print -quit)"
    if [ -z "$lib" ]; then
        echo "ProtoSpec did not build lib$name.a under $BUILD" >&2
        exit 1
    fi
    FOUND_LIBS+=("$lib")
done

echo "Staging ProtoSpec into $PROTOSPEC_INSTALL_DIR..."
rm -rf "$PROTOSPEC_INSTALL_DIR"
mkdir -p "$PROTOSPEC_INSTALL_DIR/lib" "$PROTOSPEC_INSTALL_DIR/third_party/tinyxml2"
mkdir -p "$PROTOSPEC_INSTALL_DIR/bin"

for dir in include core io harness; do
    [ -d "$SRC/$dir" ] || continue
    (cd "$SRC/$dir" && find . \( -name "*.h" -o -name "*.inc" \) -print0 |
        while IFS= read -r -d '' f; do
            mkdir -p "$PROTOSPEC_INSTALL_DIR/$dir/$(dirname "$f")"
            cp -f "$f" "$PROTOSPEC_INSTALL_DIR/$dir/$f"
        done)
done
cp -f "$SRC/third_party/tinyxml2/tinyxml2.h" "$PROTOSPEC_INSTALL_DIR/third_party/tinyxml2/"

for lib in "${FOUND_LIBS[@]}"; do cp -f "$lib" "$PROTOSPEC_INSTALL_DIR/lib/"; done
cp -f "$HELPER_EXE" "$PROTOSPEC_INSTALL_DIR/bin/"
echo "ProtoSpec staged: ${#FOUND_LIBS[@]} libraries, dependency helper, headers under $PROTOSPEC_INSTALL_DIR"
