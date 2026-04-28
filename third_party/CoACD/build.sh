#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
NO_SUBMODULE_SYNC=0
for arg in "$@"; do
    if [ "$arg" = "--no-submodule-sync" ]; then NO_SUBMODULE_SYNC=1; fi
done

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/dlls under install/<dep>/, matching the .ps1 layout.
INSTALL_DIR="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")/CoACD"

# Wipe any prior install of THIS package only - cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

# Sync CoACD submodule to the SHA URLab expects (see MuJoCo/build.sh for
# rationale). Pass --no-submodule-sync to keep local edits in src/.
REPO_DIR="src"
if [ "$NO_SUBMODULE_SYNC" = "1" ]; then
    echo "Skipping submodule sync (--no-submodule-sync). Using whatever is checked out in $REPO_DIR."
    if [ ! -f "$REPO_DIR/CMakeLists.txt" ]; then
        echo "CoACD submodule not initialised at $REPO_DIR but --no-submodule-sync was set. Drop the flag or run 'git submodule update --init --recursive -- $REPO_DIR' manually." >&2
        exit 1
    fi
else
    echo "Syncing CoACD submodule to URLab's pinned commit..."
    # --force is required: this script intentionally overlays
    # ../CoACD_custom/{CMakeLists.txt,cmake/*} onto src/ below, so the working
    # tree is always dirty across runs. Without --force, the next URLab-side
    # SHA bump would hit a checkout conflict here.
    git submodule update --init --recursive --force -- "$REPO_DIR"
    if [ $? -ne 0 ]; then
        echo "git submodule update failed for CoACD - is this a git checkout? Pass --no-submodule-sync to skip." >&2
        exit 1
    fi
fi

INSTALLED_SHA=$(git -C "$REPO_DIR" rev-parse HEAD)
if [ -z "$INSTALLED_SHA" ]; then
    echo "Failed to read HEAD SHA from $REPO_DIR - is the submodule initialised?" >&2
    exit 1
fi

cd "$REPO_DIR"

echo "Applying custom CoACD configuration..."
cp -f ../../CoACD_custom/CMakeLists.txt .
if [ -d "../../CoACD_custom/cmake" ]; then
    mkdir -p cmake
    cp -rf ../../CoACD_custom/cmake/* cmake/
fi
# Overlay header patches (e.g. public/coacd.h relaxes `#if _WIN32` to
# `#if defined(_WIN32)` so URLab's build does not trip clang's -Wundef under
# UE on Linux).
if [ -d "../../CoACD_custom/public" ]; then
    mkdir -p public
    cp -rf ../../CoACD_custom/public/* public/
fi

mkdir -p build
cd build

echo "Configuring CoACD..."
cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DWITH_3RD_PARTY_LIBS=ON

echo "Building CoACD..."
cmake --build . --config "$BUILD_TYPE" --target _coacd

echo "Installing CoACD..."
cmake --install . --config "$BUILD_TYPE"

cd ../..

# Record the exact source SHA we just installed from (see MuJoCo/build.ps1).
echo "$INSTALLED_SHA" > "$INSTALL_DIR/INSTALLED_SHA.txt"
echo "Recorded INSTALLED_SHA=$INSTALLED_SHA at $INSTALL_DIR/INSTALLED_SHA.txt"

# Linux: re-stage Binaries/Linux/ symlinks. See MuJoCo/build.sh for rationale.
if [ "$(uname -s)" = "Linux" ]; then
    SETUP_RUNTIME="$(cd "$(dirname "$0")/../../Scripts" 2>/dev/null && pwd)/setup_runtime_linux.sh"
    if [ -x "$SETUP_RUNTIME" ]; then
        bash "$SETUP_RUNTIME" || echo "WARN: setup_runtime_linux.sh failed (continuing)" >&2
    fi
fi
