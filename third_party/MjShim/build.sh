#!/bin/bash
set -e

INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}

# --no-submodule-sync is accepted and ignored: this package is URLab's own
# source, not a submodule. The master script passes one argument set to all deps.

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/shared objects under install/<dep>/, matching the .ps1 layout.
INSTALL_ROOT="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")"
MUJOCO_DIR="$INSTALL_ROOT/MuJoCo"
INSTALL_DIR="$INSTALL_ROOT/MjShim"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Wipe any prior install of THIS package only - cmake --install is additive
# and would otherwise leave stale files behind across a rename.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

if [ ! -f "$MUJOCO_DIR/include/mujoco/mujoco.h" ]; then
    echo "MuJoCo is not installed at $MUJOCO_DIR. Build it first (third_party/MuJoCo/build.sh); MjShim links against it." >&2
    exit 1
fi

cd "$SCRIPT_DIR"
mkdir -p build
cd build

echo "Configuring MjShim..."
cmake .. \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DMUJOCO_INSTALL_DIR="$MUJOCO_DIR"

echo "Building MjShim..."
cmake --build . --config "$BUILD_TYPE"

echo "Installing MjShim..."
cmake --install . --config "$BUILD_TYPE"

echo "MjShim installed to $INSTALL_DIR"
