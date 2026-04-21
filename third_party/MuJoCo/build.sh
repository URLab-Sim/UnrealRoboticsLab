#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
PINNED_COMMIT="47264877"  # Pin to tested commit (MuJoCo 3.7.0 dev, polynomial stiffness/damping)

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/dlls under install/<dep>/, matching the .ps1 layout.
INSTALL_DIR="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")/MuJoCo"

# Wipe any prior install of THIS package only. cmake --install is additive and
# leaves stale files behind across version bumps — e.g. the MuJoCo 3.7.0 bump
# statically linked obj_decoder/stl_decoder into mujoco.dll, but additive
# installs over a 3.6.x tree left the old plugin DLLs in bin/, silently
# breaking OBJ loading until they were manually removed.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

REPO_DIR="src"
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning MuJoCo (pinned: $PINNED_COMMIT)..."
    git clone https://github.com/google-deepmind/mujoco "$REPO_DIR"
    cd "$REPO_DIR"
    git checkout "$PINNED_COMMIT"
    cd ..
else
    echo "MuJoCo source already exists at '$REPO_DIR'. Skipping clone."
    echo "  To rebuild from scratch, delete '$REPO_DIR/' and re-run."
fi

cd "$REPO_DIR"

echo "Initializing submodules..."
git submodule update --init --recursive

mkdir -p build
cd build

echo "Configuring MuJoCo..."
cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
         -DMUJOCO_BUILD_EXAMPLES=OFF \
         -DMUJOCO_BUILD_TESTS=OFF \
         -DMUJOCO_BUILD_SIMULATE=OFF

echo "Building MuJoCo..."
cmake --build . --config "$BUILD_TYPE"

echo "Installing MuJoCo..."
cmake --install . --config "$BUILD_TYPE"

cd ../..
