#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
PINNED_COMMIT="7d95ac02"  # Pin to tested commit (v4.3.5+)

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/dlls under install/<dep>/, matching the .ps1 layout.
INSTALL_DIR="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")/libzmq"

# Wipe any prior install of THIS package only — cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

REPO_DIR="src"
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning libzmq (pinned: $PINNED_COMMIT)..."
    git clone https://github.com/zeromq/libzmq "$REPO_DIR"
    cd "$REPO_DIR"
    git checkout "$PINNED_COMMIT"
    cd ..
else
    echo "libzmq source already exists at '$REPO_DIR'. Skipping clone."
    echo "  To rebuild from scratch, delete '$REPO_DIR/' and re-run."
fi

cd "$REPO_DIR"

echo "Initializing submodules..."
git submodule update --init --recursive

mkdir -p build
cd build

echo "Configuring libzmq..."
cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
         -DZMQ_BUILD_TESTS=OFF \
         -DWITH_PERF_TOOL=OFF \
         -DENABLE_DRAFTS=OFF

echo "Building libzmq..."
cmake --build . --config "$BUILD_TYPE"

echo "Installing libzmq..."
cmake --install . --config "$BUILD_TYPE"

cd ../..
