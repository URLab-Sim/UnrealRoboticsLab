#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
PINNED_COMMIT="c7436bf"  # Pin to tested commit (CDT include path fix + unbundled 3rd party support)

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/dlls under install/<dep>/, matching the .ps1 layout.
INSTALL_DIR="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")/CoACD"

# Wipe any prior install of THIS package only — cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

REPO_DIR="src"
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning CoACD (pinned: $PINNED_COMMIT)..."
    git clone https://github.com/SarahWeiii/CoACD "$REPO_DIR"
    cd "$REPO_DIR"
    git checkout "$PINNED_COMMIT"
    cd ..
else
    echo "CoACD source already exists at '$REPO_DIR'. Skipping clone."
    echo "  To rebuild from scratch, delete '$REPO_DIR/' and re-run."
fi

cd "$REPO_DIR"

echo "Initializing submodules..."
git submodule update --init --recursive

echo "Applying custom CoACD configuration..."
cp -f ../../CoACD_custom/CMakeLists.txt .
if [ -d "../../CoACD_custom/cmake" ]; then
    mkdir -p cmake
    cp -rf ../../CoACD_custom/cmake/* cmake/
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
