#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}

REPO_DIR="src"
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning CoACD..."
    git clone https://github.com/SarahWeiii/CoACD "$REPO_DIR"
fi

cd "$REPO_DIR"
mkdir -p build
cd build

echo "Configuring CoACD..."
cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DWITH_3RD_PARTY_LIBS=ON

echo "Building CoACD..."
cmake --build . --config "$BUILD_TYPE"

echo "Installing CoACD..."
cmake --install . --config "$BUILD_TYPE"

cd ../..
