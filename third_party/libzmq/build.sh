#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}

REPO_DIR="src"
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning libzmq..."
    git clone https://github.com/zeromq/libzmq "$REPO_DIR"
fi

cd "$REPO_DIR"
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
