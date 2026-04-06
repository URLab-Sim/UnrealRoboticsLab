#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}

REPO_DIR="src"
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning MuJoCo..."
    git clone https://github.com/google-deepmind/mujoco "$REPO_DIR"
fi

cd "$REPO_DIR"
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
