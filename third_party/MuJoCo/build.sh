#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
PINNED_COMMIT="47264877"  # Pin to tested commit (MuJoCo 3.7.0 dev, polynomial stiffness/damping)

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
