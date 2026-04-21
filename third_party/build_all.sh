#!/bin/bash
# Master Build Script for URLab Third-Party Dependencies (Linux)
#
# Each dep's build.sh will sync its submodule (third_party/<dep>/src) to the
# SHA URLab expects before building. Pass --no-submodule-sync to skip the
# sync across all three deps, e.g. when iterating on a submodule locally.

ROOT_DIR=$(pwd)
INSTALL_DIR="$ROOT_DIR/install"
BUILD_TYPE="Release"

SHARED_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--no-submodule-sync" ]; then SHARED_ARGS+=("--no-submodule-sync"); fi
done

# Ensure directories exist
mkdir -p "$INSTALL_DIR"

echo -e "\e[36mStarting Unified Build Process...\e[0m"

# 1. CoACD
echo -e "\n\e[33m--- Building CoACD ---\e[0m"
cd CoACD
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE" "${SHARED_ARGS[@]}"
else
    echo "Warning: CoACD/build.sh not found!"
fi
cd "$ROOT_DIR"

# 2. MuJoCo
echo -e "\n\e[33m--- Building MuJoCo ---\e[0m"
cd MuJoCo
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE" "${SHARED_ARGS[@]}"
else
    echo "Warning: MuJoCo/build.sh not found!"
fi
cd "$ROOT_DIR"

# 3. libzmq
echo -e "\n\e[33m--- Building libzmq ---\e[0m"
cd libzmq
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE" "${SHARED_ARGS[@]}"
else
    echo "Warning: libzmq/build.sh not found!"
fi
cd "$ROOT_DIR"

echo -e "\n\e[32mAll builds completed! check the 'install' folder.\e[0m"
