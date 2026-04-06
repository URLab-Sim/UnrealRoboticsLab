#!/bin/bash
# Master Build Script for Synthy Third-Party Dependencies (Linux)

ROOT_DIR=$(pwd)
INSTALL_DIR="$ROOT_DIR/install"
BUILD_TYPE="Release"

# Ensure directories exist
mkdir -p "$INSTALL_DIR"

echo -e "\e[36mStarting Unified Build Process...\e[0m"

# 1. CoACD
echo -e "\n\e[33m--- Building CoACD ---\e[0m"
cd CoACD
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE"
else
    echo "Warning: CoACD/build.sh not found!"
fi
cd "$ROOT_DIR"

# 2. MuJoCo
echo -e "\n\e[33m--- Building MuJoCo ---\e[0m"
cd MuJoCo
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE"
else
    echo "Warning: MuJoCo/build.sh not found!"
fi
cd "$ROOT_DIR"

# 3. libzmq
echo -e "\n\e[33m--- Building libzmq ---\e[0m"
cd libzmq
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE"
else
    echo "Warning: libzmq/build.sh not found!"
fi
cd "$ROOT_DIR"

echo -e "\n\e[32mAll builds completed! check the 'install' folder.\e[0m"
