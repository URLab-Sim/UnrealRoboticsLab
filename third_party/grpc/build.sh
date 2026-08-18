#!/bin/bash
# Master build script for gRPC and Protobuf for URLab (Linux/macOS)
set -e

INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}

INSTALL_ROOT="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")"
INSTALL_DIR="$INSTALL_ROOT/grpc"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$SCRIPT_DIR/src"

if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

if [ ! -d "$SRC" ]; then
    echo "Cloning gRPC v1.62.0..."
    git clone --depth 1 --branch v1.62.0 --recurse-submodules --shallow-submodules https://github.com/grpc/grpc.git "$SRC"
fi

SRC="$(cd "$SRC" && pwd)"
BUILD="$SRC/build"
mkdir -p "$BUILD"
BUILD="$(cd "$BUILD" && pwd)"
cd "$BUILD"

echo "Configuring gRPC..."
cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
         -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--undefined-version" \
         -DCMAKE_EXE_LINKER_FLAGS="-Wl,--undefined-version -lc++abi" \
         -DZLIB_BUILD_SHARED=OFF \
         -DBUILD_SHARED_LIBS=OFF \
         -DgRPC_BUILD_TESTS=OFF \
         -DRE2_BUILD_TESTING=OFF \
         -Dprotobuf_BUILD_TESTS=OFF \
         -DABSL_BUILD_TESTING=OFF \
         -Dutf8_range_ENABLE_TESTS=OFF \
         -DCARES_BUILD_TESTS=OFF \
         -DgRPC_BUILD_CSHARP_EXT=OFF \
         -DgRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF \
         -DgRPC_BUILD_GRPC_NODE_PLUGIN=OFF \
         -DgRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN=OFF \
         -DgRPC_BUILD_GRPC_PHP_PLUGIN=OFF \
         -DgRPC_BUILD_GRPC_PYTHON_PLUGIN=OFF \
         -DgRPC_BUILD_GRPC_RUBY_PLUGIN=OFF \
         -DgRPC_INSTALL=OFF \
         -Dprotobuf_INSTALL=OFF \
         -Dutf8_range_ENABLE_INSTALL=OFF \
         -DABSL_ENABLE_INSTALL=OFF

echo "Building gRPC and Protobuf static libraries..."
cmake --build . --config "$BUILD_TYPE" --target grpc++ grpc gpr libprotobuf -j $(nproc)

echo "Staging gRPC and Protobuf into $INSTALL_DIR..."
mkdir -p "$INSTALL_DIR/include" "$INSTALL_DIR/lib"

# Copy headers
cp -r "$SRC/include/grpc" "$SRC/include/grpcpp" "$INSTALL_DIR/include/"
cp -r "$SRC/third_party/protobuf/src/google" "$INSTALL_DIR/include/"
cp -r "$SRC/third_party/abseil-cpp/absl" "$INSTALL_DIR/include/"
cp -r "$SRC/third_party/re2/re2" "$INSTALL_DIR/include/" 2>/dev/null || true
cp -r "$SRC/third_party/boringssl-with-bazel/src/include/openssl" "$INSTALL_DIR/include/" 2>/dev/null || true

# Copy all static libraries
find "$BUILD" -name "*.a" -exec cp -f {} "$INSTALL_DIR/lib/" \;

echo "gRPC and Protobuf successfully installed to $INSTALL_DIR:"
ls -la "$INSTALL_DIR/lib"
