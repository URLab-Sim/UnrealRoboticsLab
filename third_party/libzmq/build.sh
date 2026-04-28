#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
NO_SUBMODULE_SYNC=0
for arg in "$@"; do
    if [ "$arg" = "--no-submodule-sync" ]; then NO_SUBMODULE_SYNC=1; fi
done

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/dlls under install/<dep>/, matching the .ps1 layout.
INSTALL_DIR="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")/libzmq"

# Wipe any prior install of THIS package only - cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

# Sync libzmq submodule to the SHA URLab expects (see MuJoCo/build.sh for
# rationale). Pass --no-submodule-sync to keep local edits in src/.
REPO_DIR="src"
if [ "$NO_SUBMODULE_SYNC" = "1" ]; then
    echo "Skipping submodule sync (--no-submodule-sync). Using whatever is checked out in $REPO_DIR."
    if [ ! -f "$REPO_DIR/CMakeLists.txt" ]; then
        echo "libzmq submodule not initialised at $REPO_DIR but --no-submodule-sync was set. Drop the flag or run 'git submodule update --init --recursive -- $REPO_DIR' manually." >&2
        exit 1
    fi
else
    echo "Syncing libzmq submodule to URLab's pinned commit..."
    # --force for parity with MuJoCo/CoACD (see MuJoCo/build.sh for rationale).
    git submodule update --init --recursive --force -- "$REPO_DIR"
    if [ $? -ne 0 ]; then
        echo "git submodule update failed for libzmq - is this a git checkout? Pass --no-submodule-sync to skip." >&2
        exit 1
    fi
fi

INSTALLED_SHA=$(git -C "$REPO_DIR" rev-parse HEAD)
if [ -z "$INSTALLED_SHA" ]; then
    echo "Failed to read HEAD SHA from $REPO_DIR - is the submodule initialised?" >&2
    exit 1
fi

cd "$REPO_DIR"

mkdir -p build
cd build

echo "Configuring libzmq..."
# BUILD_STATIC=OFF on Linux: the libzmq static archive's mailbox_safe.cpp
# pulls libc++ wait_until -> pthread_cond_clockwait, which UE's link
# sysroot doesn't resolve. The shared .so links its own deps internally
# and works fine, so just don't build the static lib on Linux.
LIBZMQ_STATIC_FLAG=""
case "$(uname -s)" in
    Linux) LIBZMQ_STATIC_FLAG="-DBUILD_STATIC=OFF" ;;
esac
cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
         -DZMQ_BUILD_TESTS=OFF \
         -DWITH_PERF_TOOL=OFF \
         -DENABLE_DRAFTS=OFF \
         $LIBZMQ_STATIC_FLAG

echo "Building libzmq..."
cmake --build . --config "$BUILD_TYPE"

echo "Installing libzmq..."
cmake --install . --config "$BUILD_TYPE"

cd ../..

# Record the exact source SHA we just installed from (see MuJoCo/build.ps1).
echo "$INSTALLED_SHA" > "$INSTALL_DIR/INSTALLED_SHA.txt"
echo "Recorded INSTALLED_SHA=$INSTALLED_SHA at $INSTALL_DIR/INSTALLED_SHA.txt"
