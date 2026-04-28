#!/bin/bash
INSTALL_DIR=${1:-"../install"}
BUILD_TYPE=${2:-"Release"}
NO_SUBMODULE_SYNC=0
for arg in "$@"; do
    if [ "$arg" = "--no-submodule-sync" ]; then NO_SUBMODULE_SYNC=1; fi
done

# Resolve INSTALL_DIR to an absolute per-package path. URLab.Build.cs expects
# headers/libs/dlls under install/<dep>/, matching the .ps1 layout.
INSTALL_DIR="$(cd "$(dirname "$INSTALL_DIR")" && pwd)/$(basename "$INSTALL_DIR")/MuJoCo"

# Wipe any prior install of THIS package only - cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing previous install at $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi

# Sync MuJoCo submodule to the SHA URLab expects. Unconditional by design:
# after a `git pull` the submodule pointer may have moved, and building
# against stale source produces a mismatched install that URLab.Build.cs
# then rejects. Pass --no-submodule-sync to keep local edits in src/.
REPO_DIR="src"
if [ "$NO_SUBMODULE_SYNC" = "1" ]; then
    echo "Skipping submodule sync (--no-submodule-sync). Using whatever is checked out in $REPO_DIR."
    if [ ! -f "$REPO_DIR/CMakeLists.txt" ]; then
        echo "MuJoCo submodule not initialised at $REPO_DIR but --no-submodule-sync was set. Drop the flag or run 'git submodule update --init --recursive -- $REPO_DIR' manually." >&2
        exit 1
    fi
else
    echo "Syncing MuJoCo submodule to URLab's pinned commit..."
    # --force is required because CoACD's build dirties its submodule working
    # tree (custom CMakeLists overlay) and sibling deps use the same pattern
    # for consistency. Users iterating on submodule source should use
    # --no-submodule-sync instead of relying on working-tree preservation.
    git submodule update --init --recursive --force -- "$REPO_DIR"
    if [ $? -ne 0 ]; then
        echo "git submodule update failed for MuJoCo - is this a git checkout? Pass --no-submodule-sync to skip." >&2
        exit 1
    fi
fi

# Capture the SHA we are about to build from so URLab.Build.cs can detect a
# stale install on the next UE build.
INSTALLED_SHA=$(git -C "$REPO_DIR" rev-parse HEAD)
if [ -z "$INSTALLED_SHA" ]; then
    echo "Failed to read HEAD SHA from $REPO_DIR - is the submodule initialised?" >&2
    exit 1
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

# Record the exact source SHA we just installed from (see MuJoCo/build.ps1).
echo "$INSTALLED_SHA" > "$INSTALL_DIR/INSTALLED_SHA.txt"
echo "Recorded INSTALLED_SHA=$INSTALLED_SHA at $INSTALL_DIR/INSTALLED_SHA.txt"

# Linux: re-stage the plugin's Binaries/Linux/ symlinks so a per-dep rebuild
# (e.g. someone bumps MuJoCo's submodule SHA and runs only this script)
# doesn't leave stale symlinks pointing into a wiped install/MuJoCo/lib/.
# The helper exits cleanly with a one-line note if Binaries/Linux/ doesn't
# exist yet (first-time fresh checkout — plugin .so not built).
if [ "$(uname -s)" = "Linux" ]; then
    SETUP_RUNTIME="$(cd "$(dirname "$0")/../../Scripts" 2>/dev/null && pwd)/setup_runtime_linux.sh"
    if [ -x "$SETUP_RUNTIME" ]; then
        bash "$SETUP_RUNTIME" || echo "WARN: setup_runtime_linux.sh failed (continuing)" >&2
    fi
fi
