#!/usr/bin/env bash
# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# setup_runtime_linux.sh — symlink URLab's third-party shared libraries into
# the plugin's Binaries/Linux/ directory so the editor's dynamic loader can
# resolve them via ${ORIGIN} (UBT already adds ${ORIGIN} to the plugin .so's
# RPATH).
#
# Why this is needed:
#   UBT's auto-computed relative RPATH for PublicAdditionalLibraries on Linux
#   assumes the plugin lives inside the host project's Plugins/ tree under a
#   path UBT can express relative to ${ORIGIN}. For plugins symlinked in from
#   outside the host project that calculation produces RPATH entries that
#   resolve to non-existent directories, so libmujoco / lib_coacd / libzmq
#   can't be found at editor startup.
#
# Run this after building both the third-party libs and the plugin .so. It is
# idempotent; existing symlinks are replaced.
#
# Usage: ./Scripts/setup_runtime_linux.sh
#        (run from the plugin root, or any cwd — paths are resolved relative
#        to this script's location)

set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
THIRD_PARTY="$PLUGIN_ROOT/third_party/install"
BIN_DIR="$PLUGIN_ROOT/Binaries/Linux"

if [[ ! -d "$THIRD_PARTY" ]]; then
    echo "ERROR: third_party/install not found at $THIRD_PARTY" >&2
    echo "Run third_party/build_all.sh first." >&2
    exit 1
fi

if [[ ! -d "$BIN_DIR" ]]; then
    echo "ERROR: plugin Binaries/Linux/ not found at $BIN_DIR" >&2
    echo "Build the plugin first (Build.sh URLabTestEditor Linux Development)." >&2
    exit 1
fi

count=0
for pkg_lib in "$THIRD_PARTY/MuJoCo/lib" "$THIRD_PARTY/CoACD/lib" "$THIRD_PARTY/libzmq/lib"; do
    [[ -d "$pkg_lib" ]] || continue
    for so in "$pkg_lib"/*.so*; do
        [[ -e "$so" ]] || continue
        target="$BIN_DIR/$(basename "$so")"
        ln -sfn "$so" "$target"
        count=$((count + 1))
    done
done

echo "Linked $count third-party shared libraries into $BIN_DIR"
