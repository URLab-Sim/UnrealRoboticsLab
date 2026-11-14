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

# build_and_test.sh — configure + build the standalone urlab_rcl_test harness
# against a user-installed ROS 2 Lyrical and run its selftest plus a cross-process
# `ros2 topic echo` check. Linux parity for build_and_test.ps1 (the primary
# Windows validation), plus the --libcxx clang toolchain smoke for gate A3.
#
# Assumes ROS 2 Lyrical is already installed by the user (apt into
# /opt/ros/lyrical, or a Pixi/Conda env). Sources the ROS environment
# (URLAB_ROS2_SETUP override, the apt default, or an already-active env such as
# `pixi shell`), then builds and tests. Installs nothing and writes nothing
# outside build/.
#
# Usage:
#   ./build_and_test.sh [--libcxx]
#
# --libcxx : build UrlabRclCore.cpp with clang + libc++ and link it against the
#            libstdc++-built rcl cluster (pre-validates the UE-Linux stdlib mix).
#
# Exit codes: 0 ok, 1 build/env failed, 2 tests failed, 3 bad args.

set -eu

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

LIBCXX=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --libcxx) LIBCXX=1; shift ;;
        -h|--help)
            echo "Usage: ./build_and_test.sh [--libcxx]" >&2
            exit 3 ;;
        *) echo "Unknown arg: $1" >&2; exit 3 ;;
    esac
done

DEFAULT_SETUP="/opt/ros/lyrical/setup.bash"
ROS_SETUP="${URLAB_ROS2_SETUP:-$DEFAULT_SETUP}"

if [[ -f "$ROS_SETUP" ]]; then
    echo ">>> Sourcing ROS 2 environment: $ROS_SETUP"
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
elif [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    # Already-active environment (e.g. inside `pixi shell`).
    echo ">>> Using already-active ROS 2 environment."
fi

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    cat >&2 <<EOF
No active ROS 2 environment (AMENT_PREFIX_PATH is empty).

Install ROS 2 Lyrical (see docs/ros_workspace_setup.md), then either:
  - apt install: source $DEFAULT_SETUP (the apt default), or
  - Pixi/Conda: run 'pixi shell' in your ROS project to activate it, or
  - set URLAB_ROS2_SETUP to your setup.bash / activation script.
EOF
    exit 3
fi

CMAKE_ARGS=(-B build -DCMAKE_BUILD_TYPE=Release)
if [[ "$LIBCXX" -eq 1 ]]; then
    echo ">>> Toolchain smoke: clang + libc++ (gate A3)"
    CMAKE_ARGS+=(-DURLAB_LIBCXX=ON)
fi

echo ">>> Configuring (cmake)..."
cmake "${CMAKE_ARGS[@]}"

echo ">>> Building (Release)..."
cmake --build build --config Release

EXE="build/urlab_rcl_test"
if [[ ! -x "$EXE" ]]; then
    echo "urlab_rcl_test not found at $EXE after the build." >&2
    exit 1
fi
echo ">>> Built: $EXE"

echo ">>> Running selftest..."
"$EXE" --selftest || { echo "selftest failed." >&2; exit 2; }

# Cross-process echo verify over real DDS. Publish long enough to outlast echo
# discovery on a cold ROS daemon (the publisher is killed as soon as echo
# returns). `ros2 topic echo --once` has no built-in timeout and blocks forever
# if it misses the publisher's window (or the topic name is wrong), so bound it
# with `timeout` -- the verify can never hang the script.
echo ">>> Cross-process verify (ros2 topic echo)..."
"$EXE" --publish 3000 &
PUB_PID=$!
trap 'kill "$PUB_PID" 2>/dev/null || true' EXIT
sleep 3
ECHO_OUT=$(timeout 30 ros2 topic echo --once /urlab_test/joint_states 2>&1 || true)
echo "$ECHO_OUT"
kill "$PUB_PID" 2>/dev/null || true
trap - EXIT

if echo "$ECHO_OUT" | grep -q 'joint_a' \
    && echo "$ECHO_OUT" | grep -q 'joint_b' \
    && echo "$ECHO_OUT" | grep -q 'joint_c'; then
    echo ">>> Cross-process verify OK: JointState names received."
else
    echo "Cross-process verify failed: expected joint names not seen in echo output." >&2
    exit 2
fi

echo ""
echo "=== urlab_rcl_test: ALL CHECKS PASSED (Linux) ==="
exit 0
