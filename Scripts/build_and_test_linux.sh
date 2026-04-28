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
#
# --- LEGAL DISCLAIMER ---
# UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
# endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
# trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
#
# This plugin incorporates third-party software: MuJoCo (Apache 2.0),
# CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

# Linux variant of build_and_test.sh. Same summary block format so the PR
# template is satisfied. Differs from the upstream script in:
#   - Uses the Linux Build.sh / UnrealEditor-Cmd binaries
#   - Builds the project's Editor target on the Linux platform
#   - Stages third-party shared libs into the plugin's Binaries/Linux/ via
#     setup_runtime_linux.sh after build (RuntimeDependencies aren't auto-
#     staged for editor builds on Linux).
#
# Usage:
#   ./Scripts/build_and_test_linux.sh \
#       --engine /home/ubuntu/UnrealEngine \
#       --project /home/ubuntu/URLabTest/URLabTest.uproject \
#       [--target CustomEditorTargetName] \
#       [--filter URLab] \
#       [--log /tmp/urlab_test.log]
#
# --target defaults to <ProjectName>Editor derived from the .uproject filename.
# Only override for projects that don't follow UE's naming convention.
#
# Exit codes: 0 ok, 1 build failed, 2 tests failed, 3 bad args.

set -eu

TARGET=""
FILTER="URLab"
LOG="/tmp/urlab_test.log"
ENGINE=""
PROJECT=""

usage() {
    cat >&2 <<'HELP'
build_and_test_linux.sh — build the project's Editor target and run the URLab automation suite (Linux).

Usage:
  ./Scripts/build_and_test_linux.sh \
      --engine /home/ubuntu/UnrealEngine \
      --project /home/ubuntu/URLabTest/URLabTest.uproject \
      [--target CustomEditorTargetName] \
      [--filter URLab] \
      [--log /tmp/urlab_test.log]

--target defaults to <ProjectName>Editor derived from the .uproject filename.

Exit codes: 0 ok, 1 build failed, 2 tests failed, 3 bad args.
HELP
    exit 3
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --engine)   ENGINE="$2";  shift 2 ;;
        --project)  PROJECT="$2"; shift 2 ;;
        --target)   TARGET="$2";  shift 2 ;;
        --filter)   FILTER="$2";  shift 2 ;;
        --log)      LOG="$2";     shift 2 ;;
        -h|--help)  usage ;;
        *) echo "Unknown arg: $1" >&2; usage ;;
    esac
done

[[ -z "$ENGINE"  ]] && { echo "Missing --engine"  >&2; usage; }
[[ -z "$PROJECT" ]] && { echo "Missing --project" >&2; usage; }

# Derive the editor target from the .uproject filename if not explicitly
# supplied. UE's convention is <ProjectName>Editor — e.g. MyGame.uproject ->
# MyGameEditor. Override with --target for projects that don't follow this.
if [[ -z "$TARGET" ]]; then
    PROJECT_BASENAME=$(basename "$PROJECT")
    TARGET="${PROJECT_BASENAME%.*}Editor"
fi

BUILD_SH="$ENGINE/Engine/Build/BatchFiles/Linux/Build.sh"
CMD="$ENGINE/Engine/Binaries/Linux/UnrealEditor-Cmd"

[[ -x "$BUILD_SH" ]] || { echo "Build.sh not found: $BUILD_SH" >&2; exit 3; }
[[ -x "$CMD"      ]] || { echo "UnrealEditor-Cmd not found: $CMD" >&2; exit 3; }

# Pre-flight: refuse to run while an editor instance has the project locked.
# Match only the actual UnrealEditor binary, not the project string in our own
# command line.
if pgrep -fa "/UnrealEditor( |$)" >/dev/null 2>&1; then
    echo "ERROR: an UnrealEditor process is running. Close it first." >&2
    exit 3
fi

: > "$LOG"

# --- Build -----------------------------------------------------------------
echo ">>> Building $TARGET (Linux Development)..."
BUILD_OUT=$("$BUILD_SH" "$TARGET" Linux Development "-Project=$PROJECT" -WaitMutex 2>&1 || true)
echo "$BUILD_OUT" | tail -10
if echo "$BUILD_OUT" | grep -q "Result: Succeeded\|Target is up to date"; then
    BUILD_STATUS="Succeeded"
else
    BUILD_STATUS="Failed"
fi

# --- Stage runtime libs ---------------------------------------------------
# UBT's auto-RPATH for plugins symlinked outside the host project resolves
# incorrectly. Symlink third-party .so files into the plugin's Binaries/Linux/
# so the loader resolves them via ${ORIGIN} (which UBT does add correctly).
SETUP_RUNTIME="$(dirname "$0")/setup_runtime_linux.sh"
if [[ "$BUILD_STATUS" == "Succeeded" && -x "$SETUP_RUNTIME" ]]; then
    echo ">>> Staging third-party runtime libs..."
    bash "$SETUP_RUNTIME" || echo "WARN: setup_runtime_linux.sh failed (continuing)" >&2
fi

# --- Test ------------------------------------------------------------------
PASS=0
FAIL=0
TOTAL=0
TESTS_PERFORMED_LINE=""

if [[ "$BUILD_STATUS" == "Succeeded" ]]; then
    echo ">>> Running automation tests (filter=$FILTER, log=$LOG)..."
    "$CMD" "$PROJECT" \
        -ExecCmds="Automation RunTests $FILTER" \
        -Unattended -NullRHI -NoSound -NoSplash -stdout -log \
        -TestExit="Automation Test Queue Empty" \
        > "$LOG" 2>&1 || true

    if [[ ! -s "$LOG" ]]; then
        echo "ERROR: test log is empty — editor likely held the project lock." >&2
    fi

    PASS=$(grep -cE 'Result=\{Success\}' "$LOG" || true)
    FAIL=$(grep -cE 'Result=\{(Fail|Error)\}' "$LOG" || true)
    TOTAL=$((PASS + FAIL))
    TESTS_PERFORMED_LINE=$(grep -oE '[0-9]+ tests performed' "$LOG" | tail -1 || true)
fi

# --- Fingerprint + summary -------------------------------------------------
# Note: Host and Log-path lines are intentionally omitted from the summary
# block because they leak the reporter's hostname / username on local runs.
# The SHA-256 of the log is sufficient for reviewers to verify content
# integrity when they re-run the suite themselves.
TS=$(date -u '+%Y-%m-%d %H:%M:%S UTC')
GIT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
GIT_SHA=$(git -C "$GIT_DIR" rev-parse --short=8 HEAD 2>/dev/null || echo "unknown")
GIT_BRANCH=$(git -C "$GIT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

# Strip the engine path down to its last segment (e.g. 'UnrealEngine'). The
# full path can contain the user's install root on a non-standard layout.
ENGINE_LABEL=$(basename "$ENGINE")
[[ -z "$ENGINE_LABEL" ]] && ENGINE_LABEL="$ENGINE"

# Third-party dep SHAs from third_party/install/<dep>/INSTALLED_SHA.txt
# (written by the CMake build scripts). Lets a reviewer verify they're
# comparing against the same binary toolchain. Silent skip if missing.
DEPS_LINE=""
for pair in "mj:MuJoCo" "coacd:CoACD" "zmq:libzmq"; do
    key="${pair%%:*}"
    dir="${pair##*:}"
    sha_file="$GIT_DIR/third_party/install/$dir/INSTALLED_SHA.txt"
    if [[ -s "$sha_file" ]]; then
        sha=$(head -c 7 "$sha_file")
        if [[ -n "$DEPS_LINE" ]]; then DEPS_LINE="$DEPS_LINE "; fi
        DEPS_LINE="${DEPS_LINE}${key}=${sha}"
    fi
done
[[ -z "$DEPS_LINE" ]] && DEPS_LINE="unavailable"

LOG_HASH="n/a"
if [[ -s "$LOG" ]]; then
    if command -v sha256sum >/dev/null 2>&1; then
        LOG_HASH=$(sha256sum "$LOG" | cut -c1-16)
    fi
fi

cat <<EOF

=== URLab build+test summary ===
Timestamp : $TS
Git HEAD  : $GIT_SHA ($GIT_BRANCH)
Engine    : $ENGINE_LABEL
Deps      : $DEPS_LINE
Build     : $BUILD_STATUS
Tests     : $PASS / $TOTAL passed ($FAIL failed)${TESTS_PERFORMED_LINE:+  [$TESTS_PERFORMED_LINE]}
Log sha256: $LOG_HASH
================================
EOF

[[ "$BUILD_STATUS" != "Succeeded" ]] && exit 1
[[ "$FAIL" -gt 0 || "$TOTAL" -eq 0 ]] && exit 2
exit 0
