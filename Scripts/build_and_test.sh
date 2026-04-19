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

# build_and_test.sh — build url_projEditor and run the URLab automation suite,
# then print a machine-identifiable summary block to paste into a PR.
#
# Usage:
#   ./Scripts/build_and_test.sh \
#       --engine "/c/Program Files/Epic Games/UE_5.7" \
#       --project "C:/path/to/your.uproject" \
#       [--target url_projEditor] \
#       [--filter URLab] \
#       [--log /tmp/urlab_test.log]
#
# Exit codes: 0 ok, 1 build failed, 2 tests failed, 3 bad args.

set -eu

TARGET="url_projEditor"
FILTER="URLab"
LOG="/tmp/urlab_test.log"
ENGINE=""
PROJECT=""

usage() {
    cat >&2 <<'HELP'
build_and_test.sh — build url_projEditor and run the URLab automation suite.

Usage:
  ./Scripts/build_and_test.sh \
      --engine "/c/Program Files/Epic Games/UE_5.7" \
      --project "C:/path/to/your.uproject" \
      [--target url_projEditor] \
      [--filter URLab] \
      [--log /tmp/urlab_test.log]

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

UBT="$ENGINE/Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.exe"
CMD="$ENGINE/Engine/Binaries/Win64/UnrealEditor-Cmd.exe"

[[ -x "$UBT" ]] || { echo "UBT not found: $UBT" >&2; exit 3; }
[[ -x "$CMD" ]] || { echo "UnrealEditor-Cmd not found: $CMD" >&2; exit 3; }

# Truncate the test log up-front so a build failure (or any early exit
# before UnrealEditor-Cmd writes to it) doesn't leave the SHA-256 in the
# summary pointing at a previous run's file.
: > "$LOG"

# --- Build -----------------------------------------------------------------
echo ">>> Building $TARGET (Win64 Development)..."
BUILD_OUT=$("$UBT" "$TARGET" Win64 Development "-Project=$PROJECT" -WaitMutex 2>&1 || true)
echo "$BUILD_OUT" | tail -10
if echo "$BUILD_OUT" | grep -q "Result: Succeeded"; then
    BUILD_STATUS="Succeeded"
else
    BUILD_STATUS="Failed"
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
TS=$(date -u '+%Y-%m-%d %H:%M:%S UTC')
HOST=$(hostname 2>/dev/null || echo "unknown")
GIT_SHA=$(git rev-parse --short=8 HEAD 2>/dev/null || echo "unknown")
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
LOG_HASH="n/a"
if [[ -s "$LOG" ]]; then
    if command -v sha256sum >/dev/null 2>&1; then
        LOG_HASH=$(sha256sum "$LOG" | cut -c1-16)
    elif command -v shasum >/dev/null 2>&1; then
        LOG_HASH=$(shasum -a 256 "$LOG" | cut -c1-16)
    fi
fi

cat <<EOF

=== URLab build+test summary ===
Timestamp : $TS
Host      : $HOST
Git HEAD  : $GIT_SHA ($GIT_BRANCH)
Engine    : $ENGINE
Build     : $BUILD_STATUS
Tests     : $PASS / $TOTAL passed ($FAIL failed)${TESTS_PERFORMED_LINE:+  [$TESTS_PERFORMED_LINE]}
Log       : $LOG  (sha256: $LOG_HASH)
================================
EOF

[[ "$BUILD_STATUS" != "Succeeded" ]] && exit 1
[[ "$FAIL" -gt 0 || "$TOTAL" -eq 0 ]] && exit 2
exit 0
