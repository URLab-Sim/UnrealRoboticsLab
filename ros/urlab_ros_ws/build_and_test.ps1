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

<#
.SYNOPSIS
    Configure + build the standalone urlab_rcl_test harness against a
    user-installed ROS 2 Lyrical Luth and run its selftest plus a cross-process
    `ros2 topic echo` check. This is the primary Windows validation of the
    in-process rcl publish/subscribe path.

.DESCRIPTION
    Assumes ROS 2 Lyrical is already installed by the user via Pixi/Conda
    (prefix.dev + RoboStack), the default Windows install path. The normal flow
    is to activate that environment with `pixi shell` and then run this script;
    the script detects the active ROS environment and proceeds. Alternatively an
    explicit activation script can be supplied via -RosSetup or URLAB_ROS2_SETUP.
    It installs nothing and writes nothing outside build\.

.PARAMETER RosSetup
    Full path to a ROS/conda activation .bat script to import before building.
    Overrides URLAB_ROS2_SETUP. Omit it when the ROS env is already active
    (e.g. inside `pixi shell`).

.NOTES
    Exit codes: 0 ok, 1 build/env failed, 2 tests failed, 3 bad args.
#>

[CmdletBinding()]
param(
    [string] $RosSetup = ''
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

# --- Resolve / detect the ROS 2 environment --------------------------------
if ([string]::IsNullOrWhiteSpace($RosSetup) -and
    -not [string]::IsNullOrWhiteSpace($env:URLAB_ROS2_SETUP)) {
    $RosSetup = $env:URLAB_ROS2_SETUP
}

if (-not [string]::IsNullOrWhiteSpace($RosSetup)) {
    if (-not (Test-Path $RosSetup)) {
        Write-Error "ROS activation script not found: $RosSetup"
        exit 3
    }
    Write-Host ">>> Activating ROS 2 environment: $RosSetup"
    # Run the activation script in a child cmd and import the resulting
    # environment into this session so cmake/ros2 see the ROS toolchain + libs.
    $envDump = cmd /c "call `"$RosSetup`" >nul 2>&1 && set"
    foreach ($line in $envDump) {
        if ($line -match '^([^=]+)=(.*)$') {
            Set-Item -Path ("env:" + $matches[1]) -Value $matches[2]
        }
    }
}

if ([string]::IsNullOrWhiteSpace($env:AMENT_PREFIX_PATH)) {
    Write-Error @"
No active ROS 2 environment (AMENT_PREFIX_PATH is empty).

The default Windows install is Pixi/Conda (prefix.dev + RoboStack). Activate it
and re-run, e.g.:

  cd <your ROS pixi project>
  pixi shell            # activates ROS 2 Lyrical for this shell
  cd <plugin>\ros\urlab_ros_ws
  .\build_and_test.ps1

Or point -RosSetup / `$env:URLAB_ROS2_SETUP at a ROS/conda activation .bat.
See docs/ros_workspace_setup.md.
"@
    exit 3
}
Write-Host ">>> ROS 2 environment active (AMENT_PREFIX_PATH set)."

# --- Configure + build -----------------------------------------------------
Write-Host ">>> Configuring (cmake)..."
cmake -B build -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { Write-Error "cmake configure failed."; exit 1 }

Write-Host ">>> Building (Release)..."
cmake --build build --config Release
if ($LASTEXITCODE -ne 0) { Write-Error "cmake build failed."; exit 1 }

# Locate the executable (multi-config generators nest it under Release\).
$exe = Get-ChildItem -Path build -Recurse -Filter 'urlab_rcl_test.exe' |
    Select-Object -First 1 -ExpandProperty FullName
if (-not $exe) {
    Write-Error "urlab_rcl_test.exe not found under build\ after the build."
    exit 1
}
Write-Host ">>> Built: $exe"

# --- Selftest --------------------------------------------------------------
Write-Host ">>> Running selftest..."
& $exe --selftest
if ($LASTEXITCODE -ne 0) { Write-Error "selftest failed."; exit 2 }

# --- Cross-process echo verify ---------------------------------------------
# Publish continuously in the background and confirm a second process sees the
# expected JointState over real DDS. This is the in-process Windows leg the
# whole ROS design must prove.
Write-Host ">>> Cross-process verify (ros2 topic echo)..."
$topic = '/urlab_test/joint_states'
$echoOut = Join-Path ([System.IO.Path]::GetTempPath()) "urlab_echo_$PID.out"
$echoErr = Join-Path ([System.IO.Path]::GetTempPath()) "urlab_echo_$PID.err"
# Publish long enough to outlast `ros2 topic echo` discovery on a cold ROS
# daemon; the publisher is force-killed as soon as echo returns, so this is an
# upper bound, not a fixed wait. `ros2 topic echo --once` has no built-in
# timeout and blocks forever if it misses the publisher's window (or the topic
# name is wrong), so it runs as a child process bounded by WaitForExit and is
# force-killed on timeout -- the verify can never hang the script.
$pub = Start-Process -FilePath $exe -ArgumentList '--publish', '3000' -PassThru -NoNewWindow
$echoProc = $null
$echo = ''
try {
    Start-Sleep -Seconds 3
    $echoProc = Start-Process -FilePath 'ros2' `
        -ArgumentList 'topic', 'echo', '--once', $topic `
        -NoNewWindow -PassThru `
        -RedirectStandardOutput $echoOut -RedirectStandardError $echoErr
    if (-not $echoProc.WaitForExit(30000)) {
        Write-Warning "ros2 topic echo did not return within 30s; killing it."
        Stop-Process -Id $echoProc.Id -Force -ErrorAction SilentlyContinue
    }
    $echo = ((Get-Content $echoOut -Raw -ErrorAction SilentlyContinue) + "`n" +
             (Get-Content $echoErr -Raw -ErrorAction SilentlyContinue))
    Write-Host $echo
} finally {
    if ($echoProc -and -not $echoProc.HasExited) {
        Stop-Process -Id $echoProc.Id -Force -ErrorAction SilentlyContinue
    }
    if ($pub -and -not $pub.HasExited) {
        Stop-Process -Id $pub.Id -Force -ErrorAction SilentlyContinue
    }
    Remove-Item $echoOut, $echoErr -ErrorAction SilentlyContinue
}

if ($echo -match 'joint_a' -and $echo -match 'joint_b' -and $echo -match 'joint_c') {
    Write-Host ">>> Cross-process verify OK: JointState names received."
} else {
    Write-Error "Cross-process verify failed: expected joint names not seen in `ros2 topic echo` output."
    exit 2
}

Write-Host ""
Write-Host "=== urlab_rcl_test: ALL CHECKS PASSED (Windows) ==="
exit 0
