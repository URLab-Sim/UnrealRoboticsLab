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

<#
.SYNOPSIS
    Build the project's Editor target and run the URLab automation suite, then print a
    machine-identifiable summary block to paste into a PR.

.EXAMPLE
    .\Scripts\build_and_test.ps1 `
        -Engine  'C:\Program Files\Epic Games\UE_5.7' `
        -Project 'C:\path\to\your.uproject'

.EXAMPLE
    # Override the editor target only if your project doesn't follow UE's
    # <ProjectName>Editor convention.
    .\Scripts\build_and_test.ps1 `
        -Engine  'C:\Program Files\Epic Games\UE_5.7' `
        -Project 'C:\MyGame\MyGame.uproject' `
        -Target  CustomEditorTargetName

.NOTES
    Exit codes: 0 ok, 1 build failed, 2 tests failed, 3 bad args, 4 generated-profile drift.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string] $Engine,
    [Parameter(Mandatory = $true)] [string] $Project,
    [string] $Target = '',
    [string] $Filter = 'URLab',
    # Unique per run. Two runs sharing one path collide on the file lock: the
    # loser dies outright, and the winner can be left with a truncated or empty
    # log, which greps to zero failures and reads exactly like a clean pass.
    [string] $Log    = (Join-Path $env:TEMP ('urlab_test_{0}_{1}.log' -f (Get-Date -Format 'yyyyMMdd-HHmmss'), $PID))
)

$ErrorActionPreference = 'Stop'

# Derive the editor target from the .uproject filename if not explicitly
# supplied. UE's convention is <ProjectName>Editor - e.g. MyGame.uproject ->
# MyGameEditor. Override with -Target for projects that don't follow this.
if ([string]::IsNullOrWhiteSpace($Target)) {
    $Target = [System.IO.Path]::GetFileNameWithoutExtension($Project) + 'Editor'
}

$bat = Join-Path $Engine 'Engine\Build\BatchFiles\Build.bat'
$cmd = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'

if (-not (Test-Path $bat)) { Write-Error "Build.bat not found: $bat";          exit 3 }
if (-not (Test-Path $cmd)) { Write-Error "UnrealEditor-Cmd not found: $cmd";   exit 3 }

# --- Generated-profile drift gate ------------------------------------------
# Refuse to build if the emitted MuJoCo document profile drifted from what the
# schema now says. The gate needs the generator and `uv`; when either is absent
# it cannot run, and it says so rather than passing quietly. A skipped gate and
# a passing gate are the same exit code, so silence here reads as a green gate
# in every log and every CI summary that only checks the code.
$pluginRoot = Split-Path -Parent $PSScriptRoot
$regen = Join-Path $pluginRoot 'Scripts/regen_ue_profile.ps1'
$protospec = Join-Path $pluginRoot 'protospec/protospec_gen'
$gateMissing = @()
if (-not (Test-Path $regen))     { $gateMissing += "no $regen" }
if (-not (Test-Path $protospec)) { $gateMissing += "no ProtoSpec generator at $protospec" }
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) { $gateMissing += 'uv is not on PATH' }

if ($gateMissing.Count -gt 0) {
    Write-Warning ("Profile drift gate SKIPPED ({0}). The generated MuJoCo profile is NOT checked in this run." -f ($gateMissing -join '; '))
} else {
    Write-Host '>>> Profile drift gate: regen_ue_profile.ps1 -Check'
    & $regen -Check
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Generated profile drift detected (exit $LASTEXITCODE). Re-run 'Scripts/regen_ue_profile.ps1', then re-run this script."
        exit 4
    }
}

# Truncate the test log up-front so a build failure (or any early exit
# before UnrealEditor-Cmd writes to it) doesn't leave the SHA-256 in the
# summary pointing at a previous run's file.
Set-Content -Path $Log -Value '' -NoNewline

# --- Build -----------------------------------------------------------------
Write-Host ">>> Building $Target (Win64 Development)..."
# Note: -TargetType=Editor is intentionally omitted. Combined with a positional
# target name it makes UBT's two action-graph passes ambiguous and bails with
# Result: Failed (ActionGraphInvalid). Just the positional target is enough.
$buildArgs = @($Target, 'Win64', 'Development', "-Project=$Project", '-Progress')
$buildOut  = & $bat @buildArgs 2>&1

# The whole build output goes to a file, and what reaches the console is the
# diagnostics rather than the last ten lines. A failing build is explained by
# its FIRST error; the tail is whatever the toolchain happened to say
# afterwards, which on a large generated translation unit is usually a summary
# that names no file at all.
$buildLog = "$Log.build.txt"
$buildOut | Out-File -FilePath $buildLog -Encoding utf8
$diagnostics = $buildOut | Select-String -Pattern 'error [A-Z]+[0-9]+|error:|fatal error|Result: '
if ($diagnostics) {
    $diagnostics | Select-Object -First 40 | ForEach-Object { Write-Host $_ }
} else {
    $buildOut | Select-Object -Last 10 | ForEach-Object { Write-Host $_ }
}
Write-Host ">>> Full build output: $buildLog"
$buildStatus = if ($buildOut -match 'Result: Succeeded') { 'Succeeded' } else { 'Failed' }

# --- Test ------------------------------------------------------------------
$pass  = 0
$fail  = 0
$total = 0
$testsPerformed = ''

if ($buildStatus -eq 'Succeeded') {
    Write-Host ">>> Running automation tests (filter=$Filter, log=$Log)..."
    $testArgs = @(
        $Project,
        "-ExecCmds=Automation RunTests $Filter",
        '-Unattended', '-NullRHI', '-NoSound', '-NoSplash', '-stdout', '-log',
        '-TestExit=Automation Test Queue Empty'
    )
    & $cmd @testArgs *> $Log

    if (-not (Test-Path $Log) -or (Get-Item $Log).Length -eq 0) {
        Write-Error "Test log is empty - editor likely held the project lock."
    }
    else {
        $content = Get-Content $Log -Raw
        $pass  = ([regex]::Matches($content, 'Result=\{Success\}')).Count
        $fail  = ([regex]::Matches($content, 'Result=\{(Fail|Error)\}')).Count
        $total = $pass + $fail
        if ($content -match '(\d+) tests performed') { $testsPerformed = $Matches[1] + ' tests performed' }
    }
}

# --- Fingerprint + summary -------------------------------------------------
# Note: Host and Log-path lines are intentionally omitted from the summary
# block because they leak the reporter's hostname / Windows username on local
# runs. The SHA-256 of the log is sufficient for reviewers to verify content
# integrity when they re-run the suite themselves (#37).
$ts        = (Get-Date).ToUniversalTime().ToString('yyyy-MM-dd HH:mm:ss') + ' UTC'
# Asked of the PLUGIN repository explicitly, not of whatever directory the
# script happened to be invoked from. The SHA in this block is what ties a gate
# result to a commit in the record, so it must not depend on the caller's
# working directory: run from the host project root, a bare `git rev-parse`
# answers for the wrong repository or for none.
$gitSha    = (git -C $PSScriptRoot rev-parse --short=8 HEAD 2>$null)
$gitBranch = (git -C $PSScriptRoot rev-parse --abbrev-ref HEAD 2>$null)
if (-not $gitSha)    { $gitSha    = 'unknown' } else { $gitSha    = $gitSha.Trim() }
if (-not $gitBranch) { $gitBranch = 'unknown' } else { $gitBranch = $gitBranch.Trim() }

# Strip the engine path down to its last segment (e.g. 'UE_5.7'). The full
# path can contain the user's install root on a non-standard layout.
$engineLabel = Split-Path -Leaf $Engine
if ([string]::IsNullOrWhiteSpace($engineLabel)) { $engineLabel = $Engine }

# Third-party dep SHAs — pulls from third_party/install/<dep>/INSTALLED_SHA.txt
# (written by the CMake build scripts). Lets a reviewer verify they're
# comparing against the same binary toolchain. Silent skip if missing.
$pluginRoot = Split-Path -Parent $PSScriptRoot
$depsLine = ''
foreach ($kv in @(@{name='mj';  dir='MuJoCo'},
                  @{name='coacd'; dir='CoACD'},
                  @{name='zmq'; dir='libzmq'})) {
    $shaPath = Join-Path $pluginRoot ('third_party/install/' + $kv.dir + '/INSTALLED_SHA.txt')
    if (Test-Path $shaPath) {
        $sha = (Get-Content $shaPath -Raw).Trim()
        if ($sha.Length -ge 7) {
            if ($depsLine) { $depsLine += ' ' }
            $depsLine += ('{0}={1}' -f $kv.name, $sha.Substring(0, 7))
        }
    }
}
if ([string]::IsNullOrWhiteSpace($depsLine)) { $depsLine = 'unavailable' }

$logHash = 'n/a'
if ((Test-Path $Log) -and (Get-Item $Log).Length -gt 0) {
    $logHash = (Get-FileHash $Log -Algorithm SHA256).Hash.Substring(0, 16).ToLowerInvariant()
}

$testsLine = '{0} / {1} passed ({2} failed)' -f $pass, $total, $fail
if ($testsPerformed) { $testsLine += '  [' + $testsPerformed + ']' }

Write-Host ''
Write-Host '=== URLab build+test summary ==='
Write-Host ('Timestamp : ' + $ts)
Write-Host ('Git HEAD  : {0} ({1})' -f $gitSha, $gitBranch)
Write-Host ('Engine    : ' + $engineLabel)
Write-Host ('Deps      : ' + $depsLine)
Write-Host ('Build     : ' + $buildStatus)
Write-Host ('Tests     : ' + $testsLine)
Write-Host ('Log sha256: ' + $logHash)
Write-Host '================================'

if ($buildStatus -ne 'Succeeded') { exit 1 }
if ($fail -gt 0 -or $total -eq 0) { exit 2 }
exit 0
