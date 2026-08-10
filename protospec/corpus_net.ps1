# The corpus net: build the round-trip differential's tools and run it.
#
# One entry point, so the net is a single command in CI and on a developer's
# machine: it configures and builds ps_roundtrip and mj_model_diff against a
# prebuilt MuJoCo, points the harness at the corpus, and exits non-zero on
# anything but the recorded allowed failures. The verdict itself lives in
# tools/corpus_net.py, shared with the Linux twin corpus_net.sh so the two
# cannot drift apart in what they accept.
#
# What it proves: for every model in MuJoCo's own corpus that the retained
# reader supports, parse -> write -> mj_loadXML produces the same mjModel, field
# by field, as a stock mj_loadXML of the original.

param(
    [string]$MujocoRoot = "../third_party/install/MuJoCo",
    [string]$Corpus = "../third_party/MuJoCo/src",
    [string]$BuildType = "Release"
)

$ErrorActionPreference = "Stop"

# Anchored on the script, not the caller's working directory.
function Resolve-Rooted([string]$Path) {
    if (-not [System.IO.Path]::IsPathRooted($Path)) {
        $Path = Join-Path $PSScriptRoot $Path
    }
    return [System.IO.Path]::GetFullPath($Path).Replace('\', '/')
}

$MujocoRoot = Resolve-Rooted $MujocoRoot
$Corpus = Resolve-Rooted $Corpus

if (-not (Test-Path "$MujocoRoot/include/mujoco/mujoco.h")) {
    throw "No MuJoCo headers under $MujocoRoot. Run third_party/build_all.ps1 first, or pass -MujocoRoot."
}
if (-not (Test-Path $Corpus)) {
    throw "No MuJoCo corpus at $Corpus. Pass -Corpus, or set it to a MuJoCo source checkout."
}

$Src = Resolve-Rooted "lib"
$Build = "$Src/build-urlab"

# The same build directory and the same configure line as build.ps1, so the two
# scripts share a cache instead of invalidating each other's.
Write-Host "Configuring ProtoSpec from $Src..." -ForegroundColor Gray
cmake -S $Src -B $Build -DCMAKE_BUILD_TYPE=$BuildType "-DMUJOCO_ROOT=$MujocoRoot" `
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$($BuildType.Replace('Release', '').Replace('Debug', 'Debug'))DLL"
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed for ProtoSpec" }

Write-Host "Building the differential tools..." -ForegroundColor Gray
cmake --build $Build --config $BuildType --target ps_roundtrip mj_model_diff
if ($LASTEXITCODE -ne 0) { throw "Build failed for ps_roundtrip / mj_model_diff" }

Write-Host "Running the corpus net over $Corpus..." -ForegroundColor Gray
$env:PROTOSPEC_CORPUS = $Corpus
Push-Location $PSScriptRoot
try {
    uv run python tools/corpus_net.py
    $Code = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($Code -ne 0) {
    Write-Host "Corpus net FAILED (exit $Code)" -ForegroundColor Red
} else {
    Write-Host "Corpus net passed" -ForegroundColor Green
}
exit $Code
