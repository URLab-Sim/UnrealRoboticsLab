param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release"
)

# Convert to absolute path and normalize for CMake
if (-not [System.IO.Path]::IsPathRooted($InstallDir)) {
    $InstallDir = Join-Path $PSScriptRoot $InstallDir
}
$InstallDir = Join-Path $InstallDir "MuJoCo"
$InstallDir = [System.IO.Path]::GetFullPath($InstallDir)
$InstallDir = $InstallDir.Replace('\', '/')

Write-Host "Resolved InstallDir: $InstallDir" -ForegroundColor Gray
Write-Host "Resolved BuildType: $BuildType" -ForegroundColor Gray

# Wipe any prior install of THIS package only. cmake --install is additive and
# leaves stale files behind when the upstream stops producing them — e.g. the
# MuJoCo 3.7.0 bump statically linked obj_decoder/stl_decoder into mujoco.dll
# but additive installs over a 3.6.x tree left the old plugin DLLs in bin/,
# silently breaking OBJ loading until they were manually removed.
if (Test-Path $InstallDir) {
    Write-Host "Removing previous install at $InstallDir" -ForegroundColor Gray
    Remove-Item -Recurse -Force $InstallDir
}

$PinnedCommit = "72cb2b210da6"  # Pin to MuJoCo 3.7.0 release (2026-04-14)

$RepoDir = "src"
if (-not (Test-Path $RepoDir)) {
    Write-Host "Cloning MuJoCo (pinned: $PinnedCommit)..." -ForegroundColor Gray
    git clone https://github.com/google-deepmind/mujoco $RepoDir
    Push-Location $RepoDir
    git checkout $PinnedCommit
    Pop-Location
} else {
    Write-Host "MuJoCo source already exists at '$RepoDir'. Skipping clone." -ForegroundColor Yellow
    Write-Host "  To rebuild from scratch, delete '$RepoDir/' and re-run." -ForegroundColor Yellow
}

Push-Location $RepoDir
Write-Host "Initializing submodules..." -ForegroundColor Gray
git submodule update --init --recursive
if ($LASTEXITCODE -ne 0) { Write-Warning "Submodule update failed for MuJoCo" }
if (-not (Test-Path "build")) { New-Item -ItemType Directory -Path "build" }
cd build

Write-Host "Configuring MuJoCo..." -ForegroundColor Gray
$cmakeArgs = @(
    "..",
    "-DCMAKE_INSTALL_PREFIX=$InstallDir",
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DMUJOCO_BUILD_EXAMPLES=OFF",
    "-DMUJOCO_BUILD_TESTS=OFF",
    "-DMUJOCO_BUILD_SIMULATE=OFF",
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$($BuildType.Replace('Release', '').Replace('Debug', 'Debug'))DLL"
)
cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed for MuJoCo" }

Write-Host "Building MuJoCo..." -ForegroundColor Gray
cmake --build . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Build failed for MuJoCo" }

Write-Host "Installing MuJoCo..." -ForegroundColor Gray
cmake --install . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Installation failed for MuJoCo" }

Pop-Location
