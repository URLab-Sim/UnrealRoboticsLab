param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release",
    [switch]$NoSubmoduleSync
)

# NoSubmoduleSync is accepted and ignored: this package is URLab's own source,
# not a submodule. The master script passes one argument set to every dep.
$null = $NoSubmoduleSync

if (-not [System.IO.Path]::IsPathRooted($InstallDir)) {
    $InstallDir = Join-Path $PSScriptRoot $InstallDir
}
$InstallRoot = [System.IO.Path]::GetFullPath($InstallDir).Replace('\', '/')
$MuJoCoDir = (Join-Path $InstallRoot "MuJoCo")
$InstallDir = [System.IO.Path]::GetFullPath((Join-Path $InstallDir "MjShim")).Replace('\', '/')

Write-Host "Resolved InstallDir: $InstallDir" -ForegroundColor Gray
Write-Host "Resolved BuildType: $BuildType" -ForegroundColor Gray

# Wipe any prior install of THIS package only, for the same reason MuJoCo does:
# cmake --install is additive and would leave stale files across a rename.
if (Test-Path $InstallDir) {
    Write-Host "Removing previous install at $InstallDir" -ForegroundColor Gray
    Remove-Item -Recurse -Force $InstallDir
}

if (-not (Test-Path (Join-Path $MuJoCoDir "include/mujoco/mujoco.h"))) {
    throw "MuJoCo is not installed at $MuJoCoDir. Build it first (third_party/MuJoCo/build.ps1); MjShim links against it."
}

Push-Location $PSScriptRoot

if (-not (Test-Path "build")) { New-Item -ItemType Directory -Path "build" | Out-Null }
Push-Location build

Write-Host "Configuring MjShim..." -ForegroundColor Gray
cmake .. `
    "-DCMAKE_INSTALL_PREFIX=$InstallDir" `
    "-DCMAKE_BUILD_TYPE=$BuildType" `
    "-DMUJOCO_INSTALL_DIR=$MuJoCoDir"
if ($LASTEXITCODE -ne 0) { Pop-Location; Pop-Location; throw "CMake configuration failed for MjShim" }

Write-Host "Building MjShim..." -ForegroundColor Gray
cmake --build . --config $BuildType
if ($LASTEXITCODE -ne 0) { Pop-Location; Pop-Location; throw "Build failed for MjShim" }

Write-Host "Installing MjShim..." -ForegroundColor Gray
cmake --install . --config $BuildType
if ($LASTEXITCODE -ne 0) { Pop-Location; Pop-Location; throw "Installation failed for MjShim" }

Pop-Location
Pop-Location

Write-Host "MjShim installed to $InstallDir" -ForegroundColor Gray
