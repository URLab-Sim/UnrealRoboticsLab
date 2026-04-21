# Master Build Script for URLab Third-Party Dependencies (Windows)
#
# Each dep's build.ps1 will sync its submodule (third_party/<dep>/src) to the
# SHA URLab expects before building. Pass -NoSubmoduleSync to skip the sync
# across all three deps, e.g. when iterating on a submodule locally.
param(
    [switch]$NoSubmoduleSync
)

$RootDir = Get-Location
$InstallDir = Join-Path $RootDir "install"
$BuildType = "Release"

# Ensure directories exist
if (-not (Test-Path $InstallDir)) { New-Item -ItemType Directory -Path $InstallDir }

Write-Host "Starting Unified Build Process..." -ForegroundColor Cyan

$SharedArgs = @{ InstallDir = $InstallDir; BuildType = $BuildType }
if ($NoSubmoduleSync) { $SharedArgs["NoSubmoduleSync"] = $true }

# 1. CoACD
Write-Host "`n--- Building CoACD ---" -ForegroundColor Yellow
Push-Location CoACD
if (Test-Path "build.ps1") {
    .\build.ps1 @SharedArgs
} else {
    Write-Warning "CoACD/build.ps1 not found!"
}
Pop-Location

# 2. MuJoCo
Write-Host "`n--- Building MuJoCo ---" -ForegroundColor Yellow
Push-Location MuJoCo
if (Test-Path "build.ps1") {
    .\build.ps1 @SharedArgs
} else {
    Write-Warning "MuJoCo/build.ps1 not found!"
}
Pop-Location

# 3. libzmq
Write-Host "`n--- Building libzmq ---" -ForegroundColor Yellow
Push-Location libzmq
if (Test-Path "build.ps1") {
    .\build.ps1 @SharedArgs
} else {
    Write-Warning "libzmq/build.ps1 not found!"
}
Pop-Location

Write-Host "`nAll builds completed! check the 'install' folder." -ForegroundColor Green
