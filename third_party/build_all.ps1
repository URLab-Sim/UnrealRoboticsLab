# Master Build Script for Synthy Third-Party Dependencies (Windows)

$RootDir = Get-Location
$InstallDir = Join-Path $RootDir "install"
$BuildType = "Release"

# Ensure directories exist
if (-not (Test-Path $InstallDir)) { New-Item -ItemType Directory -Path $InstallDir }

Write-Host "Starting Unified Build Process..." -ForegroundColor Cyan

# 1. CoACD
Write-Host "`n--- Building CoACD ---" -ForegroundColor Yellow
Push-Location CoACD
if (Test-Path "build.ps1") {
    .\build.ps1 -InstallDir $InstallDir -BuildType $BuildType
} else {
    Write-Warning "CoACD/build.ps1 not found!"
}
Pop-Location

# 2. MuJoCo
Write-Host "`n--- Building MuJoCo ---" -ForegroundColor Yellow
Push-Location MuJoCo
if (Test-Path "build.ps1") {
    .\build.ps1 -InstallDir $InstallDir -BuildType $BuildType
} else {
    Write-Warning "MuJoCo/build.ps1 not found!"
}
Pop-Location

# 3. libzmq
Write-Host "`n--- Building libzmq ---" -ForegroundColor Yellow
Push-Location libzmq
if (Test-Path "build.ps1") {
    .\build.ps1 -InstallDir $InstallDir -BuildType $BuildType
} else {
    Write-Warning "libzmq/build.ps1 not found!"
}
Pop-Location

Write-Host "`nAll builds completed! check the 'install' folder." -ForegroundColor Green
