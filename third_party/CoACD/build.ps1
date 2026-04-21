param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release"
)

# Convert to absolute path and normalize for CMake
if (-not [System.IO.Path]::IsPathRooted($InstallDir)) {
    $InstallDir = Join-Path $PSScriptRoot $InstallDir
}
$InstallDir = Join-Path $InstallDir "CoACD"
$InstallDir = [System.IO.Path]::GetFullPath($InstallDir)
$InstallDir = $InstallDir.Replace('\', '/')

Write-Host "Resolved InstallDir: $InstallDir" -ForegroundColor Gray
Write-Host "Resolved BuildType: $BuildType" -ForegroundColor Gray

# Wipe any prior install of THIS package only — cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if (Test-Path $InstallDir) {
    Write-Host "Removing previous install at $InstallDir" -ForegroundColor Gray
    Remove-Item -Recurse -Force $InstallDir
}

$PinnedCommit = "c7436bf"  # Pin to tested commit (CDT include path fix + unbundled 3rd party support)

$RepoDir = "src"
if (-not (Test-Path $RepoDir)) {
    Write-Host "Cloning CoACD (pinned: $PinnedCommit)..." -ForegroundColor Gray
    git clone https://github.com/SarahWeiii/CoACD $RepoDir
    Push-Location $RepoDir
    git checkout $PinnedCommit
    Pop-Location
} else {
    Write-Host "CoACD source already exists at '$RepoDir'. Skipping clone." -ForegroundColor Yellow
    Write-Host "  To rebuild from scratch, delete '$RepoDir/' and re-run." -ForegroundColor Yellow
}

Push-Location $RepoDir
Write-Host "Initializing submodules..." -ForegroundColor Gray
git submodule update --init --recursive
if ($LASTEXITCODE -ne 0) { Write-Warning "Submodule update failed for CoACD" }

Write-Host "Applying custom CoACD configuration..." -ForegroundColor Gray
Copy-Item -Path "../../CoACD_custom/CMakeLists.txt" -Destination "." -Force
if (Test-Path "../../CoACD_custom/cmake") {
    if (-not (Test-Path "cmake")) { New-Item -ItemType Directory -Path "cmake" | Out-Null }
    Copy-Item -Path "../../CoACD_custom/cmake/*" -Destination "cmake/" -Recurse -Force
}

if (-not (Test-Path "build")) { New-Item -ItemType Directory -Path "build" }
cd build

Write-Host "Configuring CoACD..." -ForegroundColor Gray
# Note: Unreal uses /MD (Dynamic Runtime). 
# We need to ensure CoACD and its deps are built with /MD.
# Usually this is default in CMake for MSVC if not specified otherwise.
$cmakeArgs = @(
    "..",
    "-G", "Visual Studio 17 2022",
    "-A", "x64",
    "-DCMAKE_INSTALL_PREFIX=$InstallDir",
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$($BuildType.Replace('Release', '').Replace('Debug', 'Debug'))DLL",
    "-DOPENVDB_CORE_SHARED=OFF",
    "-DTBB_TEST=OFF",
    "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
    "-DCMAKE_CXX_FLAGS=/MD /EHsc"
)
cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed for CoACD" }

Write-Host "Building CoACD..." -ForegroundColor Gray
cmake --build . --config $BuildType --target _coacd
if ($LASTEXITCODE -ne 0) { throw "Build failed for CoACD" }

Write-Host "Installing CoACD..." -ForegroundColor Gray
cmake --install . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Installation failed for CoACD" }

# CoACD build also produces some dependencies in _deps that we might need to manually copy if they aren't installed.
# We'll check if spdlog and others are in the install folder.

Pop-Location
