param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release",
    [switch]$NoSubmoduleSync
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

# Wipe any prior install of THIS package only - cmake --install is additive
# and would otherwise leave stale files behind across version bumps.
if (Test-Path $InstallDir) {
    Write-Host "Removing previous install at $InstallDir" -ForegroundColor Gray
    Remove-Item -Recurse -Force $InstallDir
}

# Sync CoACD submodule to the SHA URLab expects. Unconditional by design
# (see MuJoCo/build.ps1 for the rationale). Pass -NoSubmoduleSync to keep
# local edits in src/.
$RepoDir = "src"
if ($NoSubmoduleSync) {
    Write-Host "Skipping submodule sync (-NoSubmoduleSync). Using whatever is checked out in $RepoDir." -ForegroundColor Yellow
    if (-not (Test-Path (Join-Path $RepoDir "CMakeLists.txt"))) {
        throw "CoACD submodule not initialised at $RepoDir but -NoSubmoduleSync was set. Drop the flag or run 'git submodule update --init --recursive -- $RepoDir' manually."
    }
} else {
    Write-Host "Syncing CoACD submodule to URLab's pinned commit..." -ForegroundColor Gray
    # --force is required: this script intentionally overlays
    # ../CoACD_custom/{CMakeLists.txt,cmake/*} onto src/ below, so the working
    # tree is always dirty across runs. Without --force, the next URLab-side
    # SHA bump would hit a checkout conflict here.
    git submodule update --init --recursive --force -- $RepoDir
    if ($LASTEXITCODE -ne 0) { throw "git submodule update failed for CoACD - is this a git checkout? Pass -NoSubmoduleSync to skip." }
}

# Capture the SHA we are about to build from (see MuJoCo/build.ps1).
$InstalledSha = (git -C $RepoDir rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or -not $InstalledSha) { throw "Failed to read HEAD SHA from $RepoDir - is the submodule initialised?" }

Push-Location $RepoDir

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

# Record the exact source SHA we just installed from (see MuJoCo/build.ps1).
$ShaFile = Join-Path $InstallDir "INSTALLED_SHA.txt"
[System.IO.File]::WriteAllText($ShaFile, $InstalledSha)
Write-Host "Recorded INSTALLED_SHA=$InstalledSha at $ShaFile" -ForegroundColor Gray
