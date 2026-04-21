param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release",
    [switch]$NoSubmoduleSync
)

# Convert to absolute path and normalize for CMake
if (-not [System.IO.Path]::IsPathRooted($InstallDir)) {
    $InstallDir = Join-Path $PSScriptRoot $InstallDir
}
$InstallDir = Join-Path $InstallDir "libzmq"
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

# Sync libzmq submodule to the SHA URLab expects. Unconditional by design
# (see MuJoCo/build.ps1 for the rationale). Pass -NoSubmoduleSync to keep
# local edits in src/.
$RepoDir = "src"
if ($NoSubmoduleSync) {
    Write-Host "Skipping submodule sync (-NoSubmoduleSync). Using whatever is checked out in $RepoDir." -ForegroundColor Yellow
    if (-not (Test-Path (Join-Path $RepoDir "CMakeLists.txt"))) {
        throw "libzmq submodule not initialised at $RepoDir but -NoSubmoduleSync was set. Drop the flag or run 'git submodule update --init --recursive -- $RepoDir' manually."
    }
} else {
    Write-Host "Syncing libzmq submodule to URLab's pinned commit..." -ForegroundColor Gray
    # --force for parity with MuJoCo/CoACD (see MuJoCo/build.ps1 for rationale).
    git submodule update --init --recursive --force -- $RepoDir
    if ($LASTEXITCODE -ne 0) { throw "git submodule update failed for libzmq - is this a git checkout? Pass -NoSubmoduleSync to skip." }
}

# Capture the SHA we are about to build from (see MuJoCo/build.ps1).
$InstalledSha = (git -C $RepoDir rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or -not $InstalledSha) { throw "Failed to read HEAD SHA from $RepoDir - is the submodule initialised?" }

Push-Location $RepoDir
if (-not (Test-Path "build")) { New-Item -ItemType Directory -Path "build" }
cd build

Write-Host "Configuring libzmq..." -ForegroundColor Gray
$cmakeArgs = @(
    "..",
    "-DCMAKE_INSTALL_PREFIX=$InstallDir",
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DZMQ_BUILD_TESTS=OFF",
    "-DWITH_PERF_TOOL=OFF",
    "-DENABLE_DRAFTS=OFF",
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$($BuildType.Replace('Release', '').Replace('Debug', 'Debug'))DLL"
)
cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed for libzmq" }

Write-Host "Building libzmq..." -ForegroundColor Gray
cmake --build . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Build failed for libzmq" }

Write-Host "Installing libzmq..." -ForegroundColor Gray
cmake --install . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Installation failed for libzmq" }

Pop-Location

# Record the exact source SHA we just installed from (see MuJoCo/build.ps1).
$ShaFile = Join-Path $InstallDir "INSTALLED_SHA.txt"
[System.IO.File]::WriteAllText($ShaFile, $InstalledSha)
Write-Host "Recorded INSTALLED_SHA=$InstalledSha at $ShaFile" -ForegroundColor Gray
