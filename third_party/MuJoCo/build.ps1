param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release",
    [switch]$NoSubmoduleSync
)

# Convert to absolute path and normalize for CMake
if (-not [System.IO.Path]::IsPathRooted($InstallDir)) {
    $InstallDir = Join-Path $PSScriptRoot $InstallDir
}
$InstallRoot = [System.IO.Path]::GetFullPath($InstallDir).Replace('\', '/')
$InstallDir = Join-Path $InstallDir "MuJoCo"
$InstallDir = [System.IO.Path]::GetFullPath($InstallDir)
$InstallDir = $InstallDir.Replace('\', '/')

Write-Host "Resolved InstallDir: $InstallDir" -ForegroundColor Gray
Write-Host "Resolved BuildType: $BuildType" -ForegroundColor Gray

# Wipe any prior install of THIS package only. cmake --install is additive and
# would otherwise leave stale files behind across version bumps.
if (Test-Path $InstallDir) {
    Write-Host "Removing previous install at $InstallDir" -ForegroundColor Gray
    Remove-Item -Recurse -Force $InstallDir
}

# Sync MuJoCo submodule to the SHA URLab expects. This is unconditional by
# design: after a `git pull` on URLab the submodule pointer may have moved,
# and building against stale source would produce a mismatched install that
# URLab.Build.cs then rejects. Pass -NoSubmoduleSync to keep local edits in
# src/ (e.g. while iterating on MuJoCo itself).
$RepoDir = "src"
if ($NoSubmoduleSync) {
    Write-Host "Skipping submodule sync (-NoSubmoduleSync). Using whatever is checked out in $RepoDir." -ForegroundColor Yellow
    if (-not (Test-Path (Join-Path $RepoDir "CMakeLists.txt"))) {
        throw "MuJoCo submodule not initialised at $RepoDir but -NoSubmoduleSync was set. Drop the flag or run 'git submodule update --init --recursive -- $RepoDir' manually."
    }
} else {
    Write-Host "Syncing MuJoCo submodule to URLab's pinned commit..." -ForegroundColor Gray
    # --force is required because CoACD's build dirties its submodule working
    # tree (custom CMakeLists overlay) and sibling deps use the same pattern
    # for consistency. Users iterating on submodule source should use
    # -NoSubmoduleSync instead of relying on working-tree preservation.
    git submodule update --init --recursive --force -- $RepoDir
    if ($LASTEXITCODE -ne 0) { throw "git submodule update failed for MuJoCo - is this a git checkout? Pass -NoSubmoduleSync to skip." }
}

# Capture the SHA we are about to build from so URLab.Build.cs can detect
# a stale install on the next UE build (install SHA vs current submodule HEAD).
$InstalledSha = (git -C $RepoDir rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or -not $InstalledSha) { throw "Failed to read HEAD SHA from $RepoDir - is the submodule initialised?" }

Push-Location $RepoDir

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
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$($BuildType.Replace('Release', '').Replace('Debug', 'Debug'))DLL",
    # MuJoCo patches its fetched qhull with `git apply --reject`, which is not
    # idempotent: applied twice it rejects every hunk and returns non-zero. On
    # the Visual Studio generator MSBuild re-invokes CMake mid-build once the
    # FetchContent deps have landed, so a build from a clean tree would re-run
    # that patch and fail. Suppressing regeneration keeps the one successful
    # configure authoritative for the rest of the build.
    "-DCMAKE_SUPPRESS_REGENERATION=ON"
)
cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed for MuJoCo" }

Write-Host "Building MuJoCo..." -ForegroundColor Gray
cmake --build . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Build failed for MuJoCo" }

Write-Host "Installing MuJoCo..." -ForegroundColor Gray
cmake --install . --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "Installation failed for MuJoCo" }

# MuJoCo builds its first-party plugins (PID actuator, elasticity, sensor, SDF)
# as separate shared libraries but installs none of them -- upstream expects
# `simulate` to pick them up out of the build tree. URLab loads them at module
# startup, so they have to be part of the install: a model naming `mujoco.pid`
# does not build without one.
$PluginOut = Join-Path $InstallDir "bin/mujoco_plugin"
New-Item -ItemType Directory -Force -Path $PluginOut | Out-Null
$PluginBuildDir = Join-Path (Get-Location) "bin/$BuildType"
$PluginLibs = Get-ChildItem -Path $PluginBuildDir -Filter "*.dll" -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -ne "mujoco.dll" }
foreach ($Lib in $PluginLibs) {
    Copy-Item -Path $Lib.FullName -Destination $PluginOut -Force
    Write-Host "Installed plugin $($Lib.Name)" -ForegroundColor Gray
}
if (-not $PluginLibs) {
    Write-Host "WARNING: no MuJoCo plugin libraries found in $PluginBuildDir" -ForegroundColor Yellow
}

Pop-Location

# Record the exact source SHA we just installed from. URLab.Build.cs reads
# this and compares it to the submodule's current HEAD to flag "updated
# submodules but forgot to rebuild" drift.
$ShaFile = Join-Path $InstallDir "INSTALLED_SHA.txt"
[System.IO.File]::WriteAllText($ShaFile, $InstalledSha)
Write-Host "Recorded INSTALLED_SHA=$InstalledSha at $ShaFile" -ForegroundColor Gray
