param(
    [string]$InstallDir = "../install",
    [string]$BuildType = "Release"
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

$RepoDir = "src"
if (-not (Test-Path $RepoDir)) {
    Write-Host "Cloning libzmq..." -ForegroundColor Gray
    git clone https://github.com/zeromq/libzmq $RepoDir
}

Push-Location $RepoDir
Write-Host "Initializing submodules..." -ForegroundColor Gray
git submodule update --init --recursive
if ($LASTEXITCODE -ne 0) { Write-Warning "Submodule update failed for libzmq" }
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
