# Build ProtoSpec's static libraries and stage them where URLab.Build.cs looks.
#
# ProtoSpec is URLab's own code, not a vendored dependency, so it builds on its
# own rather than as a step of the MuJoCo build. That separation is the point:
# a change here used to tear down and reconfigure the entire MuJoCo install to
# recompile one file.
#
# Staged code is deliberately narrow: native-backed spatial resolvers,
# XML include expansion, tinyxml2, and the native mjModel comparison helper
# used by URLab editor tests. There is no standalone authoring AST or SDK.

param(
    [string]$InstallDir = "../third_party/install",
    [string]$MujocoRoot = "../third_party/install/MuJoCo",
    [string]$BuildType = "Release"
)

$ErrorActionPreference = "Stop"

# Anchored on the script, not the caller's working directory, so it behaves the
# same however it is invoked.
if (-not [System.IO.Path]::IsPathRooted($InstallDir)) {
    $InstallDir = Join-Path $PSScriptRoot $InstallDir
}
$InstallRoot = [System.IO.Path]::GetFullPath($InstallDir).Replace('\', '/')
$ProtospecInstallDir = "$InstallRoot/protospec"

if (-not [System.IO.Path]::IsPathRooted($MujocoRoot)) {
    $MujocoRoot = Join-Path $PSScriptRoot $MujocoRoot
}
$MujocoRoot = [System.IO.Path]::GetFullPath($MujocoRoot).Replace('\', '/')
if (-not (Test-Path "$MujocoRoot/include/mujoco/mujoco.h")) {
    throw "No MuJoCo headers under $MujocoRoot. Run third_party/build_all.ps1 first, or pass -MujocoRoot."
}

$Src = Join-Path $PSScriptRoot "lib"
if (-not (Test-Path (Join-Path $Src "CMakeLists.txt"))) {
    throw "No ProtoSpec sources at $Src."
}
$Src = [System.IO.Path]::GetFullPath($Src).Replace('\', '/')
$Build = "$Src/build-urlab"

Write-Host "Resolved install: $ProtospecInstallDir" -ForegroundColor Gray
Write-Host "Configuring ProtoSpec from $Src..." -ForegroundColor Gray
cmake -S $Src -B $Build -DCMAKE_BUILD_TYPE=$BuildType "-DMUJOCO_ROOT=$MujocoRoot" `
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$($BuildType.Replace('Release', '').Replace('Debug', 'Debug'))DLL"
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed for ProtoSpec" }

Write-Host "Building ProtoSpec..." -ForegroundColor Gray
cmake --build $Build --config $BuildType --target protospec_core protospec_mjcf protospec_harness urlab_mj_deps
if ($LASTEXITCODE -ne 0) { throw "Build failed for ProtoSpec" }

# Stage named archives and only the headers required by the direct UE gateway.
# The comparison helper is test-only and lives in a CMake subdirectory.
$StagedLibs = @("protospec_core", "protospec_mjcf", "tinyxml2", "protospec_harness")
$LibDirs = @("$Build/$BuildType", "$Build/harness/$BuildType")
$BuiltLibs = @($StagedLibs | ForEach-Object {
    $name = "$_.lib"
    $found = $LibDirs | ForEach-Object { Join-Path $_ $name } | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $found) { throw "ProtoSpec did not build $name under $Build" }
    Get-Item $found
})

if (Test-Path $ProtospecInstallDir) { Remove-Item -Recurse -Force $ProtospecInstallDir }
$Lib = Join-Path $ProtospecInstallDir "lib"
New-Item -ItemType Directory -Force -Path $Lib | Out-Null

foreach ($dir in @("include", "core", "io", "harness")) {
    $dirSrc = Join-Path $Src $dir
    if (-not (Test-Path $dirSrc)) { continue }
    $dirSrcFull = [System.IO.Path]::GetFullPath($dirSrc)
    Get-ChildItem -Path $dirSrc -Include "*.h", "*.inc" -File -Recurse | ForEach-Object {
        $rel = $_.FullName.Substring($dirSrcFull.Length).TrimStart('\', '/')
        $dest = Join-Path (Join-Path $ProtospecInstallDir $dir) $rel
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dest) | Out-Null
        Copy-Item -Force $_.FullName $dest
    }
}

$TinyXml = Join-Path $ProtospecInstallDir "third_party/tinyxml2"
New-Item -ItemType Directory -Force -Path $TinyXml | Out-Null
Copy-Item -Force "$Src/third_party/tinyxml2/tinyxml2.h" $TinyXml

$BuiltLibs | ForEach-Object { Copy-Item -Force $_.FullName $Lib }

$HelperExe = Join-Path $Build "$BuildType/urlab_mj_deps.exe"
if (-not (Test-Path $HelperExe)) { throw "ProtoSpec did not build $HelperExe" }
$Bin = Join-Path $ProtospecInstallDir "bin"
New-Item -ItemType Directory -Force -Path $Bin | Out-Null
Copy-Item -Force $HelperExe $Bin
$MujocoDll = Join-Path $MujocoRoot "bin/mujoco.dll"
if (Test-Path $MujocoDll) { Copy-Item -Force $MujocoDll $Bin }

Write-Host "ProtoSpec staged: $($BuiltLibs.Count) libraries, dependency helper, headers under $ProtospecInstallDir" -ForegroundColor Gray
