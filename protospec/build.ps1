# Build ProtoSpec's static libraries and stage them where URLab.Build.cs looks.
#
# ProtoSpec is URLab's own code, not a vendored dependency, so it builds on its
# own rather than as a step of the MuJoCo build. That separation is the point:
# a change here used to tear down and reconfigure the entire MuJoCo install to
# recompile one file.
#
# Of the MuJoCo-free targets, all are built -- the object model, the
# canonicalization resolvers and the MJCF reader/writer. The compile bridge
# links the engine and is not part of URLab's link line, because URLab already
# links MuJoCo itself.
#
# The one MuJoCo-dependent target that is built is the comparison harness
# (protospec_harness), which URLab's compile-parity goldens call to diff two
# mjModels field by field. It needs mujoco.h and nothing else that URLab does
# not already link, so the staged MuJoCo install is enough to build it.

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
cmake --build $Build --config $BuildType --target protospec protospec_core protospec_io protospec_harness
if ($LASTEXITCODE -ne 0) { throw "Build failed for ProtoSpec" }

# Staging is explicit because ProtoSpec's CMake declares no install() rules.
# The lib/ header layout is mirrored rather than flattened: the umbrella headers
# reach the generated tables through relative paths ("../../generated/types.h"),
# which only resolve in the original shape. URLab.Build.cs adds each of these
# directories to the include path, reproducing what ProtoSpec's own CMake
# target_include_directories does.
#
# Staged last, so a failed build leaves the previous install in place rather
# than none at all.
Write-Host "Staging ProtoSpec into $ProtospecInstallDir..." -ForegroundColor Gray
# The harness is its own CMake subdirectory, so its archive lands one level
# deeper than the rest; both output directories are collected.
$LibDirs = @("$Build/$BuildType", "$Build/harness/$BuildType")
$BuiltLibs = @($LibDirs | Where-Object { Test-Path $_ } |
    ForEach-Object { Get-ChildItem -Path $_ -Filter "*.lib" -File })
if (-not $BuiltLibs) { throw "ProtoSpec built no static libraries under $Build" }

if (Test-Path $ProtospecInstallDir) { Remove-Item -Recurse -Force $ProtospecInstallDir }
$Lib = Join-Path $ProtospecInstallDir "lib"
New-Item -ItemType Directory -Force -Path $Lib | Out-Null

foreach ($dir in @("include", "sdk", "generated", "core", "io", "compile", "validate", "harness")) {
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

Write-Host "ProtoSpec staged: $($BuiltLibs.Count) libraries, headers under $ProtospecInstallDir" -ForegroundColor Gray
