# Build gRPC v1.62.0 as static libs for Windows (MSVC), staged under
# third_party/install/grpc so URLabDmEnvRpc.Build.cs can link it. Windows twin of
# build.sh -- same targets and staging, just MSVC/.lib instead of gcc/.a.
#
# CRITICAL: built with the DYNAMIC CRT (/MD) to match Unreal's runtime -- a static
# CRT (/MT) gRPC will not link into a UE module. Needs git, cmake (>=3.16), and
# Visual Studio 2022 (v143). cmake finds MSVC itself; no dev-prompt required.
param(
    [string]$BuildType = "Release",
    [string]$Generator = "Visual Studio 17 2022"
)
$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Install   = Join-Path $ScriptDir "..\install\grpc"
$Src       = Join-Path $ScriptDir "src"
# Build dir sits BESIDE src, not inside it: gRPC ships a root-level file named
# 'build', so src/build would collide. third_party/grpc/build is gitignored.
$BuildDir  = Join-Path $ScriptDir "build"

if (Test-Path $Install) {
    Write-Host "Removing previous install at $Install"
    Remove-Item -Recurse -Force $Install
}

# gRPC's submodules nest deep enough to blow past Windows' 260-char MAX_PATH, so
# every git call forces core.longpaths (per-command, no global config change).
# Clone then submodule-update are both idempotent -- a re-run resumes a partial tree.
if (-not (Test-Path (Join-Path $Src "CMakeLists.txt"))) {
    Write-Host "Cloning gRPC v1.62.0 (top-level)..."
    git -c core.longpaths=true clone --depth 1 --branch v1.62.0 `
        https://github.com/grpc/grpc.git $Src
}
Write-Host "Checking out submodules (long-paths on)..."
Push-Location $Src
try {
    git -c core.longpaths=true submodule update --init --recursive --depth 1
    if ($LASTEXITCODE -ne 0) { throw "submodule update failed (exit $LASTEXITCODE)" }
}
finally {
    Pop-Location
}

[System.IO.Directory]::CreateDirectory($BuildDir) | Out-Null
Push-Location $BuildDir
try {
    Write-Host "Configuring gRPC (MSVC, /MD dynamic CRT to match UE)..."
    # cmake 4.x rejects the pre-3.5 cmake_minimum_required() in gRPC 1.62's bundled
    # deps. The env-var form is honored verbatim (a -D flag gets its '.5' mangled by
    # PowerShell native-arg parsing, arriving as an invalid "3").
    $env:CMAKE_POLICY_VERSION_MINIMUM = "3.5"
    cmake "$Src" -G $Generator -A x64 `
        -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL `
        -DgRPC_MSVC_STATIC_RUNTIME=OFF `
        -DBUILD_SHARED_LIBS=OFF `
        -DZLIB_BUILD_SHARED=OFF `
        -DABSL_PROPAGATE_CXX_STD=ON `
        -DgRPC_BUILD_TESTS=OFF `
        -DRE2_BUILD_TESTING=OFF `
        -Dprotobuf_BUILD_TESTS=OFF `
        -DABSL_BUILD_TESTING=OFF `
        -Dutf8_range_ENABLE_TESTS=OFF `
        -DCARES_BUILD_TESTS=OFF `
        -DgRPC_BUILD_CSHARP_EXT=OFF `
        -DgRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF `
        -DgRPC_BUILD_GRPC_NODE_PLUGIN=OFF `
        -DgRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN=OFF `
        -DgRPC_BUILD_GRPC_PHP_PLUGIN=OFF `
        -DgRPC_BUILD_GRPC_PYTHON_PLUGIN=OFF `
        -DgRPC_BUILD_GRPC_RUBY_PLUGIN=OFF `
        -DgRPC_INSTALL=OFF `
        -Dprotobuf_INSTALL=OFF `
        -Dutf8_range_ENABLE_INSTALL=OFF `
        -DABSL_ENABLE_INSTALL=OFF
    if ($LASTEXITCODE -ne 0) { throw "cmake configure failed (exit $LASTEXITCODE)" }

    Write-Host "Building gRPC + Protobuf static libraries (this takes a while)..."
    cmake --build . --config $BuildType --target grpc++ grpc gpr libprotobuf --parallel
    if ($LASTEXITCODE -ne 0) { throw "cmake build failed (exit $LASTEXITCODE)" }
}
finally {
    Pop-Location
}

Write-Host "Staging headers + static libs into $Install..."
$Inc = Join-Path $Install "include"
$Lib = Join-Path $Install "lib"
[System.IO.Directory]::CreateDirectory($Inc) | Out-Null
[System.IO.Directory]::CreateDirectory($Lib) | Out-Null

# Headers (mirror build.sh): grpc, grpcpp, protobuf, absl, re2, openssl.
$HeaderCopies = @(
    @{ From = (Join-Path $Src "include\grpc");                                  To = $Inc },
    @{ From = (Join-Path $Src "include\grpcpp");                                To = $Inc },
    @{ From = (Join-Path $Src "third_party\protobuf\src\google");               To = $Inc },
    @{ From = (Join-Path $Src "third_party\abseil-cpp\absl");                   To = $Inc },
    @{ From = (Join-Path $Src "third_party\re2\re2");                           To = $Inc },
    @{ From = (Join-Path $Src "third_party\boringssl-with-bazel\src\include\openssl"); To = $Inc }
)
foreach ($h in $HeaderCopies) {
    if (Test-Path $h.From) {
        Copy-Item -Recurse -Force $h.From $h.To
    }
    else {
        Write-Warning "header dir not found (skipped): $($h.From)"
    }
}

# Every static lib produced anywhere in the build tree (grpc++ pulls in absl,
# boringssl, cares, re2, upb, zlib, protobuf as deps, so their .lib files are here).
$LibFiles = Get-ChildItem -Path $BuildDir -Recurse -Filter *.lib -File
foreach ($f in $LibFiles) {
    Copy-Item -Force $f.FullName (Join-Path $Lib $f.Name)
}

Write-Host "gRPC installed to $Install"
Write-Host ("Staged {0} static libs. First few:" -f $LibFiles.Count)
Get-ChildItem (Join-Path $Lib "*.lib") | Select-Object -First 8 Name | Format-Table -HideTableHeaders
