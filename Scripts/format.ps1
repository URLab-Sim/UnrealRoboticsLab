# Format URLab C++ with the repo .clang-format (Epic's canonical UE style).
#
# Usage:
#   Scripts/format.ps1            # format Source/ in place
#   Scripts/format.ps1 -Check     # exit 1 if anything is unformatted (CI)
#
# Codegen-managed files are also formatted here, but the codegen emits the
# same formatting (see _clang_format_content in generate_ue_components.py), so
# running this after a regen is a no-op on generated files.
#
# Requires clang-format 19.x (VS2022's bundled LLVM, or set $env:CLANG_FORMAT).
param([switch]$Check)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function Resolve-ClangFormat {
    if ($env:CLANG_FORMAT -and (Test-Path $env:CLANG_FORMAT)) { return $env:CLANG_FORMAT }
    $onPath = Get-Command clang-format -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }
    $vs = "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/bin/clang-format.exe"
    if (Test-Path $vs) { return $vs }
    throw "clang-format not found. Set `$env:CLANG_FORMAT or add it to PATH (clang-format 19.x)."
}

$cf = Resolve-ClangFormat
$files = Get-ChildItem -Path (Join-Path $root 'Source') -Recurse -Include *.h, *.cpp -File |
    Where-Object { $_.FullName -notmatch '[\\/](Intermediate|Binaries)[\\/]' }

Write-Host "clang-format: $cf"
Write-Host "files: $($files.Count)"

if ($Check) {
    $bad = @()
    foreach ($f in $files) {
        & $cf --style=file --dry-run --Werror $f.FullName 2>$null
        if ($LASTEXITCODE -ne 0) { $bad += $f.FullName }
    }
    if ($bad.Count) {
        Write-Host "UNFORMATTED ($($bad.Count)):" -ForegroundColor Red
        $bad | ForEach-Object { Write-Host "  $_" }
        exit 1
    }
    Write-Host "All formatted." -ForegroundColor Green
    exit 0
}

$files | ForEach-Object { & $cf -i --style=file $_.FullName }
Write-Host "Formatted $($files.Count) files." -ForegroundColor Green
