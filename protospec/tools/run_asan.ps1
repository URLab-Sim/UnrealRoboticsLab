<#
.SYNOPSIS
  AddressSanitizer pass over the ProtoSpec C++ tree (Windows / MSVC).

.DESCRIPTION
  Builds a SEPARATE instrumented tree in lib/build_asan with MSVC
  /fsanitize=address, runs every instrumentable ctest suite under it, and
  fuzz-exercises the read/write path by running ps_roundtrip over a diverse
  corpus sample. The normal lib/build tree is never touched.

  SCOPE (important): only the MuJoCo-free surface is instrumented -- the object
  model, io (MJCF reader/writer), core resolvers, validate, and the SDK
  (including the save surface). The MuJoCo-linked suites (bridge, native/compile,
  ps_native_diff) CANNOT be built under MSVC /fsanitize=address: any TU that
  includes <mujoco/mujoco.h> pulls in vendored mujoco/mjsan.h, whose
  ADDRESS_SANITIZER path (a) uses GCC/Clang __attribute__/asm syntax cl.exe
  rejects and (b) references mj__markStack/mj__freeStack symbols that exist only
  in a libmujoco that was itself built under ASan -- the vendored mujoco.dll is a
  normal build. Instrumenting those suites needs an ASan-built MuJoCo (or the
  ClangCL toolset, which parses mjsan.h). See HANDOFF.md "ASan".

.PARAMETER CorpusCount
  Number of diverse corpus files to run ps_roundtrip over (default 60, floor 50).

.PARAMETER Corpus
  Corpus root (defaults to $env:PROTOSPEC_CORPUS, then the vendored MuJoCo src).

.PARAMETER SkipConfigure
  Reuse an existing lib/build_asan CMake cache.

.PARAMETER SkipBuild
  Reuse already-built instrumented binaries.

.EXAMPLE
  pwsh tools/run_asan.ps1
  pwsh tools/run_asan.ps1 -CorpusCount 120
#>
[CmdletBinding()]
param(
  [int]$CorpusCount = 60,
  [string]$Corpus = $env:PROTOSPEC_CORPUS,
  [switch]$SkipConfigure,
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
if ($CorpusCount -lt 50) { $CorpusCount = 50 }

$Root     = Split-Path -Parent $PSScriptRoot
$BuildDir = Join-Path $Root "lib/build_asan"
$RelDir   = Join-Path $BuildDir "Release"
$LogDir   = Join-Path $BuildDir "asan_logs"

# The instrumentable (MuJoCo-free) targets and their ctest suites.
$Targets = @(
  "protospec_tests", "protospec_io_tests",
  "protospec_validate_tests", "protospec_sdk_tests",
  "ps_roundtrip", "ps_validate"
)
$Suites = @(
  "protospec_tests", "protospec_io_tests",
  "protospec_validate_tests", "protospec_sdk_tests"
)

# ASan runtime knobs: never abort (so we can triage every file), surface stack
# reuse bugs, and force a distinct nonzero exit on a real report.
$env:ASAN_OPTIONS = "abort_on_error=0:exitcode=1:detect_stack_use_after_return=1:print_stats=0"

function Resolve-Corpus {
  if ($Corpus -and (Test-Path $Corpus)) { return $Corpus }
  $vendored = "C:\Users\jonat\Documents\Unreal Projects\url_proj\Plugins\UnrealRoboticsLab\third_party\MuJoCo\src"
  if (Test-Path $vendored) { return $vendored }
  return $null
}

# The clang_rt ASan runtime DLL from the exact toolset CMake selected (its dir is
# the CMAKE_LINKER directory), copied beside the test exes so they load it.
function Copy-AsanRuntime {
  $line = Get-Content (Join-Path $BuildDir "CMakeCache.txt") |
          Where-Object { $_ -match '^CMAKE_LINKER:FILEPATH=' } | Select-Object -First 1
  if (-not $line) { throw "CMAKE_LINKER not found in cache; configure first." }
  $dir = Split-Path (($line -split '=', 2)[1])
  $dll = Join-Path $dir "clang_rt.asan_dynamic-x86_64.dll"
  if (-not (Test-Path $dll)) { throw "ASan runtime not found: $dll" }
  Copy-Item $dll $RelDir -Force
  Write-Host "  ASan runtime: $dll"
}

# ---- configure ------------------------------------------------------------- #
if (-not $SkipConfigure) {
  Write-Host "== configure (MSVC /fsanitize=address) =="
  # /Zi gives ASan usable stack traces; TRY_COMPILE=Release keeps /RTC1 (which is
  # incompatible with ASan) out of CMake's Debug-default compiler probe.
  cmake -S (Join-Path $Root "lib") -B $BuildDir `
        "-DCMAKE_CXX_FLAGS=/fsanitize=address /Zi" `
        "-DCMAKE_TRY_COMPILE_CONFIGURATION=Release" | Out-Null
  if ($LASTEXITCODE -ne 0) { throw "configure failed" }
}

# ---- build ----------------------------------------------------------------- #
if (-not $SkipBuild) {
  Write-Host "== build instrumented (MuJoCo-free) targets =="
  foreach ($t in $Targets) {
    cmake --build $BuildDir --config Release --target $t | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "build failed: $t" }
  }
}
Copy-AsanRuntime
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

# ---- ctest suites ---------------------------------------------------------- #
Write-Host "`n== ctest suites under ASan =="
$fail = 0; $asanHits = 0
foreach ($s in $Suites) {
  $exe = Join-Path $RelDir "$s.exe"
  $out = & $exe 2>&1
  $code = $LASTEXITCODE
  $hit = ($out | Select-String "AddressSanitizer").Count
  if ($hit -gt 0) { $asanHits += $hit; $out | Set-Content (Join-Path $LogDir "$s.asan.txt") }
  if ($code -ne 0 -or $hit -gt 0) { $fail++ }
  $last = ($out | Select-Object -Last 1)
  "{0,-28} exit={1,-4} asan={2}  {3}" -f $s, $code, $hit, $last | Write-Host
}

# ---- corpus: ps_roundtrip over a diverse sample ---------------------------- #
Write-Host "`n== ps_roundtrip corpus under ASan =="
$corpusRoot = Resolve-Corpus
$roundtrip = Join-Path $RelDir "ps_roundtrip.exe"
$ok = 0; $skip = 0; $perr = 0; $crash = 0; $corpusAsan = 0; $ran = 0
if (-not $corpusRoot) {
  Write-Host "  corpus not found (set PROTOSPEC_CORPUS) -- skipping corpus pass"
} else {
  $all = Get-ChildItem -Path $corpusRoot -Recurse -Filter *.xml -File |
         Where-Object { $_.FullName -notmatch '\\build\\' -and $_.Name -notlike '._ps_*' } |
         Sort-Object FullName
  $stride = [Math]::Max(1, [int]($all.Count / $CorpusCount))
  $sample = New-Object System.Collections.Generic.List[object]
  for ($i = 0; $i -lt $all.Count -and $sample.Count -lt $CorpusCount; $i += $stride) {
    $sample.Add($all[$i])
  }
  Write-Host "  corpus=$($all.Count) files; sampling $($sample.Count) (stride $stride)"
  foreach ($f in $sample) {
    $ran++
    $o = & $roundtrip $f.FullName 2>&1
    $code = $LASTEXITCODE
    if (($o | Select-String "AddressSanitizer").Count -gt 0) {
      $corpusAsan++
      $o | Set-Content (Join-Path $LogDir ("roundtrip_" + $f.BaseName + ".asan.txt"))
      Write-Host "  ASAN REPORT: $($f.FullName)"
    }
    switch ($code) {
      0 { $ok++ }
      3 { $skip++ }
      1 { $perr++ }
      default { $crash++; Write-Host "  CRASH(exit=$code): $($f.FullName)" }
    }
  }
  Write-Host "  ps_roundtrip: ok=$ok skip-unsupported=$skip parse-err=$perr crash=$crash asan_reports=$corpusAsan"
}

# ---- verdict --------------------------------------------------------------- #
$totalAsan = $asanHits + $corpusAsan
Write-Host "`n== ASan summary =="
Write-Host "  suites failed/reported : $fail / $($Suites.Count)"
Write-Host "  suite asan reports     : $asanHits"
Write-Host "  corpus files exercised : $ran"
Write-Host "  corpus asan reports    : $corpusAsan"
Write-Host "  crashes                : $crash"
if ($totalAsan -eq 0 -and $fail -eq 0 -and $crash -eq 0) {
  Write-Host "  RESULT: CLEAN (no AddressSanitizer reports)" -ForegroundColor Green
  exit 0
} else {
  Write-Host "  RESULT: ISSUES (see $LogDir)" -ForegroundColor Red
  exit 1
}
