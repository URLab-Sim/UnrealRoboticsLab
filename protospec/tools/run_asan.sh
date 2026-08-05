#!/usr/bin/env bash
# AddressSanitizer pass over the ProtoSpec C++ tree (Linux / gcc or clang).
#
# Linux companion of tools/run_asan.ps1. Builds a SEPARATE instrumented tree in
# lib/build_asan with -fsanitize=address, runs every instrumentable ctest suite
# under it, and fuzz-exercises the read/write path with ps_roundtrip over a
# diverse corpus sample. The normal lib/build tree is untouched.
#
# SCOPE: only the MuJoCo-free surface is instrumented (object model, io, core,
# validate, SDK incl. the save surface). A TU including <mujoco/mujoco.h> pulls
# in vendored mujoco/mjsan.h, whose ASan path references mj__markStack /
# mj__freeStack -- symbols present only in a libmujoco that was itself built
# under ASan. The vendored MuJoCo is a normal build, so the bridge / native /
# ps_native_diff suites need an ASan-built MuJoCo to instrument. See HANDOFF.md.
#
# Usage: tools/run_asan.sh [CORPUS_COUNT]   (default 60, floor 50)
#   env PROTOSPEC_CORPUS  corpus root (else the vendored MuJoCo src is tried)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT/lib/build_asan"
LOG_DIR="$BUILD_DIR/asan_logs"
COUNT="${1:-60}"
[ "$COUNT" -lt 50 ] && COUNT=50

TARGETS=(protospec_tests protospec_io_tests protospec_validate_tests
         protospec_sdk_tests ps_roundtrip ps_validate)
SUITES=(protospec_tests protospec_io_tests protospec_validate_tests protospec_sdk_tests)

export ASAN_OPTIONS="abort_on_error=0:exitcode=1:detect_stack_use_after_return=1:detect_leaks=0"

echo "== configure (-fsanitize=address) =="
cmake -S "$ROOT/lib" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address" >/dev/null

echo "== build instrumented (MuJoCo-free) targets =="
for t in "${TARGETS[@]}"; do
  cmake --build "$BUILD_DIR" --target "$t" >/dev/null
done

# Locate built exes (single-config generators drop them in the build root).
find_exe() { find "$BUILD_DIR" -type f -name "$1" ! -path '*/CMakeFiles/*' | head -n1; }

mkdir -p "$LOG_DIR"
echo
echo "== ctest suites under ASan =="
fail=0; asan_hits=0
for s in "${SUITES[@]}"; do
  exe="$(find_exe "$s")"
  out="$("$exe" 2>&1)"; code=$?
  hit="$(printf '%s' "$out" | grep -c 'AddressSanitizer' || true)"
  if [ "$hit" -gt 0 ]; then asan_hits=$((asan_hits+hit)); printf '%s\n' "$out" > "$LOG_DIR/$s.asan.txt"; fi
  if [ "$code" -ne 0 ] || [ "$hit" -gt 0 ]; then fail=$((fail+1)); fi
  printf '%-28s exit=%-4s asan=%s  %s\n' "$s" "$code" "$hit" "$(printf '%s' "$out" | tail -n1)"
done

echo
echo "== ps_roundtrip corpus under ASan =="
CORPUS="${PROTOSPEC_CORPUS:-}"
if [ -z "$CORPUS" ] || [ ! -d "$CORPUS" ]; then
  CORPUS="/opt/mujoco/src"  # best-effort default; override with PROTOSPEC_CORPUS
fi
ok=0; skip=0; perr=0; crash=0; corpus_asan=0; ran=0
if [ ! -d "$CORPUS" ]; then
  echo "  corpus not found (set PROTOSPEC_CORPUS) -- skipping corpus pass"
else
  rt="$(find_exe ps_roundtrip)"
  mapfile -t all < <(find "$CORPUS" -name '*.xml' -type f ! -path '*/build/*' ! -name '._ps_*' | sort)
  stride=$(( ${#all[@]} / COUNT )); [ "$stride" -lt 1 ] && stride=1
  echo "  corpus=${#all[@]} files; sampling up to $COUNT (stride $stride)"
  i=0
  while [ "$i" -lt "${#all[@]}" ] && [ "$ran" -lt "$COUNT" ]; do
    f="${all[$i]}"; i=$((i+stride)); ran=$((ran+1))
    out="$("$rt" "$f" 2>&1)"; code=$?
    if printf '%s' "$out" | grep -q 'AddressSanitizer'; then
      corpus_asan=$((corpus_asan+1)); printf '%s\n' "$out" > "$LOG_DIR/roundtrip_$(basename "$f" .xml).asan.txt"
      echo "  ASAN REPORT: $f"
    fi
    case "$code" in
      0) ok=$((ok+1));; 3) skip=$((skip+1));; 1) perr=$((perr+1));;
      *) crash=$((crash+1)); echo "  CRASH(exit=$code): $f";;
    esac
  done
  echo "  ps_roundtrip: ok=$ok skip-unsupported=$skip parse-err=$perr crash=$crash asan_reports=$corpus_asan"
fi

echo
echo "== ASan summary =="
echo "  suites failed/reported : $fail / ${#SUITES[@]}"
echo "  suite asan reports     : $asan_hits"
echo "  corpus files exercised : $ran"
echo "  corpus asan reports    : $corpus_asan"
echo "  crashes                : $crash"
total=$((asan_hits + corpus_asan))
if [ "$total" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$crash" -eq 0 ]; then
  echo "  RESULT: CLEAN (no AddressSanitizer reports)"; exit 0
else
  echo "  RESULT: ISSUES (see $LOG_DIR)"; exit 1
fi
