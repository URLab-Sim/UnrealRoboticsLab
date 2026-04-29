#!/bin/bash
# Master Build Script for URLab Third-Party Dependencies (Linux/macOS)
#
# Each dep's build.sh will sync its submodule (third_party/<dep>/src) to the
# SHA URLab expects before building. Pass --no-submodule-sync to skip the
# sync across all three deps, e.g. when iterating on a submodule locally.
#
# Linux note: pass --engine <UE_ROOT> to build against UE's bundled clang +
# libc++. Without it the system gcc + libstdc++ are used, which produces
# .so files whose C++ ABI doesn't match UE's plugin link (you'll get a wall
# of std::* undefined-symbol errors during the URLab plugin link step).
# Worse, with some toolchain combinations the system gcc emits LTO bitcode
# objects rather than real .so files, which fail to load with a cryptic
# "file too short" at editor startup. The --engine path makes the toolchain
# explicit.

set -e

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
INSTALL_DIR="$ROOT_DIR/install"
BUILD_TYPE="Release"
ENGINE=""

usage() {
    cat >&2 <<'HELP'
build_all.sh — build URLab's third-party dependencies (MuJoCo, CoACD, libzmq).

Usage:
  ./third_party/build_all.sh [--engine <UE_ROOT>] [--no-submodule-sync]

Options:
  --engine <path>        Linux only: point CC/CXX/AR/RANLIB at UE's bundled
                         clang + libc++ under <UE_ROOT>/Engine/Extras/...,
                         and inject the matching CFLAGS/CXXFLAGS/LDFLAGS so
                         the resulting .so files are ABI-compatible with
                         UE's plugin link. Strongly recommended on Linux.
  --no-submodule-sync    Skip `git submodule update` on each dep's src/.
                         Use when iterating on a submodule locally.
HELP
    exit 3
}

# Pre-parse args. We collect --no-submodule-sync into SHARED_ARGS so it
# propagates to each per-dep build.sh; --engine is consumed here and used
# only to set CC/CXX/CFLAGS/etc.
SHARED_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --engine)             ENGINE="$2"; shift 2 ;;
        --no-submodule-sync)  SHARED_ARGS+=("--no-submodule-sync"); shift ;;
        -h|--help)            usage ;;
        *) echo "Unknown arg: $1" >&2; usage ;;
    esac
done

# Linux: when --engine is given, locate UE's bundled clang under
# Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/<v_clang_dir>/
# x86_64-unknown-linux-gnu/. Glob the v*_clang-* directory so future UE
# bumps (v26 -> v27) survive without a script edit.
if [[ -n "$ENGINE" && "$(uname -s)" = "Linux" ]]; then
    SDK_BASE="$ENGINE/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64"
    if [[ ! -d "$SDK_BASE" ]]; then
        echo "ERROR: --engine path doesn't contain UE's Linux SDK base:" >&2
        echo "  expected: $SDK_BASE" >&2
        exit 3
    fi
    # Pick the highest-numbered v*_clang-* toolchain. Bash globs sort
    # lexicographically so v25 sorts before v26 — version-aware sort with
    # `sort -V` ensures v27 wins over v26 on a future UE bump. Latent today
    # since UE ships one toolchain per engine version, but cheap insurance.
    UE_TC=$(ls -d "$SDK_BASE"/v*_clang-*/x86_64-unknown-linux-gnu 2>/dev/null | sort -V | tail -1)
    if [[ -z "$UE_TC" || ! -x "$UE_TC/bin/clang++" ]]; then
        echo "ERROR: no v*_clang-*/x86_64-unknown-linux-gnu toolchain found under $SDK_BASE" >&2
        exit 3
    fi
    echo "Using UE toolchain: $UE_TC"

    export CC="$UE_TC/bin/clang"
    export CXX="$UE_TC/bin/clang++"
    export AR="$UE_TC/bin/llvm-ar"
    export RANLIB="$UE_TC/bin/llvm-ranlib"

    # Notes on the CXX/LDFLAGS choices:
    # -nostdinc++ -isystem <ue_libcxx>: force libc++ headers from UE's SDK
    #     so we don't accidentally pick up libstdc++ headers from the host.
    # -stdlib=libc++: link against UE's libc++ at link time.
    # -fuse-ld=lld: lld is what UE uses; avoids surprises on plugin link.
    # -Qunused-arguments: silence the "argument unused during compilation"
    #     warning -stdlib=libc++ produces on .c-mode compile-only steps
    #     (some MuJoCo deps build with -Werror).
    # -Wno-unknown-warning-option: lets clang ignore -Werror=stringop-overflow
    #     and similar GCC-only flags TBB (CoACD's transitive dep) uses.
    # -Wno-missing-template-arg-list-after-template-kw: clang 20 (UE 5.7's
    #     bundled) rejects OpenVDB's `OpT::template eval(...)` syntax that
    #     older clang accepted. CoACD pulls OpenVDB transitively.
    export CFLAGS="-fPIC -Qunused-arguments -Wno-unknown-warning-option"
    export CXXFLAGS="-stdlib=libc++ -nostdinc++ -isystem $UE_TC/include/c++/v1 -fPIC -Qunused-arguments -Wno-unknown-warning-option -Wno-missing-template-arg-list-after-template-kw"
    export LDFLAGS="-stdlib=libc++ -fuse-ld=lld -L$UE_TC/lib64 -Wl,-rpath,$UE_TC/lib64"
fi

mkdir -p "$INSTALL_DIR"

echo -e "\e[36mStarting Unified Build Process...\e[0m"

cd "$ROOT_DIR"

# 1. CoACD
echo -e "\n\e[33m--- Building CoACD ---\e[0m"
cd CoACD
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE" "${SHARED_ARGS[@]}"
else
    echo "Warning: CoACD/build.sh not found!"
fi
cd "$ROOT_DIR"

# 2. MuJoCo
echo -e "\n\e[33m--- Building MuJoCo ---\e[0m"
cd MuJoCo
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE" "${SHARED_ARGS[@]}"
else
    echo "Warning: MuJoCo/build.sh not found!"
fi
cd "$ROOT_DIR"

# 3. libzmq
echo -e "\n\e[33m--- Building libzmq ---\e[0m"
cd libzmq
if [ -f "./build.sh" ]; then
    bash ./build.sh "$INSTALL_DIR" "$BUILD_TYPE" "${SHARED_ARGS[@]}"
else
    echo "Warning: libzmq/build.sh not found!"
fi
cd "$ROOT_DIR"

echo -e "\n\e[32mAll builds completed! check the 'install' folder.\e[0m"
