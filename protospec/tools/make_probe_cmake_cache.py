"""Prepare a host-runnable defaults probe for a UE-toolchain MuJoCo install.

The UE Linux SDK builds a shared MuJoCo library against static libc++ but
does not record those archives as ELF dependencies. A standalone probe needs
the same libc++ archives linked into its executable, while using the host's
glibc to satisfy the library's host-versioned C symbols. This command derives
the compiler and SDK paths from MuJoCo's own CMakeCache; it never edits MuJoCo.

From ``protospec/``::

    uv run python tools/make_probe_cmake_cache.py \
      --source ../third_party/MuJoCo/src \
      --mujoco ../third_party/install/MuJoCo \
      --source-cache ../third_party/MuJoCo/src/build/CMakeCache.txt \
      --out /tmp/mujoco-probe-toolchain.txt
    uv run python tools/refresh_mj_defaults.py --source ../third_party/MuJoCo/src \
      --mujoco ../third_party/install/MuJoCo --cmake-cache /tmp/mujoco-probe-toolchain.txt \
      --out /tmp/mujoco-defaults.json
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import sys


def die(message: str) -> None:
    raise SystemExit(message)


def cache_value(cache: Path, key: str) -> str:
    rows = [line.split("=", 1)[1].strip()
            for line in cache.read_text(encoding="utf-8").splitlines()
            if line.startswith(key + ":") and "=" in line]
    if len(rows) != 1:
        die(f"{cache} has no unique {key}")
    return rows[0]


def main() -> int:
    if not sys.platform.startswith("linux"):
        die("this UE libc++ probe recipe is for Linux only")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--mujoco", type=Path, required=True)
    parser.add_argument("--source-cache", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    source = args.source.expanduser().resolve(strict=True)
    install = args.mujoco.expanduser().resolve(strict=True)
    source_cache = args.source_cache.expanduser().resolve(strict=True)
    out = args.out.expanduser().resolve()

    cache_source = Path(cache_value(source_cache, "CMAKE_HOME_DIRECTORY")).resolve()
    if cache_source != source:
        die(f"{source_cache} was configured from {cache_source}, not {source}")
    source_sha = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    marker = install / "INSTALLED_SHA.txt"
    if not marker.is_file() or marker.read_text(encoding="utf-8").strip() != source_sha:
        die(f"{marker} does not match selected source {source_sha}")
    if not (install / "lib/libmujoco.so").is_file():
        die(f"{install} has no lib/libmujoco.so")

    compiler = Path(cache_value(source_cache, "CMAKE_CXX_COMPILER")).resolve(strict=True)
    flags = cache_value(source_cache, "CMAKE_CXX_FLAGS")
    match = re.search(r"(?:^|\s)--target=([A-Za-z0-9_-]+)(?:\s|$)", flags)
    if compiler.name != "clang++" or not match or "-stdlib=libc++" not in flags:
        die(f"{source_cache} does not describe a UE Clang/libc++ build")
    sdk = compiler.parent.parent
    include = sdk / "include/c++/v1"
    libcxx = sdk / "lib64/libc++.a"
    abi = sdk / "lib64/libc++abi.a"
    if not include.is_dir() or not libcxx.is_file() or not abi.is_file():
        die(f"UE libc++ headers or static archives missing below {sdk}")

    # The source cache's sysroot targets an older distro. The installed
    # library here also requires host GLIBC symbols; the probe's host sysroot
    # satisfies them while the explicit UE archives provide its C++ ABI.
    cxx_flags = (f"--target={match.group(1)} --sysroot=/ -nostdinc++ "
                 f"-isystem {include} -nostdlib++")
    link_flags = (f"-Wl,--whole-archive {libcxx} {abi} "
                  f"-Wl,--no-whole-archive -ldl -lpthread "
                  f"-Wl,-rpath,{install / 'lib'}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        f"CMAKE_CXX_COMPILER:FILEPATH={compiler}\n"
        f"CMAKE_CXX_FLAGS:STRING={cxx_flags}\n"
        f"CMAKE_EXE_LINKER_FLAGS:STRING={link_flags}\n",
        encoding="utf-8")
    print(f"wrote standalone probe toolchain cache for {source_sha}: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
