"""Rehearse a MuJoCo source upgrade without changing URLab's staged runtime.

Example (from ``protospec/``)::

    uv run python tools/rehearse_upgrade.py \
      --source /tmp/mujoco-head --mujoco /tmp/mujoco-head-install \
      --workdir /tmp/urlab-head-rehearsal

The source checkout and installed headers/library must be from the same commit.
All refresh and generator output stays below ``--workdir``. Pass
``--compare-out`` for a read-only freshness check of an existing URLab tree,
normally only for the currently pinned MuJoCo revision.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
REFRESH = ROOT / "tools" / "refresh_mj_defaults.py"
sys.path.insert(0, str(ROOT))


def fail(message: str) -> None:
    raise SystemExit(message)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_sha(source: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(source), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True)
    return result.stdout.strip()


def native_identity(source: Path, install: Path, *,
                    defer_runtime_check: bool = False) -> dict:
    sha = source_sha(source)
    marker = install / "INSTALLED_SHA.txt"
    if not marker.is_file():
        fail(f"matched install marker missing: {marker}")
    if marker.read_text(encoding="utf-8").strip() != sha:
        fail(f"install marker {marker} does not match source {sha}")

    source_header = source / "include/mujoco/mujoco.h"
    install_header = install / "include/mujoco/mujoco.h"
    if not source_header.is_file() or not install_header.is_file():
        fail("source or installed mujoco.h is missing")
    header_hash = digest(source_header)
    if digest(install_header) != header_hash:
        fail("installed mujoco.h differs from selected source header")
    match = re.search(r"^#define\s+mjVERSION_HEADER\s+(\d+)\s*$",
                      source_header.read_text(encoding="utf-8"), re.M)
    if not match:
        fail(f"mjVERSION_HEADER missing in {source_header}")
    header_version = int(match.group(1))

    library = install / "lib/libmujoco.so"
    if not library.is_file():
        fail(f"installed native library missing: {library}")
    actual_library = library.resolve(strict=True)
    try:
        native = ctypes.CDLL(str(actual_library))
    except OSError as exc:
        if not defer_runtime_check:
            fail(f"cannot load {actual_library} in this Python runtime: {exc}; "
                 "use a matched standalone native install or its required "
                 "toolchain runtime, or pass --cmake-cache for the native "
                 "probe")
        runtime_version = None
        runtime_version_string = None
    else:
        native.mj_version.restype = ctypes.c_int
        native.mj_versionString.restype = ctypes.c_char_p
        runtime_version = native.mj_version()
        runtime_version_string = native.mj_versionString().decode("ascii")
        if runtime_version != header_version:
            fail(f"native mj_version {runtime_version} differs from header "
                 f"mjVERSION_HEADER {header_version}")
    return {
        "source_sha": sha,
        "source": str(source),
        "install": str(install),
        "header_sha256": header_hash,
        "library": str(actual_library),
        "library_sha256": digest(actual_library),
        "mj_version": runtime_version,
        "mj_version_string": runtime_version_string,
    }


def run_logged(command: list[str], env: dict[str, str], path: Path) -> None:
    with path.open("w", encoding="utf-8") as log:
        result = subprocess.run(command, cwd=ROOT, env=env,
                                stdout=log, stderr=subprocess.STDOUT,
                                text=True)
    if result.returncode:
        fail(f"command failed ({result.returncode}): {' '.join(command)}\n"
             f"see {path}")


def generated_files(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): digest(path)
        for path in sorted(root.rglob("*")) if path.is_file()
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="MuJoCo Git checkout to rehearse")
    parser.add_argument("--mujoco", type=Path, required=True,
                        help="matched isolated native install")
    parser.add_argument("--workdir", type=Path, required=True,
                        help="new output directory, outside the URLab checkout")
    parser.add_argument("--compare-out", type=Path,
                        help="optional existing URLab tree to check read-only")
    parser.add_argument("--cmake-cache", type=Path,
                        help="matching probe cache for UE Clang/libc++ installs")
    args = parser.parse_args()

    source = args.source.expanduser().resolve(strict=True)
    install = args.mujoco.expanduser().resolve(strict=True)
    workdir = args.workdir.expanduser().resolve()
    checkout = ROOT.parent.resolve()
    if workdir == checkout or checkout in workdir.parents:
        fail("--workdir must be outside the URLab checkout")
    if workdir.exists() and any(workdir.iterdir()):
        fail(f"--workdir is not empty: {workdir}")
    workdir.mkdir(parents=True, exist_ok=True)

    identity = native_identity(source, install,
                               defer_runtime_check=args.cmake_cache is not None)
    env = os.environ.copy()
    env["PROTOSPEC_MUJOCO_SRC"] = str(source)
    # The refresh bootstraps against MuJoCo's initialisers; generation reads
    # its freshly written snapshot, never the pinned snapshot in the checkout.
    env.pop("PROTOSPEC_MJ_DEFAULTS", None)
    first_defaults = workdir / "defaults-a.json"
    second_defaults = workdir / "defaults-b.json"
    for suffix, output in (("a", first_defaults), ("b", second_defaults)):
        command = [sys.executable, str(REFRESH), "--source", str(source),
                   "--mujoco", str(install)]
        if args.cmake_cache:
            command += ["--cmake-cache", str(args.cmake_cache.resolve(strict=True))]
        command += ["--out", str(output)]
        run_logged(command,
                   env, workdir / f"refresh-{suffix}.log")
    if first_defaults.read_bytes() != second_defaults.read_bytes():
        fail("two native defaults refreshes produced different bytes")
    snapshot = json.loads(first_defaults.read_text(encoding="utf-8"))
    if snapshot.get("mujoco") != identity["source_sha"]:
        fail("refreshed defaults snapshot has the wrong source revision")
    if snapshot.get("header_sha256") != identity["header_sha256"]:
        fail("refreshed defaults snapshot has the wrong native header hash")
    if snapshot.get("library_sha256") != identity["library_sha256"]:
        fail("refreshed defaults snapshot has the wrong native library hash")
    if snapshot.get("runtime_version") != snapshot.get("header_version"):
        fail("native defaults probe found a header/runtime version mismatch")
    if identity["mj_version"] is None:
        identity["mj_version"] = snapshot["runtime_version"]
        identity["mj_version_string"] = (
            f"verified by native defaults probe ({snapshot['runtime_version']})")
    from protospec_gen import mjdefaults  # import after source validation
    counts = mjdefaults.counts(snapshot)
    covered = len(mjdefaults.coverage(snapshot))
    if covered != sum(counts.values()):
        fail("defaults/skips overlap or their coverage count is inconsistent")

    env["PROTOSPEC_MJ_DEFAULTS"] = str(first_defaults)
    generated: list[dict[str, str]] = []
    for suffix in ("a", "b"):
        output = workdir / f"generated-{suffix}"
        (output / "Source/URLab").mkdir(parents=True)
        command = [sys.executable, "-m", "protospec_gen.emit_ue",
                   "--out", str(output)]
        run_logged(command, env, workdir / f"generate-{suffix}.log")
        run_logged([*command, "--check"], env,
                   workdir / f"freshness-{suffix}.log")
        generated.append(generated_files(output))
    if not generated[0] or generated[0] != generated[1]:
        fail("repeated UE generation is empty or differs by content")

    if args.compare_out:
        compare_out = args.compare_out.expanduser().resolve(strict=True)
        run_logged([sys.executable, "-m", "protospec_gen.emit_ue",
                    "--out", str(compare_out), "--check"], env,
                   workdir / "production-freshness.log")

    manifest = {
        "native": identity,
        "defaults_sha256": digest(first_defaults),
        "defaults_counts": counts,
        "defaults_covered": covered,
        "generated_files": len(generated[0]),
        "generated_sha256": generated[0],
        "compare_out": str(args.compare_out.resolve()) if args.compare_out else None,
        "scope": "native defaults and UE code generation; no UE runtime build",
    }
    (workdir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"{identity['source_sha']} MuJoCo {identity['mj_version_string']}: "
          f"{covered} defaults/skip fields, {len(generated[0])} reproducible "
          f"UE generated files; evidence: {workdir / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
