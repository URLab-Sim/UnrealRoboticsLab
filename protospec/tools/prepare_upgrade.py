"""Prepare and check an isolated MuJoCo upgrade candidate.

The command never changes the URLab submodule, staged libraries, defaults
snapshot, or generated UE files. It resolves ``HEAD`` once, then records the
selected commit and every stage in ``<workdir>/report.json``. Existing source
and install directories can be supplied for a cheap rerun.

Example from ``protospec/``::

    uv run python tools/prepare_upgrade.py --target HEAD \
      --workdir /tmp/urlab-mujoco-candidate
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT.parent
sys.path.insert(0, str(ROOT))
from protospec_gen import frontend
from protospec_gen.frontend import retired_classified_attrs
from protospec_gen.xmlref import retired_doc_anchors

UPSTREAM = "https://github.com/google-deepmind/mujoco.git"
REHEARSE = ROOT / "tools" / "rehearse_upgrade.py"
MAKE_PROBE_CACHE = ROOT / "tools" / "make_probe_cmake_cache.py"


class UpgradeError(RuntimeError):
    pass


def _git_target(target: str, remote: str) -> str:
    if target.upper() == "HEAD":
        result = subprocess.run(["git", "ls-remote", remote, "HEAD"],
                                check=True, capture_output=True, text=True)
        rows = [row.split()[0] for row in result.stdout.splitlines()
                if row.strip()]
        if len(rows) != 1 or not re.fullmatch(r"[0-9a-fA-F]{40}", rows[0]):
            raise UpgradeError(f"could not resolve one remote HEAD from {remote}")
        return rows[0].lower()
    if not re.fullmatch(r"[0-9a-fA-F]{40}", target):
        raise UpgradeError("--target must be a full 40-character Git SHA or HEAD")
    return target.lower()


def _run(command: list[str], *, cwd: Path, env: dict[str, str], log: Path,
         stage: str) -> None:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w", encoding="utf-8") as stream:
        result = subprocess.run(command, cwd=cwd, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, text=True)
    if result.returncode:
        raise UpgradeError(
            f"{stage} failed ({result.returncode}); inspect {log}")


def _inside(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def _marker_sha(install: Path) -> str:
    marker = install / "INSTALLED_SHA.txt"
    if not marker.is_file():
        raise UpgradeError(f"matched install marker missing: {marker}")
    return marker.read_text(encoding="utf-8").strip()


def _source_sha(source: Path) -> str:
    result = subprocess.run(["git", "-C", str(source), "rev-parse", "HEAD"],
                            check=True, capture_output=True, text=True)
    return result.stdout.strip().lower()


def _source_is_clean(source: Path) -> bool:
    result = subprocess.run(["git", "-C", str(source), "status", "--porcelain"],
                            check=True, capture_output=True, text=True)
    return not result.stdout.strip()


def _committed_mujoco_sha() -> str | None:
    result = subprocess.run(
        ["git", "-C", str(PLUGIN), "ls-tree", "HEAD", "third_party/MuJoCo/src"],
        check=True, capture_output=True, text=True)
    match = re.search(r"^160000 commit ([0-9a-f]{40})\t", result.stdout)
    return match.group(1) if match else None


def _build(source: Path, install: Path, workdir: Path, jobs: int,
           env: dict[str, str], stages: dict[str, dict], report: dict) -> Path:
    def save() -> None:
        (workdir / "report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")

    build = workdir / "build"
    cmake = ["cmake", "-S", str(source), "-B", str(build), "-G", "Ninja",
             "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_INSTALL_PREFIX={install}",
             "-DMUJOCO_BUILD_EXAMPLES=OFF", "-DMUJOCO_BUILD_SIMULATE=OFF",
             "-DMUJOCO_BUILD_TESTS=OFF", "-DMUJOCO_TEST_PYTHON_UTIL=OFF",
             "-DMUJOCO_BUILD_STUDIO=OFF"]
    stages["configure"] = {"status": "running", "command": cmake}
    save()
    _run(cmake, cwd=workdir, env=env, log=workdir / "configure.log",
         stage="configure")
    stages["configure"] = {"status": "passed"}
    save()

    command = ["cmake", "--build", str(build), "--parallel", str(jobs)]
    stages["build"] = {"status": "running", "command": command}
    save()
    _run(command, cwd=workdir, env=env, log=workdir / "build.log", stage="build")
    stages["build"] = {"status": "passed"}
    save()

    install_command = ["cmake", "--install", str(build), "--config", "Release"]
    stages["install"] = {"status": "running", "command": install_command}
    save()
    _run(install_command, cwd=workdir, env=env,
         log=workdir / "install.log", stage="install")
    (install / "INSTALLED_SHA.txt").write_text(_source_sha(source) + "\n",
                                                 encoding="utf-8")
    stages["install"] = {"status": "passed"}
    save()
    return build / "CMakeCache.txt"


def _xmlref_check(source: Path, env: dict[str, str]) -> None:
    # This is the same read-only gate used by generation, exposed as a separate
    # report stage so a moved manual anchor has an obvious failure location.
    snippet = (
        "from protospec_gen import frontend, xmlref; "
        "ast=frontend.load_schema(); anchors=xmlref.load_anchors(); "
        "live={f\"{e['schema_name']}.{f['xml']}\" for e in ast['elements'] "
        "for f in e['fields']}; "
        "xmlref.check_drift(xmlref.bind(ast, anchors), anchors, live_keys=live)"
    )
    result = subprocess.run([sys.executable, "-c", snippet], cwd=ROOT,
                            env=env, capture_output=True, text=True)
    if result.returncode:
        raise UpgradeError("xmlref drift check failed: "
                           + (result.stderr.strip() or result.stdout.strip()))


def _native_migration_probes(source: Path, install: Path,
                             pytest_log: Path) -> dict:
    """Native-check corpus examples named by candidate schema-coverage errors."""
    if not pytest_log.is_file():
        return {"plugins": {"available": [], "load_attempts": []},
                "checks": []}
    log = pytest_log.read_text(encoding="utf-8", errors="replace")
    examples = sorted(set(re.findall(
        r"(?:model|test)/[A-Za-z0-9_./-]+\.xml", log)))
    if not examples:
        return {"plugins": {"available": [], "load_attempts": []},
                "checks": []}
    corpus_root = Path(os.environ.get(
        "PROTOSPEC_CORPUS", str(PLUGIN / "third_party/MuJoCo/src"))).resolve()
    library = install / "lib" / "libmujoco.so"
    if not library.is_file():
        return {"plugins": {"available": [], "load_attempts": []},
                "checks": [{"path": path, "status": "not_checked",
                            "diagnostic": f"candidate library missing: {library}"}
                           for path in examples]}
    native = ctypes.CDLL(str(library))
    native.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p,
                                  ctypes.c_char_p, ctypes.c_int]
    native.mj_loadXML.restype = ctypes.c_void_p
    native.mj_deleteModel.argtypes = [ctypes.c_void_p]
    native.mj_loadPluginLibrary.argtypes = [ctypes.c_char_p]
    plugins = sorted(path for path in (install / "lib").rglob("*.so")
                     if path.resolve() != library.resolve())
    load_attempts = []
    for plugin in plugins:
        native.mj_loadPluginLibrary(str(plugin).encode())
        load_attempts.append(str(plugin))
    result = []
    for relative in examples:
        model_path = corpus_root / relative
        if not model_path.is_file():
            result.append({"path": relative, "status": "not_checked",
                           "diagnostic": "example path not found in candidate source"})
            continue
        error = ctypes.create_string_buffer(8192)
        model = native.mj_loadXML(str(model_path).encode(), None, error,
                                  len(error))
        if model:
            native.mj_deleteModel(model)
            result.append({"path": relative, "status": "accepted"})
            continue
        diagnostic = error.value.decode("utf-8", "replace")
        line = re.search(r"line\s+(\d+)", diagnostic, re.IGNORECASE)
        status = ("inconclusive_plugin_missing"
                  if "plugin" in diagnostic.lower() and "not found" in diagnostic.lower()
                  else "native_rejects")
        result.append({"path": relative, "status": status,
                       "diagnostic": diagnostic,
                       "line": int(line.group(1)) if line else None})
    return {"plugins": {"available": [str(path) for path in plugins],
                        "load_attempts": load_attempts},
            "checks": result}


def prepare(args: argparse.Namespace) -> dict:
    workdir = args.workdir.expanduser().resolve()
    if _inside(workdir, PLUGIN):
        raise UpgradeError("--workdir must be outside the URLab checkout")
    if workdir.exists() and any(workdir.iterdir()):
        raise UpgradeError(f"--workdir is not empty: {workdir}")
    workdir.mkdir(parents=True, exist_ok=True)

    report_path = workdir / "report.json"
    stages: dict[str, dict] = {}
    report: dict = {"status": "running", "target_requested": args.target,
                    "remote": args.remote, "stages": stages}

    def save() -> None:
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")

    try:
        stages["resolve_target"] = {"status": "running"}
        save()
        target_sha = _git_target(args.target, args.remote)
        report["target_sha"] = target_sha
        stages["resolve_target"] = {"status": "passed", "sha": target_sha}
        save()

        env = os.environ.copy()
        source = (args.source.expanduser().resolve(strict=True)
                  if args.source else workdir / "source")
        stages["source"] = {"status": "running", "path": str(source)}
        save()
        if args.source:
            actual = _source_sha(source)
            if actual != target_sha:
                raise UpgradeError(
                    f"reused source is {actual}, expected selected {target_sha}")
            source_mode = "reused"
        else:
            _run(["git", "clone", "--filter=blob:none", "--no-checkout",
                  args.remote, str(source)], cwd=workdir, env=env,
                 log=workdir / "clone.log", stage="source clone")
            _run(["git", "-C", str(source), "fetch", "--depth=1", "origin",
                  target_sha], cwd=workdir, env=env,
                 log=workdir / "fetch.log", stage="source fetch")
            _run(["git", "-C", str(source), "checkout", "--detach", target_sha],
                 cwd=workdir, env=env, log=workdir / "checkout.log",
                 stage="source checkout")
            source_mode = "cloned"
        if _source_sha(source) != target_sha:
            raise UpgradeError("source checkout does not match selected target SHA")
        if not _source_is_clean(source):
            raise UpgradeError(
                "source checkout has local modifications; use a clean checkout "
                "of the selected SHA")
        report["source_mode"] = source_mode
        stages["source"] = {"status": "passed", "path": str(source),
                             "mode": source_mode}
        save()

        install = (args.install.expanduser().resolve()
                   if args.install else workdir / "install")
        stages["native_install"] = {"status": "running", "path": str(install)}
        save()
        probe_cache: Path | None = None
        if args.install:
            install = install.resolve(strict=True)
            marker_sha = _marker_sha(install)
            if marker_sha != target_sha:
                raise UpgradeError(
                    f"reused install marker is {marker_sha}, expected {target_sha}")
            mode = "reused"
            if args.probe_cache:
                probe_cache = args.probe_cache.expanduser().resolve(strict=True)
            else:
                cache = source / "build/CMakeCache.txt"
                if cache.is_file():
                    probe_cache = workdir / "probe-cmake-cache.txt"
                    _run([sys.executable, str(MAKE_PROBE_CACHE), "--source",
                          str(source), "--mujoco", str(install), "--source-cache",
                          str(cache), "--out", str(probe_cache)], cwd=ROOT, env=env,
                         log=workdir / "probe-cache.log", stage="probe cache")
        else:
            install.mkdir(parents=True, exist_ok=True)
            build_cache = _build(source, install, workdir, args.jobs, env,
                                 stages, report)
            mode = "built"
            if args.probe_cache:
                raise UpgradeError("--probe-cache applies only to a reused install")
            # A normal host build can be loaded by Python directly. UE builds
            # can opt into a derived probe cache via --install above.
            del build_cache
        if _marker_sha(install) != target_sha:
            raise UpgradeError("native install SHA marker changed or is missing")
        if args.probe_cache and not args.install:
            raise UpgradeError("--probe-cache requires --install")
        stages["native_install"] = {"status": "passed", "path": str(install),
                                     "mode": mode,
                                     "probe_cache": str(probe_cache) if probe_cache else None}
        save()

        stages["retired_schema_metadata"] = {"status": "running"}
        save()
        report["retired_classified_attrs"] = retired_classified_attrs(str(source))
        report["retired_doc_anchors"] = retired_doc_anchors(str(source))
        candidate_ast = frontend.load_schema(str(source),
                                             include_probed_defaults=False)
        live_attrs = {f"{element['schema_name']}.{field['xml']}"
                      for element in candidate_ast["elements"]
                      for field in element["fields"]}
        report["new_schema_attrs"] = sorted(
            live_attrs - set(frontend._read_baseline()))
        stages["retired_schema_metadata"] = {
            "status": "passed",
            "removed_classifications": len(report["retired_classified_attrs"]),
            "removed_doc_bindings": len(report["retired_doc_anchors"]),
            "new_schema_attrs": len(report["new_schema_attrs"]),
        }
        save()

        rehearsal_dir = workdir / "rehearsal"
        rehearsal_cmd = [sys.executable, str(REHEARSE), "--source", str(source),
                         "--mujoco", str(install), "--workdir", str(rehearsal_dir)]
        if probe_cache:
            rehearsal_cmd += ["--cmake-cache", str(probe_cache)]
        committed_sha = _committed_mujoco_sha()
        if target_sha == committed_sha:
            rehearsal_cmd += ["--compare-out", str(PLUGIN)]
        stages["rehearsal"] = {"status": "running", "command": rehearsal_cmd}
        save()
        _run(rehearsal_cmd, cwd=ROOT, env=env,
             log=workdir / "rehearsal.log", stage="rehearsal")
        snapshot = rehearsal_dir / "defaults-a.json"
        if not snapshot.is_file():
            raise UpgradeError("rehearsal passed but wrote no defaults snapshot")
        rehearsal_manifest = json.loads(
            (rehearsal_dir / "manifest.json").read_text(encoding="utf-8"))
        report["native"] = rehearsal_manifest["native"]
        report["defaults_covered"] = rehearsal_manifest["defaults_covered"]
        report["generated_files"] = rehearsal_manifest["generated_files"]
        env["PROTOSPEC_MUJOCO_SRC"] = str(source)
        env["PROTOSPEC_MJ_DEFAULTS"] = str(snapshot)
        stages["rehearsal"] = {"status": "passed",
                               "manifest": str(rehearsal_dir / "manifest.json")}
        save()

        stages["pytest_candidate"] = {
            "status": "running",
            "selection": "all tests except @pytest.mark.pinned"}
        save()
        try:
            _run(["uv", "run", "--offline", "pytest", "-q", "-m", "not pinned"],
                 cwd=ROOT, env=env, log=workdir / "pytest-candidate.log",
                 stage="candidate pytest")
        except UpgradeError as exc:
            stages["pytest_candidate"] = {
                "status": "failed", "error": str(exc),
                "log": str(workdir / "pytest-candidate.log")}
        else:
            stages["pytest_candidate"] = {
                "status": "passed", "log": str(workdir / "pytest-candidate.log")}
        native_probe = _native_migration_probes(
            source, install, workdir / "pytest-candidate.log")
        report["native_plugins"] = native_probe["plugins"]
        report["native_migration_probes"] = native_probe["checks"]
        save()

        stages["pytest_pinned_regression"] = {
            "status": "running",
            "selection": "@pytest.mark.pinned against committed source/default snapshot"}
        save()
        pinned_env = env.copy()
        pinned_env.pop("PROTOSPEC_MUJOCO_SRC", None)
        pinned_env.pop("PROTOSPEC_MJ_DEFAULTS", None)
        try:
            _run(["uv", "run", "--offline", "pytest", "-q", "-m", "pinned"],
                 cwd=ROOT, env=pinned_env, log=workdir / "pytest-pinned.log",
                 stage="pinned regression pytest")
        except UpgradeError as exc:
            stages["pytest_pinned_regression"] = {
                "status": "failed", "error": str(exc),
                "log": str(workdir / "pytest-pinned.log")}
        else:
            stages["pytest_pinned_regression"] = {
                "status": "passed", "log": str(workdir / "pytest-pinned.log")}
        save()

        stages["xmlref"] = {"status": "running"}
        save()
        try:
            _xmlref_check(source, env)
        except UpgradeError as exc:
            stages["xmlref"] = {"status": "failed", "error": str(exc)}
        else:
            stages["xmlref"] = {"status": "passed"}
        failed_stages = [name for name, stage in stages.items()
                         if stage.get("status") == "failed"]
        report["status"] = "failed" if failed_stages else "passed"
        report["failed_stages"] = failed_stages
        report["scope"] = (
            "isolated native defaults, schema/UE generation, generator tests, "
            "and XMLreference anchor drift; no Unreal runtime build")
        save()
        if failed_stages:
            raise UpgradeError("failed stages: " + ", ".join(failed_stages))
        return report
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        for name, stage in stages.items():
            if stage.get("status") == "running":
                stage["status"] = "failed"
                stage["error"] = str(exc)
        save()
        raise UpgradeError(f"{exc}\nreport: {report_path}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, help="full MuJoCo SHA or HEAD")
    parser.add_argument("--remote", default=UPSTREAM,
                        help=f"MuJoCo Git remote (default: {UPSTREAM})")
    parser.add_argument("--source", type=Path,
                        help="existing source checkout at exactly --target")
    parser.add_argument("--install", type=Path,
                        help="existing isolated install at exactly --target")
    parser.add_argument("--probe-cache", type=Path,
                        help="matching probe CMake cache for a reused UE libc++ install")
    parser.add_argument("--workdir", type=Path, required=True,
                        help="new or empty external scratch directory")
    parser.add_argument("--jobs", type=int, default=8,
                        help="parallel native build jobs (default: 8)")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    if args.probe_cache and not args.install:
        parser.error("--probe-cache requires --install")
    try:
        report = prepare(args)
    except UpgradeError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        print(f"Report: {args.workdir.expanduser().resolve() / 'report.json'}",
              file=sys.stderr)
        return 1
    print(f"PASS {report['target_sha']}: {report['scope']}")
    print(f"Report: {args.workdir.expanduser().resolve() / 'report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
