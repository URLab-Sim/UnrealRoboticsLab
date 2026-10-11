import argparse
import json
from pathlib import Path
import subprocess

import pytest

from tools import prepare_upgrade


def git_repo(path: Path) -> str:
    path.mkdir()
    subprocess.run(["git", "init", "-q", str(path)], check=True)
    subprocess.run(["git", "-C", str(path), "config", "user.email",
                    "test@example.invalid"], check=True)
    subprocess.run(["git", "-C", str(path), "config", "user.name", "Test"],
                   check=True)
    (path / "marker").write_text("source\n")
    subprocess.run(["git", "-C", str(path), "add", "marker"], check=True)
    subprocess.run(["git", "-C", str(path), "commit", "-qm", "fixture"],
                   check=True)
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def args_for(source: Path, install: Path, workdir: Path, sha: str):
    return argparse.Namespace(target=sha, remote="fixture", source=source,
                              install=install, probe_cache=None,
                              workdir=workdir, jobs=2)


def test_head_is_resolved_once(monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 0,
                                           stdout="a" * 40 + "\tHEAD\n")

    monkeypatch.setattr(prepare_upgrade.subprocess, "run", run)
    assert prepare_upgrade._git_target("HEAD", "remote") == "a" * 40
    assert len(calls) == 1
    assert calls[0] == ["git", "ls-remote", "remote", "HEAD"]


def test_candidate_corpus_example_is_checked_by_matching_native(
        tmp_path, monkeypatch):
    source = tmp_path / "source"
    model = source / "model/flex/bunny.xml"
    model.parent.mkdir(parents=True)
    model.write_text("<mujoco/>\n")
    install = tmp_path / "install"
    library = install / "lib/libmujoco.so"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"candidate library placeholder")
    pytest_log = tmp_path / "pytest.log"
    pytest_log.write_text("example_files: ['model/flex/bunny.xml']\n")
    monkeypatch.setenv("PROTOSPEC_CORPUS", str(source))

    class Function:
        argtypes = None
        restype = None

        def __init__(self, callback=None):
            self.callback = callback

        def __call__(self, *args):
            return self.callback(*args) if self.callback else None

    class Native:
        def __init__(self):
            def load(path, vfs, error, length):
                error.value = b"Schema violation: unrecognized attribute; line 35"
                return None
            self.mj_loadXML = Function(load)
            self.mj_deleteModel = Function()
            self.mj_loadPluginLibrary = Function()

    monkeypatch.setattr(prepare_upgrade.ctypes, "CDLL", lambda path: Native())
    result = prepare_upgrade._native_migration_probes(source, install, pytest_log)
    assert result == {
        "plugins": {"available": [], "load_attempts": []},
        "checks": [{
        "path": "model/flex/bunny.xml", "status": "native_rejects",
        "diagnostic": "Schema violation: unrecognized attribute; line 35",
        "line": 35,
        }],
    }


def test_reused_pin_install_runs_isolated_gates_and_records_stages(
        tmp_path, monkeypatch):
    source = tmp_path / "source"
    sha = git_repo(source)
    install = tmp_path / "install"
    install.mkdir()
    (install / "INSTALLED_SHA.txt").write_text(sha + "\n")
    workdir = tmp_path / "scratch"
    args = args_for(source, install, workdir, sha)
    calls = []

    def run(command, *, cwd, env, log, stage):
        calls.append((stage, command))
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text("stubbed " + stage + "\n")
        if stage == "rehearsal":
            rehearsal = workdir / "rehearsal"
            rehearsal.mkdir()
            (rehearsal / "defaults-a.json").write_text("{}\n")
            (rehearsal / "manifest.json").write_text(json.dumps({
                "native": {"source_sha": args.target, "mj_version": 3011001},
                "defaults_covered": 1362,
                "generated_files": 159,
            }) + "\n")

    checked = []
    monkeypatch.setattr(prepare_upgrade, "_run", run)
    monkeypatch.setattr(prepare_upgrade, "_xmlref_check",
                        lambda candidate, env: checked.append(candidate))
    monkeypatch.setattr(prepare_upgrade.frontend, "load_schema",
                        lambda *args, **kwargs: {"elements": []})
    monkeypatch.setattr(prepare_upgrade.frontend, "_read_baseline", lambda: {})
    monkeypatch.setattr(prepare_upgrade, "retired_classified_attrs", lambda root: [])
    monkeypatch.setattr(prepare_upgrade, "retired_doc_anchors", lambda root: [])
    report = prepare_upgrade.prepare(args)

    assert report["status"] == "passed"
    assert report["target_sha"] == sha
    assert report["source_mode"] == "reused"
    assert report["stages"]["native_install"]["mode"] == "reused"
    assert report["stages"]["pytest_candidate"]["status"] == "passed"
    assert report["stages"]["pytest_pinned_regression"]["status"] == "passed"
    assert report["stages"]["xmlref"]["status"] == "passed"
    assert [stage for stage, _ in calls] == [
        "rehearsal", "candidate pytest", "pinned regression pytest"]
    assert checked == [source]
    assert (install / "INSTALLED_SHA.txt").read_text() == sha + "\n"
    assert json.loads((workdir / "report.json").read_text())["status"] == "passed"
    assert not (workdir / "generated").exists()


def test_candidate_test_failure_is_reported_and_xmlref_still_runs(
        tmp_path, monkeypatch):
    source = tmp_path / "source"
    sha = git_repo(source)
    install = tmp_path / "install"
    install.mkdir()
    (install / "INSTALLED_SHA.txt").write_text(sha + "\n")
    workdir = tmp_path / "scratch"
    args = args_for(source, install, workdir, sha)
    checked = []

    def run(command, *, cwd, env, log, stage):
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text("stubbed " + stage + "\n")
        if stage == "rehearsal":
            rehearsal = workdir / "rehearsal"
            rehearsal.mkdir()
            (rehearsal / "defaults-a.json").write_text("{}\n")
            (rehearsal / "manifest.json").write_text(json.dumps({
                "native": {"source_sha": sha, "mj_version": 3011001},
                "defaults_covered": 1362, "generated_files": 159,
            }) + "\n")
        elif stage == "candidate pytest":
            raise prepare_upgrade.UpgradeError("candidate corpus gate failed")

    monkeypatch.setattr(prepare_upgrade, "_run", run)
    monkeypatch.setattr(prepare_upgrade, "_xmlref_check",
                        lambda candidate, env: checked.append(candidate))
    monkeypatch.setattr(prepare_upgrade.frontend, "load_schema",
                        lambda *args, **kwargs: {"elements": []})
    monkeypatch.setattr(prepare_upgrade.frontend, "_read_baseline", lambda: {})
    monkeypatch.setattr(prepare_upgrade, "retired_classified_attrs", lambda root: [])
    monkeypatch.setattr(prepare_upgrade, "retired_doc_anchors", lambda root: [])

    with pytest.raises(prepare_upgrade.UpgradeError,
                       match="failed stages: pytest_candidate"):
        prepare_upgrade.prepare(args)
    report = json.loads((workdir / "report.json").read_text())
    assert report["stages"]["pytest_candidate"]["status"] == "failed"
    assert report["stages"]["pytest_pinned_regression"]["status"] == "passed"
    assert report["stages"]["xmlref"]["status"] == "passed"
    assert report["failed_stages"] == ["pytest_candidate"]
    assert checked == [source]


def test_reused_install_mismatch_fails_before_probe(tmp_path, monkeypatch):
    source = tmp_path / "source"
    sha = git_repo(source)
    install = tmp_path / "install"
    install.mkdir()
    (install / "INSTALLED_SHA.txt").write_text("0" * 40 + "\n")
    workdir = tmp_path / "scratch"
    monkeypatch.setattr(prepare_upgrade, "_run",
                        lambda *args, **kwargs: pytest.fail("must stop at pin gate"))

    with pytest.raises(prepare_upgrade.UpgradeError,
                       match="reused install marker"):
        prepare_upgrade.prepare(args_for(source, install, workdir, sha))
    report = json.loads((workdir / "report.json").read_text())
    assert report["status"] == "failed"
    assert report["stages"]["native_install"]["status"] == "failed"


def test_reused_dirty_source_fails_with_target_in_report(tmp_path, monkeypatch):
    source = tmp_path / "source"
    sha = git_repo(source)
    (source / "uncommitted.schema").write_text("changed\n")
    workdir = tmp_path / "scratch"
    args = argparse.Namespace(target=sha, remote="fixture", source=source,
                              install=None, probe_cache=None,
                              workdir=workdir, jobs=2)

    with pytest.raises(prepare_upgrade.UpgradeError, match="local modifications"):
        prepare_upgrade.prepare(args)
    report = json.loads((workdir / "report.json").read_text())
    assert report["target_sha"] == sha
    assert report["stages"]["source"]["status"] == "failed"


def test_isolated_native_build_records_commands_and_sha(tmp_path, monkeypatch):
    source = tmp_path / "source"
    sha = git_repo(source)
    install = tmp_path / "install"
    install.mkdir()
    workdir = tmp_path / "scratch"
    workdir.mkdir()
    calls = []

    def run(command, *, cwd, env, log, stage):
        calls.append((stage, command))
        log.write_text("stubbed " + stage + "\n")
        if stage == "install":
            (install / "include").mkdir()

    monkeypatch.setattr(prepare_upgrade, "_run", run)
    report = {"target_sha": sha, "stages": {}}
    cache = prepare_upgrade._build(source, install, workdir, 3, {},
                                   report["stages"], report)

    assert cache == workdir / "build/CMakeCache.txt"
    assert (install / "INSTALLED_SHA.txt").read_text() == sha + "\n"
    assert [stage for stage, _ in calls] == ["configure", "build", "install"]
    assert "-DMUJOCO_BUILD_TESTS=OFF" in calls[0][1]
    assert calls[1][1][-1] == "3"
    assert all(report["stages"][stage]["status"] == "passed"
               for stage in ("configure", "build", "install"))
    assert json.loads((workdir / "report.json").read_text())["target_sha"] == sha
