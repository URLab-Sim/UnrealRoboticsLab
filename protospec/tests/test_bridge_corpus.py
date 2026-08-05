"""Corpus compile harness for the ProtoSpec bridge (plan milestone 5 exit).

For every corpus model the pipeline runs ``ps_compile <model.xml> --base-dir
<dir>``, which reads the file with the ProtoSpec reader, serializes it (with
auto-naming), compiles it through the bridge (WriteMjcf -> mjVFS -> mj_loadXML)
and builds the name-based Binding. The tool prints a JSON verdict.

The contract this test enforces (task milestone 5):

* Every corpus file that plain MuJoCo can load and the ProtoSpec reader can
  parse must compile through the bridge with **zero errors**. Files that MuJoCo
  itself cannot load standalone (unregistered plugins, fixtures) are skipped --
  the differential harness skips them too, so they are not "reported identical".
* Binding coverage: every **authored-named** element binds to a compiled id,
  except where MuJoCo mangles names at compile (replicate/attach/composite/
  flexcomp expansion) or drops elements (discardvisual/fusestatic). The tool's
  id cross-check (mj_id2name(id) == name) must hold for every bound element.

Authored independently of the bridge code, driving only the stable ps_compile
process contract (exit codes + JSON), mirroring test_differential.py's
conventions (corpus discovery, binary lookup, supported-tag prefilter,
dotfile-beside-original asset strategy, size cap).
"""

from __future__ import annotations

import json
import os
import subprocess
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
MAX_XML_BYTES = 5 * 1024 * 1024

# Tags handled through the body row rather than their own schema row.
_BODY_CONTEXT = {"worldbody", "frame", "replicate"}
_IGNORE_TAGS = {"mujoco"}

# Tags after which an authored name may legitimately fail to bind: MuJoCo clones
# and renames (macro expansion) or drops elements.
_NAME_MANGLING_TAGS = {"replicate", "attach", "composite", "flexcomp"}


def _corpus_root() -> Path | None:
    env = os.environ.get("PROTOSPEC_CORPUS")
    candidates = [Path(env)] if env else []
    candidates.append(
        Path(
            r"C:\Users\jonat\Documents\Unreal Projects\url_proj\Plugins"
            r"\UnrealRoboticsLab\third_party\MuJoCo\src"
        )
    )
    for c in candidates:
        if c.is_dir():
            return c
    return None


def _find_binary(name: str) -> Path | None:
    # An explicit override wins (used to point at an out-of-tree build).
    env = os.environ.get("PROTOSPEC_" + name.upper().replace(".", "_").replace("-", "_"))
    if env and Path(env).is_file():
        return Path(env)
    matches = sorted(
        (ROOT / "lib").rglob(name),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return matches[0] if matches else None


def _load_supported() -> set[str] | None:
    path = ROOT / "lib" / "io" / "supported.json"
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None
    return {t.lower() for t in data.get("elements", [])}


CORPUS_ROOT = _corpus_root()
PS_COMPILE = _find_binary("ps_compile.exe")
MJ_MODEL_DIFF = _find_binary("mj_model_diff.exe")
SUPPORTED = _load_supported()


def _mujoco_available() -> bool:
    if PS_COMPILE is None:
        return False
    return (PS_COMPILE.parent / "mujoco.dll").is_file()


_STATS: Counter = Counter()


@pytest.fixture(scope="session", autouse=True)
def _summary():
    yield
    total = sum(_STATS.values())
    if not total:
        return
    print("\n\n=== bridge corpus summary ===")
    for key in (
        "compiled",
        "skip-unsupported",
        "skip-parse",
        "skip-unloadable-original",
        "skip-size",
        "compile-error",
        "binding-gap",
    ):
        if _STATS[key]:
            print(f"  {key:28s} {_STATS[key]}")
    print(f"  {'TOTAL':28s} {total}")


def _corpus_files() -> list[Path]:
    if CORPUS_ROOT is None:
        return []
    files = []
    for p in CORPUS_ROOT.rglob("*.xml"):
        parts = {s.lower() for s in p.parts}
        if "build" in parts:
            continue
        if p.name.startswith("._ps_"):  # stray harness temp files
            continue
        files.append(p)
    return sorted(files)


def _rel_id(p: Path) -> str:
    try:
        return p.relative_to(CORPUS_ROOT).as_posix()
    except ValueError:
        return p.name


_CORPUS = _corpus_files()
_CORPUS_IDS = [_rel_id(p) for p in _CORPUS]


def _scan_tags(path: Path) -> set[str] | None:
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError):
        return None
    tags = set()
    for el in root.iter():
        tag = el.tag
        if not isinstance(tag, str):
            continue
        tag = tag.lower()
        if tag in _IGNORE_TAGS:
            continue
        tags.add("body" if tag in _BODY_CONTEXT else tag)
    return tags


def _raw_tags(path: Path) -> set[str]:
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError):
        return set()
    return {el.tag.lower() for el in root.iter() if isinstance(el.tag, str)}


def _mujoco_loads(path: Path) -> bool:
    """True when MuJoCo can load the model standalone (self-diff != load error)."""
    r = subprocess.run(
        [str(MJ_MODEL_DIFF), str(path), str(path)],
        capture_output=True,
        text=True,
    )
    return r.returncode != 1


_READY = (
    CORPUS_ROOT is not None
    and _mujoco_available()
    and MJ_MODEL_DIFF is not None
    and SUPPORTED is not None
)


def _skip_reason() -> str:
    if CORPUS_ROOT is None:
        return "MuJoCo corpus not found (set PROTOSPEC_CORPUS)"
    if PS_COMPILE is None or not _mujoco_available():
        return "ps_compile.exe / mujoco.dll not built"
    if MJ_MODEL_DIFF is None:
        return "mj_model_diff.exe not built"
    if SUPPORTED is None:
        return "lib/io/supported.json missing"
    return ""


@pytest.mark.skipif(not _READY, reason=_skip_reason())
@pytest.mark.parametrize("model", _CORPUS, ids=_CORPUS_IDS or ["<none>"])
def test_bridge_compiles_corpus(model: Path):
    tags = _scan_tags(model)
    if tags is None:
        _STATS["skip-parse"] += 1
        pytest.skip("not parseable XML")
    unsupported = tags - SUPPORTED
    if unsupported:
        _STATS["skip-unsupported"] += 1
        pytest.skip(f"unsupported tags: {sorted(unsupported)}")
    if model.stat().st_size > MAX_XML_BYTES:
        _STATS["skip-size"] += 1
        pytest.skip(f"XML exceeds {MAX_XML_BYTES} bytes")

    proc = subprocess.run(
        [str(PS_COMPILE), str(model), "--base-dir", str(model.parent)],
        capture_output=True,
        text=True,
    )

    if proc.returncode == 3:
        _STATS["skip-unsupported"] += 1
        pytest.skip("ps_compile: unsupported elements (exit 3)")
    if proc.returncode == 1:
        # The ProtoSpec reader could not parse it (same signal as ps_roundtrip);
        # a reader concern, not the bridge's. Only in scope if it is a real model.
        if _mujoco_loads(model):
            pytest.fail(f"reader rejected a MuJoCo-loadable model:\n{proc.stderr}")
        _STATS["skip-parse"] += 1
        pytest.skip("ProtoSpec reader rejected (non-model / malformed fixture)")

    if proc.returncode == 2:
        # Compile failed. In scope only if MuJoCo itself can load the original.
        if not _mujoco_loads(model):
            _STATS["skip-unloadable-original"] += 1
            pytest.skip("MuJoCo cannot load the original standalone (plugin/fixture)")
        _STATS["compile-error"] += 1
        pytest.fail(f"bridge compile failed on a loadable model:\n{proc.stderr}")

    assert proc.returncode == 0, f"unexpected exit {proc.returncode}\n{proc.stderr}"

    verdict = json.loads(proc.stdout)
    assert verdict["ok"] is True, f"compile not ok: {verdict['errors']}"
    assert not verdict["errors"], verdict["errors"]
    # Auto resolves to the XML path (native is the atticked compiler, built only
    # under PROTOSPEC_BUILD_NATIVE). The Binding cross-check below holds
    # regardless of path.
    assert verdict["path_taken"] in ("xml", "native")

    # id cross-check must hold for every bound element.
    assert verdict["id_crosscheck_ok"] is True, "mj_id2name(id) != bound name"

    # Binding coverage: every authored name binds, unless names are mangled by
    # macro expansion or an element was dropped by a compiler flag.
    raw = _raw_tags(model)
    mangles = bool(raw & _NAME_MANGLING_TAGS)
    drops = bool(verdict["warnings"])  # bridge warns on flag-dropped elements
    if verdict["unbound_authored"] and not (mangles or drops):
        _STATS["binding-gap"] += 1
        pytest.fail(
            "authored names failed to bind with no macro/flag reason: "
            f"{verdict['unbound_authored']}"
        )

    _STATS["compiled"] += 1
