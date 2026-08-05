"""Corpus sanity gate for the three-tier validator (plan Section 9, milestone 4).

Runs the ``ps_validate`` CLI over MuJoCo's whole vendored ``.xml`` corpus and
asserts the load-bearing property:

* **every file ProtoSpec parses must produce ZERO tier-1/tier-2 errors** --
  structural and referential validation must never false-positive on a model
  MuJoCo accepts. Tier-3 warnings (semantic lint) are allowed and their count is
  reported in the session summary, not asserted.

``ps_validate`` exit codes drive the per-file verdict:

* ``0`` -- parsed, zero tier-1/tier-2 errors: PASS (tier-3 count recorded);
* ``2`` -- parsed, at least one tier-1/tier-2 error: FAIL (a false positive to
  fix) unless the file is a reviewed intentional-failure fixture (below);
* ``3`` -- the file uses elements outside the reader's supported set: skip;
* ``1`` -- ProtoSpec could not parse the file: skip (a reader concern the
  differential harness owns, not a validation false positive).

``KNOWN_MALFORMED`` lists the handful of corpus files MuJoCo itself rejects at
compile -- ProtoSpec's form-preserving reader parses them, so validation
correctly reports the resulting structural/referential problem. These are the
only files allowed to exit 2, each with a reviewed reason (silence is not
allowed, matching the corpus-coverage gate's KNOWN_GAPS convention).

Discovery mirrors ``test_differential.py``: the binary is found via the
``PS_VALIDATE`` env var or an ``rglob`` under ``protospec/lib/**``; the corpus root via
``PROTOSPEC_CORPUS`` or the vendored plugin path. Absent either, the module
skips wholesale.
"""

from __future__ import annotations

import os
import subprocess
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent

# Corpus files MuJoCo rejects at compile; ProtoSpec's reader parses them, so a
# tier-1/tier-2 finding here is correct, not a false positive. Keyed by basename
# with the reviewed reason.
KNOWN_MALFORMED: dict[str, str] = {
    "malformed_duplicated.xml": (
        "two unnamed <mesh file='cube.obj'> collide on their file-derived name; "
        "the geoms reference MuJoCo's compile-time disambiguated names "
        "(cube_0/cube_1) which do not exist pre-compile -- MuJoCo rejects this "
        "file, so the tier-2 dangling-mesh reports are correct"
    ),
}


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


def _find_binary() -> Path | None:
    env = os.environ.get("PS_VALIDATE")
    if env and Path(env).is_file():
        return Path(env)
    matches = sorted(
        (ROOT / "lib").rglob("ps_validate.exe"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if matches:
        return matches[0]
    # POSIX build (no .exe suffix).
    matches = sorted(
        (ROOT / "lib").rglob("ps_validate"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return matches[0] if matches else None


CORPUS_ROOT = _corpus_root()
PS_VALIDATE = _find_binary()


def _corpus_files() -> list[Path]:
    if CORPUS_ROOT is None:
        return []
    files = []
    for p in CORPUS_ROOT.rglob("*.xml"):
        if "build" in {s.lower() for s in p.parts}:
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

_STATS: Counter = Counter()


@pytest.fixture(scope="session", autouse=True)
def _summary():
    yield
    total = sum(_STATS.values())
    if not total:
        return
    print("\n\n=== validation corpus summary ===")
    for key in (
        "clean",
        "tier3-warnings",
        "known-malformed",
        "skip-unsupported",
        "skip-parse-error",
    ):
        if _STATS[key]:
            print(f"  {key:22s} {_STATS[key]}")
    print(f"  {'files':22s} {_STATS['files']}")


def _parse_summary(stdout: str) -> tuple[int, int, int]:
    for line in stdout.splitlines():
        if line.startswith("SUMMARY "):
            kv = dict(tok.split("=") for tok in line.split()[1:])
            return (
                int(kv.get("tier1_errors", 0)),
                int(kv.get("tier2_errors", 0)),
                int(kv.get("tier3_warnings", 0)),
            )
    return (0, 0, 0)


@pytest.mark.skipif(PS_VALIDATE is None, reason="ps_validate not built")
@pytest.mark.skipif(CORPUS_ROOT is None, reason="MuJoCo corpus not found")
@pytest.mark.parametrize("model", _CORPUS, ids=_CORPUS_IDS)
def test_validate_corpus(model: Path):
    _STATS["files"] += 1
    r = subprocess.run(
        [str(PS_VALIDATE), str(model)],
        capture_output=True,
        text=True,
    )

    if r.returncode == 3:
        _STATS["skip-unsupported"] += 1
        pytest.skip("uses elements outside the reader's supported set")
    if r.returncode == 1:
        _STATS["skip-parse-error"] += 1
        pytest.skip("ProtoSpec reader could not parse the file")

    t1, t2, t3 = _parse_summary(r.stdout)
    _STATS["tier3-warnings"] += t3

    if r.returncode == 2:
        if model.name in KNOWN_MALFORMED:
            _STATS["known-malformed"] += 1
            pytest.skip(f"intentional-failure fixture: {KNOWN_MALFORMED[model.name]}")
        pytest.fail(
            f"{t1} tier-1 + {t2} tier-2 error(s) on a MuJoCo-loadable file "
            f"(false positive):\n{r.stdout}"
        )

    assert r.returncode == 0, r.stdout + r.stderr
    assert t1 == 0 and t2 == 0
    _STATS["clean"] += 1
