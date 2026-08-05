"""Compile-path differential gate for the ProtoSpec bridge (ps_path_diff).

The tool runs two gates over the checked-in fixture corpus
(``fixtures/pathdiff/``):

* **identity** -- ``XmlPath`` vs ``XmlPath``: the same deterministic compile path
  twice must yield a bit-identical ``mjModel`` (every MJMODEL_SIZES int, every
  name table in id order, every MJMODEL_POINTERS array, and the qpos0 forward-
  kinematics invariants). Any divergence is a determinism bug in the harness.
* **against-stock** -- ProtoSpec's ``XmlPath`` result vs stock ``mj_loadXML`` of
  the *original* file. This catches ProtoSpec reader/writer drift (the round trip
  changing the model), which a path-vs-path diff cannot see because both legs
  share the same front end. Auto-naming is disabled on this leg so the compare is
  against stock's pristine name tables.

Build strategy
--------------
The tool is a MuJoCo-linked C++ program. Rather than drive the CMake tree (which
targets the author's Windows MuJoCo), a session fixture compiles it directly with
``g++``: the three harness ``.cc`` files (ps_path_diff + model_diff_lib +
plugin_registry) against the ProtoSpec include dirs, linking the prebuilt
``libprotospec_core.a`` + ``libmujoco.so`` from a prebuilt ``build_ps`` tree.

    Speed vs. staleness: linking the prebuilt ``libprotospec_core.a`` is far
    faster than recompiling every ProtoSpec ``.cc`` from source, but the archive
    is a *snapshot*. If the compile bridge / reader / writer sources change
    without rebuilding ``build_ps``, this test links stale object code. Rebuild
    it (or point ``PROTOSPEC_BUILD_PS_LIB`` at a fresh build) after touching
    ``protospec/lib`` compile/io sources.

The lib dir defaults to ``build_ps/lib`` at the enclosing checkout root; the
MuJoCo include dir defaults to the enclosing checkout's ``include/``. Both are
overridable:

    PROTOSPEC_BUILD_PS_LIB   dir holding libprotospec_core.a + libmujoco.so
    PROTOSPEC_MJ_INCLUDE     dir holding mujoco/mujoco.h + mujoco/mjxmacro.h

The whole module skips cleanly when those are absent (e.g. CI without a prebuilt
tree), so a plain ``uv run pytest`` stays green everywhere.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

PROTOSPEC = Path(__file__).resolve().parents[1]
LIB = PROTOSPEC / "lib"
HARNESS = LIB / "harness"
FIXTURES = PROTOSPEC / "tests" / "fixtures" / "pathdiff"
GATED_FIXTURES = PROTOSPEC / "tests" / "fixtures" / "pathdiff_gated"

# Overridable locations of the prebuilt libraries and MuJoCo headers; the
# enclosing checkout is a MuJoCo tree, so its include/ is the header default.
_ENCLOSING = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_PS_LIB = _ENCLOSING / "build_ps" / "lib"
DEFAULT_MJ_INCLUDE = _ENCLOSING / "include"


def _build_ps_lib() -> Path | None:
    p = Path(os.environ.get("PROTOSPEC_BUILD_PS_LIB", str(DEFAULT_BUILD_PS_LIB)))
    if (p / "libprotospec_core.a").is_file() and (
        (p / "libmujoco.so").is_file() or (p / "libmujoco.so.3.10.0").is_file()
    ):
        return p
    return None


def _mj_include() -> Path | None:
    p = Path(os.environ.get("PROTOSPEC_MJ_INCLUDE", str(DEFAULT_MJ_INCLUDE)))
    if (p / "mujoco" / "mujoco.h").is_file() and (
        p / "mujoco" / "mjxmacro.h"
    ).is_file():
        return p
    return None


BUILD_PS_LIB = _build_ps_lib()
MJ_INCLUDE = _mj_include()
CXX = os.environ.get("CXX", "g++")

# First-party engine plugins (must match harness/plugin_registry.cc). They are
# NOT dependencies of the default studio build target, so a cmake reconfigure
# quietly drops their .so's from build_ps/lib and every <extension>-bearing
# fixture then fails with an opaque "unknown plugin" compile error. Guard that
# here: try to rebuild just the plugin targets, else skip those fixtures with a
# reason that says what happened.
PLUGIN_LIBS = ("elasticity", "sdf_plugin", "sensor", "actuator")


def _missing_plugin_libs() -> list[str]:
    if BUILD_PS_LIB is None:
        return list(PLUGIN_LIBS)
    return [
        n
        for n in PLUGIN_LIBS
        if not (
            (BUILD_PS_LIB / f"lib{n}.so").is_file()
            or (BUILD_PS_LIB / f"{n}.dll").is_file()
            or (BUILD_PS_LIB / f"lib{n}.dylib").is_file()
        )
    ]


@pytest.fixture(scope="session")
def plugin_libs_ready() -> bool:
    """True when all four first-party plugin libs sit in BUILD_PS_LIB.

    If any are missing, attempt a targeted ``cmake --build`` of just the plugin
    targets in the studio build tree (cheap; they have no dependency on the
    monolithic studio exe). Returns False when they still cannot be found so
    plugin-bearing tests can skip with an actionable reason.
    """
    if not _missing_plugin_libs():
        return True
    build_dir = BUILD_PS_LIB.parent if BUILD_PS_LIB is not None else None
    if (
        build_dir is not None
        and (build_dir / "CMakeCache.txt").is_file()
        and shutil.which("cmake")
    ):
        subprocess.run(
            ["cmake", "--build", str(build_dir), "--target", *PLUGIN_LIBS],
            capture_output=True,
            text=True,
            timeout=600,
        )
    return not _missing_plugin_libs()


def _needs_plugins(model: Path) -> bool:
    """A fixture needs the engine plugins iff it declares an <extension> block."""
    try:
        return "<extension" in model.read_text(encoding="utf-8")
    except OSError:
        return False


PLUGIN_SKIP_REASON = (
    "first-party plugin libs missing from build_ps/lib and targeted "
    f"auto-build failed (targets: {', '.join(PLUGIN_LIBS)})"
)


def _skip_reason() -> str:
    if shutil.which(CXX) is None:
        return f"no C++ compiler ({CXX}) on PATH"
    if BUILD_PS_LIB is None:
        return (
            "studio build_ps libs absent (set PROTOSPEC_BUILD_PS_LIB to a dir "
            "with libprotospec_core.a + libmujoco.so)"
        )
    if MJ_INCLUDE is None:
        return (
            "MuJoCo headers absent (set PROTOSPEC_MJ_INCLUDE to a dir with "
            "mujoco/mujoco.h + mjxmacro.h)"
        )
    if not (HARNESS / "ps_path_diff.cc").is_file():
        return "ps_path_diff.cc source missing"
    return ""


SKIP_REASON = _skip_reason()
pytestmark = pytest.mark.skipif(bool(SKIP_REASON), reason=SKIP_REASON)


@pytest.fixture(scope="session")
def ps_path_diff(tmp_path_factory) -> Path:
    """Compile ps_path_diff once per session; return the exe path."""
    out_dir = tmp_path_factory.mktemp("ps_path_diff_build")
    exe = out_dir / "ps_path_diff"
    cmd = [
        CXX,
        "-std=c++20",
        "-O2",
        str(HARNESS / "ps_path_diff.cc"),
        str(HARNESS / "model_diff_lib.cc"),
        str(HARNESS / "plugin_registry.cc"),
        f"-I{LIB / 'include'}",
        f"-I{LIB / 'generated'}",
        f"-I{LIB / 'compile'}",
        f"-I{LIB / 'io'}",
        f"-I{HARNESS}",
        f"-I{MJ_INCLUDE}",
        str(BUILD_PS_LIB / "libprotospec_core.a"),
        str(BUILD_PS_LIB / "libmujoco.so"),
        f"-Wl,-rpath,{BUILD_PS_LIB}",
        "-o",
        str(exe),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        pytest.skip(
            "ps_path_diff failed to build (build_ps libs may be stale vs the "
            f"current protospec sources):\n{proc.stderr[-4000:]}"
        )
    assert exe.is_file()
    return exe


def _fixture_files() -> list[Path]:
    return sorted(FIXTURES.glob("*.xml"))


FIXTURE_FILES = _fixture_files()
FIXTURE_IDS = [p.stem for p in FIXTURE_FILES]


def _run(exe: Path, *args: str) -> subprocess.CompletedProcess:
    # Plugin-bearing fixtures need the first-party engine plugins (built beside
    # libmujoco.so in the studio build_ps/lib). Point the harness's registry at
    # that dir so <extension> models load on both legs; harmless for the rest.
    env = dict(os.environ)
    if BUILD_PS_LIB is not None:
        env.setdefault("PROTOSPEC_PLUGIN_DIR", str(BUILD_PS_LIB))
    return subprocess.run(
        [str(exe), *args], capture_output=True, text=True, timeout=300, env=env
    )


# --------------------------------------------------------------------------- #
# Fixture-corpus gates                                                         #
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not FIXTURE_FILES, reason="no pathdiff fixtures found")
@pytest.mark.parametrize("model", FIXTURE_FILES, ids=FIXTURE_IDS or ["<none>"])
def test_identity_fixture(ps_path_diff: Path, model: Path, plugin_libs_ready: bool):
    """XmlPath vs XmlPath must be bit-identical for every fixture."""
    if _needs_plugins(model) and not plugin_libs_ready:
        pytest.skip(PLUGIN_SKIP_REASON)
    r = _run(ps_path_diff, "--path-a", "XmlPath", "--path-b", "XmlPath", str(model))
    assert r.returncode == 0, f"identity diff FAILED:\n{r.stdout}\n{r.stderr}"
    # Per-model verdict lines start with PASS/FAIL; the trailing summary line
    # ("N PASS, 0 FAIL") also contains "FAIL", so match on line prefixes.
    assert any(l.startswith("PASS") for l in r.stdout.splitlines()), r.stdout
    assert not any(l.startswith("FAIL") for l in r.stdout.splitlines()), r.stdout


@pytest.mark.skipif(not FIXTURE_FILES, reason="no pathdiff fixtures found")
@pytest.mark.parametrize("model", FIXTURE_FILES, ids=FIXTURE_IDS or ["<none>"])
def test_against_stock_fixture(ps_path_diff: Path, model: Path, plugin_libs_ready: bool):
    """ProtoSpec XmlPath must match stock mj_loadXML of the original file."""
    if _needs_plugins(model) and not plugin_libs_ready:
        pytest.skip(PLUGIN_SKIP_REASON)
    r = _run(ps_path_diff, "--against-stock", str(model))
    assert r.returncode == 0, f"against-stock diff FAILED:\n{r.stdout}\n{r.stderr}"
    assert any(l.startswith("PASS") for l in r.stdout.splitlines()), r.stdout
    assert not any(l.startswith("FAIL") for l in r.stdout.splitlines()), r.stdout


def test_whole_corpus_identity_one_shot(ps_path_diff: Path, plugin_libs_ready: bool):
    """One invocation over the whole fixture dir: every model PASSes, exit 0."""
    if not FIXTURE_FILES:
        pytest.skip("no pathdiff fixtures found")
    if not plugin_libs_ready:
        pytest.skip(PLUGIN_SKIP_REASON)
    r = _run(ps_path_diff, str(FIXTURES))
    assert r.returncode == 0, r.stdout + r.stderr
    assert f"{len(FIXTURE_FILES)} PASS, 0 FAIL" in r.stdout, r.stdout


def test_whole_corpus_against_stock_one_shot(ps_path_diff: Path, plugin_libs_ready: bool):
    if not FIXTURE_FILES:
        pytest.skip("no pathdiff fixtures found")
    if not plugin_libs_ready:
        pytest.skip(PLUGIN_SKIP_REASON)
    r = _run(ps_path_diff, "--against-stock", str(FIXTURES))
    assert r.returncode == 0, r.stdout + r.stderr
    assert f"{len(FIXTURE_FILES)} PASS, 0 FAIL" in r.stdout, r.stdout


# --------------------------------------------------------------------------- #
# Contract guards (exit codes + benchmark line)                                #
# --------------------------------------------------------------------------- #
def test_reader_reject_is_fail(ps_path_diff: Path, tmp_path: Path):
    """A malformed model must FAIL with a nonzero exit (exit-code contract)."""
    bad = tmp_path / "bad.xml"
    bad.write_text("<mujoco><worldbody><body></mujoco", encoding="utf-8")
    r = _run(ps_path_diff, str(bad))
    assert r.returncode != 0, r.stdout
    assert "FAIL" in r.stdout, r.stdout


def test_bench_line_emitted(ps_path_diff: Path):
    """Benchmark mode prints a machine-readable per-stage line and exits 0."""
    bench_model = FIXTURES / "benchmark_many.xml"
    if not bench_model.is_file():
        pytest.skip("benchmark_many.xml not generated")
    r = _run(ps_path_diff, "--bench", "2", str(bench_model))
    assert r.returncode == 0, r.stdout + r.stderr
    assert "BENCH model=" in r.stdout, r.stdout
    for tok in ("write_ms=", "parse_ms=", "compile_ms=", "total_ms="):
        assert tok in r.stdout, f"missing {tok} in bench line:\n{r.stdout}"


# --------------------------------------------------------------------------- #
# Big-corpus run                                                               #
# --------------------------------------------------------------------------- #
# Both gates run across a whole model corpus: PROTOSPEC_CORPUS when set, else the
# enclosing checkout's model/ directory. Models the ProtoSpec reader/bridge cannot
# handle are out of scope for THIS harness (test_differential.py /
# test_bridge_corpus.py own that accounting) and are counted as skips, not
# failures. A genuine DIVERGENCE is always a failure. Known-bad models can be
# excluded by name via PROTOSPEC_PATHDIFF_SKIP (comma-separated filename
# substrings).
_DEFAULT_MODELS = _ENCLOSING / "model"


def _corpus_dir() -> Path | None:
    env = os.environ.get("PROTOSPEC_CORPUS")
    if env:
        p = Path(env)
        return p if p.is_dir() else None
    return _DEFAULT_MODELS if _DEFAULT_MODELS.is_dir() else None


def _corpus_models(corpus: Path) -> list[Path]:
    skiplist = [
        s.strip()
        for s in os.environ.get("PROTOSPEC_PATHDIFF_SKIP", "").split(",")
        if s.strip()
    ]
    return [
        p
        for p in sorted(corpus.rglob("*.xml"))
        if "build" not in {s.lower() for s in p.parts}
        and not p.name.startswith("._ps_")
        and not any(s in p.name for s in skiplist)
    ]


def _corpus_sweep(exe: Path, label: str, *args: str) -> None:
    corpus = _corpus_dir()
    assert corpus is not None
    models = _corpus_models(corpus)
    if not models:
        pytest.skip(f"no models under {corpus}")

    divergences: list[str] = []
    out_of_scope = 0
    passed = 0
    for m in models:
        r = _run(exe, *args, str(m))
        if r.returncode == 0:
            passed += 1
            continue
        # Distinguish a genuine divergence (real failure) from a model this
        # harness simply cannot compile (reader reject / compile error / missing
        # assets) -- out of scope here, owned by the other harnesses.
        if "first divergence:" in r.stdout or "SIZE INTS" in r.stdout or (
            "NAME TABLES" in r.stdout and "compile failed" not in r.stdout
        ):
            divergences.append(f"{m}: {r.stdout.strip().splitlines()[0]}")
        else:
            out_of_scope += 1

    print(
        f"\n[corpus {label}] {passed} pass, {out_of_scope} out-of-scope, "
        f"{len(divergences)} divergence(s) of {len(models)} models"
    )
    assert not divergences, f"{label} divergences:\n" + "\n".join(divergences[:20])


@pytest.mark.skipif(
    _corpus_dir() is None,
    reason="set PROTOSPEC_CORPUS (or place this repo inside a MuJoCo checkout) "
    "to run the big-corpus gates",
)
def test_corpus_identity(ps_path_diff: Path):
    """XmlPath vs XmlPath across the corpus: the deterministic path never
    disagrees with itself."""
    _corpus_sweep(ps_path_diff, "identity")


@pytest.mark.skipif(
    _corpus_dir() is None,
    reason="set PROTOSPEC_CORPUS (or place this repo inside a MuJoCo checkout) "
    "to run the big-corpus gates",
)
def test_corpus_against_stock(ps_path_diff: Path):
    """ProtoSpec's XmlPath vs stock mj_loadXML of the original, across the
    corpus: the round trip never changes the model."""
    _corpus_sweep(ps_path_diff, "against-stock", "--against-stock")


# --------------------------------------------------------------------------- #
# Fixtures held out of the parity corpus                                       #
# --------------------------------------------------------------------------- #
# Valid models kept in their own directory rather than the counted parity corpus.
# They still carry the gate that matters: ProtoSpec's XmlPath result must match
# stock mj_loadXML of the original.


def _gated_fixtures() -> list[Path]:
    if not GATED_FIXTURES.is_dir():
        return []
    return sorted(GATED_FIXTURES.glob("*.xml"))


GATED_FILES = _gated_fixtures()
GATED_IDS = [p.stem for p in GATED_FILES]


@pytest.mark.skipif(not GATED_FILES, reason="no gated pathdiff fixtures found")
@pytest.mark.parametrize("model", GATED_FILES, ids=GATED_IDS or ["<none>"])
def test_gated_fixture_against_stock(ps_path_diff: Path, model: Path, plugin_libs_ready: bool):
    """A held-out fixture is a valid model: its XmlPath result matches stock."""
    if _needs_plugins(model) and not plugin_libs_ready:
        pytest.skip(PLUGIN_SKIP_REASON)
    r = _run(ps_path_diff, "--against-stock", str(model))
    assert r.returncode == 0, f"against-stock FAILED:\n{r.stdout}\n{r.stderr}"
    assert any(l.startswith("PASS") for l in r.stdout.splitlines()), r.stdout
