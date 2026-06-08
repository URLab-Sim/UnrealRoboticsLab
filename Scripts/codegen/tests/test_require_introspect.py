# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Verifies the ``--require-introspect`` flag's exit-code contract.

The clang-AST introspect snapshot drives the embedded-C++ drift checks
(value_map / hand-enum / mjs_array shape). Letting codegen silently fall
back to a stale or missing snapshot turned subtle rule typos into runtime
fall-through; this flag is the hard-fail gate.

These tests shell out to ``generate_ue_components.py`` so the real
``main()`` arg-parsing + early-exit branches are exercised.
"""

from __future__ import annotations

import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
_PLUGIN_ROOT = os.path.dirname(os.path.dirname(_CODEGEN_DIR))
_GENERATOR = os.path.join(_CODEGEN_DIR, "generate_ue_components.py")


def _run(extra_args: list[str], introspect_path: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, _GENERATOR,
         "--check", "--introspect", introspect_path,
         *extra_args],
        capture_output=True, text=True, cwd=_PLUGIN_ROOT,
    )


def test_require_introspect_default_hard_fails_when_missing(tmp_path):
    """Default behaviour (no flag = require_introspect=True) MUST exit
    non-zero with the documented exit code (3) when the snapshot is
    absent. Closes the silent-fallback path."""
    missing = str(tmp_path / "introspect_snapshot.json")
    assert not os.path.exists(missing)
    proc = _run([], missing)
    assert proc.returncode == 3, (
        f"expected exit 3, got {proc.returncode}\n"
        f"stdout={proc.stdout!r}\nstderr={proc.stderr!r}"
    )
    assert "introspect snapshot missing" in proc.stderr


def test_no_require_introspect_bypasses_missing_snapshot(tmp_path):
    """The opt-out flag prints a warning but does not exit non-zero on
    a missing snapshot alone — the rest of the run still has to be
    clean for an exit-0."""
    missing = str(tmp_path / "introspect_snapshot.json")
    proc = _run(["--no-require-introspect"], missing)
    assert "warning" in proc.stderr.lower()
    assert "introspect snapshot missing" in proc.stderr
    assert proc.returncode in (0, 2), (
        f"expected exit 0 (clean) or 2 (strict diags), got {proc.returncode}\n"
        f"stdout={proc.stdout!r}\nstderr={proc.stderr!r}"
    )
