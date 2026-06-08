# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Golden-file gate.

Runs ``generate_ue_components.py --check`` against the real schema /
mjxmacro / rules / mjspec snapshots and asserts:
  1. Every emitted file is byte-identical to what's on disk (exit 0).
  2. The run produced zero diagnostics in stderr — catches new warn-mode
     drift introduced by a rule change that the byte-identity check
     can't see (drift fires on stderr without changing the emit output).
"""

from __future__ import annotations

import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_GENERATOR = os.path.normpath(os.path.join(_HERE, "..", "generate_ue_components.py"))


def test_codegen_check_clean() -> None:
    """`generate_ue_components.py --check` exits 0 with no diagnostics."""
    proc = subprocess.run(
        [sys.executable, _GENERATOR, "--check"],
        capture_output=True,
        text=True,
    )
    stdout = proc.stdout or ""
    stderr = proc.stderr or ""
    assert proc.returncode == 0, (
        "codegen drift detected: re-running the generator would rewrite one or "
        "more on-disk files. Re-run `python Scripts/codegen/generate_ue_components.py` "
        "to regenerate, or update the emit functions if hand-edits to "
        "CODEGEN_*_START/END regions slipped past the generator.\n"
        f"--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
    )
    # Diagnostics fire on stderr without changing emit output — catches the
    # warn-mode drift class that the byte-identity check is blind to.
    assert "[diagnostic]" not in stderr, (
        "codegen ran clean (no file rewrites) but emitted one or more "
        "diagnostics. A rule entry or snapshot likely drifted; rules JSON or "
        "an intentionally_* allowlist needs updating.\n"
        f"--- stderr ---\n{stderr}"
    )
