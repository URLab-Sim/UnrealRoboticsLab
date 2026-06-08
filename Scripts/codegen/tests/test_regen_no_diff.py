# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Golden-file gate.

Runs ``generate_ue_components.py --check`` against the real schema /
mjxmacro / rules / mjspec snapshots and asserts every emitted file is
byte-identical to what's already on disk. Catches the failure mode that
prompted D4: hand-edits to codegen-managed regions that the emitters
weren't updated to match.

Run as part of the codegen pytest suite, and from CI.
"""

from __future__ import annotations

import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_GENERATOR = os.path.normpath(os.path.join(_HERE, "..", "generate_ue_components.py"))


def test_codegen_check_clean() -> None:
    """`generate_ue_components.py --check` must exit 0 (zero files changed)."""
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
