"""Opt-in AddressSanitizer gate.

Wraps ``tools/run_asan.ps1`` (Windows) / ``tools/run_asan.sh`` (Linux) so the
ASan pass can run as part of pytest, but only when explicitly asked for -- it
rebuilds a separate instrumented tree and is far slower than the unit suites.

Run it with::

    PROTOSPEC_RUN_ASAN=1 pytest -m asan

Without ``PROTOSPEC_RUN_ASAN`` the single test skips (so a plain ``pytest`` run
and even ``pytest -m asan`` on a machine without the ASan toolchain stay green).
The scope is the MuJoCo-free surface; see the script headers and HANDOFF.md for
why the MuJoCo-linked suites are excluded on the vendored toolchain.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.asan
def test_asan_pass() -> None:
    if not os.environ.get("PROTOSPEC_RUN_ASAN"):
        pytest.skip("set PROTOSPEC_RUN_ASAN=1 to run the ASan pass (slow; rebuilds)")

    if sys.platform == "win32":
        pwsh = shutil.which("pwsh") or shutil.which("powershell")
        if pwsh is None:
            pytest.skip("no PowerShell found")
        cmd = [pwsh, "-NoProfile", "-File", str(ROOT / "tools" / "run_asan.ps1")]
    else:
        cmd = ["bash", str(ROOT / "tools" / "run_asan.sh")]

    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    sys.stdout.write(proc.stdout)
    sys.stderr.write(proc.stderr)
    assert proc.returncode == 0, "AddressSanitizer reported issues (see output above)"
