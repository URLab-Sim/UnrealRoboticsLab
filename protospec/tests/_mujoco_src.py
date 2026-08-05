"""Shared discovery of a MuJoCo source checkout for the integration tests.

The bootstrap extractors (tools/bootstrap/extract_*.py) and the lift registry re-run
against a real MuJoCo source tree. ``protospec/`` lives inside a MuJoCo checkout, so the
enclosing repository is the default; ``PROTOSPEC_MUJOCO_SRC`` overrides it (point it at a
checkout root, i.e. the directory containing ``include/`` and ``src/``). When neither
resolves to a plausible tree the integration tests skip rather than fail, so collection
never depends on the tree existing.
"""

from __future__ import annotations

import os
from pathlib import Path

ENV_VAR = "PROTOSPEC_MUJOCO_SRC"

SKIP_REASON = f"set {ENV_VAR} to a MuJoCo source checkout to run this test"


def _plausible(root: Path) -> bool:
    return (root / "include" / "mujoco" / "mjspec.h").is_file() and (root / "src").is_dir()


def mujoco_src() -> Path | None:
    """Return the MuJoCo checkout root, or ``None`` (drives ``pytest.mark.skipif``).

    ``PROTOSPEC_MUJOCO_SRC`` wins when set (``None`` if it is not a plausible MuJoCo
    tree — missing ``include/mujoco/mjspec.h`` or ``src/``); otherwise the repository
    enclosing ``protospec/`` is used when plausible.
    """
    raw = os.environ.get(ENV_VAR)
    if raw:
        root = Path(raw).expanduser()
        return root if _plausible(root) else None
    enclosing = Path(__file__).resolve().parents[2]
    return enclosing if _plausible(enclosing) else None


MUJOCO_SRC = mujoco_src()
