# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""One-shot regen pipeline for a MuJoCo version bump.

Runs every codegen step in order:

  1. Rebuild snapshots from the installed MuJoCo headers + submodule source:
     - Scripts/codegen/snapshots/mjspec_snapshot.json   (mjsX struct fields + mjs_setTo* sigs + mjt* enums)
     - Scripts/codegen/snapshots/mjxmacro_snapshot.json (model/data array layouts)
     - Scripts/codegen/snapshots/mjcf_schema_snapshot.json (MJCF schema + per-sensor objtype scrape)
     - Scripts/codegen/snapshots/introspect_snapshot.json (clang-AST scrape — optional, requires libclang)
  2. Run the C++ generator (generate_ue_components.py) against the fresh
     snapshots. Emits / updates the per-component .h/.cpp files in Source/.

Day-to-day this is the script developers run after either:
  - bumping the third_party/MuJoCo submodule pointer + rebuilding the
    install (build_all.ps1 / build_all.sh), OR
  - editing Scripts/codegen/codegen_rules.json.

Usage:
    python Scripts/codegen/regen_all.py
    python Scripts/codegen/regen_all.py --skip-codegen   # snapshots only

Exit code:
    0 on success, non-zero if any sub-step fails.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time


HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))


def _run(label: str, cmd: list[str]) -> int:
    print(f"\n>>> {label}", flush=True)
    t0 = time.monotonic()
    proc = subprocess.run(cmd, cwd=PLUGIN_ROOT)
    dt = time.monotonic() - t0
    if proc.returncode != 0:
        print(f"!!! {label} FAILED (exit {proc.returncode}, {dt:.1f}s)", flush=True)
    else:
        print(f"... {label} ok ({dt:.1f}s)", flush=True)
    return proc.returncode


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--skip-codegen", action="store_true",
        help="Only rebuild the three snapshot JSONs; skip generate_ue_components.py.",
    )
    args = ap.parse_args(argv)

    py = sys.executable

    steps: list[tuple[str, list[str]]] = [
        ("mjspec snapshot",
            [py, os.path.join(HERE, "build_mjspec_snapshot.py")]),
        ("mjxmacro snapshot",
            [py, os.path.join(HERE, "build_mjxmacro_snapshot.py")]),
        ("MJCF schema snapshot",
            [py, os.path.join(HERE, "build_mjcf_schema_snapshot.py")]),
    ]
    # Introspect snapshot needs libclang — optional, but if absent the
    # codegen falls back to the stale on-disk JSON. Run it AFTER the
    # other snapshots so its failure (e.g. missing libclang) doesn't
    # block the rest.
    introspect_step = (
        "introspect snapshot (clang AST — optional)",
        [py, os.path.join(HERE, "build_introspect_snapshot.py")],
    )
    steps.append(introspect_step)
    if not args.skip_codegen:
        steps.append(("UE component codegen",
            [py, os.path.join(HERE, "generate_ue_components.py")]))

    for label, cmd in steps:
        rc = _run(label, cmd)
        if rc != 0:
            # Introspect is optional — warn but continue when it's the
            # one that failed (most commonly: libclang not on PATH).
            if label.startswith("introspect snapshot"):
                print(
                    "    (introspect is optional; codegen will use the "
                    "existing snapshot — type-shape drift may be stale "
                    "until you install libclang.)",
                    flush=True,
                )
                continue
            return rc

    print("\nAll regen steps completed.", flush=True)
    print("Next: close the editor, run the URLab build + test cycle to verify.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
