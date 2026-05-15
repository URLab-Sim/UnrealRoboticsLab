# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Parse mjspec.h and emit a JSON snapshot of each `mjs<Name>` struct's
fields so codegen can look up the real C field name (e.g. mjsExclude
holds `bodyname1`, not the schema attr `body1`).

Output: ``Scripts/mjspec_snapshot.json`` with shape
    {
      "structs": {
        "mjsBody":  ["element", "name", "childclass", ...],
        "mjsPair":  ["element", "geomname1", "geomname2", "condim", ...],
        ...
      }
    }

Usage:
    python build_mjspec_snapshot.py
        [--mjspec PATH]   (default: third_party/install/MuJoCo/include/mujoco/mjspec.h)
        [--output PATH]   (default: Scripts/mjspec_snapshot.json)
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))

DEFAULT_MJSPEC = os.path.join(
    PLUGIN_ROOT, "third_party", "install", "MuJoCo", "include", "mujoco", "mjspec.h"
)
DEFAULT_MUJOCO_H = os.path.join(
    PLUGIN_ROOT, "third_party", "install", "MuJoCo", "include", "mujoco", "mujoco.h"
)
DEFAULT_OUTPUT = os.path.join(PLUGIN_ROOT, "Scripts", "mjspec_snapshot.json")

# Matches `typedef struct mjsName_ { ... } mjsName;` blocks.
_STRUCT_RE = re.compile(
    r"typedef\s+struct\s+(mjs\w+)_\s*\{(.*?)\}\s*\1\s*;",
    re.DOTALL,
)

# Inside a struct body, a field line looks like:
#   `<type> <name>;`              -> e.g.  `int condim;`
#   `<type> <name>[<dim>];`       -> e.g.  `mjtNum solref[mjNREF];`
#   `<type>* <name>;`             -> e.g.  `mjString* name;`
#   `<type> <name>[d1][d2];`      -> e.g.  `double data[mjNEQDATA];`
# Comments after the `;` are stripped before matching.
_FIELD_RE = re.compile(
    r"^\s*"
    r"(?:struct\s+)?"                  # optional `struct` keyword
    r"(?P<type>[\w:]+\s*\**)\s+"        # type, possibly with `*`
    r"(?P<name>\w+)"                    # field name
    r"(?:\s*\[[^\]]*\])*"               # optional [dim] suffix(es)
    r"\s*;",
    re.MULTILINE,
)


def parse_structs(text: str) -> dict[str, list[str]]:
    structs: dict[str, list[str]] = {}
    for m in _STRUCT_RE.finditer(text):
        name = m.group(1)
        body = m.group(2)
        # Strip line-end comments.
        body_clean = re.sub(r"//[^\n]*", "", body)
        body_clean = re.sub(r"/\*.*?\*/", "", body_clean, flags=re.DOTALL)
        fields: list[str] = []
        for fm in _FIELD_RE.finditer(body_clean):
            fname = fm.group("name")
            # Skip C-keyword-only matches; the regex shouldn't, but be safe.
            if fname in ("struct", "const", "typedef", "if", "else"):
                continue
            fields.append(fname)
        structs[name] = fields
    return structs


# Matches `MJAPI const char* mjs_setToX(mjsActuator* actuator, ... params ...);`.
# DOTALL so the param list can span lines. `?:` non-capturing to keep groups
# minimal; we re-parse the param string with `_PARAM_RE` below.
_SETTO_RE = re.compile(
    r"MJAPI\s+const\s+char\s*\*\s*"
    r"(?P<name>mjs_setTo\w+)\s*"
    r"\(\s*mjsActuator\s*\*\s*\w+\s*(?:,\s*(?P<params>[^)]*))?\s*\)",
    re.DOTALL,
)

# Inside the param list, each param looks like `<type> <name>[<dim>]?`,
# separated by commas. Handles `double kv[1]`, `double timeconst`, `int input_mode`.
_PARAM_RE = re.compile(
    r"^\s*(?P<c_type>[\w]+)\s+"
    r"(?P<name>\w+)"
    r"(?:\s*\[\s*(?P<dim>\d+)\s*\])?\s*$"
)


def parse_setto_functions(text: str) -> dict[str, dict]:
    """Scrape ``mjs_setTo*`` function declarations from mujoco.h. Skips the
    first ``mjsActuator* actuator`` parameter; returns the remaining
    parameter list with C type and (if present) fixed array dimension."""
    out: dict[str, dict] = {}
    for m in _SETTO_RE.finditer(text):
        name = m.group("name")
        params_text = (m.group("params") or "").strip()
        params: list[dict] = []
        if params_text:
            for raw in params_text.split(","):
                raw_clean = re.sub(r"//[^\n]*", "", raw).strip()
                if not raw_clean:
                    continue
                pm = _PARAM_RE.match(raw_clean)
                if not pm:
                    raise RuntimeError(
                        f"unparsed mjs_setTo* param: {raw_clean!r} in {name}"
                    )
                params.append({
                    "name": pm.group("name"),
                    "c_type": pm.group("c_type"),
                    "array_dim": int(pm.group("dim")) if pm.group("dim") else None,
                })
        out[name] = {"params": params}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mjspec", default=DEFAULT_MJSPEC)
    ap.add_argument("--mujoco-h", default=DEFAULT_MUJOCO_H)
    ap.add_argument("--output", default=DEFAULT_OUTPUT)
    args = ap.parse_args()

    if not os.path.exists(args.mjspec):
        print(f"ERROR: mjspec.h not found at {args.mjspec}", file=sys.stderr)
        return 1

    with open(args.mjspec, "r", encoding="utf-8") as f:
        text = f.read()

    structs = parse_structs(text)
    out = {"structs": structs}

    # Also scrape mjs_setTo* signatures from mujoco.h so codegen can emit
    # the actuator preset calls from introspection instead of hand-written
    # C++ literals in codegen_rules.json.
    if os.path.exists(args.mujoco_h):
        with open(args.mujoco_h, "r", encoding="utf-8") as f:
            mj_text = f.read()
        out["setto_functions"] = parse_setto_functions(mj_text)
    else:
        print(f"WARN: mujoco.h not found at {args.mujoco_h} — skipping setto functions", file=sys.stderr)
        out["setto_functions"] = {}

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    total_fields = sum(len(v) for v in structs.values())
    print(f"wrote {args.output}: {len(structs)} structs, {total_fields} fields, "
          f"{len(out['setto_functions'])} setto functions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
