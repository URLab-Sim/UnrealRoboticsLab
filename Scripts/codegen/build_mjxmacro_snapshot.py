# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""
Parse MuJoCo's mjxmacro.h into a machine-readable JSON snapshot.

Output: Scripts/mjxmacro_snapshot.json with mjModel/mjData pointer
fields grouped by category, plus mjOption / mjStatistic / mjVisual
field lists for top-level struct codegen.

Each entry carries the C type, field name, outer dimension (e.g.
``njnt``, ``nv``), and inner stride (e.g. ``1``, ``3``, ``mjNREF``,
``MJ_M(nuser_jnt)`` for runtime-dimensioned arrays).
"""

from __future__ import annotations

import json
import os
import re
import sys
from typing import Any, Dict, List, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
DEFAULT_MJXMACRO = os.path.join(
    PLUGIN_ROOT, "third_party", "install", "MuJoCo", "include", "mujoco", "mjxmacro.h"
)
DEFAULT_OUTPUT = os.path.join(PLUGIN_ROOT, "Scripts", "mjxmacro_snapshot.json")

# Block names we care about. Categorized so the consumer knows the
# kind of data (per-element pointer table vs flat struct fields).
POINTER_BLOCKS_MJMODEL = [
    "MJMODEL_POINTERS_BODY",
    "MJMODEL_POINTERS_JOINT",
    "MJMODEL_POINTERS_DOF",
    "MJMODEL_POINTERS_TREE",
    "MJMODEL_POINTERS_GEOM",
    "MJMODEL_POINTERS_SITE",
    "MJMODEL_POINTERS_CAMERA",
    "MJMODEL_POINTERS_LIGHT",
    "MJMODEL_POINTERS_FLEX",
    "MJMODEL_POINTERS_FLEXVERT",
    "MJMODEL_POINTERS_FLEXEDGE",
    "MJMODEL_POINTERS_FLEXELEM",
    "MJMODEL_POINTERS_FLEXTEXCOORD",
    "MJMODEL_POINTERS_MESH",
    "MJMODEL_POINTERS_SKIN",
    "MJMODEL_POINTERS_HFIELD",
    "MJMODEL_POINTERS_TEXTURE",
    "MJMODEL_POINTERS_MATERIAL",
    "MJMODEL_POINTERS_PAIR",
    "MJMODEL_POINTERS_EXCLUDE",
    "MJMODEL_POINTERS_EQUALITY",
    "MJMODEL_POINTERS_TENDON",
    "MJMODEL_POINTERS_ACTUATOR",
    "MJMODEL_POINTERS_SENSOR",
    "MJMODEL_POINTERS_NUMERIC",
    "MJMODEL_POINTERS_TEXT",
    "MJMODEL_POINTERS_TUPLE",
    "MJMODEL_POINTERS_KEY",
    "MJMODEL_POINTERS_PLUGIN",
    "MJMODEL_POINTERS_NAMES",
    "MJMODEL_POINTERS_PATHS",
]

POINTER_BLOCKS_MJDATA = [
    "MJDATA_POINTERS_PREAMBLE",
    "MJDATA_POINTERS",
    "MJDATA_ARENA_POINTERS",
    "MJDATA_ARENA_POINTERS_PRIMAL",
    "MJDATA_ARENA_POINTERS_DUAL",
    "MJDATA_ARENA_POINTERS_ISLAND",
    "MJDATA_ARENA_POINTERS_SOLVER",
]

# Flat struct field blocks (no outer dimension — just type + name + size).
STRUCT_FIELD_BLOCKS = [
    "MJOPTION_FIELDS",
    "MJSTATISTIC_FIELDS",
    "MJVISUAL_GLOBAL_FIELDS",
    "MJVISUAL_QUALITY_FIELDS",
    "MJVISUAL_HEADLIGHT_FIELDS",
    "MJVISUAL_MAP_FIELDS",
    "MJVISUAL_SCALE_FIELDS",
    "MJVISUAL_RGBA_FIELDS",
]

# Regexes
_BLOCK_START_RE = re.compile(r"^\s*#define\s+(\w+)\s*(?:\(\s*\w+\s*\))?\s*\\?\s*$")
# X-macro entry. Tolerates 4 or 3-tuple forms (pointer vs flat field).
# Captures parenthesised contents; field split below trims/normalizes.
_X_ENTRY_RE = re.compile(r"^\s*X(?:VEC)?\s*\(\s*(.*?)\s*\)\s*\\?\s*$")


def _strip_comments(text: str) -> str:
    # Drop //... comments + /* ... */ blocks.
    text = re.sub(r"//[^\n]*", "", text)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return text


def _join_continued_lines(text: str) -> str:
    # mjxmacro spreads multi-line macros via backslash continuation; preserve
    # individual entries on their own lines for downstream parsing.
    return text


_BODY_ENTRY_RE = re.compile(r"^\s*(X|XVEC)\s*\(")


def _parse_block_body(lines: List[str], start_idx: int) -> List[str]:
    """Collect the entries of a block starting at ``start_idx``. Returns
    the entry lines as raw strings (each looks like ``X ( ... )`` or
    ``XVEC ( ... )``). Stops at the first non-entry, non-empty line."""
    out: List[str] = []
    i = start_idx
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if not stripped:
            break
        if _BODY_ENTRY_RE.match(stripped):
            out.append(stripped.rstrip("\\").rstrip())
            i += 1
            continue
        break
    return out


def _split_csv(tup: str) -> List[str]:
    """Split a comma-separated tuple body that may contain MJ_M(...) calls."""
    parts: List[str] = []
    depth = 0
    cur: List[str] = []
    for ch in tup:
        if ch == "," and depth == 0:
            parts.append("".join(cur).strip())
            cur = []
            continue
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        cur.append(ch)
    if cur:
        parts.append("".join(cur).strip())
    return parts


def _parse_pointer_entry(raw: str) -> Optional[Dict[str, Any]]:
    """Parse `X(type, name, outer_dim, stride)` or `XVEC(...)`."""
    m = _X_ENTRY_RE.match(raw)
    if not m:
        return None
    fields = _split_csv(m.group(1))
    if len(fields) == 4:
        ctype, name, outer_dim, stride = fields
        return {
            "type": ctype,
            "name": name,
            "outer_dim": outer_dim,
            "stride": stride,
        }
    return None


def _parse_struct_field_entry(raw: str) -> Optional[Dict[str, Any]]:
    """Parse `X(type, name, count)` or `X(name, count)` or `XVEC(...)`.

    mjxmacro flat-struct blocks have a few shapes:
    - `X(type, name, count)` (mjOption)
    - `X(name, count)` (mjStatistic — no type, defaults to mjtNum)
    - `X(type, name)` (mjVisual fields — no explicit count, treated as scalar)
    - `XVEC(type, name, count)` (vec3 / vec5 / etc.)
    """
    m = _X_ENTRY_RE.match(raw)
    if not m:
        return None
    fields = _split_csv(m.group(1))
    is_vec = raw.lstrip().startswith("XVEC")
    if len(fields) == 3:
        ctype, name, count = fields
    elif len(fields) == 2:
        # Could be (type, name) or (name, count). If first token looks
        # like a C type keep it as type; else treat as (name, count) and
        # default type to mjtNum.
        if fields[0] in ("int", "float", "mjtNum", "mjtByte", "char"):
            ctype, name, count = fields[0], fields[1], "1"
        else:
            ctype, name, count = "mjtNum", fields[0], fields[1]
    else:
        return None
    return {
        "type": ctype,
        "name": name,
        "count": count,
        "is_vec": is_vec,
    }


def parse_mjxmacro(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        raw_text = f.read()
    text = _strip_comments(raw_text)
    lines = text.split("\n")

    out: Dict[str, Any] = {
        "_meta": {
            "source": os.path.relpath(path, PLUGIN_ROOT),
            "schema_kind": "mjxmacro",
        },
        "mjmodel_pointers": {},  # block_name -> list of entries
        "mjdata_pointers": {},
        "struct_fields": {},
    }

    i = 0
    while i < len(lines):
        line = lines[i]
        m = _BLOCK_START_RE.match(line)
        if not m:
            i += 1
            continue
        block_name = m.group(1)
        # Collect block body — entries on subsequent lines.
        entries = _parse_block_body(lines, i + 1)
        if block_name in POINTER_BLOCKS_MJMODEL:
            parsed = [e for e in (_parse_pointer_entry(r) for r in entries) if e]
            out["mjmodel_pointers"][block_name] = parsed
        elif block_name in POINTER_BLOCKS_MJDATA:
            parsed = [e for e in (_parse_pointer_entry(r) for r in entries) if e]
            out["mjdata_pointers"][block_name] = parsed
        elif block_name in STRUCT_FIELD_BLOCKS:
            parsed = [e for e in (_parse_struct_field_entry(r) for r in entries) if e]
            out["struct_fields"][block_name] = parsed
        i += 1
    return out


def main() -> int:
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MJXMACRO
    dst = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUTPUT
    if not os.path.isfile(src):
        print(f"error: mjxmacro.h not found at {src}", file=sys.stderr)
        return 1
    snapshot = parse_mjxmacro(src)
    # Stats for sanity.
    total_model = sum(len(v) for v in snapshot["mjmodel_pointers"].values())
    total_data = sum(len(v) for v in snapshot["mjdata_pointers"].values())
    total_structs = sum(len(v) for v in snapshot["struct_fields"].values())
    print(
        f"parsed mjxmacro.h: {len(snapshot['mjmodel_pointers'])} model blocks "
        f"({total_model} entries), "
        f"{len(snapshot['mjdata_pointers'])} data blocks ({total_data} entries), "
        f"{len(snapshot['struct_fields'])} struct blocks ({total_structs} entries)"
    )
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(snapshot, f, indent=2)
    print(f"wrote {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
