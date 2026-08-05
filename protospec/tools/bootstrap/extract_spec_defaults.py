"""Extract MuJoCo's spec-struct default values from ``user_init.c`` into a JSON snapshot.

Every ``mjs_default*`` initializer in ``src/user/user_init.c`` sets the default value of one
spec struct (``mjsBody``, ``mjsGeom``, ``mjSpec``, ...). Each function opens with a
``memset(x, 0, sizeof(...))`` and then explicitly assigns the fields whose default is not the
all-zero pattern. This module parses those functions into ``{struct: {field: default}}`` where
a default is one of:

  * a JSON number / string literal as written,
  * a fixed array of such values (per-index assignments folded into a dense list, gaps filled
    from the leading ``memset``),
  * a symbolic constant kept verbatim as ``{"value": "mjGEOM_SPHERE"}``,
  * ``{"value": "mjLIMITED_AUTO", "auto": true}`` for ``*_AUTO`` tri-state enum defaults,
  * a sentinel marker ``{"sentinel": "mjNAN"}`` / ``{"sentinel": -1}`` for values that encode
    "unset" (mjSpec's pre-presence-tracking wart; see plan DR-1),
  * ``{"unset_by_memset": true}`` for struct members the initializer never touches (their
    default is whatever the leading ``memset(0)`` leaves), and
  * ``{"opaque": "<statement>"}`` for values produced by a helper call (``mj_defaultSolRefImp``,
    ``mj_defaultOption``, ...) that is not worth reimplementing here.

The member list per struct (needed to enumerate the memset-only fields) comes from a deliberately
minimal parse of ``include/mujoco/mjspec.h`` -- member names only, no types.

A scalar ``-1`` denotes "unset/auto" throughout this file (settotalmass, memory, nuser_*,
maxhullvert, actdim, thickness, metallic, roughness) and is recorded as a sentinel; ``-1`` as an
array component (light dir, tendon springlength) is a positional value and kept literal.

Any assignment whose right-hand side cannot be classified fails loudly with a file:line, never a
silent skip.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

SENTINEL_NAN = "mjNAN"

_FUNC_OPEN = re.compile(
    r"^\s*void\s+mjs_default\w+\s*\(\s*(?P<type>[A-Za-z_]\w*)\s*\*\s*(?P<var>[A-Za-z_]\w*)\s*\)\s*\{"
)
_STRUCT_OPEN = re.compile(r"^\s*typedef struct\s+(?P<tag>\w+)\s*\{")
_STRUCT_CLOSE = re.compile(r"^\s*\}\s*(?P<name>\w+)\s*;")
_MEMBER = re.compile(r"([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*;\s*(?://.*)?$")

_LOCAL_STR = re.compile(r'^char\s+\w+\s*\[[^\]]*\]\s*=\s*"(?P<val>.*)"$')
_LOCAL_DECL_NAME = re.compile(r'^char\s+(?P<name>\w+)\s*\[')
_MEMSET = re.compile(r"^memset\s*\(\s*(?P<dst>\w+)\s*,\s*0\s*,\s*sizeof\([^)]*\)\s*\)$")
_MEMCPY = re.compile(r"^memcpy\s*\(\s*(?P<dst>[^,]+?)\s*,\s*(?P<src>\w+)\s*,\s*sizeof\([^)]*\)\s*\)$")
_CALL = re.compile(r"^(?P<fn>\w+)\s*\((?P<args>.*)\)$")

_NUMBER = re.compile(r"^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?[fFlL]?$")
_CHAR = re.compile(r"^'(.)'$")
_STRING = re.compile(r'^"(.*)"$')
_SYMBOL = re.compile(r"^[A-Za-z_]\w*(?:[+\-*/][A-Za-z0-9_]+)*$")


class SpecDefaultsError(Exception):
    """Raised when ``user_init.c`` contains a pattern this extractor cannot classify."""


def parse_struct_members(header_text: str) -> dict[str, list[str]]:
    """Map every ``typedef struct NAME_ { ... } NAME;`` to its member names, in order."""
    members: dict[str, list[str]] = {}
    current: list[str] | None = None
    for raw in header_text.splitlines():
        if current is None:
            if _STRUCT_OPEN.match(raw):
                current = []
            continue
        close = _STRUCT_CLOSE.match(raw)
        if close:
            members[close.group("name")] = current
            current = None
            continue
        m = _MEMBER.search(raw.strip())
        if m:
            current.append(m.group(1))
    return members


def _strip_comment(line: str) -> str:
    idx = line.find("//")
    return line if idx < 0 else line[:idx]


def _iter_functions(src_text: str):
    """Yield ``(type, var, body_statements)`` for each ``mjs_default*`` function.

    Each statement is ``(line_no, text)`` with comments removed and the trailing ``;`` dropped.
    """
    lines = src_text.splitlines()
    i = 0
    while i < len(lines):
        m = _FUNC_OPEN.match(lines[i])
        if not m:
            i += 1
            continue
        depth = lines[i].count("{") - lines[i].count("}")
        body: list[tuple[int, str]] = []
        buf = ""
        buf_line = 0
        j = i + 1
        while j < len(lines) and depth > 0:
            code = _strip_comment(lines[j])
            depth += code.count("{") - code.count("}")
            if depth <= 0:
                code = code[: code.rfind("}")] if "}" in code else code
            if buf == "":
                buf_line = j + 1
            buf += code
            while ";" in buf:
                stmt, _, buf = buf.partition(";")
                body.append((buf_line, stmt.strip()))
                buf_line = j + 1
            j += 1
        yield m.group("type"), m.group("var"), body
        i = j


def _parse_number(token: str):
    text = token[:-1] if token[-1] in "fFlL" else token
    if "." in text or "e" in text.lower():
        return float(text)
    return int(text)


def _classify(expr: str, where: str):
    """Classify an assignment right-hand side into its JSON default representation."""
    expr = expr.strip()
    if expr == SENTINEL_NAN:
        return {"sentinel": SENTINEL_NAN}
    m = _CHAR.match(expr)
    if m:
        return m.group(1)
    if _NUMBER.match(expr):
        return _parse_number(expr)
    m = _STRING.match(expr)
    if m:
        return m.group(1)
    if _SYMBOL.match(expr):
        if expr.endswith("_AUTO"):
            return {"value": expr, "auto": True}
        return {"value": expr}
    raise SpecDefaultsError(f"{where}: cannot classify value {expr!r}")


def _scalar_value(value):
    """A bare scalar ``-1`` means "unset/auto" throughout user_init.c (not for array members)."""
    if isinstance(value, (int, float)) and value == -1:
        return {"sentinel": -1}
    return value


def _parse_target(target: str, var: str, where: str) -> tuple[str, int | None]:
    prefix = var + "->"
    if not target.startswith(prefix):
        raise SpecDefaultsError(f"{where}: assignment target {target!r} does not address {var!r}")
    rest = target[len(prefix):].strip()
    m = re.fullmatch(r"([A-Za-z_][\w.]*)\[(\d+)\]", rest)
    if m:
        return m.group(1), int(m.group(2))
    m = re.fullmatch(r"[A-Za-z_][\w.]*", rest)
    if m:
        return rest, None
    raise SpecDefaultsError(f"{where}: cannot parse assignment target {target!r}")


def _fields_touched(args: str, var: str) -> list[str]:
    """Field paths of ``var`` referenced inside a helper-call argument list."""
    found: list[str] = []
    for path in re.findall(rf"&?\b{re.escape(var)}->([A-Za-z_][\w.]*)(?:\[\d+\])?", args):
        if path not in found:
            found.append(path)
    return found


def extract_defaults(
    src_text: str, struct_members: dict[str, list[str]], source_label: str = "user_init.c"
) -> dict[str, dict]:
    """Parse every ``mjs_default*`` function into ``{struct: {field: default}}``."""
    defaults: dict[str, dict] = {}
    for ctype, var, body in _iter_functions(src_text):
        fields: dict[str, object] = {}
        arrays: dict[str, dict[int, object]] = {}
        assigned_top: set[str] = set()
        locals_str: dict[str, str] = {}
        memset_zero = False

        for line_no, stmt in body:
            where = f"{source_label}:{line_no}"
            if not stmt:
                continue

            local = _LOCAL_STR.match(stmt)
            if local:
                name = _LOCAL_DECL_NAME.match(stmt).group("name")
                locals_str[name] = local.group("val")
                continue
            if stmt.startswith("char ") or stmt.startswith("const "):
                raise SpecDefaultsError(f"{where}: unrecognized local declaration {stmt!r}")

            memset = _MEMSET.match(stmt)
            if memset:
                if memset.group("dst") == var:
                    memset_zero = True
                continue

            memcpy = _MEMCPY.match(stmt)
            if memcpy:
                field, idx = _parse_target(memcpy.group("dst").strip(), var, where)
                src_name = memcpy.group("src")
                if src_name not in locals_str:
                    fields[field] = {"opaque": stmt}
                else:
                    fields[field] = locals_str[src_name]
                assigned_top.add(field.split(".")[0])
                continue

            if "=" not in stmt:
                call = _CALL.match(stmt)
                if not call:
                    raise SpecDefaultsError(f"{where}: unrecognized statement {stmt!r}")
                touched = _fields_touched(call.group("args"), var)
                if not touched:
                    raise SpecDefaultsError(
                        f"{where}: helper call {stmt!r} touches no field of {var!r}"
                    )
                for path in touched:
                    fields[path] = {"opaque": stmt}
                    assigned_top.add(path.split(".")[0])
                continue

            parts = stmt.split("=")
            value = _classify(parts[-1], where)
            for target in parts[:-1]:
                field, idx = _parse_target(target.strip(), var, where)
                assigned_top.add(field.split(".")[0])
                if idx is None:
                    fields[field] = _scalar_value(value)
                else:
                    arrays.setdefault(field, {})[idx] = value

        for field, indexed in arrays.items():
            top = max(indexed)
            dense = [0] * (top + 1)
            for idx, value in indexed.items():
                dense[idx] = value
            fields[field] = dense

        if memset_zero:
            for member in struct_members.get(ctype, []):
                if member not in assigned_top and member not in fields:
                    fields[member] = {"unset_by_memset": True}

        defaults[ctype] = fields
    return defaults


def _walk_values(defaults: dict[str, dict]):
    for struct in defaults.values():
        for value in struct.values():
            if isinstance(value, list):
                yield from value
            else:
                yield value


def compute_stats(defaults: dict[str, dict]) -> dict:
    opaque_examples: list[str] = []
    counts = {"sentinel_nan": 0, "sentinel_neg1": 0, "auto_enum": 0, "unset_by_memset": 0,
              "opaque": 0}
    for value in _walk_values(defaults):
        if not isinstance(value, dict):
            continue
        if value.get("sentinel") == SENTINEL_NAN:
            counts["sentinel_nan"] += 1
        elif value.get("sentinel") == -1:
            counts["sentinel_neg1"] += 1
        elif value.get("auto"):
            counts["auto_enum"] += 1
        elif value.get("unset_by_memset"):
            counts["unset_by_memset"] += 1
        elif "opaque" in value:
            counts["opaque"] += 1
            if value["opaque"] not in opaque_examples:
                opaque_examples.append(value["opaque"])
    field_count = sum(len(s) for s in defaults.values())
    return {
        "structs": len(defaults),
        "fields": field_count,
        "opaque_examples": opaque_examples,
        **counts,
    }


def run_sanity_checks(defaults: dict[str, dict]) -> None:
    if len(defaults) < 15:
        raise SpecDefaultsError(f"only {len(defaults)} structs extracted; expected >= 15")

    geom = defaults.get("mjsGeom", {})
    if geom.get("type") != {"value": "mjGEOM_SPHERE"}:
        raise SpecDefaultsError(f"mjsGeom.type default is {geom.get('type')!r}, expected mjGEOM_SPHERE")

    body = defaults.get("mjsBody", {})
    for field in ("ipos", "fullinertia"):
        value = body.get(field)
        if not (isinstance(value, list) and {"sentinel": SENTINEL_NAN} in value):
            raise SpecDefaultsError(f"mjsBody.{field} should carry an mjNAN sentinel, got {value!r}")

    limited = defaults.get("mjsJoint", {}).get("limited")
    if not (isinstance(limited, dict) and limited.get("auto")):
        raise SpecDefaultsError(f"mjsJoint.limited should be an AUTO value, got {limited!r}")


def read_mujoco_version(mujoco_src: Path) -> str:
    header = mujoco_src / "include" / "mujoco" / "mujoco.h"
    text = header.read_text(encoding="utf-8")
    m = re.search(r"#define\s+mjVERSION_HEADER\s+(\d+)", text)
    if m is None:
        raise SpecDefaultsError(f"mjVERSION_HEADER not found in {header}")
    n = int(m.group(1))
    return f"{n // 1_000_000}.{(n // 1000) % 1000}.{n % 1000}"


def build_snapshot(mujoco_src: Path) -> tuple[dict, dict]:
    src = mujoco_src / "src" / "user" / "user_init.c"
    header = mujoco_src / "include" / "mujoco" / "mjspec.h"
    struct_members = parse_struct_members(header.read_text(encoding="utf-8"))
    defaults = extract_defaults(
        src.read_text(encoding="utf-8"), struct_members, source_label="src/user/user_init.c"
    )
    run_sanity_checks(defaults)
    snapshot = {
        "source": {
            "file": "src/user/user_init.c",
            "mujoco_version": read_mujoco_version(mujoco_src),
        },
        "defaults": defaults,
    }
    return snapshot, compute_stats(defaults)


def write_snapshot(snapshot: dict, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(snapshot, indent=2, ensure_ascii=False) + "\n"
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--mujoco-src",
        type=Path,
        required=True,
        help="Root of the vendored MuJoCo src checkout (contains src/ and include/).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=_repo_root() / "snapshots" / "spec_defaults.json",
        help="Destination JSON path (default: snapshots/spec_defaults.json).",
    )
    args = parser.parse_args(argv)

    try:
        snapshot, stats = build_snapshot(args.mujoco_src)
    except SpecDefaultsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    write_snapshot(snapshot, args.out)
    print(
        f"wrote {args.out} "
        f"({stats['structs']} structs, {stats['fields']} fields, "
        f"mujoco {snapshot['source']['mujoco_version']})"
    )
    print(
        f"  sentinels: {stats['sentinel_nan']} mjNAN, {stats['sentinel_neg1']} -1; "
        f"{stats['auto_enum']} AUTO enums; {stats['unset_by_memset']} memset-only; "
        f"{stats['opaque']} opaque"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
