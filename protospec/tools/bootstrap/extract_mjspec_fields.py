"""Extract MuJoCo's mjSpec attribute structs from the vendored header into a JSON snapshot.

The authoring-level model lives as a set of ``typedef struct mjsXxx_`` and ``typedef enum
mjtXxx_`` declarations in ``include/mujoco/mjspec.h``. This module parses every enum and
struct into a field inventory: per field its declared C type, array extent (numeric or a
symbolic macro like ``mjNREF`` resolved against ``mjmodel.h``), trailing comment, and a
classified ``kind`` (scalar / fixed_array / string / *_vec / nested_vec / struct /
element_ptr). The result feeds the ProtoSpec drift gate (plan DR-12), which diffs the
hand-curated IDL against this surface on every MuJoCo bump.

The header aliases ``mjString``/``mj*Vec`` to ``std::string``/``std::vector`` only under
``__cplusplus`` (mjspec.h:35-55); that alias table is captured verbatim as ``handle_types``.

Every non-blank line inside a struct or enum body must parse; an unrecognized type or an
unresolvable array extent fails loudly with the offending source line, never a silent skip.
"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# mjString/mj*Vec are pointers-to-std-type handles (mjspec.h:35-55). A field of one of these
# handle types classifies by the handle, not by its pointer-ness.
HANDLE_KIND = {
    "mjString": "string",
    "mjStringVec": "string_vec",
    "mjIntVec": "int_vec",
    "mjIntVecVec": "nested_vec",
    "mjFloatVec": "float_vec",
    "mjFloatVecVec": "nested_vec",
    "mjDoubleVec": "double_vec",
    "mjByteVec": "byte_vec",
}

# Engine structs from mjmodel.h embedded by value into mjSpec/mjsCompiler. They are kept
# symbolic (kind "struct") and never expanded here.
ENGINE_STRUCTS = frozenset({"mjOption", "mjVisual", "mjStatistic", "mjLROpt"})

# Bare C primitives that may appear as a scalar field type. Every MuJoCo typedef used as a
# scalar (mjtByte, mjtNum, mjtSize, and the mjt* enums) starts with "mjt" and is caught
# separately, so this set only needs the language primitives.
SCALAR_PRIMS = frozenset({
    "double", "int", "float", "char", "bool", "short", "long", "unsigned",
    "uint8_t", "int8_t", "uint16_t", "int16_t", "uint32_t", "int32_t",
    "uint64_t", "int64_t", "size_t",
})


class MjSpecParseError(Exception):
    """Raised when the vendored header format has drifted from what we can parse."""


@dataclass
class Field:
    name: str
    ctype: str
    pointer: bool
    array: str | None  # raw extent expression, e.g. "3", "mjNREF", "mjNPOLY+1"
    comment: str
    line: int
    kind: str = ""
    array_size: int | None = None

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "ctype": self.ctype,
            "pointer": self.pointer,
            "array": None if self.array is None
            else {"extent": self.array, "value": self.array_size},
            "comment": self.comment,
            "kind": self.kind,
        }


@dataclass
class Enumerator:
    name: str
    value: str | None
    comment: str

    def to_dict(self) -> dict:
        return {"name": self.name, "value": self.value, "comment": self.comment}


@dataclass
class Enum:
    name: str
    tag: str
    doc: str
    enumerators: list[Enumerator] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "tag": self.tag,
            "doc": self.doc,
            "enumerators": [e.to_dict() for e in self.enumerators],
        }


@dataclass
class Struct:
    name: str
    tag: str
    doc: str
    fields: list[Field] = field(default_factory=list)

    def embeds_engine(self) -> list[str]:
        seen: list[str] = []
        for f in self.fields:
            if f.ctype in ENGINE_STRUCTS and f.ctype not in seen:
                seen.append(f.ctype)
        return seen

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "tag": self.tag,
            "doc": self.doc,
            "embeds_engine": self.embeds_engine(),
            "fields": [f.to_dict() for f in self.fields],
        }


_ENUM_OPEN = re.compile(r"^typedef enum\s+(?P<tag>\w+)\s*\{(?:\s*//\s*(?P<doc>.*?))?\s*$")
_STRUCT_OPEN = re.compile(r"^typedef struct\s+(?P<tag>\w+)\s*\{(?:\s*//\s*(?P<doc>.*?))?\s*$")
_CLOSE = re.compile(r"^\}\s*(?P<name>\w+)\s*;")
_COMMENT_ONLY = re.compile(r"^//")
_USING = re.compile(r"^using\s+(?P<alias>\w+)\s*=\s*(?P<target>.+?);")

_ENUMERATOR = re.compile(
    r"^(?P<name>\w+)\s*(?:=\s*(?P<value>.+?))?\s*,?\s*(?://\s*(?P<comment>.*))?$"
)
_FIELD = re.compile(
    r"^(?P<type>\w+)\s*(?P<ptr>\*)?\s*(?P<name>\w+)"
    r"(?:\[(?P<arr>[^\]]*)\])?\s*;\s*(?://\s*(?P<comment>.*))?$"
)


def _eval_extent(expr: str, macros: dict[str, int], line: int) -> int:
    """Resolve an array-extent expression (int literals + macro names + - * arithmetic)."""
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as exc:
        raise MjSpecParseError(f"line {line}: cannot parse array extent '{expr}': {exc}")

    def ev(node: ast.AST) -> int:
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            return node.value
        if isinstance(node, ast.Name):
            if node.id not in macros:
                raise MjSpecParseError(
                    f"line {line}: unknown macro '{node.id}' in array extent '{expr}'"
                )
            return macros[node.id]
        if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult)):
            left, right = ev(node.left), ev(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            return left * right
        raise MjSpecParseError(f"line {line}: unsupported array extent '{expr}'")

    return ev(tree)


def parse_handle_types(lines: list[str]) -> dict[str, str]:
    """Capture the ``using mjX = std::Y;`` alias table (the __cplusplus branch)."""
    aliases: dict[str, str] = {}
    for raw in lines:
        m = _USING.match(raw.strip())
        if m:
            aliases[m.group("alias")] = m.group("target").strip()
    if not aliases:
        raise MjSpecParseError("no 'using mjX = ...;' handle-type aliases found")
    missing = set(HANDLE_KIND) - set(aliases)
    if missing:
        raise MjSpecParseError(f"handle-type aliases missing: {sorted(missing)}")
    return aliases


def parse_enum(lines: list[str], start: int) -> tuple[Enum, int]:
    m = _ENUM_OPEN.match(lines[start].strip())
    tag = m.group("tag")
    enum = Enum(name="", tag=tag, doc=(m.group("doc") or "").strip())
    i = start + 1
    while i < len(lines):
        stripped = lines[i].strip()
        if not stripped or _COMMENT_ONLY.match(stripped):
            i += 1
            continue
        close = _CLOSE.match(stripped)
        if close:
            enum.name = close.group("name")
            return enum, i + 1
        em = _ENUMERATOR.match(stripped)
        if not em:
            raise MjSpecParseError(
                f"line {i + 1}: unparseable enumerator in {tag}: {stripped!r}"
            )
        value = em.group("value")
        enum.enumerators.append(
            Enumerator(
                name=em.group("name"),
                value=value.strip() if value else None,
                comment=(em.group("comment") or "").strip(),
            )
        )
        i += 1
    raise MjSpecParseError(f"line {start + 1}: unterminated enum {tag}")


def parse_struct(lines: list[str], start: int) -> tuple[Struct, int]:
    m = _STRUCT_OPEN.match(lines[start].strip())
    tag = m.group("tag")
    struct = Struct(name="", tag=tag, doc=(m.group("doc") or "").strip())
    i = start + 1
    while i < len(lines):
        stripped = lines[i].strip()
        if not stripped or _COMMENT_ONLY.match(stripped):
            i += 1
            continue
        close = _CLOSE.match(stripped)
        if close:
            struct.name = close.group("name")
            return struct, i + 1
        fm = _FIELD.match(stripped)
        if not fm:
            raise MjSpecParseError(
                f"line {i + 1}: unparseable field in {tag}: {stripped!r}"
            )
        struct.fields.append(
            Field(
                name=fm.group("name"),
                ctype=fm.group("type"),
                pointer=fm.group("ptr") is not None,
                array=fm.group("arr"),
                comment=(fm.group("comment") or "").strip(),
                line=i + 1,
            )
        )
        i += 1
    raise MjSpecParseError(f"line {start + 1}: unterminated struct {tag}")


def parse_header(text: str) -> tuple[list[Enum], list[Struct], dict[str, str]]:
    lines = text.splitlines()
    handle_types = parse_handle_types(lines)
    enums: list[Enum] = []
    structs: list[Struct] = []
    i = 0
    while i < len(lines):
        stripped = lines[i].strip()
        if _ENUM_OPEN.match(stripped):
            enum, i = parse_enum(lines, i)
            enums.append(enum)
        elif _STRUCT_OPEN.match(stripped):
            struct, i = parse_struct(lines, i)
            structs.append(struct)
        else:
            i += 1
    return enums, structs, handle_types


def classify_and_resolve(structs: list[Struct], macros: dict[str, int]) -> dict[str, int]:
    """Assign every field a kind, resolve array extents, and return the macros actually used."""
    known_structs = {s.name for s in structs} | ENGINE_STRUCTS
    used_macros: dict[str, int] = {}

    for struct in structs:
        for f in struct.fields:
            if f.array is not None:
                f.array_size = _eval_extent(f.array, macros, f.line)
                for node in ast.walk(ast.parse(f.array, mode="eval")):
                    if isinstance(node, ast.Name):
                        used_macros[node.id] = macros[node.id]
                f.kind = "fixed_array"
            elif f.pointer:
                if f.ctype in HANDLE_KIND:
                    f.kind = HANDLE_KIND[f.ctype]
                elif f.ctype.startswith("mjs"):
                    f.kind = "element_ptr"
                else:
                    raise MjSpecParseError(
                        f"line {f.line}: unrecognized pointer type '{f.ctype}*' "
                        f"for field '{f.name}'"
                    )
            elif f.ctype in known_structs or f.ctype.startswith("mjs"):
                f.kind = "struct"
            elif f.ctype.startswith("mjt") or f.ctype in SCALAR_PRIMS:
                f.kind = "scalar"
            else:
                raise MjSpecParseError(
                    f"line {f.line}: unrecognized type '{f.ctype}' for field '{f.name}'"
                )
    return used_macros


def collect_macros(*headers: Path) -> dict[str, int]:
    """Collect every ``#define NAME <int>`` from the given headers (later files win ties)."""
    macros: dict[str, int] = {}
    pattern = re.compile(r"^#define\s+(?P<name>\w+)\s+(?P<value>-?\d+)\b")
    for header in headers:
        for raw in header.read_text(encoding="utf-8").splitlines():
            m = pattern.match(raw)
            if m:
                macros[m.group("name")] = int(m.group("value"))
    return macros


def read_mujoco_version(mujoco_src: Path) -> str:
    header = mujoco_src / "include" / "mujoco" / "mujoco.h"
    text = header.read_text(encoding="utf-8")
    m = re.search(r"#define\s+mjVERSION_HEADER\s+(\d+)", text)
    if m is None:
        raise MjSpecParseError(f"mjVERSION_HEADER not found in {header}")
    n = int(m.group(1))
    return f"{n // 1_000_000}.{(n // 1000) % 1000}.{n % 1000}"


def run_sanity_checks(enums: list[Enum], structs: list[Struct]) -> None:
    if len(structs) < 25:
        raise MjSpecParseError(f"only {len(structs)} structs parsed; expected >= 25")

    by_name = {s.name: s for s in structs}
    body = by_name.get("mjsBody")
    if body is None:
        raise MjSpecParseError("mjsBody struct not found")
    body_fields = {f.name for f in body.fields}
    for required in ("pos", "quat", "alt", "fullinertia"):
        if required not in body_fields:
            raise MjSpecParseError(f"mjsBody is missing expected field '{required}'")

    wrap = by_name.get("mjsWrap")
    if wrap is None:
        raise MjSpecParseError("mjsWrap struct not found")
    if len(wrap.fields) > 4:
        raise MjSpecParseError(
            f"mjsWrap has {len(wrap.fields)} fields; expected a stub (few fields)"
        )

    for struct in structs:
        for f in struct.fields:
            if not f.kind:
                raise MjSpecParseError(
                    f"line {f.line}: field '{struct.name}.{f.name}' was not classified"
                )
    for enum in enums:
        if not enum.enumerators:
            raise MjSpecParseError(f"enum {enum.name!r} has no enumerators")


def build_snapshot(mujoco_src: Path) -> dict:
    include_dir = mujoco_src / "include" / "mujoco"
    header = include_dir / "mjspec.h"
    text = header.read_text(encoding="utf-8")

    enums, structs, handle_types = parse_header(text)
    macros = collect_macros(include_dir / "mjmodel.h", header)
    used_macros = classify_and_resolve(structs, macros)
    run_sanity_checks(enums, structs)

    return {
        "source": {
            "file": "include/mujoco/mjspec.h",
            "mujoco_version": read_mujoco_version(mujoco_src),
        },
        "handle_types": handle_types,
        "macros": {name: used_macros[name] for name in sorted(used_macros)},
        "enums": [e.to_dict() for e in enums],
        "structs": [s.to_dict() for s in structs],
    }


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
        default=_repo_root() / "snapshots" / "mjspec_fields.json",
        help="Destination JSON path (default: snapshots/mjspec_fields.json).",
    )
    args = parser.parse_args(argv)

    try:
        snapshot = build_snapshot(args.mujoco_src)
    except MjSpecParseError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    write_snapshot(snapshot, args.out)
    field_count = sum(len(s["fields"]) for s in snapshot["structs"])
    print(
        f"wrote {args.out} "
        f"({len(snapshot['enums'])} enums, {len(snapshot['structs'])} structs, "
        f"{field_count} fields, mujoco {snapshot['source']['mujoco_version']})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
