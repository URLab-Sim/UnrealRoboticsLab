# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
clang.cindex-based scrape of MuJoCo headers (mjspec.h + mjmodel.h +
optionally the vendored mjspecmacro.h) into ``Scripts/introspect_snapshot.json``.

This is the canonical Phase 2d-3 output that replaces / supersedes the
regex-based mjspec_snapshot.json once consumers migrate. Until then
both snapshots coexist and ``generate_ue_components.py`` reads whichever
is present.

Captures:
- function signatures (every MJAPI-tagged declaration in the headers)
- enum decls with values + per-member doc comments
- struct decls with fields + per-field doc comments + array dims
- ``#define`` constants (after preprocessor evaluation when literal)

Usage:
    python Scripts/codegen/build_introspect_snapshot.py
        [--mujoco-include /path/to/mujoco/include]
        [--output Scripts/introspect_snapshot.json]
        [--libclang /path/to/libclang.dll]      (only if not auto-found)

libclang resolution: respects ``$LIBCLANG_LIBRARY_FILE`` env var; falls
back to clang.cindex's default search. Set ``--libclang`` to override.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from typing import List

_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))

# Use the local vendored ast_nodes copy.
sys.path.insert(0, os.path.join(_HERE, "_vendored"))
from ast_nodes import (  # noqa: E402
    CFunction,
    CFunctionParam,
    CEnum,
    CEnumMember,
    CStruct,
    CStructField,
    CDefine,
    IntrospectSnapshot,
)


def _header_hash(paths: List[str]) -> str:
    """SHA-256 of the concatenated header content. Lets callers skip
    re-running the snapshot when nothing upstream changed."""
    h = hashlib.sha256()
    for p in sorted(paths):
        if not os.path.exists(p):
            continue
        with open(p, "rb") as f:
            h.update(f.read())
    return h.hexdigest()


def _strip_doc(raw: str) -> str:
    """Collapse a libclang raw doc comment into one line, stripping
    leading ``//`` / ``/*`` decoration."""
    if not raw:
        return ""
    out_lines: List[str] = []
    for line in raw.splitlines():
        s = line.strip()
        for prefix in ("///", "//!", "//", "/**", "*/", "*", "/*"):
            if s.startswith(prefix):
                s = s[len(prefix):].strip()
                break
        if s.endswith("*/"):
            s = s[:-2].strip()
        if s:
            out_lines.append(s)
    return " ".join(out_lines)


def _walk(tu, mujoco_include: str, snapshot: IntrospectSnapshot) -> None:
    """Visit every cursor in the translation unit. ``mujoco_include`` is
    the include root; we only capture decls whose file lives under it
    (so libc / Windows SDK / TinyXML noise is filtered out)."""
    import clang.cindex as cx  # noqa: E402, local-only

    def _normalised_path(loc) -> str:
        if not loc or not loc.file:
            return ""
        return os.path.normcase(os.path.abspath(loc.file.name))

    norm_root = os.path.normcase(os.path.abspath(mujoco_include))

    for cursor in tu.cursor.get_children():
        fp = _normalised_path(cursor.location)
        if not fp.startswith(norm_root):
            continue

        kind = cursor.kind

        if kind == cx.CursorKind.FUNCTION_DECL:
            params: List[CFunctionParam] = []
            for arg in cursor.get_arguments():
                t = arg.type.spelling
                dim = None
                if "[" in t and "]" in t:
                    # e.g. "double [4]" -> dim=4
                    try:
                        dim_str = t[t.index("[") + 1:t.index("]")]
                        if dim_str.strip().isdigit():
                            dim = int(dim_str)
                    except ValueError:
                        pass
                params.append(CFunctionParam(
                    name=arg.spelling, c_type=t, array_dim=dim,
                ))
            fn = CFunction(
                name=cursor.spelling,
                return_type=cursor.result_type.spelling,
                params=params,
                doc=_strip_doc(cursor.raw_comment or ""),
                file=fp,
                line=cursor.location.line if cursor.location else 0,
            )
            snapshot.functions[fn.name] = fn

        elif kind == cx.CursorKind.ENUM_DECL:
            # Anonymous enums get spelling = "" — skip; we only want
            # named typedef enums. The C tag name is ``mjtJoint_`` but
            # callers reference the typedef ``mjtJoint``; normalise here.
            name = cursor.spelling or cursor.type.spelling
            if not name or name.startswith("(anonymous"):
                continue
            if name.endswith("_"):
                name = name[:-1]
            members: List[CEnumMember] = []
            for child in cursor.get_children():
                if child.kind == cx.CursorKind.ENUM_CONSTANT_DECL:
                    members.append(CEnumMember(
                        name=child.spelling,
                        value=child.enum_value,
                        doc=_strip_doc(child.raw_comment or ""),
                    ))
            underlying = cursor.enum_type.spelling if cursor.enum_type else None
            snapshot.enums[name] = CEnum(
                name=name, members=members,
                underlying_type=underlying,
                doc=_strip_doc(cursor.raw_comment or ""),
                file=fp,
                line=cursor.location.line if cursor.location else 0,
            )

        elif kind == cx.CursorKind.STRUCT_DECL:
            name = cursor.spelling or cursor.type.spelling
            if not name:
                continue
            # Strip trailing underscore from C tag names ("mjsBody_" -> "mjsBody").
            if name.endswith("_"):
                name = name[:-1]
            fields: List[CStructField] = []
            for child in cursor.get_children():
                if child.kind != cx.CursorKind.FIELD_DECL:
                    continue
                t = child.type.spelling
                dim = None
                is_ptr = "*" in t
                if "[" in t and "]" in t:
                    try:
                        dim_str = t[t.index("[") + 1:t.index("]")]
                        if dim_str.strip().isdigit():
                            dim = int(dim_str)
                    except ValueError:
                        pass
                fields.append(CStructField(
                    name=child.spelling,
                    c_type=t,
                    array_dim=dim,
                    is_pointer=is_ptr,
                    doc=_strip_doc(child.raw_comment or ""),
                ))
            snapshot.structs[name] = CStruct(
                name=name, fields=fields,
                doc=_strip_doc(cursor.raw_comment or ""),
                file=fp,
                line=cursor.location.line if cursor.location else 0,
            )

        elif kind == cx.CursorKind.MACRO_DEFINITION:
            name = cursor.spelling
            # Skip function-like macros; libclang doesn't expand them.
            # Skip header guards.
            if name.endswith("_H_") or name.startswith("__"):
                continue
            # Extract tokens between the cursor's extent to get the value.
            tokens = list(tu.get_tokens(extent=cursor.extent))
            if len(tokens) < 2:
                continue
            value = " ".join(t.spelling for t in tokens[1:])
            snapshot.defines[name] = CDefine(
                name=name, value=value,
                doc=_strip_doc(cursor.raw_comment or ""),
                file=fp,
                line=cursor.location.line if cursor.location else 0,
            )


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mujoco-include", default=os.path.join(
        _PLUGIN_ROOT, "third_party", "install", "MuJoCo", "include",
    ))
    ap.add_argument("--output", default=os.path.join(
        _PLUGIN_ROOT, "Scripts", "codegen", "snapshots", "introspect_snapshot.json",
    ))
    ap.add_argument("--libclang", default=None,
                    help="path to libclang.dll/.so/.dylib; overrides auto-detect")
    args = ap.parse_args(argv)

    try:
        import clang.cindex as cx
    except ImportError:
        print("ERROR: clang.cindex not installed. Run "
              "`uv run --with libclang python build_introspect_snapshot.py` "
              "or `pip install libclang`.", file=sys.stderr)
        return 1

    if args.libclang:
        cx.Config.set_library_file(args.libclang)
    elif os.environ.get("LIBCLANG_LIBRARY_FILE"):
        cx.Config.set_library_file(os.environ["LIBCLANG_LIBRARY_FILE"])

    mujoco_include = args.mujoco_include
    if not os.path.exists(mujoco_include):
        print(f"ERROR: --mujoco-include not found: {mujoco_include}", file=sys.stderr)
        return 2

    mjspec_h = os.path.join(mujoco_include, "mujoco", "mjspec.h")
    mjmodel_h = os.path.join(mujoco_include, "mujoco", "mjmodel.h")
    mjxmacro_h = os.path.join(mujoco_include, "mujoco", "mjxmacro.h")

    # Vendored mjspecmacro.h (only used if non-stub; sync_vendored.py
    # fills it from upstream).
    vendored_mjspecmacro = os.path.join(_HERE, "_vendored", "mjspecmacro.h")

    # Synthesize a small dispatch source that #includes the headers we
    # care about. Letting clang chew on a real .c lets it resolve types
    # without us threading -include flags.
    dispatch_c = "\n".join([
        "#include <mujoco/mjmodel.h>",
        "#include <mujoco/mjspec.h>",
        "#include <mujoco/mjxmacro.h>",
        "#include <mujoco/mujoco.h>",
    ])
    args_clang = [
        "-x", "c",
        "-std=c11",
        f"-I{mujoco_include}",
        "-DMJ_STATIC",     # avoid MJAPI dllimport/export linkage attrs
    ]

    try:
        idx = cx.Index.create()
    except cx.LibclangError as exc:
        print(f"ERROR: libclang failed to load: {exc}", file=sys.stderr)
        return 3

    tu = idx.parse(
        path="<urlab_dispatch>.c",
        args=args_clang,
        unsaved_files=[("<urlab_dispatch>.c", dispatch_c)],
        options=(cx.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD
                 | cx.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES),
    )
    fatal_diags = [d for d in tu.diagnostics if d.severity >= cx.Diagnostic.Error]
    if fatal_diags:
        print(f"libclang errors:", file=sys.stderr)
        for d in fatal_diags[:5]:
            print(f"  {d.spelling} ({d.location})", file=sys.stderr)
        return 4

    snapshot = IntrospectSnapshot(
        snapshot_version=1,
        header_hash=_header_hash([mjspec_h, mjmodel_h, mjxmacro_h,
                                  vendored_mjspecmacro]),
    )
    _walk(tu, mujoco_include, snapshot)

    out_dict = snapshot.to_dict()
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out_dict, f, indent=2)
    print(f"wrote {args.output}: "
          f"{len(snapshot.functions)} functions, "
          f"{len(snapshot.enums)} enums, "
          f"{len(snapshot.structs)} structs, "
          f"{len(snapshot.defines)} defines, "
          f"header_hash={snapshot.header_hash[:12]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
