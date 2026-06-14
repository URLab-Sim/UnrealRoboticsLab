# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
clang.cindex-based scrape of MuJoCo headers (mjspec.h + mjmodel.h) into
``Scripts/codegen/snapshots/introspect_snapshot.json``. The codegen
consumes this snapshot as its single source of truth for the MuJoCo C
API surface.

Captures:
- function signatures (every MJAPI-tagged declaration in the headers)
- enum decls with values + per-member doc comments
- struct decls with fields + per-field doc comments + array dims
- ``#define`` constants (after preprocessor evaluation when literal)

Usage:
    python Scripts/codegen/build_introspect_snapshot.py
        [--mujoco-include /path/to/mujoco/include]
        [--output Scripts/codegen/snapshots/introspect_snapshot.json]
        [--libclang /path/to/libclang.dll]      (only if not auto-found)

libclang resolution: respects ``$LIBCLANG_LIBRARY_FILE`` env var; falls
back to clang.cindex's default search. Set ``--libclang`` to override.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from typing import Dict, List

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

    plugin_root_norm = os.path.normcase(os.path.abspath(_PLUGIN_ROOT))

    def _normalised_path(loc) -> str:
        """Return the location's file as a forward-slash path relative to
        the plugin root, so the snapshot is byte-identical across
        developers (no absolute ``c:\\users\\...`` strings)."""
        if not loc or not loc.file:
            return ""
        abs_path = os.path.normcase(os.path.abspath(loc.file.name))
        if abs_path.startswith(plugin_root_norm):
            rel = os.path.relpath(abs_path, plugin_root_norm)
            return rel.replace("\\", "/")
        return abs_path.replace("\\", "/")

    # Filter root mirrors _normalised_path: relative-to-plugin-root with
    # forward slashes.
    mujoco_abs = os.path.normcase(os.path.abspath(mujoco_include))
    if mujoco_abs.startswith(plugin_root_norm):
        norm_root = os.path.relpath(mujoco_abs, plugin_root_norm).replace("\\", "/")
    else:
        norm_root = mujoco_abs.replace("\\", "/")

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


# ---------------------------------------------------------------------------
# URLab hand-enum scrape
#
# UE hand-rolled ``enum class EMj* : uint8 { ... }`` declarations live in
# C++ headers that #include CoreMinimal.h + lots of UE machinery libclang
# can't parse standalone. Extract each enum block via regex (the
# delimiters are stable), strip the UE-macro surface (UMETA/UENUM/UMETA),
# and hand the cleaned single-enum source to libclang for AST-level
# member extraction. That gives us libclang's accuracy on member values
# without the UE-include hell, and is byte-comparable to the regex
# scraper that used to live in generate_ue_components.py.
# ---------------------------------------------------------------------------

_HAND_ENUM_BLOCK_RE = re.compile(
    r"(enum\s+class\s+EMj\w+\s*:\s*uint8\s*\{[^}]*\}\s*;)",
    re.DOTALL,
)
_HAND_ENUM_NAME_RE = re.compile(r"enum\s+class\s+(EMj\w+)")


_UE_FN_MACRO_RE = re.compile(r"\b(UMETA|UENUM)\s*\(")


def _strip_balanced_call(text: str, start: int) -> int:
    """Return the index just past the matching ``)`` for a function-call
    opener at ``start``. Handles nested parens — naive ``[^)]*`` would
    bail on ``UMETA(DisplayName = "Track (Centre of Mass)")``."""
    depth = 1
    i = start + 1
    in_str = None
    while i < len(text) and depth > 0:
        ch = text[i]
        if in_str:
            if ch == "\\" and i + 1 < len(text):
                i += 2
                continue
            if ch == in_str:
                in_str = None
        elif ch in ('"', "'"):
            in_str = ch
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        i += 1
    return i


def _strip_ue_macros_from_enum_block(text: str) -> str:
    """Erase ``UMETA(...)`` and ``UENUM(...)`` function-like macros so a
    plain libclang pass can compile the enum block. Each macro turns
    into the empty string; the surrounding member identifiers + comma
    separators stay intact. Handles nested parens inside the macro
    argument — ``UMETA(DisplayName = "Track (Centre of Mass)")`` etc."""
    out: List[str] = []
    i = 0
    while i < len(text):
        m = _UE_FN_MACRO_RE.search(text, i)
        if not m:
            out.append(text[i:])
            break
        out.append(text[i:m.start()])
        i = _strip_balanced_call(text, m.end() - 1)
    stripped = "".join(out)
    stripped = re.sub(r"//[^\n]*", "", stripped)
    stripped = re.sub(r"/\*.*?\*/", "", stripped, flags=re.DOTALL)
    return stripped


def _scrape_hand_enums_via_libclang(
    urlab_source_root: str,
) -> Dict[str, CEnum]:
    """Walk every .h under ``urlab_source_root`` looking for ``enum class
    EMj* : uint8`` blocks; hand each cleaned block to libclang for
    member + value extraction.

    Returns ``{enum_name: CEnum}``. The plugin_root-relative path is
    stamped on each entry so the snapshot stays byte-identical across
    developer machines (no absolute ``c:\\users\\...`` strings)."""
    import clang.cindex as cx  # noqa: E402, local-only

    plugin_root_norm = os.path.normcase(os.path.abspath(_PLUGIN_ROOT))
    out: Dict[str, CEnum] = {}
    idx = cx.Index.create()

    for root, _dirs, files in os.walk(urlab_source_root):
        for name in files:
            if not name.endswith(".h"):
                continue
            path = os.path.join(root, name)
            try:
                with open(path, "r", encoding="utf-8") as f:
                    text = f.read()
            except OSError:
                continue
            rel = os.path.relpath(
                os.path.normcase(os.path.abspath(path)),
                plugin_root_norm,
            ).replace("\\", "/")
            for block_match in _HAND_ENUM_BLOCK_RE.finditer(text):
                block = block_match.group(1)
                name_m = _HAND_ENUM_NAME_RE.search(block)
                if not name_m:
                    continue
                enum_name = name_m.group(1)
                cleaned = _strip_ue_macros_from_enum_block(block)
                # Compute 1-based source line of the enum's opening
                # ``enum`` keyword (the block_match is anchored on the
                # whole declaration, so its start offset is also the
                # enum's start).
                start_line = text[:block_match.start()].count("\n") + 1
                # UE typedefs uint8 to unsigned char; provide our own
                # typedef so libclang doesn't need any system headers
                # (which on Windows pull in MSVC STL that triggers a
                # "Unexpected compiler version" static_assert).
                tu = idx.parse(
                    path="<urlab_enum>.cpp",
                    args=["-x", "c++", "-std=c++17", "-nostdinc",
                          "-nostdinc++", "-ferror-limit=0"],
                    unsaved_files=[(
                        "<urlab_enum>.cpp",
                        "typedef unsigned char uint8;\n" + cleaned + "\n",
                    )],
                    options=cx.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES,
                )
                # Don't bail on diagnostics — even with errors libclang
                # produces usable ENUM_DECL cursors. The downstream
                # match-by-name + member iteration covers correctness.
                for cursor in tu.cursor.walk_preorder():
                    if cursor.kind != cx.CursorKind.ENUM_DECL:
                        continue
                    if cursor.spelling != enum_name:
                        continue
                    members: List[CEnumMember] = []
                    for child in cursor.get_children():
                        if child.kind == cx.CursorKind.ENUM_CONSTANT_DECL:
                            members.append(CEnumMember(
                                name=child.spelling,
                                value=child.enum_value,
                                doc="",
                            ))
                    if not members:
                        continue
                    out[enum_name] = CEnum(
                        name=enum_name,
                        members=members,
                        underlying_type="uint8",
                        doc="",
                        file=rel,
                        line=start_line,
                    )
                    break
    return out


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
    ap.add_argument("--urlab-source-root", default=os.path.join(
        _PLUGIN_ROOT, "Source", "URLab", "Public",
    ), help="root for the EMj* hand-enum libclang scrape; skipped if empty")
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

    # mjspecmacro.h ships in the installed MuJoCo headers (post-3.9.0); hashed
    # alongside the others so the snapshot invalidates when it changes.
    mjspecmacro_h = os.path.join(mujoco_include, "mujoco", "mjspecmacro.h")

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
                                  mjspecmacro_h]),
    )
    _walk(tu, mujoco_include, snapshot)

    if args.urlab_source_root and os.path.isdir(args.urlab_source_root):
        try:
            snapshot.hand_enums = _scrape_hand_enums_via_libclang(
                args.urlab_source_root,
            )
        except cx.LibclangError as exc:
            print(f"warning: hand-enum libclang scrape skipped: {exc}",
                  file=sys.stderr)

    out_dict = snapshot.to_dict()
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out_dict, f, indent=2)
    print(f"wrote {args.output}: "
          f"{len(snapshot.functions)} functions, "
          f"{len(snapshot.enums)} enums, "
          f"{len(snapshot.structs)} structs, "
          f"{len(snapshot.defines)} defines, "
          f"{len(snapshot.hand_enums)} hand_enums, "
          f"header_hash={snapshot.header_hash[:12]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
