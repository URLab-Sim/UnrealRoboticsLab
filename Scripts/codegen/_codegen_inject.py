# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""URLab codegen — text-injection machinery (Phase 5 module split,
step 3).

Pure text operations: locate ``// --- CODEGEN_<tag>_START/END ---``
marker pairs in a source file, splice a body between them, and emit
diagnostics when the body has unbalanced braces or the markers are
missing. No knowledge of rules, schema, or mjspec — just file text in,
file text out.

The brace-aware helpers (``_find_matching_brace``,
``_strip_cpp_braces_in_strings``, ``_check_brace_balance``) also live
here because the banner-overwrite audit + brace-balance gate both
consume them — moving them keeps every text-aware operation in one
module."""

from __future__ import annotations

import os
import re
from typing import List, Optional, Sequence, Tuple

from _codegen_core import _diag_add, FileWrite


def _make_tag_pair(tag: str) -> Tuple[str, str]:
    return (f"// --- CODEGEN_{tag}_START ---", f"// --- CODEGEN_{tag}_END ---")


def _strip_cpp_braces_in_strings(text: str) -> str:
    """Remove characters that look like braces but live inside a
    string/char literal or comment. Used by ``_check_brace_balance``
    so a diagnostic message containing ``"{"`` doesn't trip the
    balance gate."""
    text = re.sub(r"//[^\n]*", "", text)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    text = re.sub(r"'(?:\\.|[^'\\])*'", "''", text)
    return text


def _find_matching_brace(text: str, open_idx: int) -> Optional[int]:
    """Return the index of the ``}`` matching the ``{`` at
    ``open_idx``. Tracks brace depth across string/char literals and
    line/block comments so a nested ``{`` inside a TEXT(...) literal
    doesn't shift the balance. Returns None on EOF / unmatched."""
    if open_idx < 0 or open_idx >= len(text) or text[open_idx] != "{":
        return None
    depth = 1
    i = open_idx + 1
    in_str: Optional[str] = None
    while i < len(text) and depth > 0:
        ch = text[i]
        if in_str is not None:
            if ch == "\\" and i + 1 < len(text):
                i += 2
                continue
            if ch == in_str:
                in_str = None
            i += 1
            continue
        if ch == "/" and i + 1 < len(text) and text[i + 1] == "/":
            nl = text.find("\n", i)
            i = nl if nl != -1 else len(text)
            continue
        if ch == "/" and i + 1 < len(text) and text[i + 1] == "*":
            end = text.find("*/", i + 2)
            i = end + 2 if end != -1 else len(text)
            continue
        if ch in ('"', "'"):
            in_str = ch
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return None


def _check_brace_balance(body: str, tag: str, source: str) -> None:
    """Emit a diagnostic when an injected block has an obvious brace
    imbalance."""
    stripped = _strip_cpp_braces_in_strings(body)
    opens = stripped.count("{")
    closes = stripped.count("}")
    if opens != closes:
        _diag_add(
            f"[diagnostic] {source}: injected block for tag "
            f"CODEGEN_{tag} has unbalanced braces (open={opens}, "
            f"close={closes}). The generated .cpp will not compile.",
            source=source,
        )


def inject_between_tags(file_text: str, tag: str, new_inner: str,
                        *, balance_source: Optional[str] = None) -> Tuple[str, bool]:
    """Replace the content between ``// --- CODEGEN_<tag>_START ---``
    and ``// --- CODEGEN_<tag>_END ---`` with ``new_inner``. Returns
    (new_text, was_found).

    The END marker's leading whitespace is rewritten to match the
    START marker's indent. Phase 4.4 fix: when ``new_inner`` is empty
    or whitespace-only, the helper collapses the block so START + END
    sit on adjacent lines with no stray blank line between them.

    When ``balance_source`` is set, emit a diagnostic if the injected
    block has unbalanced braces."""
    start, end = _make_tag_pair(tag)
    pattern = re.compile(
        r"([ \t]*)(" + re.escape(start) + r")(.*?)([ \t]*)(" + re.escape(end) + r")",
        re.DOTALL,
    )
    matched = [False]

    def repl(m: re.Match) -> str:
        matched[0] = True
        start_indent = m.group(1)
        trimmed = new_inner.rstrip()
        if trimmed:
            body = "\n" + trimmed + "\n" + start_indent
        else:
            # Empty body: collapse to one newline so START + END sit
            # adjacent (no stray blank line between them).
            body = "\n" + start_indent
        return m.group(1) + m.group(2) + body + m.group(5)

    new_text = pattern.sub(repl, file_text)
    if matched[0] and balance_source:
        _check_brace_balance(new_inner, tag, balance_source)
    return new_text, matched[0]


def _inject_tags_into_cpp(
    cpp_path: str,
    tagged_bodies: Sequence[Tuple[str, str]],
    writes: List[FileWrite],
    *,
    diag_source: str,
) -> None:
    """Open ``cpp_path`` once, inject every (tag, body) pair, surface
    a diagnostic for each missing marker pair, append a single
    FileWrite if any tag actually changed the text."""
    if not os.path.exists(cpp_path):
        _diag_add(
            f"[diagnostic] {diag_source}: host .cpp '{cpp_path}' does "
            f"not exist; no blocks injected.",
            source=diag_source,
        )
        return
    with open(cpp_path, "r", encoding="utf-8") as f:
        text = f.read()
    new_text = text
    for tag, body in tagged_bodies:
        new_text, ok = inject_between_tags(
            new_text, tag, body, balance_source=diag_source,
        )
        if not ok:
            _diag_add(
                f"[diagnostic] {diag_source}: '{cpp_path}' is missing "
                f"the CODEGEN_{tag}_START/END marker pair.",
                source=diag_source,
            )
    if new_text != text:
        writes.append(FileWrite(path=cpp_path, content=new_text))
