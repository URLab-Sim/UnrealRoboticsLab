# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the codegen injection helpers: brace-balance gate +
multi-tag inject + string/comment-aware brace stripping.

The brace-balance check exists because a stray ``}`` lost inside a
long composed block silently relocates the compile error to a place
unrelated to the codegen site. We want the diagnostic to point at the
exact tag instead."""

from __future__ import annotations

import os

import generate_ue_components as gen
import _codegen_inject as _inject
gen._check_brace_balance = _inject._check_brace_balance


def _write(tmp_path, name: str, body: str) -> str:
    p = tmp_path / name
    p.write_text(body, encoding="utf-8")
    return str(p)


# ---------- brace-balance --------------------------------------------------

def test_brace_balance_flags_unbalanced_block():
    body = "if (x) { do_thing(); "  # missing close
    gen._check_brace_balance(body, "FOO", "unit_test")
    assert any("unbalanced braces" in d.message and "CODEGEN_FOO" in d.message
               for d in gen._DIAGS_BUFFER.pending)


def test_brace_balance_quiet_on_balanced_block():
    body = "if (x) { do_thing(); }"
    gen._check_brace_balance(body, "FOO", "unit_test")
    assert len(gen._DIAGS_BUFFER.pending) == 0


def test_brace_balance_ignores_braces_in_strings_and_comments():
    """``"{"`` inside a TEXT() literal must not count toward the open
    tally; same for braces in // and /* */ comments."""
    body = (
        'if (x)\n'
        '{\n'
        '    Log(TEXT("brace { in string"));  // and { in line comment\n'
        '    /* and { in block comment */\n'
        '}\n'
    )
    gen._check_brace_balance(body, "FOO", "unit_test")
    assert len(gen._DIAGS_BUFFER.pending) == 0


def test_inject_between_tags_with_balance_flag_fires_diag():
    """``balance_source`` kwarg routes through the brace-balance check."""
    file_text = "before\n// --- CODEGEN_FOO_START ---\nold\n// --- CODEGEN_FOO_END ---\nafter\n"
    new_inner = "if (x) {"  # unbalanced
    _, ok = gen.inject_between_tags(file_text, "FOO", new_inner,
                                    balance_source="unit_test")
    assert ok is True
    assert any("unbalanced braces" in d.message for d in gen._DIAGS_BUFFER.pending)


def test_inject_between_tags_default_skips_balance_check():
    """Without ``balance_source``, the helper does not run the gate —
    callers writing non-C++ blocks (markdown, JSON) aren't forced to
    opt out."""
    file_text = "// --- CODEGEN_FOO_START ---\nold\n// --- CODEGEN_FOO_END ---\n"
    gen.inject_between_tags(file_text, "FOO", "{ stray")
    assert len(gen._DIAGS_BUFFER.pending) == 0


# ---------- _inject_tags_into_cpp -----------------------------------------

def test_inject_tags_into_cpp_handles_multiple_tags(tmp_path):
    body = (
        "// --- CODEGEN_FOO_START ---\n"
        "old foo\n"
        "// --- CODEGEN_FOO_END ---\n"
        "between\n"
        "// --- CODEGEN_BAR_START ---\n"
        "old bar\n"
        "// --- CODEGEN_BAR_END ---\n"
    )
    cpp_path = _write(tmp_path, "host.cpp", body)
    writes: list = []
    gen._inject_tags_into_cpp(
        cpp_path,
        [("FOO", "new foo body"), ("BAR", "new bar body")],
        writes,
        diag_source="unit_test",
    )
    assert len(writes) == 1
    out = writes[0].content
    assert "new foo body" in out
    assert "new bar body" in out
    assert "old foo" not in out
    assert "old bar" not in out


def test_inject_tags_into_cpp_diagnoses_missing_file(tmp_path):
    missing = str(tmp_path / "nope.cpp")
    writes: list = []
    gen._inject_tags_into_cpp(
        missing, [("FOO", "x")], writes, diag_source="unit_test",
    )
    assert writes == []
    assert any("does not exist" in d.message for d in gen._DIAGS_BUFFER.pending)


def test_inject_tags_into_cpp_diagnoses_missing_marker(tmp_path):
    cpp_path = _write(tmp_path, "host.cpp",
                      "// --- CODEGEN_FOO_START ---\nx\n// --- CODEGEN_FOO_END ---\n")
    writes: list = []
    gen._inject_tags_into_cpp(
        cpp_path,
        [("FOO", "new foo"), ("MISSING_TAG", "new missing")],
        writes,
        diag_source="unit_test",
    )
    # The FOO write still landed; MISSING_TAG fires a diagnostic.
    assert len(writes) == 1
    assert any("CODEGEN_MISSING_TAG" in d.message for d in gen._DIAGS_BUFFER.pending)


def test_inject_tags_into_cpp_skips_write_when_unchanged(tmp_path):
    """Idempotency: re-injecting the same body must not append a
    FileWrite. The ``--check`` gate relies on this. The END marker's
    indent mirrors the START marker's, so a START at column 0 lands
    END at column 0 too."""
    body = (
        "// --- CODEGEN_FOO_START ---\n"
        "same body\n"
        "// --- CODEGEN_FOO_END ---\n"
    )
    cpp_path = _write(tmp_path, "host.cpp", body)
    writes: list = []
    gen._inject_tags_into_cpp(
        cpp_path, [("FOO", "same body")], writes, diag_source="unit_test",
    )
    assert writes == []


def test_inject_between_tags_normalises_end_indent_to_match_start(tmp_path):
    """A hand-prepped source where START is at 8 spaces but END is
    at 4 spaces (or any mismatch) gets its END normalised to match
    START."""
    text = (
        "        // --- CODEGEN_FOO_START ---\n"
        "        body content\n"
        "    // --- CODEGEN_FOO_END ---\n"
    )
    new_text, ok = gen.inject_between_tags(text, "FOO", "new body")
    assert ok is True
    # Both marker lines now sit at the START's 8-space indent — assert
    # the END's leading whitespace exactly so we don't accidentally
    # match a substring of a deeper indent.
    import re
    match = re.search(r"^(\s*)// --- CODEGEN_FOO_END ---$", new_text,
                      re.MULTILINE)
    assert match is not None
    assert match.group(1) == "        ", repr(match.group(1))
