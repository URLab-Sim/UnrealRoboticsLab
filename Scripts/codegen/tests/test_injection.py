# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Boundary-tag injection tests."""

from __future__ import annotations

from generate_ue_components import inject_between_tags


def test_replace_between_existing_tags():
    src = """class UFoo {
public:
    // --- CODEGEN_PROPERTIES_START ---
    int OldStuff;
    // --- CODEGEN_PROPERTIES_END ---
};
"""
    out, found = inject_between_tags(src, "PROPERTIES", "    int NewStuff;\n")
    assert found is True
    assert "OldStuff" not in out
    assert "NewStuff" in out
    # Tag lines preserved.
    assert "// --- CODEGEN_PROPERTIES_START ---" in out
    assert "// --- CODEGEN_PROPERTIES_END ---" in out


def test_missing_tags_returns_unchanged():
    src = "class UFoo { int X; };"
    out, found = inject_between_tags(src, "PROPERTIES", "int Y;")
    assert found is False
    assert out == src


def test_idempotent_replace():
    src = """// --- CODEGEN_FOO_START ---
old body
// --- CODEGEN_FOO_END ---"""
    out1, _ = inject_between_tags(src, "FOO", "new body\n")
    out2, _ = inject_between_tags(out1, "FOO", "new body\n")
    # second injection of the same body produces byte-identical output.
    assert out1 == out2


def test_preserves_text_outside_tags():
    prefix = "// custom comment that must survive\n"
    suffix = "\n// trailing custom code\nvoid Helper() {}\n"
    src = prefix + "// --- CODEGEN_X_START ---\nold\n// --- CODEGEN_X_END ---" + suffix
    out, _ = inject_between_tags(src, "X", "new\n")
    assert out.startswith(prefix)
    assert out.endswith(suffix)


def test_dotall_does_not_swallow_subsequent_tags():
    # Two separate codegen blocks must be replaced independently.
    src = """// --- CODEGEN_A_START ---
A old
// --- CODEGEN_A_END ---
some middle code
// --- CODEGEN_B_START ---
B old
// --- CODEGEN_B_END ---"""
    out, _ = inject_between_tags(src, "A", "A new\n")
    out, _ = inject_between_tags(out, "B", "B new\n")
    assert "A new" in out and "B new" in out
    assert "A old" not in out and "B old" not in out
    # Middle code is still there.
    assert "some middle code" in out
