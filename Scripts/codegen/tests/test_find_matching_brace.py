# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks ``_find_matching_brace`` — the brace-aware extractor that
backs the banner-overwrite audit. The previous ``\\{([^}]*)\\}`` regex
truncated at the first inner ``}``, silently underreporting extra ctor
content when a real ctor body contained a range-for loop or nested
if. The extractor must track string literals and comments so a ``{``
inside ``TEXT("...")`` does not shift the depth count."""

from __future__ import annotations

import generate_ue_components as gen


def test_finds_simple_matching_brace():
    text = "x { hello } y"
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    assert close_idx is not None
    assert text[close_idx] == "}"
    assert text[open_idx + 1:close_idx] == " hello "


def test_handles_nested_braces():
    text = "ctor() { if (x) { do(); } else { other(); } }"
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    # close_idx must be the FINAL '}', not the first inner one.
    assert close_idx == len(text) - 1


def test_ignores_braces_in_string_literal():
    text = 'fn() { Print(TEXT("{nested}")); }'
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    # The string contains '{' and '}' but they must NOT change depth;
    # the matcher should land on the final outer '}'.
    assert close_idx == len(text) - 1


def test_ignores_braces_in_char_literal():
    text = "fn() { char c = '{'; print(c); }"
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    assert close_idx == len(text) - 1


def test_ignores_braces_in_line_comment():
    text = "fn() { // tail {\n    real(); }"
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    assert close_idx == len(text) - 1


def test_ignores_braces_in_block_comment():
    text = "fn() { /* tail { in comment */ real(); }"
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    assert close_idx == len(text) - 1


def test_returns_none_for_unmatched_open():
    """If the input is malformed (open without a matching close), the
    helper returns None — caller decides whether that's a diag or a
    silent skip."""
    assert gen._find_matching_brace("fn() { unclosed", 5) is None


def test_returns_none_when_index_is_not_open_brace():
    assert gen._find_matching_brace("x }", 1) is None
    assert gen._find_matching_brace("x", 5) is None  # off-end


def test_escape_sequences_in_string_dont_terminate_early():
    """A ``\\"`` inside a string literal must not close the string
    early — the matcher would otherwise re-enter brace-counting mode
    inside the literal and mis-count."""
    text = r'fn() { print("a\"b{c"); real(); }'
    open_idx = text.index("{")
    close_idx = gen._find_matching_brace(text, open_idx)
    assert close_idx == len(text) - 1
