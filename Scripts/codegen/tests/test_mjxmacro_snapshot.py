# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the mjxmacro snapshot parser.

The XNV-accepting regex was added after a silent-drop bug — every entry
listed AFTER an XNV line in the same block (e.g. ``geom_user``, ``geom_rgba``
after ``XNV(mjtNum, geom_fluid, ...)`` in MJMODEL_POINTERS_GEOM) was
discarded because the body-collector treated XNV as a non-entry sentinel
and stopped the loop. These tests pin the regex behaviour so adding a
new tag is a deliberate change with a test failure to acknowledge.
"""

from __future__ import annotations

import textwrap

from build_mjxmacro_snapshot import (
    _BODY_ENTRY_RE,
    _X_ENTRY_RE,
    _X_TAG_RE,
    _parse_block_body,
    _parse_pointer_entry,
)


# ---------------------------------------------------------------------------
# Regex behaviour
# ---------------------------------------------------------------------------

def test_x_tag_re_accepts_known_tags():
    """X, XVEC, XNV are the three macro tags MuJoCo uses today."""
    import re
    rx = re.compile(rf"^{_X_TAG_RE}$")
    assert rx.match("X")
    assert rx.match("XVEC")
    assert rx.match("XNV")
    # Future tags should fail until added (the failure is the canary).
    assert not rx.match("XSEG")
    assert not rx.match("XMODEL")


def test_body_entry_re_matches_all_three_tags():
    assert _BODY_ENTRY_RE.match("    X   ( int,    foo,  nfoo, 1 )")
    assert _BODY_ENTRY_RE.match("    XVEC( mjtNum, bar,  nbar, 3 )")
    assert _BODY_ENTRY_RE.match("    XNV ( mjtNum, baz,  nbaz, mjNFLUID )")
    # Tags that look similar but aren't in the set.
    assert not _BODY_ENTRY_RE.match("    X3 ( ... )")


def test_x_entry_re_captures_payload_for_all_tags():
    m = _X_ENTRY_RE.match("    XNV ( mjtNum, geom_fluid, ngeom, mjNFLUID ) \\")
    assert m
    assert "geom_fluid" in m.group(1)
    m2 = _X_ENTRY_RE.match("    X   ( int, geom_type, ngeom, 1 ) \\")
    assert m2
    assert "geom_type" in m2.group(1)


# ---------------------------------------------------------------------------
# End-to-end block parsing — the bug that motivated this file
# ---------------------------------------------------------------------------

def _block_lines(block_body: str) -> list:
    """Strip the leading newline + dedent so test bodies read naturally."""
    return textwrap.dedent(block_body).strip("\n").split("\n")


def test_block_body_does_not_stop_at_xnv():
    """The original regex matched (X|XVEC) only; XNV broke the body collector
    so every subsequent entry in the block was silently dropped. This pins
    the behaviour so MJMODEL_POINTERS_GEOM can never lose geom_user /
    geom_rgba again."""
    lines = _block_lines("""
        X   ( int,    geom_type,     ngeom, 1 )
        XNV ( mjtNum, geom_fluid,    ngeom, mjNFLUID )
        X   ( mjtNum, geom_user,     ngeom, MJ_M(nuser_geom) )
        X   ( float,  geom_rgba,     ngeom, 4 )
    """)
    entries = _parse_block_body(lines, 0)
    assert len(entries) == 4, "XNV must not abort the body collector"
    parsed = [_parse_pointer_entry(e) for e in entries]
    names = [p["name"] for p in parsed if p]
    assert names == ["geom_type", "geom_fluid", "geom_user", "geom_rgba"]


def test_parse_pointer_entry_for_xnv():
    """XNV entries parse as 4-tuples identical to X entries."""
    entry = _parse_pointer_entry("XNV ( mjtNum, geom_fluid, ngeom, mjNFLUID )")
    assert entry == {
        "type": "mjtNum",
        "name": "geom_fluid",
        "outer_dim": "ngeom",
        "stride": "mjNFLUID",
    }


# ---------------------------------------------------------------------------
# Two-source fold-in: mjxmacro.h + mjspecmacro.h => one snapshot
# ---------------------------------------------------------------------------

def test_shipped_snapshot_has_struct_field_sources(real_mjxmacro):
    """The snapshot writer should record which header each struct-field
    block came from so consumers can tell mjmodel-side from mjspec-side."""
    meta = real_mjxmacro.get("_meta", {})
    sources = meta.get("struct_field_sources")
    assert isinstance(sources, dict) and sources, (
        "expected _meta.struct_field_sources to be populated"
    )


def test_shipped_snapshot_attributes_mjspec_blocks_to_mjspecmacro(real_mjxmacro):
    """MJSCOMPILER_FIELDS + MJSPEC_FIELDS must come from mjspecmacro.h,
    not mjxmacro.h — they only exist in the spec header."""
    sources = real_mjxmacro["_meta"]["struct_field_sources"]
    for block in ("MJSCOMPILER_FIELDS", "MJSPEC_FIELDS"):
        assert block in sources, f"snapshot missing {block} source attribution"
        assert "mjspecmacro.h" in sources[block], (
            f"{block} should be sourced from mjspecmacro.h, got {sources[block]!r}"
        )


def test_shipped_snapshot_attributes_mjoption_blocks_to_mjxmacro(real_mjxmacro):
    """MJOPTION_FIELDS / MJSTATISTIC_FIELDS live in mjxmacro.h."""
    sources = real_mjxmacro["_meta"]["struct_field_sources"]
    for block in ("MJOPTION_FIELDS", "MJSTATISTIC_FIELDS"):
        assert block in sources, f"snapshot missing {block} source attribution"
        assert "mjxmacro.h" in sources[block], (
            f"{block} should be sourced from mjxmacro.h, got {sources[block]!r}"
        )
