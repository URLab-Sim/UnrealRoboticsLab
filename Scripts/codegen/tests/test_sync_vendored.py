# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Unit tests for the manifest parser + formatter in sync_vendored.py.

Network fetches are NOT exercised here — they need the real GitHub
endpoints. The tests cover only the deterministic-rewrite property
(re-parsing + re-formatting the same content produces the same bytes)
and the column-ordering / sort-by-dest invariants.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)

from sync_vendored import (  # noqa: E402
    _parse_manifest,
    _format_manifest,
    _extract_preamble,
)


_SAMPLE = """\
# Vendored sources

Preamble paragraph here.

| file | upstream_url | upstream_sha | retrieved_date | license | local_dest |
|------|--------------|--------------|-----------------|---------|------------|
| foo.h | https://example.com/{sha}/foo.h | abc123 | 2026-01-01 | MIT | Scripts/codegen/_vendored/foo.h |
| bar.h | https://example.com/{sha}/bar.h | def456 | 2026-01-02 | Apache-2.0 | Scripts/codegen/_vendored/bar.h |
"""


def test_parse_skips_separator_and_preamble():
    rows = _parse_manifest(_SAMPLE)
    assert len(rows) == 2
    assert rows[0]["file"] == "foo.h"
    assert rows[0]["upstream_sha"] == "abc123"
    assert rows[1]["file"] == "bar.h"


def test_extract_preamble_stops_at_first_row():
    pre = _extract_preamble(_SAMPLE)
    assert "# Vendored sources" in pre
    assert "abc123" not in pre


def test_format_sorts_by_local_dest():
    rows = _parse_manifest(_SAMPLE)
    out = _format_manifest(rows, "# header\n")
    # bar.h comes first because Scripts/.../bar.h < Scripts/.../foo.h
    bar_idx = out.index("| bar.h |")
    foo_idx = out.index("| foo.h |")
    assert bar_idx < foo_idx


def test_format_is_deterministic_round_trip():
    """Parse + format + parse + format must produce identical bytes —
    that's the merge-conflict-resistance property the manifest design
    promises."""
    pre = _extract_preamble(_SAMPLE)
    rows = _parse_manifest(_SAMPLE)
    once = _format_manifest(rows, pre)
    twice = _format_manifest(_parse_manifest(once), _extract_preamble(once))
    assert once == twice
