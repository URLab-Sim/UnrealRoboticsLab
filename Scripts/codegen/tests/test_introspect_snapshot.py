# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Smoke tests for the clang-AST introspect snapshot.

Skipped automatically when libclang isn't available (e.g. CI runners
without the package). The real validation happens on the committed
``Scripts/introspect_snapshot.json`` itself — these tests just spot-check
that schema-evolution surprises don't slip through.
"""

from __future__ import annotations

import json
import os

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_ROOT = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
_SNAPSHOT = os.path.join(_PLUGIN_ROOT, "Scripts", "codegen", "snapshots", "introspect_snapshot.json")


def _load() -> dict:
    if not os.path.exists(_SNAPSHOT):
        pytest.skip("introspect_snapshot.json not built yet; run "
                    "build_introspect_snapshot.py")
    with open(_SNAPSHOT, "r", encoding="utf-8") as f:
        return json.load(f)


def test_snapshot_meta_and_top_level_keys_present():
    """One combined smoke test: top-level sections are present, each is
    non-empty, and the ``_meta`` block carries a stable header hash.
    Substring presence of specific MuJoCo names (mjs_addBody, mjtJoint,
    etc.) is covered by ``test_enum_scrape::test_real_snapshot_has_
    expected_enums`` plus the regen --check gate."""
    snap = _load()
    for section in ("functions", "enums", "structs", "defines"):
        assert section in snap, section
        assert len(snap[section]) > 0, section
    assert "_meta" in snap
    assert snap["_meta"]["snapshot_version"] >= 1
    assert len(snap["_meta"]["header_hash"]) == 64
