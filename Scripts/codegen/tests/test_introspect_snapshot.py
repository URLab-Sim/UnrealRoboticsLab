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


def test_snapshot_has_known_mjs_functions():
    snap = _load()
    fns = snap["functions"]
    # mjs_addBody is one of the canonical mjsX constructors. If the
    # scraper missed it, the schema-evolution audit is broken.
    assert "mjs_addBody" in fns, list(fns)[:5]
    body_fn = fns["mjs_addBody"]
    # Body takes (mjsBody* body) as first arg per upstream API.
    assert body_fn["return_type"].endswith("mjsBody *") or body_fn["return_type"] == "mjsBody *"
    assert any("body" in (p["c_type"].lower()) for p in body_fn["params"])


def test_snapshot_has_known_enums():
    snap = _load()
    enums = snap["enums"]
    for expected in ("mjtJoint", "mjtGeom", "mjtSensor", "mjtObj"):
        assert expected in enums, expected
        members = enums[expected]["members"]
        assert len(members) > 0


def test_snapshot_has_known_structs():
    snap = _load()
    structs = snap["structs"]
    # mjsBody is the canonical spec struct; mjStatistic is one of the
    # synthetic_categories pilots' c_struct_type targets.
    assert any(name in structs for name in ("mjsBody_", "mjsBody"))


def test_snapshot_has_known_defines():
    snap = _load()
    defs = snap["defines"]
    for expected in ("mjNREF", "mjNIMP", "mjMAXVAL", "mjPI"):
        assert expected in defs, expected


def test_header_hash_present_and_stable():
    snap = _load()
    assert "_meta" in snap
    assert snap["_meta"]["snapshot_version"] >= 1
    h = snap["_meta"]["header_hash"]
    assert len(h) == 64  # SHA-256 hex
