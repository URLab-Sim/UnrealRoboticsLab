# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the ``objtype_dispatch`` rule shape — emits a switch on a
UE enum that assigns each case's ``expr`` to ``target``."""

from __future__ import annotations

import generate_ue_components as gen
from generate_ue_components import _emit_objtype_dispatch_block


def test_objtype_dispatch_emits_switch_with_default():
    cat_rules = {
        "type_enum_name": "EMjEqualityType",
        "objtype_dispatch": {
            "discriminator": "Type",
            "target": "Element->objtype",
            "default": "mjOBJ_UNKNOWN",
            "cases": [
                {"keys": ["Connect", "Weld"], "expr": "mjOBJ_BODY"},
                {"keys": ["Joint"], "expr": "mjOBJ_JOINT"},
            ],
        },
    }
    out = _emit_objtype_dispatch_block(cat_rules)
    assert "switch (Type)" in out
    assert "case EMjEqualityType::Connect:" in out
    assert "case EMjEqualityType::Weld:" in out
    assert "Element->objtype = mjOBJ_BODY;" in out
    assert "case EMjEqualityType::Joint:" in out
    assert "Element->objtype = mjOBJ_JOINT;" in out
    assert "default:" in out
    assert "Element->objtype = mjOBJ_UNKNOWN;" in out
    assert len(gen._DIAGS) == 0


def test_objtype_dispatch_returns_none_when_no_rule():
    assert _emit_objtype_dispatch_block({}) is None
    assert len(gen._DIAGS) == 0


def test_objtype_dispatch_diagnoses_missing_type_enum_name():
    out = _emit_objtype_dispatch_block({
        "objtype_dispatch": {
            "discriminator": "Type",
            "target": "Element->objtype",
            "cases": [{"keys": ["A"], "expr": "X"}],
        },
    })
    assert out is None
    assert any("type_enum_name" in d.message for d in gen._DIAGS)


def test_objtype_dispatch_diagnoses_empty_cases():
    out = _emit_objtype_dispatch_block({
        "type_enum_name": "EMjEqualityType",
        "objtype_dispatch": {
            "discriminator": "Type",
            "target": "Element->objtype",
            "cases": [],
        },
    })
    assert out is None
    assert any("no cases" in d.message for d in gen._DIAGS)


def test_objtype_dispatch_diagnoses_empty_keys():
    out = _emit_objtype_dispatch_block({
        "type_enum_name": "EMjEqualityType",
        "objtype_dispatch": {
            "discriminator": "Type",
            "target": "Element->objtype",
            "cases": [{"keys": [], "expr": "mjOBJ_BODY"}],
        },
    })
    assert out is None
    assert any("empty" in d.message and "keys" in d.message
               for d in gen._DIAGS)


def test_objtype_dispatch_diagnoses_missing_expr():
    out = _emit_objtype_dispatch_block({
        "type_enum_name": "EMjEqualityType",
        "objtype_dispatch": {
            "discriminator": "Type",
            "target": "Element->objtype",
            "cases": [{"keys": ["Connect"]}],
        },
    })
    assert out is None
    assert any("missing 'expr'" in d.message for d in gen._DIAGS)
