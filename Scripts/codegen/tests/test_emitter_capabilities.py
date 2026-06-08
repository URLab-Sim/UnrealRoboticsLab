# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Tests for opt-in emitter capabilities in generate_ue_components.py:

1. ``_resolve_value_map``: returns the rule's literal ``value_map``;
   missing ``value_map`` raises.
2. ``xml_enum_attrs.emit_property_decl: true``: codegen emits the UE
   enum UPROPERTY decl into the schema PROPERTIES block.

Each test uses synthetic rules to exercise its capability in isolation.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)

import pytest  # noqa: E402

from generate_ue_components import (  # noqa: E402
    _resolve_value_map,
    _emit_xml_enum_import,
    emit_schema_for_attrs,
)


# ----- value_map resolver ----------------------------------------------

def test_value_map_returned_verbatim():
    """The resolver returns the rule's literal ``value_map`` unchanged."""
    enum_def = {
        "ue_property": "Type",
        "ue_enum_type": "EMjJointType",
        "value_map":   {"hinge": ["Hinge", "mjJNT_HINGE"]},
    }
    resolved = _resolve_value_map("type", enum_def)
    assert resolved == {"hinge": ["Hinge", "mjJNT_HINGE"]}


def test_value_map_resolver_errors_when_missing():
    enum_def = {"ue_property": "X", "ue_enum_type": "E"}
    with pytest.raises(RuntimeError, match="missing value_map"):
        _resolve_value_map("attr", enum_def)


def test_xml_enum_import_emits_expected_blocks():
    enum_def = {
        "ue_property": "Type",
        "ue_enum_type": "EMjJointType",
        "value_map":   {"hinge": ["Hinge", "mjJNT_HINGE"], "slide": ["Slide", "mjJNT_SLIDE"]},
    }
    out = _emit_xml_enum_import("type", enum_def)
    assert "Type = EMjJointType::Hinge" in out
    assert "Type = EMjJointType::Slide" in out
    assert "bOverride_Type = true" in out


# ----- emit_property_decl opt-in ---------------------------------------

def test_emit_property_decl_off_skips_uproperty():
    """Default behaviour: xml_enum_attrs entries do NOT emit the UE
    UPROPERTY decl (the .h hand-declares it)."""
    rules = {
        "element_rules": {
            "joint": {
                "xml_enum_attrs": {
                    "type": {
                        "ue_property": "Type",
                        "ue_enum_type": "EMjJointType",
                        "value_map": {"hinge": ["Hinge", "mjJNT_HINGE"]},
                    }
                }
            }
        }
    }
    out = emit_schema_for_attrs([], rules, element_name="joint", category_label="Joint")
    assert "EMjJointType" not in out.properties_h


def test_emit_property_decl_on_emits_uproperty_pair():
    """With opt-in, codegen emits both the override toggle and the
    EMjX UPROPERTY using the standard _emit_uproperty shape."""
    rules = {
        "element_rules": {
            "joint": {
                "xml_enum_attrs": {
                    "type": {
                        "ue_property": "Type",
                        "ue_enum_type": "EMjJointType",
                        "emit_property_decl": True,
                        "value_map": {"hinge": ["Hinge", "mjJNT_HINGE"]},
                    }
                }
            }
        }
    }
    out = emit_schema_for_attrs([], rules, element_name="joint", category_label="Joint")
    assert "EMjJointType Type" in out.properties_h
    assert "bOverride_Type" in out.properties_h


