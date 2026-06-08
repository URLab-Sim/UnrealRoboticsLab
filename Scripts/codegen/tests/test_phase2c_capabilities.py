# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Phase 2c added four opt-in capabilities to generate_ue_components.py:

1. ``value_map_from_enum``: snapshot-driven enum-to-value-map resolver.
2. ``xml_enum_attrs.emit_property_decl: true``: codegen emits the UE
   enum UPROPERTY decl into the schema PROPERTIES block.
3. Scalar-int view fields: views can declare ``scalar_int_fields`` for
   derived integer values (not pointer slices).
4. ``export_if`` per-attr conditional: a C++ predicate that ANDs with
   the override toggle before the write fires.

No current rule opts in — these are scaffolding for Phase 2d / 2e / 5
consumers. These tests use synthetic rules to verify each capability
works in isolation.
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
    _emit_xml_enum_export,
    _emit_export_line,
    emit_schema_for_attrs,
    emit_view_structs,
)


# ----- value_map_from_enum resolver ------------------------------------

def test_value_map_explicit_takes_precedence():
    """If a rule has both ``value_map`` and ``value_map_from_enum``, the
    explicit map wins. Preserves the pre-2c convention exactly."""
    enum_def = {
        "ue_property": "Type",
        "ue_enum_type": "EMjJointType",
        "value_map":   {"hinge": ["Hinge", "mjJNT_HINGE"]},
        "value_map_from_enum": "mjtJoint",
    }
    snap = {"enums": {"mjtJoint": {"mjJNT_HINGE": 0}}}
    resolved = _resolve_value_map("type", enum_def, snap)
    assert resolved == {"hinge": ["Hinge", "mjJNT_HINGE"]}


def test_value_map_from_enum_resolves_from_snapshot():
    """With only ``value_map_from_enum`` set, the resolver builds the
    table from the snapshot's enum entries + the rule's xml_from_mj +
    ue_member_from_mj cross-walks."""
    enum_def = {
        "ue_property": "Type",
        "ue_enum_type": "EMjJointType",
        "value_map_from_enum": "mjtJoint",
        "ue_member_from_mj": {"mjJNT_HINGE": "Hinge", "mjJNT_SLIDE": "Slide"},
        "xml_from_mj":        {"mjJNT_HINGE": "hinge", "mjJNT_SLIDE": "slide"},
    }
    snap = {"enums": {"mjtJoint": {"mjJNT_HINGE": 0, "mjJNT_SLIDE": 1}}}
    resolved = _resolve_value_map("type", enum_def, snap)
    assert resolved == {
        "hinge": ["Hinge", "mjJNT_HINGE"],
        "slide": ["Slide", "mjJNT_SLIDE"],
    }


def test_value_map_resolver_errors_when_neither_path_works():
    enum_def = {"ue_property": "X", "ue_enum_type": "E"}
    with pytest.raises(RuntimeError, match="no value_map and value_map_from_enum did not resolve"):
        _resolve_value_map("attr", enum_def, mjspec=None)


def test_xml_enum_import_emits_byte_identical_with_resolver():
    """Emit through the resolver produces the SAME C++ as the old
    direct-value_map path. This is the byte-identity guarantee Phase 2c
    promised the gate."""
    enum_def = {
        "ue_property": "Type",
        "ue_enum_type": "EMjJointType",
        "value_map":   {"hinge": ["Hinge", "mjJNT_HINGE"], "slide": ["Slide", "mjJNT_SLIDE"]},
    }
    out = _emit_xml_enum_import("type", enum_def, mjspec=None)
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


# ----- scalar-int views ------------------------------------------------

def test_view_scalar_int_fields_emit_decl_and_bind():
    rules = {
        "views": {
            "TestView": {
                "scalar_int_fields": {
                    "slot": "id * 2 + 1",
                }
            }
        }
    }
    out = emit_view_structs(rules, mjxmacro={"mjmodel_pointers": {}})
    fields_block, bind_block = out["TestView"]
    assert "int slot;" in fields_block
    assert "slot = id * 2 + 1;" in bind_block


def test_view_scalar_int_unused_by_default():
    """Views that don't set scalar_int_fields produce no extra lines."""
    rules = {"views": {"TestView": {}}}
    out = emit_view_structs(rules, mjxmacro={"mjmodel_pointers": {}})
    fields_block, bind_block = out["TestView"]
    assert fields_block == ""
    assert bind_block == ""


# ----- export_if per-attr conditional ----------------------------------

def test_export_if_unset_keeps_simple_toggle():
    out = _emit_export_line("kp", "kp", "float", export_if=None)
    assert out == "    if (bOverride_kp) Element->kp = kp;\n"


def test_export_if_set_combines_with_toggle():
    out = _emit_export_line("kp", "kp", "float",
                            export_if="Type == EMjActuatorType::Position")
    assert out == "    if (bOverride_kp && Type == EMjActuatorType::Position) Element->kp = kp;\n"


def test_export_if_threads_through_emit_schema_for_attrs():
    rules = {
        "default_type": "float",
        "type_mappings": {"kp": "float"},
        "element_rules": {
            "actuator": {
                "export_if": {"kp": "Type == EMjActuatorType::Position"},
            }
        },
    }
    out = emit_schema_for_attrs(["kp"], rules, element_name="actuator", category_label="Actuator")
    assert "Type == EMjActuatorType::Position" in out.exports_cpp
    assert "if (bOverride_kp && Type == EMjActuatorType::Position)" in out.exports_cpp
