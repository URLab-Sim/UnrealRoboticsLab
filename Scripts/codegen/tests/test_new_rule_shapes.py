# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Unit tests for the v8/v9 rule shapes introduced during the codegen
overhaul: objtype_dispatch, geom_final_type, fully_emitted (banner
overwrite + safety audit), property_default (with cross-check), and
_guarded_export(multiline=True).

Each test runs in isolation against ``_DIAGS`` so it can assert on
diagnostics without leaking state.
"""

from __future__ import annotations

import pytest

import generate_ue_components as gen
from generate_ue_components import (
    _emit_objtype_dispatch_block,
    _emit_geom_final_type_block,
    _guarded_export,
    _audit_banner_safety,
)


@pytest.fixture(autouse=True)
def _clear_diags():
    """Each test starts with a clean diagnostics buffer + strict counter."""
    gen._DIAGS.clear()
    gen._STRICT_DIAGS_FIRED[0] = 0
    yield
    gen._DIAGS.clear()
    gen._STRICT_DIAGS_FIRED[0] = 0


# ---------- objtype_dispatch -------------------------------------------------

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


# ---------- geom_final_type --------------------------------------------------

def _geom_value_map_rules():
    """Minimal cat/element rules that exercise the FinalType switch."""
    cat_rules = {
        "geom_final_type": {
            "xml_enum_ref": "type",
            "override_field": "bOverride_Type",
            "default_lookup": "Default->Type",
            "default_fallback": "mjGEOM_SPHERE",
            "name_export_for": "mjGEOM_MESH",
            "name_field": "MeshName",
            "name_setter": "Element->meshname.assign",
            "name_target": "*Element",
        },
    }
    element_rules = {
        "xml_enum_attrs": {
            "type": {
                "ue_property": "Type",
                "ue_enum": "EMjGeomType",
                "ue_enum_type": "EMjGeomType",
                "override_toggle": "bOverride_Type",
                "default_lookup": "Default->Type",
                "value_map": {
                    "sphere": ["Sphere", "mjGEOM_SPHERE"],
                    "box":    ["Box",    "mjGEOM_BOX"],
                    "mesh":   ["Mesh",   "mjGEOM_MESH"],
                },
            },
        },
    }
    return cat_rules, element_rules


def test_geom_final_type_emits_full_block():
    cat_rules, element_rules = _geom_value_map_rules()
    out = _emit_geom_final_type_block(cat_rules, element_rules)
    assert out is not None
    assert "FinalType" in out
    assert "case EMjGeomType::Sphere: FinalType = mjGEOM_SPHERE;" in out
    assert "case EMjGeomType::Box: FinalType = mjGEOM_BOX;" in out
    assert "case EMjGeomType::Mesh: FinalType = mjGEOM_MESH;" in out
    assert "MeshName" in out
    assert len(gen._DIAGS) == 0


def test_geom_final_type_returns_none_when_no_rule():
    assert _emit_geom_final_type_block({}, {}) is None
    assert len(gen._DIAGS) == 0


def test_geom_final_type_diagnoses_missing_enum_def():
    cat_rules = {
        "geom_final_type": {
            "xml_enum_ref": "nonexistent",
            "default_fallback": "mjGEOM_SPHERE",
            "first_mj_const_token": "mjGEOM_PLANE",
            "name_for": "mjGEOM_MESH",
            "name_field": "MeshName",
            "name_setter": "Element->meshname.assign",
            "name_target": "*Element",
        },
    }
    out = _emit_geom_final_type_block(cat_rules, {"xml_enum_attrs": {}})
    assert out is None
    assert any("nonexistent" in d.message for d in gen._DIAGS)


def test_geom_final_type_diagnoses_empty_value_map():
    cat_rules, element_rules = _geom_value_map_rules()
    # Wipe out the value_map and provide no mjspec to fall back to.
    element_rules["xml_enum_attrs"]["type"]["value_map"] = {}
    out = _emit_geom_final_type_block(cat_rules, element_rules)
    assert out is None
    assert any("empty value_map" in d.message for d in gen._DIAGS)


# ---------- property_default + cross-check -----------------------------------

def test_property_default_emitted_on_base():
    """`property_default` ends up as the UPROPERTY initializer when
    ``emit_property_decl: true`` is set on an xml_enum_attr."""
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {},
        "canonicalizations": {},
        "element_rules": {
            "actuator": {
                "xml_enum_attrs": {
                    "gaintype": {
                        "ue_property": "GainType",
                        "ue_enum": "EMjGainType",
                        "ue_enum_type": "EMjGainType",
                        "override_toggle": "bOverride_GainType",
                        "default_lookup": "Default->GainType",
                        "emit_property_decl": True,
                        "property_default": "Fixed",
                        "value_map": {
                            "fixed":  ["Fixed",  "mjGAIN_FIXED"],
                            "affine": ["Affine", "mjGAIN_AFFINE"],
                        },
                    },
                },
            },
        },
    }
    out = gen.emit_schema_for_attrs(
        ["gaintype"], rules, "actuator", "Actuator",
        apply_canonicalizations=True,
    )
    assert "EMjGainType GainType = EMjGainType::Fixed;" in out.properties_h


def test_property_default_cross_check_diagnoses_bad_member():
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {},
        "canonicalizations": {},
        "element_rules": {
            "actuator": {
                "xml_enum_attrs": {
                    "gaintype": {
                        "ue_property": "GainType",
                        "ue_enum": "EMjGainType",
                        "ue_enum_type": "EMjGainType",
                        "override_toggle": "bOverride_GainType",
                        "default_lookup": "Default->GainType",
                        "emit_property_decl": True,
                        # NotAMember does not appear in value_map below.
                        "property_default": "NotAMember",
                        "value_map": {
                            "fixed":  ["Fixed",  "mjGAIN_FIXED"],
                            "affine": ["Affine", "mjGAIN_AFFINE"],
                        },
                    },
                },
            },
        },
    }
    gen.emit_schema_for_attrs(
        ["gaintype"], rules, "actuator", "Actuator",
        apply_canonicalizations=True,
    )
    assert any("NotAMember" in d.message for d in gen._DIAGS)


# ---------- _guarded_export(multiline=True) ----------------------------------

def test_guarded_export_singleline_form():
    """Single-line form: ``if (toggle) <body>`` on one line."""
    out = _guarded_export("bOverride_x", "Element->x = X;")
    assert out == "    if (bOverride_x) Element->x = X;\n"


def test_guarded_export_multiline_brace_layout():
    """Multiline form: brace on its own line; body is emitted verbatim
    between braces (caller owns inner indentation)."""
    body = "        line_a;\n        line_b;\n"
    out = _guarded_export("bOverride_x", body, multiline=True)
    expected = (
        "    if (bOverride_x)\n"
        "    {\n"
        "        line_a;\n"
        "        line_b;\n"
        "    }\n"
    )
    assert out == expected


def test_guarded_export_multiline_with_extra_cond():
    """``extra_cond`` (default order) appears after the toggle."""
    body = "        do_thing();\n"
    out = _guarded_export(
        "bOverride_x", body,
        extra_cond="ready",
        multiline=True,
    )
    assert out.startswith("    if (bOverride_x && ready)\n    {\n")
    assert "        do_thing();\n" in out
    assert out.endswith("    }\n")


def test_guarded_export_extra_cond_first_order():
    """``extra_cond_first=True`` flips the order and parenthesises
    the extra cond."""
    out = _guarded_export(
        "bOverride_x", "Element->x = X;",
        extra_cond="ready",
        extra_cond_first=True,
    )
    assert out == "    if ((ready) && bOverride_x) Element->x = X;\n"


# ---------- banner-safety audit ---------------------------------------------

def test_audit_banner_safety_silent_on_template_only(tmp_path):
    """A subclass file containing only its own header include and a
    bare ``Type = ...`` constructor should not trigger a diagnostic."""
    p = tmp_path / "MjBoxGeom.cpp"
    p.write_text(
        '#include "MuJoCo/Components/Geometry/MjBoxGeom.h"\n'
        "UMjBoxGeom::UMjBoxGeom()\n"
        "{\n"
        "    Type = EMjGeomType::Box;\n"
        "}\n",
        encoding="utf-8",
    )
    subtype = {"class_name": "UMjBoxGeom"}
    _audit_banner_safety(str(p), subtype)
    assert len(gen._DIAGS) == 0


def test_audit_banner_safety_flags_extra_include(tmp_path):
    """An extra non-template #include in a soon-to-be-overwritten file
    should trigger a diagnostic so the rule author knows hand-rolled
    content will be discarded."""
    p = tmp_path / "MjBoxGeom.cpp"
    p.write_text(
        '#include "MuJoCo/Components/Geometry/MjBoxGeom.h"\n'
        '#include "SomeOther/Header.h"\n'
        "UMjBoxGeom::UMjBoxGeom()\n"
        "{\n"
        "    Type = EMjGeomType::Box;\n"
        "}\n",
        encoding="utf-8",
    )
    subtype = {"class_name": "UMjBoxGeom"}
    _audit_banner_safety(str(p), subtype)
    assert any("SomeOther/Header.h" in d.message for d in gen._DIAGS)


def test_audit_banner_safety_flags_extra_ctor_body(tmp_path):
    """Constructor body content beyond ``Type = EMj...::X`` and the
    rule-provided ``extra_constructor`` should be flagged — that's where
    the joint-subtype bOverride_Type bug came from."""
    p = tmp_path / "MjHingeJoint.cpp"
    p.write_text(
        '#include "MuJoCo/Components/Joints/MjHingeJoint.h"\n'
        "UMjHingeJoint::UMjHingeJoint()\n"
        "{\n"
        "    Type = EMjJointType::Hinge;\n"
        "    HandRolledHelperCall();\n"
        "}\n",
        encoding="utf-8",
    )
    subtype = {
        "class_name": "UMjHingeJoint",
        "extra_constructor": "",
    }
    _audit_banner_safety(str(p), subtype)
    assert any("non-template ctor content" in d.message
               and "HandRolledHelperCall" in d.message
               for d in gen._DIAGS)
