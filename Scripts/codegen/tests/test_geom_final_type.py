# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the ``geom_final_type`` rule shape — resolves an mjsGeom
type at compile time + emits the mesh-name export under the right
EMjGeomType condition."""

from __future__ import annotations

import generate_ue_components as gen
from generate_ue_components import _emit_geom_final_type_block


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
