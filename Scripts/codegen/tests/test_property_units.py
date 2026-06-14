# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the editor display-unit rule shapes: ``property_units``
(UE-native ``Units="m"`` meta on scalars, custom ``MjUnit="m"`` on
TArrays), ``hidden_attrs`` / ``hidden_canon_properties``
(EditConditionHides), and the validity drift check."""

from __future__ import annotations

import generate_ue_components as gen


def test_property_units_emits_units_meta_on_scalar():
    """Scalar/struct UE types get UE-native meta=(Units="...")."""
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {"margin": "float"},
        "canonicalizations": {},
        "element_rules": {
            "geom": {"property_units": {"margin": "m"}},
        },
    }
    out = gen.emit_schema_for_attrs(["margin"], rules, "geom", "Geom")
    assert 'Units="m"' in out.properties_h, out.properties_h
    assert out.properties_h.count('Units="m"') == 1


def test_property_units_emits_mjunit_meta_on_tarray():
    """TArray UE types get MjUnit="..." — UHT rejects Units on arrays;
    URLab's detail customization reads MjUnit at runtime."""
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {"size": "TArray<float>"},
        "canonicalizations": {},
        "element_rules": {
            "geom": {"property_units": {"size": "m"}},
        },
    }
    out = gen.emit_schema_for_attrs(["size"], rules, "geom", "Geom")
    assert 'MjUnit="m"' in out.properties_h, out.properties_h
    assert 'Units="m"' not in out.properties_h


def test_property_units_coexists_with_property_meta():
    """Units composes with existing property_meta (e.g. DisplayName)
    without one clobbering the other."""
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {},
        "canonicalizations": {},
        "element_rules": {
            "geom": {
                "property_meta": {"foo": 'DisplayName="Foo Bar"'},
                "property_units": {"foo": "m"},
            },
        },
    }
    out = gen.emit_schema_for_attrs(["foo"], rules, "geom", "Geom")
    assert 'DisplayName="Foo Bar"' in out.properties_h
    assert 'Units="m"' in out.properties_h


def test_property_units_validity_diagnoses_bad_unit():
    rules = {
        "element_rules": {
            "geom": {"property_units": {"size": "meters"}},
        },
    }
    gen._check_property_units_validity(rules)
    assert any("'meters'" in d.message and "geom" in d.message
               for d in gen._DIAGS_BUFFER.pending)


def test_property_units_validity_silent_on_known_units():
    rules = {
        "element_rules": {
            "geom": {"property_units": {"size": "m", "fovy": "deg"}},
        },
    }
    gen._check_property_units_validity(rules)
    assert len(gen._DIAGS_BUFFER.pending) == 0


def test_hidden_attrs_emits_edit_condition_hides():
    """hidden_attrs entries emit EditCondition=false + EditConditionHides
    so the field stays in C++ but the Details panel widget vanishes."""
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {"size": "TArray<float>"},
        "canonicalizations": {},
        "element_rules": {
            "geom": {"hidden_attrs": ["size"]},
        },
    }
    out = gen.emit_schema_for_attrs(["size"], rules, "geom", "Geom")
    assert 'EditCondition="false"' in out.properties_h
    assert 'EditConditionHides' in out.properties_h


def test_hidden_canon_properties_emits_edit_condition_hides():
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {},
        "canonicalizations": {
            "spatial_pose": {
                "absorbs_attrs": ["pos"],
                "emits_property": {"name": "Pos", "type": "FVector"},
                "import_helper": "X",
                "export_helper": "Y",
            },
        },
        "element_rules": {
            "geom": {
                "applies_canonicalizations": ["spatial_pose"],
                "hidden_canon_properties": ["spatial_pose"],
            },
        },
    }
    out = gen.emit_schema_for_attrs([], rules, "geom", "Geom")
    assert 'FVector Pos' in out.properties_h
    assert 'EditConditionHides' in out.properties_h
