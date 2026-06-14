# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks ``kept_props_for_category`` — the ordered prop-list helper
extracted from emit_schema_for_tests. Used by the
schema-completeness header that drives URLab.Codegen.* automation
tests, so the order + composition must stay deterministic."""

from __future__ import annotations

import generate_ue_components as gen


def test_emits_canon_property_first_then_per_attr_in_schema_order():
    rules = {
        "global_exclusions": [],
        "default_type": "float",
        "property_renames": {},
        "canonicalizations": {
            "spatial_pose": {
                "absorbs_attrs": ["pos"],
                "emits_property": {"name": "Pos", "type": "FVector"},
                "import_helper": "X",
                "export_helper": "Y",
            },
        },
        "element_rules": {
            "joint": {
                "applies_canonicalizations": ["spatial_pose"],
            },
        },
    }
    schema = {"joint": {"attrs": ["pos", "type", "axis", "limited"]}}
    cat_rules = {"schema_common_block": "joint.attrs"}
    out = gen.kept_props_for_category("joint", cat_rules, schema, rules)
    # Pos comes first (canon-owned), then per-attr in schema order
    # minus the canon-absorbed ``pos``.
    assert out == ["Pos", "type", "axis", "limited"]


def test_disable_schema_emission_returns_empty():
    rules = {"global_exclusions": [], "canonicalizations": {},
             "element_rules": {}}
    cat_rules = {"disable_schema_emission": True,
                 "schema_common_block": "joint.attrs"}
    out = gen.kept_props_for_category("joint", cat_rules, {}, rules)
    assert out == []


def test_xml_enum_attr_lowers_to_ue_property():
    rules = {
        "global_exclusions": [],
        "property_renames": {},
        "canonicalizations": {},
        "element_rules": {
            "camera": {
                "xml_enum_attrs": {
                    "mode": {
                        "ue_property": "TrackingMode",
                        "ue_enum_type": "EMjCameraTrackingMode",
                        "value_map": {"fixed": ["Fixed", "mjCAMLIGHT_FIXED"]},
                    },
                },
            },
        },
    }
    schema = {"camera": {"attrs": ["mode", "fovy"]}}
    cat_rules = {"schema_common_block": "camera.attrs"}
    out = gen.kept_props_for_category("camera", cat_rules, schema, rules)
    # ``mode`` -> ``TrackingMode``; ``fovy`` passes through.
    assert out == ["TrackingMode", "fovy"]


def test_schema_common_extra_attrs_append_after_block():
    rules = {"global_exclusions": [], "property_renames": {},
             "canonicalizations": {}, "element_rules": {}}
    schema = {"frame": {"attrs": []}}
    cat_rules = {
        "schema_common_block": "frame.attrs",
        "schema_common_extra_attrs": ["pos", "quat", "childclass"],
    }
    out = gen.kept_props_for_category("frame", cat_rules, schema, rules)
    assert out == ["pos", "quat", "childclass"]


def test_global_and_element_exclusions_drop_attrs():
    rules = {
        "global_exclusions": ["type"],
        "property_renames": {},
        "canonicalizations": {},
        "element_rules": {
            "joint": {"exclude_attrs": ["limited"]},
        },
    }
    schema = {"joint": {"attrs": ["type", "axis", "limited", "stiffness"]}}
    cat_rules = {"schema_common_block": "joint.attrs"}
    out = gen.kept_props_for_category("joint", cat_rules, schema, rules)
    assert "type" not in out
    assert "limited" not in out
    assert out == ["axis", "stiffness"]


def test_property_rename_lowers_to_renamed_prop():
    rules = {
        "global_exclusions": [],
        "property_renames": {"qpos": "Qpos"},
        "canonicalizations": {},
        "element_rules": {},
    }
    schema = {"keyframe": {"attrs": ["qpos", "time"]}}
    cat_rules = {"schema_common_block": "keyframe.attrs"}
    out = gen.kept_props_for_category("keyframe", cat_rules, schema, rules)
    assert out == ["Qpos", "time"]
