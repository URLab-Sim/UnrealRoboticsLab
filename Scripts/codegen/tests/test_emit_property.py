# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Per-property emission shape tests."""

from __future__ import annotations

import pytest

from generate_ue_components import (
    EmittedSchema,
    emit_schema_for_attrs,
)


# override_toggle_name is exercised everywhere emit_schema_for_attrs runs;
# the cross-category guarantee is pinned in
# test_behavioral_emit_guarantees::test_override_toggle_name_matches_uproperty_for_all_categories.


def test_emit_basic_property(minimal_rules):
    out = emit_schema_for_attrs(["stiffness"], minimal_rules, "joint", "Joint")
    assert "float stiffness" in out.properties_h
    assert "bOverride_stiffness" in out.properties_h
    assert "= 0.0f;" in out.properties_h
    assert "MjXmlUtils::ReadAttrFloat" in out.imports_cpp
    assert 'TEXT("stiffness")' in out.imports_cpp
    assert "Element->stiffness" in out.exports_cpp


def test_emit_typed_property_range(minimal_rules):
    out = emit_schema_for_attrs(["range"], minimal_rules, "joint", "Joint")
    assert "TArray<float> range" in out.properties_h
    assert "MjXmlUtils::ReadAttrFloatArray" in out.imports_cpp
    assert "Element->range[i]" in out.exports_cpp


def test_emit_bool_property(minimal_rules):
    out = emit_schema_for_attrs(["limited"], minimal_rules, "joint", "Joint")
    assert "bool limited" in out.properties_h
    assert "MjXmlUtils::ReadAttrBool" in out.imports_cpp
    assert "Element->limited = limited ? 1 : 0;" in out.exports_cpp


def test_emit_vector_property(minimal_rules):
    out = emit_schema_for_attrs(["axis"], minimal_rules, "joint", "Joint")
    # 'axis' is in joint excludes; should not appear.
    assert "FVector axis" not in out.properties_h
    rules2 = {**minimal_rules,
              "element_rules": {"joint": {"exclude_attrs": []}}}
    out2 = emit_schema_for_attrs(["axis"], rules2, "joint", "Joint")
    assert "FVector axis" in out2.properties_h
    assert "MjXmlUtils::ReadAttrVec3" in out2.imports_cpp


def test_global_exclude_skipped(minimal_rules):
    out = emit_schema_for_attrs(["user", "stiffness"], minimal_rules, "joint", "Joint")
    assert "stiffness" in out.properties_h
    assert " user " not in out.properties_h


def test_element_exclude_skipped(minimal_rules):
    out = emit_schema_for_attrs(["pos", "axis", "stiffness"], minimal_rules, "joint", "Joint")
    assert "stiffness" in out.properties_h
    assert "FVector pos" not in out.properties_h
    assert "FVector axis" not in out.properties_h


def test_no_b_prefix_on_bool(minimal_rules):
    """`limited` emits as `bool limited`, not `bool bLimited`."""
    out = emit_schema_for_attrs(["limited"], minimal_rules, "joint", "Joint")
    assert "bool limited" in out.properties_h
    assert "bOverride_limited" in out.properties_h
    assert "bLimited" not in out.properties_h
