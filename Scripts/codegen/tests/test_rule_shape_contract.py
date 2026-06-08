# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Contract tests for codegen_rules.json. The rules file is the
single source of truth for the codegen — these tests pin the
structural invariants every emitter depends on so a hand-edit that
breaks them surfaces in pytest, not in a UE compile failure far
downstream.

Phase 7 of the codegen overhaul focuses on broader coverage; the
per-shape tests below complement the per-emitter unit tests that
landed in earlier phases."""

from __future__ import annotations

import generate_ue_components as gen


_KNOWN_TOP_LEVEL_KEYS = {
    # data driving the emit path
    "global_exclusions",
    "default_type",
    "property_renames",
    "setto_param_defaults",
    "type_mappings",
    "categories",
    "element_rules",
    "canonicalizations",
    "synthetic_categories",
    "generated_enums",
    "editor_option_helpers",
    "articulation_registry_path",
    "views",
    # drift-suppression allowlists
    "intentionally_unmodeled_elements",
    "intentionally_unmodeled_mjs_fields",
    "intentionally_unmodeled_compiler_attrs",
    "intentionally_default_typed_attrs",
    "intentionally_unmapped_mj_enum_values",
    # shared per-rule namespaces
    "compiler_attr_field_map",
}


def test_top_level_keys_are_recognised(real_rules):
    """Every top-level key in the rules JSON is either a known shape
    or a ``_note_*`` sibling string. Catches typo'd renames at edit
    time."""
    unknown = []
    for key in real_rules:
        if key.startswith("_") or key in _KNOWN_TOP_LEVEL_KEYS:
            continue
        unknown.append(key)
    assert not unknown, (
        f"unknown top-level keys in codegen_rules.json: {unknown}. "
        f"Either add them to _KNOWN_TOP_LEVEL_KEYS in this test or "
        f"_note_-prefix them to mark as documentation."
    )


def test_every_category_carries_required_keys(real_rules):
    """Every entry under ``categories`` needs a schema_common_block.
    Missing the block silently produces an empty emit; the test
    surfaces the missing key at JSON-edit time."""
    for cat, cat_rules in real_rules.get("categories", {}).items():
        if cat.startswith("_"):
            continue
        assert "schema_common_block" in cat_rules, (
            f"category '{cat}' missing schema_common_block")


def test_canonicalisations_referenced_by_element_rules_exist(real_rules):
    """element_rules[X].applies_canonicalizations must list canon
    names that exist in the top-level ``canonicalizations`` block.
    Otherwise per-element XML import silently drops absorbed attrs."""
    canon_keys = set(real_rules.get("canonicalizations", {}).keys())
    for elem, elem_rule in real_rules.get("element_rules", {}).items():
        for canon in elem_rule.get("applies_canonicalizations", []):
            assert canon in canon_keys, (
                f"element_rules['{elem}'].applies_canonicalizations "
                f"references '{canon}' which is not declared in the "
                f"top-level canonicalizations block.")


def test_xml_enum_attr_value_maps_have_correct_shape(real_rules):
    """Each ``value_map`` entry must be a 2-element ``[ue_member,
    mj_const]`` list. Mistyped entries (string vs list) silently fail
    to emit case labels in the codegen switch."""
    for elem, elem_rule in real_rules.get("element_rules", {}).items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            value_map = enum_def.get("value_map", {})
            for xml_val, mapping in value_map.items():
                assert isinstance(mapping, (list, tuple)), (
                    f"element_rules['{elem}'].xml_enum_attrs['{attr}']"
                    f".value_map['{xml_val}'] must be a list, got "
                    f"{type(mapping).__name__}")
                assert len(mapping) == 2, (
                    f"element_rules['{elem}'].xml_enum_attrs['{attr}']"
                    f".value_map['{xml_val}'] must be "
                    f"[ue_member, mj_const]; got {mapping!r}")


def test_synthetic_categories_carry_block_or_skip_flag(real_rules):
    """Every synthetic_categories entry must either name an
    mjxmacro_block or carry a disabled flag. A bare entry would
    silently emit nothing."""
    synth = real_rules.get("synthetic_categories", {})
    for cat_name, def_ in synth.items():
        if cat_name.startswith("_") or not isinstance(def_, dict):
            continue
        assert def_.get("mjxmacro_block") or def_.get("disabled"), (
            f"synthetic_categories['{cat_name}'] needs either an "
            f"mjxmacro_block or a disabled flag")


def test_type_mappings_use_known_ue_types(real_rules):
    """Every type in type_mappings must be a UE type the codegen
    can emit (i.e. resolves through _UE_TYPE_INFO or is a known
    TArray<...>). Surfacing typo'd types here catches "TArray<flot>"
    long before UHT fails."""
    known = set(gen._UE_TYPE_INFO.keys())
    for attr, ue_type in real_rules.get("type_mappings", {}).items():
        if attr.startswith("_"):
            continue
        if ue_type in known:
            continue
        if ue_type.startswith("TArray<") and ue_type.endswith(">"):
            inner = ue_type[len("TArray<"):-1]
            assert (inner in known or inner.startswith("U")
                    or inner.startswith("EMj")), (
                f"type_mappings['{attr}'] = '{ue_type}' has unknown "
                f"inner type '{inner}'")
            continue
        # URLab-side hand-rolled enums (EMj*) are legitimate UPROPERTY
        # types but live outside the registry's UE type universe.
        if ue_type.startswith("EMj"):
            continue
        # Unrecognised types fall through to default_type with a diag;
        # the test surfaces the JSON-edit-time intent.
        assert False, (
            f"type_mappings['{attr}'] = '{ue_type}' is not a "
            f"recognised UE type")
