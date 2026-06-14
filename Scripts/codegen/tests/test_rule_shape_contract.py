# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Contract tests for codegen_rules.json. The rules file is the
single source of truth for the codegen — these tests pin the
structural invariants every emitter depends on, so a hand-edit
that breaks them surfaces in pytest instead of as a UE compile
failure far downstream."""

from __future__ import annotations

import generate_ue_components as gen  # noqa: F401  (kept for module discovery)
from _codegen_core import _UE_TYPE_INFO


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
    mj_const]`` list, non-empty xml key, and the map itself non-empty.
    An empty map silently emits a dead block that toggles the override
    flag without ever assigning the enum; empty xml keys match every
    missing attr at runtime via ``GetAttribute()`` returning ``""``."""
    for elem, elem_rule in real_rules.get("element_rules", {}).items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            value_map = enum_def.get("value_map", {})
            assert value_map, (
                f"element_rules['{elem}'].xml_enum_attrs['{attr}']"
                f".value_map is empty — emit would set the override "
                f"toggle without ever assigning the enum.")
            for xml_val, mapping in value_map.items():
                assert xml_val != "", (
                    f"element_rules['{elem}'].xml_enum_attrs['{attr}']"
                    f".value_map has an empty-string xml key — at "
                    f"runtime this matches every missing attr.")
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


# ---------- inner-key shape contracts ----------------------------------
#
# The top-level allowlist catches typo'd renames at the rules root, but
# inner-key typos (e.g. ``attr_to_mjs_fields`` instead of
# ``attr_to_mjs_field``) silently fall through every reader's
# ``.get(..., {})``. The contracts below close that loop: every reader
# fetches by bracketed access against the keys listed here, so a typo
# surfaces in pytest instead of as a KeyError mid-emit (or worse — a
# silent no-op that ships wrong-runtime C++).


_CATEGORY_REQUIRED = {"schema_common_block", "base_class_name", "base_class_header", "mjs_struct"}
_CATEGORY_OPTIONAL = {
    "subtypes", "layout", "subtype_setto", "type_enum_name",
    "schema_subtypes_block", "schema_common_extra_attrs",
    "objtype_dispatch", "geom_final_type", "applies_canonicalizations",
    "subclass_directory_pub", "subclass_directory_priv",
    "xml_passthrough_emission", "disable_mjs_export_emission",
}
_SUBTYPE_REQUIRED = {"key", "enum_value", "class_name"}
_SUBTYPE_OPTIONAL = {
    "header", "fully_emitted", "case_body_override", "extra_constructor",
}
_ELEMENT_RULE_OPTIONAL = {
    "exclude_attrs", "applies_canonicalizations", "xml_enum_attrs",
    "attr_to_mjs_field", "property_renames", "property_meta",
    "property_units", "property_defaults", "target_collations",
    "common_imports", "mjs_data_packed_attrs", "mjs_double_vec_attrs",
    "sentinel_skip_export", "export_skip_attrs", "unit_conversion",
    "vec3_convert", "hidden_canon_properties", "hidden_attrs",
    "canon_export_skip", "xml_passthrough_emission", "common_attrs",
    "field_apply_to_spec_mode", "field_defaults",
}
_XML_ENUM_ATTR_REQUIRED = {"ue_property", "ue_enum_type", "value_map"}
_XML_ENUM_ATTR_OPTIONAL = {
    "mjs_field", "mjs_cast", "has_override_toggle",
    "emit_property_decl", "property_default", "property_meta",
    "from_mj_enum",
}
_CANON_REQUIRED = {"absorbs_attrs"}
_CANON_OPTIONAL = {
    "emits_property", "category_specifics",
    "import_helper", "export_helper",
}


def _assert_inner_keys(path: str, obj: dict, required: set, optional: set):
    for k in required:
        assert k in obj, f"{path}: missing required key '{k}'"
    allowed = required | optional
    bad = [k for k in obj if not k.startswith("_") and k not in allowed]
    assert not bad, (
        f"{path}: unknown key(s) {bad}. Either add to the contract "
        f"allowlist in test_rule_shape_contract.py or _note_-prefix "
        f"as documentation."
    )


def test_category_inner_keys_match_contract(real_rules):
    for cat, cat_rules in real_rules.get("categories", {}).items():
        if cat.startswith("_"):
            continue
        _assert_inner_keys(
            f"categories['{cat}']", cat_rules,
            _CATEGORY_REQUIRED, _CATEGORY_OPTIONAL,
        )


def test_category_subtype_inner_keys_match_contract(real_rules):
    for cat, cat_rules in real_rules.get("categories", {}).items():
        if cat.startswith("_"):
            continue
        for i, sub in enumerate(cat_rules.get("subtypes", [])):
            if not isinstance(sub, dict):
                continue
            _assert_inner_keys(
                f"categories['{cat}'].subtypes[{i}] ({sub.get('key', '?')})",
                sub, _SUBTYPE_REQUIRED, _SUBTYPE_OPTIONAL,
            )


def test_element_rule_inner_keys_match_contract(real_rules):
    for elem, elem_rule in real_rules.get("element_rules", {}).items():
        if elem.startswith("_"):
            continue
        _assert_inner_keys(
            f"element_rules['{elem}']", elem_rule,
            set(), _ELEMENT_RULE_OPTIONAL,
        )


def test_xml_enum_attr_inner_keys_match_contract(real_rules):
    for elem, elem_rule in real_rules.get("element_rules", {}).items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            if not isinstance(enum_def, dict):
                continue
            _assert_inner_keys(
                f"element_rules['{elem}'].xml_enum_attrs['{attr}']",
                enum_def,
                _XML_ENUM_ATTR_REQUIRED, _XML_ENUM_ATTR_OPTIONAL,
            )


def test_canonicalisation_inner_keys_match_contract(real_rules):
    for canon, canon_def in real_rules.get("canonicalizations", {}).items():
        if canon.startswith("_"):
            continue
        _assert_inner_keys(
            f"canonicalizations['{canon}']", canon_def,
            _CANON_REQUIRED, _CANON_OPTIONAL,
        )


def test_synthetic_category_required_public_header(real_rules):
    """Every active synthetic_categories entry needs ``public_header``
    pointing under ``MuJoCo/Generated/``. Missing key would KeyError at
    emit; a stray absolute path would write outside the plugin tree."""
    for cat_name, def_ in real_rules.get("synthetic_categories", {}).items():
        if cat_name.startswith("_") or not isinstance(def_, dict):
            continue
        if def_.get("disabled"):
            continue
        assert "public_header" in def_, (
            f"synthetic_categories['{cat_name}']: missing public_header"
        )
        ph = def_["public_header"]
        assert ph.startswith("MuJoCo/Generated/") and ph.endswith(".h"), (
            f"synthetic_categories['{cat_name}'].public_header = "
            f"'{ph}' must be 'MuJoCo/Generated/*.h'"
        )


def test_generated_enums_carry_ue_name_and_mapping(real_rules):
    """generated_enums entries must declare ue_name (starts with EMj)
    and at least one mapping source (ue_member_from_mj or
    extra_members)."""
    for group, group_def in real_rules.get("generated_enums", {}).items():
        if group.startswith("_") or not isinstance(group_def, dict):
            continue
        for i, entry in enumerate(group_def.get("enums", [])):
            if entry.get("disabled"):
                continue
            ue_name = entry.get("ue_name", "")
            assert ue_name.startswith("EMj"), (
                f"generated_enums['{group}'].enums[{i}].ue_name = "
                f"'{ue_name}' must start with 'EMj'"
            )
            has_mj = entry.get("from_mj_enum")
            has_members = entry.get("ue_member_from_mj") or entry.get("extra_members")
            assert has_mj or has_members, (
                f"generated_enums['{group}'].enums[{i}] ({ue_name}): "
                f"needs from_mj_enum or extra_members; otherwise emits "
                f"nothing."
            )


def test_setto_param_defaults_are_finite_numeric_literals(real_rules):
    """Values land VERBATIM in the generated C++ as a ternary's else
    branch (see ``_emit_setto_call``). Allowed shapes: int / signed-or-
    -unsigned float with optional decimal + exponent. Reject ``"inf"``,
    ``"nan"``, hex, or empty strings — they compile to non-finite or
    invalid C++."""
    import re
    finite_re = re.compile(r"^-?\d+(\.\d*)?([eE][+-]?\d+)?$")
    defaults = real_rules.get("setto_param_defaults", {})
    for fn_name, params in defaults.items():
        if fn_name.startswith("_") or not isinstance(params, dict):
            continue
        for pname, value in params.items():
            assert isinstance(value, str), (
                f"setto_param_defaults['{fn_name}']['{pname}'] = "
                f"{value!r}: must be a string (drops verbatim into C++)"
            )
            assert finite_re.match(value), (
                f"setto_param_defaults['{fn_name}']['{pname}'] = "
                f"'{value}': must be a finite decimal literal "
                f"('inf' / 'nan' / hex / empty rejected)"
            )


def test_type_mappings_use_known_ue_types(real_rules):
    """Every type in type_mappings must be a UE type the codegen
    can emit (i.e. resolves through _UE_TYPE_INFO or is a known
    TArray<...>). Surfacing typo'd types here catches "TArray<flot>"
    long before UHT fails."""
    known = set(_UE_TYPE_INFO.keys())
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
