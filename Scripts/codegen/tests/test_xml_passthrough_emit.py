# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for ``emit_xml_passthrough_body``. Categories with
``xml_passthrough_emission: true`` (flexcomp) build an MJCF XML string
instead of writing through the mjs spec API — the codegen owns the
per-attr formatter so a new schema attr flows end-to-end. The body
emits ``name="value"`` substrings for every override-tracked
UPROPERTY."""

from __future__ import annotations

import generate_ue_components as gen


def _rules(attr: str, ue_type: str) -> dict:
    return {
        "default_type": "float",
        "type_mappings": {attr: ue_type},
        "element_rules": {"thing": {}},
    }


def test_float_attr_emits_printf_with_toggle_guard():
    body = gen.emit_xml_passthrough_body(["stiffness"], _rules("stiffness", "float"), element_name="thing")
    assert "FString Out;" in body
    assert "if (bOverride_stiffness)" in body
    assert '" stiffness=\\"%f\\""' in body
    assert "return Out;" in body


def test_int32_attr_emits_printf_with_d_format():
    body = gen.emit_xml_passthrough_body(["group"], _rules("group", "int32"), element_name="thing")
    assert "if (bOverride_group)" in body
    assert '" group=\\"%d\\""' in body


def test_bool_attr_emits_ternary_string():
    body = gen.emit_xml_passthrough_body(["limited"], _rules("limited", "bool"), element_name="thing")
    assert 'TEXT("true") : TEXT("false")' in body
    assert '" limited=\\"%s\\""' in body


def test_tarray_float_emits_space_separated_loop():
    body = gen.emit_xml_passthrough_body(["range"], _rules("range", "TArray<float>"), element_name="thing")
    assert "if (bOverride_range && range.Num() > 0)" in body
    assert "for (int32 i = 0; i < range.Num(); ++i)" in body
    assert 'if (i > 0) Out += TEXT(" ");' in body


def test_flinearcolor_emits_four_component_format():
    body = gen.emit_xml_passthrough_body(["rgba"], _rules("rgba", "FLinearColor"), element_name="thing")
    assert "rgba.R" in body and "rgba.G" in body and "rgba.B" in body and "rgba.A" in body
    assert '" rgba=\\"%f %f %f %f\\""' in body


def test_name_attr_is_skipped_by_caller_owned_logic():
    """The caller handles ``name`` (prefix / MjName logic); the body must
    not duplicate it."""
    body = gen.emit_xml_passthrough_body(["name", "stiffness"], _rules("stiffness", "float"), element_name="thing")
    assert '" name=' not in body
    assert "stiffness" in body


def test_global_exclusion_skips_attr():
    rules = {**_rules("stiffness", "float"), "global_exclusions": ["stiffness"]}
    body = gen.emit_xml_passthrough_body(["stiffness", "group"], rules, element_name="thing")
    assert "stiffness" not in body
    # 'group' still passes if it's typed (it isn't here -> falls back to default_type 'float').


def test_unsupported_type_emits_todo_marker():
    """Novel UE types produce a TODO comment so a reviewer sees the gap
    instead of a silent missing attr."""
    body = gen.emit_xml_passthrough_body(["mystery"], _rules("mystery", "FQuat"), element_name="thing")
    assert "TODO(xml_passthrough)" in body
    assert '"mystery"' in body and "FQuat" in body
