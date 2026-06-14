# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Behavioural lock-ins for the codegen's emit surface.

Substring-only assertions catch typos but not semantics — these tests
pin three load-bearing properties of the generated C++:

1. XML enum roundtrip — every ``value_map`` entry produces matching
   import + export arms whose XML strings and UE members agree.
2. ``mjs_setTo*`` argument order — the codegen feeds params to the
   call in the exact order declared by the C signature in the
   snapshot, not by rule-side iteration order. Misordered args here
   would call ``mjs_setToPosition(kp_value, kv_value, ...)`` swapped
   and silently miscalibrate every Position actuator.
3. Override-toggle gate — every codegen-emitted UPROPERTY pair is
   ``bOverride_X`` (toggle) + ``X`` (value). Both names must agree
   so the editor InlineEditConditionToggle wiring binds correctly.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)

import re

import generate_ue_components as gen


# ---------- xml_enum roundtrip ---------------------------------------------

def test_xml_enum_roundtrip_arms_match(real_rules):
    """Every value_map entry must be reachable from both directions:
    ``XML string -> UE member`` (import) and ``UE member -> XML
    string`` (export). A typo in either arm would silently misimport
    or misexport the attr without a compile error."""
    for elem, elem_rule in real_rules.get("element_rules", {}).items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            value_map = enum_def.get("value_map") or {}
            if not value_map:
                continue
            ue_enum_type = enum_def["ue_enum_type"]
            import_cpp = gen._emit_xml_enum_import(attr, enum_def)
            # Enums with no ``mjs_field`` (e.g. dcmotor.input, routed
            # through mjs_setToDCMotor) emit nothing on the export side
            # by design — skip the export arm in that case.
            export_cpp = (
                gen._emit_xml_enum_export(attr, enum_def)
                if enum_def.get("mjs_field") else ""
            )
            for xml_val, mapping in value_map.items():
                ue_member = mapping[0] if isinstance(mapping, list) else mapping
                # Import side mentions both XML literal and UE member.
                assert f'"{xml_val}"' in import_cpp, (
                    f"{elem}.{attr}: import arm missing XML literal "
                    f"'{xml_val}'\n{import_cpp}"
                )
                assert f"{ue_enum_type}::{ue_member}" in import_cpp, (
                    f"{elem}.{attr}: import arm missing "
                    f"{ue_enum_type}::{ue_member}\n{import_cpp}"
                )
                if export_cpp:
                    assert f"{ue_enum_type}::{ue_member}" in export_cpp, (
                        f"{elem}.{attr}: export arm missing "
                        f"{ue_enum_type}::{ue_member}\n{export_cpp}"
                    )


# ---------- mjs_setTo* arg order -------------------------------------------

def test_setto_call_args_follow_c_signature_order():
    """The generated ``mjs_setToX(Element, a, b, c)`` call must list
    args in the order declared by the C signature. Reordering by
    rule-side iteration would silently swap kp/kv on actuator
    subtypes."""
    rules = {
        "default_type": "float",
        "type_mappings": {"kp": "float", "kv": "float", "dampratio": "float"},
        "setto_param_defaults": {},
    }
    mjspec = {
        "setto_functions": {
            "mjs_setToPosition": {
                "params": [
                    {"name": "kp",        "c_type": "double", "array_dim": None},
                    {"name": "kv",        "c_type": "double", "array_dim": None},
                    {"name": "dampratio", "c_type": "double", "array_dim": None},
                ],
            },
        },
    }
    setto_rules = {"call": "mjs_setToPosition"}
    out = gen._emit_setto_call(
        subtype_key="position",
        setto_rules=setto_rules,
        rules=rules,
        mjspec=mjspec,
        subtype_schema_attrs=["kp", "kv", "dampratio"],
        base_schema_attrs=[],
    )
    assert "mjs_setToPosition(Element" in out
    # Capture the argument list and confirm kp appears before kv before
    # dampratio — exact textual order in the call.
    match = re.search(r"mjs_setToPosition\(Element,(.*?)\);", out, re.DOTALL)
    assert match is not None, out
    args_blob = match.group(1)
    kp_pos        = args_blob.find("(double)kp")
    kv_pos        = args_blob.find("(double)kv")
    dampratio_pos = args_blob.find("(double)dampratio")
    assert 0 < kp_pos < kv_pos < dampratio_pos, (
        f"setto args out of order: kp@{kp_pos} kv@{kv_pos} "
        f"dampratio@{dampratio_pos}\n{out}"
    )


def test_setto_call_respects_signature_order_when_rule_lists_differently():
    """Rule-side ``param_renames`` lookup order MUST NOT influence the
    emitted arg order — the C signature is the source of truth."""
    rules = {
        "default_type": "float",
        "type_mappings": {"PropB": "float", "PropA": "float"},
        "setto_param_defaults": {},
    }
    mjspec = {
        "setto_functions": {
            "mjs_setToFake": {
                "params": [
                    {"name": "a", "c_type": "double", "array_dim": None},
                    {"name": "b", "c_type": "double", "array_dim": None},
                ],
            },
        },
    }
    # param_renames iteration order intentionally NOT matching signature order.
    setto_rules = {"call": "mjs_setToFake",
                   "param_renames": {"b": "PropB", "a": "PropA"}}
    out = gen._emit_setto_call(
        subtype_key="fake",
        setto_rules=setto_rules,
        rules=rules,
        mjspec=mjspec,
        subtype_schema_attrs=["PropA", "PropB"],
        base_schema_attrs=[],
    )
    match = re.search(r"mjs_setToFake\(Element,(.*?)\);", out, re.DOTALL)
    assert match is not None, out
    args_blob = match.group(1)
    a_pos = args_blob.find("PropA")
    b_pos = args_blob.find("PropB")
    assert 0 < a_pos < b_pos, (
        f"renamed setto args out of signature order: A@{a_pos} B@{b_pos}\n{out}"
    )


# ---------- override-toggle gate -------------------------------------------

_UPROPERTY_PAIR_RE = re.compile(
    r"bool\s+(bOverride_(\w+))\s*=\s*false;\s*\n+"
    r"\s*UPROPERTY\(.*?\)\)\s*\n"
    r"\s*\S+\s+(\w+)\s*="
)


def test_override_toggle_name_matches_uproperty_for_all_categories(real_rules):
    """For every category whose schema produces a UPROPERTY pair, the
    ``bOverride_X`` toggle and the value property ``X`` must share the
    same final identifier. A drift between them would still compile
    but the InlineEditConditionToggle wiring in the editor would
    silently dead-end."""
    schema_path = os.path.join(_CODEGEN_DIR, "snapshots", "mjcf_schema_snapshot.json")
    if not os.path.exists(schema_path):
        import pytest
        pytest.skip("mjcf_schema.json snapshot not present")
    import json
    with open(schema_path, "r", encoding="utf-8") as f:
        schema = json.load(f)
    saw_pair = False
    for cat_name, cat_rules in real_rules.get("categories", {}).items():
        attrs = gen.schema_attrs(schema, cat_rules.get("schema_common_block", ""))
        if not attrs:
            continue
        out = gen.emit_schema_for_attrs(
            attrs, real_rules,
            element_name=cat_name,
            category_label=cat_name,
        )
        for match in _UPROPERTY_PAIR_RE.finditer(out.properties_h):
            full_toggle, toggle_suffix, value_name = match.groups()
            assert toggle_suffix == value_name, (
                f"category '{cat_name}': UPROPERTY pair drift — toggle "
                f"'{full_toggle}' but value name is '{value_name}'"
            )
            saw_pair = True
    assert saw_pair, "no UPROPERTY pairs found in any category — check broken"
