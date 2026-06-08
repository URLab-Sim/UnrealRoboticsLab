# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Residual unit tests for v8/v9 rule shapes that don't have a dedicated
file: ``property_default`` (with value_map cross-check),
``_guarded_export(multiline=...)`` emission shapes, and the full-coverage
drift checks (value_map stale, orphan type_mappings, compiler attrs,
mjxmacro block coverage, mjs_array_caps_for).

Larger concerns moved to dedicated files: see ``test_objtype_dispatch``,
``test_geom_final_type``, ``test_inject_helpers``, ``test_hand_enum_drift``,
``test_type_shape_drift``, ``test_property_units``.
"""

from __future__ import annotations

import generate_ue_components as gen
from generate_ue_components import _guarded_export


# ---------- property_default + cross-check ---------------------------------

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


# ---------- _guarded_export(multiline=True) --------------------------------

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


# ---------- full-coverage drift checks -------------------------------------

def test_value_map_stale_mj_const_flagged():
    """A value_map entry referencing a constant NOT in any mjspec enum
    is stale — surface it before the C++ compile fails."""
    rules = {
        "element_rules": {
            "joint": {
                "xml_enum_attrs": {
                    "type": {
                        "value_map": {
                            "hinge":  ["Hinge",  "mjJNT_HINGE"],
                            "screwd": ["Screwd", "mjJNT_NONEXISTENT"],
                        },
                    },
                },
            },
        },
    }
    mjspec = {"enums": {"mjtJoint": ["mjJNT_HINGE", "mjJNT_SLIDE"]}}
    gen._check_value_map_stale_mj_consts(rules, mjspec)
    assert any("mjJNT_NONEXISTENT" in d.message for d in gen._DIAGS)


def test_orphan_type_mappings_flagged():
    schema = {"joint": {"attrs": ["stiffness"]}}
    rules = {
        "type_mappings": {
            "stiffness": "TArray<float>",
            "vanishedattr": "TArray<float>",
        },
        "categories": {"joint": {"schema_common_block": "joint.attrs"}},
        "element_rules": {},
    }
    gen._check_orphan_rule_entries(schema, rules)
    msgs = [d.message for d in gen._DIAGS]
    assert any("'vanishedattr'" in m for m in msgs)
    assert not any("'stiffness'" in m for m in msgs)


def test_compiler_attrs_coverage_flags_unhandled():
    schema = {"compiler": {"attrs": ["angle", "newflag2030", "eulerseq"]}}
    rules = {
        "compiler_attr_field_map": {"angle": "bAngleInDegrees", "eulerseq": "EulerSeq"},
        "intentionally_unmodeled_compiler_attrs": [],
    }
    gen._check_compiler_attrs_coverage(schema, rules)
    msgs = [d.message for d in gen._DIAGS]
    assert any("newflag2030" in m for m in msgs), msgs
    assert not any("'angle'" in m for m in msgs)


def test_compiler_attrs_coverage_respects_intentional_skip():
    schema = {"compiler": {"attrs": ["angle", "balanceinertia"]}}
    rules = {
        "compiler_attr_field_map": {"angle": "bAngleInDegrees"},
        "intentionally_unmodeled_compiler_attrs": ["balanceinertia"],
    }
    gen._check_compiler_attrs_coverage(schema, rules)
    assert len(gen._DIAGS) == 0


def test_mjxmacro_block_coverage_forward():
    rules = {
        "synthetic_categories": {
            "MjOption": {"mjxmacro_block": "MJOPTION_FIELDS"},
        },
    }
    mjxmacro = {
        "struct_fields": {
            "MJOPTION_FIELDS": [{"name": "timestep"}],
            "MJNEW_FIELDS":    [{"name": "wat"}],  # no rule entry
        },
    }
    gen._check_mjxmacro_block_coverage(rules, mjxmacro)
    msgs = [d.message for d in gen._DIAGS]
    assert any("MJNEW_FIELDS" in m for m in msgs)
    assert not any("MJOPTION_FIELDS" in m for m in msgs)


def test_mjxmacro_block_coverage_reverse():
    rules = {
        "synthetic_categories": {
            "MjGhost": {"mjxmacro_block": "MJGHOST_FIELDS"},
        },
    }
    mjxmacro = {"struct_fields": {}}
    gen._check_mjxmacro_block_coverage(rules, mjxmacro)
    msgs = [d.message for d in gen._DIAGS]
    assert any("MJGHOST_FIELDS" in m and "MjGhost" in m for m in msgs)


def test_mjs_array_caps_for_extracts_fixed_size_arrays():
    cat_rules = {"mjs_struct": "mjsActuator"}
    mjspec = {
        "introspect": {
            "structs": {
                "mjsActuator": {
                    "fields": [
                        {"name": "gainprm", "c_type": "double [10]", "array_dim": 10},
                        {"name": "scalar",  "c_type": "double",      "array_dim": None},
                        {"name": "noslot",  "c_type": "double [1]",  "array_dim": 1},
                    ],
                },
            },
        },
    }
    caps = gen._mjs_array_caps_for(cat_rules, mjspec)
    assert caps == {"gainprm": 10}
