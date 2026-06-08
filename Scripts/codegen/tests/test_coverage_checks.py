# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Behavioural coverage tests for the six drift checks that previously
had no direct test — every check now has a "should fire" path and a
"should be quiet" path. Each check fires through ``_diag_add`` with a
specific ``source=`` tag so assertions can target the message bucket
without depending on prose."""

from __future__ import annotations

import _codegen_checks as checks
import generate_ue_components as gen


def _pending_sources() -> list:
    return [d.source for d in gen._DIAGS_BUFFER.pending]


def _reset() -> None:
    gen._reset_diags()


# ---------- _check_top_level_elements_coverage --------------------------

def test_top_level_elements_coverage_fires_for_unmodeled_element():
    _reset()
    schema = {"new_widget": {}, "joint": {}}
    rules = {
        "categories": {"joint": {"schema_common_block": "joint"}},
        "intentionally_unmodeled_elements": {},
    }
    checks._check_top_level_elements_coverage(schema, rules)
    assert any("new_widget" in d.message and d.source == "top_level_coverage"
               for d in gen._DIAGS_BUFFER.pending)


def test_top_level_elements_coverage_quiet_for_documented_element():
    _reset()
    schema = {"asset": {}, "joint": {}}
    rules = {
        "categories": {"joint": {"schema_common_block": "joint"}},
        "intentionally_unmodeled_elements": {},
    }
    # 'asset' is in the container_keys allowlist.
    checks._check_top_level_elements_coverage(schema, rules)
    assert "top_level_coverage" not in _pending_sources()


# ---------- _check_actuator_subtypes_coverage --------------------------

def test_actuator_subtypes_coverage_fires_for_unmodeled_subtype():
    _reset()
    schema = {"actuator_types": {"newkind": []}}
    rules = {"categories": {"actuator": {
        "subtypes": [{"key": "motor"}, {"key": "position"}],
    }}}
    checks._check_actuator_subtypes_coverage(schema, rules)
    assert any("newkind" in d.message and d.source == "actuator_subtypes_coverage"
               for d in gen._DIAGS_BUFFER.pending)


def test_actuator_subtypes_coverage_quiet_when_all_modeled():
    _reset()
    schema = {"actuator_types": {"motor": []}}
    rules = {"categories": {"actuator": {
        "subtypes": [{"key": "motor"}],
    }}}
    checks._check_actuator_subtypes_coverage(schema, rules)
    assert "actuator_subtypes_coverage" not in _pending_sources()


# ---------- _check_sensor_subtypes_coverage ----------------------------

def test_sensor_subtypes_coverage_fires_for_unmodeled_subtype():
    _reset()
    schema = {"sensor_types": ["touch", "newsensor"]}
    rules = {"categories": {"sensor": {
        "subtypes": [{"key": "touch"}],
    }}}
    checks._check_sensor_subtypes_coverage(schema, rules)
    assert any("newsensor" in d.message and d.source == "sensor_subtypes_coverage"
               for d in gen._DIAGS_BUFFER.pending)


def test_sensor_subtypes_coverage_quiet_when_all_modeled():
    _reset()
    schema = {"sensor_types": ["touch"]}
    rules = {"categories": {"sensor": {
        "subtypes": [{"key": "touch"}],
    }}}
    checks._check_sensor_subtypes_coverage(schema, rules)
    assert "sensor_subtypes_coverage" not in _pending_sources()


# ---------- _check_setto_param_coverage --------------------------------

def test_setto_param_coverage_fires_for_unmapped_param():
    _reset()
    schema = {"actuator_common": {"attrs": ["kp"]}, "actuator_types": {"position": ["kv"]}}
    rules = {
        "categories": {"actuator": {
            "schema_common_block": "actuator_common",
            "subtype_setto": {"position": {"call": "mjs_setToPosition"}},
        }},
        "setto_param_defaults": {},
    }
    mjspec = {"setto_functions": {"mjs_setToPosition": {
        "params": [
            {"name": "kp",        "c_type": "double", "array_dim": None},
            {"name": "kv",        "c_type": "double", "array_dim": None},
            {"name": "noveltie",  "c_type": "double", "array_dim": None},
        ],
    }}}
    checks._check_setto_param_coverage(schema, rules, mjspec)
    assert any("noveltie" in d.message and d.source == "setto_param_coverage"
               for d in gen._DIAGS_BUFFER.pending)


def test_setto_param_coverage_quiet_when_all_mapped():
    _reset()
    schema = {"actuator_common": {"attrs": ["kp"]}, "actuator_types": {"position": ["kv"]}}
    rules = {
        "categories": {"actuator": {
            "schema_common_block": "actuator_common",
            "subtype_setto": {"position": {"call": "mjs_setToPosition"}},
        }},
        "setto_param_defaults": {"mjs_setToPosition": {"inheritrange": "0.0"}},
    }
    mjspec = {"setto_functions": {"mjs_setToPosition": {
        "params": [
            {"name": "kp",           "c_type": "double", "array_dim": None},
            {"name": "kv",           "c_type": "double", "array_dim": None},
            {"name": "inheritrange", "c_type": "double", "array_dim": None},
        ],
    }}}
    checks._check_setto_param_coverage(schema, rules, mjspec)
    assert "setto_param_coverage" not in _pending_sources()


# ---------- _check_mjs_struct_field_drift ------------------------------

def test_mjs_struct_field_drift_fires_for_uncovered_field():
    _reset()
    schema = {"joint_common": ["stiffness"]}
    rules = {
        "categories": {"joint": {
            "schema_common_block": "joint_common",
            "mjs_struct": "mjsJoint",
        }},
        "intentionally_unmodeled_mjs_fields": {},
    }
    mjspec = {"structs": {"mjsJoint": ["stiffness", "mystery_field"]}}
    checks._check_mjs_struct_field_drift(schema, rules, mjspec)
    assert any("mystery_field" in d.message and d.source == "mjs_field_drift"
               for d in gen._DIAGS_BUFFER.pending)


def test_mjs_struct_field_drift_quiet_when_all_modeled():
    _reset()
    schema = {"joint_common": ["stiffness"]}
    rules = {
        "categories": {"joint": {
            "schema_common_block": "joint_common",
            "mjs_struct": "mjsJoint",
        }},
        "intentionally_unmodeled_mjs_fields": {},
    }
    mjspec = {"structs": {"mjsJoint": ["stiffness", "name"]}}  # 'name' is structural
    checks._check_mjs_struct_field_drift(schema, rules, mjspec)
    assert "mjs_field_drift" not in _pending_sources()


# ---------- _check_generated_enum_coverage -----------------------------

def test_generated_enum_coverage_fires_for_unmapped_member():
    _reset()
    rules = {"generated_enums": {"MjOptionEnums": {
        "enums": [{
            "ue_name": "EMjIntegrator",
            "from_mj_enum": "mjtIntegrator",
            "ue_member_from_mj": {"mjINT_EULER": "Euler"},
            # mjINT_RK4 missing from the rule -> should diag.
        }],
    }}}
    mjspec = {"enums": {"mjtIntegrator": {"mjINT_EULER": 0, "mjINT_RK4": 1}}}
    checks._check_generated_enum_coverage(rules, mjspec)
    assert any("mjINT_RK4" in d.message and d.source == "enum_drift"
               for d in gen._DIAGS_BUFFER.pending)


def test_generated_enum_coverage_quiet_when_all_mapped():
    _reset()
    rules = {"generated_enums": {"MjOptionEnums": {
        "enums": [{
            "ue_name": "EMjIntegrator",
            "from_mj_enum": "mjtIntegrator",
            "ue_member_from_mj": {"mjINT_EULER": "Euler", "mjINT_RK4": "RK4"},
        }],
    }}}
    mjspec = {"enums": {"mjtIntegrator": {"mjINT_EULER": 0, "mjINT_RK4": 1}}}
    checks._check_generated_enum_coverage(rules, mjspec)
    assert "enum_drift" not in _pending_sources()


def test_generated_enum_coverage_fires_when_from_mj_enum_unknown():
    """Snapshot drift: a from_mj_enum that's not in the projected
    mjspec must produce a diagnostic instead of a silent skip."""
    _reset()
    rules = {"generated_enums": {"MjOptionEnums": {
        "enums": [{
            "ue_name": "EMjGone",
            "from_mj_enum": "mjtGoneRenamed",
            "ue_member_from_mj": {"mjGONE_X": "X"},
        }],
    }}}
    mjspec = {"enums": {"mjtIntegrator": {"mjINT_EULER": 0}}}
    checks._check_generated_enum_coverage(rules, mjspec)
    assert any("mjtGoneRenamed" in d.message and d.source == "enum_drift"
               for d in gen._DIAGS_BUFFER.pending)
