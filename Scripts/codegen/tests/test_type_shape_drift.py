# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the type-shape reconcile + new-attr fail-loud checks.

Covers ``_classify_c_type`` (C-type kind classifier), ``_shapes_compatible``
(UE-vs-C shape gate), ``_check_type_shape_drift`` (rule-vs-introspect
cross-check), and ``_check_new_attr_typing`` (every schema attr must
be typed or explicitly default-acknowledged)."""

from __future__ import annotations

import generate_ue_components as gen
import _codegen_core as _core
import _codegen_checks as _checks

# Stitch test-only re-exports onto `gen` so the assertions below keep
# their compact ``gen._classify_c_type`` form. The orchestrator's import
# block intentionally no longer carries these — they're private to
# _codegen_core / _codegen_checks.
gen._classify_c_type = _core._classify_c_type
gen._shapes_compatible = _core._shapes_compatible
gen._check_type_shape_drift = _checks._check_type_shape_drift
gen._check_new_attr_typing = _checks._check_new_attr_typing


# ---------- _classify_c_type ------------------------------------------------

def test_classify_c_type_scalar_num():
    assert gen._classify_c_type("double", None) == ("scalar", "num")
    assert gen._classify_c_type("mjtNum", None) == ("scalar", "num")
    assert gen._classify_c_type("float", 1) == ("scalar", "num")


def test_classify_c_type_vec_dim():
    assert gen._classify_c_type("double [3]", 3) == ("vec", 3)
    assert gen._classify_c_type("mjtNum [4]", 4) == ("vec", 4)


def test_classify_c_type_dynamic_vec_pointer():
    """mjFloatVec* / mjDoubleVec* are dynamic-length vectors, NOT
    opaque pointers — must classify as ('array', None) so TArray<X>
    rules match cleanly."""
    assert gen._classify_c_type("mjDoubleVec *", None) == ("array", None)
    assert gen._classify_c_type("mjFloatVec *", None) == ("array", None)


def test_classify_c_type_opaque_element_pointer():
    """mjsElement* is the parent-spec back-pointer URLab never models;
    classify as opaque so the shape check skips it."""
    assert gen._classify_c_type("mjsElement *", None) == ("opaque", None)


def test_classify_c_type_string_pointer():
    assert gen._classify_c_type("mjString", None) == ("scalar", "string")
    assert gen._classify_c_type("char *", None) == ("scalar", "string")


# ---------- _shapes_compatible ----------------------------------------------

def test_shapes_compatible_array_matches_vec():
    """TArray<float> at the UE layer is the universal carrier for
    fixed-size C arrays — codegen writes prop.Num() elements and the
    common case is len(prop) <= C cap."""
    assert gen._shapes_compatible(("array", None), ("vec", 3))
    assert gen._shapes_compatible(("array", None), ("array", None))


def test_shapes_compatible_scalar_vs_array_is_mismatch():
    assert not gen._shapes_compatible(("scalar", "num"), ("vec", 3))
    assert not gen._shapes_compatible(("array", None), ("scalar", "num"))
    assert not gen._shapes_compatible(("vec", 3), ("vec", 4))


# ---------- _check_type_shape_drift ----------------------------------------

def test_type_shape_drift_flags_scalar_vs_array_mismatch():
    """Rule says TArray<float> but the introspect struct field is a
    scalar mjtNum — diagnostic fires; emitted code would not compile
    correctly."""
    rules = {
        "type_mappings": {"stiffness": "TArray<float>"},
        "categories": {
            "joint": {"mjs_struct": "mjsJoint"},
        },
    }
    mjspec = {
        "introspect": {
            "structs": {
                "mjsJoint": {
                    "fields": [
                        {"name": "stiffness", "c_type": "double",
                         "array_dim": None, "is_pointer": False},
                    ],
                },
            },
        },
    }
    gen._check_type_shape_drift({}, rules, mjspec)
    assert any("stiffness" in d.message and "mismatch" in d.message
               for d in gen._DIAGS_BUFFER.pending)


def test_type_shape_drift_quiet_on_unmodeled_field():
    """A field listed in intentionally_unmodeled_mjs_fields is not
    emitted by codegen — drift check must NOT fire on it."""
    rules = {
        "type_mappings": {"damping": "TArray<float>"},
        "categories": {
            "flexcomp": {"mjs_struct": "mjsFlex"},
        },
        "intentionally_unmodeled_mjs_fields": {
            "mjsFlex": ["damping"],
        },
    }
    mjspec = {
        "introspect": {
            "structs": {
                "mjsFlex": {
                    "fields": [
                        {"name": "damping", "c_type": "double",
                         "array_dim": None, "is_pointer": False},
                    ],
                },
            },
        },
    }
    gen._check_type_shape_drift({}, rules, mjspec)
    assert len(gen._DIAGS_BUFFER.pending) == 0


# ---------- _check_new_attr_typing -----------------------------------------

def test_new_attr_typing_flags_unmapped_attr():
    schema = {
        "joint": {"attrs": ["stiffness", "novel_attr"]},
    }
    rules = {
        "type_mappings": {"stiffness": "TArray<float>"},
        "default_type": "float",
        "global_exclusions": [],
        "intentionally_default_typed_attrs": [],
        "element_rules": {"joint": {}},
        "canonicalizations": {},
        "categories": {
            "joint": {"schema_common_block": "joint.attrs"},
        },
    }
    gen._check_new_attr_typing(schema, rules)
    msgs = [d.message for d in gen._DIAGS_BUFFER.pending]
    assert any("novel_attr" in m and "joint" in m for m in msgs), msgs
    # stiffness is typed -> no diag.
    assert not any("'stiffness'" in m for m in msgs)


def test_new_attr_typing_respects_intentional_default():
    schema = {"sensor": {"attrs": ["cutoff"]}}
    rules = {
        "type_mappings": {},
        "default_type": "float",
        "global_exclusions": [],
        "intentionally_default_typed_attrs": ["cutoff"],
        "element_rules": {"sensor": {}},
        "canonicalizations": {},
        "categories": {
            "sensor": {"schema_common_block": "sensor.attrs"},
        },
    }
    gen._check_new_attr_typing(schema, rules)
    assert len(gen._DIAGS_BUFFER.pending) == 0
