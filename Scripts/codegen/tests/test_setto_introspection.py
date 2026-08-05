# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Regression guard for introspected mjs_setTo* emission.

The 9 hand-written C++ literal blocks that used to live in
``codegen_rules.json`` were replaced by ``_emit_setto_call``, which derives
the call body from the parsed mjs_setTo* signature. The signature now
comes from the clang-AST introspect snapshot, projected into the
legacy mjspec shape via ``_mjspec_from_introspect``."""

from __future__ import annotations

import json
import os

from generate_ue_components import _emit_setto_call, _mjspec_from_introspect


_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))


def _load_mjspec():
    with open(os.path.join(_PLUGIN_ROOT, "Scripts", "codegen", "snapshots", "introspect_snapshot.json"), "r") as f:
        return _mjspec_from_introspect(json.load(f))


def _setto_for(subtype_key, real_rules, real_schema, mjspec):
    """Helper: invoke _emit_setto_call with the rules-real args for one actuator subtype."""
    actuator = real_rules["categories"]["actuator"]
    setto_rules = actuator["subtype_setto"].get(subtype_key)
    if setto_rules is None:
        return ""
    per_type_attrs = real_schema["actuator_types"].get(subtype_key, [])
    base_attrs = real_schema.get("actuator_common", {}).get("attrs", [])
    return _emit_setto_call(
        subtype_key, setto_rules, real_rules, mjspec,
        subtype_schema_attrs=per_type_attrs, base_schema_attrs=base_attrs,
    )


def test_motor_call_minimal(real_rules, real_schema):
    """Motor's setTo has no params besides Element — output is a one-liner."""
    mjspec = _load_mjspec()
    out = _setto_for("motor", real_rules, real_schema, mjspec)
    assert "mjs_setToMotor(Element);" in out


def test_position_inherits_kp_and_nulls_unset_pointers(real_rules, real_schema):
    """Position's kv/dampratio/timeconst are double[1] in C and nullable: the
    buffer is built but passed only when overridden, else nullptr. kp is written
    unconditionally by MuJoCo, so an unset kp re-asserts the inherited element
    gain. inheritrange keeps its 0.0 sentinel (MuJoCo guards it with `> 0`)."""
    mjspec = _load_mjspec()
    out = _setto_for("position", real_rules, real_schema, mjspec)
    assert "double kvBuf[1] = { bOverride_kv ? (double)kv : -1.0 };" in out
    assert "double dampratioBuf[1] = { bOverride_dampratio ? (double)dampratio : -1.0 };" in out
    # kp is unconditional-by-value -> re-assert the inherited gain, not -1.0.
    assert "bOverride_kp ? (double)kp : Element->gainprm[0]" in out
    # Pointer params passed only when authored, else nullptr (MuJoCo's unset
    # convention) so the class default survives.
    assert "bOverride_kv ? kvBuf : nullptr" in out
    assert "bOverride_dampratio ? dampratioBuf : nullptr" in out
    # timeconst is TArray-backed: an authored-but-empty array is unset too.
    assert "(bOverride_timeconst && timeconst.Num() > 0) ? timeconstBuf : nullptr" in out
    # inheritrange override sentinel from setto_param_defaults.
    assert "bOverride_inheritrange ? (double)inheritrange : 0.0" in out
    assert "mjs_setToPosition(Element," in out


def test_unconditional_gain_params_reassert_inherited_field(real_rules, real_schema):
    """velocity/adhesion/damper each write a single gain slot unconditionally.
    An unset param must re-assert the inherited element field so a class default
    (e.g. <velocity kv=...> under a <default class>) is preserved, not clobbered
    to a sentinel. damper's kv maps to -gainprm[2] (setToDamper does gainprm[2]
    = -kv), so the re-assert carries the sign."""
    mjspec = _load_mjspec()
    assert "bOverride_kv ? (double)kv : Element->gainprm[0]" in \
        _setto_for("velocity", real_rules, real_schema, mjspec)
    assert "bOverride_gain ? (double)gain : Element->gainprm[0]" in \
        _setto_for("adhesion", real_rules, real_schema, mjspec)
    assert "bOverride_kv ? (double)kv : -Element->gainprm[2]" in \
        _setto_for("damper", real_rules, real_schema, mjspec)


def test_setto_call_surfaces_error_return(real_rules, real_schema):
    """Every mjs_setTo* return string is checked and logged, not discarded —
    that discarded return is what hid the gain-clobber bug for so long."""
    mjspec = _load_mjspec()
    for sub in ("position", "velocity", "cylinder", "motor"):
        out = _setto_for(sub, real_rules, real_schema, mjspec)
        assert "const char* SetToErr =" in out
        assert "if (SetToErr && *SetToErr)" in out


def test_muscle_scale_projection_and_tausmooth_zero(real_rules, real_schema):
    """Muscle's scale (TArray<float>) is collapsed to a scalar for the
    setTo call — we take element [0]. tausmooth has the 0.0 sentinel
    (so MuJoCo treats unset as 'no smoothing', not 'broken default')."""
    mjspec = _load_mjspec()
    out = _setto_for("muscle", real_rules, real_schema, mjspec)
    assert "(bOverride_scale && scale.Num() > 0) ? (double)scale[0] : -1.0" in out
    assert "bOverride_tausmooth ? (double)tausmooth : 0.0" in out
    assert "double timeconstBuf[2]" in out
    assert "double rangeBuf[2]" in out


def test_dcmotor_nullable_arrays_use_filldouble_and_input_rename(real_rules, real_schema):
    """DCMotor passes nullptr when arrays not overridden; uses FillDouble
    helper. input_mode (C param) -> input (UE prop) rename."""
    mjspec = _load_mjspec()
    out = _setto_for("dcmotor", real_rules, real_schema, mjspec)
    assert "FillDouble(motorconstBuf, motorconst)" in out
    assert "bOverride_motorconst ? motorconstBuf : nullptr" in out
    # input_mode -> input rename: the C arg name in the signature is input_mode,
    # but the UE-side property is `input`. The expression must reference `input`,
    # not `input_mode`.
    assert "bOverride_input ? (int)input : 0" in out
    # resistance has 0.0 sentinel per setto_param_defaults.
    assert "bOverride_resistance ? (double)resistance : 0.0" in out


def test_intvelocity_inherits_kp_and_nulls_unset_pointers(real_rules, real_schema):
    """IntVelocity writes gainprm[0] = kp unconditionally, so an unset kp must
    re-assert the inherited element value, not a sentinel that clobbers it.
    timeconst (not in the intvelocity schema) is a nullable pointer param, so it
    passes nullptr — MuJoCo then leaves the type-derived default alone."""
    mjspec = _load_mjspec()
    out = _setto_for("intvelocity", real_rules, real_schema, mjspec)
    assert "double kvBuf[1]" in out
    # kp is unconditional-by-value -> re-assert the inherited gain, not -1.0.
    assert "bOverride_kp ? (double)kp : Element->gainprm[0]" in out
    # timeconst nullable + not authored -> bare nullptr, no sentinel buffer.
    assert "double timeconstBuf" not in out
    assert "nullptr" in out
    # kv/dampratio are nullable too: buffer built, but passed only when set.
    assert "bOverride_kv ? kvBuf : nullptr" in out


def test_cylinder_reasserts_inherited_gain_fields(real_rules, real_schema):
    """Cylinder writes dynprm[0]/biasprm[0]/gainprm[0] unconditionally, so unset
    timeconst/bias/area re-assert the inherited element field rather than a -1
    sentinel that would clobber the class default. diameter keeps its sentinel
    because MuJoCo guards it with `>= 0`."""
    mjspec = _load_mjspec()
    out = _setto_for("cylinder", real_rules, real_schema, mjspec)
    assert "(bOverride_timeconst && timeconst.Num() > 0) ? (double)timeconst[0] : Element->dynprm[0]" in out
    assert "bOverride_bias ? (double)bias : Element->biasprm[0]" in out
    assert "bOverride_area ? (double)area : Element->gainprm[0]" in out
    assert "bOverride_diameter ? (double)diameter : -1.0" in out
