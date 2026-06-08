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


def test_position_uses_scalar_in_buffer_and_inheritrange_zero_sentinel(real_rules, real_schema):
    """Position's kv/dampratio/timeconst are double[1] in C; UE side is float
    (kv, dampratio) or TArray<float> (timeconst). All three become 1-elem buffers.
    inheritrange uses the 0.0 sentinel (per setto_param_defaults), NOT -1.0."""
    mjspec = _load_mjspec()
    out = _setto_for("position", real_rules, real_schema, mjspec)
    assert "double kvBuf[1] = { bOverride_kv ? (double)kv : -1.0 };" in out
    assert "double dampratioBuf[1] = { bOverride_dampratio ? (double)dampratio : -1.0 };" in out
    # TArray UE side -> .Num() guard with [0] indexing.
    assert "(bOverride_timeconst && timeconst.Num() > 0) ? (double)timeconst[0] : -1.0" in out
    # inheritrange override sentinel from setto_param_defaults.
    assert "bOverride_inheritrange ? (double)inheritrange : 0.0" in out
    # mjs_setToPosition uses the buffers, not raw scalars.
    assert "mjs_setToPosition(Element," in out


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


def test_intvelocity_drops_uninvolved_params_to_sentinels(real_rules, real_schema):
    """IntVelocity exposes only kp/kv/dampratio in the schema. The C signature
    still requires timeconst and inheritrange — they should default to
    sentinel values via the 'param not in schema -> force sentinel' rule."""
    mjspec = _load_mjspec()
    out = _setto_for("intvelocity", real_rules, real_schema, mjspec)
    assert "double kvBuf[1]" in out
    # timeconst not in intvelocity schema -> forced sentinel buffer.
    assert "double timeconstBuf[1] = { -1.0 }" in out
    # inheritrange not in intvelocity schema -> default override per setto_param_defaults (0.0).
    assert "0.0);" in out or "0.0)" in out


def test_cylinder_timeconst_scalar_projection(real_rules, real_schema):
    """Cylinder's timeconst C param is scalar (not [1]). UE side is
    TArray<float>. The emitter must project to element [0]."""
    mjspec = _load_mjspec()
    out = _setto_for("cylinder", real_rules, real_schema, mjspec)
    assert "(bOverride_timeconst && timeconst.Num() > 0) ? (double)timeconst[0] : -1.0" in out
