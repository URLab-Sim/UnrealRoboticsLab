# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for mjs_data_packed_attrs emission, including the per-element
``export_op`` unit-conversion path used by mjsEquality.anchor (cm -> m
before packing into data[0..2])."""

from __future__ import annotations

import pytest

from generate_ue_components import _emit_data_packed_export


def test_slot_scalar_no_conversion():
    """Single-slot pack without export_op writes raw scalar."""
    cpp = _emit_data_packed_export(
        attr="torquescale",
        packed_def={
            "slot": 10,
            "condition": "EqualityType == EMjEqualityType::Weld",
        },
        ue_type="float",
        prop_name="torquescale",
    )
    assert "Element->data[10] = (mjtNum)torquescale" in cpp
    assert "bOverride_torquescale" in cpp
    assert "EqualityType == EMjEqualityType::Weld" in cpp
    # No temp var needed when no conversion
    assert "float V" not in cpp


def test_slot_range_no_conversion():
    """Slot-range pack without export_op writes raw array elements."""
    cpp = _emit_data_packed_export(
        attr="polycoef",
        packed_def={
            "slot_range": [0, 5],
            "condition": "(EqualityType == EMjEqualityType::Joint) || (EqualityType == EMjEqualityType::Tendon)",
        },
        ue_type="TArray<float>",
        prop_name="polycoef",
    )
    assert "Element->data[0 + i] = (mjtNum)polycoef[i]" in cpp
    assert "i < polycoef.Num() && i < 5" in cpp
    # No temp var needed when no conversion
    assert "float V" not in cpp


def test_slot_range_with_cm_to_m():
    """anchor pattern: cm -> m per-element conversion before packing."""
    cpp = _emit_data_packed_export(
        attr="anchor",
        packed_def={
            "slot_range": [0, 3],
            "condition": "(EqualityType == EMjEqualityType::Connect) || (EqualityType == EMjEqualityType::Weld)",
            "export_op": "cm_to_m",
        },
        ue_type="TArray<float>",
        prop_name="anchor",
    )
    # Temp var used to apply conversion before write
    assert "float V = anchor[i]" in cpp
    assert "V *= 0.01f" in cpp
    assert "Element->data[0 + i] = (mjtNum)V" in cpp
    assert "i < anchor.Num() && i < 3" in cpp


def test_slot_scalar_with_m_to_cm():
    """Scalar slot with export_op — symmetric to the range case."""
    cpp = _emit_data_packed_export(
        attr="z_offset_cm",
        packed_def={
            "slot": 4,
            "export_op": "m_to_cm",
        },
        ue_type="float",
        prop_name="z_offset_cm",
    )
    assert "float V = (float)z_offset_cm" in cpp
    assert "V *= 100.0f" in cpp
    assert "Element->data[4] = (mjtNum)V" in cpp


def test_unknown_export_op_raises():
    with pytest.raises(RuntimeError, match="unknown export_op"):
        _emit_data_packed_export(
            attr="x",
            packed_def={"slot": 0, "export_op": "lightyears_to_furlongs"},
            ue_type="float",
            prop_name="x",
        )


def test_missing_slot_and_slot_range_raises():
    with pytest.raises(RuntimeError, match="must specify either"):
        _emit_data_packed_export(
            attr="x",
            packed_def={"condition": "true"},
            ue_type="float",
            prop_name="x",
        )


def test_slot_range_requires_tarray():
    """slot_range path needs a TArray UPROPERTY — fail loudly on scalar."""
    with pytest.raises(RuntimeError, match="slot_range requires UE TArray"):
        _emit_data_packed_export(
            attr="x",
            packed_def={"slot_range": [0, 3]},
            ue_type="float",
            prop_name="x",
        )
