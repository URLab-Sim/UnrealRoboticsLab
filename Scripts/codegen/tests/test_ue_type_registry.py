# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks the ``_UE_TYPE_INFO`` registry as the single source of truth
for every UE type codegen emits. Prevents the previous failure mode
where ``_attr_default_value`` / ``_UE_TYPE_FAMILY`` / ``Units``-meta
gates each declared their own type list and drifted apart."""

from __future__ import annotations

from _codegen_core import (
    _UE_TYPE_INFO, _UE_TYPE_FAMILY, _UE_TYPES_ACCEPTING_UNITS_META,
    _attr_default_value, _ue_type_accepts_units_meta,
)


def test_ue_type_family_is_view_over_registry():
    """``_UE_TYPE_FAMILY`` (drift-check input) must be a faithful view
    over ``_UE_TYPE_INFO`` — same keys, same shape values."""
    assert set(_UE_TYPE_FAMILY.keys()) == set(_UE_TYPE_INFO.keys())
    for k, info in _UE_TYPE_INFO.items():
        assert _UE_TYPE_FAMILY[k] == info.shape, k


def test_units_meta_set_is_view_over_registry():
    expected = {k for k, v in _UE_TYPE_INFO.items() if v.accepts_units_meta}
    assert _UE_TYPES_ACCEPTING_UNITS_META == expected


def test_attr_default_value_uses_registry():
    for ue_type, info in _UE_TYPE_INFO.items():
        assert _attr_default_value(ue_type) == info.default_init, ue_type


def test_attr_default_value_falls_back_for_novel_tarray():
    """Novel TArray<...> shapes (e.g. TArray<UObject*>) the registry
    doesn't enumerate still default-construct empty — the catch-all
    stays load-bearing for forward compatibility."""
    assert _attr_default_value("TArray<UMjFoo*>") == "{}"


def test_ue_type_accepts_units_meta_matches_registry():
    for ue_type, info in _UE_TYPE_INFO.items():
        assert _ue_type_accepts_units_meta(ue_type) == info.accepts_units_meta, ue_type
    # Unknown types must default to "no Units meta" — UHT would reject.
    assert _ue_type_accepts_units_meta("UMjFoo*") is False
