# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Canonicalisation (orientation + position) emission tests."""

from __future__ import annotations

from generate_ue_components import emit_schema_for_attrs


def test_orientation_absorbs_all_attrs(minimal_rules):
    """Every orientation sibling (euler / axisangle / xyaxes / zaxis)
    is absorbed into the canonical Quat UPROPERTY — none should leak
    out as a standalone property."""
    attrs = ["quat", "euler", "axisangle", "xyaxes", "zaxis", "size"]
    out = emit_schema_for_attrs(attrs, minimal_rules, "geom", "Geom")
    assert "FQuat Quat" in out.properties_h
    assert "bOverride_Quat" in out.properties_h
    assert "FQuat Euler" not in out.properties_h
    assert "AxisAngle " not in out.properties_h
    assert "Xyaxes " not in out.properties_h


def test_orientation_roundtrips_through_canon_helpers(minimal_rules):
    """A single ``quat`` schema attr drives both arms of the canon — the
    import pulls through ``MjOrientationUtils::OrientationToMjQuat`` +
    ``MjToUERotation``, and the export pushes through
    ``UEToMjRotation``."""
    out = emit_schema_for_attrs(["quat"], minimal_rules, "geom", "Geom")
    # Import arm.
    assert "MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat)" in out.imports_cpp
    assert "Quat = MjUtils::MjToUERotation(TmpQuat)" in out.imports_cpp
    assert "bOverride_Quat = true" in out.imports_cpp
    # Export arm.
    assert "MjUtils::UEToMjRotation(Quat, TmpQuat)" in out.exports_cpp
    assert "Element->quat[0] = TmpQuat[0]" in out.exports_cpp
    assert "Element->quat[3] = TmpQuat[3]" in out.exports_cpp


def test_position_absorbs_pos(minimal_rules):
    out = emit_schema_for_attrs(["pos", "stiffness"], minimal_rules, "geom", "Geom")
    # Canonical Pos is emitted; raw pos attr is not.
    assert "FVector Pos" in out.properties_h
    # The non-canonical attr survives.
    assert "stiffness" in out.properties_h
    # Import + export route through the helpers.
    assert "MjUtils::ReadVec3InMeters(Node, TEXT(\"pos\"), Pos, bOverride_Pos)" in out.imports_cpp
    assert "MjUtils::UEToMjPosition(Pos, TmpPos)" in out.exports_cpp
    assert "Element->pos[0] = TmpPos[0]" in out.exports_cpp


def test_no_canonicalization_when_element_opts_out(minimal_rules):
    # 'joint' has no applies_canonicalizations in minimal_rules — make sure
    # nothing canonical leaks in.
    out = emit_schema_for_attrs(["pos", "axis", "stiffness"], minimal_rules, "joint", "Joint")
    assert "FQuat Quat" not in out.properties_h
    assert "FVector Pos" not in out.properties_h
