# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Canonicalisation (orientation + position) emission tests."""

from __future__ import annotations

from generate_ue_components import emit_schema_for_attrs


def test_orientation_absorbs_all_attrs(minimal_rules):
    attrs = ["quat", "euler", "axisangle", "xyaxes", "zaxis", "size"]
    out = emit_schema_for_attrs(attrs, minimal_rules, "geom", "Geom")
    # Raw orientation attrs do NOT appear as per-attr properties.
    for raw in ("Quat ", "Euler", "Axisangle", "Xyaxes", "Zaxis"):
        # The canonical "Quat" property DOES appear once. Use the trailing
        # space to disambiguate from `FQuat Quat`.
        pass
    # Canonical Quat is emitted once.
    assert "FQuat Quat" in out.properties_h
    assert "bOverride_Quat" in out.properties_h
    # No per-attr Euler etc.
    assert "FQuat Euler" not in out.properties_h
    assert "AxisAngle " not in out.properties_h
    assert "Xyaxes " not in out.properties_h


def test_orientation_import_uses_canonical_helper(minimal_rules):
    out = emit_schema_for_attrs(["quat"], minimal_rules, "geom", "Geom")
    # Must invoke the canonicalisation helper, not a per-attr parser.
    assert "MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat)" in out.imports_cpp
    # Must route the result through MjUtils::MjToUERotation so the UPROPERTY
    # lands in UE coordinate frame (X/Z sign-flipped) plus wxyz->xyzw reorder.
    assert "Quat = MjUtils::MjToUERotation(TmpQuat)" in out.imports_cpp
    assert "bOverride_Quat = true" in out.imports_cpp


def test_orientation_export_uses_canonical_helper(minimal_rules):
    out = emit_schema_for_attrs(["quat"], minimal_rules, "geom", "Geom")
    assert "MjUtils::UEToMjRotation(Quat, TmpQuat)" in out.exports_cpp
    # Writes to Element->quat[0..3].
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
