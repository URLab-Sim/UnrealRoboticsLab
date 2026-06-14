# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the ``emit_enum_switch`` codegen helper.

The helper backs three sites: xml_enum exports (single-line cases +
``default: break;``), objtype dispatch (multi-key groups + multi-line
default), and geom FinalType resolution (single-line cases, no
default). Each shape is locked here so a future tweak to the helper
can't silently break byte-identity on the emitted .cpp."""

from __future__ import annotations

import generate_ue_components as gen


def test_single_line_case_shape_matches_xml_enum_export():
    """Single ``str`` member → one-line ``case Enum::M: <body>; break;``
    — the shape every xml_enum_export emits."""
    out = gen.emit_enum_switch(
        "JointType", "EMjJointType",
        [
            ("Hinge", "Element->type = (mjtJoint)mjJNT_HINGE"),
            ("Slide", "Element->type = (mjtJoint)mjJNT_SLIDE"),
        ],
        indent="    ",
        default_break=True,
    )
    expected = (
        "    switch (JointType)\n"
        "    {\n"
        "        case EMjJointType::Hinge: Element->type = (mjtJoint)mjJNT_HINGE; break;\n"
        "        case EMjJointType::Slide: Element->type = (mjtJoint)mjJNT_SLIDE; break;\n"
        "        default: break;\n"
        "    }\n"
    )
    assert out == expected


def test_multi_key_group_stacks_labels_with_multiline_body():
    """A list-of-strings member stacks case labels and emits body +
    break on indented separate lines — the shape objtype_dispatch
    needs for ``case Body:`` / ``case XBody:`` collapsing to one expr."""
    out = gen.emit_enum_switch(
        "ObjType", "EMjObjType",
        [
            (["Body", "XBody"], "Target = mjOBJ_BODY"),
            ("Geom",            "Target = mjOBJ_GEOM"),
        ],
        indent="    ",
        default_body="Target = mjOBJ_UNKNOWN",
    )
    expected = (
        "    switch (ObjType)\n"
        "    {\n"
        "        case EMjObjType::Body:\n"
        "        case EMjObjType::XBody:\n"
        "            Target = mjOBJ_BODY;\n"
        "            break;\n"
        "        case EMjObjType::Geom: Target = mjOBJ_GEOM; break;\n"
        "        default:\n"
        "            Target = mjOBJ_UNKNOWN;\n"
        "            break;\n"
        "    }\n"
    )
    assert out == expected


def test_no_default_when_neither_flag_set():
    """geom_final_type emits no default branch — the helper must
    omit it cleanly."""
    out = gen.emit_enum_switch(
        "Type", "EMjGeomType",
        [("Sphere", "FinalType = mjGEOM_SPHERE")],
        indent="        ",
    )
    expected = (
        "        switch (Type)\n"
        "        {\n"
        "            case EMjGeomType::Sphere: FinalType = mjGEOM_SPHERE; break;\n"
        "        }\n"
    )
    assert out == expected


def test_default_body_takes_priority_over_default_break():
    """When both are set, the multi-line default wins. Keeps caller
    intent obvious — ``default_break`` is the simple short-form."""
    out = gen.emit_enum_switch(
        "X", "EMjX",
        [("A", "y = 1")],
        default_break=True,
        default_body="y = 0",
    )
    assert "default:\n" in out
    assert "default: break;" not in out


def test_indent_propagates_to_case_labels_and_body():
    """Case labels sit at ``indent + 4 spaces``; multi-line case bodies
    at ``indent + 8 spaces``. The lock prevents indentation drift."""
    out = gen.emit_enum_switch(
        "X", "EMjX",
        [(["A", "B"], "z = 1")],
        indent="            ",  # 12 spaces
    )
    # switch line at 12 spaces.
    assert out.startswith("            switch (X)\n")
    # case labels at 12 + 4 = 16 spaces.
    assert "                case EMjX::A:\n" in out
    # body at 12 + 8 = 20 spaces.
    assert "                    z = 1;\n" in out
