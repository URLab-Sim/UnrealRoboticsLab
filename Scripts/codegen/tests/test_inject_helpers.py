# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the codegen's inject-guard layer: ``_audit_banner_safety``
on ``fully_emitted`` overwrites, layout dispatcher fall-through,
and canonicalisation dispatch on unknown canon names."""

from __future__ import annotations

import generate_ue_components as gen
from generate_ue_components import _audit_banner_safety


# ---------- banner-safety audit ---------------------------------------------

def test_audit_banner_safety_silent_on_template_only(tmp_path):
    """A subclass file containing only its own header include and a
    bare ``Type = ...`` constructor should not trigger a diagnostic."""
    p = tmp_path / "MjBoxGeom.cpp"
    p.write_text(
        '#include "MuJoCo/Components/Geometry/MjBoxGeom.h"\n'
        "UMjBoxGeom::UMjBoxGeom()\n"
        "{\n"
        "    Type = EMjGeomType::Box;\n"
        "}\n",
        encoding="utf-8",
    )
    subtype = {"class_name": "UMjBoxGeom"}
    _audit_banner_safety(str(p), subtype)
    assert len(gen._DIAGS) == 0


def test_audit_banner_safety_flags_extra_include(tmp_path):
    """An extra non-template #include in a soon-to-be-overwritten file
    triggers a diagnostic so the rule author knows hand-rolled content
    will be discarded."""
    p = tmp_path / "MjBoxGeom.cpp"
    p.write_text(
        '#include "MuJoCo/Components/Geometry/MjBoxGeom.h"\n'
        '#include "SomeOther/Header.h"\n'
        "UMjBoxGeom::UMjBoxGeom()\n"
        "{\n"
        "    Type = EMjGeomType::Box;\n"
        "}\n",
        encoding="utf-8",
    )
    subtype = {"class_name": "UMjBoxGeom"}
    _audit_banner_safety(str(p), subtype)
    assert any("SomeOther/Header.h" in d.message for d in gen._DIAGS)


def test_audit_banner_safety_flags_extra_ctor_body(tmp_path):
    """Constructor body content beyond ``Type = EMj...::X`` and the
    rule-provided ``extra_constructor`` is flagged — that's where the
    joint-subtype bOverride_Type bug came from."""
    p = tmp_path / "MjHingeJoint.cpp"
    p.write_text(
        '#include "MuJoCo/Components/Joints/MjHingeJoint.h"\n'
        "UMjHingeJoint::UMjHingeJoint()\n"
        "{\n"
        "    Type = EMjJointType::Hinge;\n"
        "    HandRolledHelperCall();\n"
        "}\n",
        encoding="utf-8",
    )
    subtype = {
        "class_name": "UMjHingeJoint",
        "extra_constructor": "",
    }
    _audit_banner_safety(str(p), subtype)
    assert any("non-template ctor content" in d.message
               and "HandRolledHelperCall" in d.message
               for d in gen._DIAGS)


# ---------- inject-guard diagnostics ----------------------------------------

def test_layout_dispatch_diagnoses_unknown_layout():
    """`_phase_categories` else-branch fires a diag for typo'd layouts.
    Catches the silent fall-through where mistyped 'single_uclass'
    instead of 'single_uclass_per_file' produces zero output."""
    writes = []
    rules = {
        "categories": {
            "bogus": {"layout": "single_uclass"},
        },
    }
    ctx = gen.PhaseContext(
        schema={}, rules=rules, mjxmacro={}, mjspec={},
        public_root="", private_root="", bind_h_path="", writes=writes,
    )
    gen._phase_categories(ctx)
    assert any("unknown layout" in d.message and "bogus" in d.message
               and "single_uclass" in d.message
               for d in gen._DIAGS)
    assert writes == []


def test_canon_import_block_diagnoses_unknown_canon():
    """An unknown canonicalisation name produces an empty string AND a
    diagnostic — silent skip would let a new canon rule in JSON ship
    UPROPERTYs with no import."""
    out = gen._canon_import_block(
        "made_up_canon",
        {"emits_property": {"name": "P", "type": "FVector"},
         "import_helper": "Foo::Bar"},
    )
    assert out == ""
    assert any("made_up_canon" in d.message for d in gen._DIAGS)


def test_canon_export_block_diagnoses_unknown_canon():
    out = gen._canon_export_block(
        "made_up_canon",
        {"emits_property": {"name": "P", "type": "FVector"},
         "export_helper": "Foo::Baz"},
    )
    assert out == ""
    assert any("made_up_canon" in d.message for d in gen._DIAGS)
