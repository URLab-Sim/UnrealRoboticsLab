# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Behavioural lock-ins for the canon emitter bodies that previously
only had registry-presence coverage. ``actuator_transmission`` and
``fromto_decompose`` are inline (no rule-side helper string); their
emitted C++ is hand-crafted, so any rename or signature change there
should surface in pytest rather than at UE compile."""

from __future__ import annotations

import generate_ue_components as gen


# ---------- actuator_transmission ---------------------------------------

def test_actuator_transmission_import_reads_every_target_attr():
    """The import block reads joint/jointinparent/tendon/site/body and
    the slidersite/refsite siblings. A missing branch means a rule
    author can't author that transmission type via XML."""
    out = gen._canon_import_actuator_transmission(canon_def={}, element_name="actuator")
    for attr in ("joint", "jointinparent", "tendon", "site", "body",
                 "slidersite", "refsite"):
        assert f'TEXT("{attr}")' in out, f"import missing TEXT(\"{attr}\")"
    # Each path resolves to an EMjActuatorTrnType.
    for trn in ("Joint", "JointInParent", "Tendon", "Site", "Body",
                "SliderCrank"):
        assert f"EMjActuatorTrnType::{trn}" in out, trn


def test_actuator_transmission_export_emits_switch_with_undefined_default():
    out = gen._canon_export_actuator_transmission(canon_def={})
    # Every case writes Element->trntype.
    for trn, mj_const in [
        ("Joint", "mjTRN_JOINT"),
        ("JointInParent", "mjTRN_JOINTINPARENT"),
        ("Tendon", "mjTRN_TENDON"),
        ("Site", "mjTRN_SITE"),
        ("Body", "mjTRN_BODY"),
        ("SliderCrank", "mjTRN_SLIDERCRANK"),
    ]:
        assert f"EMjActuatorTrnType::{trn}" in out, trn
        assert mj_const in out, mj_const
    assert "default:" in out and "mjTRN_UNDEFINED" in out
    # slidersite / refsite only written when their type is set.
    assert "SliderCrank && !SliderSite.IsEmpty()" in out
    assert "Site && !RefSite.IsEmpty()" in out


# ---------- fromto_decompose --------------------------------------------

def test_fromto_decompose_import_uses_helper_and_writes_pos_quat_size():
    canon_def = {
        "import_helper": "MjUtils::DecomposeFromTo",
        "category_specifics": {"geom": {
            "z_slot_check": "(Type == EMjGeomType::Box || Type == EMjGeomType::Ellipsoid)"
        }},
    }
    out = gen._canon_import_fromto_decompose(canon_def, element_name="geom")
    assert "MjUtils::DecomposeFromTo(Node, FTPos, FTQuat, FTHalf)" in out
    # Writes Pos / Quat with their override toggles.
    assert "Pos = FTPos;" in out and "bOverride_Pos = true;" in out
    assert "Quat = FTQuat;" in out and "bOverride_Quat = true;" in out
    # Uses the category-specific z_slot_check inline.
    assert "EMjGeomType::Box" in out and "EMjGeomType::Ellipsoid" in out
    # Grows the size array to accommodate the slot.
    assert "while (size.Num() <= FTSlot)" in out


def test_fromto_decompose_import_falls_back_to_y_slot_for_capsule():
    canon_def = {
        "import_helper": "MjUtils::DecomposeFromTo",
        "category_specifics": {"site": {
            "z_slot_check": "(Type == EMjSiteType::Box || Type == EMjSiteType::Ellipsoid)"
        }},
    }
    out = gen._canon_import_fromto_decompose(canon_def, element_name="site")
    # The z_slot_check is element-specific; site uses EMjSiteType, not Geom.
    assert "EMjSiteType::Box" in out
    assert "EMjGeomType" not in out
    # Without a matching category_specifics entry, the slot would be `false` (Y slot).
    out_other = gen._canon_import_fromto_decompose(
        {"import_helper": "MjUtils::DecomposeFromTo"},
        element_name="unknown",
    )
    assert "bFTZSlot = false" in out_other


def test_fromto_decompose_export_is_empty():
    """The export path is a no-op: ``fromto`` decomposes to Pos/Quat/Size
    on import; those properties carry the values through to export via
    their own canonicalisations."""
    assert gen._canon_export_fromto_decompose(canon_def={}) == ""
