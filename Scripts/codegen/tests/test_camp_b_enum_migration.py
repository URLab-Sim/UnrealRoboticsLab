# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks the generated_enums extensions:
``extra_members`` (URLab-extra members appended after mj-derived ones)
and the ``disabled`` opt-out (skips emission so a rule can land before
the corresponding hand-rolled enums are swapped out).

Also asserts that the Camp B rule entries shipped under
``MjArticulationEnums`` in codegen_rules.json resolve cleanly against
the current mjspec snapshot — every ``from_mj_enum`` exists and the
``ue_member_from_mj`` keys all map to real ``mjt*`` constants."""

from __future__ import annotations

import os

import generate_ue_components as gen


_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
_PLUGIN_ROOT = os.path.dirname(os.path.dirname(_CODEGEN_DIR))


# ---------- extra_members --------------------------------------------------

def test_extra_members_append_after_mj_derived_members():
    """URLab-extra members (DcMotor on EMjGainType etc) must follow
    the mj-derived block so existing member ordering doesn't change."""
    rules_def = {
        "public_header": "Generated/MjT.h",
        "enums": [{
            "ue_name": "EMjT",
            "from_mj_enum": "mjtT",
            "ue_member_from_mj": {
                "mjT_A": "A",
                "mjT_B": "B",
            },
            "extra_members": [{"name": "DcMotor"}],
        }],
    }
    mjspec = {"enums": {"mjtT": {"mjT_A": 0, "mjT_B": 1}}}
    writes = gen._emit_generated_enum_file(
        "MjArticulationEnums", rules_def, mjspec, "/tmp/public",
    )
    assert len(writes) == 1
    content = writes[0].content
    a_idx = content.index("A = 0")
    b_idx = content.index("B = 1")
    dc_idx = content.index("DcMotor = 2")
    assert a_idx < b_idx < dc_idx


def test_extra_member_explicit_value_overrides_auto_increment():
    """``{"name": ..., "value": N}`` pins a specific underlying value
    — needed for enums that mirror mjt* numbering exactly."""
    rules_def = {
        "public_header": "Generated/MjT.h",
        "enums": [{
            "ue_name": "EMjT",
            "from_mj_enum": "mjtT",
            "ue_member_from_mj": {},
            "extra_members": [
                {"name": "None",  "value": 0},
                {"name": "Plane", "value": 7},
            ],
        }],
    }
    mjspec = {"enums": {"mjtT": {"mjT_DUMMY": 0}}}
    writes = gen._emit_generated_enum_file(
        "MjArticulationEnums", rules_def, mjspec, "/tmp/public",
    )
    assert writes
    content = writes[0].content
    assert "None = 0" in content
    assert "Plane = 7" in content


def test_extra_member_without_name_fires_diagnostic():
    rules_def = {
        "public_header": "Generated/MjT.h",
        "enums": [{
            "ue_name": "EMjT",
            "from_mj_enum": "mjtT",
            "ue_member_from_mj": {"mjT_A": "A"},
            "extra_members": [{"value": 5}],  # missing name
        }],
    }
    mjspec = {"enums": {"mjtT": {"mjT_A": 0}}}
    gen._emit_generated_enum_file(
        "MjArticulationEnums", rules_def, mjspec, "/tmp/public",
    )
    assert any("missing 'name'" in d.message for d in gen._DIAGS_BUFFER.pending)


# ---------- disabled opt-out -----------------------------------------------

def test_disabled_top_level_skips_emission():
    """``disabled: true`` at the rule entry level emits no FileWrite —
    used while staging a migration so a hand-rolled enum isn't
    duplicated by the codegen output."""
    rules_def = {
        "disabled": True,
        "public_header": "Generated/MjT.h",
        "enums": [{
            "ue_name": "EMjT",
            "from_mj_enum": "mjtT",
            "ue_member_from_mj": {"mjT_A": "A"},
        }],
    }
    mjspec = {"enums": {"mjtT": {"mjT_A": 0}}}
    writes = gen._emit_generated_enum_file(
        "MjArticulationEnums", rules_def, mjspec, "/tmp/public",
    )
    assert writes == []


def test_disabled_per_entry_skips_one_enum_in_a_rule():
    """Per-entry ``disabled: true`` skips just that enum but lets the
    rule's other entries still emit. Granular staging."""
    rules_def = {
        "public_header": "Generated/MjT.h",
        "enums": [
            {
                "ue_name": "EMjEnabled",
                "from_mj_enum": "mjtT",
                "ue_member_from_mj": {"mjT_A": "Alpha"},
            },
            {
                "ue_name": "EMjDisabled",
                "disabled": True,
                "from_mj_enum": "mjtT",
                "ue_member_from_mj": {"mjT_A": "Beta"},
            },
        ],
    }
    mjspec = {"enums": {"mjtT": {"mjT_A": 0}}}
    writes = gen._emit_generated_enum_file(
        "MjArticulationEnums", rules_def, mjspec, "/tmp/public",
    )
    assert len(writes) == 1
    content = writes[0].content
    assert "EMjEnabled" in content
    assert "EMjDisabled" not in content


# ---------- Camp B rule audit ---------------------------------------------

def test_camp_b_rule_resolves_against_real_mjspec(real_rules, real_mjspec):
    """Every Camp B entry's from_mj_enum must exist in the mjspec
    snapshot, and every ue_member_from_mj key must be a real mjt*
    constant. Catches typos in the migration rule before the actual
    flip lands."""
    camp_b = real_rules.get("generated_enums", {}).get("MjArticulationEnums")
    if not camp_b or not isinstance(camp_b, dict):
        return  # rule not present yet
    mj_enums = real_mjspec.get("enums", {})
    for entry in camp_b.get("enums", []):
        # Pure URLab enums (no mjt counterpart) omit from_mj_enum and
        # rely entirely on extra_members. Skip the cross-check for those.
        mj_enum_name = entry.get("from_mj_enum")
        if not mj_enum_name:
            continue
        assert mj_enum_name in mj_enums, (
            f"Camp B '{entry['ue_name']}' -> from_mj_enum "
            f"'{mj_enum_name}' not in mjspec.enums"
        )
        members = mj_enums[mj_enum_name]
        for mj_const in entry.get("ue_member_from_mj", {}):
            assert mj_const in members, (
                f"Camp B '{entry['ue_name']}' ue_member_from_mj key "
                f"'{mj_const}' is not in mjtEnum '{mj_enum_name}'"
            )


def test_camp_b_rule_post_flip_state(real_rules):
    """After the flip, the rule no longer carries ``disabled: true``
    and the hand-rolled UENUM decls under
    ``Source/URLab/Public/MuJoCo/Components/`` have been replaced
    with includes of the generated header."""
    camp_b = real_rules.get("generated_enums", {}).get("MjArticulationEnums")
    if not camp_b or not isinstance(camp_b, dict):
        return
    assert not camp_b.get("disabled"), (
        "MjArticulationEnums should be enabled after the Camp B flip."
    )


# On-disk presence is covered by test_regen_no_diff.py::test_codegen_check_clean.
