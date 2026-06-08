# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the three new drift checks introduced by Phase 2.6:
``_check_apply_mode_validity``, ``_check_embedded_cpp_references``,
and ``_check_allowlist_staleness`` (narrowed to 4 of 6 allowlists per
the locked plan)."""

from __future__ import annotations

import generate_ue_components as gen


# ---------- _check_apply_mode_validity -------------------------------------

def test_apply_mode_unknown_value_fires_diag():
    """A typo'd mode (``"guarderd"`` instead of ``"guarded"``) would
    silently fall through to the unconditional branch — the check
    surfaces it before the codegen emits a bad write path."""
    rules = {
        "synthetic_categories": {
            "MjOption": {
                "field_apply_to_spec_mode": {"timestep": "guarderd"},
            },
        },
    }
    gen._check_apply_mode_validity(rules)
    assert any("guarderd" in d.message and "timestep" in d.message
               for d in gen._DIAGS)


def test_apply_mode_known_values_silent():
    rules = {
        "synthetic_categories": {
            "MjOption": {
                "field_apply_to_spec_mode": {
                    "timestep": "unconditional",
                    "gravity":  "guarded_pos",
                    "wind":     "vec3_y_flip",
                },
            },
        },
    }
    gen._check_apply_mode_validity(rules)
    assert len(gen._DIAGS) == 0


def test_apply_mode_skips_note_keys():
    """The phase iterator must ignore ``_note_*`` sibling keys; they
    appear as strings, not dicts."""
    rules = {
        "synthetic_categories": {
            "_note_intent": "doc string for the section",
            "MjOption": {
                "field_apply_to_spec_mode": {"timestep": "unconditional"},
            },
        },
    }
    gen._check_apply_mode_validity(rules)
    assert len(gen._DIAGS) == 0


# ---------- _check_embedded_cpp_references ---------------------------------

def test_embedded_cpp_refs_flags_stale_mj_const():
    """``value_map[...][1]`` referencing a constant not in any mjspec
    enum is a UHT-bound failure — the check moves it forward into
    codegen-time."""
    rules = {
        "element_rules": {
            "joint": {
                "xml_enum_attrs": {
                    "type": {
                        "value_map": {
                            "hinge":  ["Hinge",  "mjJNT_HINGE"],
                            "screw":  ["Screw",  "mjJNT_FXED"],  # typo
                        },
                    },
                },
            },
        },
    }
    mjspec = {"enums": {"mjtJoint": {"mjJNT_HINGE": 0, "mjJNT_SLIDE": 1}}}
    gen._check_embedded_cpp_references(rules, mjspec)
    assert any("mjJNT_FXED" in d.message for d in gen._DIAGS)


def test_embedded_cpp_refs_silent_on_real_const():
    rules = {
        "element_rules": {
            "joint": {
                "xml_enum_attrs": {
                    "type": {
                        "value_map": {
                            "hinge": ["Hinge", "mjJNT_HINGE"],
                        },
                    },
                },
            },
        },
    }
    mjspec = {"enums": {"mjtJoint": {"mjJNT_HINGE": 0}}}
    gen._check_embedded_cpp_references(rules, mjspec)
    assert len(gen._DIAGS) == 0


def test_embedded_cpp_refs_flags_bad_geom_final_type_fallback():
    rules = {
        "categories": {
            "geom": {
                "geom_final_type": {
                    "xml_enum_ref": "type",
                    "override_field": "bOverride_Type",
                    "default_lookup": "Default->Type",
                    "default_fallback": "mjGEOM_FXED",  # typo
                    "name_field": "MeshName",
                    "name_export_for": "mjGEOM_MESH",
                    "name_target": "Element->mesh",
                    "name_setter": "MjSetString",
                },
            },
        },
    }
    mjspec = {"enums": {"mjtGeom": {"mjGEOM_SPHERE": 0, "mjGEOM_MESH": 7}}}
    gen._check_embedded_cpp_references(rules, mjspec)
    assert any("mjGEOM_FXED" in d.message for d in gen._DIAGS)


def test_embedded_cpp_refs_quiet_without_snapshot():
    """No mjspec / no enums -> can't check; the helper no-ops without
    crashing."""
    gen._check_embedded_cpp_references({}, None)
    assert len(gen._DIAGS) == 0
    gen._check_embedded_cpp_references({}, {"enums": {}})
    assert len(gen._DIAGS) == 0


# ---------- _check_allowlist_staleness -------------------------------------

def test_unmodeled_element_must_exist_in_schema():
    rules = {
        "intentionally_unmodeled_elements": {
            "skin": "URLab uses UE skeletal meshes instead",
            "phantom_element": "no longer exists",  # stale
        },
    }
    schema = {"body": {"attrs": ["pos"]}, "skin": {"attrs": []}}
    gen._check_allowlist_staleness(schema, rules, None)
    msgs = [d.message for d in gen._DIAGS]
    assert any("phantom_element" in m and "no longer carries" in m for m in msgs)
    assert not any("'skin'" in m for m in msgs)


def test_unmodeled_mjs_field_must_exist_on_struct():
    rules = {
        "intentionally_unmodeled_mjs_fields": {
            "mjsBody": ["explicitinertial", "phantom_field"],
            "mjsGhost": ["any"],  # stale struct
        },
    }
    mjspec = {
        "structs": {"mjsBody": ["explicitinertial", "name", "parent"]},
    }
    gen._check_allowlist_staleness({}, rules, mjspec)
    msgs = [d.message for d in gen._DIAGS]
    assert any("phantom_field" in m and "mjsBody" in m for m in msgs)
    assert any("mjsGhost" in m for m in msgs)
    assert not any("'explicitinertial'" in m and "mjsBody" in m for m in msgs)


def test_default_typed_attr_must_exist_somewhere_in_schema():
    rules = {
        "intentionally_default_typed_attrs": [
            "kp",                # in actuator_types.position list
            "torquescale",       # nested under equality.weld
            "phantom_attr",      # stale
        ],
    }
    schema = {
        "actuator_types": {"position": ["kp", "kv"]},
        "equality": {"weld": ["torquescale", "anchor"]},
    }
    gen._check_allowlist_staleness(schema, rules, None)
    msgs = [d.message for d in gen._DIAGS]
    assert any("phantom_attr" in m for m in msgs)
    assert not any("'kp'" in m for m in msgs)
    assert not any("'torquescale'" in m for m in msgs)


def test_unmapped_mj_enum_must_exist_in_mjspec():
    rules = {
        "intentionally_unmapped_mj_enum_values": {
            "mjtJoint": ["mjJNT_SLIDE"],
            "mjtGhost": ["mjGHOST_X"],
            "__per_element__": {"site.type": ["mjGEOM_PLANE"]},  # skipped
        },
    }
    mjspec = {"enums": {"mjtJoint": {"mjJNT_HINGE": 0, "mjJNT_SLIDE": 1}}}
    gen._check_allowlist_staleness({}, rules, mjspec)
    msgs = [d.message for d in gen._DIAGS]
    assert any("mjtGhost" in m for m in msgs)
    assert not any("mjtJoint" in m for m in msgs)
    # __per_element__ never fires its own staleness diag.
    assert not any("__per_element__" in m for m in msgs)


# ---------- _collect_every_schema_attr ------------------------------------

def test_collect_every_schema_attr_picks_up_three_shapes():
    """Cover the three list shapes the walker handles:
    nested-attrs key, flat list at any depth (actuator_types,
    equality.weld), nested-attrs inside sensor_types entries."""
    schema = {
        "body": {"attrs": ["pos", "quat"]},
        "actuator_types": {"position": ["kp", "kv"]},
        "equality": {"weld": ["torquescale"]},
        "sensor_types": [{"name": "accel", "attrs": ["cutoff"]}],
    }
    out = gen._collect_every_schema_attr(schema)
    for needed in ("pos", "quat", "kp", "kv", "torquescale", "cutoff"):
        assert needed in out, f"missing {needed} from {out}"
