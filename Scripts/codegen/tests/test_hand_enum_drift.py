# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the hand-enum drift checks: ``_scan_hand_enums`` scrapes
URLab ``EMj*`` enum members from Source/, and ``_check_hand_enum_drift``
cross-references them against MuJoCo's ``mjt*`` enums and the rules'
``value_map`` UE-member references."""

from __future__ import annotations

import generate_ue_components as gen


def _write_enum_header(tmp_path, filename: str, enum_text: str):
    p = tmp_path / filename
    p.write_text(enum_text, encoding="utf-8")
    return p


# ---------- _scan_hand_enums -----------------------------------------------

def test_scan_hand_enums_parses_umeta_members(tmp_path):
    """UMETA(DisplayName=...) on each member must NOT hide the member
    from the scanner (was the original parser bug — EMjCameraTrackingMode
    lost TrackCom and TargetBodyCom because the per-line regex required
    the line to end at the member)."""
    _write_enum_header(tmp_path, "Sample.h", """
        UENUM(BlueprintType)
        enum class EMjSample : uint8
        {
            Fixed         UMETA(DisplayName = "Fixed"),
            Track         UMETA(DisplayName = "Track"),
            TrackCom      UMETA(DisplayName = "Track (Centre of Mass)"),
            TargetBody    UMETA(DisplayName = "Target Body"),
            TargetBodyCom UMETA(DisplayName = "Target Body (Centre of Mass)"),
        };
    """)
    enums = gen._scan_hand_enums(str(tmp_path))
    assert "EMjSample" in enums
    assert enums["EMjSample"] == [
        "Fixed", "Track", "TrackCom", "TargetBody", "TargetBodyCom",
    ]


def test_scan_hand_enums_handles_explicit_values(tmp_path):
    """Explicit ``= N`` initialisers must not break member capture."""
    _write_enum_header(tmp_path, "WithVals.h", """
        enum class EMjWithVals : uint8
        {
            Connect     = 0,
            Weld        = 1,
            Joint       = 2,
        };
    """)
    enums = gen._scan_hand_enums(str(tmp_path))
    assert enums["EMjWithVals"] == ["Connect", "Weld", "Joint"]


# ---------- _check_hand_enum_drift -----------------------------------------

def test_hand_enum_drift_diagnoses_subtype_enum_value_typo(tmp_path):
    _write_enum_header(tmp_path, "MjJoint.h", """
        enum class EMjJointType : uint8 { Hinge, Slide, Ball };
    """)
    rules = {
        "categories": {
            "joint": {
                "type_enum_name": "EMjJointType",
                "subtypes": [
                    {"key": "hinge", "enum_value": "Hinge"},
                    {"key": "screw", "enum_value": "Screwd"},  # typo
                ],
            },
        },
    }
    gen._check_hand_enum_drift({}, rules, None, str(tmp_path))
    assert any("Screwd" in d.message and "EMjJointType" in d.message
               for d in gen._DIAGS)


def test_hand_enum_drift_diagnoses_value_map_ue_member_typo(tmp_path):
    _write_enum_header(tmp_path, "MjGeom.h", """
        enum class EMjGeomType : uint8 { Sphere, Box, Mesh };
    """)
    rules = {
        "categories": {},
        "element_rules": {
            "geom": {
                "xml_enum_attrs": {
                    "type": {
                        "ue_enum_type": "EMjGeomType",
                        "value_map": {
                            "sphere": ["Sphere", "mjGEOM_SPHERE"],
                            "box":    ["Bxox",   "mjGEOM_BOX"],  # typo
                        },
                    },
                },
            },
        },
    }
    gen._check_hand_enum_drift({}, rules, None, str(tmp_path))
    assert any("Bxox" in d.message and "EMjGeomType" in d.message
               for d in gen._DIAGS)


def test_hand_enum_drift_surfaces_new_mj_enum_value(tmp_path):
    """A new mjJNT_SCREW upstream with no value_map entry fires a
    diagnostic — closes the silent-runtime-fallthrough class."""
    _write_enum_header(tmp_path, "MjJoint.h", """
        enum class EMjJointType : uint8 { Hinge, Slide, Ball };
    """)
    rules = {
        "categories": {},
        "element_rules": {
            "joint": {
                "xml_enum_attrs": {
                    "type": {
                        "ue_enum_type": "EMjJointType",
                        "mjs_cast": "mjtJoint",
                        "value_map": {
                            "hinge": ["Hinge", "mjJNT_HINGE"],
                            "slide": ["Slide", "mjJNT_SLIDE"],
                            "ball":  ["Ball",  "mjJNT_BALL"],
                        },
                    },
                },
            },
        },
    }
    mjspec = {
        "enums": {
            "mjtJoint": ["mjJNT_FREE", "mjJNT_BALL", "mjJNT_SLIDE",
                         "mjJNT_HINGE", "mjJNT_SCREW"],
        },
    }
    gen._check_hand_enum_drift({}, rules, mjspec, str(tmp_path))
    msgs = [d.message for d in gen._DIAGS]
    assert any("mjJNT_SCREW" in m for m in msgs), msgs
    assert any("mjJNT_FREE" in m for m in msgs), msgs


def test_hand_enum_drift_respects_intentional_skips(tmp_path):
    _write_enum_header(tmp_path, "MjActuator.h", """
        enum class EMjGainType : uint8 { Fixed, Affine, Muscle };
    """)
    rules = {
        "categories": {},
        "intentionally_unmapped_mj_enum_values": {
            "mjtGain": ["mjGAIN_DCMOTOR"],
        },
        "element_rules": {
            "actuator": {
                "xml_enum_attrs": {
                    "gaintype": {
                        "ue_enum_type": "EMjGainType",
                        "mjs_cast": "mjtGain",
                        "value_map": {
                            "fixed":   ["Fixed",   "mjGAIN_FIXED"],
                            "affine":  ["Affine",  "mjGAIN_AFFINE"],
                            "muscle":  ["Muscle",  "mjGAIN_MUSCLE"],
                        },
                    },
                },
            },
        },
    }
    mjspec = {
        "enums": {
            "mjtGain": ["mjGAIN_FIXED", "mjGAIN_AFFINE", "mjGAIN_MUSCLE",
                        "mjGAIN_USER", "mjGAIN_DCMOTOR"],
        },
    }
    gen._check_hand_enum_drift({}, rules, mjspec, str(tmp_path))
    msgs = [d.message for d in gen._DIAGS]
    # DCMOTOR is in the skip list -> no diag.
    assert not any("mjGAIN_DCMOTOR" in m for m in msgs)
    # USER is not -> diag fires.
    assert any("mjGAIN_USER" in m for m in msgs), msgs


def test_hand_enum_drift_per_element_overrides(tmp_path):
    """Same mjt* enum (mjtGeom) admits different subsets for geom vs
    site. The __per_element__ override must scope correctly."""
    _write_enum_header(tmp_path, "MjGeom.h", """
        enum class EMjGeomType : uint8 { Sphere, Box, Plane, Mesh };
    """)
    _write_enum_header(tmp_path, "MjSite.h", """
        enum class EMjSiteType : uint8 { Sphere, Box };
    """)
    rules = {
        "categories": {},
        "intentionally_unmapped_mj_enum_values": {
            "__per_element__": {
                "site.type": ["mjGEOM_PLANE", "mjGEOM_MESH"],
            },
        },
        "element_rules": {
            "geom": {
                "xml_enum_attrs": {
                    "type": {
                        "ue_enum_type": "EMjGeomType",
                        "mjs_cast": "mjtGeom",
                        "value_map": {
                            "sphere": ["Sphere", "mjGEOM_SPHERE"],
                            "box":    ["Box",    "mjGEOM_BOX"],
                            "plane":  ["Plane",  "mjGEOM_PLANE"],
                            "mesh":   ["Mesh",   "mjGEOM_MESH"],
                        },
                    },
                },
            },
            "site": {
                "xml_enum_attrs": {
                    "type": {
                        "ue_enum_type": "EMjSiteType",
                        "mjs_cast": "mjtGeom",
                        "value_map": {
                            "sphere": ["Sphere", "mjGEOM_SPHERE"],
                            "box":    ["Box",    "mjGEOM_BOX"],
                        },
                    },
                },
            },
        },
    }
    mjspec = {"enums": {"mjtGeom": ["mjGEOM_SPHERE", "mjGEOM_BOX",
                                      "mjGEOM_PLANE", "mjGEOM_MESH"]}}
    gen._check_hand_enum_drift({}, rules, mjspec, str(tmp_path))
    msgs = [d.message for d in gen._DIAGS]
    # Geom has all 4 mapped -> no diag.
    assert not any("element 'geom'" in m for m in msgs)
    # Site skips Plane + Mesh -> no diag for those either.
    assert not any("mjGEOM_PLANE" in m and "site" in m for m in msgs)
    assert not any("mjGEOM_MESH" in m and "site" in m for m in msgs)
