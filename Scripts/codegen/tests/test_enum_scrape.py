# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Tests for the typedef-enum extractor in build_mjspec_snapshot.py.

Regex scrape of mjmodel.h ``typedef enum mjtX_ { ... } mjtX;`` blocks.
Output lands under ``"enums"`` in mjspec_snapshot.json and feeds the
``value_map_from_enum`` resolver in generate_ue_components.py.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)

from build_mjspec_snapshot import parse_enums  # noqa: E402


_FAKE_HEADER = r"""
// Some unrelated declarations.
struct SomeStruct { int x; };

typedef enum mjtJoint_ {          // type of degree of freedom
  mjJNT_FREE          = 0,        // 7 dofs
  mjJNT_BALL,                     // 4 dofs
  mjJNT_SLIDE,                    // 1 dof
  mjJNT_HINGE                     // 1 dof
} mjtJoint;

typedef enum mjtGeom_ {
  mjGEOM_PLANE        = 0,
  mjGEOM_HFIELD,
  mjGEOM_SPHERE,

  mjNGEOMTYPES,                   // counter sentinel; auto = 3

  mjGEOM_ARROW        = 100,
  mjGEOM_ARROW1,                  // 101
  mjGEOM_NONE         = 1001
} mjtGeom;

typedef enum mjtBitFlag_ {
  mjBIT_A = 1 << 0,
  mjBIT_B = 1 << 1,
} mjtBitFlag;
"""


def test_basic_enum_with_auto_increment():
    enums = parse_enums(_FAKE_HEADER)
    assert enums["mjtJoint"] == {
        "mjJNT_FREE":  0,
        "mjJNT_BALL":  1,
        "mjJNT_SLIDE": 2,
        "mjJNT_HINGE": 3,
    }


def test_explicit_overrides_reset_counter():
    enums = parse_enums(_FAKE_HEADER)
    # mjGEOM_PLANE explicitly = 0; HFIELD/SPHERE follow; mjNGEOMTYPES auto = 3;
    # mjGEOM_ARROW explicitly = 100; mjGEOM_ARROW1 = 101; mjGEOM_NONE = 1001.
    assert enums["mjtGeom"]["mjGEOM_PLANE"]  == 0
    assert enums["mjtGeom"]["mjGEOM_HFIELD"] == 1
    assert enums["mjtGeom"]["mjGEOM_SPHERE"] == 2
    assert enums["mjtGeom"]["mjNGEOMTYPES"]  == 3
    assert enums["mjtGeom"]["mjGEOM_ARROW"]  == 100
    assert enums["mjtGeom"]["mjGEOM_ARROW1"] == 101
    assert enums["mjtGeom"]["mjGEOM_NONE"]   == 1001


def test_bitshift_expressions_resolve():
    """``1 << 0`` / ``1 << 1`` are non-int-literal but eval cleanly in the
    sandbox. The extractor handles this so URLab's bitflag enums (e.g.
    mjtDisableBit, mjtEnableBit) come through with the right values."""
    enums = parse_enums(_FAKE_HEADER)
    assert enums["mjtBitFlag"] == {"mjBIT_A": 1, "mjBIT_B": 2}


def test_non_mjt_enums_skipped():
    """Only typedef enums whose tag matches ``mjt\\w+`` are captured. The
    snapshot is scoped to enums URLab might map through xml_enum_attrs;
    private/internal enums (e.g. tinyxml2's) stay out of the snapshot."""
    src = r"""
    typedef enum NotMine_ { FOO = 0 } NotMine;
    typedef enum mjtThing_ { mjT_X = 0 } mjtThing;
    """
    enums = parse_enums(src)
    assert "NotMine" not in enums
    assert "mjtThing" in enums


def test_real_snapshot_has_expected_enums():
    """The shipped mjspec_snapshot.json must carry the enums codegen rules
    reference via xml_enum_attrs / value_map_from_enum."""
    import json
    plugin_root = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
    snap = json.load(open(os.path.join(plugin_root, "Scripts", "codegen", "snapshots", "mjspec_snapshot.json")))
    enums = snap.get("enums", {})
    for expected in (
        "mjtJoint",      # joint.type
        "mjtGeom",       # geom.type
        "mjtSensor",     # sensor types
        "mjtObj",        # objtype / reftype
        "mjtTrn",        # actuator transmission
        "mjtIntegrator", # mjOption.integrator
        "mjtCone",       # mjOption.cone
        "mjtSolver",     # mjOption.solver
    ):
        assert expected in enums, f"missing enum {expected} from snapshot"
        assert len(enums[expected]) > 0, f"enum {expected} has no members"


def test_value_map_from_enum_resolves_with_real_snapshot():
    """Wire the 2c resolver to the real snapshot and confirm it builds a
    sensible value_map. Catches drift between mjmodel.h enum names and
    rule-side ue_member_from_mj / xml_from_mj tables."""
    import json
    from generate_ue_components import _resolve_value_map  # noqa: E501

    plugin_root = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
    snap = json.load(open(os.path.join(plugin_root, "Scripts", "codegen", "snapshots", "mjspec_snapshot.json")))

    enum_def = {
        "ue_property": "Type",
        "ue_enum_type": "EMjJointType",
        "value_map_from_enum": "mjtJoint",
        "ue_member_from_mj": {
            "mjJNT_FREE":  "Free",
            "mjJNT_BALL":  "Ball",
            "mjJNT_SLIDE": "Slide",
            "mjJNT_HINGE": "Hinge",
        },
        "xml_from_mj": {
            "mjJNT_FREE":  "free",
            "mjJNT_BALL":  "ball",
            "mjJNT_SLIDE": "slide",
            "mjJNT_HINGE": "hinge",
        },
    }
    resolved = _resolve_value_map("type", enum_def, snap)
    # Mapping order follows snapshot's enum iteration order.
    assert resolved == {
        "free":  ["Free",  "mjJNT_FREE"],
        "ball":  ["Ball",  "mjJNT_BALL"],
        "slide": ["Slide", "mjJNT_SLIDE"],
        "hinge": ["Hinge", "mjJNT_HINGE"],
    }
