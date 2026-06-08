# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the mjt* enum surface the codegen consumes.

build_mjspec_snapshot.py's regex-based enum extractor was retired in
Phase 3; mjt* enums now come from the clang-AST introspect snapshot
(libclang gives us the real member values + handles every
``typedef enum mjtX_ { ... } mjtX;`` shape uniformly). The shipped
snapshot still has to carry every enum codegen rules reference, so
the cross-check below is the surviving guard."""

from __future__ import annotations

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)


def _load_projected_mjspec():
    """Load introspect + project into the legacy mjspec shape via the
    same helper the runtime uses. Mirrors the conftest fixture."""
    import generate_ue_components as gen
    plugin_root = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
    snap_path = os.path.join(
        plugin_root, "Scripts", "codegen", "snapshots",
        "introspect_snapshot.json",
    )
    with open(snap_path, "r", encoding="utf-8") as f:
        introspect = json.load(f)
    return gen._mjspec_from_introspect(introspect)


def test_real_snapshot_has_expected_enums():
    """Introspect must carry every mjt* enum the codegen rules
    reference via xml_enum_attrs / value_map_from_enum."""
    mjspec = _load_projected_mjspec()
    enums = mjspec.get("enums", {})
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
    """The snapshot-driven ``value_map_from_enum`` resolver still
    produces a sensible value_map when wired against the projected
    mjspec. Catches drift between mjmodel.h enum names and rule-side
    ue_member_from_mj / xml_from_mj tables."""
    from generate_ue_components import _resolve_value_map
    mjspec = _load_projected_mjspec()
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
    resolved = _resolve_value_map("type", enum_def, mjspec)
    assert resolved == {
        "free":  ["Free",  "mjJNT_FREE"],
        "ball":  ["Ball",  "mjJNT_BALL"],
        "slide": ["Slide", "mjJNT_SLIDE"],
        "hinge": ["Hinge", "mjJNT_HINGE"],
    }
