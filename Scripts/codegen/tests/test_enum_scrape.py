# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the mjt* enum surface the codegen consumes.

mjt* enums come from the clang-AST introspect snapshot — libclang
gives us the real member values and handles every ``typedef enum
mjtX_ { ... } mjtX;`` shape uniformly. The shipped snapshot has to
carry every enum the codegen rules reference; the cross-check below
is the guard."""

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
    """Introspect must carry every mjt* enum the codegen rules drift
    checks rely on (the cross-walk between value_map entries and the
    enum members the snapshot reports for that mjt* type)."""
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
