# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Shared fixtures + helpers for codegen pytest tests.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict

# Make the parent codegen package importable when running pytest from the
# plugin root or from this directory.
_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)
_PLUGIN_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))

import pytest


@pytest.fixture(scope="session")
def real_schema() -> Dict[str, Any]:
    """The actual MJCF schema snapshot shipped with the plugin."""
    path = os.path.join(_PLUGIN_ROOT, "Scripts", "codegen", "snapshots", "mjcf_schema_snapshot.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="session")
def real_rules() -> Dict[str, Any]:
    path = os.path.join(_CODEGEN_DIR, "codegen_rules.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="session")
def real_mjxmacro() -> Dict[str, Any]:
    path = os.path.join(_PLUGIN_ROOT, "Scripts", "codegen", "snapshots", "mjxmacro_snapshot.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="session")
def real_mjspec() -> Dict[str, Any]:
    """The mjspec snapshot file was retired in Phase 3; the runtime
    now projects the introspect snapshot into the legacy mjspec shape.
    The fixture mirrors that projection so tests stay isomorphic."""
    import generate_ue_components as gen
    introspect_path = os.path.join(
        _PLUGIN_ROOT, "Scripts", "codegen", "snapshots", "introspect_snapshot.json",
    )
    with open(introspect_path, "r", encoding="utf-8") as f:
        introspect = json.load(f)
    return gen._mjspec_from_introspect(introspect)


@pytest.fixture(autouse=True)
def _clear_diags():
    """Each test starts with a clean diagnostics buffer + strict counter.
    Module-level state survives across tests otherwise — autouse keeps
    drift-check tests from leaking state into one another."""
    import generate_ue_components as gen  # noqa: E402, local import
    gen._reset_diags()
    yield
    gen._reset_diags()


@pytest.fixture
def minimal_rules() -> Dict[str, Any]:
    """A small synthetic rules block, useful for unit-testing emission
    shapes without coupling to the real schema."""
    return {
        "global_exclusions": ["user"],
        "default_type": "float",
        "property_renames": {},
        "type_mappings": {
            "range":      "TArray<float>",
            "limited":    "bool",
            "rgba":       "FLinearColor",
            "axis":       "FVector",
        },
        "canonicalizations": {
            "spatial_pose": {
                "absorbs_attrs": ["pos"],
                "emits_property": {"name": "Pos", "type": "FVector"},
                "import_helper": "MjUtils::ReadVec3InMeters",
                "export_helper": "MjUtils::UEToMjPosition",
            },
            "orientation": {
                "absorbs_attrs": ["quat", "euler", "axisangle", "xyaxes", "zaxis"],
                "emits_property": {"name": "Quat", "type": "FQuat"},
                "import_helper": "MjOrientationUtils::OrientationToMjQuat",
                "export_helper": "MjUtils::UEToMjRotation",
            },
        },
        "element_rules": {
            "joint": {
                "exclude_attrs": ["pos", "axis"],
            },
            "geom": {
                "exclude_attrs": ["material", "pos", "quat", "size", "fromto",
                                   "axisangle", "euler", "xyaxes", "zaxis"],
                "applies_canonicalizations": ["spatial_pose", "orientation"],
            },
        },
    }
