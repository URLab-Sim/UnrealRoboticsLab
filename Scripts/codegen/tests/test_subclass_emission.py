# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Subclass file emission tests (single_uclass_per_file layout)."""

from __future__ import annotations

import json
import os
import tempfile

import pytest

from generate_ue_components import emit_subclass_files


_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))


def _find_writes(writes, suffix):
    return [w for w in writes if w.path.endswith(suffix)]


def test_actuator_subclass_count(real_schema, real_rules, real_mjspec):
    cat_rules = real_rules["categories"]["actuator"]
    writes = emit_subclass_files(
        "actuator", cat_rules, real_schema, real_rules,
        "/tmp/pub", "/tmp/priv", mjspec=real_mjspec,
    )
    # 10 subtypes * 2 (h + cpp) = 20 writes.
    assert len(writes) == 20
    # Every advertised class_name has an .h.
    for st in cat_rules["subtypes"]:
        assert _find_writes(writes, st["header"]), f"missing {st['header']}"


def test_sensor_subclass_count(real_schema, real_rules):
    cat_rules = real_rules["categories"]["sensor"]
    writes = emit_subclass_files(
        "sensor", cat_rules, real_schema, real_rules,
        "/tmp/pub", "/tmp/priv",
    )
    # 49 subtypes * 2 = 98 writes.
    assert len(writes) == 98


def test_subclass_header_has_codegen_tags(real_schema, real_rules):
    cat_rules = real_rules["categories"]["joint"]
    writes = emit_subclass_files(
        "joint", cat_rules, real_schema, real_rules, "/tmp/pub", "/tmp/priv",
    )
    hinge_h = _find_writes(writes, "MjHingeJoint.h")[0]
    assert "// --- CODEGEN_PROPERTIES_START ---" in hinge_h.content
    assert "// --- CODEGEN_PROPERTIES_END ---" in hinge_h.content
    assert "GENERATED_BODY()" in hinge_h.content
    # Inherits base class.
    assert ": public UMjJoint" in hinge_h.content


def test_subclass_source_has_super_export(real_schema, real_rules):
    cat_rules = real_rules["categories"]["joint"]
    writes = emit_subclass_files(
        "joint", cat_rules, real_schema, real_rules, "/tmp/pub", "/tmp/priv",
    )
    hinge_cpp = _find_writes(writes, "MjHingeJoint.cpp")[0]
    # ImportFromXml chains to Super.
    assert "Super::ImportFromXml(Node, CompilerSettings);" in hinge_cpp.content
    # ExportTo chains to Super.
    assert "Super::ExportTo(Element, Default);" in hinge_cpp.content
    # Constructor sets the right enum.
    assert "Type = EMjJointType::Hinge;" in hinge_cpp.content


def test_actuator_subclass_calls_mjs_set_to(real_schema, real_rules, real_mjspec):
    cat_rules = real_rules["categories"]["actuator"]
    writes = emit_subclass_files(
        "actuator", cat_rules, real_schema, real_rules,
        "/tmp/pub", "/tmp/priv", mjspec=real_mjspec,
    )
    motor_cpp = _find_writes(writes, "MjMotorActuator.cpp")[0]
    # The introspected setto emitter places ``mjs_setToMotor(Element);`` at
    # the TOP of the CODEGEN_EXPORT block. Super::ExportTo is called BEFORE
    # the codegen block (so that base attrs are written before subtype
    # presets). Both orderings used to be claimed in different places of the
    # generator — assert the current actual behaviour.
    src = motor_cpp.content
    set_idx = src.index("mjs_setToMotor(Element);")
    super_idx = src.index("Super::ExportTo(Element, Default);")
    # Super first, then setto preset.
    assert super_idx < set_idx
    # No ApplyRawOverrides call anywhere.
    assert "ApplyRawOverrides" not in src


def test_new_geom_subtypes_emitted(real_schema, real_rules):
    cat_rules = real_rules["categories"]["geom"]
    writes = emit_subclass_files(
        "geom", cat_rules, real_schema, real_rules, "/tmp/pub", "/tmp/priv",
    )
    for new_subtype in ("MjEllipsoid.h", "MjPlane.h", "MjSdf.h"):
        assert _find_writes(writes, new_subtype), f"missing geom subtype {new_subtype}"


def test_new_sensor_subtypes_emitted(real_schema, real_rules):
    cat_rules = real_rules["categories"]["sensor"]
    writes = emit_subclass_files(
        "sensor", cat_rules, real_schema, real_rules, "/tmp/pub", "/tmp/priv",
    )
    for new_subtype in ("MjUserSensor.h", "MjPluginSensor.h"):
        assert _find_writes(writes, new_subtype), f"missing sensor subtype {new_subtype}"
