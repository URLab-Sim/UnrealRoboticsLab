# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""End-to-end dry-run test: generator runs against the real schema +
mjxmacro snapshots and produces a deterministic set of writes."""

from __future__ import annotations

import json
import os
import sys

import pytest


_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))


def test_collect_all_writes_includes_expected_categories(real_schema, real_rules, real_mjxmacro, real_mjspec):
    from generate_ue_components import collect_all_writes
    # The plugin's actual Public/Private roots (read-only — we don't write).
    plugin_root = _PLUGIN_ROOT
    public_root = os.path.join(plugin_root, "Source", "URLab", "Public")
    private_root = os.path.join(plugin_root, "Source", "URLab", "Private")
    bind_h = os.path.join(public_root, "MuJoCo", "Utils", "MjBind.h")
    writes = collect_all_writes(real_schema, real_rules, real_mjxmacro,
                                public_root, private_root, bind_h,
                                mjspec=real_mjspec)
    # We always emit the test-schema header.
    paths = [w.path for w in writes]
    assert any(p.endswith("MjSchemaForTests.h") for p in paths)
    # 49 sensor subtypes × (.h + .cpp) = 98 subclass writes; plus the
    # base files (MjSensor.h/.cpp + MjCamera.h/.cpp, post-migration).
    sensor_subclass_writes = [
        p for p in paths
        if "Components" in p and "Sensors" in p
        and (p.endswith(".h") or p.endswith(".cpp"))
        and "MjSensor" not in os.path.basename(p)
        and "MjCamera" not in os.path.basename(p)
    ]
    assert len(sensor_subclass_writes) == 98, (
        f"expected 98 sensor subclass file writes, got {len(sensor_subclass_writes)}"
    )


def test_generator_is_deterministic(real_schema, real_rules, real_mjxmacro, real_mjspec):
    """Same inputs -> same outputs."""
    from generate_ue_components import collect_all_writes
    plugin_root = _PLUGIN_ROOT
    public_root = os.path.join(plugin_root, "Source", "URLab", "Public")
    private_root = os.path.join(plugin_root, "Source", "URLab", "Private")
    bind_h = os.path.join(public_root, "MuJoCo", "Utils", "MjBind.h")
    a = collect_all_writes(real_schema, real_rules, real_mjxmacro,
                           public_root, private_root, bind_h, mjspec=real_mjspec)
    b = collect_all_writes(real_schema, real_rules, real_mjxmacro,
                           public_root, private_root, bind_h, mjspec=real_mjspec)
    a_map = {w.path: w.content for w in a}
    b_map = {w.path: w.content for w in b}
    assert a_map == b_map
