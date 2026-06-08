# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the canonicalisation registry introduced by Phase 1.7.

The registry collapses the previous duplicated if-chains in
``_canon_import_block`` / ``_canon_export_block`` into one dict.
Adding a new canon means adding one ``Canonicalisation`` entry; an
unknown canon name in the JSON rules still fires the same loud
diagnostic instead of silently dropping its emission."""

from __future__ import annotations

import generate_ue_components as gen


def test_registry_covers_every_canonicalisation_in_rules_json(real_rules):
    """Every canonicalisation declared in codegen_rules.json must have
    a registry entry. Mismatch fires the silent-drop diagnostic at
    runtime — catch it here at test time instead."""
    declared = set(real_rules.get("canonicalizations", {}).keys())
    registered = set(gen._CANONICALISATIONS.keys())
    missing = declared - registered
    assert not missing, (
        f"canonicalizations declared in rules JSON but not in the "
        f"registry: {sorted(missing)}"
    )


def test_unknown_canon_fires_import_diagnostic():
    out = gen._canon_import_block("nonexistent_canon", {}, element_name="geom")
    assert out == ""
    assert any("nonexistent_canon" in d.message and "import" in d.message
               for d in gen._DIAGS)


def test_unknown_canon_fires_export_diagnostic():
    out = gen._canon_export_block("nonexistent_canon", {})
    assert out == ""
    assert any("nonexistent_canon" in d.message and "export" in d.message
               for d in gen._DIAGS)


def test_spatial_pose_import_is_one_liner_helper_call():
    """Locks the trivial-shape canonicalisation so a regression in the
    registry dispatch shows up directly."""
    canon_def = {
        "import_helper": "MjUtils::ReadVec3InMeters",
        "emits_property": {"name": "Pos", "type": "FVector"},
    }
    out = gen._canon_import_block("spatial_pose", canon_def, "")
    assert out.strip() == (
        'MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);'
    )


def test_registry_entries_are_callable():
    for name, entry in gen._CANONICALISATIONS.items():
        assert callable(entry.import_emitter), name
        assert callable(entry.export_emitter), name


def test_canonicalisation_class_is_frozen():
    """Registry entries must be frozen so a phase can't accidentally
    swap an emitter at runtime."""
    entry = next(iter(gen._CANONICALISATIONS.values()))
    import dataclasses
    assert dataclasses.is_dataclass(entry)
    # FrozenInstanceError on attempted assignment.
    import pytest
    with pytest.raises(dataclasses.FrozenInstanceError):
        entry.name = "mutated"  # type: ignore[misc]
