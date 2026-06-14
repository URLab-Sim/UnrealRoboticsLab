# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks the two-case ``_should_emit_xml_enum_at_this_layer`` decision
the previous closure inside ``emit_schema_for_attrs`` encoded
implicitly:
  - subtype-specific xml_enum attrs fire on the subtype's own emission
  - "type"-like globally-excluded xml_enum attrs fire on the base
    (apply_canonicalizations=True) layer."""

from __future__ import annotations

import generate_ue_components as gen


def test_attr_in_attrs_list_fires_at_any_layer():
    """Subtype case — dcmotor.input is a subtype attr the base doesn't
    carry; the subtype layer (apply_canonicalizations=False) owns it."""
    assert gen._should_emit_xml_enum_at_this_layer(
        "input", attrs=["input"], global_excl=set(),
        apply_canonicalizations=False,
    )
    assert gen._should_emit_xml_enum_at_this_layer(
        "input", attrs=["input"], global_excl=set(),
        apply_canonicalizations=True,
    )


def test_globally_excluded_attr_fires_only_on_base_layer():
    """Always-at-base case — joint.type is globally excluded from
    per-attr UPROPERTY emission but xml_enum still owns its import/
    export at the base. Subtype layer must NOT re-emit it (would cause
    duplicate switch case + UHT shadowing)."""
    assert gen._should_emit_xml_enum_at_this_layer(
        "type", attrs=[], global_excl={"type"},
        apply_canonicalizations=True,
    )
    assert not gen._should_emit_xml_enum_at_this_layer(
        "type", attrs=[], global_excl={"type"},
        apply_canonicalizations=False,
    )


def test_unrelated_attr_never_fires():
    assert not gen._should_emit_xml_enum_at_this_layer(
        "stranger", attrs=["unrelated"], global_excl={"other"},
        apply_canonicalizations=True,
    )
