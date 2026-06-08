# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks ``_resolve_mjs_field``'s 5-step resolution chain and the
per-strategy helpers introduced by Phase 2.5. Each strategy lives in
its own function and is composed via ``_MJS_FIELD_RESOLVERS``."""

from __future__ import annotations

import generate_ue_components as gen


# ---------- per-strategy resolvers ----------------------------------------

def test_resolve_direct_match():
    assert gen._resolve_direct("condim", {"condim", "type"}) == "condim"
    assert gen._resolve_direct("absent", {"other"}) is None


def test_resolve_name_suffix():
    """MuJoCo name-ref fields uniformly append 'name'."""
    assert gen._resolve_name_suffix("mesh", {"meshname"}) == "meshname"
    assert gen._resolve_name_suffix("scalar", {"scalarvalue"}) is None


def test_resolve_root_name_digits_variants():
    """``body1`` -> ``bodyname1`` and the underscore variant
    ``body_name1`` both resolve."""
    assert gen._resolve_root_name_digits("body1", {"bodyname1"}) == "bodyname1"
    assert gen._resolve_root_name_digits("body1", {"body_name1"}) == "body_name1"
    # No-digit suffix -> no match.
    assert gen._resolve_root_name_digits("body", {"bodyname"}) is None


def test_resolve_underscore_norm_matches_drop_underscores():
    assert gen._resolve_underscore_norm("solreflimit", {"solref_limit"}) == "solref_limit"
    assert gen._resolve_underscore_norm("solreflimit", {"solref_friction"}) is None


def test_resolve_actuator_prefix_with_underscore_norm():
    """schema ``actuatorfrclimited`` -> mjs ``actfrclimited`` (direct)
    or ``act_frc_limited`` (via underscore norm)."""
    assert gen._resolve_actuator_prefix("actuatorfrclimited", {"actfrclimited"}) == "actfrclimited"
    assert gen._resolve_actuator_prefix("actuatorfrclimited", {"act_frc_limited"}) == "act_frc_limited"
    assert gen._resolve_actuator_prefix("notanactuator", {"anything"}) is None


# ---------- composition through _resolve_mjs_field ------------------------

def test_resolve_mjs_field_prefers_override_table():
    """Per-element ``attr_to_mjs_field`` rules ALWAYS win — no
    convention can override a rule author's explicit mapping."""
    out = gen._resolve_mjs_field(
        "target", {"target", "targetbody"},
        attr_to_mjs_field={"target": "targetbody"},
    )
    assert out == "targetbody"


def test_resolve_mjs_field_falls_through_strategy_order():
    """Each strategy fires in declared order; only the first match
    wins. Strategies that don't apply return None and fall through."""
    # No direct match; falls through to root+name+digits.
    out = gen._resolve_mjs_field("body1", {"bodyname1", "geomname1"})
    assert out == "bodyname1"


def test_resolve_mjs_field_falls_back_to_attr_when_nothing_matches():
    """If every strategy returns None, return the attr verbatim — the
    downstream C++ build will fail loudly on the bad field name."""
    out = gen._resolve_mjs_field(
        "novel_attr", {"unrelated_field"},
    )
    assert out == "novel_attr"


def test_resolve_mjs_field_empty_fields_returns_attr():
    """When ``mjs_fields`` is empty, no resolution is possible — the
    attr passes through unchanged."""
    assert gen._resolve_mjs_field("any", set()) == "any"


def test_resolvers_tuple_is_module_constant():
    """The resolver chain order must be a module-level constant so a
    test can assert it explicitly. Catches accidental reordering that
    would change which strategy wins on ambiguous inputs."""
    assert gen._MJS_FIELD_RESOLVERS == (
        gen._resolve_direct,
        gen._resolve_name_suffix,
        gen._resolve_root_name_digits,
        gen._resolve_underscore_norm,
        gen._resolve_actuator_prefix,
    )
