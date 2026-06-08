# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the rule-iteration helpers ``compute_canon_absorbed`` /
``iter_canon_absorbed`` / ``iter_category_attrs``. They are the single
source of truth for the standard three-skip exclusion gate that every
per-attr emission site used to inline."""

from __future__ import annotations

import generate_ue_components as gen
import _codegen_core as _core
gen.iter_canon_absorbed = _core.iter_canon_absorbed


# ---------- iter_canon_absorbed / compute_canon_absorbed -------------------

def test_compute_canon_absorbed_unions_applied_canons():
    canons = {
        "spatial_pose": {"absorbs_attrs": ["pos", "quat", "euler", "axisangle"]},
        "fromto_decompose": {"absorbs_attrs": ["fromto"]},
        "unused": {"absorbs_attrs": ["other"]},
    }
    got = gen.compute_canon_absorbed(["spatial_pose", "fromto_decompose"], canons)
    assert got == {"pos", "quat", "euler", "axisangle", "fromto"}
    # An unapplied canon stays out — gating is by applies_canonicalizations.
    assert "other" not in got


def test_compute_canon_absorbed_silent_on_missing_canon():
    canons = {"spatial_pose": {"absorbs_attrs": ["pos"]}}
    got = gen.compute_canon_absorbed(["spatial_pose", "ghost_canon"], canons)
    assert got == {"pos"}


def test_compute_canon_absorbed_handles_empty_inputs():
    assert gen.compute_canon_absorbed([], {}) == set()
    assert gen.compute_canon_absorbed(["any"], {}) == set()
    assert gen.compute_canon_absorbed([], {"spatial_pose": {"absorbs_attrs": ["pos"]}}) == set()


def test_iter_canon_absorbed_is_lazy_view():
    canons = {"c": {"absorbs_attrs": ["a", "b"]}}
    it = gen.iter_canon_absorbed(["c"], canons)
    # Confirm it's an iterator, not a materialised list — call sites that
    # only want a single membership check shouldn't pay for a list build.
    assert iter(it) is it
    assert list(it) == ["a", "b"]


# ---------- iter_category_attrs --------------------------------------------

def test_iter_category_attrs_filters_three_gates():
    attrs = ["a", "b", "c", "d", "e"]
    got = list(gen.iter_category_attrs(
        attrs,
        global_excl={"a"},
        elem_excl={"b"},
        canon_absorbed={"c"},
    ))
    assert got == ["d", "e"]


def test_iter_category_attrs_preserves_order():
    """Codegen output depends on attr order — schema order must survive
    the filter so emitted .h/.cpp stay byte-identical to the schema."""
    attrs = ["zeta", "alpha", "mu"]
    got = list(gen.iter_category_attrs(
        attrs, global_excl=set(), elem_excl=set(), canon_absorbed=set(),
    ))
    assert got == ["zeta", "alpha", "mu"]


def test_iter_category_attrs_handles_empty_attrs():
    got = list(gen.iter_category_attrs(
        [], global_excl={"a"}, elem_excl=set(), canon_absorbed=set(),
    ))
    assert got == []
