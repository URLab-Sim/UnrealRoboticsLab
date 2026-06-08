# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the ``PhaseContext`` introduced by Phase 1.6. Every
emission phase used to take 7 (or 8) positional args; they now take
one ``PhaseContext`` so adding a new field to the pipeline doesn't
touch a dozen signatures."""

from __future__ import annotations

import generate_ue_components as gen


def test_every_emission_phase_accepts_phase_context():
    """Every entry in EMISSION_PHASES.fn must be callable with one
    PhaseContext arg. Catches refactor drift where one phase keeps the
    old 7-positional-arg signature."""
    ctx = gen.PhaseContext(
        schema={}, rules={}, mjxmacro={}, mjspec={},
        public_root="", private_root="", bind_h_path="",
        writes=[],
    )
    for phase in gen.EMISSION_PHASES:
        # No assertion on side effects — just that the call doesn't
        # TypeError on signature mismatch.
        phase.fn(ctx)


def test_phase_context_is_dataclass_with_writes_list():
    """Quick sanity: ``writes`` must be a mutable list, not a frozen
    sequence — phases append to it."""
    ctx = gen.PhaseContext(
        schema={}, rules={}, mjxmacro={}, mjspec={},
        public_root="", private_root="", bind_h_path="",
        writes=[],
    )
    ctx.writes.append("sentinel")
    assert ctx.writes == ["sentinel"]


def test_phase_categories_via_ctx_emits_unknown_layout_diag():
    ctx = gen.PhaseContext(
        schema={}, rules={"categories": {"bogus": {"layout": "typo"}}},
        mjxmacro={}, mjspec={},
        public_root="", private_root="", bind_h_path="", writes=[],
    )
    gen._phase_categories(ctx)
    assert any("unknown layout" in d.message and "bogus" in d.message
               for d in gen._DIAGS)
    assert ctx.writes == []
