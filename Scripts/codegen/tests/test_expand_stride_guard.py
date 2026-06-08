# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Locks the ``_expand_stride`` guard. ``MJ_M(...)`` lowers to
``m->...``; numeric and ``mjN*`` strides pass through unchanged. Any
``MJ_*( ... )`` macro that survives is an unhandled shape and must
surface a diagnostic before the generated MjBind.h fails to compile."""

from __future__ import annotations

import generate_ue_components as gen


def test_mj_m_macro_lowers_to_pointer_field():
    assert gen._expand_stride("MJ_M(nuser_jnt)") == "m->nuser_jnt"


def test_numeric_stride_passes_through_unchanged():
    for s in ("1", "3", "9"):
        assert gen._expand_stride(s) == s


def test_mj_n_constant_passes_through_unchanged():
    assert gen._expand_stride("mjNREF") == "mjNREF"
    assert gen._expand_stride("mjNBIAS") == "mjNBIAS"


def test_compound_expression_expands_and_keeps_other_tokens():
    """Real MuJoCo strides occasionally combine ``MJ_M(...)*N`` for
    derived per-element block sizes. The guard must NOT fire because
    the substitution removes the only MJ_ macro."""
    assert gen._expand_stride("MJ_M(nuser_geom)*3") == "m->nuser_geom*3"
    assert len(gen._DIAGS_BUFFER.pending) == 0


def test_unknown_macro_fires_guard_diagnostic():
    """A future MuJoCo bump that introduces e.g. ``MJ_D(foo)`` must
    not silently land verbatim in MjBind.h."""
    out = gen._expand_stride("MJ_D(foo)")
    assert "MJ_D(foo)" in out
    assert any("unhandled MuJoCo macro" in d.message and "MJ_D(foo)" in d.message
               for d in gen._DIAGS_BUFFER.pending)


def test_multiple_unknown_macros_listed_together():
    gen._expand_stride("MJ_D(a) + MJ_V(b)")
    diags = [d.message for d in gen._DIAGS_BUFFER.pending]
    assert any("MJ_D(a)" in m and "MJ_V(b)" in m for m in diags), diags


def test_known_and_unknown_mixed_still_fires_for_unknown():
    """Known macro expanded, unknown surfaces a diag."""
    out = gen._expand_stride("MJ_M(nuser_jnt) * MJ_D(foo)")
    assert "m->nuser_jnt" in out
    assert "MJ_D(foo)" in out
    assert any("MJ_D(foo)" in d.message for d in gen._DIAGS_BUFFER.pending)
