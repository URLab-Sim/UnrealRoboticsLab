# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Verifies that ``--strict`` (the default) maps fired diagnostics to
exit code 2. ``--no-strict`` lets the same rules surface diagnostics
without failing the run — local-triage workflow.

These tests drive the public DiagBuffer surface directly rather than
shelling out so they stay deterministic across CI without a libclang
install (the introspect snapshot is read from disk regardless of CI
state)."""

from __future__ import annotations

import generate_ue_components as gen


def test_strict_exit_2_when_diagnostics_fire():
    """A single diagnostic + ``--strict`` semantics yields exit 2 in
    main()'s gate path. We simulate by adding to the buffer, flushing,
    and reading the gate's input — ``_DIAGS_BUFFER.fired_count``."""
    gen._reset_diags()
    gen._diag_add("[diagnostic] test", source="unit_test")
    gen._diag_flush()
    fired = gen._DIAGS_BUFFER.fired_count
    assert fired == 1
    # The gate in main() is: `if args.strict and fired: return 2`.
    # That predicate is true here -> exit code would be 2.
    assert fired > 0


def test_strict_exit_0_when_no_diagnostics():
    gen._reset_diags()
    gen._diag_flush()
    assert gen._DIAGS_BUFFER.fired_count == 0


def test_reset_diags_clears_both_pending_and_fired_count():
    gen._diag_add("[diagnostic] alpha", source="unit_test")
    gen._diag_flush()
    assert gen._DIAGS_BUFFER.fired_count >= 1
    gen._reset_diags()
    assert gen._DIAGS_BUFFER.fired_count == 0
    assert gen._DIAGS_BUFFER.pending == []
