# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Tests for the libclang-backed EMj* hand-enum scrape added by Phase
1.8 to ``build_introspect_snapshot.py`` (and the snapshot consumer
``_hand_enums_from_snapshot`` in ``generate_ue_components.py``).

The libclang scrape:
  - extracts every ``enum class EMj* : uint8 { ... }`` block in the
    URLab Public/ tree
  - strips UE-only macros (UMETA / UENUM) so a minimal libclang TU can
    parse the block — including nested parens like
    ``UMETA(DisplayName="Track (Centre of Mass)")``
  - lands the result under ``snapshot.hand_enums`` in
    ``introspect_snapshot.json``
The runtime drift check ``_check_hand_enum_drift`` prefers this over the
fallback regex scrape — the snapshot is byte-stable across machines
because the macro stripping is deterministic."""

from __future__ import annotations

import json
import os

import pytest
import generate_ue_components as gen

# build_introspect_snapshot lives alongside generate_ue_components but is
# not always import-clean (it would try to load clang.cindex at import
# time). Test the strip helper via a tolerant import path.
import importlib.util
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
_BUILD_SNAP = os.path.join(_CODEGEN_DIR, "build_introspect_snapshot.py")


def _load_build_snapshot_module():
    spec = importlib.util.spec_from_file_location(
        "_build_introspect_snapshot_for_test", _BUILD_SNAP,
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def build_snap():
    return _load_build_snapshot_module()


# ---------- macro stripper -------------------------------------------------

def test_strip_handles_nested_parens_in_umeta(build_snap):
    """``UMETA(DisplayName="Track (Centre of Mass)")`` — the inner
    ``(Centre`` opens a paren the regex must not bail on. The previous
    naive ``UMETA\\([^)]*\\)`` regex stopped at the first ``)``, leaving
    a stray ``)`` that broke libclang's enum-member walk for any member
    after the first nested-paren UMETA."""
    src = (
        "enum class EMjSample : uint8 {\n"
        "    Fixed         UMETA(DisplayName = \"Fixed\"),\n"
        "    TrackCom      UMETA(DisplayName = \"Track (Centre of Mass)\"),\n"
        "    TargetBody    UMETA(DisplayName = \"Target Body\"),\n"
        "    TargetBodyCom UMETA(DisplayName = \"Target Body (Centre of Mass)\"),\n"
        "};\n"
    )
    out = build_snap._strip_ue_macros_from_enum_block(src)
    assert "UMETA" not in out
    # No stray close-parens left behind after the strip.
    assert ")" not in out
    # All four members survive.
    for m in ("Fixed", "TrackCom", "TargetBody", "TargetBodyCom"):
        assert m in out, f"member {m} lost during strip; out={out!r}"


def test_strip_handles_uenum_header(build_snap):
    src = (
        "UENUM(BlueprintType)\n"
        "enum class EMjFoo : uint8 { A, B };\n"
    )
    out = build_snap._strip_ue_macros_from_enum_block(src)
    assert "UENUM" not in out
    assert "enum class EMjFoo : uint8 { A, B };" in out


def test_strip_handles_line_and_block_comments(build_snap):
    src = (
        "enum class EMjFoo : uint8 {\n"
        "    A,  // first\n"
        "    B,  /* second */\n"
        "    /* a multi\n"
        "       line */ C,\n"
        "};\n"
    )
    out = build_snap._strip_ue_macros_from_enum_block(src)
    assert "first" not in out
    assert "second" not in out
    assert "multi" not in out
    for m in ("A", "B", "C"):
        assert m in out


def test_strip_balanced_call_terminates_on_unbalanced_input(build_snap):
    """An open-paren without a matching close should consume to EOF
    rather than infinite-loop the macro stripper."""
    src = "UMETA(unbalanced"
    out = build_snap._strip_ue_macros_from_enum_block(src)
    # Strip drove the cursor to EOF -> empty residue.
    assert out == ""


# ---------- snapshot output ------------------------------------------------

def test_snapshot_contains_at_least_one_hand_enum():
    """If the shipped snapshot has hand_enums, we trust the build step
    ran libclang against URLab's Public/ tree."""
    snap_path = os.path.join(
        os.path.dirname(_CODEGEN_DIR),
        "codegen", "snapshots", "introspect_snapshot.json",
    )
    if not os.path.exists(snap_path):
        pytest.skip("introspect_snapshot.json not present")
    with open(snap_path, "r", encoding="utf-8") as f:
        snap = json.load(f)
    hand_enums = snap.get("hand_enums", {})
    if not hand_enums:
        pytest.skip("hand_enums empty (libclang scrape disabled?)")
    # At least one EMj* enum and every entry has members.
    emj = [k for k in hand_enums if k.startswith("EMj")]
    assert emj, f"hand_enums missing EMj* entries; keys={list(hand_enums)[:5]}"
    for name, entry in hand_enums.items():
        assert "members" in entry, name
        assert entry["members"], f"{name} has no members"


# ---------- runtime consumer ----------------------------------------------

def test_hand_enums_from_snapshot_returns_flat_member_list():
    """The runtime drift check expects ``{enum_name: [member,...]}`` —
    the same shape the regex scrape produced. Confirm the converter
    flattens the introspect dict shape correctly."""
    mjspec = {
        "introspect": {
            "hand_enums": {
                "EMjSample": {
                    "members": {"A": 0, "B": 1, "C": 2},
                },
            },
        },
    }
    out = gen._hand_enums_from_snapshot(mjspec)
    assert out == {"EMjSample": ["A", "B", "C"]}


def test_hand_enums_from_snapshot_empty_when_missing():
    """No introspect / no hand_enums section -> empty dict (the drift
    check then falls back to the regex scrape)."""
    assert gen._hand_enums_from_snapshot({}) == {}
    assert gen._hand_enums_from_snapshot({"introspect": {}}) == {}
    assert gen._hand_enums_from_snapshot(None) == {}
