"""The MuJoCo-defaults layer: its size, its coverage, and what it refuses.

The schema layer is what a details panel resolves an unset attribute through,
and for most of the schema `mjcf.schema` states no `=` default at all. The
values MuJoCo's own initialisers leave in the mjs structs fill that gap
(:mod:`protospec_gen.mjdefaults`), and these tests hold the result to exact
numbers rather than to "more than before".

The counts are pinned deliberately. A layer that quietly shrinks is invisible
-- every affected row simply goes back to reading as undefaulted, which is the
state the whole change exists to remove -- so the number moves only when
somebody moves the generator and updates it here with a reason.

`WITHOUT_MUJOCO` is measured, not remembered: the layer is switched off and the
schema re-read, so the split between the two sources is asserted against the
front end's own behaviour rather than against a number copied from a report.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile

import pytest

from protospec_gen import emit_ue, load_schema, mjdefaults

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))
import refresh_mj_defaults as refresh  # noqa: E402

# The schema layer, at the pinned MuJoCo. Move these only with the generator.
DEFAULTED_FIELDS = 856    # every stored field carrying a default of either origin
WITHOUT_MUJOCO = 209      # the `=` rows in mjcf.schema, alone
PROBE_COUNTS = {"defaults": 830, "dynamic": 186, "sentinel": 9, "unmapped": 17}

EMPTY = {"mujoco": "none", "defaults": {}, "skipped": {}}


@pytest.fixture(scope="module")
def ir():
    return load_schema()


@pytest.fixture(scope="module")
def data():
    return mjdefaults.load()


def defaulted(doc) -> set[tuple[str, str]]:
    return {(e["schema_name"], f["xml"]) for e in doc["elements"]
            for f in e["fields"] if "default" in f}


def written(payload) -> str:
    """`payload` as a data file on disk, for the reader's own error paths."""
    path = os.path.join(tempfile.mkdtemp(), "mujoco_defaults.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)
    return path


def test_defaulted_field_count_is_pinned(ir):
    assert len(defaulted(ir)) == DEFAULTED_FIELDS


def test_the_layer_without_mujoco_is_the_schemas_own_rows(monkeypatch):
    monkeypatch.setattr(mjdefaults, "load", lambda *a, **k: dict(EMPTY))
    assert len(defaulted(load_schema())) == WITHOUT_MUJOCO


def test_probe_table_counts_are_pinned(data):
    assert mjdefaults.counts(data) == PROBE_COUNTS


def test_every_probed_default_reaches_a_field(ir, data):
    probed = {(el, a) for el, rows in data["defaults"].items() for a in rows}
    assert len(probed) == PROBE_COUNTS["defaults"]
    assert probed <= defaulted(ir)


def test_probe_covers_exactly_what_the_spec_write_reaches(ir, data):
    """Neither side may drift: an unprobed binding loses its lowest layer
    silently, and a probed row the schema dropped is a stale value."""
    plan = emit_ue.SpecWritePlan(emit_ue.UeSchema(ir))
    reached = {(name, w.f["xml"]) for name, p in plan.by_name.items()
               for w in p.writes}
    assert reached == mjdefaults.coverage(data)


def test_a_stale_probed_row_fails_generation_by_name(monkeypatch):
    stale = {"mujoco": "none", "skipped": {},
             "defaults": {"geom": {"nosuchattribute": 1.0}}}
    monkeypatch.setattr(mjdefaults, "load", lambda *a, **k: stale)
    with pytest.raises(mjdefaults.DefaultsError, match="geom.nosuchattribute"):
        load_schema()


def test_an_unset_geom_margin_resolves_to_mujocos_value(ir, data):
    """The case the panel could not answer: `<geom>` states no margin, no
    `<default>` class mentions one, and MuJoCo still has a value for it."""
    geom = next(e for e in ir["elements"] if e["schema_name"] == "geom")
    margin = next(f for f in geom["fields"] if f["xml"] == "margin")
    assert margin["default"] == {"kind": "scalar", "value": 0.0}
    assert data["defaults"]["geom"]["margin"] == 0.0


def test_the_schema_default_wins_where_both_sources_have_one(ir, data):
    """`<general dynprm>` is the clearest case: the schema authors the one
    coefficient MJCF documents, MuJoCo's struct holds all ten slots."""
    general = next(e for e in ir["elements"] if e["schema_name"] == "general")
    dynprm = next(f for f in general["fields"] if f["xml"] == "dynprm")
    assert dynprm["default"]["values"] == [1.0]
    assert data["defaults"]["general"]["dynprm"][0] == 1.0
    assert len(data["defaults"]["general"]["dynprm"]) == 10


def test_a_computed_default_is_skipped_rather_than_displayed(ir, data):
    """mjNAN is MuJoCo saying "the compiler works this out", not a value.

    `<geom mass>` and the whole of `<statistic>` are initialised to it.
    """
    assert data["skipped"]["geom"]["mass"] == "sentinel"
    geom = next(e for e in ir["elements"] if e["schema_name"] == "geom")
    assert "default" not in next(f for f in geom["fields"]
                                 if f["xml"] == "mass")
    statistic = next(e for e in ir["elements"]
                     if e["schema_name"] == "statistic")
    assert all("default" not in f for f in statistic["fields"])


def test_an_owning_pointer_field_carries_no_default(ir, data):
    """An empty `mjString`/`mjDoubleVec` is what unset already looks like."""
    assert data["skipped"]["mesh"]["file"] == "dynamic"
    mesh = next(e for e in ir["elements"] if e["schema_name"] == "mesh")
    assert "default" not in next(f for f in mesh["fields"]
                                 if f["xml"] == "file")


def test_an_enum_default_resolves_through_the_schemas_own_keyword(ir, data):
    """The struct holds an int; only the keyword is displayable, and where no
    keyword names the int (mjOBJ_UNKNOWN) nothing is invented."""
    assert data["defaults"]["joint"]["limited"] == "auto"
    joint = next(e for e in ir["elements"] if e["schema_name"] == "joint")
    limited = next(f for f in joint["fields"] if f["xml"] == "limited")
    assert limited["default"] == {"kind": "enum", "member": "auto"}
    assert data["skipped"]["framepos"]["objtype"] == "unmapped"


def test_a_negative_marker_under_a_nonnegative_bound_is_a_sentinel():
    """The rule that keeps `-1 meaning auto` out of a display, stated against
    the schema's own bound rather than against a list of known markers."""
    assert refresh._nonnegative({"annotations": {"positive": True}})
    assert refresh._nonnegative({"annotations": {"min": 0}})
    assert not refresh._nonnegative({"annotations": {"min": -1}})
    assert not refresh._nonnegative({})


def test_the_skip_reasons_are_a_closed_set(data):
    reasons = {r for rows in data["skipped"].values() for r in rows.values()}
    assert reasons == set(mjdefaults.SKIP_REASONS)
    with pytest.raises(mjdefaults.DefaultsError, match="computed"):
        mjdefaults.load(written({"mujoco": "none", "defaults": {},
                                 "skipped": {"geom": {"mass": "computed"}}}))


def test_a_missing_data_file_says_how_to_refresh():
    """Not an empty layer: a run with no table would leave every row unresolved
    and nothing would say why."""
    with pytest.raises(mjdefaults.DefaultsError, match="refresh_mj_defaults"):
        mjdefaults.load(os.path.join(os.path.dirname(__file__), "nowhere.json"))
