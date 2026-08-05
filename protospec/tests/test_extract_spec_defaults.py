"""Tests for the spec-struct default-value extractor.

Unit tests run against small embedded C excerpts (no dependency on the vendored tree).
One integration test parses the real vendored ``user_init.c`` when it is present.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from _mujoco_src import MUJOCO_SRC, SKIP_REASON

_MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "tools" / "bootstrap" / "extract_spec_defaults.py"
)
_spec = importlib.util.spec_from_file_location("extract_spec_defaults", _MODULE_PATH)
mod = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = mod
_spec.loader.exec_module(mod)


def extract(src, members=None):
    return mod.extract_defaults(src, members or {})


def test_scalar_assign():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  thing->density = 1000;
  thing->fovy = 45;
  thing->random = 0.01;
}
"""
    fields = extract(src)["mjsThing"]
    assert fields["density"] == 1000
    assert fields["fovy"] == 45
    assert fields["random"] == 0.01


def test_array_assign_per_index_and_chained():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  thing->friction[0] = 1;
  thing->friction[1] = 0.005;
  thing->friction[2] = 0.0001;
  thing->rgba[0] = thing->rgba[1] = thing->rgba[2] = 0.5f;
  thing->rgba[3] = 1.0f;
  thing->dir[2] = -1;
}
"""
    fields = extract(src)["mjsThing"]
    assert fields["friction"] == [1, 0.005, 0.0001]
    assert fields["rgba"] == [0.5, 0.5, 0.5, 1.0]
    # gap before the assigned index is filled from the leading memset(0);
    # -1 as an array component stays literal, not a sentinel.
    assert fields["dir"] == [0, 0, -1]


def test_enum_assign_symbolic_and_auto():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  thing->type = mjGEOM_SPHERE;
  thing->limited = mjLIMITED_AUTO;
  thing->grouprange[1] = mjNGROUP-1;
}
"""
    fields = extract(src)["mjsThing"]
    assert fields["type"] == {"value": "mjGEOM_SPHERE"}
    assert fields["limited"] == {"value": "mjLIMITED_AUTO", "auto": True}
    assert fields["grouprange"] == [0, {"value": "mjNGROUP-1"}]


def test_nan_and_neg1_sentinels():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  thing->mass = mjNAN;
  thing->ipos[0] = mjNAN;
  thing->settotalmass = -1;
  thing->springlength[0] = thing->springlength[1] = -1;
}
"""
    fields = extract(src)["mjsThing"]
    assert fields["mass"] == {"sentinel": "mjNAN"}
    assert fields["ipos"] == [{"sentinel": "mjNAN"}]
    # scalar -1 is an "unset" sentinel; array -1 is a positional value.
    assert fields["settotalmass"] == {"sentinel": -1}
    assert fields["springlength"] == [-1, -1]


def test_memset_then_partial_assign():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  thing->quat[0] = 1;
}
"""
    members = {"mjsThing": ["pos", "quat", "mass", "info"]}
    fields = extract(src, members)["mjsThing"]
    assert fields["quat"] == [1]
    assert fields["pos"] == {"unset_by_memset": True}
    assert fields["mass"] == {"unset_by_memset": True}
    assert fields["info"] == {"unset_by_memset": True}


def test_helper_call_is_opaque():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  mj_defaultSolRefImp(thing->solref, thing->solimp);
}
"""
    fields = extract(src)["mjsThing"]
    stmt = "mj_defaultSolRefImp(thing->solref, thing->solimp)"
    assert fields["solref"] == {"opaque": stmt}
    assert fields["solimp"] == {"opaque": stmt}


def test_memcpy_from_local_string():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  char defaultlayout[sizeof(thing->gridlayout)] = "............";
  memcpy(thing->gridlayout, defaultlayout, sizeof(thing->gridlayout));
}
"""
    fields = extract(src)["mjsThing"]
    assert fields["gridlayout"] == "............"


def test_struct_type_from_signature_not_name():
    # The function is mjs_defaultSpec but the struct is mjSpec (from the parameter type).
    src = """\
void mjs_defaultSpec(mjSpec* spec) {
  memset(spec, 0, sizeof(mjSpec));
  spec->memory = -1;
}
"""
    defaults = extract(src)
    assert "mjSpec" in defaults
    assert defaults["mjSpec"]["memory"] == {"sentinel": -1}


def test_unclassifiable_assignment_raises_with_line_number():
    src = """\
void mjs_defaultThing(mjsThing* thing) {
  memset(thing, 0, sizeof(mjsThing));
  thing->mass = compute_mass(thing);
}
"""
    with pytest.raises(mod.SpecDefaultsError) as excinfo:
        extract(src)
    message = str(excinfo.value)
    assert ":3:" in message
    assert "compute_mass(thing)" in message


def test_parse_struct_members():
    header = """\
typedef struct mjsThing_ {
  mjsElement* element;
  double pos[3];
  char eulerseq[3];
  mjString* info;
} mjsThing;
"""
    members = mod.parse_struct_members(header)
    assert members["mjsThing"] == ["element", "pos", "eulerseq", "info"]


@pytest.mark.skipif(MUJOCO_SRC is None, reason=SKIP_REASON)
def test_integration_real_file_sanity():
    snapshot, stats = mod.build_snapshot(MUJOCO_SRC)
    defaults = snapshot["defaults"]

    assert len(defaults) >= 15
    assert defaults["mjsGeom"]["type"] == {"value": "mjGEOM_SPHERE"}

    for field in ("ipos", "fullinertia"):
        assert {"sentinel": "mjNAN"} in defaults["mjsBody"][field]

    assert defaults["mjsJoint"]["limited"] == {"value": "mjLIMITED_AUTO", "auto": True}

    # Helper defaults (mj_defaultSolRefImp / mj_defaultOption / ...) stay opaque, and every
    # opaque entry records its source statement.
    assert stats["opaque"] > 0
    assert all(text.startswith("mj_default") for text in stats["opaque_examples"])

    # run_sanity_checks must accept the real snapshot.
    mod.run_sanity_checks(defaults)
