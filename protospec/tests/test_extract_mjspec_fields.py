"""Tests for the mjspec.h field-inventory extractor.

Unit tests run against small embedded header excerpts (no dependency on the vendored tree).
One integration test parses the real vendored header when it is present.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from _mujoco_src import MUJOCO_SRC, SKIP_REASON

_MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "tools" / "bootstrap" / "extract_mjspec_fields.py"
)
_spec = importlib.util.spec_from_file_location("extract_mjspec_fields", _MODULE_PATH)
mod = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = mod  # dataclasses resolves string annotations via sys.modules
_spec.loader.exec_module(mod)

MACROS = {"mjNREF": 2, "mjNIMP": 5, "mjNPOLY": 2}


TYPEDEF_BLOCK = """\
#ifdef __cplusplus
  using mjString      = std::string;
  using mjStringVec   = std::vector<std::string>;
  using mjIntVec      = std::vector<int>;
  using mjIntVecVec   = std::vector<std::vector<int>>;
  using mjFloatVec    = std::vector<float>;
  using mjFloatVecVec = std::vector<std::vector<float>>;
  using mjDoubleVec   = std::vector<double>;
  using mjByteVec     = std::vector<std::byte>;
#else
  typedef void mjString;
#endif
"""

SMALL_ENUM = """\
typedef enum mjtLimited_ {         // type of limit specification
  mjLIMITED_FALSE = 0,             // not limited
  mjLIMITED_TRUE,                  // limited
  mjLIMITED_AUTO,                  // limited inferred from presence of range
} mjtLimited;
"""

SMALL_STRUCT = """\
typedef struct mjsThing_ {         // a thing
  mjsElement* element;             // element type
  double pos[3];                   // position
  mjtNum solref[mjNREF];           // solver reference
  double stiffness[mjNPOLY+1];     // stiffness coefficients
  mjString* name;                  // the name
  mjStringVec* tags;               // tags
  mjDoubleVec* userdata;           // user data
  mjIntVecVec* vertid;             // vertex ids
  mjtByte active;                  // is active
  mjsOrientation alt;              // alternative orientation
  mjOption option;                 // physics options
  mjsGeom* geom;                   // geom defaults
} mjsThing;
"""


def _prelude() -> str:
    return TYPEDEF_BLOCK + "\n"


def test_handle_types_alias_table():
    aliases = mod.parse_handle_types(TYPEDEF_BLOCK.splitlines())
    assert aliases["mjString"] == "std::string"
    assert aliases["mjIntVecVec"] == "std::vector<std::vector<int>>"
    assert aliases["mjByteVec"] == "std::vector<std::byte>"
    assert len(aliases) == 8


def test_parse_small_enum():
    enums, structs, _ = mod.parse_header(_prelude() + SMALL_ENUM)
    assert len(enums) == 1
    enum = enums[0]
    assert enum.name == "mjtLimited"
    assert enum.tag == "mjtLimited_"
    assert enum.doc == "type of limit specification"
    names = [e.name for e in enum.enumerators]
    assert names == ["mjLIMITED_FALSE", "mjLIMITED_TRUE", "mjLIMITED_AUTO"]
    assert enum.enumerators[0].value == "0"
    assert enum.enumerators[1].value is None
    assert enum.enumerators[2].comment == "limited inferred from presence of range"


def test_parse_small_struct_kinds_and_extents():
    enums, structs, _ = mod.parse_header(_prelude() + SMALL_STRUCT)
    assert len(structs) == 1
    struct = structs[0]
    assert struct.name == "mjsThing"
    mod.classify_and_resolve(structs, MACROS)
    kinds = {f.name: f.kind for f in struct.fields}
    assert kinds == {
        "element": "element_ptr",
        "pos": "fixed_array",
        "solref": "fixed_array",
        "stiffness": "fixed_array",
        "name": "string",
        "tags": "string_vec",
        "userdata": "double_vec",
        "vertid": "nested_vec",
        "active": "scalar",
        "alt": "struct",
        "option": "struct",
        "geom": "element_ptr",
    }
    by_name = {f.name: f for f in struct.fields}
    assert by_name["pos"].array_size == 3
    assert by_name["solref"].array_size == 2  # mjNREF
    assert by_name["stiffness"].array_size == 3  # mjNPOLY + 1
    assert struct.embeds_engine() == ["mjOption"]


def test_used_macros_reported():
    _, structs, _ = mod.parse_header(_prelude() + SMALL_STRUCT)
    used = mod.classify_and_resolve(structs, MACROS)
    assert used == {"mjNREF": 2, "mjNPOLY": 2}


def test_field_comment_preserved():
    _, structs, _ = mod.parse_header(_prelude() + SMALL_STRUCT)
    mod.classify_and_resolve(structs, MACROS)
    by_name = {f.name: f for f in structs[0].fields}
    assert by_name["solref"].comment == "solver reference"
    assert by_name["element"].comment == "element type"


def test_unrecognized_type_raises_with_line():
    bad = _prelude() + (
        "typedef struct mjsBad_ {         // bad struct\n"
        "  mjsElement* element;           // element type\n"
        "  WeirdType foo;                 // not a known type\n"
        "} mjsBad;\n"
    )
    _, structs, _ = mod.parse_header(bad)
    with pytest.raises(mod.MjSpecParseError) as exc:
        mod.classify_and_resolve(structs, MACROS)
    msg = str(exc.value)
    assert "WeirdType" in msg
    # The bad field is on the 3rd line of the struct excerpt, whose absolute line
    # depends on the prelude; assert a line number is cited.
    assert "line " in msg


def test_unknown_macro_extent_raises():
    bad = _prelude() + (
        "typedef struct mjsBad_ {         // bad struct\n"
        "  mjsElement* element;           // element type\n"
        "  double vals[mjNBOGUS];         // bogus extent\n"
        "} mjsBad;\n"
    )
    _, structs, _ = mod.parse_header(bad)
    with pytest.raises(mod.MjSpecParseError) as exc:
        mod.classify_and_resolve(structs, MACROS)
    assert "mjNBOGUS" in str(exc.value)


def test_unparseable_field_line_raises():
    bad = _prelude() + (
        "typedef struct mjsBad_ {         // bad struct\n"
        "  this is not a field line\n"
        "} mjsBad;\n"
    )
    with pytest.raises(mod.MjSpecParseError) as exc:
        mod.parse_header(bad)
    assert "unparseable field" in str(exc.value)


def test_pointer_type_full_matrix():
    struct_src = _prelude() + (
        "typedef struct mjsP_ {           // pointers\n"
        "  mjIntVec* a;                   // ints\n"
        "  mjFloatVec* b;                 // floats\n"
        "  mjFloatVecVec* c;              // nested floats\n"
        "  mjByteVec* d;                  // bytes\n"
        "} mjsP;\n"
    )
    _, structs, _ = mod.parse_header(struct_src)
    mod.classify_and_resolve(structs, MACROS)
    kinds = {f.name: f.kind for f in structs[0].fields}
    assert kinds == {"a": "int_vec", "b": "float_vec", "c": "nested_vec", "d": "byte_vec"}


@pytest.mark.skipif(MUJOCO_SRC is None, reason=SKIP_REASON)
def test_integration_real_header():
    snapshot = mod.build_snapshot(MUJOCO_SRC)

    assert snapshot["source"]["file"] == "include/mujoco/mjspec.h"
    assert snapshot["source"]["mujoco_version"].count(".") == 2

    handles = snapshot["handle_types"]
    assert handles["mjString"] == "std::string"
    assert len(handles) == 8

    structs = snapshot["structs"]
    assert len(structs) >= 25

    by_name = {s["name"]: s for s in structs}
    body = by_name["mjsBody"]
    body_fields = {f["name"] for f in body["fields"]}
    assert {"pos", "quat", "alt", "fullinertia"} <= body_fields

    wrap = by_name["mjsWrap"]
    assert len(wrap["fields"]) <= 4

    spec = by_name["mjSpec"]
    assert set(spec["embeds_engine"]) >= {"mjOption", "mjVisual", "mjStatistic"}

    for struct in structs:
        for f in struct["fields"]:
            assert f["kind"], f"{struct['name']}.{f['name']} unclassified"

    macros = snapshot["macros"]
    assert macros["mjNREF"] == 2
    assert macros["mjNEQDATA"] == 11
    assert macros["mjNPOLY"] == 2

    enum_names = {e["name"] for e in snapshot["enums"]}
    assert {"mjtGeomInertia", "mjtLimited", "mjtOrientation"} <= enum_names
