# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Unit tests for the per-sensor objtype/reftype extractor in
build_mjcf_schema_snapshot.py.

The extractor scrapes xml_native_reader.cc's ``Sensor()`` if/else cascade
so codegen rules don't have to hand-list each sensor's mjsSensor object
defaults. Tests cover the four shape classes the extractor produces:
literal default, from_xml, computed (multi-attr derivation), and the
nested-branch suppression case (distance/normal/fromto share an outer
branch whose inner ``if`` would otherwise overwrite the entry).
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)

from build_mjcf_schema_snapshot import _extract_sensor_per_type  # noqa: E402


_FAKE_SOURCE = r"""
void mjXReader::Sensor(XMLElement* section) {
  while (elem) {
    mjsSensor* sensor = mjs_addSensor(spec);
    string type = elem->Value();
    string text, name, objname, refname;

    if (type == "touch") {
      sensor->type = mjSENS_TOUCH;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "framepos") {
      sensor->type = mjSENS_FRAMEPOS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      }
    } else if (type == "distance" || type == "normal" || type == "fromto") {
      bool has_body1 = ReadAttrTxt(elem, "body1", objname);
      sensor->objtype = has_body1 ? mjOBJ_BODY : mjOBJ_GEOM;
      ReadAttrTxt(elem, "body2", refname);
      sensor->reftype = mjOBJ_BODY;
      if (type == "distance") {
        sensor->type = mjSENS_GEOMDIST;
      } else if (type == "normal") {
        sensor->type = mjSENS_GEOMNORMAL;
      } else {
        sensor->type = mjSENS_GEOMFROMTO;
      }
    } else if (type == "clock") {
      sensor->type = mjSENS_CLOCK;
      sensor->objtype = mjOBJ_UNKNOWN;
    }
    elem = NextSiblingElement(elem);
  }
}
"""


def test_literal_objtype_with_site_name_read():
    result = _extract_sensor_per_type(_FAKE_SOURCE)
    assert result["touch"] == {
        "mj_type":  "mjSENS_TOUCH",
        "objtype":  "mjOBJ_SITE",
        "reftype":  None,
        "obj_attr": "site",
        "ref_attr": None,
        "computed": False,
    }


def test_from_xml_objtype_picks_up_objname_attr():
    result = _extract_sensor_per_type(_FAKE_SOURCE)
    framepos = result["framepos"]
    assert framepos["objtype"] == "from_xml"
    assert framepos["obj_attr"] == "objname"
    assert framepos["ref_attr"] == "refname"
    assert framepos["computed"] is False


def test_multi_name_branch_emits_one_entry_per_name():
    result = _extract_sensor_per_type(_FAKE_SOURCE)
    for name in ("distance", "normal", "fromto"):
        assert name in result, f"missing {name}"
        assert result[name]["computed"] is True
        assert result[name]["objtype"] == "computed"
        # All three share the same body1/body2 attr-read.
        assert result[name]["obj_attr"] == "body1"
        assert result[name]["ref_attr"] == "body2"


def test_inner_if_inside_multi_name_branch_does_not_overwrite():
    """Regression: the body of the distance/normal/fromto branch contains
    ``if (type == "distance")`` and ``else if (type == "normal")`` which set
    only ->type. Those inner matches would override the outer entry's
    obj_attr/ref_attr with None if the extractor walked them naively."""
    result = _extract_sensor_per_type(_FAKE_SOURCE)
    assert result["distance"]["obj_attr"] == "body1"
    assert result["normal"]["obj_attr"] == "body1"


def test_clock_has_unknown_objtype_no_name_read():
    result = _extract_sensor_per_type(_FAKE_SOURCE)
    assert result["clock"] == {
        "mj_type":  "mjSENS_CLOCK",
        "objtype":  "mjOBJ_UNKNOWN",
        "reftype":  None,
        "obj_attr": None,
        "ref_attr": None,
        "computed": False,
    }


def test_missing_sensor_method_returns_empty():
    assert _extract_sensor_per_type("// no Sensor method here") == {}


# ---------- sensor switch + TagToType codegen ----------------------------

def test_sensor_switch_body_emits_static_objtype_when_present():
    from generate_ue_components import _emit_sensor_switch_block  # noqa
    cat_rules = {
        "type_enum_name": "EMjSensorType",
        "subtypes": [
            {"key": "touch",     "enum_value": "Touch"},
            {"key": "framepos",  "enum_value": "FramePos"},
        ],
        "default_subtype_key": "touch",
    }
    sensor_per_type = {
        "touch":    {"mj_type": "mjSENS_TOUCH",    "objtype": "mjOBJ_SITE", "reftype": None},
        "framepos": {"mj_type": "mjSENS_FRAMEPOS", "objtype": "from_xml",   "reftype": None},
    }
    out = _emit_sensor_switch_block(cat_rules, sensor_per_type)
    # Touch gets static objtype literal.
    assert "case EMjSensorType::Touch: Element->type = mjSENS_TOUCH; Element->objtype = mjOBJ_SITE; break;" in out
    # framepos has objtype=from_xml -> NO static objtype emitted (handled in post-switch block).
    assert "case EMjSensorType::FramePos: Element->type = mjSENS_FRAMEPOS; break;" in out
    # Default fallback matches the chosen subtype.
    assert "default: Element->type = mjSENS_TOUCH; Element->objtype = mjOBJ_SITE; break;" in out


def test_sensor_switch_body_honours_case_override():
    from generate_ue_components import _emit_sensor_switch_block
    cat_rules = {
        "type_enum_name": "EMjSensorType",
        "subtypes": [
            {"key": "rangefinder", "enum_value": "RangeFinder",
             "case_body_override": "Element->type = mjSENS_RANGEFINDER; Element->objtype = X;"},
        ],
        "default_subtype_key": "rangefinder",
    }
    out = _emit_sensor_switch_block(cat_rules, {"rangefinder": {"mj_type": "mjSENS_RANGEFINDER"}})
    # Override appears verbatim; the emitter's automatic mj_type / objtype
    # lines are SKIPPED for this case.
    assert "Element->objtype = X;" in out
    assert out.count("RangeFinder") >= 1


def test_sensor_tag_to_type_map_uses_xml_key_verbatim():
    from generate_ue_components import _emit_sensor_tag_to_type_block
    cat_rules = {
        "type_enum_name": "EMjSensorType",
        "subtypes": [
            {"key": "touch",            "enum_value": "Touch"},
            {"key": "e_potential",      "enum_value": "EPotential"},
            {"key": "jointactuatorfrc", "enum_value": "JointActFrc"},
        ],
    }
    out = _emit_sensor_tag_to_type_block(cat_rules)
    assert '{TEXT("touch"), EMjSensorType::Touch},' in out
    # Underscore-containing XML keys preserved literally.
    assert '{TEXT("e_potential"), EMjSensorType::EPotential},' in out
    # XML key and UE enum_value can diverge; map uses XML key.
    assert '{TEXT("jointactuatorfrc"), EMjSensorType::JointActFrc},' in out


def test_real_snapshot_covers_every_schema_sensor():
    """End-to-end: the snapshot shipped in Scripts/ should have one
    per_type entry for every sensor_types entry — and vice versa. Catches
    drift between the static MJCF[] schema and the parser cascade."""
    import json
    plugin_root = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
    snap = json.load(open(os.path.join(plugin_root, "Scripts", "codegen", "snapshots", "mjcf_schema_snapshot.json")))
    schema_names = set(snap["sensor_types"])
    per_type_names = set(snap.get("sensor_per_type", {}).keys())
    assert schema_names == per_type_names, (
        f"schema/per_type drift: only-in-schema={sorted(schema_names - per_type_names)}; "
        f"only-in-per_type={sorted(per_type_names - schema_names)}"
    )
