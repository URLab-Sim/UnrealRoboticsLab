"""Front-end tests: upstream's MJCF schema -> the emitter AST.

The grammar itself is upstream's and is gated upstream (test/doc/mjcf_schema_test.py
checks the schema against the C enums and against a freshly constructed spec), so
nothing here re-checks what MJCF means. What these tests pin is the front end's
own contract: the naming rules and their overlay overrides, the variant-group
fold, the interleaved child lists and the alias admission that fills them, the
typing of references by namespace, symbolic arity resolution, and the drift gates
that fail an overlay entry the schema no longer supports.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from protospec_gen import frontend, overlay


@pytest.fixture(scope="module")
def ast() -> dict:
    return frontend.load_schema()


@pytest.fixture(scope="module")
def elements(ast: dict) -> dict:
    return {e["name"]: e for e in ast["elements"]}


@pytest.fixture(scope="module")
def enums(ast: dict) -> dict:
    return {e["name"]: e for e in ast["enums"]}


def fields(elem: dict) -> dict:
    return {f["name"]: f for f in elem["fields"]}


def children(elem: dict) -> dict:
    return {c["name"]: c for c in elem["children"]}


# --------------------------------------------------------------------------- #
# Naming                                                                       #
# --------------------------------------------------------------------------- #
def test_element_names_default_to_pascal_case(elements):
    assert "SensorContact" in elements
    assert "EqualityJoint" in elements
    assert elements["SensorContact"]["schema_name"] == "sensor_contact"


def test_overlay_renames_are_applied(elements):
    assert elements["Model"]["schema_name"] == "mujoco"
    assert elements["ActuatorGeneral"]["schema_name"] == "general"
    assert elements["VisualGlobal"]["schema_name"] == "global"


def test_skipped_elements_are_absent(elements):
    assert "Worldbody" not in elements


def test_xml_tag_comes_from_the_schema(elements):
    assert elements["SensorContact"]["xml"] == "contact"
    assert elements["EqualityDefault"]["xml"] == "equality"
    assert elements["Geom"]["xml"] == "geom"


def test_enum_names_and_keywords(enums):
    assert enums["GeomType"]["schema_name"] == "geomtype"
    members = {m["name"]: m["value"] for m in enums["GeomType"]["members"]}
    assert members["sphere"] == "sphere"
    # A keyword that is not an identifier gets an overlay enumerator; the value
    # stays the wire keyword.
    texture = {m["name"]: m["value"] for m in enums["TextureType"]["members"]}
    assert texture["twod"] == "2d"


def test_field_names_default_to_the_attribute(elements):
    geom = fields(elements["Geom"])
    assert "condim" in geom
    assert geom["condim"]["xml"] == "condim"


def test_keyword_attribute_is_renamed_and_keeps_its_wire_name(elements):
    geom = fields(elements["Geom"])
    assert "dclass" in geom and "class" not in geom
    assert geom["dclass"]["xml"] == "class"


# --------------------------------------------------------------------------- #
# Types                                                                        #
# --------------------------------------------------------------------------- #
def test_required_attributes_are_not_optional(elements):
    inertial = fields(elements["Inertial"])
    assert inertial["pos"]["optional"] is False
    assert inertial["mass"]["optional"] is False
    assert inertial["diaginertia"]["optional"] is True


def test_symbolic_arity_bounds_resolve_against_the_engine_constants(elements):
    solref = fields(elements["Geom"])["solref"]["type"]
    assert solref["arity"] == {"kind": "range", "min": 1, "max": 2}   # mjNREF
    solimp = fields(elements["Geom"])["solimp"]["type"]
    assert solimp["arity"] == {"kind": "range", "min": 1, "max": 5}   # mjNIMP


def test_flags_attributes_are_unbounded_keyword_sets(elements):
    data = fields(elements["Rangefinder"])["data"]["type"]
    assert data["kind"] == "named" and data["arity"]["kind"] == "unbounded"


def test_chars_arity_counts_characters_not_tokens(elements):
    eulerseq = fields(elements["Compiler"])["eulerseq"]
    assert eulerseq["type"] == {"kind": "prim", "prim": "string"}
    assert eulerseq["annotations"]["max_chars"] == 3


def test_defaults_carry_through(elements):
    geom = fields(elements["Geom"])
    assert geom["condim"]["default"] == {"kind": "scalar", "value": 3}
    assert geom["type"]["default"] == {"kind": "enum", "member": "sphere"}
    assert geom["friction"]["default"]["values"] == [1, 0.005, 0.0001]


# --------------------------------------------------------------------------- #
# References                                                                   #
# --------------------------------------------------------------------------- #
def test_single_declarer_namespace_types_the_reference_directly(elements):
    material = fields(elements["Geom"])["material"]["type"]
    assert material == {"kind": "ref", "target": "Material"}


def test_multi_declarer_namespace_types_the_reference_by_union(elements, ast):
    unions = {u["name"]: u["members"] for u in ast["unions"]}
    actuator = fields(elements["Actuatorpos"])["actuator"]["type"]
    assert actuator["target"] == "ActuatorAny"
    assert "Motor" in unions["ActuatorAny"]
    # joint is declared by both joint and freejoint, so it needs its own union.
    joint = fields(elements["Jointpos"])["joint"]["type"]
    assert joint["target"] == "JointAny"
    assert unions["JointAny"] == ["Joint", "FreeJoint"]


def test_dynamic_references_record_their_type_sibling(elements):
    objname = fields(elements["Framepos"])["objname"]
    assert objname["annotations"]["target_from"] == "objtype"


# --------------------------------------------------------------------------- #
# Variant groups                                                               #
# --------------------------------------------------------------------------- #
def test_orientation_folds_to_a_canonical_quat(elements):
    body = fields(elements["Body"])
    assert "quat" in body
    for spelling in ("axisangle", "xyaxes", "zaxis", "euler"):
        assert spelling not in body
    assert body["quat"]["annotations"]["resolver"] == "orientation"
    assert body["quat"]["annotations"]["aliases"] == "axisangle euler xyaxes zaxis"


def test_folded_spellings_are_accepted_as_input_aliases(elements):
    aliases = {a["attr"]: a["resolver"] for a in elements["Body"]["input_aliases"]}
    assert aliases == {
        "axisangle": "orientation",
        "xyaxes": "orientation",
        "zaxis": "orientation",
        "euler": "orientation",
    }


def test_inertia_spelling_folds_into_diaginertia(elements):
    inertial = fields(elements["Inertial"])
    assert "fullinertia" not in inertial
    assert inertial["diaginertia"]["annotations"]["aliases"] == "fullinertia"
    aliases = {a["attr"]: a["resolver"]
               for a in elements["Inertial"]["input_aliases"]}
    assert aliases["fullinertia"] == "inertia"


def test_no_variant_typed_field_survives(ast):
    for elem in ast["elements"]:
        for f in elem["fields"]:
            assert f["type"]["kind"] in ("prim", "ref", "named")


def test_element_level_alias_folds_into_a_child_list(elements):
    material = elements["Material"]
    assert "texture" not in fields(material)
    aliases = {a["attr"]: a["resolver"] for a in material["input_aliases"]}
    assert aliases == {"texture": "materiallayer"}
    assert "layers" in children(material)


# --------------------------------------------------------------------------- #
# Child lists                                                                  #
# --------------------------------------------------------------------------- #
def test_interleaved_sections_become_one_ordered_union_list(elements):
    actuator = children(elements["Actuator"])
    assert list(actuator) == ["actuators"]
    assert actuator["actuators"]["union"] == "ActuatorAny"
    assert actuator["actuators"]["tag"] == ""


def test_aliased_elements_join_the_list_that_admits_their_target(ast):
    unions = {u["name"]: u["members"] for u in ast["unions"]}
    body_children = unions["BodyChildAny"]
    assert "Body" in body_children
    assert "Frame" in body_children and "Replicate" in body_children


def test_aliased_elements_inherit_their_target_s_children(elements):
    for name in ("Frame", "Replicate"):
        assert children(elements[name]) == children(elements["Body"])


def test_homogeneous_lists_keep_the_child_tag(elements):
    asset = children(elements["Asset"])
    assert asset["meshes"]["element"] == "Mesh"
    assert asset["meshes"]["tag"] == "mesh"


def test_world_body_slot_carries_the_overlay_tag(elements):
    worldbody = children(elements["Model"])["worldbody"]
    assert worldbody["element"] == "Body"
    assert worldbody["tag"] == "worldbody"


# --------------------------------------------------------------------------- #
# Presence constraints                                                         #
# --------------------------------------------------------------------------- #
def test_constraints_carry_through_from_the_schema(elements):
    camera = elements["Camera"]["constraints"]
    assert {"kind": "exclusive", "bundles": [["fovy"], ["sensorsize"]]} in camera


def test_group_constraints_reach_the_using_element(elements):
    framepos = elements["Framepos"]["constraints"]
    assert {"kind": "together", "bundles": [["reftype"], ["refname"]]} in framepos


def test_constraints_over_folded_spellings_are_dropped(elements):
    # inertial's exclusive row spans fullinertia and the five orientation
    # spellings; none of those is a stored field, so no row survives.
    for con in elements["Inertial"].get("constraints", []):
        for bundle in con["bundles"]:
            assert "fullinertia" not in bundle


def test_multi_attribute_bundles_survive(elements):
    connect = elements["Connect"]["constraints"]
    kinds = {c["kind"] for c in connect}
    assert {"exclusive", "oneof", "together"} <= kinds
    exclusive = next(c for c in connect if c["kind"] == "exclusive")
    assert ["site1", "site2"] in exclusive["bundles"]


# --------------------------------------------------------------------------- #
# mjSpec bindings                                                              #
# --------------------------------------------------------------------------- #
def test_elements_carry_their_spec_struct(elements):
    assert elements["Geom"]["spec"] == "mjsGeom"
    assert elements["Gyro"]["spec"] == "mjsSensor"
    assert "spec" not in elements["Model"]


def test_field_bindings_carry_through(elements):
    geom = fields(elements["Geom"])
    assert geom["mesh"]["annotations"]["spec_field"] == "meshname"
    assert geom["user"]["annotations"]["spec_field"] == "userdata"


def test_set_constants_are_recorded(elements):
    consts = {c["field"]: c["value"] for c in elements["Gyro"]["consts"]}
    assert consts == {"type": "mjSENS_GYRO", "objtype": "mjOBJ_SITE"}


# --------------------------------------------------------------------------- #
# Drift gates                                                                  #
# --------------------------------------------------------------------------- #
def _with_overlay(monkeypatch, name, value):
    monkeypatch.setattr(overlay, name, value)


def test_stale_element_name_is_rejected(monkeypatch):
    _with_overlay(monkeypatch, "ELEMENT_NAMES",
                  dict(overlay.ELEMENT_NAMES, no_such_element="Nope"))
    with pytest.raises(frontend.OverlayError, match="no longer declares"):
        frontend.load_schema()


def test_stale_enum_name_is_rejected(monkeypatch):
    _with_overlay(monkeypatch, "ENUM_NAMES",
                  dict(overlay.ENUM_NAMES, no_such_enum="Nope"))
    with pytest.raises(frontend.OverlayError, match="no longer declares"):
        frontend.load_schema()


def test_stale_attribute_key_is_rejected(monkeypatch):
    notes = dict(overlay.READ_NOTES)
    notes[("geom", "no_such_attr")] = "x"
    _with_overlay(monkeypatch, "READ_NOTES", notes)
    with pytest.raises(frontend.OverlayError, match="no longer declares"):
        frontend.load_schema()


def test_colliding_cpp_names_are_rejected(monkeypatch):
    _with_overlay(monkeypatch, "ELEMENT_NAMES",
                  dict(overlay.ELEMENT_NAMES, geom="Site"))
    with pytest.raises(frontend.OverlayError, match="both map to"):
        frontend.load_schema()


def test_interleave_naming_a_non_child_is_rejected(monkeypatch):
    rows = dict(overlay.INTERLEAVE)
    rows["tendon"] = [("tendons", "TendonAny", ["spatial", "geom"])]
    _with_overlay(monkeypatch, "INTERLEAVE", rows)
    with pytest.raises(frontend.OverlayError, match="not children of"):
        frontend.load_schema()


def test_unhandled_custom_facet_is_rejected(monkeypatch):
    notes = dict(overlay.READ_NOTES)
    del notes[("compiler", "strippath")]
    _with_overlay(monkeypatch, "READ_NOTES", notes)
    with pytest.raises(frontend.OverlayError, match="reading=custom"):
        frontend.load_schema()


def test_every_custom_facet_is_dispositioned(ast):
    # The positive form of the gate above: nothing in the schema's custom facets
    # is silently unhandled, and no disposition is dead.
    schema_keys = set()
    for elem in ast["elements"]:
        for f in elem["fields"]:
            ann = f.get("annotations", {})
            if ann.get("reading") == "custom":
                schema_keys.add((elem["schema_name"], f["xml"]))
    dispositioned = set(overlay.READ_HANDLERS) | set(overlay.READ_NOTES)
    assert schema_keys <= dispositioned


def test_every_read_handler_binds_a_resolver_name(ast):
    named = set()
    for elem in ast["elements"]:
        for f in elem["fields"]:
            resolver = f.get("annotations", {}).get("resolver")
            if resolver:
                named.add(resolver)
        for alias in elem.get("input_aliases", []):
            named.add(alias["resolver"])
    assert named == set(overlay.READ_HANDLERS.values()) | {"materiallayer"}


# --------------------------------------------------------------------------- #
# Addition-side gates                                                          #
# --------------------------------------------------------------------------- #
# The gates above all read the overlay and ask the schema about it, so an
# upstream ADDITION the overlay has never heard of passes every one of them.
# These drive the three gates that ask the question the other way round, each
# against a synthetic schema carrying exactly one addition.
def _schema_with(tmp_path: Path, old: str, new: str) -> str:
    """A MuJoCo root whose schema has one edit; everything else is upstream's.

    Only two files are read from the root -- the schema and `mjmodel.h`, for the
    symbolic arity bounds -- so the copy is two files rather than a checkout.
    Upstream's schema PARSER is loaded once from the real checkout and is shared,
    which is what makes this cheap enough to do per test.
    """
    src = Path(frontend.mujoco_src())
    text = (src / "src" / "xml" / "mjcf.schema").read_text(encoding="utf-8")
    assert text.count(old) == 1, f"anchor {old!r} is not unique in the schema"

    root = tmp_path / "mujoco"
    (root / "src" / "xml").mkdir(parents=True)
    (root / "include" / "mujoco").mkdir(parents=True)
    (root / "src" / "xml" / "mjcf.schema").write_text(
        text.replace(old, new), encoding="utf-8")
    shutil.copyfile(src / "include" / "mujoco" / "mjmodel.h",
                    root / "include" / "mujoco" / "mjmodel.h")
    return str(root)


_SITE = "element site : mjsSite {"


def test_new_angle_attribute_must_be_classified(tmp_path):
    root = _schema_with(tmp_path, _SITE, _SITE + "\n  swingangle : double = 0")
    with pytest.raises(frontend.OverlayError, match=r"site\.swingangle .* angle"):
        frontend.load_schema(root)


def test_new_angle_attribute_can_be_waived(monkeypatch, tmp_path):
    root = _schema_with(tmp_path, _SITE, _SITE + "\n  swingangle : double = 0")
    monkeypatch.setattr(
        overlay, "NOT_ANGLE",
        {("site", "swingangle"): "a synthetic attribute, not an angle"})
    ast = frontend.load_schema(root)
    site = next(e for e in ast["elements"] if e["schema_name"] == "site")
    assert "swingangle" in {f["xml"] for f in site["fields"]}


def test_new_reference_attribute_must_be_classified(tmp_path):
    root = _schema_with(tmp_path, _SITE, _SITE + "\n  targetname : string")
    with pytest.raises(frontend.OverlayError,
                       match=r"site\.targetname .* reference"):
        frontend.load_schema(root)


def test_new_reference_attribute_can_be_waived(monkeypatch, tmp_path):
    root = _schema_with(tmp_path, _SITE, _SITE + "\n  targetname : string")
    monkeypatch.setattr(
        overlay, "NOT_TARGET",
        {("site", "targetname"): "a synthetic attribute, not a reference"})
    assert frontend.load_schema(root)


def test_new_repeatable_child_must_join_the_interleave_row(tmp_path):
    # <tendon> is one ordered heterogeneous list; a repeatable child outside it
    # is written after the whole list and shifts every id it carries.
    root = _schema_with(tmp_path, "element tendon {",
                        "element tendon {\n  child geom *")
    with pytest.raises(frontend.OverlayError, match=r"tendon\.geom repeats"):
        frontend.load_schema(root)


def test_one_at_a_time_child_may_stay_outside_the_row(ast):
    # The other half of the gate: <body> carries <inertial> once, so it has no
    # order to be wrong about and needs no row. The whole suite loading proves
    # it, and this states why.
    body = next(e for e in ast["elements"] if e["schema_name"] == "body")
    assert "inertial" in {c["name"] for c in body["children"]}
    assert "subtree" in {c["name"] for c in body["children"]}


def test_reclassified_attribute_is_rejected(monkeypatch):
    _with_overlay(monkeypatch, "ANGLE_ATTRS",
                  overlay.ANGLE_ATTRS - {("joint", "range")})
    with pytest.raises(frontend.OverlayError, match=r"joint\.range was recorded"):
        frontend.load_schema()


def test_stale_waiver_is_rejected(monkeypatch):
    monkeypatch.setattr(overlay, "NOT_ANGLE",
                        {("site", "no_such_attr"): "stale"})
    with pytest.raises(frontend.OverlayError, match="no longer declares"):
        frontend.load_schema()


def test_classified_baseline_covers_every_live_attribute(ast):
    baseline = frontend._read_baseline()
    live = {f"{elem['schema_name']}.{f['xml']}"
            for elem in ast["elements"] for f in elem["fields"]}
    # Folded spellings have no field of their own, so the AST is the smaller
    # set; nothing it carries may be missing from the baseline.
    assert live <= set(baseline)
    assert set(baseline) >= {"joint.range", "framepos.objname"}
    assert baseline["joint.range"] == "angle"
    assert baseline["framepos.objname"] == "target"
