"""Unreal emitter tests: coverage, the hard UHT rules, and --check parity.

These guard the schema -> UE codegen contract without an engine. Two of them are
the ones that matter:

* **coverage** -- every element has a UCLASS, every attribute a UPROPERTY and an
  accessor quartet, every enum a UENUM with its keywords in schema order. It is
  read off the schema rather than pinned to a count, so a schema that grows an
  element cannot silently lose it.
* **the hard rules** -- a Blueprint specifier on a TOptional UPROPERTY, a
  Replicated one, a TOptional accessor parameter, and indexed access to a
  fixed-arity vector type. UHT accepts three of those four, so a compiler is not
  a substitute for asserting them here.

The quaternion case gets its own test because it is the one mistake that
compiles everywhere and is wrong everywhere: MJCF authors [w, x, y, z] and FQuat
constructs from (X, Y, Z, W).
"""

from __future__ import annotations

import os
import re

import pytest

from protospec_gen import emit_ue, load_schema, overlay, overlay_ue
from protospec_gen.emit_ue import PRIVATE, PUBLIC, TESTS, UeError, UeSchema

TOP_LEVEL = {
    f"{PUBLIC}/MjElements.gen.h",
    f"{PUBLIC}/MjEnums.gen.h",
    f"{PUBLIC}/MjKeywords.gen.h",
    f"{PUBLIC}/MjVisit.gen.h",
    f"{PUBLIC}/MjStorage.gen.h",
    f"{PUBLIC}/MjDefaults.gen.h",
    f"{PUBLIC}/MjDispatch.gen.h",
    f"{PUBLIC}/MjReflect.gen.h",
    f"{PUBLIC}/MjProfile.gen.h",
    f"{PUBLIC}/MjSpecWrite.gen.h",
    f"{PRIVATE}/MjSpecWrite.gen.cpp",
    f"{PRIVATE}/MjKeywords.gen.cpp",
    f"{PRIVATE}/MjDefaults.gen.cpp",
    f"{PRIVATE}/MjDispatch.gen.cpp",
    f"{PRIVATE}/MjReflect.gen.cpp",
    f"{TESTS}/MjGenCoverage.gen.cpp",
    "Scripts/regen_ue_profile.ps1",
    "Scripts/regen_ue_profile.sh",
}


def element_header_path(schema: UeSchema, name: str) -> str:
    """Where the emitter puts one element's header.

    Derived from `element_family` rather than spelled out, so the grouping is
    read from the emitter that owns it and a regrouping moves these tests with
    it instead of breaking them.
    """
    family = emit_ue.element_family(schema, name)
    sub = f"{family}/" if family else ""
    return f"{PUBLIC}/Elements/{sub}Mj{name}.gen.h"


@pytest.fixture(scope="module")
def files() -> dict[str, str]:
    return emit_ue.generate()


@pytest.fixture(scope="module")
def schema() -> UeSchema:
    return UeSchema(load_schema())


# --------------------------------------------------------------------------- #
# Manifest and coverage                                                        #
# --------------------------------------------------------------------------- #
def test_manifest(files, schema):
    """One header per element, plus the aggregate files and the regen scripts."""
    element_headers = {element_header_path(schema, e["name"])
                       for e in schema.elements}
    assert TOP_LEVEL <= set(files)
    assert element_headers <= set(files)
    assert set(files) == TOP_LEVEL | element_headers


def test_every_element_has_a_class(files, schema):
    for e in schema.elements:
        text = files[element_header_path(schema, e["name"])]
        assert f"class URLAB_API {schema.cls[e['name']]} : public " \
               "UMjNodeComponent" in text


def test_element_class_name_is_the_element_name(files, schema):
    """An element is its own class. Only the five URLab subclasses by hand give
    up the canonical name, and they give it up to the hand class."""
    for e in schema.elements:
        name = e["name"]
        expected = f"UMj{name}Base" if name in emit_ue.HAND_SUBCLASSED \
            else f"UMj{name}"
        assert schema.cls[name] == expected, name


def test_only_hand_bases_are_kept_out_of_the_picker(files, schema):
    """BlueprintSpawnableComponent is what puts a component in Add Component.
    The five hand bases must not be there; the hand subclass is."""
    for e in schema.elements:
        text = files[element_header_path(schema, e["name"])]
        head = text.split(f"class URLAB_API {schema.cls[e['name']]}", 1)[0]
        spawnable = "BlueprintSpawnableComponent" in head
        assert spawnable is not schema.is_hand_base(e["name"]), e["name"]


def test_hand_base_list_must_name_real_elements():
    doc = load_schema()
    doc["elements"] = [e for e in doc["elements"] if e["name"] != "Geom"]
    with pytest.raises(UeError, match="Geom"):
        UeSchema(doc)


def test_every_attribute_has_a_property_and_a_quartet(files, schema):
    for e in schema.elements:
        text = files[element_header_path(schema, e["name"])]
        for _fid, f in schema.stored_fields(e):
            member = schema.member(e, f)
            assert re.search(rf"^\t\S.* {member};$", text, re.M), \
                f"{e['name']}.{f['name']}: no UPROPERTY"
            for verb in ("Get", "Set", "Has"):
                assert f"{verb}{member}(" in text, \
                    f"{e['name']}.{f['name']}: no {verb}{member}"
            if f["optional"]:
                assert f"Clear{member}()" in text
            else:
                # A required attribute has no unset state to clear into.
                assert f"Clear{member}()" not in text


def test_identity_attribute_is_not_a_property(files, schema):
    """`name` is UMjNodeComponent::MjName, not a per-element property, but it
    still reaches Visit at its schema field id."""
    visit = files[f"{PUBLIC}/MjVisit.gen.h"]
    for e in schema.elements:
        text = files[element_header_path(schema, e["name"])]
        for fid, f in enumerate(e["fields"]):
            if not UeSchema.is_identity(f):
                continue
            assert "MjName;" not in text
            assert f'Vis.field({fid}, "{f["xml"]}", Element.MjName);' in visit


def test_every_enum_has_a_uenum_in_schema_order(files, schema):
    text = files[f"{PUBLIC}/MjEnums.gen.h"]
    for e in schema.enums:
        name = schema.enum[e["name"]]
        block = text.split(f"enum class {name} : uint8", 1)
        assert len(block) == 2, f"enum {e['name']}: no UENUM"
        body = block[1].split("};", 1)[0]
        keywords = re.findall(r'UMETA\(DisplayName = "([^"]*)"\)', body)
        assert keywords == [m["value"] for m in e["members"]]


def test_visit_ids_are_the_schema_ids(files, schema):
    """Field ids are shared with every other profile and with the reflect and
    binding tables, so they are positional and complete."""
    visit = files[f"{PUBLIC}/MjVisit.gen.h"]
    for e in schema.elements:
        head = f"void Visit({schema.cls[e['name']]}& Element, V&& Vis)"
        body = visit.split(head, 1)[1].split("\n}", 1)[0]
        ids = [int(m) for m in re.findall(r"Vis\.field\((\d+),", body)]
        assert ids == list(range(len(e["fields"]))), e["name"]


# --------------------------------------------------------------------------- #
# The hard rules                                                               #
# --------------------------------------------------------------------------- #
def test_no_blueprint_specifier_on_any_property(files):
    """UHT accepts BlueprintReadWrite on a TOptional and emits a broken
    bad_type pin, so the rule is asserted here rather than by the build."""
    for path, text in files.items():
        for line in text.splitlines():
            if line.lstrip().startswith("UPROPERTY("):
                assert "Blueprint" not in line, f"{path}: {line.strip()}"
                assert "Replicated" not in line, f"{path}: {line.strip()}"


def test_accessor_parameters_are_plain_and_undefaulted(files):
    for path, text in files.items():
        for m in re.finditer(r"\b(?:Get|Set|Has|Clear)\w+\(([^)]*)\)", text):
            params = m.group(1)
            assert "TOptional<" not in params, f"{path}: {m.group(0)}"
            assert "=" not in params, f"{path}: {m.group(0)}"


def test_no_indexed_access_to_a_fixed_arity_type(files):
    for path, text in files.items():
        assert not re.search(
            r"\b(?:FVector|FVector2D|FQuat|FLinearColor)\s*\w*\s*\[\s*\d",
            text), path


def test_output_is_ascii(files):
    for path, text in files.items():
        assert text.isascii(), path


def test_lint_rejects_a_blueprint_optional():
    bad = {"x.h": "\tUPROPERTY(EditAnywhere, BlueprintReadWrite)\n"
                  "\tTOptional<double> Mass;\n"}
    with pytest.raises(UeError, match="Blueprint"):
        emit_ue.lint(bad)


def test_lint_rejects_an_optional_accessor_parameter():
    bad = {"x.h": "\tvoid SetMass(TOptional<double> InValue) { }\n"}
    with pytest.raises(UeError, match="TOptional accessor parameter"):
        emit_ue.lint(bad)


def test_lint_rejects_indexed_fixed_access():
    bad = {"x.h": "\tOut[0] = FMjQuatRot Value[0];\n"}
    with pytest.raises(UeError, match="indexed access"):
        emit_ue.lint(bad)


# --------------------------------------------------------------------------- #
# Storage mapping                                                              #
# --------------------------------------------------------------------------- #
def test_quaternion_default_is_verbatim(files, schema):
    """An MJCF quaternion is stored as MJCF authors it, [w, x, y, z]. The
    identity is the witness: {1, 0, 0, 0} must emit W first, unpermuted."""
    text = files[f"{PRIVATE}/MjDefaults.gen.cpp"]
    assert "FMjQuatRot(/*W=*/1.0, /*X=*/0.0, /*Y=*/0.0, /*Z=*/0.0)" in text
    quat_defaults = re.findall(r"FMjQuatRot\([^)]*\)", text)
    assert quat_defaults, "no quaternion default emitted"
    for d in quat_defaults:
        assert d.startswith("FMjQuatRot(/*W=*/"), d


def test_no_authored_rotation_is_an_unreal_one(files):
    """FQuat is an Unreal rotation: left-handed, [x, y, z, w]. An MJCF `quat`
    is neither, and the two differ by a permutation AND a sign flip. Storing
    one as the other type-checks against every rotation API and is wrong, so
    the emitted profile must not name FQuat anywhere."""
    for name, text in files.items():
        assert "FQuat" not in text, name


def test_no_fvector_storage_path_is_emitted(files):
    """`value_type` can never answer FVector, so the profile must not carry a
    way to store one either. A `fixed<FVector>` specialization sitting unused
    in the storage header is a route back to the untyped triple the kinds
    exist to remove: it would make the wrong storage compile the day someone
    hand-wrote it. The rule is stated on the emitted text, not just on the
    type function, because the text is what a reader would copy."""
    for name, text in files.items():
        assert "fixed<FVector>" not in text, name
        assert "std::is_same_v<I, FVector>" not in text, name


def test_fixed_arity_maps_only_where_the_shape_matches(schema):
    """Storage follows the shape AND the kind, and nothing else.

    A four-double attribute is FMjQuatRot only when the overlay calls it a
    rotation -- `axisangle` and <hfield size> are the counter-examples the
    table exists for -- and a three-double is a position or a direction only
    when the overlay says so. FVector appears nowhere: an MJCF triple in
    Unreal's own vector type is the confusion the kinds remove.
    """
    for e in schema.elements:
        for _fid, f in schema.stored_fields(e):
            t = f["type"]
            vt = emit_ue.value_type(schema, f)
            where = f"{e['name']}.{f['name']}"
            assert vt != "FVector", where
            arity = t.get("arity")
            if t["kind"] != "prim" or not arity or arity["kind"] != "fixed":
                assert vt not in ("FVector2D", "FMjQuatRot", "FLinearColor",
                                  "FMjPosition3", "FMjDirection3",
                                  "FMjVec3"), where
                continue
            shape = (t["prim"], arity["size"])
            kind = overlay_ue.UE_KIND.get(f["xml"])
            if vt == "FMjQuatRot":
                assert shape == ("double", 4), where
                assert kind == "orientation", where
            elif vt == "FMjPosition3":
                assert shape == ("double", 3), where
                assert kind == "position", where
            elif vt == "FMjDirection3":
                assert shape == ("double", 3), where
                assert kind in ("direction", "physical"), where
            elif vt == "FMjVec3":
                assert shape == ("double", 3), where
                assert kind not in ("position", "direction", "physical",
                                    "orientation"), where
            elif vt == "FVector2D":
                assert shape == ("double", 2), where
            elif vt == "FLinearColor":
                assert shape == ("float", 4), where
            else:
                assert vt.startswith("TArray<"), where
                # A four-double that is not a rotation stays an array, and the
                # only reason it may is that the overlay classified it as
                # something with no conversion.
                if shape == ("double", 4):
                    assert kind is not None and kind != "orientation", where


def test_every_spatial_shape_is_classified(schema):
    """The UE_KIND gate is exhaustive over fixed double[3] and double[4].

    Read off the schema here rather than trusted from the emitter, so a change
    that widened the storage fall-through without widening the gate fails.
    """
    for e in schema.elements:
        for f in e["fields"]:
            if emit_ue._kind_shape(f) is None:
                continue
            assert f["xml"] in overlay_ue.UE_KIND, f"{e['name']}.{f['name']}"


def test_an_unclassified_spatial_attribute_fails_generation(schema, monkeypatch):
    """Dropping one classification is a named failure, not a silent fallback."""
    trimmed = dict(overlay_ue.UE_KIND)
    trimmed.pop("pos")
    monkeypatch.setattr(overlay_ue, "UE_KIND", trimmed)
    with pytest.raises(emit_ue.UeError) as excinfo:
        emit_ue.UeSchema(schema.doc)
    assert "pos" in str(excinfo.value)


def test_a_stale_classification_fails_generation(schema, monkeypatch):
    """And so is a kind for an attribute the schema no longer shapes that way."""
    extended = dict(overlay_ue.UE_KIND)
    extended["nosuchattribute"] = "position"
    monkeypatch.setattr(overlay_ue, "UE_KIND", extended)
    with pytest.raises(emit_ue.UeError) as excinfo:
        emit_ue.UeSchema(schema.doc)
    assert "nosuchattribute" in str(excinfo.value)


def test_required_attributes_are_not_optional(schema):
    for e in schema.elements:
        for _fid, f in schema.stored_fields(e):
            stored = emit_ue.stored_type(schema, f)
            assert stored.startswith("TOptional<") == f["optional"], \
                f"{e['name']}.{f['name']}"


def test_references_reach_visit_wrapped(files, schema):
    """A reference stores a name, which is indistinguishable from a string
    attribute; the wrapper is what the reference scan matches on."""
    visit = files[f"{PUBLIC}/MjVisit.gen.h"]
    for e in schema.elements:
        head = f"void Visit({schema.cls[e['name']]}& Element, V&& Vis)"
        body = visit.split(head, 1)[1].split("\n}", 1)[0]
        for fid, f in enumerate(e["fields"]):
            if f["type"]["kind"] != "ref":
                continue
            slot = "TOptional<FString>" if f["optional"] else "FString"
            assert (f"urlab::RefView<{schema.ref_target(f)}, {slot}> "
                    f"Ref{fid}{{ Element.{schema.member(e, f)} }};") in body


# --------------------------------------------------------------------------- #
# Dispatch                                                                     #
# --------------------------------------------------------------------------- #
def test_contextual_child_tag_is_recovered(files, schema):
    """<worldbody> is a body under the model and nowhere else: the contextual
    tag cannot be read off the child element alone."""
    text = files[f"{PRIVATE}/MjDispatch.gen.cpp"]
    assert 'ElementType::Model, 8, ElementType::Body, TEXT("worldbody")' in text
    assert f'&{schema.cls["Body"]}::StaticClass, TEXT("body")' in text


def test_every_element_admits_its_schema_children(files, schema):
    text = files[f"{PUBLIC}/MjDispatch.gen.h"]
    for e in schema.elements:
        head = f"void ChildSlots(const {schema.cls[e['name']]}*, Fn&& Callback)"
        assert head in text
        body = text.split(head, 1)[1].split("\n}", 1)[0]
        for slot, child, _tag in emit_ue.child_slots(schema, e):
            assert f"Callback({slot}, ps::sdk::TypeTag<{schema.cls[child]}>" \
                   in body, f"{e['name']} slot {slot} {child}"


def test_name_collision_guard_fires():
    """The guard is what stands between a schema rename and a class UHT will
    not build; `active` (UActorComponent::SetActive) is the live case."""
    doc = load_schema()
    for e in doc["elements"]:
        for f in e["fields"]:
            if f["name"] == "mass":
                f["name"] = "class"
                f["xml"] = "class"
                with pytest.raises(UeError, match="already declares"):
                    UeSchema(doc)
                return
    pytest.fail("no element carries a `mass` attribute to rename")


def test_active_is_renamed(files, schema):
    """`active` would produce SetActive, which UActorComponent already declares
    as a virtual UFUNCTION with a different signature."""
    text = files[element_header_path(schema, "Light")]
    assert "TOptional<bool> ActiveFlag;" in text
    assert "SetActiveFlag(bool InValue)" in text
    assert "void SetActive(" not in text


# --------------------------------------------------------------------------- #
# Drift gate                                                                   #
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not os.environ.get(emit_ue.ENV_ROOT),
                    reason=f"set ${emit_ue.ENV_ROOT} to gate the URLab tree")
def test_checked_in_urlab_tree_matches_a_fresh_emit():
    stale = emit_ue.run(check=True)
    assert not stale, ("the URLab profile is stale; run "
                       "Scripts/regen_ue_profile.ps1:\n  " + "\n  ".join(stale))
