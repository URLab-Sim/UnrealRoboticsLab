"""Reference-manual tests: MuJoCo's XMLreference.rst -> attribute documentation.

What these pin is the extraction's judgement, not its plumbing. Reading anchors
out of a Sphinx document is easy to get right; deciding which paragraph is
ABOUT an attribute is where a tooltip goes from useful to actively misleading,
so the cases here are the ones where it could: a run of anchors that documents
only some of what it labels, an attribute the manual documents differently on
different elements, and an entry that is a pointer rather than a description.

The drift gate gets the same treatment: it exists to fail when the manual moves
under a recorded binding, so it is tested by moving one.
"""

from __future__ import annotations

import pytest

from protospec_gen import frontend, overlay_ue, xmlref


@pytest.fixture(scope="module")
def anchors() -> dict:
    return xmlref.load_anchors()


@pytest.fixture(scope="module")
def ast() -> dict:
    return frontend.load_schema()


# --------------------------------------------------------------------------- #
# Flattening                                                                   #
# --------------------------------------------------------------------------- #
def test_flatten_keeps_the_text_and_drops_the_markup():
    assert xmlref.flatten(":ref:`simulate.cc <saSimulate>`") == "simulate.cc"
    assert xmlref.flatten("the :at:`meshdir` of ``compiler``") == "the meshdir of compiler"
    assert xmlref.flatten("**bold** and *italic*") == "bold and italic"
    assert xmlref.flatten("a  \n  b") == "a b"


def test_flatten_spells_out_characters_cpp_cannot_carry():
    assert xmlref.flatten("x ≥ 1 ± ε").isascii()
    assert ">=" in xmlref.flatten("x ≥ 1")


def test_flatten_removes_what_would_escape_a_string_or_a_comment():
    flat = xmlref.flatten('a */ b \\ c "d"')
    assert "*/" not in flat
    assert "\\" not in flat
    assert '"' not in flat


# --------------------------------------------------------------------------- #
# Reading the manual                                                           #
# --------------------------------------------------------------------------- #
def test_a_shared_paragraph_reaches_every_attribute_it_names(anchors):
    # <geom>'s five orientation spellings are documented together, under a run
    # of five anchors and one paragraph.
    shared = anchors["body-geom-quat"]
    assert shared
    assert anchors["body-geom-euler"] == shared
    assert anchors["body-geom-zaxis"] == shared


def test_a_run_does_not_hand_its_prose_to_what_the_paragraph_ignores(anchors):
    # The pid actuator labels its whole inherited surface in one run and then
    # documents `kp` alone. `gear` is in the run and not in the paragraph.
    assert anchors["actuator-pid-kp"] == "Position feedback gain."
    assert anchors["actuator-pid-gear"] == ""


def test_an_alias_run_carries_no_prose_of_its_own(anchors):
    # The defaults section labels every geom attribute and documents none of
    # them: they are the real <geom>'s, and that is where the prose is.
    assert anchors["default-geom-pos"] == ""
    assert anchors["body-geom-pos"].startswith("Position of the geom")


# --------------------------------------------------------------------------- #
# Binding                                                                      #
# --------------------------------------------------------------------------- #
def test_an_element_takes_its_own_paragraph(ast, anchors):
    bound = xmlref.bind(ast, anchors)
    assert bound["geom.pos"] == "body-geom-pos"
    assert bound["site.pos"] == "body-site-pos"


def test_a_class_partial_takes_the_element_it_defaults(ast, anchors):
    # <default><tendon> is the tendon template, and the manual documents its
    # attributes once, on the real <spatial> tendon.
    bound = xmlref.bind(ast, anchors)
    assert bound["default_tendon.group"] == "tendon-spatial-group"


def test_an_attribute_the_manual_disagrees_about_binds_to_nothing(ast, anchors):
    # `name` is documented differently on a mesh, a texture and a body, so an
    # element the manual does not name gets no tooltip rather than one about
    # somebody else's name.
    bound = xmlref.bind(ast, anchors)
    assert "fixed.name" not in bound


def test_a_pointer_is_not_answered_from_another_element(ast, anchors):
    # Every sensor's `noise` says "See CSensor.", a chapter this module does not
    # read. Falling through to some other element's `noise` would be inventing
    # an answer the manual declined to give.
    assert xmlref._STUB.match(anchors["sensor-accelerometer-noise"])
    bound = xmlref.bind(ast, anchors)
    assert "accelerometer.noise" not in bound


def test_a_waived_attribute_is_not_bound(ast, anchors):
    bound = xmlref.bind(ast, anchors)
    for key in overlay_ue.XMLREF_WAIVERS:
        assert key not in bound


def test_binding_is_stable_across_runs(ast, anchors):
    assert xmlref.bind(ast, anchors) == xmlref.bind(ast, anchors)


# --------------------------------------------------------------------------- #
# The gate                                                                     #
# --------------------------------------------------------------------------- #
def test_the_recorded_bindings_match_the_manual(ast):
    # The gate as the generator runs it. A failure here is the point of the
    # file: the manual moved and nobody looked.
    tooltips = xmlref.tooltips(ast)
    pairs = sum(len(e["fields"]) for e in ast["elements"])
    assert len(tooltips) > pairs // 2
    assert all(text and text.isascii() for text in tooltips.values())
    assert all(len(text) <= xmlref.MAX_TOOLTIP for text in tooltips.values())


def test_a_vanished_anchor_fails_by_name(ast, anchors):
    # The manual restructured under a recorded binding. Nothing else changes,
    # so the report is one line and it names what went missing.
    bound = xmlref.bind(ast, anchors)
    anchor = bound["geom.pos"]
    without = {a: p for a, p in anchors.items() if a != anchor}
    with pytest.raises(xmlref.XmlRefError) as excinfo:
        xmlref.check_drift(bound, without)
    message = str(excinfo.value)
    assert anchor in message and "no longer defines" in message


def test_a_rebinding_fails_by_name(ast, anchors):
    # The attribute is still documented, somewhere else. Also a stop: which
    # paragraph an editor shows is not a thing to change without reading it.
    bound = dict(xmlref.bind(ast, anchors))
    bound["geom.pos"] = "somewhere-else"
    with pytest.raises(xmlref.XmlRefError) as excinfo:
        xmlref.check_drift(bound, anchors)
    assert "geom.pos" in str(excinfo.value)
