"""Corpus coverage gate: the MJCF schema vs the vendored MuJoCo MJCF corpus.

Runs ``tools/corpus_study`` over MuJoCo's whole ``.xml`` corpus (skipped when the
vendored tree is absent) and asserts three things:

* the study actually inventories a large corpus (> 350 MJCF files);
* **zero unknown elements** -- every element tag in every file, in its parent
  context, maps to a schema element or a recognized read-time construct;
* **zero unknown containment pairs** -- every distinct (parent element, child
  element) edge in the corpus is representable as a schema child link (through
  body-context aliasing, worldbody/frame/replicate -> Body), unless listed in
  ``KNOWN_GAP_PAIRS`` with a reason. This dimension catches missing *recursion*
  edges (body-in-body via the MJCF[] R type-code) that element-tag resolution
  alone masks;
* every unknown **attribute** and every unknown **enum value** is a reviewed,
  reasoned entry in ``KNOWN_GAPS`` below -- silence is not allowed, so each gap
  is a visible decision (plan Section 10.5, task brief).

The corpus root is discovered from ``PROTOSPEC_CORPUS`` or the vendored plugin
path; set the env var to point the gate at a different checkout.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


def _load_study():
    spec = importlib.util.spec_from_file_location(
        "corpus_study", ROOT / "tools" / "corpus_study.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(mod)
    return mod


def _corpus_root() -> Path | None:
    env = os.environ.get("PROTOSPEC_CORPUS")
    candidates = [Path(env)] if env else []
    candidates.append(
        Path(
            r"C:\Users\jonat\Documents\Unreal Projects\url_proj\Plugins"
            r"\UnrealRoboticsLab\third_party\MuJoCo\src"
        )
    )
    for c in candidates:
        if c.is_dir():
            return c
    return None


# --------------------------------------------------------------------------- #
# Reviewed gaps                                                                 #
# --------------------------------------------------------------------------- #
# Attributes the corpus uses that no schema field covers. Each is a deliberate,
# understood decision -- not a silent omission.
# key: (element_tag, attribute) -> reason
KNOWN_GAP_ATTRIBUTES: dict[tuple[str, str], str] = {
    ("Replicate", "prefix"): (
        "not an MJCF attribute: OneBody reads only count/offset/euler/sep/"
        "childclass off <replicate>, and mjXSchema never validates replicate's "
        "attributes (NameMatch routes replicate to the body row, whose Check is "
        "skipped by the recursive-descent branch), so the stray prefix= in "
        "collision_primitive/spheres_cylinders.xml is silently dropped by stock "
        "MuJoCo. Namespacing there is by suffix; prefix= belongs to <composite>."
    ),
}

# Containment pairs the corpus uses that no schema child link represents. Each
# is a reviewed decision. Empty: body-in-body recursion is now wired (Body gets
# `children bodies : Body *`, mirrored onto Frame/Replicate), and the corpus
# study confirms every observed (parent, child) edge is representable.
# key: (parent_element, child_element) -> reason
KNOWN_GAP_PAIRS: dict[tuple[str, str], str] = {}

# Enum attributes whose corpus values fall outside the schema enum. Each is a
# reviewed decision. key: (element, attribute) -> reason (covers all values).
#
# The MapValues keyword-set attributes (rangefinder/contact `data`, camera
# `output`) are no longer gaps: they are typed `EnumName[]` (a space-separated
# keyword set) in the schema, and the study tokenizes and checks each keyword.
KNOWN_GAP_ENUM_VALUES: dict[tuple[str, str], str] = {}


# --------------------------------------------------------------------------- #
# The gate                                                                      #
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def report():
    corpus = _corpus_root()
    if corpus is None:
        pytest.skip("MuJoCo corpus not found (set PROTOSPEC_CORPUS)")
    study = _load_study()
    return study.build_report(corpus)


def test_study_runs_over_large_mjcf_corpus(report):
    assert report["corpus"]["mjcf"] > 350, report["corpus"]
    assert report["corpus"]["unparseable"] == 0, report["corpus"]


def test_zero_unknown_elements(report):
    unknown = report["unknown"]["elements"]
    assert unknown == [], (
        f"{len(unknown)} uncovered element(s); every element tag must map to a "
        "schema element or a recognized read-time construct:\n"
        + "\n".join(
            f"  {u['context']}/{u['element']} "
            f"({u['files_count']} files, e.g. {u['example_files']})"
            for u in unknown
        )
    )


def test_unknown_pairs_are_reviewed(report):
    unlisted = [
        u for u in report["unknown"]["pairs"]
        if (u["parent"], u["child"]) not in KNOWN_GAP_PAIRS
    ]
    assert not unlisted, (
        "unreviewed unknown containment pair(s); every (parent, child) edge must "
        "be representable as a schema child link (add the link or a "
        "KNOWN_GAP_PAIRS entry with a reason):\n"
        + "\n".join(
            f"  {u['parent']} -> {u['child']} "
            f"({u['files_count']} files, e.g. {u['example_files']})"
            for u in unlisted
        )
    )


def test_unknown_attributes_are_reviewed(report):
    unlisted = [
        u for u in report["unknown"]["attributes"]
        if (u["element"], u["attribute"]) not in KNOWN_GAP_ATTRIBUTES
    ]
    assert not unlisted, (
        "unreviewed unknown attribute(s) (add a schema field or a "
        "KNOWN_GAP_ATTRIBUTES entry with a reason):\n"
        + "\n".join(
            f"  {u['element']}.{u['attribute']} "
            f"({u['files_count']} files, e.g. {u['example_files']})"
            for u in unlisted
        )
    )


def test_unknown_enum_values_are_reviewed(report):
    unlisted = [
        u for u in report["unknown"]["enum_values"]
        if (u["element"], u["attribute"]) not in KNOWN_GAP_ENUM_VALUES
    ]
    assert not unlisted, (
        "unreviewed unknown enum value(s) (fix the schema enum/arity or add a "
        "KNOWN_GAP_ENUM_VALUES entry with a reason):\n"
        + "\n".join(
            f"  {u['element']}.{u['attribute']} = {u['value']!r} "
            f"(enum {u['enum']}, {u['files_count']} files, e.g. {u['example_files']})"
            for u in unlisted
        )
    )


def test_known_gaps_carry_reasons():
    for key, reason in {
        **KNOWN_GAP_ATTRIBUTES,
        **KNOWN_GAP_ENUM_VALUES,
        **KNOWN_GAP_PAIRS,
    }.items():
        assert isinstance(reason, str) and reason.strip(), key


def test_output_is_deterministic():
    # Two independent runs over the same inputs must be byte-identical (sorted
    # lists, capped examples) so the checked-in report and any review diff are
    # stable. Immune to the concurrent schema editor: both builds read one file.
    corpus = _corpus_root()
    if corpus is None:
        pytest.skip("MuJoCo corpus not found (set PROTOSPEC_CORPUS)")
    import json

    study = _load_study()
    a = json.dumps(study.build_report(corpus), indent=2, sort_keys=True)
    b = json.dumps(study.build_report(corpus), indent=2, sort_keys=True)
    assert a == b
