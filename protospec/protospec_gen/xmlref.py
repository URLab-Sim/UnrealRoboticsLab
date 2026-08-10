"""MuJoCo's reference manual, read as attribute documentation.

The schema is the grammar and it says almost nothing about meaning: 69 of its
1,505 attributes carry a `doc` string, and those are written for whoever is
implementing the generator. The prose a user of an editor wants is in the same
checkout, in `doc/XMLreference.rst`, where the manual documents nearly every
attribute of nearly every element.

The manual is machine-readable enough to bind to, because Sphinx makes it label
every attribute::

    .. _body-geom-solmix:

    :at:`solmix`: :at-val:`real, "1"`
       This attribute specifies the weight used for averaging of contact
       parameters...

An anchor is `<path>-<attr>`, the path being the element's own place in the
grammar, so `body-geom-solmix` is `solmix` on a `<geom>` inside a `<body>`. A
run of anchors with no `:at:` line under it is an alias: the defaults section
labels `default-geom-pos` and then says "all geom attributes are available
here", so the prose lives on `body-geom-pos` and the alias points at it.

Binding, in order, first match wins:

1. the element's own anchor -- `<tag>-<attr>` or anything ending `-<tag>-<attr>`;
2. an anchor whose path mentions the element and whose tail is the attribute;
3. the same attribute documented on any element.

Step 3 is what takes coverage from about half the surface to most of it, and it
is sound for the reason MJCF is readable at all: a name means one thing across
the grammar, so `<pair solref>` and `<geom solref>` are the same concept
documented once. It is not infallible, which is why every binding is RECORDED in
`doc_anchors.json` rather than recomputed silently: a wrong one is visible in a
diff and can be waived in `overlay_ue.XMLREF_WAIVERS`.

Units are deliberately NOT read from this prose, ever. A unit label is not
documentation: the editor rescales what a user types by it, so a label guessed
from a sentence is a silent factor of 57.3 waiting to happen. Units stay
hand-authored in `overlay.ANGLE_ATTRS` or absent.

Drift gate: `check_drift` fails generation when an anchor the baseline recorded
is no longer in the manual, because that is the manual having been restructured
under a binding that would otherwise keep serving prose from memory. A genuine
upstream removal is answered with a waiver row and a reason. The baseline is
generated, not hand-kept::

    uv run python -m protospec_gen.xmlref --update-baseline
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

from . import overlay_ue
from .frontend import load_schema, mujoco_src

__all__ = ["XmlRefError", "load_anchors", "bind", "check_drift",
           "write_baseline", "flatten"]

_HERE = os.path.dirname(os.path.abspath(__file__))

# One line per bound (element, attribute), naming the manual anchor its prose
# came from. Committed, so a rebinding is a reviewable diff rather than a
# silent change of what the editor tells a user.
DOC_ANCHORS = os.path.join(_HERE, "doc_anchors.json")

# How much prose a tooltip carries. The manual's longest entry is 5,700
# characters of discussion; a tooltip is a hint, so it takes whole sentences up
# to this bound and stops.
MAX_TOOLTIP = 400

# An entry that is only a pointer into a part of the manual this module does
# not read. "Name of the geom." is short and useful; "See CUser." is neither.
_STUB = re.compile(r"^See [A-Za-z]+\.?$")


class XmlRefError(Exception):
    """The reference manual and the recorded bindings disagree."""


# --------------------------------------------------------------------------- #
# Reading the manual                                                           #
# --------------------------------------------------------------------------- #
_ANCHOR = re.compile(r"^\.\. _([A-Za-z0-9_\-]+):\s*$")
_AT_NAMES = re.compile(r":at:`([A-Za-z0-9_]+)`")


def load_anchors(root: str | None = None) -> dict[str, str]:
    """Anchor -> its flattened prose, for every documented attribute.

    An anchor whose `:at:` line is followed by nothing is carried with empty
    prose: it exists in the manual, which is what the drift gate asks about,
    and it simply has nothing to say.
    """
    path = os.path.join(root or mujoco_src(), "doc", "XMLreference.rst")
    if not os.path.isfile(path):
        raise XmlRefError(f"no reference manual at {path}")
    with open(path, "r", encoding="utf-8") as fh:
        lines = fh.read().splitlines()

    out: dict[str, str] = {}
    i = 0
    while i < len(lines):
        match = _ANCHOR.match(lines[i])
        if not match:
            i += 1
            continue

        # Anchors come in runs, and a run means one of two things. Followed by
        # an `:at:` line, it is the manual documenting several spellings of one
        # thing together -- `quat`, `axisangle`, `xyaxes`, `zaxis` and `euler`
        # share one paragraph -- and every anchor in the run carries it. Followed
        # by anything else, it is a section of aliases: the defaults section
        # labels `default-geom-pos` and then says the geom attributes are all
        # available there, so the prose is elsewhere and these anchors exist
        # only to be linked to.
        run = []
        j = i
        while j < len(lines):
            anchor = _ANCHOR.match(lines[j])
            if anchor:
                run.append(anchor.group(1))
                j += 1
            elif not lines[j].strip():
                j += 1
            else:
                break

        prose = ""
        documented: set[str] = set()
        if j < len(lines) and lines[j].startswith(":at:`"):
            # The `:at:` line names exactly which attributes the paragraph is
            # about, and a run is not always all of them: the pid actuator
            # labels its whole inherited surface in one run and then documents
            # `kp` alone. So the names are read, and an anchor in the run that
            # the line does not name stays an alias.
            documented = set(_AT_NAMES.findall(lines[j].split(":at-val:")[0]))
            k = j + 1
            block = []
            while k < len(lines):
                line = lines[k]
                if line.strip() and not line.startswith("   "):
                    break
                block.append(line.strip())
                k += 1
            prose = flatten(" ".join(x for x in block if x))
            j = k
        for anchor in run:
            out[anchor] = prose if anchor.rsplit("-", 1)[-1] in documented else ""
        i = max(j, i + 1)
    return out


# --------------------------------------------------------------------------- #
# Flattening Sphinx to one line of ASCII                                       #
# --------------------------------------------------------------------------- #
# `:ref:`text <target>`` and friends: the text is what a reader wants, the
# target is a link this output cannot follow.
_ROLE_TARGET = re.compile(r":[a-z\-]+:`([^`<]*?)\s*<[^`>]*>`")
_ROLE = re.compile(r":[a-z\-]+:`([^`]*)`")
_LITERAL = re.compile(r"``([^`]*)``")
_EMPHASIS = re.compile(r"\*\*?([^*]+)\*\*?")
_SUBSTITUTION = re.compile(r"\|([A-Za-z0-9_/ -]*)\|")
_WHITESPACE = re.compile(r"\s+")

# Characters the manual uses that C++ source cannot carry: the generated tree is
# ASCII by rule, because MSVC decodes a BOM-less file in the system codepage.
# Every one of them has an ASCII spelling, and an unmapped one fails generation
# rather than being dropped, because dropping a `>=` inverts a sentence.
_ASCII = {
    "‘": "'", "’": "'", "“": '"', "”": '"',
    "–": "-", "—": "-", "−": "-", "­": "",
    "…": "...", " ": " ", "​": "",
    "≤": "<=", "≥": ">=", "≠": "!=", "≈": "~=",
    "±": "+/-", "×": "x", "·": ".", "⋅": ".",
    "√": "sqrt", "∞": "infinity", "∂": "d",
    "°": " degrees", "µ": "micro", "′": "'",
    "½": "1/2", "¼": "1/4", "¾": "3/4",
    "²": "^2", "³": "^3", "⁰": "^0", "¹": "^1",
    "α": "alpha", "β": "beta", "γ": "gamma",
    "δ": "delta", "ε": "epsilon", "θ": "theta",
    "λ": "lambda", "μ": "mu", "π": "pi", "ρ": "rho",
    "σ": "sigma", "τ": "tau", "φ": "phi", "ω": "omega",
    "Δ": "Delta", "Ω": "Omega", "Σ": "Sigma",
    "←": "<-", "→": "->", "⇒": "=>",
}


def flatten(text: str) -> str:
    """One line of ASCII, with Sphinx's markup taken back out.

    Not a general reStructuredText renderer: it undoes exactly what the
    attribute descriptions use, and anything it does not understand it leaves
    alone rather than guessing.
    """
    text = _ROLE_TARGET.sub(r"\1", text)
    text = _ROLE.sub(r"\1", text)
    text = _LITERAL.sub(r"\1", text)
    text = _EMPHASIS.sub(r"\1", text)
    text = _SUBSTITUTION.sub(" ", text)
    for src, dst in _ASCII.items():
        text = text.replace(src, dst)
    # A line block's leading bar survives the join as a stray token.
    text = text.replace("| ", " ")
    # `*/` would close the doc comment this ends up inside, and a backslash or a
    # quote would have to survive both a C++ string literal and UHT's metadata
    # parser. None of the three is worth a tooltip.
    text = text.replace("*/", "").replace("\\", "/").replace('"', "'")
    return _WHITESPACE.sub(" ", text).strip()


def _sentences(text: str, limit: int) -> str:
    """`text` cut to whole sentences within `limit`, or hard-cut if the first
    sentence is already longer than that."""
    if len(text) <= limit:
        return text
    cut = text[:limit]
    end = max(cut.rfind(". "), cut.rfind("; "))
    if end > limit // 3:
        return cut[:end + 1]
    return text[:limit - 3].rstrip() + "..."


# --------------------------------------------------------------------------- #
# Binding schema attributes to anchors                                         #
# --------------------------------------------------------------------------- #
def _candidates(anchors: dict[str, str], tag: str, attr: str) -> list[str]:
    """Every anchor that could document `tag`.`attr`, best rank first."""
    tail = f"-{attr}"
    exact = [a for a in anchors if a == f"{tag}-{attr}" or a.endswith(f"-{tag}{tail}")]
    same_element = [a for a in anchors
                    if a.endswith(tail)
                    and (a.startswith(f"{tag}-") or f"-{tag}-" in a)
                    and a not in exact]
    any_element = [a for a in anchors
                   if (a == attr or a.endswith(tail))
                   and a not in exact and a not in same_element]
    return [exact, same_element, any_element]


def _usable(prose: str) -> bool:
    """True when the entry says something rather than pointing somewhere.

    `See CUser.` is a cross-reference to a section of the manual this module
    does not read; as a tooltip it is worse than nothing, because it looks like
    documentation. A sentence that ENDS with such a pointer is kept: it has
    already said what the attribute is.
    """
    return bool(prose) and _STUB.match(prose) is None


def _points_elsewhere(anchors: dict[str, str], names: list[str]) -> bool:
    """True when this element's own entry is a pointer rather than a paragraph.

    `See CSensor.` under every sensor's `noise` is the manual saying the
    attribute is documented in a chapter this module does not read. Serving
    another element's paragraph instead would be inventing an answer it
    declined to give.
    """
    return any(anchors[a] and _STUB.match(anchors[a]) for a in names)


def _best_documented(anchors: dict[str, str], names: list[str]) -> str | None:
    """The fullest entry among anchors that name this element's attribute."""
    usable = [a for a in names if _usable(anchors[a])]
    if not usable:
        return None
    return max(sorted(usable), key=lambda a: len(anchors[a]))


def _best_agreed(anchors: dict[str, str], names: list[str]) -> str | None:
    """The entry every candidate agrees on, or nothing.

    This is what makes the fallback ranks safe. `solmix` is documented in one
    paragraph wherever it appears, so any element may take it; `name` is
    documented differently on a mesh, a texture and a joint, so an element the
    manual does not name gets no tooltip at all rather than another element's.
    One disagreement is enough to withhold: the alternative is a description of
    the wrong thing, which a reader has no way to detect.
    """
    usable = [a for a in names if _usable(anchors[a])]
    if not usable or len({anchors[a] for a in usable}) != 1:
        return None
    return sorted(usable)[0]


def bind(ast: dict, anchors: dict[str, str]) -> dict[str, str]:
    """`element.attribute` -> the anchor documenting it, for every match."""
    out: dict[str, str] = {}
    for element in ast["elements"]:
        tag, name = element["xml"], element["schema_name"]
        for field in element["fields"]:
            key = f"{name}.{field['xml']}"
            if key in overlay_ue.XMLREF_WAIVERS:
                continue
            exact, same_element, any_element = _candidates(anchors, tag, field["xml"])
            anchor = _best_documented(anchors, exact)
            if anchor is None and not _points_elsewhere(anchors, exact):
                # An empty anchor is an alias -- the manual labels the attribute
                # here and documents it under the element this one borrows from
                # -- so the fallback ranks are looking for the same paragraph.
                # A pointer into a section this module does not read is not: the
                # manual has answered, and the answer was "not here".
                anchor = (_best_agreed(anchors, same_element)
                          or _best_agreed(anchors, any_element))
            if anchor is not None:
                out[key] = anchor
    return dict(sorted(out.items()))


def tooltips(ast: dict, root: str | None = None) -> dict[str, str]:
    """`element.attribute` -> the prose an editor should show for it.

    The drift gate runs first, so a caller cannot get documentation out of this
    module without the recorded bindings having been checked against the manual.
    """
    anchors = load_anchors(root)
    bound = bind(ast, anchors)
    check_drift(bound, anchors)
    out = {key: _sentences(anchors[anchor], MAX_TOOLTIP)
           for key, anchor in bound.items()}

    # The generated tree is ASCII by rule, and `flatten` spells out every
    # non-ASCII character the manual uses. One it does not know is a new
    # spelling to add there, named here rather than found later as a file-level
    # complaint about bytes.
    unmapped = sorted({(key, ch) for key, text in out.items()
                       for ch in text if not ch.isascii()})
    if unmapped:
        raise XmlRefError(
            "XMLreference.rst uses characters the flattener has no ASCII "
            "spelling for (add each to xmlref._ASCII):\n  "
            + "\n  ".join(f"{key}: U+{ord(ch):04X} {ch!r}"
                          for key, ch in unmapped))
    return out


# --------------------------------------------------------------------------- #
# Drift                                                                        #
# --------------------------------------------------------------------------- #
def _read_baseline() -> dict[str, str]:
    with open(DOC_ANCHORS, "r", encoding="utf-8") as fh:
        return json.load(fh)["anchors"]


def check_drift(bound: dict[str, str], anchors: dict[str, str]) -> None:
    """Every anchor the baseline recorded is still in the manual.

    Asked of the RECORDED anchors rather than of the ones just bound, because
    the failure this exists for is invisible from the new binding alone: when an
    anchor disappears, the fallback ranks quietly serve prose from a different
    element and nothing looks wrong. Recorded, the disappearance is a name.

    A rebinding -- the same attribute now documented under a different anchor --
    is reported the same way and answered the same way, by refreshing the
    baseline once a human has looked at what moved.
    """
    baseline = _read_baseline()
    problems = []

    for key, anchor in sorted(baseline.items()):
        reason = overlay_ue.XMLREF_WAIVERS.get(key)
        if reason is not None:
            continue
        if anchor not in anchors:
            problems.append(
                f"{key} was documented at '{anchor}', which XMLreference.rst no "
                "longer defines")
        elif bound.get(key) != anchor:
            problems.append(
                f"{key} was documented at '{anchor}' and now binds to "
                f"'{bound.get(key)}'")

    for key, reason in sorted(overlay_ue.XMLREF_WAIVERS.items()):
        if not (isinstance(reason, str) and reason.strip()):
            problems.append(f"XMLREF_WAIVERS waives {key} with no reason")

    if problems:
        raise XmlRefError(
            "the recorded documentation bindings no longer match "
            "XMLreference.rst (read what moved, then refresh with `python -m "
            "protospec_gen.xmlref --update-baseline`, or waive a genuine "
            "removal in overlay_ue.XMLREF_WAIVERS):\n  "
            + "\n  ".join(problems))


def write_baseline(root: str | None = None, path: str | None = None) -> int:
    """Rewrite the recorded bindings; return how many there are.

    Deliberately does not go through :func:`tooltips`: the gate this file feeds
    is what stands between a restructured manual and a tooltip served from
    memory, and a refresh that had to pass the gate could never answer it.
    """
    anchors = load_anchors(root)
    bound = bind(load_schema(root), anchors)
    payload = {"anchors": bound}
    with open(path or DOC_ANCHORS, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(payload, fh, indent=2)
        fh.write("\n")
    return len(bound)


def _main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m protospec_gen.xmlref",
        description="Documentation bindings against MuJoCo's reference manual.")
    parser.add_argument(
        "--update-baseline", action="store_true",
        help="rewrite doc_anchors.json from the current manual and schema")
    parser.add_argument("--root", help="MuJoCo checkout to read the manual from")
    args = parser.parse_args(argv)
    if not args.update_baseline:
        parser.error("nothing to do; pass --update-baseline")
    count = write_baseline(args.root)
    print(f"{DOC_ANCHORS}: {count} bindings")
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
