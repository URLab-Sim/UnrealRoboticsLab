"""Corpus study: validate the MJCF schema against the vendored MuJoCo MJCF corpus.

For every ``.xml`` model in MuJoCo's source tree this walks the document in
parallel with the ProtoSpec schema (loaded through ``protospec_gen.load_schema``)
and inventories what the corpus actually uses: every element tag in its parent
context, every distinct (parent element, child element) containment PAIR, every
attribute per element, and every value of a schema-typed enum attribute. It then
reports what the schema does NOT cover, so each gap is a visible, actionable data
point rather than a silent omission.

The pair dimension exists because element-tag resolution alone cannot catch a
missing *containment* edge: MuJoCo encodes body-in-body (and default-in-default)
recursion via the MJCF[] R type-code self-reference, so a nested ``<body>``
always *resolves* to the Body element (the reader's body-row special case), even
when the schema's Body has no child link back to Body. The pair check verifies
each observed (parent, child) edge against the schema's actual child links
(through body-context aliasing, worldbody/frame/replicate -> Body), so a missing
self-recursive link surfaces as an unknown pair instead of passing silently.

Coverage-mapping conventions (identical to the drift gate in
``tests/test_schema_coverage.py``):

* an IDL element covers the MJCF element reached by matching its XML tag along
  the child links (the tree walk disambiguates same-tag elements in different
  positions -- e.g. body/joint vs equality/joint vs tendon/fixed/joint);
* an IDL field covers the attribute named by its ``xml=`` annotation, else its
  own name; a ``variant`` field additionally covers every attribute named by its
  variant arms; children cover child elements. An enum-typed field records every
  value of its attribute against the enum's keyword set; a keyword-set field
  (``EnumName[]``, MuJoCo's ``MapValues`` bitmask attributes) is split on
  whitespace and each token counted and checked individually.

Read-time constructs are handled the way
MuJoCo's own reader does, so they are never miscounted as unknown structure:

* ``<include>``  -- resolves nothing; usage recorded, not attribute-checked.
* ``<worldbody>`` / nested ``<body>`` -- share MuJoCo's "body row"; the reader
  nests bodies via that row (not a literal child entry), and MuJoCo's own MJCF[]
  table omits body/frame as children of body too. Validated against ``Body``.
* ``<frame>``    -- persists; own attributes validate against the ``Frame`` row,
  but its children are resolved in body context (a frame may hold bodies/frames).
* ``<replicate>``-- a first-class pass-through element; it has no MJCF[]
  row (the reader handles it against the body row, xml_util.cc:492), so it is a
  curated ``Replicate`` schema element. Own attributes validate against that row,
  its children are resolved in body context.

Output is ``snapshots/corpus_report.json``: deterministic ordering, LF line
endings, 2-space indent, repo-relative example paths capped at three per item.

CLI::

    uv run python tools/corpus_study.py --corpus PATH --out PATH

Stdlib only (plus ``protospec_gen``, this repo's own package).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field as _dc_field
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from protospec_gen import load_schema  # noqa: E402

_DEFAULT_OUT = _ROOT / "snapshots" / "corpus_report.json"

_MAX_EXAMPLES = 3

# Body-like schema element whose child set governs any body-context container.
# Body is the superset (Frame lacks only nested body/frame, which Body supplies).
_BODY_CTX = "Body"


# --------------------------------------------------------------------------- #
# Schema index                                                                  #
# --------------------------------------------------------------------------- #
@dataclass
class _ElemInfo:
    name: str
    tag: str
    attrs: set  # covered attribute names
    enum_attrs: dict  # attr name -> frozenset of legal enum values
    enum_of: dict  # attr name -> enum type name
    enum_list_attrs: set  # enum attrs typed EnumName[] (space-separated keyword set)
    child_targets: dict  # xml tag -> child element name


class _SchemaIndex:
    """Coverage view over a parsed schema: tags, covered attrs, enum tables,
    child links, with body-context aliasing baked into ``child_targets_for``."""

    def __init__(self, schema_json: dict):
        self._elements_raw = {e["name"]: e for e in schema_json["elements"]}
        self._unions = {
            u["name"]: list(u["members"]) for u in schema_json.get("unions", [])
        }
        self._enum_values = {
            e["name"]: frozenset(m["value"] for m in e["members"])
            for e in schema_json["enums"]
        }
        self._tag_of = {name: e["xml"]
                        for name, e in self._elements_raw.items()}
        self._info: dict[str, _ElemInfo] = {}
        for name, e in self._elements_raw.items():
            self._info[name] = self._build(name, e)

    def _build(self, name: str, e: dict) -> _ElemInfo:
        attrs: set = set()
        enum_attrs: dict = {}
        enum_of: dict = {}
        enum_list_attrs: set = set()
        for f in e["fields"]:
            t = f["type"]
            attr = f["xml"]
            attrs.add(attr)
            if t["kind"] == "named" and t.get("category") == "enum":
                enum_attrs[attr] = self._enum_values[t["name"]]
                enum_of[attr] = t["name"]
                # An EnumName[] field is a space-separated keyword set (a
                # MapValues bitmask); each token is checked against the enum.
                if (t.get("arity") or {}).get("kind") == "unbounded":
                    enum_list_attrs.add(attr)
        # A child list targets either a single element or a union of member
        # elements: a union child list covers its member tags.
        child_targets: dict = {}
        for c in e["children"]:
            if "union" in c:
                members = self._unions[c["union"]]
            else:
                members = [c["element"]]
            for m in members:
                child_targets[self._tag_of[m]] = m
        # Input aliases: an MJCF attribute the reader accepts and folds into a
        # sibling field or a child list (euler -> quat; Material's `texture=` ->
        # a `<layer role="rgb">`). Each has no field of its own but is covered.
        for alias in e.get("input_aliases", []):
            attrs.add(alias["attr"])
        return _ElemInfo(name, self._tag_of[name], attrs, enum_attrs, enum_of,
                         enum_list_attrs, child_targets)

    def info(self, name: str) -> _ElemInfo:
        return self._info[name]

    def child_targets_for(self, ctx_name: str) -> dict:
        # Body, Frame and Replicate all expose the full body-context child set.
        if ctx_name in ("Body", "Frame", "Replicate"):
            return self._info["Body"].child_targets
        return self._info[ctx_name].child_targets

    def global_tag_map(self) -> dict:
        # tag -> element name for context-free (fragment) resolution; ambiguous
        # tags resolve to whichever element name sorts first, deterministically.
        out: dict[str, str] = {}
        for name in sorted(self._info):
            out.setdefault(self._tag_of[name], name)
        return out


# --------------------------------------------------------------------------- #
# Inventory accumulators                                                        #
# --------------------------------------------------------------------------- #
@dataclass
class _Gap:
    files: set = _dc_field(default_factory=set)
    count: int = 0

    def hit(self, relpath: str) -> None:
        self.count += 1
        self.files.add(relpath)

    def examples(self) -> list:
        return sorted(self.files)[:_MAX_EXAMPLES]


class _Inventory:
    def __init__(self, index: _SchemaIndex):
        self._index = index
        self.n_elements = 0
        self.n_attributes = 0
        self.n_enum_values = 0
        self.n_pairs = 0
        self.elem_kinds: set = set()  # distinct (context, tag)
        self.pair_kinds: set = set()  # distinct (parent element, child element)
        self.attr_kinds: set = set()  # distinct (element, attr)
        self.enum_kinds: set = set()  # distinct (element, attr, value)
        self.unknown_elements: dict = {}  # (context, tag) -> _Gap
        self.unknown_pairs: dict = {}  # (parent element, child element) -> _Gap
        self.unknown_attributes: dict = {}  # (element, attr) -> _Gap
        self.unknown_enum_values: dict = {}  # (element, attr, value) -> _Gap

    # -- resolution -------------------------------------------------------- #
    def _resolve(self, tag: str, parent_ctx: str):
        """Return (attr_element | None, child_ctx | None, construct | None)."""
        if tag in ("include", "mujocoinclude"):
            return None, None, "include"
        if tag in ("worldbody", "body"):
            # worldbody, nested body, frame and replicate all share MuJoCo's
            # "body row" (xml_util.cc:486-498); the reader nests bodies via that
            # row, not via a literal child entry -- MuJoCo's own MJCF[] table
            # (the arbiter) likewise omits body/frame as children of body.
            return "Body", "Body", None
        if tag == "replicate":
            # replicate is a first-class element; its own attributes are
            # validated against the Replicate row and its children in body context.
            return "Replicate", "Replicate", None
        if tag == "frame":
            return "Frame", "Frame", None
        target = self._index.child_targets_for(parent_ctx).get(tag)
        if target is not None:
            return target, target, None
        return None, None, None

    # -- walk -------------------------------------------------------------- #
    def walk(self, elem, ctx_name: str, relpath: str) -> None:
        """Inventory ``elem`` (already resolved to schema element ``ctx_name``)
        and recurse into its children resolved from body/normal context."""
        info = self._index.info(ctx_name)
        self._record_attrs(elem, info, relpath)
        self._descend(elem, ctx_name, relpath)

    def _record_attrs(self, elem, info: _ElemInfo, relpath: str) -> None:
        for attr, value in sorted(elem.attrib.items()):
            self.n_attributes += 1
            self.attr_kinds.add((info.name, attr))
            if attr not in info.attrs:
                self._flag(self.unknown_attributes, (info.name, attr), relpath)
                continue
            legal = info.enum_attrs.get(attr)
            if legal is not None:
                # A keyword-set (EnumName[]) attr is space-separated; each token
                # is a distinct enum-value observation. Scalar enums yield one.
                tokens = value.split() if attr in info.enum_list_attrs else [value]
                for token in tokens:
                    self.n_enum_values += 1
                    key = (info.name, attr, token)
                    self.enum_kinds.add(key)
                    if token not in legal:
                        self._flag(self.unknown_enum_values, key, relpath)

    def _descend(self, elem, ctx_name: str, relpath: str) -> None:
        for child in elem:
            if not isinstance(child.tag, str):
                continue  # comment / processing instruction
            self.n_elements += 1
            tag = child.tag
            self.elem_kinds.add((ctx_name, tag))
            attr_elem, child_ctx, construct = self._resolve(tag, ctx_name)
            if construct == "include":
                self._record_construct_attrs(child, relpath)
                continue
            if attr_elem is None:
                self._flag(self.unknown_elements, (ctx_name, tag), relpath)
                continue
            self._record_pair(ctx_name, attr_elem, relpath)
            info = self._index.info(attr_elem)
            self._record_attrs(child, info, relpath)
            self._descend(child, child_ctx, relpath)

    def _record_pair(self, parent_ctx: str, child_elem: str, relpath: str) -> None:
        # A (parent element, child element) containment edge. Verified against
        # the schema's own child links (body-context aliasing applied via
        # child_targets_for), so a self-recursive edge (body-in-body) that the
        # reader's body-row special case masks still surfaces here if the schema
        # lacks the link.
        self.n_pairs += 1
        key = (parent_ctx, child_elem)
        self.pair_kinds.add(key)
        reachable = set(self._index.child_targets_for(parent_ctx).values())
        if child_elem not in reachable:
            self._flag(self.unknown_pairs, key, relpath)

    def _record_construct_attrs(self, elem, relpath: str) -> None:
        # Attributes of the read-time include construct: usage is counted but not
        # attribute-checked (include resolves file plumbing, not model content).
        for attr in sorted(elem.attrib):
            self.n_attributes += 1
            self.attr_kinds.add((elem.tag, attr))

    def walk_fragment(self, root, relpath: str) -> None:
        # Body-context-agnostic inventory for a bare / mujocoinclude fragment:
        # resolve each element by tag against the global tag map.
        gmap = self._index.global_tag_map()
        stack = [root]
        while stack:
            node = stack.pop()
            for child in node:
                if not isinstance(child.tag, str):
                    continue
                self.n_elements += 1
                tag = child.tag
                self.elem_kinds.add(("<fragment>", tag))
                if tag in ("include", "mujocoinclude"):
                    self._record_construct_attrs(child, relpath)
                elif tag in gmap:
                    self._record_attrs(child, self._index.info(gmap[tag]), relpath)
                else:
                    self._flag(self.unknown_elements, ("<fragment>", tag), relpath)
                stack.append(child)

    @staticmethod
    def _flag(table: dict, key, relpath: str) -> None:
        gap = table.get(key)
        if gap is None:
            gap = _Gap()
            table[key] = gap
        gap.hit(relpath)


# --------------------------------------------------------------------------- #
# File discovery + classification                                              #
# --------------------------------------------------------------------------- #
def _discover(corpus: Path) -> list:
    files = []
    for dp, dn, fn in os.walk(corpus):
        parts = Path(dp).parts[len(corpus.parts):]
        # The checkout's embedded ProtoSpec tree is not part of MuJoCo's MJCF
        # corpus, and counting its fixtures makes the coverage metric measure
        # the wrong population.
        if parts[:1] == ("protospec",):
            continue
        if "build" in parts:
            continue
        for f in fn:
            if f.endswith(".xml"):
                files.append(Path(dp) / f)
    return sorted(files)


def _relpath(path: Path, corpus: Path) -> str:
    return path.relative_to(corpus).as_posix()


def _classify_and_walk(path: Path, corpus: Path, inv: _Inventory) -> str:
    relpath = _relpath(path, corpus)
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        # A bare fragment (multiple top-level elements) is not a well-formed
        # document; wrap and inventory it body-context-agnostically.
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
            root = ET.fromstring(f"<mujocoinclude>{text}</mujocoinclude>")
        except ET.ParseError:
            return "unparseable"
        inv.walk_fragment(root, relpath)
        return "include_fragment"
    if root.tag == "mujoco":
        inv.n_elements += 1
        inv.elem_kinds.add(("<root>", "mujoco"))
        inv.walk(root, "Model", relpath)
        return "mjcf"
    if root.tag == "mujocoinclude":
        inv.walk_fragment(root, relpath)
        return "include_fragment"
    return "non_mjcf"


# --------------------------------------------------------------------------- #
# Report assembly                                                              #
# --------------------------------------------------------------------------- #
def _pct(covered: int, total: int) -> float:
    if total == 0:
        return 100.0
    return round(100.0 * covered / total, 2)


def build_report(corpus: Path) -> dict:
    schema = load_schema()
    index = _SchemaIndex(schema)
    inv = _Inventory(index)

    counts = {"mjcf": 0, "include_fragment": 0, "non_mjcf": 0, "unparseable": 0}
    files = _discover(corpus)
    for path in files:
        counts[_classify_and_walk(path, corpus, inv)] += 1

    def elem_items():
        out = []
        for (ctx, tag), gap in inv.unknown_elements.items():
            out.append({
                "context": ctx, "element": tag,
                "files_count": len(gap.files), "example_files": gap.examples(),
            })
        return sorted(out, key=lambda d: (d["context"], d["element"]))

    def attr_items():
        out = []
        for (el, attr), gap in inv.unknown_attributes.items():
            out.append({
                "element": el, "attribute": attr,
                "files_count": len(gap.files), "example_files": gap.examples(),
            })
        return sorted(out, key=lambda d: (d["element"], d["attribute"]))

    def enum_items():
        out = []
        for (el, attr, val), gap in inv.unknown_enum_values.items():
            out.append({
                "element": el, "attribute": attr, "value": val,
                "enum": index.info(el).enum_of.get(attr),
                "files_count": len(gap.files), "example_files": gap.examples(),
            })
        return sorted(out, key=lambda d: (d["element"], d["attribute"], d["value"]))

    def pair_items():
        out = []
        for (parent, child), gap in inv.unknown_pairs.items():
            out.append({
                "parent": parent, "child": child,
                "files_count": len(gap.files), "example_files": gap.examples(),
            })
        return sorted(out, key=lambda d: (d["parent"], d["child"]))

    n_elem_kinds = len(inv.elem_kinds)
    n_pair_kinds = len(inv.pair_kinds)
    n_attr_kinds = len(inv.attr_kinds)
    n_enum_kinds = len(inv.enum_kinds)
    u_elem = len(inv.unknown_elements)
    u_pair = len(inv.unknown_pairs)
    u_attr = len(inv.unknown_attributes)
    u_enum = len(inv.unknown_enum_values)

    return {
        "corpus": {
            "total_files": len(files),
            "mjcf": counts["mjcf"],
            "include_fragment": counts["include_fragment"],
            "non_mjcf": counts["non_mjcf"],
            "unparseable": counts["unparseable"],
        },
        "usage": {
            "elements": inv.n_elements,
            "pairs": inv.n_pairs,
            "attributes": inv.n_attributes,
            "enum_values": inv.n_enum_values,
        },
        "coverage": {
            "distinct_elements": n_elem_kinds,
            "unknown_elements": u_elem,
            "elements_covered_pct": _pct(n_elem_kinds - u_elem, n_elem_kinds),
            "distinct_pairs": n_pair_kinds,
            "unknown_pairs": u_pair,
            "pairs_covered_pct": _pct(n_pair_kinds - u_pair, n_pair_kinds),
            "distinct_attributes": n_attr_kinds,
            "unknown_attributes": u_attr,
            "attributes_covered_pct": _pct(n_attr_kinds - u_attr, n_attr_kinds),
            "distinct_enum_values": n_enum_kinds,
            "unknown_enum_values": u_enum,
            "enum_values_covered_pct": _pct(n_enum_kinds - u_enum, n_enum_kinds),
        },
        "unknown": {
            "elements": elem_items(),
            "pairs": pair_items(),
            "attributes": attr_items(),
            "enum_values": enum_items(),
        },
    }


def write_report(report: dict, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)


# --------------------------------------------------------------------------- #
# CLI                                                                          #
# --------------------------------------------------------------------------- #
def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--corpus", required=True, type=Path,
                    help="root of the MuJoCo source tree (the dir holding "
                         "test/ model/ mjx/ ...)")
    ap.add_argument("--out", type=Path, default=_DEFAULT_OUT,
                    help="output report path (default snapshots/corpus_report.json)")
    args = ap.parse_args(argv)

    corpus = args.corpus.resolve()
    if not corpus.is_dir():
        print(f"error: corpus root not found: {corpus}", file=sys.stderr)
        return 2

    report = build_report(corpus)
    write_report(report, args.out)

    c = report["corpus"]
    cov = report["coverage"]
    print(f"scanned {c['total_files']} files: {c['mjcf']} mjcf, "
          f"{c['include_fragment']} fragment, {c['non_mjcf']} non-mjcf, "
          f"{c['unparseable']} unparseable")
    print(f"elements {cov['elements_covered_pct']}% covered "
          f"({cov['unknown_elements']} unknown of {cov['distinct_elements']}); "
          f"pairs {cov['pairs_covered_pct']}% "
          f"({cov['unknown_pairs']} unknown of {cov['distinct_pairs']}); "
          f"attributes {cov['attributes_covered_pct']}% "
          f"({cov['unknown_attributes']} unknown of {cov['distinct_attributes']}); "
          f"enum values {cov['enum_values_covered_pct']}% "
          f"({cov['unknown_enum_values']} unknown of {cov['distinct_enum_values']})")
    print(f"report written to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
