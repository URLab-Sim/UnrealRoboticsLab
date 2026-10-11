"""ProtoSpec front end: upstream's MJCF schema -> the AST the emitters consume.

MuJoCo owns the MJCF grammar in `src/xml/mjcf.schema` and parses it with
`doc/generate/mjcf_schema.py`. This module imports that parser from the
enclosing checkout -- never a vendored copy, so a MuJoCo bump moves the grammar
under ProtoSpec with no action here -- applies :mod:`protospec_gen.overlay`, and
produces the JSON-serializable AST every emitter reads.

What the front end does beyond reading the schema:

* **Names.** Elements, enums, enumerators and fields get C++ identifiers from
  the overlay, defaulting to PascalCase for types and to the attribute name for
  fields. The schema's `xml=` facet stays the wire tag.
* **Variant groups.** `group orientation variant` is the schema's one variant.
  ProtoSpec stores only its first member (the canonical quat) and accepts the
  rest as input aliases folded at parse end by a named resolver, so no
  `std::variant` reaches the object model.
* **Interleaved child lists.** The overlay names ordered heterogeneous lists;
  their members derive from repeatable schema children in schema order.
* **Reference typing.** A `ref<ns>` is typed by the elements declaring `id<ns>`:
  one declarer types it directly, several type it by a union.
* **Symbolic arities.** `double[1..mjNREF]` resolves against the enclosing
  checkout's `mjmodel.h`, so the bounds track the engine's own constants.
* **Presence constraints.** The schema's `exclusive` / `together` / `requires` /
  `oneof` rows are carried through verbatim for the generated validator.

Everything else -- types, defaults, arities, enums, docs, declaration order, the
mjSpec struct and field bindings -- is passed through from the schema unchanged.

Drift gates reject stale structural bindings, a defaults snapshot from a
different source revision, and missing coverage for native-backed fields.

Drift gates, addition side: the removal gates all read the overlay and ask the
schema about it, which leaves upstream free to ADD something the overlay has
never heard of. One addition lacks upstream unit metadata and gets a heuristic
gate here:

* a new angle-valued attribute, unlisted in `ANGLE_ATTRS`, is read as radians
  whatever the document says and is wrong by 57.3x;

The angle name-and-doc heuristic fires only on attributes
``classified_attrs.json`` has never recorded; a false positive uses one
`overlay.NOT_ANGLE` waiver. Typed `ref<ns>` and standard obj/ref name-selector
pairs derive directly from the schema. The baseline is generated,
not hand-kept::

    uv run python -m protospec_gen.frontend --update-baseline

Public API: ``load_schema()`` returns the AST as a plain dict,
``write_classified_attrs()`` refreshes the baseline, and ``SchemaError`` is the
failure type (upstream's, re-exported).
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import os
import re
import sys

from . import mjdefaults, overlay, overlay_ue

__all__ = ["load_schema", "write_classified_attrs", "mujoco_src", "SchemaError",
           "OverlayError"]


class OverlayError(Exception):
    """An overlay entry that no longer matches the schema, or a gap in it."""


# --------------------------------------------------------------------------- #
# Locating the enclosing MuJoCo checkout                                       #
# --------------------------------------------------------------------------- #
ENV_VAR = "PROTOSPEC_MUJOCO_SRC"

_HERE = os.path.dirname(os.path.abspath(__file__))
_PROTOSPEC_ROOT = os.path.dirname(_HERE)


def _plausible(root: str) -> bool:
    return os.path.isfile(os.path.join(root, "src", "xml", "mjcf.schema"))


def mujoco_src() -> str:
    """The MuJoCo checkout owning the schema.

    ``PROTOSPEC_MUJOCO_SRC`` wins. Otherwise the known places, in order: the
    MuJoCo submodule of the URLab plugin that ``protospec/`` sits in, then the
    repository enclosing ``protospec/`` -- which is the case when ProtoSpec is
    checked out inside a MuJoCo tree rather than beside one.
    """
    raw = os.environ.get(ENV_VAR)
    if raw:
        root = os.path.abspath(os.path.expanduser(raw))
        if not _plausible(root):
            raise OverlayError(
                f"{ENV_VAR}={raw!r} has no src/xml/mjcf.schema")
        return root

    enclosing = os.path.dirname(_PROTOSPEC_ROOT)
    candidates = [
        os.path.join(enclosing, "third_party", "MuJoCo", "src"),
        enclosing,
    ]
    for root in candidates:
        if _plausible(root):
            return os.path.abspath(root)

    tried = ", ".join(repr(c) for c in candidates)
    raise OverlayError(
        f"no MuJoCo checkout with src/xml/mjcf.schema (tried {tried}); "
        f"set {ENV_VAR}")


def _import_parser(root: str):
    """Import upstream's schema parser from `root` under a private module name."""
    path = os.path.join(root, "doc", "generate", "mjcf_schema.py")
    if not os.path.isfile(path):
        raise OverlayError(f"no schema parser at {path}")
    name = "_protospec_mjcf_schema"
    cached = sys.modules.get(name)
    if cached is not None and getattr(cached, "__file__", None) == path:
        return cached
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# The parser is located and loaded on first use, not on import. Importing this
# module must not require a MuJoCo checkout: a caller that only wants the
# overlay tables, or a tool that reports a missing checkout itself, would
# otherwise fail at import with no chance to say anything useful.
_PARSER = None


def _parser():
    global _PARSER
    if _PARSER is None:
        _PARSER = _import_parser(mujoco_src())
    return _PARSER


# Names re-exported from upstream's parser. Resolved through the module hook so
# they cost nothing until something actually names one.
_PARSER_EXPORTS = ("SchemaError", "Attr", "Constraint", "Use")


def __getattr__(name):
    if name in _PARSER_EXPORTS:
        return getattr(_parser(), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


# --------------------------------------------------------------------------- #
# The classified-attribute baseline                                            #
# --------------------------------------------------------------------------- #
# One line per live (element, attribute), carrying the classification the
# overlay gives it: "angle" for an ANGLE_ATTRS member, "target" for a
# standard obj/ref name-selector pair, "plain" for everything else. It makes "new"
# computable, so the addition gates can ask a human about an attribute the
# schema has just grown without asking again about the ~1,600 that were already
# there. Committed, and regenerated with `--update-baseline`.
CLASSIFIED_ATTRS = os.path.join(_HERE, "classified_attrs.json")

# The angle heuristic catches a silent degree/radian error.
_ANGLE_NAME_RE = re.compile(r"angle|euler", re.I)
_ANGLE_DOC_RE = re.compile(r"degree", re.I)


def _angle_like(attr) -> bool:
    if _ANGLE_NAME_RE.search(attr.name):
        return True
    return bool(attr.doc and _ANGLE_DOC_RE.search(attr.doc))


def _classify(element: str, attr) -> str:
    if (element, attr.name) in overlay.ANGLE_ATTRS:
        return "angle"
    if attr.name in ("objname", "refname") and attr.type == "string":
        return "target"
    return "plain"


def _read_baseline() -> dict[str, str]:
    with open(CLASSIFIED_ATTRS, "r", encoding="utf-8") as fh:
        return json.load(fh)["attributes"]


_DEFINE_RE = re.compile(r"^#define\s+(mjN[A-Z]+)\s+(\d+)", re.M)


def _engine_constants(root: str) -> dict[str, int]:
    """The `mjN*` sizing constants a symbolic arity bound can name."""
    path = os.path.join(root, "include", "mujoco", "mjmodel.h")
    with open(path, "r", encoding="utf-8") as fh:
        return {m.group(1): int(m.group(2)) for m in _DEFINE_RE.finditer(fh.read())}


# --------------------------------------------------------------------------- #
# Naming                                                                       #
# --------------------------------------------------------------------------- #
def pascal(name: str) -> str:
    """PascalCase for a schema declaration name (`sensor_contact` -> ...)."""
    return "".join(p[:1].upper() + p[1:] for p in name.split("_"))


def lower_first(name: str) -> str:
    return name[:1].lower() + name[1:]


# The scalar types the schema declares, mapped to the AST's primitive names.
# `file` is a string resolved against asset directories; `chars` is text bound to
# a fixed char array. Neither needs a distinct storage type in the object model,
# and the arity of a `chars` attribute counts characters, not tokens, so it does
# not become an array.
_SCALAR_PRIM = {
    "double": "double",
    "float": "float",
    "int": "int32",
    "bool": "bool",
    "string": "string",
    "file": "string",
    "chars": "string",
}


# --------------------------------------------------------------------------- #
# The front end                                                                #
# --------------------------------------------------------------------------- #
class _Frontend:
    def __init__(self, root: str, *, include_probed_defaults: bool = True):
        self.root = root
        self.include_probed_defaults = include_probed_defaults
        self.parser = _import_parser(root)
        self.schema = self.parser.parse_file(
            os.path.join(root, "src", "xml", "mjcf.schema"))
        self.consts = _engine_constants(root)
        self.used: set = set()
        self.overrides_applied: set = set()
        self.retired_classified_attrs: list[str] = []
        self.mj_defaults = (mjdefaults.load() if include_probed_defaults
                            else {"defaults": {}, "skipped": {}})
        if include_probed_defaults:
            mjdefaults.check_source_revision(self.mj_defaults, root)

        self.elements = {n: e for n, e in self.schema.elements.items()
                         if n not in overlay.ELEMENT_SKIP}
        self.cpp_name = {n: overlay.ELEMENT_NAMES.get(n, pascal(n))
                         for n in self.elements}
        self.enum_name = {n: overlay.ENUM_NAMES.get(n, pascal(n))
                          for n in self.schema.enums}
        self._check_overlay_targets()
        self.id_declarers = self._id_declarers()
        self.unions = self._build_unions()
        self._check_interleave_reverse()
        self.ref_target = self._build_ref_targets()

    # -- overlay drift ------------------------------------------------------ #
    def _check_overlay_targets(self):
        # MuJoCo's native reader bypasses attribute schema checking only for
        # frame/replicate alias=body branches, then ignores unknown attributes
        # there. worldbody is also alias=body, but Body() explicitly rejects
        # all attributes on the world body.
        # A new alias kind needs native-reader review before inheriting that
        # permissive behavior in the UE importer.
        native_aliases = {name: element.facets["alias"]
                          for name, element in self.schema.elements.items()
                          if "alias" in element.facets}
        if native_aliases != {"worldbody": "body", "frame": "body", "replicate": "body"}:
            raise OverlayError(f"native alias attribute policy needs review: {native_aliases}")
        for name in overlay.ELEMENT_NAMES:
            if name not in self.schema.elements:
                raise OverlayError(
                    f"ELEMENT_NAMES names element {name!r}, which the schema "
                    "no longer declares")
        for name in overlay.ELEMENT_SKIP:
            if name not in self.schema.elements:
                raise OverlayError(
                    f"ELEMENT_SKIP names element {name!r}, which the schema "
                    "no longer declares")
        for enum, keyword in overlay.ENUM_MEMBERS:
            if enum not in self.schema.enums:
                raise OverlayError(
                    f"ENUM_MEMBERS names enum {enum!r}, which the schema no "
                    "longer declares")
            if keyword not in self.schema.enums[enum].keywords():
                raise OverlayError(
                    f"ENUM_MEMBERS names keyword {keyword!r}, which enum "
                    f"{enum!r} no longer declares")
        for group in overlay.VARIANT_GROUP_RESOLVERS:
            declared = self.schema.groups.get(group)
            if declared is None or not declared.variant:
                raise OverlayError(
                    f"VARIANT_GROUP_RESOLVERS names {group!r}, which is not a "
                    "variant group in the schema")

        cpp = {}
        for schema_name, name in self.cpp_name.items():
            if name in cpp:
                raise OverlayError(
                    f"elements {cpp[name]!r} and {schema_name!r} both map to "
                    f"the C++ name {name!r}")
            cpp[name] = schema_name
        enum_cpp = {}
        for schema_name, name in self.enum_name.items():
            if not name.isidentifier():
                raise OverlayError(f"enum {schema_name!r} maps to invalid C++ name {name!r}")
            if name in enum_cpp:
                raise OverlayError(
                    f"enums {enum_cpp[name]!r} and {schema_name!r} both map to "
                    f"the C++ name {name!r}")
            enum_cpp[name] = schema_name

    def _check_interleave_reverse(self):
        """Every repeatable child of an interleaved parent is in a row.

        `_build_unions` asks whether each row member is still a child, which
        catches a removal. This asks the other question, which catches an
        addition: a parent whose spec order across tags is semantic cannot have
        a repeatable child outside the ordered list, because such a child
        becomes a homogeneous list written after it, and its elements' ids then
        land after every element authored below them. The model still compiles
        and every other gate still passes; only the ids are wrong.

        Exempt: a child that aliases a listed member (`frame` and `replicate`
        carry `alias=body`, and `_build_unions` has already folded them into
        the union that admits their target), a child the overlay skips, and a
        child that cannot repeat -- `?` and `!` admit one element, which has no
        order to be wrong about.
        """
        for owner, rows in overlay.INTERLEAVE.items():
            if len(rows) != 1 or len(rows[0]) != 2:
                raise OverlayError(f"INTERLEAVE[{owner!r}] needs one list/union pair")
            if not self._interleave_members(owner):
                raise OverlayError(f"INTERLEAVE[{owner!r}] has no repeatable children")

    def _interleave_members(self, owner: str) -> list[str]:
        """Repeatable children in native schema order, excluding alias spellings."""
        declared = {c.name for c in self.elements[owner].children()}
        return [c.name for c in self.elements[owner].children()
                if c.card in ("*", "R") and c.name not in overlay.ELEMENT_SKIP
                and self.schema.elements[c.name].facets.get("alias") not in declared]

    def _classified(self) -> dict[str, str]:
        """Every live attribute, keyed `element.attribute`, with its class."""
        return {f"{name}.{attr.name}": _classify(name, attr)
                for name, element in self.elements.items()
                for attr in self.schema.expanded_attrs(element)}

    def _check_classified_attrs(self):
        """Attributes the schema has grown are classified by a human.

        The heuristics cannot be trusted to be right, only to be loud, so they
        are asked exclusively of attributes the baseline has never seen. An
        attribute already in the baseline was answered for when it arrived; one
        that is not is either classified now or waived now, with a reason.

        Reclassification and removal are checked too, so the file cannot go
        stale under an overlay edit and quietly stop computing "new" correctly.
        """
        baseline = _read_baseline()
        live = self._classified()
        problems = []
        for name, element in self.elements.items():
            attrs = {a.name: a for a in self.schema.expanded_attrs(element)}
            for value, selector in (("objname", "objtype"), ("refname", "reftype")):
                if value in attrs and attrs[value].type == "string" and selector not in attrs:
                    problems.append(f"{name}.{value} has no {selector} sibling")

        # A removed schema declaration has no generated field to classify.
        # Retire its old baseline row in memory so routine upstream removals do
        # not require editing URLab's pinned baseline. Candidate upgrade
        # reports record these names; the checked-in baseline remains pinned.
        self.retired_classified_attrs = sorted(set(baseline) - set(live))

        for name, element in self.elements.items():
            for attr in self.schema.expanded_attrs(element):
                key = f"{name}.{attr.name}"
                actual = live[key]
                recorded = baseline.get(key)
                if recorded is not None:
                    if recorded != actual:
                        problems.append(
                            f"{key} was recorded {recorded!r} and the overlay "
                            f"now makes it {actual!r}; refresh the baseline if "
                            "the change is deliberate")
                    continue
                if (_angle_like(attr) and actual != "angle"
                        and (name, attr.name) not in overlay.NOT_ANGLE):
                    problems.append(
                        f"{key} is new and reads as an angle; add it to "
                        "overlay.ANGLE_ATTRS or waive it in overlay.NOT_ANGLE")
        for label, table in (("NOT_ANGLE", overlay.NOT_ANGLE),):
            for key, reason in table.items():
                if f"{key[0]}.{key[1]}" not in live:
                    problems.append(
                        f"{label} waives {key[0]}.{key[1]}, which the schema "
                        "no longer declares; delete the entry")
                elif not (isinstance(reason, str) and reason.strip()):
                    problems.append(
                        f"{label} waives {key[0]}.{key[1]} with no reason")

        if problems:
            raise OverlayError(
                "the schema declares attributes the overlay has not "
                "classified (regenerate the baseline with `python -m "
                "protospec_gen.frontend --update-baseline` once each is "
                "answered):\n  " + "\n  ".join(sorted(problems)))

    def _check_attr_tables(self):
        """Every (element, attribute) keyed overlay entry names a live pair."""
        live = {(n, a.name) for n, e in self.elements.items()
                for a in self.schema.expanded_attrs(e)}
        tables = {}
        for label, table in tables.items():
            for key in table:
                if key not in live:
                    raise OverlayError(
                        f"{label} names {key[0]}.{key[1]}, which the schema no "
                        "longer declares")
        for key in overlay.ANGLE_ATTRS:
            if key not in live:
                raise OverlayError(
                    f"ANGLE_ATTRS names {key[0]}.{key[1]}, which the schema no "
                    "longer declares")
        live_names = {a for _e, a in live}
        for attr in sorted(overlay_ue.UE_ADVANCED):
            if attr not in live_names:
                raise OverlayError(
                    f"UE_ADVANCED names attribute {attr!r}, which no element "
                    "declares any more")
    def _check_type_overrides(self):
        """Every type correction still corrects something.

        Paired with the schema check in :meth:`_corrected`, this is what makes
        the table self-retiring: an entry whose attribute the schema dropped, or
        that a schema fix moved out from under, fails here rather than lingering
        as a correction of nothing.
        """
        for key in overlay.ATTR_TYPE_OVERRIDES:
            if key not in self.overrides_applied:
                raise OverlayError(
                    f"ATTR_TYPE_OVERRIDES corrects {key[0]}.{key[1]}, which the "
                    "schema no longer declares; delete the entry")

    # -- unions ------------------------------------------------------------- #
    def _aliased_to(self, name: str) -> list[str]:
        return [n for n, e in self.elements.items()
                if e.facets.get("alias") == name]

    def _id_declarers(self) -> dict[str, list[str]]:
        """Namespace -> declaring elements, in schema declaration order."""
        declarers: dict[str, list[str]] = {}
        for name, element in self.elements.items():
            for attr in self.schema.expanded_attrs(element):
                if attr.type == "id":
                    members = declarers.setdefault(attr.target, [])
                    if name not in members:
                        members.append(name)
        return declarers

    def _build_unions(self) -> dict:
        """Union name -> member schema element names, with namespace namesakes first."""
        unions = {}
        for owner, rows in overlay.INTERLEAVE.items():
            if owner not in self.elements:
                raise OverlayError(
                    f"INTERLEAVE names element {owner!r}, which the schema no "
                    "longer declares")
            declared = {c.name for c in self.elements[owner].children()}
            for _list_name, union_name in rows:
                members = self._interleave_members(owner)
                full = list(members)
                for member in members:
                    full.extend(a for a in self._aliased_to(member)
                                if a not in full)
                unions[union_name] = full
        for ns, union_name in overlay.REF_UNIONS.items():
            members = self.id_declarers.get(ns, [])
            if len(members) < 2:
                raise OverlayError(
                    f"REF_UNIONS[{ns!r}] names a namespace with fewer than "
                    "two id declarers in the schema")
            if union_name in unions:
                raise OverlayError(
                    f"REF_UNIONS[{ns!r}] uses duplicate union name "
                    f"{union_name!r}")
            # The namespace's namesake leads the public union when present;
            # remaining declarers keep schema order. This preserves the
            # established JointAny and FlexAny target ordering.
            unions[union_name] = ([ns] if ns in members else []) + [
                member for member in members if member != ns]
        return unions

    def _build_ref_targets(self) -> dict[str, str]:
        """Namespace -> the C++ type a `ref<ns>` is phantom-typed by."""
        targets = {}
        for ns, elems in self.id_declarers.items():
            if len(elems) == 1:
                targets[ns] = self.cpp_name[elems[0]]
                continue
            union_name = overlay.REF_UNIONS.get(ns)
            if union_name is not None:
                targets[ns] = union_name
                continue
            match = [u for u, members in self.unions.items()
                     if sorted(members) == sorted(elems)]
            if not match:
                raise OverlayError(
                    f"namespace {ns!r} is declared by {sorted(elems)} and no "
                    "union covers exactly those; add one to overlay.REF_UNIONS")
            targets[ns] = match[0]
        return targets

    # -- types -------------------------------------------------------------- #
    def _arity(self, attr) -> dict | None:
        arity = attr.arity
        if attr.type == "chars":
            return None  # a character bound, not a token count
        if arity.is_scalar():
            return None
        if arity.hi is None:
            return {"kind": "unbounded"}
        hi = arity.hi
        if isinstance(hi, str):
            if hi not in self.consts:
                raise self.parser.SchemaError(self.schema.path, attr.line,
                                  f"unknown arity bound {hi!r}")
            hi = self.consts[hi]
        if arity.lo == hi:
            return {"kind": "fixed", "size": hi}
        return {"kind": "range", "min": arity.lo, "max": hi}

    def _type(self, attr) -> dict:
        if attr.type in ("enum", "flags"):
            t = {"kind": "named", "name": self.enum_name[attr.target],
                 "category": "enum"}
            if attr.type == "flags":
                t["arity"] = {"kind": "unbounded"}
            return t
        if attr.type == "ref":
            if attr.target not in self.ref_target:
                raise self.parser.SchemaError(self.schema.path, attr.line,
                                  f"ref<{attr.target}> has no declaring element")
            return {"kind": "ref", "target": self.ref_target[attr.target]}
        if attr.type == "id":
            return {"kind": "prim", "prim": "string"}
        t = {"kind": "prim", "prim": _SCALAR_PRIM[attr.type]}
        arity = self._arity(attr)
        if arity is not None:
            t["arity"] = arity
        return t

    def _default(self, element: str, attr, type_: dict) -> dict | None:
        """The lowest layer's value for this attribute, or None.

        Two sources, in priority order. The schema's own `=` default wins: it is
        the authored contract, and where upstream wrote one it is a statement
        about MJCF rather than about a struct. Where the schema is silent,
        MuJoCo's initialised struct value stands in -- that is what the compiler
        will merge against whatever the schema says, so an attribute with no `=`
        row is not undefaulted, only undocumented.

        The struct values are the probed table in
        :mod:`protospec_gen.mjdefaults`; see it for what is deliberately absent.
        """
        value = attr.default
        if value is None:
            value = self._mujoco_default(element, attr, type_)
        if value is None:
            return None
        if attr.type == "enum":
            return {"kind": "enum",
                    "member": self._enum_member(attr.target, value)}
        if attr.type == "bool":
            return {"kind": "scalar", "value": value == "true"}
        if isinstance(value, tuple):
            return {"kind": "array", "values": list(value)}
        if isinstance(value, str):
            return {"kind": "scalar", "value": value}
        if type_["kind"] == "prim" and type_.get("arity") is not None:
            return {"kind": "array", "values": [value]}
        if type_["kind"] == "prim" and type_["prim"] == "int32":
            return {"kind": "scalar", "value": int(value)}
        return {"kind": "scalar", "value": value}

    def _mujoco_default(self, element: str, attr, type_: dict):
        """MuJoCo's initialised value for this attribute, in the schema parser's
        own spelling, so one conversion serves both sources."""
        raw = self.mj_defaults["defaults"].get(element, {}).get(attr.name)
        if raw is None:
            return None
        if isinstance(raw, bool):
            return "true" if raw else "false"
        if isinstance(raw, list):
            arity = type_.get("arity") or {}
            if arity.get("kind") == "fixed" and len(raw) != arity["size"]:
                raise mjdefaults.DefaultsError(
                    f"the probed default for {element}.{attr.name} has "
                    f"{len(raw)} values, but the schema declares "
                    f"{arity['size']}; re-probe against the pinned MuJoCo")
            return tuple(raw)
        return raw

    def _check_mujoco_defaults(self, elements: list[dict]) -> None:
        """Every probed row still names an attribute the schema declares.

        The removal half of the drift gate. The addition half -- an attribute
        the spec-write layer can reach that the probe never saw -- is asked
        where the write plan exists, in the UE emitter, because that is what
        knows which attributes are reachable at all.
        """
        if not self.include_probed_defaults:
            return
        expected = {(e["schema_name"], f["xml"]) for e in elements
                    if e.get("spec") or e["schema_name"].startswith("default_")
                    or e["schema_name"] == "freejoint" for f in e["fields"]}
        covered = mjdefaults.coverage(self.mj_defaults)
        stale = sorted(covered - expected)
        missing = sorted(expected - covered)
        if stale:
            named = ", ".join(f"{e}.{a}" for e, a in stale[:8])
            raise mjdefaults.DefaultsError(
                f"{len(stale)} probed MuJoCo default(s) name attributes the "
                f"schema no longer declares ({named}"
                + (", ..." if len(stale) > 8 else "")
                + "); re-probe with `uv run python tools/refresh_mj_defaults.py`")
        if missing:
            named = ", ".join(f"{e}.{a}" for e, a in missing[:8])
            raise mjdefaults.DefaultsError(
                f"{len(missing)} native-backed schema field(s) lack probed "
                f"coverage ({named}); re-probe with tools/refresh_mj_defaults.py")

    def _enum_member(self, enum: str, keyword: str) -> str:
        return overlay.ENUM_MEMBERS.get((enum, keyword), keyword)

    # -- attributes --------------------------------------------------------- #
    def _field_name(self, element: str, attr: str) -> str:
        return overlay.ATTR_NAMES.get((element, attr),
                                      overlay.ATTR_NAMES.get(attr, attr))

    def _variant_groups(self):
        """(group name, ordered member attribute names) for each variant group."""
        out = {}
        for name, group in self.schema.groups.items():
            if group.variant:
                out[name] = [m.name for m in group.members
                             if isinstance(m, self.parser.Attr)]
        return out

    def _element_fields(self, name: str, element) -> list[dict]:
        variants = self._variant_groups()
        # Attributes folded away: the tail of every variant group the element
        # uses, plus the overlay's explicit input aliases.
        folded: dict[str, tuple[str, str]] = {}  # attr -> (canonical, resolver)
        for member in element.members:
            if isinstance(member, self.parser.Use) and member.group in variants:
                names = variants[member.group]
                resolver = overlay.VARIANT_GROUP_RESOLVERS[member.group]
                for alias in names[1:]:
                    folded[alias] = (names[0], resolver)
        self.variant_alias_keys.update((name, a) for a in folded)

        fields = []
        aliases: list[tuple[str, str]] = []
        for attr, group in self._attrs_with_group(element):
            if attr.name in folded:
                aliases.append((attr.name, folded[attr.name][1]))
                continue
            field = self._field(name, attr, folded)
            # A variant group is a set of spellings for one value, and all but
            # the canonical one is folded away, so tagging what survives with
            # the group name would name a section of one.
            if group is not None and group not in variants:
                field["group"] = group
            fields.append(field)
        self.element_aliases[name] = aliases
        return fields

    def _attrs_with_group(self, element) -> list[tuple]:
        """`expanded_attrs`, paired with the `group` each attribute came from.

        The schema splices a `use` group's attributes into the element as though
        they had been written there, and everything downstream of the grammar is
        right to treat them that way. A presentation layer is the exception: a
        group is the schema's own statement that a run of attributes belongs
        together and under a name, and it is the only such statement the schema
        makes, so the provenance `expanded_attrs` discards is recovered here. An
        attribute written inline pairs with None; one spliced from a group
        nested inside another pairs with the group the element actually named,
        which is the one a reader would recognize.

        The pairing is checked against `expanded_attrs` rather than trusted,
        because it re-walks the same members and a bump that changed how a
        group expands would otherwise mislabel silently instead of failing.
        """
        paired: list[tuple] = []
        for member in element.members:
            if isinstance(member, self.parser.Attr):
                paired.append((member, None))
            elif isinstance(member, self.parser.Use):
                for attr in self._group_attrs(member.group):
                    paired.append((attr, member.group))
        expanded = self.schema.expanded_attrs(element)
        if [a for a, _g in paired] != expanded:
            raise OverlayError(
                f"group provenance for element {element.name!r} does not "
                "reproduce expanded_attrs; upstream changed how a `use` group "
                "splices and _attrs_with_group must follow it")
        return paired

    def _group_attrs(self, name: str) -> list:
        """A group's attributes, with any group it uses spliced in."""
        out = []
        for member in self.schema.groups[name].members:
            if isinstance(member, self.parser.Attr):
                out.append(member)
            elif isinstance(member, self.parser.Use):
                out.extend(self._group_attrs(member.group))
        return out

    def _corrected(self, element: str, attr):
        """An attribute the overlay retypes, or the schema's own declaration.

        The override is checked against the schema before it is applied, so it
        can only ever correct the declaration it was written for.
        """
        row = overlay.ATTR_TYPE_OVERRIDES.get((element, attr.name))
        if row is None:
            return attr
        declared = (attr.type, attr.target, (attr.arity.lo, attr.arity.hi),
                    bool(attr.facets.get("required", False)))
        if declared == tuple(row["corrected"]):
            self.overrides_applied.add((element, attr.name))
            return attr
        if declared != tuple(row["observed"]):
            raise OverlayError(
                f"ATTR_TYPE_OVERRIDES corrects {element}.{attr.name} from "
                f"{tuple(row['observed'])!r}, but the schema now declares it "
                f"{declared!r}. Re-read the schema: if upstream fixed it, "
                "delete the entry; if it changed some other way, restate the "
                "correction against the new declaration.")
        self.overrides_applied.add((element, attr.name))
        new_type, new_target, (lo, hi), new_required = row["corrected"]
        facets = dict(attr.facets)
        if new_required:
            facets["required"] = True
        else:
            facets.pop("required", None)
        return dataclasses.replace(attr, type=new_type, target=new_target,
                                   arity=self.parser.Arity(lo, hi), facets=facets)

    def _field(self, element: str, attr, folded: dict) -> dict:
        attr = self._corrected(element, attr)
        type_ = self._type(attr)
        key = (element, attr.name)
        cpp = self._field_name(element, attr.name)
        annotations: dict[str, object] = {}
        if cpp != attr.name:
            annotations["xml"] = attr.name
        if key in overlay.ANGLE_ATTRS:
            annotations["unit"] = "angle"
        if attr.name in ("objname", "refname") and attr.type == "string":
            annotations["target_from"] = {"objname": "objtype", "refname": "reftype"}[attr.name]
        folded_in = sorted(a for a, (c, _r) in folded.items() if c == attr.name)
        # A field that others fold into is read by the resolver that owns the
        # fold, whether or not the schema marks it custom: it is the resolver
        # that decides which authored spelling won.
        resolver = folded[folded_in[0]][1] if folded_in else None
        if resolver is not None:
            annotations["resolver"] = resolver
        if folded_in:
            annotations["aliases"] = " ".join(folded_in)
        if "field" in attr.facets:
            annotations["spec_field"] = attr.facets["field"]
        if attr.facets.get("reading") == "custom":
            annotations["reading"] = "custom"
        if attr.facets.get("writing") == "custom":
            annotations["writing"] = "custom"
        if "pattern" in attr.facets:
            annotations["pattern"] = attr.facets["pattern"]
        if attr.type == "chars":
            annotations["max_chars"] = int(attr.arity.hi)
        if attr.type == "file":
            annotations["file"] = True
        if attr.type == "id":
            annotations["namespace"] = attr.target
        if attr.facets.get("nodefault"):
            annotations["nodefault"] = True
        # Numeric range facets, carried verbatim. They are the schema's own
        # statement about admissible values and the only one it makes, so an
        # editor has somewhere to get an entry field's bounds from other than a
        # guess. `positive` is a bound with no number: an emitter that wants one
        # decides what to do with it, because what "greater than zero" becomes
        # depends on whether the target's clamp is inclusive.
        if "min" in attr.facets:
            annotations["min"] = attr.facets["min"]
        if "max" in attr.facets:
            annotations["max"] = attr.facets["max"]
        if attr.facets.get("positive"):
            annotations["positive"] = True

        field = {
            "name": cpp,
            "xml": attr.name,
            "type": type_,
            "optional": not attr.facets.get("required", False),
        }
        if annotations:
            field["annotations"] = dict(sorted(annotations.items()))
        default = self._default(element, attr, type_)
        if default is not None:
            field["default"] = default
        if attr.doc:
            field["doc"] = attr.doc
        return field

    # -- children ----------------------------------------------------------- #
    def _child_source(self, name: str, element):
        """The element's child rows: its own, or its alias target's.

        `alias=body` means `mjXSchema::NameMatch` validates the tag against the
        body row, so the alias admits the whole body surface. `frame` and
        `replicate` therefore carry a body's subtree, and carry it as the same
        ordered heterogeneous list, because cross-tag spec order inside a
        `<frame>` is as semantic as it is inside a `<body>`.

        The aliases also declare child rows of their own. Those are the
        narrower surface the *reader* enforces -- no `<freejoint>` under
        `<replicate>`, no `<inertial>` under `<worldbody>` -- recorded for
        emitters that want the compilable subset. ProtoSpec models what the
        grammar admits and lets MuJoCo reject the rest at compile, so it reads
        the alias target's rows regardless.
        """
        alias = element.facets.get("alias")
        if alias:
            return alias, self.schema.elements[alias]
        return name, element

    def _element_children(self, name: str, element) -> list[dict]:
        name, element = self._child_source(name, element)
        rows = overlay.INTERLEAVE.get(name, [])
        interleaved = set(self._interleave_members(name)) if rows else set()
        declared = {c.name for c in element.children()}
        out = []
        emitted_unions = set()
        for child in element.children():
            # A child that aliases a sibling child is admitted through that
            # sibling's row, and `_build_unions` has already folded it into the
            # union the sibling belongs to. Emitting its own declaration too
            # would route one tag down two paths.
            if self.schema.elements[child.name].facets.get("alias") in declared:
                continue
            if child.name in interleaved:
                for list_name, union_name in rows:
                    members = self._interleave_members(name)
                    if child.name in members and union_name not in emitted_unions:
                        emitted_unions.add(union_name)
                        out.append({
                            "name": list_name,
                            "union": union_name,
                            "cardinality": "zero_or_more",
                            "tag": "",
                        })
                continue
            if child.name in overlay.ELEMENT_SKIP:
                continue
            member = overlay.CHILD_NAMES.get(
                (name, child.name), lower_first(self.cpp_name[child.name]))
            tag = overlay.CHILD_TAGS.get(
                (name, member), self.elements[child.name].xml_name())
            out.append({
                "name": member,
                "element": self.cpp_name[child.name],
                "cardinality": _CARD[child.card],
                "tag": tag,
            })
        return out

    # -- assembly ----------------------------------------------------------- #
    def build(self) -> dict:
        self.variant_alias_keys = set()
        self.element_aliases = {}
        self._check_classified_attrs()
        self._check_attr_tables()
        self.overrides_applied.clear()

        elements = []
        for name, element in self.elements.items():
            fields = self._element_fields(name, element)
            # The standalone value profile folds alternate spellings into a
            # canonical field. Unreal's direct XML profile instead exposes
            # ordinary source attributes verbatim. Orientation spellings are
            # transient spatial import inputs; they are normalized into the
            # editor transform after the whole document is read.
            stored_xml = {f["xml"] for f in fields}
            alias_resolvers = dict(self.element_aliases.get(name, []))
            raw_aliases = []
            spatial_inputs = []
            for attr, _group in self._attrs_with_group(element):
                if attr.name in stored_xml:
                    continue
                raw = self._field(name, attr, {})
                raw_annotations = raw.get("annotations", {})
                raw_annotations.pop("resolver", None)
                raw_annotations.pop("aliases", None)
                if raw_annotations:
                    raw["annotations"] = raw_annotations
                else:
                    raw.pop("annotations", None)
                if alias_resolvers.get(attr.name) == "orientation":
                    spatial_inputs.append(raw)
                else:
                    raw_aliases.append(raw)
            children = self._element_children(name, element)
            row = {
                "name": self.cpp_name[name],
                "schema_name": name,
                "xml": element.xml_name(),
                "fields": fields,
                "ue_raw_aliases": raw_aliases,
                "ue_spatial_inputs": spatial_inputs,
                "children": children,
            }
            # `alias=body` says mjXSchema validates the tag against the body
            # row -- and, because that check is reached through the recursive
            # branch, never validates the aliased element's own attributes. The
            # reader needs to know, so it is no stricter than the engine.
            if "alias" in element.facets:
                row["alias"] = element.facets["alias"]
            if element.spec:
                row["spec"] = element.spec
                if "field" in element.facets:
                    row["spec_field"] = element.facets["field"]
            consts = [{"field": c.field, "value": c.value}
                      for c in element.consts()]
            if consts:
                row["consts"] = consts
            constraints = self._constraints(name, element, fields)
            if constraints:
                row["constraints"] = constraints
            aliases = self.element_aliases.get(name, [])
            if aliases:
                row["input_aliases"] = [{"attr": a, "resolver": r}
                                        for a, r in aliases]
            elements.append(row)

        self._check_child_names(elements)
        self._check_type_overrides()
        self._check_mujoco_defaults(elements)

        return {
            "mujoco_schema": os.path.relpath(
                self.schema.path, self.root).replace(os.sep, "/"),
            "enums": [self._enum(n) for n in self.schema.enums],
            "unions": [{"name": u, "members": [self.cpp_name[m] for m in members]}
                       for u, members in sorted(self.unions.items())],
            "elements": elements,
        }

    def _enum(self, name: str) -> dict:
        enum = self.schema.enums[name]
        row = {
            "name": self.enum_name[name],
            "schema_name": name,
            "members": [self._enum_member_row(name, keyword, c)
                        for keyword, c in enum.items],
        }
        if enum.ctype:
            row["ctype"] = enum.ctype
        if enum.doc:
            row["doc"] = enum.doc
        return row

    def _enum_member_row(self, name: str, keyword: str, c) -> dict:
        """One enum member: its C++ name, its MJCF keyword, and its C constant.

        The constant is what a spec write has to store in the mjs struct, whose
        fields are the C enums rather than the keywords. It is carried verbatim,
        constant or numeric literal alike, because the schema is the authority on
        which of the two a member is spelled with. A member with no binding
        carries no key at all rather than a null.
        """
        row = {"name": self._enum_member(name, keyword), "value": keyword}
        if c is not None:
            row["c"] = c
        return row

    def _constraints(self, name: str, element, fields: list[dict]) -> list[dict]:
        """The element's presence constraints, plus those its groups carry.

        Bundles name XML attributes; a bundle naming an attribute that folded
        into a canonical field (an orientation spelling) is dropped, because
        presence of the fold is what the constraint is really about and the
        reader has already reported a multi-spelling conflict.
        """
        stored = {f["xml"] for f in fields}
        rows = list(element.constraints())
        for member in element.members:
            if isinstance(member, self.parser.Use):
                rows.extend(m for m in self.schema.groups[member.group].members
                            if isinstance(m, self.parser.Constraint))
        out = []
        for con in rows:
            bundles = [list(b) for b in con.bundles
                       if all(a in stored for a in b)]
            if len(bundles) < 2:
                continue
            row = {"kind": con.kind, "bundles": bundles}
            if con.doc:
                row["doc"] = con.doc
            out.append(row)
        return out

    def _check_child_names(self, elements: list[dict]):
        for name in overlay.CHILD_NAMES:
            if name[0] not in self.elements:
                raise OverlayError(
                    f"CHILD_NAMES names element {name[0]!r}, which the schema "
                    "no longer declares")
            if name[1] not in {c.name for c in self.elements[name[0]].children()}:
                raise OverlayError(
                    f"CHILD_NAMES names child {name[1]!r} of {name[0]!r}, which "
                    "the schema no longer declares")
        for row in elements:
            members = {f["name"] for f in row["fields"]}
            for child in row["children"]:
                if child["name"] in members:
                    raise OverlayError(
                        f"{row['name']}.{child['name']} is both an attribute "
                        "and a child list; give the list a CHILD_NAMES entry")
                members.add(child["name"])


_CARD = {"?": "zero_or_one", "!": "one", "*": "zero_or_more",
         "R": "zero_or_more"}


def load_schema(root: str | None = None, *, include_probed_defaults: bool = True) -> dict:
    """Parse the MJCF schema and apply the overlay. Refresh may bypass an old defaults snapshot."""
    return _Frontend(root or mujoco_src(), include_probed_defaults=include_probed_defaults).build()


def retired_classified_attrs(root: str | None = None) -> list[str]:
    """Prior baseline keys absent from this source's schema.

    The result is informational for upgrade reports. Removed declarations are
    ignored in memory, while live reclassifications still fail the drift gate.
    """
    frontend = _Frontend(root or mujoco_src(), include_probed_defaults=False)
    live = frontend._classified()
    return sorted(set(_read_baseline()) - set(live))


def write_classified_attrs(root: str | None = None,
                           path: str | None = None) -> int:
    """Rewrite the classified-attribute baseline; return the entry count.

    Deliberately does not go through :func:`load_schema`: the gate this file
    feeds is what stands between an unclassified addition and a silently wrong
    model, and a refresh that had to pass the gate first could never be used to
    answer it.
    """
    fe = _Frontend(root or mujoco_src())
    attrs = fe._classified()
    payload = {"attributes": {k: attrs[k] for k in sorted(attrs)}}
    with open(path or CLASSIFIED_ATTRS, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(payload, fh, indent=2)
        fh.write("\n")
    return len(attrs)


def _main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m protospec_gen.frontend",
        description="ProtoSpec front end maintenance.")
    parser.add_argument(
        "--update-baseline", action="store_true",
        help="rewrite classified_attrs.json from the current schema and overlay")
    parser.add_argument("--root", help="MuJoCo checkout to read the schema from")
    args = parser.parse_args(argv)
    if not args.update_baseline:
        parser.error("nothing to do; pass --update-baseline")
    count = write_classified_attrs(args.root)
    print(f"{CLASSIFIED_ATTRS}: {count} attributes")
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
