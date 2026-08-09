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
* **Interleaved child lists.** The schema declares one `child` row per tag; MJCF
  sections whose cross-tag spec order is semantic need one ordered
  heterogeneous list. The overlay says which rows collapse; membership picks up
  every element the schema aliases (`alias=`) to a member, which is how `frame`
  and `replicate` reach the body child list.
* **Reference typing.** A `ref<ns>` is typed by the elements declaring `id<ns>`:
  one declarer types it directly, several type it by a union.
* **Symbolic arities.** `double[1..mjNREF]` resolves against the enclosing
  checkout's `mjmodel.h`, so the bounds track the engine's own constants.
* **Presence constraints.** The schema's `exclusive` / `together` / `requires` /
  `oneof` rows are carried through verbatim for the generated validator.

Everything else -- types, defaults, arities, enums, docs, declaration order, the
mjSpec struct and field bindings -- is passed through from the schema unchanged.

Drift gates: an overlay entry naming something the schema no longer declares is
an error, as is a `reading=custom` or `writing=custom` facet the overlay neither
binds to a handler nor waives with a reason.

Public API: ``load_schema()`` returns the AST as a plain dict, and
``SchemaError`` is the failure type (upstream's, re-exported).
"""

from __future__ import annotations

import dataclasses
import importlib.util
import os
import re
import sys

from . import overlay, overlay_ue

__all__ = ["load_schema", "mujoco_src", "SchemaError", "OverlayError"]


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
    def __init__(self, root: str):
        self.root = root
        self.schema = _parser().parse_file(
            os.path.join(root, "src", "xml", "mjcf.schema"))
        self.consts = _engine_constants(root)
        self.used: set = set()
        self.overrides_applied: set = set()

        self.elements = {n: e for n, e in self.schema.elements.items()
                         if n not in overlay.ELEMENT_SKIP}
        self.cpp_name = {n: overlay.ELEMENT_NAMES.get(n, pascal(n))
                         for n in self.elements}
        self.enum_name = {n: overlay.ENUM_NAMES.get(n, pascal(n))
                          for n in self.schema.enums}
        self._check_overlay_targets()
        self.unions = self._build_unions()
        self.ref_target = self._build_ref_targets()

    # -- overlay drift ------------------------------------------------------ #
    def _check_overlay_targets(self):
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
        for name in overlay.ENUM_NAMES:
            if name not in self.schema.enums:
                raise OverlayError(
                    f"ENUM_NAMES names enum {name!r}, which the schema no "
                    "longer declares")
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

    def _check_attr_tables(self):
        """Every (element, attribute) keyed overlay entry names a live pair."""
        live = {(n, a.name) for n, e in self.elements.items()
                for a in self.schema.expanded_attrs(e)}
        tables = {
            "INPUT_ALIASES": overlay.INPUT_ALIASES,
            "READ_HANDLERS": overlay.READ_HANDLERS,
            "READ_NOTES": overlay.READ_NOTES,
            "WRITE_NOTES": overlay.WRITE_NOTES,
            "TARGET_FROM": overlay.TARGET_FROM,
        }
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
        for element in overlay.ELEMENT_INPUT_ALIASES:
            if element not in self.elements:
                raise OverlayError(
                    f"ELEMENT_INPUT_ALIASES names element {element!r}, which "
                    "the schema no longer declares")
        for target, alias in overlay.INPUT_ALIASES.items():
            if (target[0], alias) not in live:
                raise OverlayError(
                    f"INPUT_ALIASES folds {target[0]}.{target[1]} into "
                    f"{alias!r}, which {target[0]} does not declare")

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

    def _check_custom_facets(self):
        """Every schema custom facet binds to a handler or carries a reason."""
        unhandled = []
        for name, element in self.elements.items():
            for attr in self.schema.expanded_attrs(element):
                key = (name, attr.name)
                if attr.facets.get("reading") == "custom":
                    if (key not in overlay.READ_HANDLERS
                            and key not in overlay.READ_NOTES
                            and key not in self.variant_alias_keys):
                        unhandled.append(f"reading=custom {name}.{attr.name}")
                if attr.facets.get("writing") == "custom":
                    if key not in overlay.WRITE_NOTES:
                        unhandled.append(f"writing=custom {name}.{attr.name}")
        if unhandled:
            raise OverlayError(
                "schema custom facets with no handler binding and no waiver "
                "(add each to overlay.READ_HANDLERS/READ_NOTES or "
                "overlay.WRITE_NOTES):\n  " + "\n  ".join(sorted(unhandled)))

    def _spec_write_keys(self) -> set:
        """The attributes the spec write has to have an answer for.

        An attribute the reader interprets rather than stores is an attribute
        whose write cannot be assumed plain, so the same set drives both: the
        schema's own `reading=custom` facets, plus the attributes the overlay
        binds to a resolver.
        """
        keys = set()
        for name, element in self.elements.items():
            for attr in self.schema.expanded_attrs(element):
                if attr.facets.get("reading") == "custom":
                    keys.add((name, attr.name))
        keys |= set(overlay.INPUT_ALIASES)
        keys |= set(overlay.READ_HANDLERS)
        keys |= set(self.variant_alias_keys)
        return keys

    def _check_spec_write(self):
        """Every element has a creation category and a target where it needs
        one, and every interpreted attribute a spec-write disposition.

        The other half of the rule -- that an attribute with no field on the
        bound struct is named somewhere -- cannot be asked here, because it
        needs the mjs struct layouts and only the emitter reads those. This gate
        covers what the schema alone answers; the emitter's gate covers the rest
        and is what makes the two-way check exhaustive.
        """
        problems = []

        for name in self.elements:
            if name not in overlay_ue.SPEC_CREATE:
                problems.append(f"element {name} has no SPEC_CREATE category")
        for name in overlay_ue.SPEC_CREATE:
            if name not in self.elements:
                problems.append(
                    f"SPEC_CREATE names {name}, which the schema no longer declares")

        problems += self._check_spec_target()

        for key in self._spec_write_keys():
            if (key not in overlay_ue.SPEC_WRITE_HANDLERS
                    and key not in overlay_ue.SPEC_WRITE_NOTES):
                problems.append(
                    f"{key[0]}.{key[1]} has no spec-write handler and no waiver")

        for element in overlay_ue.SPEC_WRITE_ELEMENT_HOOKS:
            if element not in self.elements:
                problems.append(
                    f"SPEC_WRITE_ELEMENT_HOOKS names {element}, which the "
                    "schema no longer declares")

        for table, label in ((overlay_ue.SPEC_WRITE_HANDLERS, "SPEC_WRITE_HANDLERS"),
                             (overlay_ue.SPEC_WRITE_NOTES, "SPEC_WRITE_NOTES")):
            for element, attr in table:
                if element not in self.elements:
                    problems.append(
                        f"{label} names {element}.{attr}, and the schema no "
                        "longer declares that element")
                    continue
                live = {a.name for a in
                        self.schema.expanded_attrs(self.elements[element])}
                if attr not in live:
                    problems.append(
                        f"{label} names {element}.{attr}, which {element} no "
                        "longer declares")

        if problems:
            raise OverlayError(
                "spec-write bindings out of step with the schema (fix "
                "overlay_ue.SPEC_CREATE, overlay_ue.SPEC_TARGET, "
                "overlay_ue.SPEC_WRITE_HANDLERS or "
                "overlay_ue.SPEC_WRITE_NOTES):\n  " + "\n  ".join(sorted(problems)))

    def _check_spec_target(self) -> list[str]:
        """Exactly the embedded elements carry a target expression.

        A visual sub-block is excluded in both directions: its own `field=`
        facet already names it relative to its parent, so a row here would be a
        second spelling of the same fact, free to disagree with it.
        """
        problems = []
        wanted = {
            name for name, element in self.elements.items()
            if overlay_ue.SPEC_CREATE.get(name, (None, None))[0] in
            ("spec_embedded", "parent_embedded") and "field" not in element.facets
        }
        for name in sorted(wanted - set(overlay_ue.SPEC_TARGET)):
            problems.append(
                f"element {name} is {overlay_ue.SPEC_CREATE[name][0]} and has no "
                "SPEC_TARGET row, so there is nowhere to write it")
        for name in sorted(set(overlay_ue.SPEC_TARGET) - wanted):
            problems.append(
                f"SPEC_TARGET names {name}, which is not an embedded element "
                "needing a target expression")
        return problems

    # -- unions ------------------------------------------------------------- #
    def _aliased_to(self, name: str) -> list[str]:
        return [n for n, e in self.elements.items()
                if e.facets.get("alias") == name]

    def _build_unions(self) -> dict:
        """Union name -> member schema element names, in declaration order."""
        unions = {}
        for owner, rows in overlay.INTERLEAVE.items():
            if owner not in self.elements:
                raise OverlayError(
                    f"INTERLEAVE names element {owner!r}, which the schema no "
                    "longer declares")
            declared = {c.name for c in self.elements[owner].children()}
            for _list_name, union_name, members in rows:
                missing = [m for m in members if m not in declared]
                if missing:
                    raise OverlayError(
                        f"INTERLEAVE[{owner!r}] union {union_name!r} names "
                        f"{missing}, which are not children of {owner!r}")
                full = list(members)
                for member in members:
                    full.extend(a for a in self._aliased_to(member)
                                if a not in full)
                unions[union_name] = full
        for ns, (union_name, members) in overlay.REF_UNIONS.items():
            for member in members:
                if member not in self.elements:
                    raise OverlayError(
                        f"REF_UNIONS[{ns!r}] names element {member!r}, which "
                        "the schema no longer declares")
            unions[union_name] = list(members)
        return unions

    def _build_ref_targets(self) -> dict[str, str]:
        """Namespace -> the C++ type a `ref<ns>` is phantom-typed by."""
        declarers: dict[str, list[str]] = {}
        for name, element in self.elements.items():
            for attr in self.schema.expanded_attrs(element):
                if attr.type == "id":
                    declarers.setdefault(attr.target, []).append(name)
        targets = {}
        for ns, elems in declarers.items():
            if len(elems) == 1:
                targets[ns] = self.cpp_name[elems[0]]
                continue
            union = overlay.REF_UNIONS.get(ns)
            if union is not None:
                union_name, members = union
                if sorted(members) != sorted(elems):
                    raise OverlayError(
                        f"REF_UNIONS[{ns!r}] lists {sorted(members)} but the "
                        f"schema declares id<{ns}> on {sorted(elems)}")
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
                raise _parser().SchemaError(self.schema.path, attr.line,
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
                raise _parser().SchemaError(self.schema.path, attr.line,
                                  f"ref<{attr.target}> has no declaring element")
            return {"kind": "ref", "target": self.ref_target[attr.target]}
        if attr.type == "id":
            return {"kind": "prim", "prim": "string"}
        t = {"kind": "prim", "prim": _SCALAR_PRIM[attr.type]}
        arity = self._arity(attr)
        if arity is not None:
            t["arity"] = arity
        return t

    def _default(self, attr, type_: dict) -> dict | None:
        if attr.default is None:
            return None
        value = attr.default
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
                             if isinstance(m, _parser().Attr)]
        return out

    def _element_fields(self, name: str, element) -> list[dict]:
        variants = self._variant_groups()
        # Attributes folded away: the tail of every variant group the element
        # uses, plus the overlay's explicit input aliases.
        folded: dict[str, tuple[str, str]] = {}  # attr -> (canonical, resolver)
        for member in element.members:
            if isinstance(member, _parser().Use) and member.group in variants:
                names = variants[member.group]
                resolver = overlay.VARIANT_GROUP_RESOLVERS[member.group]
                for alias in names[1:]:
                    folded[alias] = (names[0], resolver)
        for (owner, alias), canonical in overlay.INPUT_ALIASES.items():
            if owner == name:
                resolver = overlay.READ_HANDLERS.get((owner, alias))
                if resolver is None:
                    raise OverlayError(
                        f"INPUT_ALIASES[{owner}.{alias}] has no READ_HANDLERS "
                        "entry naming the resolver that folds it")
                folded[alias] = (canonical, resolver)
        self.variant_alias_keys.update((name, a) for a in folded)

        # An element-level alias is canonicalized into a child list, so it has no
        # field either; it is folded with no canonical sibling.
        into_children = dict(overlay.ELEMENT_INPUT_ALIASES.get(name, ()))
        self.variant_alias_keys.update((name, a) for a in into_children)

        fields = []
        aliases: list[tuple[str, str]] = []
        for attr, group in self._attrs_with_group(element):
            if attr.name in folded:
                aliases.append((attr.name, folded[attr.name][1]))
                continue
            if attr.name in into_children:
                aliases.append((attr.name, into_children[attr.name]))
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
            if isinstance(member, _parser().Attr):
                paired.append((member, None))
            elif isinstance(member, _parser().Use):
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
            if isinstance(member, _parser().Attr):
                out.append(member)
            elif isinstance(member, _parser().Use):
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
                                   arity=_parser().Arity(lo, hi), facets=facets)

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
        if key in overlay.TARGET_FROM:
            annotations["target_from"] = overlay.TARGET_FROM[key]
        folded_in = sorted(a for a, (c, _r) in folded.items() if c == attr.name)
        # A field that others fold into is read by the resolver that owns the
        # fold, whether or not the schema marks it custom: it is the resolver
        # that decides which authored spelling won.
        resolver = overlay.READ_HANDLERS.get(key)
        if resolver is None and folded_in:
            resolver = folded[folded_in[0]][1]
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

        field = {
            "name": cpp,
            "xml": attr.name,
            "type": type_,
            "optional": not attr.facets.get("required", False),
        }
        if annotations:
            field["annotations"] = dict(sorted(annotations.items()))
        default = self._default(attr, type_)
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
        interleaved = {m for _l, _u, members in rows for m in members}
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
                for list_name, union_name, members in rows:
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
        self._check_attr_tables()
        self.overrides_applied.clear()

        elements = []
        for name, element in self.elements.items():
            fields = self._element_fields(name, element)
            children = self._element_children(name, element)
            row = {
                "name": self.cpp_name[name],
                "schema_name": name,
                "xml": element.xml_name(),
                "fields": fields,
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

        self._check_custom_facets()
        self._check_spec_write()
        self._check_child_names(elements)
        self._check_type_overrides()

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
            if isinstance(member, _parser().Use):
                rows.extend(m for m in self.schema.groups[member.group].members
                            if isinstance(m, _parser().Constraint))
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


def load_schema(root: str | None = None) -> dict:
    """Parse the MJCF schema, apply the overlay, return the emitter AST."""
    return _Frontend(root or mujoco_src()).build()
