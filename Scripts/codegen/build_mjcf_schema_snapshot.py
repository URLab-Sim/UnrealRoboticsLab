# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""Parse the MJCF schema declaration in MuJoCo's ``xml_native_reader.cc``
and emit ``Scripts/mjcf_schema_snapshot.json`` in URLab's curated shape.

The schema source lives in a single C++ initializer-list array
``std::vector<const char*> MJCF[nMJCF]`` and looks like:

    {"body",       "*",  "name", "childclass", "pos", ...},
    {"<"},
        {"inertial", "?", "pos", "mass", ...},
        {"joint",    "*", "name", "type", ...},
        ...
    {">"},

This script tokenises that file into (element, occurrence, attrs) tuples
with parent/child relationships derived from the ``"<"`` / ``">"`` markers,
then projects the tree onto URLab's hand-curated snapshot shape (the same
shape codegen_rules.json consumes).

Usage:
    python Scripts/codegen/build_mjcf_schema_snapshot.py
        [--src PATH]    (default: third_party/MuJoCo/src/src/xml/xml_native_reader.cc)
        [--output PATH] (default: Scripts/mjcf_schema_snapshot.json)
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional


# ----------------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------------

# Matches one C++ initializer block on a line, capturing the comma-separated
# string list inside the outer braces. We deliberately tolerate any whitespace,
# trailing commas, and continuation across lines.
_BLOCK_RE = re.compile(r"\{((?:[^{}]|\{[^}]*\})*?)\}\s*,?", re.DOTALL)
_STRING_RE = re.compile(r'"([^"\\]*)"')


@dataclass
class SchemaNode:
    name: str
    occurrence: str = "*"
    attrs: List[str] = field(default_factory=list)
    children: List["SchemaNode"] = field(default_factory=list)


def _extract_mjcf_block(src_text: str) -> str:
    """Pull out the body of ``std::vector<const char*> MJCF[nMJCF] = { ... };``."""
    start_marker = "MJCF[nMJCF]"
    i = src_text.find(start_marker)
    if i < 0:
        raise SystemExit(f"could not find '{start_marker}' in source")
    # find the '=' then the opening '{'
    eq = src_text.find("=", i)
    open_brace = src_text.find("{", eq)
    # walk braces to find the matching close
    depth = 0
    j = open_brace
    n = len(src_text)
    while j < n:
        c = src_text[j]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                # strip the outermost braces — return body
                return src_text[open_brace + 1:j]
        j += 1
    raise SystemExit("unbalanced braces in MJCF declaration")


def _tokenise_blocks(body: str) -> List[List[str]]:
    """Return a flat list of ``[strings...]`` for each ``{...}`` block in the
    MJCF body, in source order. Comments are stripped (``//`` line comments
    and ``/* ... */`` block comments)."""
    # Strip /* ... */
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)
    # Strip // ... to end-of-line
    body = re.sub(r"//[^\n]*", "", body)

    blocks: List[List[str]] = []
    for m in _BLOCK_RE.finditer(body):
        inner = m.group(1)
        strings = _STRING_RE.findall(inner)
        if strings:
            blocks.append(strings)
    return blocks


def _parse_tree(blocks: List[List[str]]) -> SchemaNode:
    """Build a tree from the flat block list using ``"<"`` / ``">"`` markers."""
    root = SchemaNode(name="__root__")
    # Each entry on the stack is (parent_node, last_added_child_under_it).
    # When we see ``"<"`` we recurse INTO the last child of the current top.
    # When we see ``">"`` we pop back out.
    stack: List[SchemaNode] = [root]
    for block in blocks:
        head = block[0]
        if head == "<":
            # Open scope on the last-added child of the current top.
            top = stack[-1]
            if not top.children:
                # Defensive: malformed input. Fall back to treating the
                # current top itself as the scope.
                stack.append(top)
            else:
                stack.append(top.children[-1])
            continue
        if head == ">":
            stack.pop()
            continue
        # Element declaration: ("name", "occurrence", "attr1", "attr2", ...)
        occ = block[1] if len(block) > 1 else "*"
        attrs = block[2:]
        node = SchemaNode(name=head, occurrence=occ, attrs=list(attrs))
        stack[-1].children.append(node)
    return root


def _find_child(parent: SchemaNode, name: str) -> Optional[SchemaNode]:
    for c in parent.children:
        if c.name == name:
            return c
    return None


# ----------------------------------------------------------------------------
# Bucketing into URLab's curated snapshot shape
# ----------------------------------------------------------------------------

# Top-level worldbody children whose attrs we expose directly. The schema also
# has these under <default>; we use the worldbody copy because it carries
# `name`/`class` which the default copy doesn't.
_WORLDBODY_DIRECT = [
    "body", "inertial", "joint", "freejoint", "geom", "site",
    "camera", "light",
]


# URLab convention for splitting per-subtype attr lists into "common" (lives
# on the base UMjActuator / UMjSensor) vs "extras" (lives on the subtype).
# These are NOT strict intersections — they're the URLab-side curation of
# "what makes sense as a shared base attr". Without baking them in, new
# MuJoCo subtypes that don't carry every common attr would shrink the
# intersection and force the codegen to re-emit common attrs on every
# subtype.
#
# When MuJoCo adds a brand-new attr that should be considered "common" (e.g.
# a new lifecycle hook on every actuator), extend the list below. The
# extractor's "added attrs to known elements" warning will surface the
# candidate.
_ACTUATOR_COMMON_ATTRS = [
    "name", "class", "group", "nsample", "interp", "delay",
    "ctrllimited", "forcelimited", "actlimited",
    "ctrlrange", "forcerange", "actrange", "lengthrange",
    "gear", "damping", "armature", "cranklength", "user",
]
_ACTUATOR_TARGETS = [
    "joint", "jointinparent", "tendon",
    "slidersite", "cranksite", "site", "refsite",
]
_SENSOR_COMMON_ATTRS = [
    "name", "nsample", "interp", "delay", "interval",
    "cutoff", "noise", "user",
]


# ----------------------------------------------------------------------------
# Per-sensor objtype/reftype extraction
# ----------------------------------------------------------------------------
#
# The static MJCF[nMJCF] schema only carries attr lists — not the per-sensor
# mapping between the XML element name and the mjsSensor objtype/reftype the
# compiler expects. That logic lives in xml_native_reader.cc's `Sensor()`
# method (a giant if/else cascade, one branch per sensor type). Phase 2b
# scrapes those branches so codegen rules don't have to hand-list them
# (Phase 2c wires this into the emitter).

# Inside Sensor(), each "regular" branch follows:
#   if (type == "<NAME>") {
#     sensor->type    = mjSENS_X;
#     sensor->objtype = mjOBJ_Y;            // OPTIONAL: literal default
#     ReadAttrTxt(elem, "<attr>", objname, true);   // OPTIONAL: name source
#     sensor->reftype = mjOBJ_Z;            // OPTIONAL
#     ReadAttrTxt(elem, "<attr>", refname, true);   // OPTIONAL: ref name source
#   }
#
# "Computed" branches (rangefinder, distance/normal/fromto, contact, user,
# plugin) derive objtype/reftype from attribute presence at parse time;
# those get marked computed=True so codegen rules keep their explicit
# overrides for them.
# Matches the open of a sensor branch, capturing the parenthesised condition
# string. We post-process the condition to pull out every ``type == "NAME"``
# clause so ``else if (type == "distance" || type == "normal" || type ==
# "fromto") { ... }`` produces three entries that share the branch body.
_SENSOR_BRANCH_RE = re.compile(
    r'(?:if|else if)\s*\((?P<cond>[^{]*?type\s*==\s*"[^"]+"[^{]*)\)\s*\{',
    re.DOTALL,
)
_SENSOR_NAMES_IN_COND_RE = re.compile(r'type\s*==\s*"([^"]+)"')
_TYPE_LITERAL_RE     = re.compile(r'sensor->type\s*=\s*(mjSENS_\w+)\s*;')
_OBJTYPE_LITERAL_RE  = re.compile(r'sensor->objtype\s*=\s*(mjOBJ_\w+)\s*;')
_REFTYPE_LITERAL_RE  = re.compile(r'sensor->reftype\s*=\s*(mjOBJ_\w+)\s*;')
_OBJTYPE_FROM_XML_RE = re.compile(
    r'ReadAttrTxt\s*\(\s*elem\s*,\s*"objtype"\s*,\s*text'
)
_NAME_READ_RE = re.compile(
    r'ReadAttrTxt\s*\(\s*elem\s*,\s*"([^"]+)"\s*,\s*(objname|refname)\b'
)


def _slice_balanced_braces(src: str, open_idx: int) -> str:
    """Given an index pointing AT '{', return the substring up to the
    matching close (exclusive of both braces). Tracks string literals and
    line/block comments so braces inside ``"..."``, ``//`` or ``/* */`` are
    ignored. Returns "" if no matching close found."""
    assert src[open_idx] == "{"
    n = len(src)
    depth = 0
    i = open_idx
    in_str = False
    str_q = ""
    in_line_comment = False
    in_block_comment = False
    while i < n:
        c = src[i]
        c2 = src[i:i+2]
        if in_line_comment:
            if c == "\n":
                in_line_comment = False
            i += 1
            continue
        if in_block_comment:
            if c2 == "*/":
                in_block_comment = False
                i += 2
                continue
            i += 1
            continue
        if in_str:
            if c == "\\":
                i += 2
                continue
            if c == str_q:
                in_str = False
            i += 1
            continue
        if c2 == "//":
            in_line_comment = True
            i += 2
            continue
        if c2 == "/*":
            in_block_comment = True
            i += 2
            continue
        if c in ('"', "'"):
            in_str = True
            str_q = c
            i += 1
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return src[open_idx+1:i]
        i += 1
    return ""


# Branches where objtype/reftype are derived from attribute presence at
# parse time (multiple ReadAttrTxt + sensor->objtype = X ? mjOBJ_A : mjOBJ_B
# pattern). Codegen rules keep explicit overrides for these; the extractor
# marks them computed=True so the generated codegen doesn't pretend to know.
_COMPUTED_OBJTYPE_SENSORS = {
    "rangefinder", "distance", "normal", "fromto", "contact",
    "user", "plugin",
}


def _extract_sensor_per_type(src_text: str) -> Dict[str, Dict[str, object]]:
    """Scrape ``Sensor()``'s per-type if/else cascade. Returns a dict
    keyed by XML element name; each entry has:

        objtype:    "mjOBJ_X" | "from_xml" | "computed" | None
        reftype:    "mjOBJ_Y" | "computed" | None
        obj_attr:   "site"/"joint"/.../None    (XML attr name carrying objname)
        ref_attr:   "camera"/"site"/.../None   (XML attr name carrying refname)
        computed:   True iff objtype/reftype are derived from attr presence
    """
    # Slice out the Sensor() method body so we don't pick up branches from
    # other methods that happen to share regex shapes.
    sig_re = re.compile(r"void\s+mjXReader::Sensor\s*\(")
    m = sig_re.search(src_text)
    if not m:
        return {}
    open_idx = src_text.find("{", m.end())
    if open_idx < 0:
        return {}
    body = _slice_balanced_braces(src_text, open_idx)
    if not body:
        return {}

    result: Dict[str, Dict[str, object]] = {}
    # Some outer branches (distance/normal/fromto, contact, plugin) contain
    # an inner ``if (type == "X")`` that toggles ->type. Those inner matches
    # would otherwise overwrite the outer-branch entry with a stripped-down
    # body. Track the [start, end) of each outer-branch body and skip
    # matches whose '{' falls inside.
    consumed_ranges: List[tuple] = []
    for bm in _SENSOR_BRANCH_RE.finditer(body):
        names = _SENSOR_NAMES_IN_COND_RE.findall(bm.group("cond"))
        if not names:
            continue
        brace_idx = bm.end() - 1
        if brace_idx < 0 or body[brace_idx] != "{":
            continue
        if any(s < brace_idx < e for s, e in consumed_ranges):
            continue
        branch = _slice_balanced_braces(body, brace_idx)
        consumed_ranges.append((brace_idx, brace_idx + 1 + len(branch)))

        obj_literal = _OBJTYPE_LITERAL_RE.search(branch)
        ref_literal = _REFTYPE_LITERAL_RE.search(branch)
        obj_from_xml = bool(_OBJTYPE_FROM_XML_RE.search(branch)) and not obj_literal

        obj_attr: Optional[str] = None
        ref_attr: Optional[str] = None
        for nm in _NAME_READ_RE.finditer(branch):
            attr, target = nm.group(1), nm.group(2)
            if attr == "objtype":
                continue   # the literal "objtype" attr selector, not a name source
            if target == "objname" and obj_attr is None:
                obj_attr = attr
            elif target == "refname" and ref_attr is None:
                ref_attr = attr

        # Capture the mjSENS_X assigned in this branch. Multi-name branches
        # (distance/normal/fromto) carry an inner positional switch with
        # one ->type per name. Single-name branches (touch, framepos, ...)
        # have exactly one mjSENS_X.
        mj_types = [m.group(1) for m in _TYPE_LITERAL_RE.finditer(branch)]
        if len(mj_types) == len(names):
            name_to_mj_type = dict(zip(names, mj_types))
        elif mj_types:
            # Single-type branch — every name shares it.
            name_to_mj_type = {n: mj_types[0] for n in names}
        else:
            name_to_mj_type = {n: None for n in names}

        # Multi-name branches (distance/normal/fromto share one block,
        # routed inside by `if (type == ...)` to set ->type). All share
        # the same objtype/reftype derivation, so emit one entry per name.
        for name in names:
            computed = name in _COMPUTED_OBJTYPE_SENSORS
            result[name] = {
                "mj_type":  name_to_mj_type.get(name),
                "objtype":  ("computed" if computed
                             else ("from_xml" if obj_from_xml
                                   else (obj_literal.group(1) if obj_literal else None))),
                "reftype":  ("computed" if computed
                             else (ref_literal.group(1) if ref_literal else None)),
                "obj_attr": obj_attr,
                "ref_attr": ref_attr,
                "computed": computed,
            }
    return result


def _build_snapshot(root: SchemaNode, mujoco_version: str, *,
                    src_text: str = "") -> Dict:
    """Walk the parsed MJCF tree and project it onto URLab's curated shape."""
    out: Dict = {
        "_meta": {
            "mujoco_version": mujoco_version,
            "snapshot_date": _dt.date.today().isoformat(),
            "source": "https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_reader.cc",
            "description": (
                "MJCF schema snapshot, auto-extracted by "
                "Scripts/codegen/build_mjcf_schema_snapshot.py from the "
                "MuJoCo submodule's xml_native_reader.cc."
            ),
        }
    }

    mujoco = _find_child(root, "mujoco")
    if mujoco is None:
        raise SystemExit("schema source: no <mujoco> root element")

    # There is no explicit ``<worldbody>`` declaration in the MJCF[] schema —
    # ``<body>`` sits directly under ``<mujoco>`` with occurrence "R"
    # (recursive). That body's child list is what the runtime parser uses
    # for both worldbody contents and nested-body contents.
    body = _find_child(mujoco, "body")

    # Top-level worldbody-like elements (attrs + children). The body itself
    # is the worldbody schema; geom/joint/inertial/site/camera/light/etc.
    # live underneath it.
    for name in _WORLDBODY_DIRECT:
        node = body if name == "body" else (_find_child(body, name) if body else None)
        if node is None:
            continue
        entry: Dict = {"attrs": list(node.attrs)}
        if node.children:
            entry["children"] = [c.name for c in node.children]
        out[name] = entry

    # Actuator section. Each subtype's full attr list lives directly under
    # <actuator>. URLab's curated shape splits this into:
    #   actuator_common.attrs (URLab convention, _ACTUATOR_COMMON_ATTRS above)
    #   actuator_common.targets (transmission target attrs, _ACTUATOR_TARGETS)
    #   actuator_types[name] (per-subtype EXTRAS only)
    actuator = _find_child(mujoco, "actuator")
    if actuator is not None:
        type_attrs = {c.name: list(c.attrs) for c in actuator.children
                      if c.name != "plugin"}
        common_set = set(_ACTUATOR_COMMON_ATTRS)
        targets_set = set(_ACTUATOR_TARGETS)
        out["actuator_common"] = {
            "attrs": list(_ACTUATOR_COMMON_ATTRS),
            "targets": list(_ACTUATOR_TARGETS),
        }
        actuator_types: Dict[str, List[str]] = {}
        for name, attrs in type_attrs.items():
            extras = [a for a in attrs
                      if a not in common_set and a not in targets_set]
            actuator_types[name] = extras
        out["actuator_types"] = actuator_types

    # Sensor section: per-type attr declarations. URLab tracks:
    #   sensor_types = list of names
    #   sensor_common = shared attrs (_SENSOR_COMMON_ATTRS, URLab convention)
    #   sensor_per_type = per-sensor objtype/reftype/attr defaults scraped
    #     from xml_native_reader.cc's Sensor() method body
    sensor = _find_child(mujoco, "sensor")
    if sensor is not None:
        sensor_attrs = {c.name: list(c.attrs) for c in sensor.children}
        out["sensor_types"] = list(sensor_attrs.keys())
        out["sensor_common"] = {"attrs": list(_SENSOR_COMMON_ATTRS)}
        if src_text:
            per_type = _extract_sensor_per_type(src_text)
            # Only emit per-type rows for sensors that exist in the static
            # MJCF schema — keeps the snapshot in sync if MuJoCo adds a
            # parser branch but not a schema entry (or vice versa).
            out["sensor_per_type"] = {
                name: per_type[name] for name in sensor_attrs
                if name in per_type
            }

    # Tendon section: spatial vs fixed, plus wrap types.
    tendon = _find_child(mujoco, "tendon")
    if tendon is not None:
        tdict: Dict[str, Dict] = {}
        for sub in tendon.children:
            entry = {"attrs": list(sub.attrs)}
            if sub.children:
                entry["wrap_types"] = [c.name for c in sub.children]
            tdict[sub.name] = entry
        out["tendon"] = tdict

    # Equality section.
    equality = _find_child(mujoco, "equality")
    if equality is not None:
        out["equality"] = {c.name: list(c.attrs) for c in equality.children}

    # Contact section (pair + exclude).
    contact = _find_child(mujoco, "contact")
    if contact is not None:
        out["contact"] = {c.name: list(c.attrs) for c in contact.children}

    # Asset section (mesh / hfield / texture / material / etc.).
    asset = _find_child(mujoco, "asset")
    if asset is not None:
        out["asset"] = {c.name: list(c.attrs) for c in asset.children}

    # Compiler section.
    compiler = _find_child(mujoco, "compiler")
    if compiler is not None:
        out["compiler"] = {"attrs": list(compiler.attrs)}

    # Option section (top-level attrs + <flag> sub-element).
    option = _find_child(mujoco, "option")
    if option is not None:
        opt_entry: Dict[str, List[str]] = {"attrs": list(option.attrs)}
        flag = _find_child(option, "flag")
        if flag is not None:
            opt_entry["flag"] = list(flag.attrs)
        out["option"] = opt_entry

    # Keyframe section: <keyframe> -> <key>.
    keyframe = _find_child(mujoco, "keyframe")
    if keyframe is not None:
        key = _find_child(keyframe, "key")
        if key is not None:
            out["keyframe"] = {"key": list(key.attrs)}

    # Default section: just the child element names the codegen needs to
    # know about so it can wire UMjDefault sub-component creation.
    default = _find_child(mujoco, "default")
    if default is not None:
        # Filter out nested defaults — only direct element children.
        kids = [c.name for c in default.children
                if c.name not in ("default", "config")]
        out["default"] = {"child_elements": kids}

    # Deformable/flexcomp/flex — capture top-level attrs for downstream rules.
    deformable = _find_child(mujoco, "deformable")
    if deformable is not None:
        for c in deformable.children:
            out[c.name] = {"attrs": list(c.attrs)}
            if c.children:
                out[c.name]["children"] = [cc.name for cc in c.children]
    # flexcomp lives under body, not <deformable>.
    flexcomp = _find_child(body, "flexcomp") if body else None
    if flexcomp is not None:
        out["flexcomp"] = {"attrs": list(flexcomp.attrs)}
        if flexcomp.children:
            out["flexcomp"]["children"] = [c.name for c in flexcomp.children]

    return out


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def _detect_mujoco_version(src_path: str) -> str:
    """Read the version from include/mujoco/mujoco.h next to xml_native_reader.cc."""
    # src_path is .../mujoco/src/xml/xml_native_reader.cc
    repo_root = os.path.normpath(os.path.join(os.path.dirname(src_path), "..", ".."))
    header = os.path.join(repo_root, "include", "mujoco", "mujoco.h")
    if not os.path.exists(header):
        return "unknown"
    with open(header, "r", encoding="utf-8") as f:
        head = f.read(4096)
    # MuJoCo encodes the version as MMmmpp (e.g. 3008001 = 3.8.1) — 7 digits
    # for 3.10+, 6 digits before. Parse the last 3 digit pairs as patch/minor/major.
    m = re.search(r"#define\s+mjVERSION_HEADER\s+(\d+)", head)
    if m:
        v = int(m.group(1))
        patch = v % 1000
        minor = (v // 1000) % 1000
        major = v // 1000000
        return f"{major}.{minor}.{patch}"
    return "unknown"


def main(argv: Optional[List[str]] = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    plugin_root = os.path.normpath(os.path.join(here, "..", ".."))
    default_src = os.path.join(
        plugin_root, "third_party", "MuJoCo", "src", "src", "xml",
        "xml_native_reader.cc",
    )
    default_output = os.path.join(plugin_root, "Scripts", "codegen", "snapshots", "mjcf_schema_snapshot.json")

    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=default_src,
                    help=f"MuJoCo xml_native_reader.cc (default: {default_src})")
    ap.add_argument("--output", default=default_output,
                    help=f"Output JSON path (default: {default_output})")
    args = ap.parse_args(argv)

    if not os.path.exists(args.src):
        print(f"error: schema source not found: {args.src}", file=sys.stderr)
        return 2

    with open(args.src, "r", encoding="utf-8") as f:
        src_text = f.read()
    body = _extract_mjcf_block(src_text)
    blocks = _tokenise_blocks(body)
    root = _parse_tree(blocks)
    version = _detect_mujoco_version(args.src)
    snapshot = _build_snapshot(root, mujoco_version=version, src_text=src_text)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(snapshot, f, indent=2)
        f.write("\n")
    print(f"wrote {args.output}: mujoco_version={version}, top-level keys={len(snapshot) - 1}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
