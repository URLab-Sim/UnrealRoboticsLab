# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""URLab codegen — drift checks (Phase 5 module split, step 2).

Every ``_check_*`` function that scans rules + snapshots for drift
lives here. The orchestrator (``generate_ue_components``) imports the
dispatcher functions (``_emit_drift_diagnostics``, ``_check_*``) and
wires them into ``_phase_diagnostics``.

Dependencies are strictly downward: this module imports from
``_codegen_core`` only. No emit logic, no .cpp injection — drift checks
read the rules + snapshots and route diagnostics through
``_codegen_core._diag_add``.
"""

from __future__ import annotations

import os
import re
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from _codegen_core import (
    _diag_add,
    _UE_TYPE_FAMILY, _ue_type_accepts_units_meta, _KNOWN_UE_UNITS,
    _classify_c_type, _shapes_compatible, _VEC3_CONVERT_FMT,
    schema_attrs, schema_subtype_attrs,
    compute_canon_absorbed,
    _resolve_mjs_field, _resolve_value_map,
)


# ---------------------------------------------------------------------------
# Hand-enum scrape (regex fallback when the libclang scrape didn't run)
# ---------------------------------------------------------------------------

_HAND_ENUM_RE = re.compile(
    r"enum\s+class\s+(EMj\w+)\s*:\s*uint8\s*\{([^}]*)\}",
    re.DOTALL,
)
_HAND_ENUM_MEMBER_RE = re.compile(r"\A\s*(\w+)")


def _scan_hand_enums(public_root: str) -> Dict[str, List[str]]:
    """Walk ``public_root`` for every ``.h`` file declaring
    ``enum class EMj* : uint8 { ... }`` and return
    ``{enum_name: [members]}``. Members are stripped of inline
    initialisers, UMETA macros, and comments."""
    out: Dict[str, List[str]] = {}
    for root, _dirs, files in os.walk(public_root):
        for name in files:
            if not name.endswith(".h"):
                continue
            with open(os.path.join(root, name), "r", encoding="utf-8") as f:
                text = f.read()
            for m in _HAND_ENUM_RE.finditer(text):
                enum_name = m.group(1)
                body = m.group(2)
                body = re.sub(r"UMETA\([^)]*\)", "", body)
                body = re.sub(r"//[^\n]*", "", body)
                body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)
                members: List[str] = []
                for line in body.split(","):
                    mm = _HAND_ENUM_MEMBER_RE.match(line)
                    if mm:
                        members.append(mm.group(1))
                if members:
                    out[enum_name] = members
    return out


def _hand_enums_from_snapshot(mjspec: Optional[Dict[str, Any]]) -> Dict[str, List[str]]:
    """Pull URLab's hand-rolled ``EMj*`` enums out of the merged
    introspect snapshot when present. Snapshot path produces real
    ENUM_CONSTANT_DECL members from libclang. Regex fallback covers
    the local-dev case where the snapshot wasn't regenerated."""
    if not mjspec:
        return {}
    introspect = mjspec.get("introspect", {}) or {}
    hand_enums = introspect.get("hand_enums") or {}
    out: Dict[str, List[str]] = {}
    for name, entry in hand_enums.items():
        if not isinstance(entry, dict):
            continue
        members = entry.get("members", {})
        if isinstance(members, dict):
            out[name] = list(members.keys())
    return out


# ---------------------------------------------------------------------------
# Drift checks
# ---------------------------------------------------------------------------


def _check_hand_enum_drift(
    schema: Dict[str, Any],
    rules: Dict[str, Any],
    mjspec: Optional[Dict[str, Any]],
    public_root: str,
) -> None:
    """Cross-check rule-declared UE enum members against the actual
    hand enums AND against MuJoCo's mjt* enums."""
    hand_enums = _hand_enums_from_snapshot(mjspec)
    if not hand_enums:
        hand_enums = _scan_hand_enums(public_root)
    if not hand_enums:
        return

    for cat_name, cat_rules in rules.get("categories", {}).items():
        type_enum = cat_rules.get("type_enum_name")
        if not type_enum or type_enum not in hand_enums:
            continue
        members = set(hand_enums[type_enum])
        for sub in cat_rules.get("subtypes", []):
            ev = sub.get("enum_value")
            if ev and ev not in members:
                _diag_add(
                    f"[diagnostic] category '{cat_name}' subtype '{sub.get('key')}' "
                    f"references enum_value '{type_enum}::{ev}' that does not "
                    f"exist in {type_enum} members {sorted(members)}.",
                    source="hand_enum_drift",
                )

    element_rules = rules.get("element_rules", {})
    mjspec_enums = (mjspec or {}).get("enums", {})
    intentional_skips = rules.get("intentionally_unmapped_mj_enum_values", {})
    for elem, elem_rule in element_rules.items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            ue_enum_type = enum_def.get("ue_enum_type") or enum_def.get("ue_enum")
            value_map = enum_def.get("value_map", {})
            if not ue_enum_type or ue_enum_type not in hand_enums:
                continue
            members = set(hand_enums[ue_enum_type])
            for xml_val, mapping in value_map.items():
                if not isinstance(mapping, (list, tuple)) or not mapping:
                    continue
                ue_member = mapping[0]
                if ue_member not in members:
                    _diag_add(
                        f"[diagnostic] element '{elem}' xml_enum '{attr}' "
                        f"value_map entry '{xml_val}' maps to "
                        f"'{ue_enum_type}::{ue_member}' that does not exist "
                        f"in members {sorted(members)}.",
                        source="hand_enum_drift",
                    )
            from_mj_enum = enum_def.get("from_mj_enum")
            if not from_mj_enum:
                mjs_cast = enum_def.get("mjs_cast", "")
                if mjs_cast.startswith("mjt"):
                    from_mj_enum = mjs_cast
            if from_mj_enum and from_mj_enum in mjspec_enums:
                mj_values = mjspec_enums[from_mj_enum]
                covered_mj: set = set()
                for mapping in value_map.values():
                    if isinstance(mapping, (list, tuple)) and len(mapping) >= 2:
                        covered_mj.add(mapping[1])
                covered_mj.update(enum_def.get("xml_from_mj", {}).keys())
                skips = set(intentional_skips.get(from_mj_enum, []))
                per_elem = intentional_skips.get("__per_element__", {})
                skips |= set(per_elem.get(f"{elem}.{attr}", []))
                for mj_const in mj_values:
                    if mj_const in covered_mj or mj_const in skips:
                        continue
                    _diag_add(
                        f"[diagnostic] {from_mj_enum}::{mj_const} (from "
                        f"mjspec snapshot) has no mapping in element "
                        f"'{elem}' xml_enum_attrs.{attr}.value_map; new "
                        f"MuJoCo enum values silently fall through to the "
                        f"default branch at runtime. Add a value_map entry "
                        f"or list under 'intentionally_unmapped_mj_enum_values"
                        f".{from_mj_enum}'.",
                        source="hand_enum_drift",
                    )


def _check_type_shape_drift(
    schema: Dict[str, Any],
    rules: Dict[str, Any],
    mjspec: Optional[Dict[str, Any]],
) -> None:
    """Cross-check ``type_mappings[attr]`` against the actual C-type
    shape of the corresponding mjs struct field."""
    if not mjspec:
        return
    struct_table = (mjspec.get("introspect") or {}).get("structs", {})
    if not struct_table:
        return
    type_map = rules.get("type_mappings", {})
    unmodeled_mjs = rules.get("intentionally_unmodeled_mjs_fields", {})
    element_rules = rules.get("element_rules", {})
    for cat_name, cat_rules in rules.get("categories", {}).items():
        mjs_struct = cat_rules.get("mjs_struct")
        if not mjs_struct or mjs_struct not in struct_table:
            continue
        fields = {
            f["name"]: (f.get("c_type", ""), f.get("array_dim"))
            for f in struct_table[mjs_struct].get("fields", [])
        }
        skip_fields = set(unmodeled_mjs.get(mjs_struct, []))
        skip_fields |= set(element_rules.get(cat_name, {}).get("exclude_attrs", []))
        for attr, ue_type in type_map.items():
            if attr not in fields or attr in skip_fields:
                continue
            ue_shape = _UE_TYPE_FAMILY.get(ue_type)
            if not ue_shape:
                continue
            c_type, array_dim = fields[attr]
            c_shape = _classify_c_type(c_type, array_dim)
            if c_shape is None or c_shape[0] == "opaque":
                continue
            if not _shapes_compatible(ue_shape, c_shape):
                _diag_add(
                    f"[diagnostic] type_mappings['{attr}'] = '{ue_type}' "
                    f"(category '{cat_name}') but {mjs_struct}.{attr} is "
                    f"'{c_type}' (shape={c_shape}); rule expects "
                    f"shape={ue_shape}. UE/C shape mismatch — emitted "
                    f"code may overflow, truncate, or fail to compile.",
                    source="type_shape_drift",
                )


def _check_new_attr_typing(
    schema: Dict[str, Any],
    rules: Dict[str, Any],
) -> None:
    """Every schema attr that lands as a UPROPERTY but has no explicit
    type_mappings entry silently inherits ``default_type`` ('float')."""
    type_map = rules.get("type_mappings", {})
    default_type = rules.get("default_type", "float")
    global_excl = set(rules.get("global_exclusions", []))
    intentional_float = set(rules.get("intentionally_default_typed_attrs", []))
    element_rules = rules.get("element_rules", {})
    canonicalizations = rules.get("canonicalizations", {})

    def _collect_attrs(cat_rules: Dict[str, Any]) -> set:
        out: set = set(schema_attrs(schema, cat_rules.get("schema_common_block", "")))
        out |= set(cat_rules.get("schema_common_extra_attrs", []))
        sub_block = cat_rules.get("schema_subtypes_block")
        if sub_block and sub_block in schema:
            block = schema[sub_block]
            if isinstance(block, dict):
                for sub_attrs in block.values():
                    if isinstance(sub_attrs, list):
                        out |= set(sub_attrs)
                    elif isinstance(sub_attrs, dict):
                        out |= set(sub_attrs.get("attrs", []))
            elif isinstance(block, list):
                for entry in block:
                    if isinstance(entry, dict):
                        out |= set(entry.get("attrs", []))
        return out

    flagged: set = set()
    for cat_name, cat_rules in rules.get("categories", {}).items():
        elem_rule = element_rules.get(cat_name, {})
        elem_excl = set(elem_rule.get("exclude_attrs", []))
        xml_enum_attrs = set(elem_rule.get("xml_enum_attrs", {}).keys())
        renames = set(elem_rule.get("property_renames", {}).keys())
        collations = set(elem_rule.get("target_collations", {}).keys())
        export_skip = set(elem_rule.get("export_skip_attrs", []))
        common_imports = set()
        for entry in elem_rule.get("common_imports", []):
            if isinstance(entry, str):
                common_imports.add(entry)
            elif isinstance(entry, dict):
                common_imports.add(entry.get("attr"))
        canon_absorbed: set = compute_canon_absorbed(
            elem_rule.get("applies_canonicalizations", []),
            canonicalizations,
        )
        for attr in _collect_attrs(cat_rules):
            if attr in global_excl or attr in elem_excl:
                continue
            if attr in xml_enum_attrs or attr in renames or attr in collations:
                continue
            if attr in canon_absorbed or attr in export_skip or attr in common_imports:
                continue
            if attr in type_map:
                continue
            if attr in flagged or attr in intentional_float:
                continue
            flagged.add(attr)
            _diag_add(
                f"[diagnostic] attr '{attr}' (used by {cat_name}) has no "
                f"type_mappings entry and would fall back to default_type "
                f"('{default_type}'). Pin its UE type in type_mappings, or "
                f"list it under 'intentionally_default_typed_attrs' to "
                f"acknowledge the fallback.",
                source="new_attr_typing",
            )


_ATTR_NAME_RE = re.compile(r"^[a-z_][a-z0-9_]*$")


def _all_schema_attrs(schema: Dict[str, Any], rules: Dict[str, Any]) -> set:
    """Union of every schema attr observed anywhere in the snapshot."""
    out: set = set()

    def _walk(node: Any) -> None:
        if isinstance(node, list):
            for item in node:
                if isinstance(item, str) and _ATTR_NAME_RE.match(item):
                    out.add(item)
                else:
                    _walk(item)
        elif isinstance(node, dict):
            for k, v in node.items():
                if k in ("obj_attr", "ref_attr") and isinstance(v, str):
                    if _ATTR_NAME_RE.match(v):
                        out.add(v)
                _walk(v)

    _walk(schema)
    for cat_rules in rules.get("categories", {}).values():
        out |= set(cat_rules.get("schema_common_extra_attrs", []))
    return out


def _check_orphan_rule_entries(
    schema: Dict[str, Any],
    rules: Dict[str, Any],
) -> None:
    """Diagnose rule entries that reference attrs no schema element
    uses. Catches stale rules left over from MuJoCo renaming/removing
    an attr."""
    known_attrs = _all_schema_attrs(schema, rules)
    if not known_attrs:
        return
    type_map = rules.get("type_mappings", {})
    for attr in type_map.keys():
        if attr not in known_attrs:
            _diag_add(
                f"[diagnostic] type_mappings['{attr}'] is set but no "
                f"schema element exposes an attr by that name. Dead rule "
                f"entry — remove, or check whether MuJoCo renamed/removed "
                f"the attr.",
                source="orphan_rule",
            )
    PER_ELEM_KEYS = (
        "exclude_attrs",
        "vec3_convert",
        "property_renames",
        "property_meta",
        "property_defaults",
        "export_skip_attrs",
        "mjs_data_packed_attrs",
    )
    for elem, elem_rule in rules.get("element_rules", {}).items():
        for key in PER_ELEM_KEYS:
            block = elem_rule.get(key, {})
            if isinstance(block, dict):
                rule_attrs = list(block.keys())
            elif isinstance(block, list):
                rule_attrs = [
                    a if isinstance(a, str) else a.get("attr")
                    for a in block
                ]
            else:
                continue
            for attr in rule_attrs:
                if not attr or attr in known_attrs:
                    continue
                _diag_add(
                    f"[diagnostic] element_rules['{elem}']['{key}'] "
                    f"references attr '{attr}', which is not present in "
                    f"ANY schema element. Stale rule.",
                    source="orphan_rule",
                )


def _check_mjxmacro_block_coverage(
    rules: Dict[str, Any],
    mjxmacro: Dict[str, Any],
) -> None:
    """Bi-directional check between mjxmacro snapshot blocks and
    ``synthetic_categories`` entries. Forward catches new blocks
    upstream URLab doesn't model; reverse catches rule entries
    pointing at vanished blocks."""
    blocks = mjxmacro.get("struct_fields", {})
    synthetic = rules.get("synthetic_categories", {})
    if not blocks and not synthetic:
        return
    block_to_rule: Dict[str, str] = {}
    for key, entry in synthetic.items():
        if not isinstance(entry, dict):
            continue
        mb = entry.get("mjxmacro_block")
        if mb:
            block_to_rule[mb] = key
    for block in blocks:
        if block not in block_to_rule:
            _diag_add(
                f"[diagnostic] mjxmacro struct_fields block '{block}' has "
                f"no synthetic_categories rule entry; the {len(blocks[block])} "
                f"field(s) it declares will not be emitted as a UE struct.",
                source="mjxmacro_block_coverage",
            )
    for block, rule_key in block_to_rule.items():
        if block not in blocks:
            _diag_add(
                f"[diagnostic] synthetic_categories '{rule_key}' references "
                f"mjxmacro_block '{block}' that does not exist in the current "
                f"mjxmacro snapshot — block renamed or removed upstream.",
                source="mjxmacro_block_coverage",
            )


# Apply-mode values _emit_synth_apply_methods_cpp understands. Anything
# else lands as a silent passthrough to vec3_y_flip / unconditional —
# the diagnostic surfaces typos so a rule author's intent is real.
_VALID_APPLY_MODES = (
    {"unconditional", "guarded", "pos"}
    | set(_VEC3_CONVERT_FMT.keys())
    | {f"guarded_{k}" for k in _VEC3_CONVERT_FMT.keys()}
    | {"guarded_pos"}
)


def _check_apply_mode_validity(rules: Dict[str, Any]) -> None:
    """Diagnose ``field_apply_to_spec_mode`` values the codegen would
    silently fall back on."""
    synth = rules.get("synthetic_categories", {})
    for cat_name, def_ in synth.items():
        if cat_name.startswith("_") or not isinstance(def_, dict):
            continue
        modes = def_.get("field_apply_to_spec_mode", {})
        if not isinstance(modes, dict):
            continue
        for field, mode in modes.items():
            if mode not in _VALID_APPLY_MODES:
                _diag_add(
                    f"[diagnostic] synthetic_categories['{cat_name}']"
                    f".field_apply_to_spec_mode['{field}'] = '{mode}' "
                    f"is not a recognised apply mode. Expected one of "
                    f"{sorted(_VALID_APPLY_MODES)}.",
                    source="apply_mode",
                )


def _check_embedded_cpp_references(
    rules: Dict[str, Any], mjspec: Optional[Dict[str, Any]],
) -> None:
    """Diagnose rule entries that name a MuJoCo C symbol the mjspec
    snapshot doesn't carry."""
    if not mjspec or "enums" not in mjspec:
        return
    known_consts: Set[str] = set()
    for members in mjspec["enums"].values():
        if isinstance(members, dict):
            known_consts.update(members.keys())
        elif isinstance(members, list):
            known_consts.update(members)
    if not known_consts:
        return
    for elem, elem_rule in rules.get("element_rules", {}).items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            for xml_val, mapping in enum_def.get("value_map", {}).items():
                if not isinstance(mapping, (list, tuple)) or len(mapping) < 2:
                    continue
                mj_const = mapping[1]
                if not isinstance(mj_const, str):
                    continue
                if not mj_const.startswith("mj"):
                    continue
                if mj_const not in known_consts:
                    _diag_add(
                        f"[diagnostic] element_rules['{elem}']"
                        f".xml_enum_attrs['{attr}'].value_map['{xml_val}'][1]"
                        f" = '{mj_const}' is not present in any mjspec enum. "
                        f"Likely typo — UE compile would fail with "
                        f"'undeclared identifier'.",
                        source="embedded_cpp_refs",
                    )
    for cat_name, cat_rules in rules.get("categories", {}).items():
        final = cat_rules.get("geom_final_type")
        if not final:
            continue
        fallback = final.get("default_fallback")
        if isinstance(fallback, str) and fallback.startswith("mj") and fallback not in known_consts:
            _diag_add(
                f"[diagnostic] categories['{cat_name}'].geom_final_type"
                f".default_fallback = '{fallback}' is not present in any "
                f"mjspec enum.",
                source="embedded_cpp_refs",
            )


def _collect_every_schema_attr(schema: Dict[str, Any]) -> Set[str]:
    """Walk the entire schema tree returning every attr name found.
    Used by ``_check_allowlist_staleness``."""
    out: Set[str] = set()

    def walk(node: Any) -> None:
        if isinstance(node, dict):
            for v in node.values():
                if isinstance(v, list) and all(isinstance(s, str) for s in v):
                    out.update(v)
                else:
                    walk(v)
        elif isinstance(node, list):
            for entry in node:
                walk(entry)

    walk(schema)
    return out


def _check_allowlist_staleness(
    schema: Dict[str, Any], rules: Dict[str, Any],
    mjspec: Optional[Dict[str, Any]],
) -> None:
    """Surface allowlist entries the codebase no longer needs. Narrows
    to 4 of 6 allowlists per the locked plan."""
    unmodeled_elems: Dict[str, str] = rules.get("intentionally_unmodeled_elements", {})
    schema_keys = set(schema.keys())
    for key in unmodeled_elems:
        if key not in schema_keys:
            _diag_add(
                f"[diagnostic] intentionally_unmodeled_elements lists '{key}' "
                f"but the schema no longer carries that top-level element. "
                f"Drop the entry.",
                source="allowlist_staleness",
            )

    unmodeled_fields: Dict[str, List[str]] = rules.get("intentionally_unmodeled_mjs_fields", {})
    structs = (mjspec or {}).get("structs", {}) if mjspec else {}
    if structs:
        for struct, fields in unmodeled_fields.items():
            if struct not in structs:
                _diag_add(
                    f"[diagnostic] intentionally_unmodeled_mjs_fields names "
                    f"struct '{struct}' that is missing from the current "
                    f"mjspec snapshot. Drop the entry.",
                    source="allowlist_staleness",
                )
                continue
            struct_field_set = set(structs[struct])
            for field in fields or []:
                if field not in struct_field_set:
                    _diag_add(
                        f"[diagnostic] intentionally_unmodeled_mjs_fields"
                        f"['{struct}'] lists '{field}' but the field no "
                        f"longer exists on the mjs struct. Drop the entry.",
                        source="allowlist_staleness",
                    )

    default_typed: List[str] = rules.get("intentionally_default_typed_attrs", [])
    every_attr: Set[str] = _collect_every_schema_attr(schema)
    for attr in default_typed:
        if attr not in every_attr:
            _diag_add(
                f"[diagnostic] intentionally_default_typed_attrs lists "
                f"'{attr}' but no schema element carries that attr. "
                f"Drop the entry.",
                source="allowlist_staleness",
            )

    unmapped: Dict[str, Any] = rules.get("intentionally_unmapped_mj_enum_values", {})
    mj_enums = (mjspec or {}).get("enums", {}) if mjspec else {}
    if mj_enums:
        for key in unmapped:
            if key.startswith("_") or key == "__per_element__":
                continue
            if key not in mj_enums:
                _diag_add(
                    f"[diagnostic] intentionally_unmapped_mj_enum_values "
                    f"names mjt enum '{key}' that is missing from the "
                    f"current mjspec snapshot. Drop the entry.",
                    source="allowlist_staleness",
                )


def _check_property_units_validity(
    rules: Dict[str, Any],
) -> None:
    """Diagnose property_units entries that use an unrecognised UE
    display unit, or target a UE type that UHT rejects Units meta on."""
    for elem, elem_rule in rules.get("element_rules", {}).items():
        units = elem_rule.get("property_units", {})
        if not isinstance(units, dict):
            continue
        for attr, unit in units.items():
            if unit not in _KNOWN_UE_UNITS:
                _diag_add(
                    f"[diagnostic] element_rules['{elem}'].property_units"
                    f"['{attr}'] = '{unit}' is not a UE-recognised display "
                    f"unit. Use one of {sorted(_KNOWN_UE_UNITS)} or extend "
                    f"_KNOWN_UE_UNITS in _codegen_core.",
                    source="property_units",
                )


def _check_compiler_attrs_coverage(
    schema: Dict[str, Any],
    rules: Dict[str, Any],
) -> None:
    """Diagnose any <compiler> attribute the MJCF schema lists that
    URLab neither models in FMjCompilerSettings nor explicitly skips."""
    compiler_attrs = schema.get("compiler", {}).get("attrs", [])
    if not compiler_attrs:
        return
    mapped = set(rules.get("compiler_attr_field_map", {}).keys())
    skipped = set(rules.get("intentionally_unmodeled_compiler_attrs", []))
    for attr in compiler_attrs:
        if attr in mapped or attr in skipped:
            continue
        _diag_add(
            f"[diagnostic] <compiler {attr}='...'> is in the MJCF schema "
            f"but URLab does not handle it. Either add a field to "
            f"FMjCompilerSettings + an entry in 'compiler_attr_field_map', "
            f"or add to 'intentionally_unmodeled_compiler_attrs' if the "
            f"flag has no UE-side effect.",
            source="compiler_coverage",
        )


def _check_value_map_stale_mj_consts(
    rules: Dict[str, Any],
    mjspec: Optional[Dict[str, Any]],
) -> None:
    """Every value_map[*][1] is a literal mj* C constant emitted into
    the generated .cpp; the snapshot cross-check surfaces removed/
    renamed constants before they break the C++ compile."""
    if not mjspec:
        return
    enums = mjspec.get("enums", {})
    if not enums:
        return
    known: set = set()
    for members in enums.values():
        if isinstance(members, list):
            known.update(members)
        elif isinstance(members, dict):
            known.update(members.keys())
    if not known:
        return
    for elem, elem_rule in rules.get("element_rules", {}).items():
        for attr, enum_def in elem_rule.get("xml_enum_attrs", {}).items():
            value_map = enum_def.get("value_map", {})
            for xml_val, mapping in value_map.items():
                if not isinstance(mapping, (list, tuple)) or len(mapping) < 2:
                    continue
                mj_const = mapping[1]
                if not isinstance(mj_const, str) or not mj_const.startswith("mj"):
                    continue
                if mj_const not in known:
                    _diag_add(
                        f"[diagnostic] element '{elem}' xml_enum '{attr}' "
                        f"value_map entry '{xml_val}' references "
                        f"'{mj_const}', which is NOT in the mjspec "
                        f"snapshot's enum members. The generated .cpp "
                        f"will fail to compile when the MuJoCo headers "
                        f"update.",
                        source="value_map_stale",
                    )


def _check_top_level_elements_coverage(
    schema: Dict[str, Any], rules: Dict[str, Any],
) -> None:
    """Flag top-level schema elements with no category in rules."""
    categories = rules.get("categories", {})
    known_blocks = {c.get("schema_common_block", "").split(".", 1)[0]
                    for c in categories.values()}
    known_blocks.discard("")
    container_keys = {
        "_meta", "actuator_common", "actuator_types", "sensor_common",
        "sensor_types", "sensor_per_type", "sensor_extras",
        "tendon", "equality", "contact",
        "asset", "compiler", "option", "keyframe", "default",
    }
    unmodeled: Dict[str, str] = rules.get("intentionally_unmodeled_elements", {})
    for key in schema:
        if key.startswith("_") or key in container_keys or key in known_blocks:
            continue
        if key in unmodeled:
            continue
        _diag_add(
            f"[diagnostic] schema has top-level element '{key}' but no "
            f"category in codegen_rules.json (rules['categories']). "
            f"Either add a category entry, list '{key}' in "
            f"container_keys, or add an "
            f"'intentionally_unmodeled_elements' entry explaining why "
            f"it's skipped."
        )


def _check_actuator_subtypes_coverage(
    schema: Dict[str, Any], rules: Dict[str, Any],
) -> None:
    act_cat = rules.get("categories", {}).get("actuator", {})
    known_act_subs = {s.get("key") for s in act_cat.get("subtypes", [])}
    for name in schema.get("actuator_types", {}):
        if name not in known_act_subs:
            _diag_add(
                f"[diagnostic] schema actuator subtype '{name}' has no "
                f"entry in categories.actuator.subtypes. Add a subtype "
                f"+ subtype_setto rule pair so codegen emits the UMj"
                f"{name.title()}Actuator class."
            )


def _check_sensor_subtypes_coverage(
    schema: Dict[str, Any], rules: Dict[str, Any],
) -> None:
    sens_cat = rules.get("categories", {}).get("sensor", {})
    known_sens_subs = {s.get("key") for s in sens_cat.get("subtypes", [])}
    for name in schema.get("sensor_types", []):
        if name not in known_sens_subs:
            _diag_add(
                f"[diagnostic] schema sensor type '{name}' has no entry "
                f"in categories.sensor.subtypes. Add it so the per-sensor "
                f"UMjXSensor subclass is emitted."
            )


def _check_setto_param_coverage(
    schema: Dict[str, Any], rules: Dict[str, Any],
    mjspec: Optional[Dict[str, Any]],
) -> None:
    """Flag mjs_setTo* parameters with no UE-side mapping AND no
    explicit sentinel default."""
    if not mjspec or "setto_functions" not in mjspec:
        return
    categories = rules.get("categories", {})
    setto_defaults = rules.get("setto_param_defaults", {})
    for cat_name, cat_rules in categories.items():
        setto_block = cat_rules.get("subtype_setto") or {}
        common_attrs = set(schema_attrs(
            schema, cat_rules.get("schema_common_block", "")))
        for sub_key, setto_def in setto_block.items():
            if not isinstance(setto_def, dict):
                continue
            fn_name = setto_def.get("call")
            if not fn_name or fn_name not in mjspec["setto_functions"]:
                continue
            sig = mjspec["setto_functions"][fn_name]
            renames = setto_def.get("param_renames", {})
            fn_defaults = set(setto_defaults.get(fn_name, {}).keys())
            sub_attrs = set(schema_subtype_attrs(schema, cat_name, sub_key))
            schema_attr_set = common_attrs | sub_attrs
            for p in sig.get("params", []):
                pname = p["name"]
                ue_prop = renames.get(pname, pname)
                if ue_prop in schema_attr_set or pname in fn_defaults:
                    continue
                _diag_add(
                    f"[diagnostic] {fn_name} param '{pname}' is not "
                    f"in the {cat_name}.{sub_key} schema attrs and "
                    f"has no param_renames or setto_param_defaults "
                    f"entry. Codegen passes the generic sentinel for "
                    f"it. If MuJoCo added this param, decide whether "
                    f"to expose it as a UE UPROPERTY (schema attr + "
                    f"type_mapping) or pin its sentinel."
                )


# Canonicalisation name -> set of mjs struct fields its export block
# writes. The mjs-field-drift check uses this to recognise fields that
# canon already covers as NOT drift.
_CANON_MJS_FIELDS = {
    "body_sleep_policy":     {"sleep"},
    "actuator_transmission": {"target", "trntype", "slidersite", "refsite"},
    "fromto_decompose":      set(),
    "orientation":           {"quat"},
    "spatial_pose":          {"pos"},
}


def _check_mjs_struct_field_drift(
    schema: Dict[str, Any], rules: Dict[str, Any],
    mjspec: Optional[Dict[str, Any]],
) -> None:
    """Flag mjs struct fields with no schema-attr coverage or rule
    binding."""
    if not mjspec or "structs" not in mjspec:
        return
    categories = rules.get("categories", {})
    unmodeled_by_struct: Dict[str, set] = {
        struct: set(fields) for struct, fields in
        rules.get("intentionally_unmodeled_mjs_fields", {}).items()
    }
    for cat_name, cat_rules in categories.items():
        mjs_struct = cat_rules.get("mjs_struct")
        if not mjs_struct or mjs_struct not in mjspec["structs"]:
            continue
        block = cat_rules.get("schema_common_block", "")
        schema_attrs_set = set(schema_attrs(schema, block))
        for sub in cat_rules.get("subtypes", []):
            schema_attrs_set.update(
                schema_subtype_attrs(schema, cat_name, sub.get("key", ""))
            )
        mjs_field_set = set(mjspec["structs"][mjs_struct])
        elem_rules = rules.get("element_rules", {}).get(cat_name, {})
        attr_to_mjs = elem_rules.get("attr_to_mjs_field", {})
        mapped_fields = set(attr_to_mjs.values())
        for coll_def in elem_rules.get("target_collations", {}).values():
            if isinstance(coll_def, dict) and coll_def.get("mjs_field"):
                mapped_fields.add(coll_def["mjs_field"])
        for enum_def in elem_rules.get("xml_enum_attrs", {}).values():
            if isinstance(enum_def, dict) and enum_def.get("mjs_field"):
                mapped_fields.add(enum_def["mjs_field"])
        for packed in elem_rules.get("mjs_data_packed_attrs", {}).values():
            if isinstance(packed, dict):
                mapped_fields.add(packed.get("target") or "data")
        for da in elem_rules.get("mjs_double_vec_attrs", []):
            mapped_fields.add(da)
        if cat_rules.get("type_enum_name"):
            mapped_fields.add("type")
        mapped_fields.add("objtype")
        for conv_list in elem_rules.get("unit_conversion", {}).values():
            for entry in conv_list if isinstance(conv_list, list) else []:
                if isinstance(entry, dict) and entry.get("mjs_field"):
                    mapped_fields.add(entry["mjs_field"])
        for canon in elem_rules.get("applies_canonicalizations", []):
            mapped_fields |= _CANON_MJS_FIELDS.get(canon, set())
        structural = {
            "element", "info", "userdata", "user", "name", "classname",
            "parent", "frame", "explicitinertial",
        }
        structural |= unmodeled_by_struct.get(mjs_struct, set())
        structural |= mapped_fields
        normalised_schema = {a.replace("_", "").lower()
                             for a in schema_attrs_set}
        for mjs_field in mjs_field_set:
            if mjs_field in structural:
                continue
            candidate = mjs_field.replace("_", "").lower()
            if candidate in normalised_schema:
                continue
            stripped = candidate.replace("name", "")
            if stripped in normalised_schema:
                continue
            if candidate.startswith("act") and ("actuator" + candidate[3:]) in normalised_schema:
                continue
            _diag_add(
                f"[diagnostic] {mjs_struct}.{mjs_field} (in mjspec_snapshot) "
                f"has no corresponding schema attr in '{cat_name}'. Either "
                f"add a schema canonicalisation, list it under "
                f"intentionally_unmodeled_mjs_fields, or accept that codegen "
                f"silently skips it.",
                source="mjs_field_drift",
            )


def _check_generated_enum_coverage(
    rules: Dict[str, Any], mjspec: Optional[Dict[str, Any]],
) -> None:
    """Flag generated_enums entries that map fewer mj enum members
    than the snapshot carries."""
    if not mjspec or "enums" not in mjspec:
        return
    for gen_name, gen_def in rules.get("generated_enums", {}).items():
        if gen_name.startswith("_") or not isinstance(gen_def, dict):
            continue
        if gen_def.get("disabled"):
            continue
        for entry in gen_def.get("enums", []):
            if entry.get("disabled"):
                continue
            mj_enum_name = entry.get("from_mj_enum")
            ue_member_from_mj = entry.get("ue_member_from_mj", {})
            exclude = set(entry.get("exclude_mj_members", []))
            c_enum = mjspec["enums"].get(mj_enum_name, {})
            if not c_enum:
                continue
            mapped = set(ue_member_from_mj) | exclude
            missing = [m for m in c_enum if m not in mapped]
            if missing:
                _diag_add(
                    f"[diagnostic] generated_enums['{gen_name}'].{entry.get('ue_name')}: "
                    f"C enum '{mj_enum_name}' has {len(missing)} member(s) with no UE "
                    f"mapping: {missing[:5]}{'...' if len(missing) > 5 else ''}. Add to "
                    f"ue_member_from_mj or exclude_mj_members.",
                    source="enum_drift",
                )


def _emit_drift_diagnostics(schema: Dict[str, Any], rules: Dict[str, Any],
                            mjspec: Optional[Dict[str, Any]]) -> None:
    """Run every drift-coverage check. Each helper surfaces one class
    of drift. All diagnostics route through the module-level
    ``_DIAGS`` collector and flush once at the end of
    ``collect_all_writes``."""
    _check_top_level_elements_coverage(schema, rules)
    _check_actuator_subtypes_coverage(schema, rules)
    _check_sensor_subtypes_coverage(schema, rules)
    _check_setto_param_coverage(schema, rules, mjspec)
    _check_mjs_struct_field_drift(schema, rules, mjspec)
    _check_generated_enum_coverage(rules, mjspec)
