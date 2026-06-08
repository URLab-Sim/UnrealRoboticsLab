# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""URLab codegen — core shared bits.

Dependency root. Owns:

- Diagnostics buffer (DiagBuffer + the _DIAGS / _STRICT_DIAGS_FIRED
  legacy aliases).
- UE type registry (_UE_TYPE_INFO + its three derived views).
- Naming helpers (pascal_case, override_toggle_name).
- Schema readers (schema_attrs, schema_subtype_attrs) — pure dict
  walks against the MJCF schema snapshot.
- Rule-iteration helpers (iter_canon_absorbed / compute_canon_absorbed
  / iter_category_attrs) shared between emitters and drift checks.
- C-type classifier (_classify_c_type / _shapes_compatible) and the
  vec3 convert format table — both drift-check inputs, no emit
  semantics.
- mjs field resolver chain (the 5-strategy bridge between MJCF attr
  names and mjs C struct field names).
- xml_enum value-map resolver (literal + snapshot-driven shapes).

Everything here is read-only against the rules/schema/snapshot —
no file writes, no UPROPERTY emission, no .cpp injection. The
orchestrator (``generate_ue_components``) and the drift-check module
(``_codegen_checks``) both import from here.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Iterator, List, Optional, Sequence, Set, Tuple


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------

@dataclass
class Diagnostic:
    """A non-fatal codegen warning. Collected by ``_diag_add`` during a run
    and flushed once at the end via ``_diag_flush``."""
    message:  str
    source:   str = ""    # short tag for grouping (e.g. "schema_drift")


class DiagBuffer:
    """Collects codegen diagnostics during a run. Encapsulates the
    previous module-global ``_DIAGS`` list + ``_STRICT_DIAGS_FIRED``
    counter behind one object so a future PhaseContext can pass a
    buffer explicitly instead of every emitter mutating module state.

    ``pending`` and ``fired_count`` are surfaced as attributes (not
    properties) so the legacy module-globals can alias the underlying
    list in-place and existing test fixtures keep working without an
    indirection layer."""

    def __init__(self) -> None:
        self.pending: List[Diagnostic] = []
        self.fired_count: int = 0

    def add(self, message: str, *, source: str = "") -> None:
        self.pending.append(Diagnostic(message=message, source=source))

    def flush(self) -> None:
        """Print collected diagnostics to stderr, advance ``fired_count``
        by the number flushed, and clear the pending list. Format
        matches the legacy ``_emit_drift_diagnostics`` output so log-
        greppers keep working."""
        if not self.pending:
            return
        print(f"\n--- codegen diagnostics ({len(self.pending)}) ---",
              file=sys.stderr)
        for d in self.pending:
            print(d.message, file=sys.stderr)
        print(
            "(diagnostics are non-fatal; the codegen has emitted what it "
            "can — fix the rules to silence them.)",
            file=sys.stderr,
        )
        self.fired_count += len(self.pending)
        self.pending.clear()

    def reset(self) -> None:
        self.pending.clear()
        self.fired_count = 0


_DIAGS_BUFFER = DiagBuffer()
# Backward-compat aliases. ``_DIAGS`` is the buffer's own list so
# ``_DIAGS.append(...)`` / ``_DIAGS.clear()`` in legacy code still
# mutate the canonical state. ``_STRICT_DIAGS_FIRED[0]`` is a mirror
# kept in sync by ``_diag_flush`` / ``_reset_diags``.
_DIAGS: List[Diagnostic] = _DIAGS_BUFFER.pending
_STRICT_DIAGS_FIRED: List[int] = [0]


def _diag_add(message: str, *, source: str = "") -> None:
    _DIAGS_BUFFER.add(message, source=source)


def _diag_flush() -> None:
    _DIAGS_BUFFER.flush()
    _STRICT_DIAGS_FIRED[0] = _DIAGS_BUFFER.fired_count


def _reset_diags() -> None:
    """Reset both the diag buffer and the legacy mirror counter. Used
    at the top of ``collect_all_writes`` (so back-to-back runs in the
    same process don't carry diagnostics from the previous invocation)
    and by the pytest fixture in tests/conftest.py."""
    _DIAGS_BUFFER.reset()
    _STRICT_DIAGS_FIRED[0] = 0


# ---------------------------------------------------------------------------
# UE type registry
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _UEType:
    """Single source of truth for a UE type codegen can emit. Every per-
    type dispatch (default initializer, drift-check shape, UHT Units-meta
    gate) reads this row instead of redeclaring the type universe."""
    default_init: str                  # C++ initializer (e.g. ``"0.0f"``)
    shape: Tuple[str, Any]             # drift-check shape (see _classify_c_type)
    accepts_units_meta: bool           # UHT allows Units="..." on this type


# Every UE type codegen can target. ``_attr_default_value``,
# ``_UE_TYPE_FAMILY``, and ``_UE_TYPES_ACCEPTING_UNITS_META`` below are
# thin views over this dict.
_UE_TYPE_INFO: Dict[str, _UEType] = {
    "float":           _UEType("0.0f",                  ("scalar", "num"),    True),
    "double":          _UEType("0.0",                   ("scalar", "num"),    True),
    "int32":           _UEType("0",                     ("scalar", "int"),    True),
    "uint8":           _UEType("0",                     ("scalar", "int"),    True),
    "int16":           _UEType("0",                     ("scalar", "int"),    True),
    "int64":           _UEType("0",                     ("scalar", "int"),    True),
    "bool":            _UEType("false",                 ("scalar", "bool"),   False),
    "FString":         _UEType('TEXT("")',              ("scalar", "string"), False),
    "FVector":         _UEType("FVector::ZeroVector",   ("vec", 3),           True),
    "FVector2D":       _UEType("FVector2D::ZeroVector", ("vec", 2),           True),
    "FIntPoint":       _UEType("FIntPoint(0, 0)",       ("vec", 2),           True),
    "FRotator":        _UEType("FRotator::ZeroRotator", ("vec", 3),           True),
    "FQuat":           _UEType("FQuat::Identity",       ("vec", 4),           False),
    "FLinearColor":    _UEType("FLinearColor::White",   ("vec", 4),           False),
    "TArray<float>":   _UEType("{}",                    ("array", None),      False),
    "TArray<double>":  _UEType("{}",                    ("array", None),      False),
    "TArray<int32>":   _UEType("{}",                    ("array", None),      False),
    "TArray<uint8>":   _UEType("{}",                    ("array", None),      False),
    "TArray<FString>": _UEType("{}",                    ("array", None),      False),
}


def _attr_default_value(ue_type: str) -> str:
    info = _UE_TYPE_INFO.get(ue_type)
    if info is not None:
        return info.default_init
    # Catch-all for novel TArray<...> shapes (e.g. TArray<UObject*>)
    # the registry doesn't enumerate but which still default-construct
    # empty.
    if ue_type.startswith("TArray"):
        return "{}"
    return "{}"


# UE type → shape for ``_check_type_shape_drift``. Derived from
# ``_UE_TYPE_INFO`` so the type universe is declared once.
_UE_TYPE_FAMILY = {k: v.shape for k, v in _UE_TYPE_INFO.items()}


_UE_TYPES_ACCEPTING_UNITS_META = {
    k for k, v in _UE_TYPE_INFO.items() if v.accepts_units_meta
}


def _ue_type_accepts_units_meta(ue_type: str) -> bool:
    """UHT enforces 'Units' meta only on numeric scalars + numeric
    structs. Returns False for TArray<...>, FString, FLinearColor,
    FQuat — codegen silently drops the Units meta for those types so
    the build succeeds."""
    info = _UE_TYPE_INFO.get(ue_type)
    return bool(info and info.accepts_units_meta)


_DYNAMIC_VEC_TYPES = {
    "mjFloatVec", "mjDoubleVec", "mjIntVec", "mjStringVec",
    "mjFloatVecVec", "mjDoubleVecVec", "mjIntVecVec",
}


_KNOWN_UE_UNITS = {
    # UE5 numeric units the editor knows how to render + (optionally)
    # convert via Display Units preferences.
    "m", "cm", "mm", "km",
    "rad", "deg",
}


@dataclass
class FileWrite:
    """One file write the codegen would perform. Collected during the
    phase walk; ``apply_writes`` either flushes them to disk, prints
    diffs, or returns the count under ``--check``."""
    path: str
    content: str


# ---------------------------------------------------------------------------
# Naming helpers
# ---------------------------------------------------------------------------


def pascal_case(snake_or_camel: str) -> str:
    """**Schema-verbatim** naming. We use MuJoCo's attribute name AS-IS
    as the C++ UPROPERTY identifier — no PascalCase, no renames. This
    keeps the codegen pure: schema name == C++ name == wire field name.
    """
    return snake_or_camel


def override_toggle_name(prop_name: str) -> str:
    return f"bOverride_{prop_name}"


# ---------------------------------------------------------------------------
# Rule-iteration helpers
# ---------------------------------------------------------------------------


def iter_canon_absorbed(elem_canons: Iterable[str],
                        canonicalizations: Dict[str, Any]) -> Iterator[str]:
    """Yield every attr absorbed by any canonicalisation in
    ``elem_canons``. Skips unknown canon keys silently — diagnostics
    for those live in the rule-validity drift checks."""
    if not canonicalizations:
        return
    for c in elem_canons:
        cd = canonicalizations.get(c)
        if cd:
            yield from cd.get("absorbs_attrs", ())


def compute_canon_absorbed(elem_canons: Iterable[str],
                           canonicalizations: Dict[str, Any]) -> Set[str]:
    return set(iter_canon_absorbed(elem_canons, canonicalizations))


def iter_category_attrs(attrs: Iterable[str], *,
                        global_excl: Set[str],
                        elem_excl: Set[str],
                        canon_absorbed: Set[str]) -> Iterator[str]:
    """Yield attrs that pass the three standard codegen exclusion
    gates. Call sites layer additional skips after the helper
    (xml_enum_attrs, target-collation-absorbed, etc) — those stay
    local because the extra-skip sets differ per emission stage."""
    for attr in attrs:
        if attr in global_excl or attr in elem_excl or attr in canon_absorbed:
            continue
        yield attr


# ---------------------------------------------------------------------------
# Schema readers
# ---------------------------------------------------------------------------


def schema_attrs(schema: Dict[str, Any], path: str) -> List[str]:
    """Read an attribute list at a dotted path, e.g.
    ``actuator_common.attrs``. Returns [] if missing."""
    cur: Any = schema
    for piece in path.split("."):
        if isinstance(cur, dict) and piece in cur:
            cur = cur[piece]
        else:
            return []
    if isinstance(cur, list):
        if cur and isinstance(cur[0], str):
            return list(cur)
        attrs: List[str] = []
        for entry in cur:
            if isinstance(entry, dict):
                if "attrs" in entry and isinstance(entry["attrs"], list):
                    attrs.extend([str(x) for x in entry["attrs"]])
                else:
                    for v in entry.values():
                        if isinstance(v, list):
                            attrs.extend([str(x) for x in v])
        seen = set()
        out = []
        for a in attrs:
            if a not in seen:
                seen.add(a)
                out.append(a)
        return out
    if isinstance(cur, dict):
        attrs2: List[str] = []
        for v in cur.values():
            if isinstance(v, list):
                attrs2.extend([str(x) for x in v if isinstance(x, str)])
        return attrs2
    return []


def schema_subtype_attrs(schema: Dict[str, Any], category: str,
                         subtype_key: str) -> List[str]:
    """Look up per-subtype attrs from schema['actuator_types'][key] /
    schema['sensor_types'] (list-of-dict). Returns [] if not found."""
    if category == "actuator":
        block = schema.get("actuator_types", {})
        if isinstance(block, dict):
            entry = block.get(subtype_key)
            if isinstance(entry, list):
                return [str(x) for x in entry]
            if isinstance(entry, dict) and "attrs" in entry:
                return [str(x) for x in entry["attrs"]]
        return []
    if category == "sensor":
        block = schema.get("sensor_types", [])
        if isinstance(block, list):
            for entry in block:
                if isinstance(entry, dict) and entry.get("name") == subtype_key:
                    attrs = entry.get("attrs", [])
                    return [str(x) for x in attrs] if isinstance(attrs, list) else []
                if isinstance(entry, str) and entry == subtype_key:
                    return []
        return []
    return []


# ---------------------------------------------------------------------------
# C-type classifier (drift-check inputs)
# ---------------------------------------------------------------------------


def _classify_c_type(c_type: str, array_dim) -> Optional[tuple]:
    """Reduce an introspect c_type + array_dim down to a comparable
    (shape, dim_or_family) tuple. Shapes:
      ``("scalar", fam)`` — single value, fam ∈ {num,int,bool,string}
      ``("vec", N)``      — fixed-size C array of length N
      ``("array", None)`` — dynamic-length C vec (mjFloatVec, etc.)
      ``("opaque", _)``   — mjsElement* and other spec-internal
                            pointers URLab never exposes; returning a
                            distinct tag lets the check skip them
                            without silently accepting "scalar"
                            matches.
    Returns None for genuinely unknown bases so the check stays quiet
    on novel types instead of producing noise.
    """
    ct = c_type.strip()
    base_no_arr = re.sub(r"\s*\[[^\]]*\]\s*", "", ct).strip()
    base_no_ptr = base_no_arr.rstrip(" *").strip()
    if base_no_ptr in _DYNAMIC_VEC_TYPES:
        return ("array", None)
    if base_no_ptr.startswith("mjsElement") or base_no_ptr.endswith("Element"):
        return ("opaque", None)
    if "*" in ct or "mjString" in ct or "std::string" in ct:
        return ("scalar", "string")
    is_array = array_dim is not None and array_dim != 1
    base = base_no_arr
    fam: Optional[str] = None
    if base in ("mjtNum", "float", "double"):
        fam = "num"
    elif base in ("int", "mjtByte", "unsigned char", "uint8_t", "uchar"):
        fam = "int"
    elif base in ("bool", "_Bool"):
        fam = "bool"
    else:
        return None
    if is_array:
        if array_dim in (3, 4):
            return ("vec", array_dim)
        return ("array", None)
    return ("scalar", fam)


def _shapes_compatible(ue_shape: tuple, c_shape: tuple) -> bool:
    """True when ``ue_shape`` can legitimately back ``c_shape``.

    A ``TArray<X>`` (array) is shape-compatible with a fixed-size C
    array (vec) because the codegen writes ``prop.Num()`` elements
    into the C array.
    Vec-to-vec requires the dim to match exactly.
    Scalar↔array always mismatches. Opaque C shapes are skipped."""
    if c_shape[0] == "opaque":
        return True
    if ue_shape[0] == c_shape[0]:
        if ue_shape[0] == "vec":
            return ue_shape[1] == c_shape[1]
        return True
    if ue_shape[0] == "array" and c_shape[0] == "vec":
        return True
    return False


# ---------------------------------------------------------------------------
# Vec3 convert format table
# ---------------------------------------------------------------------------
#
# Maps the per-field ``apply_to_spec_mode`` rule values for FVector
# UPROPERTYs to the per-element write template the synth-struct emitter
# uses. Each entry is a Python format string with two slots: ``{c}``
# (the destination C-array expression) and ``{ue}`` (the source UE
# FVector expression).

_VEC3_CONVERT_FMT = {
    "pos":         "{c}[0] =  {ue}.X / 100.0; {c}[1] = -{ue}.Y / 100.0; {c}[2] =  {ue}.Z / 100.0;",
    "vec3_cm":     "{c}[0] =  {ue}.X / 100.0; {c}[1] = -{ue}.Y / 100.0; {c}[2] =  {ue}.Z / 100.0;",
    "vec3_y_flip": "{c}[0] =  {ue}.X; {c}[1] = -{ue}.Y; {c}[2] =  {ue}.Z;",
    "passthrough": "{c}[0] = {ue}.X; {c}[1] = {ue}.Y; {c}[2] = {ue}.Z;",
}


# ---------------------------------------------------------------------------
# mjs-field resolver chain (the 5-strategy bridge)
# ---------------------------------------------------------------------------


def _resolve_direct(attr: str, mjs_fields: Set[str]) -> Optional[str]:
    """``attr`` matches a struct field verbatim (condim -> condim)."""
    return attr if attr in mjs_fields else None


def _resolve_name_suffix(attr: str, mjs_fields: Set[str]) -> Optional[str]:
    """``attr + "name"`` matches (mesh -> meshname). MuJoCo's name-
    ref fields uniformly append ``name`` to the schema attr."""
    cand = attr + "name"
    return cand if cand in mjs_fields else None


def _resolve_root_name_digits(attr: str, mjs_fields: Set[str]) -> Optional[str]:
    """``<root><digits>`` -> ``<root>name<digits>`` (body1 ->
    bodyname1)."""
    m = re.match(r"^(.*?)(\d+)$", attr)
    if not m:
        return None
    root, digits = m.group(1), m.group(2)
    for candidate in (f"{root}name{digits}", f"{root}_name{digits}"):
        if candidate in mjs_fields:
            return candidate
    return None


def _resolve_underscore_norm(attr: str, mjs_fields: Set[str]) -> Optional[str]:
    """Strip all underscores and look for matching letters
    (solreflimit -> solref_limit). Lossy — only used when nothing
    cleaner fits."""
    norm = attr.replace("_", "")
    for f in mjs_fields:
        if f.replace("_", "") == norm:
            return f
    return None


def _resolve_actuator_prefix(attr: str, mjs_fields: Set[str]) -> Optional[str]:
    """Schema ``actuator*`` -> mjs ``act*`` (actuatorfrclimited ->
    actfrclimited). MuJoCo abbreviates the prefix on its struct
    fields even though the XML attr spells it out."""
    if not attr.startswith("actuator"):
        return None
    cand = "act" + attr[len("actuator"):]
    if cand in mjs_fields:
        return cand
    cand_norm = cand.replace("_", "")
    for f in mjs_fields:
        if f.replace("_", "") == cand_norm:
            return f
    return None


_MJS_FIELD_RESOLVERS = (
    _resolve_direct,
    _resolve_name_suffix,
    _resolve_root_name_digits,
    _resolve_underscore_norm,
    _resolve_actuator_prefix,
)


def _resolve_mjs_field(attr: str, mjs_fields: set,
                       attr_to_mjs_field: Optional[Dict[str, str]] = None) -> str:
    """Pick the correct C field name on a category's mjs struct for a
    given schema attribute. Per-element ``attr_to_mjs_field``
    overrides apply first; otherwise the ``_MJS_FIELD_RESOLVERS``
    chain captures MuJoCo's own naming conventions in priority order.
    Falls back to ``attr`` if nothing matches — the build then
    surfaces the mismatch loudly when the export line tries to access
    ``Element->attr``."""
    if attr_to_mjs_field and attr in attr_to_mjs_field:
        return attr_to_mjs_field[attr]
    if not mjs_fields:
        return attr
    for resolver in _MJS_FIELD_RESOLVERS:
        hit = resolver(attr, mjs_fields)
        if hit is not None:
            return hit
    return attr


# ---------------------------------------------------------------------------
# xml_enum value-map resolver
# ---------------------------------------------------------------------------


def _resolve_value_map(attr: str, enum_def: Dict[str, Any],
                      mjspec: Optional[Dict[str, Any]] = None) -> Dict[str, Sequence[str]]:
    """Return the ``{xml_val -> [UE_enum_member, mj_const]}`` table
    for an xml_enum attr. Three sources, checked in order:

      1) Explicit ``value_map`` in the rule.
      2) ``value_map_from_enum: "mjtX"`` — looks up the C enum in the
         introspect-projected mjspec and derives UE-side member names
         from a sibling ``ue_member_from_mj`` table.
      3) Error.
    """
    if "value_map" in enum_def:
        return enum_def["value_map"]
    enum_name = enum_def.get("value_map_from_enum")
    if enum_name and mjspec:
        c_enum = mjspec.get("enums", {}).get(enum_name)
        ue_member_from_mj: Dict[str, str] = enum_def.get("ue_member_from_mj", {})
        if c_enum:
            out: Dict[str, Sequence[str]] = {}
            for mj_const, _value in c_enum.items():
                ue_member = ue_member_from_mj.get(mj_const)
                if not ue_member:
                    continue
                xml_val = enum_def.get("xml_from_mj", {}).get(mj_const)
                if not xml_val:
                    continue
                out[xml_val] = [ue_member, mj_const]
            if out:
                return out
    raise RuntimeError(
        f"xml_enum {attr}: no value_map and value_map_from_enum did not "
        f"resolve (missing snapshot enum data or ue_member_from_mj rules)"
    )
