"""Refresh `protospec_gen/mujoco_defaults.json` from the pinned MuJoCo.

Generates a C++ probe from the spec-write plan, builds it against the staged
MuJoCo install, runs it, and normalises what it prints into the data file the
front end reads. See :mod:`protospec_gen.mjdefaults` for why the values come
from a compiled probe rather than from parsing `user_init.c`.

The probe is generated, not hand-written, so a MuJoCo bump that moves a field
moves the probe with it: the field names come from the same `field=` facets and
`SPEC_TARGET` rows the generated spec writes come from, and an attribute whose
binding breaks fails `SpecWritePlan` before it ever reaches here.

    uv run python tools/refresh_mj_defaults.py [--mujoco <install root>]

Needs CMake and a C++ compiler. Nothing else in the toolchain does, which is
why the output is committed: generation reads the file, never the probe.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)

from protospec_gen import emit_ue, frontend, mjdefaults  # noqa: E402

# A struct MuJoCo gives no `mjs_defaultX` of its own is reached through a
# default-initialised mjSpec, which is where its initialiser is actually called
# from (`mjs_defaultSpec` -> `mj_defaultOption`, `mj_defaultVisual`,
# `mj_defaultLROpt`, and the inline `spec->compiler`/`spec->stat` assignments).
# Explicit rather than derived: these are the six the header offers no accessor
# for, and a seventh appearing is a fact about MuJoCo that a person should see.
_FROM_SPEC = {
    "mjSpec": "Spec",
    "mjOption": "Spec.option",
    "mjVisual": "Spec.visual",
    "mjStatistic": "Spec.stat",
    "mjsCompiler": "Spec.compiler",
    "mjLROpt": "Spec.compiler.LRopt",
}


def _default_fn(struct: str, declared: set[str]) -> str | None:
    """MuJoCo's own initialiser for `struct`, if it ships one."""
    name = "mjs_default" + (struct[3:] if struct.startswith("mjs") else struct[2:])
    return name if name in declared else None


def _declared_defaults(include_dir: str) -> set[str]:
    import re
    path = os.path.join(include_dir, "mujoco.h")
    with open(path, "r", encoding="utf-8") as fh:
        return set(re.findall(r"\bmjs_default[A-Za-z]+\b", fh.read()))


# --------------------------------------------------------------------------- #
# Classifying one write                                                        #
# --------------------------------------------------------------------------- #
class Row:
    """One probed attribute: how the probe reads it, and how to read it back."""

    def __init__(self, element: str, attr: str, access: str, kind: str,
                 count: int = 0):
        self.element = element
        self.attr = attr
        self.access = access
        self.kind = kind
        self.count = count


def classify(f: dict, write) -> tuple[str, int]:
    """(probe kind, slot count) for a resolved write, or ("dynamic", 0).

    Mirrors `SpecWritePlan._writable`'s own case split rather than re-deriving
    it: every pairing that rule admits lands in exactly one branch here, and a
    pairing it does not admit never reaches this function.
    """
    t = f["type"]
    arity = t.get("arity")
    ctype, dim = write.ctype, write.dim
    if ctype.endswith("*"):
        return "dynamic", 0
    if t["kind"] == "ref":
        return "dynamic", 0
    if t["kind"] == "named":
        # A flags field is a keyword SET packed into one int. Its initialised
        # state is the empty set, which is what unset already displays, so it
        # carries no default rather than a bitmask nobody can read.
        if arity is not None:
            return "dynamic", 0
        return "enum", 0
    prim = t["prim"]
    if prim == "string":
        if "max_chars" in f.get("annotations", {}) and dim is not None:
            return "chars", 0
        return "dynamic", 0
    scalar = {"bool": "bool", "int32": "int"}.get(prim, "num")
    if arity is None:
        # The class partials' narrower spelling: a scalar attribute onto an
        # array field is that field's first coefficient.
        return (scalar + "0") if dim is not None else scalar, 0
    if arity["kind"] == "unbounded":
        return "dynamic", 0
    count = arity["size"] if arity["kind"] == "fixed" else arity["max"]
    return "array_" + scalar, count


def plan_rows(s: emit_ue.UeSchema, plan: emit_ue.SpecWritePlan):
    """(instance table, probe rows, dynamic skips), in emission order."""
    declared = _declared_defaults(
        os.path.join(frontend.mujoco_src(), "include", "mujoco"))
    instances: dict[str, str] = {}
    rows: list[Row] = []
    dynamic: list[tuple[str, str]] = []
    problems: list[str] = []

    for elem in s.elements:
        name = elem["schema_name"]
        p = plan.by_name[name]
        if not p.writes:
            continue
        wanted = []
        for w in p.writes:
            kind, count = classify(w.f, w)
            if kind == "dynamic":
                dynamic.append((name, w.f["xml"]))
                continue
            wanted.append((w, kind, count))
        if not wanted:
            continue

        struct = p.fields_key.split(".", 1)[0]
        if struct in _FROM_SPEC:
            base = _FROM_SPEC[struct]
        else:
            fn = _default_fn(struct, declared)
            if fn is None:
                problems.append(
                    f"{name} writes onto {struct}, which MuJoCo ships no "
                    f"mjs_default for and which _FROM_SPEC does not reach")
                continue
            base = f"Default_{struct}"
            instances[struct] = fn
        for w, kind, count in wanted:
            access = f"{base}.{p.prefix}{w.field}"
            rows.append(Row(name, w.f["xml"], access, kind, count))

    if problems:
        raise SystemExit("cannot probe:\n  " + "\n  ".join(problems))
    return instances, rows, dynamic


# --------------------------------------------------------------------------- #
# The probe                                                                    #
# --------------------------------------------------------------------------- #
_PREAMBLE = r"""// Generated by tools/refresh_mj_defaults.py -- do not edit, do not commit.
//
// Prints one JSON object per line: the value MuJoCo's own initialisers leave in
// every mjs struct field the spec-write layer binds an MJCF attribute to.
#include <cmath>
#include <cstdio>
#include <cstring>

#include <mujoco/mujoco.h>

namespace {

void Head(const char* element, const char* attr, const char* kind) {
  std::printf("{\"element\":\"%s\",\"attr\":\"%s\",\"kind\":\"%s\",\"value\":",
              element, attr, kind);
}

void Number(double v) {
  if (std::isfinite(v)) {
    std::printf("%.17g", v);
  } else {
    std::printf("null");  // mjNAN and the infinities are markers, not values
  }
}

void Num(const char* e, const char* a, double v) {
  Head(e, a, "num");
  Number(v);
  std::printf("}\n");
}

void Int(const char* e, const char* a, long long v) {
  Head(e, a, "int");
  std::printf("%lld}\n", v);
}

void Bool(const char* e, const char* a, long long v) {
  Head(e, a, "bool");
  std::printf("%s}\n", v ? "true" : "false");
}

void Enum(const char* e, const char* a, long long v) {
  Head(e, a, "enum");
  std::printf("%lld}\n", v);
}

void Chars(const char* e, const char* a, const char* p, int n) {
  Head(e, a, "chars");
  std::printf("\"");
  for (int i = 0; i < n && p[i]; ++i) {
    if (p[i] == '"' || p[i] == '\\') std::printf("\\");
    std::printf("%c", p[i]);
  }
  std::printf("\"}\n");
}

void Constant(const char* name, long long v) {
  std::printf("{\"kind\":\"constant\",\"name\":\"%s\",\"value\":%lld}\n",
              name, v);
}

}  // namespace

int main() {
  mjSpec Spec;
  mjs_defaultSpec(&Spec);
"""


def probe_source(instances: dict[str, str], rows: list[Row],
                 constants: list[str]) -> str:
    o = [_PREAMBLE]
    w = o.append
    for struct, fn in sorted(instances.items()):
        w(f"  {struct} Default_{struct};")
        w(f"  {fn}(&Default_{struct});")
    w("")
    for name in constants:
        w(f'  Constant("{name}", (long long){name});')
    w("")
    for r in rows:
        e, a = f'"{r.element}"', f'"{r.attr}"'
        if r.kind == "enum":
            w(f"  Enum({e}, {a}, (long long)({r.access}));")
        elif r.kind == "bool":
            w(f"  Bool({e}, {a}, (long long)({r.access}));")
        elif r.kind == "bool0":
            w(f"  Bool({e}, {a}, (long long)({r.access}[0]));")
        elif r.kind == "int":
            w(f"  Int({e}, {a}, (long long)({r.access}));")
        elif r.kind == "int0":
            w(f"  Int({e}, {a}, (long long)({r.access}[0]));")
        elif r.kind == "num":
            w(f"  Num({e}, {a}, (double)({r.access}));")
        elif r.kind == "num0":
            w(f"  Num({e}, {a}, (double)({r.access}[0]));")
        elif r.kind == "chars":
            w(f"  Chars({e}, {a}, {r.access}, "
              f"(int)(sizeof({r.access}) / sizeof({r.access}[0])));")
        elif r.kind.startswith("array_"):
            scalar = {"array_num": "Number(", "array_int": "Number(",
                      "array_bool": "Number("}[r.kind]
            w(f'  Head({e}, {a}, "array");')
            w('  std::printf("[");')
            # Bounded by the C array as well as by the schema arity: a bump that
            # shortened the field would otherwise read past it, and a short
            # result is rejected by name where the file is read.
            w(f"  for (int i = 0; i < {r.count} && i < (int)(sizeof({r.access})"
              f" / sizeof({r.access}[0])); ++i) {{")
            w('    if (i) std::printf(",");')
            w(f"    {scalar}(double)({r.access}[i]));")
            w("  }")
            w('  std::printf("]}\\n");')
        else:
            raise SystemExit(f"unhandled probe kind {r.kind!r}")
    w("  return 0;")
    w("}")
    return "\n".join(o) + "\n"


_CMAKE = """\
cmake_minimum_required(VERSION 3.16)
project(mjs_defaults_probe CXX)
set(CMAKE_CXX_STANDARD 17)
add_executable(mjs_defaults_probe mjs_defaults_probe.cc)
target_include_directories(mjs_defaults_probe PRIVATE "${MUJOCO_ROOT}/include")
find_library(MUJOCO_LIB mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
target_link_libraries(mjs_defaults_probe PRIVATE "${MUJOCO_LIB}")
"""


def build_and_run(source: str, mujoco_root: str, workdir: str) -> list[dict]:
    with open(os.path.join(workdir, "mjs_defaults_probe.cc"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(source)
    with open(os.path.join(workdir, "CMakeLists.txt"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(_CMAKE)
    build = os.path.join(workdir, "build")
    subprocess.run(
        ["cmake", "-S", workdir, "-B", build, "-DCMAKE_BUILD_TYPE=Release",
         f"-DMUJOCO_ROOT={mujoco_root}"], check=True)
    subprocess.run(["cmake", "--build", build, "--config", "Release"],
                   check=True)
    exe = None
    for candidate in ("mjs_defaults_probe", "mjs_defaults_probe.exe",
                      os.path.join("Release", "mjs_defaults_probe.exe")):
        path = os.path.join(build, candidate)
        if os.path.isfile(path):
            exe = path
            break
    if exe is None:
        raise SystemExit(f"the probe built but produced no executable in {build}")
    # The shared MuJoCo sits beside the import library on Windows.
    env = dict(os.environ)
    bindir = os.path.join(mujoco_root, "bin")
    for var in ("PATH", "LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH"):
        env[var] = bindir + os.pathsep + env.get(var, "")
    out = subprocess.run([exe], check=True, capture_output=True, text=True,
                         env=env).stdout
    return [json.loads(line) for line in out.splitlines() if line.strip()]


# --------------------------------------------------------------------------- #
# Normalisation                                                                #
# --------------------------------------------------------------------------- #
def _nonnegative(f: dict) -> bool:
    """True when the schema itself forbids a negative value for this field.

    That is what makes a negative initialised value readable as a marker rather
    than as a number: `compiler.settotalmass = -1` is MuJoCo saying "use the
    authored masses", and the schema saying the attribute is positive is the
    only independent evidence of it there is.
    """
    annotations = f.get("annotations", {})
    if annotations.get("positive"):
        return True
    low = annotations.get("min")
    return low is not None and float(low) >= 0.0


def normalise(s: emit_ue.UeSchema, rows: list[Row], probed: list[dict],
              dynamic: list[tuple[str, str]]) -> tuple[dict, dict]:
    constants = {r["name"]: r["value"] for r in probed
                 if r.get("kind") == "constant"}
    values = {(r["element"], r["attr"]): r["value"] for r in probed
              if r.get("kind") != "constant"}
    field_of = {(e["schema_name"], f["xml"]): f
                for e in s.elements for f in e["fields"]}
    enum_by_name = {e["name"]: e for e in s.enums}

    defaults: dict[str, dict] = {}
    skipped: dict[str, dict] = {}

    def skip(element: str, attr: str, reason: str) -> None:
        skipped.setdefault(element, {})[attr] = reason

    for element, attr in dynamic:
        skip(element, attr, "dynamic")

    for row in rows:
        key = (row.element, row.attr)
        raw = values.get(key)
        f = field_of[key]
        if row.kind == "enum":
            keyword = _keyword_for(enum_by_name, f, raw, constants)
            if keyword is None:
                skip(row.element, row.attr, "unmapped")
                continue
            defaults.setdefault(row.element, {})[row.attr] = keyword
            continue
        if row.kind == "chars":
            defaults.setdefault(row.element, {})[row.attr] = raw
            continue
        if row.kind.startswith("array_"):
            if any(v is None for v in raw):
                skip(row.element, row.attr, "sentinel")
                continue
            if _nonnegative(f) and any(v < 0 for v in raw):
                skip(row.element, row.attr, "sentinel")
                continue
            prim = f["type"]["prim"]
            defaults.setdefault(row.element, {})[row.attr] = [
                _cast(v, prim) for v in raw]
            continue
        if raw is None:
            skip(row.element, row.attr, "sentinel")
            continue
        if not isinstance(raw, bool) and _nonnegative(f) and raw < 0:
            skip(row.element, row.attr, "sentinel")
            continue
        if isinstance(raw, bool):
            defaults.setdefault(row.element, {})[row.attr] = raw
        else:
            defaults.setdefault(row.element, {})[row.attr] = _cast(
                raw, f["type"]["prim"])

    return (_sorted(defaults), _sorted(skipped))


def _cast(value, prim: str):
    if prim == "int32":
        return int(value)
    if value == int(value) and abs(value) < 2 ** 53:
        return float(value)
    return value


def _keyword_for(enum_by_name: dict, f: dict, raw, constants: dict):
    """The MJCF keyword whose C constant equals the initialised integer."""
    if raw is None:
        return None
    enum = enum_by_name[f["type"]["name"]]
    for member in enum["members"]:
        c = member.get("c")
        if c is None:
            continue
        value = int(c) if str(c).lstrip("+-").isdigit() else constants.get(str(c))
        if value is not None and value == raw:
            return member["value"]
    return None


def _sorted(table: dict) -> dict:
    return {k: {a: table[k][a] for a in sorted(table[k])}
            for k in sorted(table)}


def _pin(mujoco_root: str) -> str:
    path = os.path.join(mujoco_root, "INSTALLED_SHA.txt")
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            return fh.read().strip().splitlines()[0]
    return "unknown"


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mujoco",
        default=os.path.join(_ROOT, os.pardir, "third_party", "install",
                             "MuJoCo"),
        help="the staged MuJoCo install (headers, lib/, bin/)")
    parser.add_argument("--out", default=mjdefaults.DEFAULTS_PATH)
    parser.add_argument("--keep", action="store_true",
                        help="leave the probe build tree in place")
    args = parser.parse_args(argv)

    mujoco_root = os.path.abspath(args.mujoco).replace("\\", "/")
    if not os.path.isfile(os.path.join(mujoco_root, "include", "mujoco",
                                       "mujoco.h")):
        raise SystemExit(f"no MuJoCo headers under {mujoco_root}")

    ir = frontend.load_schema()
    s = emit_ue.UeSchema(ir)
    plan = emit_ue.SpecWritePlan(s)
    instances, rows, dynamic = plan_rows(s, plan)
    constants = sorted({
        str(m["c"]) for e in s.enums for m in e["members"]
        if m.get("c") is not None and not str(m["c"]).lstrip("+-").isdigit()
        and str(m["c"]) in plan.constants})

    workdir = tempfile.mkdtemp(prefix="mjdefaults-")
    try:
        probed = build_and_run(probe_source(instances, rows, constants),
                               mujoco_root, workdir)
    finally:
        if not args.keep:
            shutil.rmtree(workdir, ignore_errors=True)

    defaults, skipped = normalise(s, rows, probed, dynamic)
    payload = {
        "mujoco": _pin(mujoco_root),
        "probe": "tools/refresh_mj_defaults.py",
        "defaults": defaults,
        "skipped": skipped,
    }
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(payload, fh, indent=1, sort_keys=False)
        fh.write("\n")
    counts = mjdefaults.counts(payload)
    print(f"{args.out}: {counts['defaults']} defaults, "
          + ", ".join(f"{counts[r]} {r}" for r in mjdefaults.SKIP_REASONS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
