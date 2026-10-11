"""Refresh `protospec_gen/mujoco_defaults.json` from the pinned MuJoCo.

Generates a C++ probe from the selected MuJoCo schema's native struct and
`field=` bindings. Public mjs_default* initialisers supply per-element values;
the small set of top-level structs comes from mjs_defaultSpec. Fields with no
public direct binding are recorded as `unbound`, while a previously probed
default losing its binding fails refresh. The selected install must advertise
the exact source revision, and the compiled probe checks header/runtime version.

    uv run python tools/refresh_mj_defaults.py --source <checkout> \
        --mujoco <matching install> --out <snapshot.json>

Needs CMake and a C++ compiler compatible with the selected MuJoCo library.
Generation reads the committed snapshot, never the probe.
For a UE-toolchain install on Linux, first run
``tools/make_probe_cmake_cache.py`` and pass its output as ``--cmake-cache``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)

from protospec_gen import frontend, mjdefaults  # noqa: E402

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


def classify(f: dict) -> tuple[str, int]:
    """Probe shape from the schema type; native C++ compilation checks layout."""
    t = f["type"]
    arity = t.get("arity")
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
        if "max_chars" in f.get("annotations", {}):
            return "chars", 0
        return "dynamic", 0
    scalar = {"bool": "bool", "int32": "int"}.get(prim, "num")
    if arity is None:
        return scalar + "0", 0
    if arity["kind"] == "unbounded":
        return "dynamic", 0
    count = arity["size"] if arity["kind"] == "fixed" else arity["max"]
    return "array_" + scalar, count


def _native_struct(elem: dict, by_name: dict[str, dict], unions: dict[str, list[str]]) -> str | None:
    struct = elem.get("spec")
    if struct:
        return struct
    name = elem["schema_name"]
    if name.startswith("default_"):
        base = by_name.get(name.removeprefix("default_"))
        if base:
            if base.get("spec"):
                return base["spec"]
            children = {c["tag"] for c in base["children"] if c.get("tag")}
            member_types = {member for c in base["children"] if c.get("union")
                            for member in unions[c["union"]]}
            choices = {e.get("spec") for e in by_name.values()
                       if (e["xml"] in children or e["name"] in member_types)
                       and e.get("spec")}
            if len(choices) == 1:
                return choices.pop()
    if name == "freejoint":
        return by_name["joint"]["spec"]
    return None


def _native_field(f: dict, struct_tokens: set[str]) -> str:
    annotations = f.get("annotations", {})
    field = annotations.get("spec_field", f["xml"])
    if "spec_field" in annotations or field in struct_tokens:
        return field
    candidates = [field[:i] + "_" + field[i:]
                  for i in range(1, len(field))
                  if field[:i] + "_" + field[i:] in struct_tokens]
    if len(candidates) > 1:
        raise SystemExit(f"ambiguous native member for {f['xml']}: {candidates}")
    return candidates[0] if candidates else field


def plan_rows(s: dict, include_dir: str):
    """(instance table, probe rows, dynamic skips), in schema order."""
    declared = _declared_defaults(
        os.path.join(include_dir, "mujoco"))
    native_tokens = set()
    with open(os.path.join(include_dir, "mujoco", "mjspec.h"), encoding="utf-8") as fh:
        spec_header = fh.read()
        native_tokens.update(re.findall(r"\b[A-Za-z_]\w*\b", spec_header))
    instances: dict[str, str] = {}
    rows: list[Row] = []
    dynamic: list[tuple[str, str]] = []
    problems: list[str] = []

    by_name = {e["schema_name"]: e for e in s["elements"]}
    unions = {u["name"]: u["members"] for u in s["unions"]}
    for elem in s["elements"]:
        name = elem["schema_name"]
        struct = _native_struct(elem, by_name, unions)
        if not struct:
            continue
        wanted = []
        for f in elem["fields"]:
            kind, count = classify(f)
            if kind == "dynamic":
                dynamic.append((name, f["xml"]))
                continue
            wanted.append((f, kind, count))
        if not wanted:
            continue

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
        prefix = elem.get("spec_field", "")
        prefix = prefix + "." if prefix else ""
        match = re.search(r"typedef struct " + re.escape(struct) +
                          r"_\s*\{(.*?)\}\s*" + re.escape(struct) + r"\s*;",
                          spec_header, re.S)
        struct_tokens = set(re.findall(r"\b[A-Za-z_]\w*\b", match.group(1))) if match else native_tokens
        for f, kind, count in wanted:
            field = _native_field(f, struct_tokens)
            rows.append(Row(name, f["xml"], f"{base}.{prefix}{field}", kind, count))

    if problems:
        raise SystemExit("cannot probe:\n  " + "\n  ".join(problems))
    return instances, rows, dynamic


# --------------------------------------------------------------------------- #
# The probe                                                                    #
# --------------------------------------------------------------------------- #
_PREAMBLE = r"""// Generated by tools/refresh_mj_defaults.py -- do not edit, do not commit.
//
// Prints one JSON object per line: the value MuJoCo's own initialisers leave in
// every schema-bound native field, including explicit unbound results.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <utility>

#include <mujoco/mujoco.h>

namespace {

template <typename T> auto First(const T& value) { return value; }
template <typename T, size_t N> auto First(const T (&value)[N]) { return value[0]; }

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

void Unbound(const char* e, const char* a) {
  Head(e, a, "unbound");
  std::printf("null}\n");
}

}  // namespace

int main() {
  std::printf("{\"kind\":\"version\",\"header\":%d,\"runtime\":%d}\n",
              mjVERSION_HEADER, mj_version());
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
        base, field = r.access.rsplit(".", 1)
        w(f"  [&]<typename T>(const T& obj) {{")
        w(f"    if constexpr (requires {{ obj.{field}; }}) {{")
        access = f"obj.{field}"
        if r.kind == "enum":
            w(f"  Enum({e}, {a}, (long long)First({access}));")
        elif r.kind.startswith("bool"):
            w(f"  Bool({e}, {a}, (long long)First({access}));")
        elif r.kind.startswith("int"):
            w(f"  Int({e}, {a}, (long long)First({access}));")
        elif r.kind.startswith("num"):
            w(f"  Num({e}, {a}, (double)First({access}));")
        elif r.kind == "chars":
            w(f"  Chars({e}, {a}, {access}, "
              f"(int)(sizeof({access}) / sizeof({access}[0])));")
        elif r.kind.startswith("array_"):
            scalar = {"array_num": "Number(", "array_int": "Number(",
                      "array_bool": "Number("}[r.kind]
            w(f'  Head({e}, {a}, "array");')
            w('  std::printf("[");')
            # Bounded by the C array as well as by the schema arity: a bump that
            # shortened the field would otherwise read past it, and a short
            # result is rejected by name where the file is read.
            w(f"  for (int i = 0; i < {r.count} && i < (int)(sizeof({access})"
              f" / sizeof({access}[0])); ++i) {{")
            w('    if (i) std::printf(",");')
            w(f"    {scalar}(double)({access}[i]));")
            w("  }")
            w('  std::printf("]}\\n");')
        else:
            raise SystemExit(f"unhandled probe kind {r.kind!r}")
        w(f"    }} else Unbound({e}, {a});")
        w(f"  }}({base});")
    w("  return 0;")
    w("}")
    return "\n".join(o) + "\n"


_CMAKE = """\
cmake_minimum_required(VERSION 3.16)
project(mjs_defaults_probe CXX)
set(CMAKE_CXX_STANDARD 20)
add_executable(mjs_defaults_probe mjs_defaults_probe.cc)
target_include_directories(mjs_defaults_probe PRIVATE "${MUJOCO_ROOT}/include")
find_library(MUJOCO_LIB mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
target_link_libraries(mjs_defaults_probe PRIVATE "${MUJOCO_LIB}")
"""


def _toolchain_args(cache_path: str | None) -> list[str]:
    if not cache_path:
        return []
    if not os.path.isfile(cache_path):
        raise SystemExit(f"MuJoCo CMake cache not found: {cache_path}")
    with open(cache_path, encoding="utf-8") as fh:
        lines = fh.readlines()
    settings = {}
    for key in ("CMAKE_CXX_COMPILER", "CMAKE_CXX_FLAGS", "CMAKE_EXE_LINKER_FLAGS"):
        prefix = key + ":"
        found = [line.split("=", 1)[1].strip() for line in lines
                 if line.startswith(prefix) and "=" in line]
        if len(found) != 1:
            raise SystemExit(f"{cache_path} has no unique {key} setting")
        settings[key] = found[0]
    return [f"-D{key}={value}" for key, value in settings.items()]


def build_and_run(source: str, mujoco_root: str, workdir: str,
                  cmake_cache: str | None = None) -> list[dict]:
    with open(os.path.join(workdir, "mjs_defaults_probe.cc"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(source)
    with open(os.path.join(workdir, "CMakeLists.txt"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(_CMAKE)
    build = os.path.join(workdir, "build")
    subprocess.run(
        ["cmake", "-S", workdir, "-B", build, "-DCMAKE_BUILD_TYPE=Release",
         f"-DMUJOCO_ROOT={mujoco_root}", *_toolchain_args(cmake_cache)], check=True)
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


def normalise(s: dict, rows: list[Row], probed: list[dict],
              dynamic: list[tuple[str, str]]) -> tuple[dict, dict]:
    constants = {r["name"]: r["value"] for r in probed
                 if r.get("kind") == "constant"}
    values = {(r["element"], r["attr"]): r for r in probed
              if r.get("kind") not in ("constant", "version")}
    field_of = {(e["schema_name"], f["xml"]): f
                for e in s["elements"] for f in e["fields"]}
    enum_by_name = {e["name"]: e for e in s["enums"]}

    defaults: dict[str, dict] = {}
    skipped: dict[str, dict] = {}

    def skip(element: str, attr: str, reason: str) -> None:
        skipped.setdefault(element, {})[attr] = reason

    for element, attr in dynamic:
        skip(element, attr, "dynamic")

    for row in rows:
        key = (row.element, row.attr)
        record = values.get(key)
        if record is None:
            raise SystemExit(f"probe produced no result for {row.element}.{row.attr}")
        if record["kind"] == "unbound":
            skip(row.element, row.attr, "unbound")
            continue
        raw = record["value"]
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
    raise SystemExit(f"{path} is required to match source and installed library")


def _file_hash(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _library(mujoco_root: str) -> str:
    libdir = os.path.join(mujoco_root, "lib")
    choices = sorted(os.path.join(libdir, name) for name in os.listdir(libdir)
                     if name.startswith("libmujoco.so.") and os.path.isfile(os.path.join(libdir, name)))
    if not choices:
        raise SystemExit(f"no versioned MuJoCo library under {libdir}")
    return choices[-1]


def check_no_lost_bindings(baseline: dict, skipped: dict,
                           schema: dict | None = None) -> None:
    lost = sorted((e, a) for e, attrs in baseline["defaults"].items() for a in attrs
                  if skipped.get(e, {}).get(a) == "unbound")
    if lost:
        raise SystemExit("previously native-bound defaults became unbound: "
                         + ", ".join(f"{e}.{a}" for e, a in lost[:20]))
    if schema is not None:
        prior = {(e, a) for e, attrs in baseline["skipped"].items()
                 for a, reason in attrs.items() if reason == "unbound"}
        fields = {(e["schema_name"], f["xml"]): f
                  for e in schema["elements"] for f in e["fields"]}
        unexpected = []
        for element, attrs in skipped.items():
            for attr, reason in attrs.items():
                if reason != "unbound" or (element, attr) in prior:
                    continue
                annotations = fields[element, attr].get("annotations", {})
                if (annotations.get("reading") != "custom"
                        and annotations.get("writing") != "custom") or "spec_field" in annotations:
                    unexpected.append((element, attr))
        if unexpected:
            raise SystemExit("new ordinary schema fields lack public native "
                             "bindings: " + ", ".join(
                                 f"{e}.{a}" for e, a in sorted(unexpected)[:20]))


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mujoco",
        default=os.path.join(_ROOT, os.pardir, "third_party", "install",
                             "MuJoCo"),
        help="the staged MuJoCo install (headers, lib/, bin/)")
    parser.add_argument("--source", default=frontend.mujoco_src(),
                        help="matching MuJoCo Git checkout containing mjcf.schema")
    parser.add_argument("--cmake-cache", default=None,
                        help="CMakeCache.txt of the build that produced the selected install")
    parser.add_argument("--out", default=mjdefaults.DEFAULTS_PATH)
    parser.add_argument("--keep", action="store_true",
                        help="leave the probe build tree in place")
    args = parser.parse_args(argv)

    mujoco_root = os.path.abspath(args.mujoco).replace("\\", "/")
    if not os.path.isfile(os.path.join(mujoco_root, "include", "mujoco",
                                       "mujoco.h")):
        raise SystemExit(f"no MuJoCo headers under {mujoco_root}")

    source_root = os.path.abspath(args.source)
    revision = mjdefaults.source_revision(source_root)
    if _pin(mujoco_root) != revision:
        raise SystemExit(f"MuJoCo install revision {_pin(mujoco_root)} differs from source {revision}")
    for name in ("mujoco.h", "mjspec.h", "mjmodel.h"):
        source_header = os.path.join(source_root, "include", "mujoco", name)
        installed_header = os.path.join(mujoco_root, "include", "mujoco", name)
        if _file_hash(source_header) != _file_hash(installed_header):
            raise SystemExit(f"installed MuJoCo header {name} differs from selected source")
    ir = frontend.load_schema(source_root, include_probed_defaults=False)
    instances, rows, dynamic = plan_rows(ir, os.path.join(mujoco_root, "include"))
    public_tokens = set()
    for directory, _, names in os.walk(os.path.join(mujoco_root, "include", "mujoco")):
        for name in names:
            if name.endswith(".h"):
                with open(os.path.join(directory, name), encoding="utf-8") as fh:
                    public_tokens.update(re.findall(r"\b[A-Za-z_]\w*\b", fh.read()))
    constants = sorted({
        str(m["c"]) for e in ir["enums"] for m in e["members"]
        if m.get("c") is not None and not str(m["c"]).lstrip("+-").isdigit()
        and str(m["c"]) in public_tokens
        })

    workdir = tempfile.mkdtemp(prefix="mjdefaults-")
    try:
        probed = build_and_run(probe_source(instances, rows, constants),
                               mujoco_root, workdir, args.cmake_cache)
    finally:
        if not args.keep:
            shutil.rmtree(workdir, ignore_errors=True)

    versions = [r for r in probed if r.get("kind") == "version"]
    if len(versions) != 1 or versions[0]["header"] != versions[0]["runtime"]:
        raise SystemExit(f"MuJoCo header/runtime mismatch: {versions}")
    defaults, skipped = normalise(ir, rows, probed, dynamic)
    baseline = mjdefaults.load(mjdefaults.DEFAULTS_PATH)
    check_no_lost_bindings(baseline, skipped, ir)
    payload = {
        "mujoco": revision,
        "header_version": versions[0]["header"],
        "runtime_version": versions[0]["runtime"],
        "header_sha256": _file_hash(os.path.join(mujoco_root, "include", "mujoco", "mujoco.h")),
        "library_sha256": _file_hash(_library(mujoco_root)),
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
