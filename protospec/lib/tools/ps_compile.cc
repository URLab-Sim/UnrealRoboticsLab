// ps_compile: read an MJCF file, compile it through the ProtoSpec bridge
// (Model -> WriteMjcf -> mjVFS -> mj_loadXML -> Binding), and print a JSON
// verdict a corpus test can key off.
//
// Contract (stable; the Python corpus harness invokes this exactly so):
//   ps_compile <in.xml> [--base-dir DIR]
//   exit 0  -> compiled with zero errors; stdout carries the JSON verdict
//   exit 3  -> the file uses elements outside the supported IO families (skip);
//              the unsupported tags are on stderr (same signal as ps_roundtrip)
//   exit 2  -> parsed + serialized but the bridge compile failed, or a named
//              element failed to bind with no drop flag; JSON on stdout, detail
//              on stderr
//   exit 1  -> any other error (malformed input, missing file, bad usage)
//
// JSON verdict shape (append-only):
//   { "ok", "path_taken", "errors":[...], "warnings":[...],
//     "bindable_total", "bound", "authored_total", "authored_bound",
//     "unbound_authored":[...], "id_crosscheck_ok" }

#include <cstdio>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "bridge.h"
#include "mjcf.h"
#include "plugin_registry.h"

namespace {

std::string JsonEscape(const std::string& s) {
  std::string o;
  for (char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\t': o += "\\t"; break;
      default: o += c;
    }
  }
  return o;
}

std::string PathName(ps::mjcf::CompilePath p) {
  using P = ps::mjcf::CompilePath;
  switch (p) {
    case P::Auto: return "auto";
    case P::XmlPath: return "xml";
    case P::NativePath: return "native";
  }
  return "?";
}

const std::string kAutoPrefix{ps::kReservedNamePrefix};

bool IsAuto(const std::string& name) {
  return name.rfind(kAutoPrefix, 0) == 0;
}

}  // namespace

int main(int argc, char** argv) {
  const char* in = nullptr;
  std::string base_dir;
  std::string plugin_dir;
  bool emit_xml = false;
  bool allow_external_includes = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--base-dir" && i + 1 < argc) {
      base_dir = argv[++i];
    } else if (a == "--plugin-dir" && i + 1 < argc) {
      plugin_dir = argv[++i];
    } else if (a == "--emit-xml") {
      emit_xml = true;
    } else if (a == "--allow-external-includes") {
      // Opt out of the include-traversal guard: permit <include> paths that
      // resolve outside the root model's directory tree (trusted inputs only).
      allow_external_includes = true;
    } else if (!in) {
      in = argv[i];
    } else {
      std::fprintf(stderr, "unexpected argument: %s\n", a.c_str());
      return 1;
    }
  }
  if (!in) {
    std::fprintf(stderr,
                 "usage: ps_compile <in.xml> [--base-dir DIR] [--plugin-dir DIR] "
                 "[--allow-external-includes]\n");
    return 1;
  }

  // Register the first-party engine plugins so the bridge's mj_loadXML resolves
  // plugin-bearing models (elasticity/sdf/sensor/pid).
  ps::plugin::RegisterFirstPartyPlugins(plugin_dir);

  ps::mjcf::io::ParseOptions parse_opts;
  parse_opts.allow_external_includes = allow_external_includes;
  ps::mjcf::io::ParseResult parsed =
      ps::mjcf::io::ParseMjcfFile(in, parse_opts);
  if (!parsed.ok()) {
    if (parsed.unsupported_only()) {
      for (const auto& d : parsed.errors)
        std::fprintf(stderr, "%s\n", d.Render().c_str());
      return 3;
    }
    for (const auto& d : parsed.errors)
      std::fprintf(stderr, "%s\n", d.Render().c_str());
    return 1;
  }

  ps::mjcf::CompileOptions opts;
  opts.base_dir = base_dir;

  if (emit_xml) {
    std::string xml = ps::mjcf::CompileToXml(*parsed.model, opts);
    std::fwrite(xml.data(), 1, xml.size(), stdout);
    return 0;
  }

  ps::mjcf::Compiled c =
      ps::mjcf::Compile(*parsed.model, opts);

  // Binding coverage + id cross-check.
  int bindable_total = 0, bound = 0, authored_total = 0, authored_bound = 0;
  bool crosscheck_ok = true;
  std::vector<std::string> unbound_authored;
  if (c.model) {
    for (const auto& e : c.binding.entries()) {
      ++bindable_total;
      const bool authored = !e.name.empty() && !IsAuto(e.name);
      if (authored) ++authored_total;
      if (e.id >= 0) {
        ++bound;
        if (authored) ++authored_bound;
        const char* nm = mj_id2name(c.model.get(), e.objtype, e.id);
        if (!nm || e.name != nm) crosscheck_ok = false;
      } else if (authored) {
        unbound_authored.push_back(e.name);
      }
    }
  }

  // Emit the verdict.
  std::printf("{\n");
  std::printf("  \"ok\": %s,\n", c.ok() ? "true" : "false");
  std::printf("  \"path_taken\": \"%s\",\n",
              PathName(c.report.taken).c_str());
  std::printf("  \"errors\": [");
  for (size_t i = 0; i < c.report.errors.size(); ++i)
    std::printf("%s\"%s\"", i ? ", " : "",
                JsonEscape(c.report.errors[i].Render()).c_str());
  std::printf("],\n  \"warnings\": [");
  for (size_t i = 0; i < c.report.warnings.size(); ++i)
    std::printf("%s\"%s\"", i ? ", " : "",
                JsonEscape(c.report.warnings[i].Render()).c_str());
  std::printf("],\n");
  std::printf("  \"bindable_total\": %d,\n", bindable_total);
  std::printf("  \"bound\": %d,\n", bound);
  std::printf("  \"authored_total\": %d,\n", authored_total);
  std::printf("  \"authored_bound\": %d,\n", authored_bound);
  std::printf("  \"unbound_authored\": [");
  for (size_t i = 0; i < unbound_authored.size(); ++i)
    std::printf("%s\"%s\"", i ? ", " : "",
                JsonEscape(unbound_authored[i]).c_str());
  std::printf("],\n");
  std::printf("  \"id_crosscheck_ok\": %s\n", crosscheck_ok ? "true" : "false");
  std::printf("}\n");

  if (!c.ok()) {
    for (const auto& d : c.report.errors)
      std::fprintf(stderr, "%s\n", d.Render().c_str());
    return 2;
  }
  // Binding-coverage judgment (which unbound names are acceptable, e.g. under
  // replicate/attach/composite name mangling) is left to the corpus harness,
  // which has the model's tag set. Exit 0 whenever the compile itself succeeded.
  return 0;
}
