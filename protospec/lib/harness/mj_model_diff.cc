// mj_model_diff: structurally compare two mjModel structs compiled from MJCF.
//
// Usage: mj_model_diff a.xml b.xml [--json] [--tol T] [--examples N]
//
// Exit codes:
//   0  models are identical within tolerance
//   2  models differ
//   1  a load error occurred (stderr names the file and MuJoCo's message)
//
// The comparison itself lives in model_diff_lib (shared with ps_native_diff);
// this file is now a thin CLI over it: parse args, load both models, diff, and
// print. Output format is unchanged from the original single-file tool.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "model_diff_lib.h"
#include "plugin_registry.h"

namespace {

using ps::harness::DiffReport;
using ps::harness::FieldDiff;
using ps::harness::Tol;

std::string JsonEscape(const std::string& s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      default: out += c;
    }
  }
  return out;
}

void PrintFieldExamples(const FieldDiff& fd) {
  std::printf("  %-28s %lld/%lld differ", fd.field.c_str(),
              static_cast<long long>(fd.num_diff),
              static_cast<long long>(fd.count_a));
  if (!fd.note.empty()) std::printf(" (%s)", fd.note.c_str());
  std::printf("\n");
  for (const auto& e : fd.examples) {
    std::printf("      [%lld]  a=%.17g  b=%.17g\n",
                static_cast<long long>(e.index), e.a, e.b);
  }
}

void PrintText(const DiffReport& report, const char* fa, const char* fb) {
  if (!report.Differs()) {
    std::printf("IDENTICAL\n  %s\n  %s\n", fa, fb);
    return;
  }
  std::printf("DIFFER\n  a: %s\n  b: %s\n", fa, fb);
  if (!report.sizes.empty()) {
    std::printf("\nSIZE INTS (%zu differ):\n", report.sizes.size());
    for (const auto& s : report.sizes) {
      std::printf("  %-20s a=%lld  b=%lld\n", s.name.c_str(),
                  static_cast<long long>(s.a), static_cast<long long>(s.b));
    }
  }
  if (!report.names.empty()) {
    std::printf("\nNAME TABLES (%zu differ):\n", report.names.size());
    for (const auto& n : report.names) {
      std::printf("  %s[%d]  a=\"%s\"  b=\"%s\"\n", n.objtype.c_str(), n.id,
                  n.a.c_str(), n.b.c_str());
    }
  }
  if (!report.invariants.empty()) {
    std::printf("\nFORWARD-KINEMATICS INVARIANTS (%zu differ):\n",
                report.invariants.size());
    for (const auto& fd : report.invariants) PrintFieldExamples(fd);
  }
  if (!report.sizes_equal) {
    std::printf(
        "\nARRAY FIELDS: skipped (size ints differ; align sizes first)\n");
  } else if (!report.fields.empty()) {
    std::printf("\nARRAY FIELDS (%zu differ):\n", report.fields.size());
    for (const auto& fd : report.fields) PrintFieldExamples(fd);
  }
}

void PrintFieldJson(const FieldDiff& fd, bool& first) {
  if (!first) std::printf(",");
  first = false;
  std::printf(
      "\n    {\"field\":\"%s\",\"num_diff\":%lld,\"count\":%lld,\"examples\":[",
      JsonEscape(fd.field).c_str(), static_cast<long long>(fd.num_diff),
      static_cast<long long>(fd.count_a));
  bool ef = true;
  for (const auto& e : fd.examples) {
    if (!ef) std::printf(",");
    ef = false;
    std::printf("{\"index\":%lld,\"a\":%.17g,\"b\":%.17g}",
                static_cast<long long>(e.index), e.a, e.b);
  }
  std::printf("]}");
}

void PrintJson(const DiffReport& report, const char* fa, const char* fb) {
  std::printf("{\n");
  std::printf("  \"a\":\"%s\",\n  \"b\":\"%s\",\n", JsonEscape(fa).c_str(),
              JsonEscape(fb).c_str());
  std::printf("  \"identical\":%s,\n", report.Differs() ? "false" : "true");
  std::printf("  \"sizes\":[");
  bool first = true;
  for (const auto& s : report.sizes) {
    if (!first) std::printf(",");
    first = false;
    std::printf("\n    {\"name\":\"%s\",\"a\":%lld,\"b\":%lld}", s.name.c_str(),
                static_cast<long long>(s.a), static_cast<long long>(s.b));
  }
  std::printf("],\n  \"names\":[");
  first = true;
  for (const auto& n : report.names) {
    if (!first) std::printf(",");
    first = false;
    std::printf(
        "\n    {\"objtype\":\"%s\",\"id\":%d,\"a\":\"%s\",\"b\":\"%s\"}",
        n.objtype.c_str(), n.id, JsonEscape(n.a).c_str(),
        JsonEscape(n.b).c_str());
  }
  std::printf("],\n  \"invariants\":[");
  first = true;
  for (const auto& fd : report.invariants) PrintFieldJson(fd, first);
  std::printf("],\n  \"fields\":[");
  first = true;
  for (const auto& fd : report.fields) PrintFieldJson(fd, first);
  std::printf("]\n}\n");
}

mjModel* Load(const char* path, std::string& err) {
  char errbuf[1024] = {0};
  mjModel* m = mj_loadXML(path, nullptr, errbuf, sizeof(errbuf));
  if (!m) err = errbuf[0] ? errbuf : "unknown load error";
  return m;
}

}  // namespace

int main(int argc, char** argv) {
  const char* fa = nullptr;
  const char* fb = nullptr;
  bool json = false;
  Tol tol;
  int max_examples = 8;
  std::string plugin_dir;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--json") {
      json = true;
    } else if (arg == "--plugin-dir" && i + 1 < argc) {
      plugin_dir = argv[++i];
    } else if (arg == "--tol" && i + 1 < argc) {
      double t = std::atof(argv[++i]);
      tol.rtol = t;
      tol.atol = t;
    } else if (arg == "--atol" && i + 1 < argc) {
      tol.atol = std::atof(argv[++i]);
    } else if (arg == "--rtol" && i + 1 < argc) {
      tol.rtol = std::atof(argv[++i]);
    } else if (arg == "--examples" && i + 1 < argc) {
      max_examples = std::atoi(argv[++i]);
    } else if (!fa) {
      fa = argv[i];
    } else if (!fb) {
      fb = argv[i];
    } else {
      std::fprintf(stderr, "unexpected argument: %s\n", arg.c_str());
      return 1;
    }
  }

  if (!fa || !fb) {
    std::fprintf(stderr,
                 "usage: mj_model_diff a.xml b.xml [--json] [--tol T] "
                 "[--atol A] [--rtol R] [--examples N] [--plugin-dir DIR]\n");
    return 1;
  }

  // Register the first-party engine plugins so plugin-bearing corpus models load
  // (mujoco.elasticity.*, mujoco.sdf.*, mujoco.sensor.touch_grid, mujoco.pid).
  ps::plugin::RegisterFirstPartyPlugins(plugin_dir);

  std::string err;
  mjModel* a = Load(fa, err);
  if (!a) {
    std::fprintf(stderr, "load error in a (%s): %s\n", fa, err.c_str());
    return 1;
  }
  mjModel* b = Load(fb, err);
  if (!b) {
    std::fprintf(stderr, "load error in b (%s): %s\n", fb, err.c_str());
    mj_deleteModel(a);
    return 1;
  }

  std::string diff_err;
  DiffReport report =
      ps::harness::DiffModels(a, b, tol, max_examples, diff_err);
  if (!diff_err.empty()) {
    std::fprintf(stderr, "invariant check failed: %s\n", diff_err.c_str());
    mj_deleteModel(a);
    mj_deleteModel(b);
    return 1;
  }

  if (json) {
    PrintJson(report, fa, fb);
  } else {
    PrintText(report, fa, fb);
  }

  int exit_code = report.Differs() ? 2 : 0;
  mj_deleteModel(a);
  mj_deleteModel(b);
  return exit_code;
}
