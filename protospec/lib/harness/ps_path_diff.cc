// ps_path_diff: compile-path differential harness for the ProtoSpec bridge.
//
// For each input model (a file, or every *.xml under a directory) the tool:
//   1. parses it with the ProtoSpec reader  -> ps::mjcf::Model
//   2. compiles it via ps::mjcf::Compile with compile PATH A -> mjModel A
//   3. compiles it via ps::mjcf::Compile with compile PATH B -> mjModel B
//   4. field-diffs A vs B COMPREHENSIVELY (ps::harness::DiffModels: every
//      MJMODEL_SIZES int, per-object-type name tables in id order, every
//      MJMODEL_POINTERS array field, plus qpos0-forward-kinematics invariants)
//   5. prints PASS / FAIL, and on FAIL a compact field-level report.
// Any FAIL makes the process exit non-zero.
//
// Path A and B are CLI-selectable ps::mjcf::CompilePath values. The default is
// XmlPath vs XmlPath -- an *identity sanity* run: the same deterministic path
// twice must be bit-identical, so any non-PASS is a harness/determinism bug, not
// a path-parity claim.
//
// Mode --against-stock replaces path B with stock mj_loadXML of the ORIGINAL
// file (not a ProtoSpec re-serialization). That catches ProtoSpec reader/writer
// drift -- the round trip changing the model -- which a path-vs-path diff cannot
// see because both legs share the same reader+writer front end.
//
// Mode --bench N times the XML pipeline's three stages (CompileToXml serialize,
// mj_parseXML, mj_compile) plus the total, averaged over N runs, and emits one
// machine-readable line per model.
//
// Exit codes:
//   0  every model PASSed (identical within tolerance) / bench completed
//   2  at least one model FAILed (differed, or a compile/load error in scope)
//   1  usage error / no inputs

#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "bridge.h"
#include "mjcf.h"
#include "model_diff_lib.h"
#include "plugin_registry.h"

namespace {

using ps::harness::DiffReport;
using ps::harness::FieldDiff;
using ps::harness::Tol;
namespace fs = std::filesystem;

// --------------------------------------------------------------------------- //
// CompilePath <-> string                                                       //
// --------------------------------------------------------------------------- //
using Path = ps::mjcf::CompilePath;

bool ParsePath(const std::string& s, Path& out) {
  if (s == "XmlPath" || s == "xml") { out = Path::XmlPath; return true; }
  if (s == "NativePath" || s == "native") { out = Path::NativePath; return true; }
  if (s == "Auto" || s == "auto") { out = Path::Auto; return true; }
  return false;
}

const char* PathName(Path p) {
  switch (p) {
    case Path::Auto: return "auto";
    case Path::XmlPath: return "xml";
    case Path::NativePath: return "native";
  }
  return "?";
}

// --------------------------------------------------------------------------- //
// Compact field-level report (a trimmed form of mj_model_diff's printer)       //
// --------------------------------------------------------------------------- //
void PrintFieldExamples(const FieldDiff& fd) {
  std::printf("    %-26s %lld/%lld differ", fd.field.c_str(),
              static_cast<long long>(fd.num_diff),
              static_cast<long long>(fd.count_a));
  if (!fd.note.empty()) std::printf(" (%s)", fd.note.c_str());
  std::printf("\n");
  for (const auto& e : fd.examples) {
    std::printf("        [%lld]  a=%.17g  b=%.17g\n",
                static_cast<long long>(e.index), e.a, e.b);
  }
}

void PrintReport(const DiffReport& r) {
  if (!r.sizes.empty()) {
    std::printf("  SIZE INTS (%zu differ):\n", r.sizes.size());
    for (const auto& s : r.sizes)
      std::printf("    %-20s a=%lld  b=%lld\n", s.name.c_str(),
                  static_cast<long long>(s.a), static_cast<long long>(s.b));
  }
  if (!r.names.empty()) {
    std::printf("  NAME TABLES (%zu differ):\n", r.names.size());
    for (const auto& n : r.names)
      std::printf("    %s[%d]  a=\"%s\"  b=\"%s\"\n", n.objtype.c_str(), n.id,
                  n.a.c_str(), n.b.c_str());
  }
  if (!r.invariants.empty()) {
    std::printf("  FK INVARIANTS (%zu differ):\n", r.invariants.size());
    for (const auto& fd : r.invariants) PrintFieldExamples(fd);
  }
  if (!r.sizes_equal) {
    std::printf("  ARRAY FIELDS: skipped (size ints differ; align sizes first)\n");
  } else if (!r.fields.empty()) {
    std::printf("  ARRAY FIELDS (%zu differ):\n", r.fields.size());
    for (const auto& fd : r.fields) PrintFieldExamples(fd);
  }
}

// --------------------------------------------------------------------------- //
// Compile helpers                                                              //
// --------------------------------------------------------------------------- //
struct Opts {
  Path path_a = Path::XmlPath;
  Path path_b = Path::XmlPath;
  bool against_stock = false;
  bool json = false;
  int bench = 0;  // >0 enables benchmark mode with N runs
  Tol tol;
  int examples = 6;
  std::string base_dir;       // override; empty -> model's parent dir
  std::string plugin_dir;
};

std::string BaseDirFor(const std::string& model_path, const Opts& o) {
  if (!o.base_dir.empty()) return o.base_dir;
  fs::path p(model_path);
  return p.has_parent_path() ? p.parent_path().string() : std::string(".");
}

// Load the ORIGINAL file with stock MuJoCo (assets resolve relative to the
// file's own directory, exactly as the ProtoSpec bridge's base_dir does).
mjModel* StockLoad(const std::string& path, std::string& err) {
  char buf[1024] = {0};
  mjModel* m = mj_loadXML(path.c_str(), nullptr, buf, sizeof(buf));
  if (!m) err = buf[0] ? buf : "unknown mj_loadXML error";
  return m;
}

// --------------------------------------------------------------------------- //
// One model, one diff verdict                                                  //
// --------------------------------------------------------------------------- //
// Returns true on PASS, false on FAIL. `n_pass`/`n_fail` accumulate.
bool DiffOne(const std::string& model_path, const Opts& o) {
  const std::string base_dir = BaseDirFor(model_path, o);

  ps::mjcf::io::ParseResult parsed = ps::mjcf::io::ParseMjcfFile(model_path);
  if (!parsed.ok()) {
    std::printf("FAIL  %s\n  reader rejected the model (%zu error(s)):\n",
                model_path.c_str(), parsed.errors.size());
    for (const auto& d : parsed.errors)
      std::printf("    %s\n", d.Render().c_str());
    return false;
  }

  ps::mjcf::CompileOptions ca;
  ca.base_dir = base_dir;
  ca.path = o.path_a;
  // Against stock, path A must emit pristine name tables: stock mj_loadXML does
  // not synthesize names for unnamed elements, so ProtoSpec's auto-naming (an
  // intentional bindability feature, not model content) would otherwise flag a
  // spurious nnames/name-table divergence that hides real reader/writer drift.
  if (o.against_stock) ca.auto_name = false;
  ps::mjcf::Compiled a = ps::mjcf::Compile(*parsed.model, ca);
  if (!a.ok()) {
    std::printf("FAIL  %s\n  path A (%s) compile failed:\n", model_path.c_str(),
                PathName(o.path_a));
    for (const auto& d : a.report.errors)
      std::printf("    %s\n", d.Render().c_str());
    return false;
  }

  // Path B is either a second ProtoSpec compile, or stock mj_loadXML.
  ps::mjcf::Compiled b_owned;      // holds the model when path B is a compile
  mjModel* b = nullptr;
  mjModel* b_stock = nullptr;      // holds the model when path B is stock load
  std::string b_label;

  if (o.against_stock) {
    std::string err;
    b_stock = StockLoad(model_path, err);
    if (!b_stock) {
      std::printf("FAIL  %s\n  stock mj_loadXML of the original failed:\n    %s\n",
                  model_path.c_str(), err.c_str());
      return false;
    }
    b = b_stock;
    b_label = "stock mj_loadXML";
  } else {
    ps::mjcf::CompileOptions cb;
    cb.base_dir = base_dir;
    cb.path = o.path_b;
    b_owned = ps::mjcf::Compile(*parsed.model, cb);
    if (!b_owned.ok()) {
      std::printf("FAIL  %s\n  path B (%s) compile failed:\n", model_path.c_str(),
                  PathName(o.path_b));
      for (const auto& d : b_owned.report.errors)
        std::printf("    %s\n", d.Render().c_str());
      return false;
    }
    b = b_owned.model.get();
    b_label = std::string("path ") + PathName(o.path_b);
  }

  std::string diff_err;
  DiffReport r = ps::harness::DiffModels(a.model.get(), b, o.tol, o.examples,
                                         diff_err);

  bool pass = !r.Differs() && diff_err.empty();
  if (pass) {
    std::printf("PASS  %s\n", model_path.c_str());
  } else {
    std::printf("FAIL  %s  [A=path %s | B=%s]\n", model_path.c_str(),
                PathName(o.path_a), b_label.c_str());
    if (!diff_err.empty())
      std::printf("  invariant check error: %s\n", diff_err.c_str());
    if (!r.FirstDivergence().empty())
      std::printf("  first divergence: %s\n", r.FirstDivergence().c_str());
    PrintReport(r);
  }

  if (b_stock) mj_deleteModel(b_stock);
  return pass;
}

// --------------------------------------------------------------------------- //
// Benchmark mode                                                               //
// --------------------------------------------------------------------------- //
double Now() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch())
      .count();
}

// Times the XML compile pipeline's three stages for `model_path`, N runs,
// printing one machine-readable line. Returns false only on hard error.
bool BenchOne(const std::string& model_path, const Opts& o) {
  const std::string base_dir = BaseDirFor(model_path, o);
  ps::mjcf::io::ParseResult parsed = ps::mjcf::io::ParseMjcfFile(model_path);
  if (!parsed.ok()) {
    std::fprintf(stderr, "bench: reader rejected %s\n", model_path.c_str());
    return false;
  }
  ps::mjcf::CompileOptions opts;
  opts.base_dir = base_dir;
  opts.path = Path::XmlPath;

  // A dotfile beside the original so mj_parseXML resolves meshdir/texturedir the
  // same way the bridge's base_dir does (relative to the model's own directory).
  // Thread id, not pid: unique per concurrent run and available everywhere.
  std::ostringstream tag;
  tag << std::this_thread::get_id();
  fs::path sib = fs::path(base_dir) /
                 (std::string("._ps_pd_bench_") + tag.str() + ".xml");

  double t_write = 0, t_parse = 0, t_compile = 0;
  for (int i = 0; i < o.bench; ++i) {
    double s0 = Now();
    std::string xml = ps::mjcf::CompileToXml(*parsed.model, opts);
    double s1 = Now();
    if (xml.empty()) {
      std::fprintf(stderr, "bench: CompileToXml empty for %s\n",
                   model_path.c_str());
      return false;
    }
    if (FILE* f = std::fopen(sib.string().c_str(), "wb")) {
      std::fwrite(xml.data(), 1, xml.size(), f);
      std::fclose(f);
    } else {
      std::fprintf(stderr, "bench: cannot write %s\n", sib.string().c_str());
      return false;
    }
    char err[1024] = {0};
    double p0 = Now();
    mjSpec* spec = mj_parseXML(sib.string().c_str(), nullptr, err, sizeof(err));
    double p1 = Now();
    if (!spec) {
      std::fprintf(stderr, "bench: mj_parseXML failed on %s: %s\n",
                   model_path.c_str(), err);
      fs::remove(sib);
      return false;
    }
    double c0 = Now();
    mjModel* m = mj_compile(spec, nullptr);
    double c1 = Now();
    if (!m) {
      std::fprintf(stderr, "bench: mj_compile failed on %s: %s\n",
                   model_path.c_str(), mjs_getError(spec));
      mj_deleteSpec(spec);
      fs::remove(sib);
      return false;
    }
    t_write += (s1 - s0);
    t_parse += (p1 - p0);
    t_compile += (c1 - c0);
    mj_deleteModel(m);
    mj_deleteSpec(spec);
  }
  fs::remove(sib);

  const double n = static_cast<double>(o.bench);
  const double w = t_write / n, p = t_parse / n, c = t_compile / n;
  std::printf(
      "BENCH model=%s path=xml n=%d write_ms=%.3f parse_ms=%.3f "
      "compile_ms=%.3f total_ms=%.3f\n",
      model_path.c_str(), o.bench, w, p, c, w + p + c);
  return true;
}

// --------------------------------------------------------------------------- //
// Input enumeration                                                            //
// --------------------------------------------------------------------------- //
void CollectModels(const std::string& in, std::vector<std::string>& out) {
  fs::path p(in);
  std::error_code ec;
  if (fs::is_directory(p, ec)) {
    std::vector<std::string> found;
    for (auto it = fs::recursive_directory_iterator(p, ec);
         it != fs::recursive_directory_iterator(); it.increment(ec)) {
      if (ec) break;
      const fs::path& fp = it->path();
      if (fp.extension() == ".xml" &&
          fp.filename().string().rfind("._ps_", 0) != 0)
        found.push_back(fp.string());
    }
    std::sort(found.begin(), found.end());
    for (auto& f : found) out.push_back(std::move(f));
  } else {
    out.push_back(in);
  }
}

void Usage() {
  std::fprintf(
      stderr,
      "usage: ps_path_diff [options] <model.xml | dir>...\n"
      "  --path-a P        compile path A (XmlPath|NativePath|Auto; default XmlPath)\n"
      "  --path-b P        compile path B (default XmlPath)\n"
      "  --against-stock   diff ProtoSpec path A against stock mj_loadXML of the original\n"
      "  --bench N         benchmark the XML pipeline (write/parse/compile), N-run average\n"
      "  --tol T           abs+rel float tolerance (default 1e-9)\n"
      "  --atol A / --rtol R   set abs / rel tolerance separately\n"
      "  --examples N      max example indices per differing field (default 6)\n"
      "  --base-dir DIR    asset base dir override (default: each model's parent)\n"
      "  --plugin-dir DIR  first-party engine-plugin dir (default: beside libmujoco)\n"
      "  --json            (reserved) machine-readable verdicts\n");
}

}  // namespace

int main(int argc, char** argv) {
  Opts o;
  std::vector<std::string> inputs;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs an argument\n", name);
        std::exit(1);
      }
      return argv[++i];
    };
    if (a == "--path-a") {
      if (!ParsePath(need("--path-a"), o.path_a)) {
        std::fprintf(stderr, "unknown compile path for --path-a\n");
        return 1;
      }
    } else if (a == "--path-b") {
      if (!ParsePath(need("--path-b"), o.path_b)) {
        std::fprintf(stderr, "unknown compile path for --path-b\n");
        return 1;
      }
    } else if (a == "--against-stock") {
      o.against_stock = true;
    } else if (a == "--bench") {
      o.bench = std::atoi(need("--bench"));
      if (o.bench <= 0) { std::fprintf(stderr, "--bench N must be > 0\n"); return 1; }
    } else if (a == "--tol") {
      double t = std::atof(need("--tol"));
      o.tol.rtol = t; o.tol.atol = t;
    } else if (a == "--atol") {
      o.tol.atol = std::atof(need("--atol"));
    } else if (a == "--rtol") {
      o.tol.rtol = std::atof(need("--rtol"));
    } else if (a == "--examples") {
      o.examples = std::atoi(need("--examples"));
    } else if (a == "--base-dir") {
      o.base_dir = need("--base-dir");
    } else if (a == "--plugin-dir") {
      o.plugin_dir = need("--plugin-dir");
    } else if (a == "--json") {
      o.json = true;
    } else if (a == "-h" || a == "--help") {
      Usage();
      return 0;
    } else if (!a.empty() && a[0] == '-') {
      std::fprintf(stderr, "unknown option: %s\n", a.c_str());
      Usage();
      return 1;
    } else {
      inputs.push_back(a);
    }
  }

  if (inputs.empty()) {
    Usage();
    return 1;
  }

  // Register the first-party engine plugins so plugin-bearing models load.
  ps::plugin::RegisterFirstPartyPlugins(o.plugin_dir);

  std::vector<std::string> models;
  for (const auto& in : inputs) CollectModels(in, models);
  if (models.empty()) {
    std::fprintf(stderr, "no .xml models found in inputs\n");
    return 1;
  }

  if (o.bench > 0) {
    bool ok = true;
    for (const auto& m : models) ok = BenchOne(m, o) && ok;
    return ok ? 0 : 2;
  }

  int n_pass = 0, n_fail = 0;
  for (const auto& m : models) {
    if (DiffOne(m, o)) ++n_pass; else ++n_fail;
  }
  std::printf("\n=== ps_path_diff: %d PASS, %d FAIL (of %zu) ===\n", n_pass,
              n_fail, models.size());
  return n_fail == 0 ? 0 : 2;
}
