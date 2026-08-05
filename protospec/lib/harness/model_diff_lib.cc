// model_diff_lib implementation. The comparison core lifted verbatim from the
// original mj_model_diff.cc (impl-plan T0.2); mj_model_diff is now a thin CLI
// over this, and ps_native_diff reuses DiffModels for its three-way verdict.

#include "model_diff_lib.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>

namespace ps::harness {
namespace {

bool NearlyEqual(double a, double b, const Tol& tol) {
  if (a == b) return true;
  if (std::isnan(a) && std::isnan(b)) return true;  // NaN sentinels match
  if (std::isnan(a) || std::isnan(b)) return false;
  double diff = std::fabs(a - b);
  double scale = std::fmax(std::fabs(a), std::fabs(b));
  return diff <= tol.atol + tol.rtol * scale;
}

// Element-wise comparison of one array field. Floating types use the rel+abs
// tolerance; integral/byte types compare exactly.
template <typename T>
void CompareField(const char* name, const T* a, const T* b, std::int64_t count,
                  const Tol& tol, int max_examples, DiffReport& report) {
  if (count <= 0 || a == nullptr || b == nullptr) return;
  FieldDiff fd;
  fd.field = name;
  fd.count_a = count;
  fd.count_b = count;
  for (std::int64_t i = 0; i < count; ++i) {
    bool eq;
    if constexpr (std::is_floating_point_v<T>) {
      eq = NearlyEqual(static_cast<double>(a[i]), static_cast<double>(b[i]),
                       tol);
    } else {
      eq = (a[i] == b[i]);
    }
    if (!eq) {
      ++fd.num_diff;
      if (static_cast<int>(fd.examples.size()) < max_examples) {
        fd.examples.push_back(
            {i, static_cast<double>(a[i]), static_cast<double>(b[i])});
      }
    }
  }
  if (fd.num_diff > 0) report.fields.push_back(std::move(fd));
}

struct ObjClass {
  const char* label;
  int type;
  std::int64_t count;
};

void CompareNames(const mjModel* a, const mjModel* b, DiffReport& report) {
  const ObjClass classes[] = {
      {"body", mjOBJ_BODY, a->nbody},        {"joint", mjOBJ_JOINT, a->njnt},
      {"geom", mjOBJ_GEOM, a->ngeom},        {"site", mjOBJ_SITE, a->nsite},
      {"camera", mjOBJ_CAMERA, a->ncam},     {"light", mjOBJ_LIGHT, a->nlight},
      {"flex", mjOBJ_FLEX, a->nflex},        {"mesh", mjOBJ_MESH, a->nmesh},
      {"skin", mjOBJ_SKIN, a->nskin},        {"hfield", mjOBJ_HFIELD, a->nhfield},
      {"texture", mjOBJ_TEXTURE, a->ntex},   {"material", mjOBJ_MATERIAL, a->nmat},
      {"pair", mjOBJ_PAIR, a->npair},        {"exclude", mjOBJ_EXCLUDE, a->nexclude},
      {"equality", mjOBJ_EQUALITY, a->neq},  {"tendon", mjOBJ_TENDON, a->ntendon},
      {"actuator", mjOBJ_ACTUATOR, a->nu},   {"sensor", mjOBJ_SENSOR, a->nsensor},
      {"numeric", mjOBJ_NUMERIC, a->nnumeric}, {"text", mjOBJ_TEXT, a->ntext},
      {"tuple", mjOBJ_TUPLE, a->ntuple},     {"key", mjOBJ_KEY, a->nkey},
      {"plugin", mjOBJ_PLUGIN, a->nplugin},
  };
  for (const auto& c : classes) {
    for (std::int64_t id = 0; id < c.count; ++id) {
      const char* na = mj_id2name(a, c.type, static_cast<int>(id));
      const char* nb = mj_id2name(b, c.type, static_cast<int>(id));
      std::string sa = na ? na : "";
      std::string sb = nb ? nb : "";
      if (sa != sb)
        report.names.push_back({c.label, static_cast<int>(id), sa, sb});
    }
  }
}

void CompareSizes(const mjModel* a, const mjModel* b, DiffReport& report) {
#define X(name)                                                       \
  if (static_cast<std::int64_t>(a->name) != static_cast<std::int64_t>(b->name)) \
    report.sizes.push_back({#name, static_cast<std::int64_t>(a->name),          \
                            static_cast<std::int64_t>(b->name)});
  MJMODEL_SIZES
#undef X
  report.sizes_equal = report.sizes.empty();
}

void CompareFields(const mjModel* a, const mjModel* b, const Tol& tol,
                   int max_examples, DiffReport& report) {
  const mjModel* m = a;  // sizes are equal here; row counts read from a
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4244)  // MuJoCo's preamble narrows mjtSize -> int
#endif
  MJMODEL_POINTERS_PREAMBLE(m)
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
  (void)nq;
  (void)nv;
  (void)na;
  (void)nu;
  (void)nmocap;
#define X(type, name, nr, nc)                                             \
  CompareField<type>(#name, a->name, b->name,                            \
                     static_cast<std::int64_t>(m->nr) * static_cast<std::int64_t>(nc), \
                     tol, max_examples, report);
  MJMODEL_POINTERS
#undef X
}

// Forward-kinematics invariant: mj_makeData initializes qpos to qpos0, and
// mj_forward then fills xpos/xquat. Comparing those exposes orientation, unit,
// and frame bugs that identical array sizes would hide.
bool CompareInvariants(const mjModel* a, const mjModel* b, const Tol& tol,
                       int max_examples, DiffReport& report, std::string& err) {
  mjData* da = mj_makeData(a);
  mjData* db = mj_makeData(b);
  if (!da || !db) {
    if (da) mj_deleteData(da);
    if (db) mj_deleteData(db);
    err = "mj_makeData failed";
    return false;
  }
  mj_forward(a, da);
  mj_forward(b, db);
  if (a->nbody == b->nbody) {
    CompareField<mjtNum>("xpos", da->xpos, db->xpos,
                         static_cast<std::int64_t>(a->nbody) * 3, tol,
                         max_examples, report);
    CompareField<mjtNum>("xquat", da->xquat, db->xquat,
                         static_cast<std::int64_t>(a->nbody) * 4, tol,
                         max_examples, report);
    for (auto it = report.fields.begin(); it != report.fields.end();) {
      if (it->field == "xpos" || it->field == "xquat") {
        report.invariants.push_back(std::move(*it));
        it = report.fields.erase(it);
      } else {
        ++it;
      }
    }
  }
  mj_deleteData(da);
  mj_deleteData(db);
  return true;
}

}  // namespace

std::string DiffReport::FirstDivergence() const {
  if (!sizes.empty()) return "size:" + sizes.front().name;
  if (!names.empty())
    return "name:" + names.front().objtype + "[" +
           std::to_string(names.front().id) + "]";
  if (!invariants.empty()) return "invariant:" + invariants.front().field;
  if (!fields.empty()) return "field:" + fields.front().field;
  return "";
}

DiffReport DiffModels(const mjModel* a, const mjModel* b, const Tol& tol,
                      int max_examples, std::string& err) {
  DiffReport report;
  CompareSizes(a, b, report);
  CompareNames(a, b, report);
  if (report.sizes_equal) {
    CompareFields(a, b, tol, max_examples, report);
  }
  CompareInvariants(a, b, tol, max_examples, report, err);
  return report;
}

}  // namespace ps::harness
