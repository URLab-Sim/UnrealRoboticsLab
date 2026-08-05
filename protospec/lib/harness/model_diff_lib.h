// model_diff_lib: the structural mjModel comparison core, factored out of
// mj_model_diff.cc (impl-plan T0.2) so both the mj_model_diff CLI and the
// three-way ps_native_diff harness share one comparison. The comparison is
// driven entirely by MuJoCo's own mjxmacro tables (MJMODEL_SIZES /
// MJMODEL_POINTERS) so coverage is total and survives version bumps.
//
// This header includes mujoco.h (it is harness code, not shipped product).
#ifndef PROTOSPEC_HARNESS_MODEL_DIFF_LIB_H
#define PROTOSPEC_HARNESS_MODEL_DIFF_LIB_H

#include <cstdint>
#include <string>
#include <vector>

struct mjModel_;
typedef struct mjModel_ mjModel;

namespace ps::harness {

struct Tol {
  double rtol = 1e-9;
  double atol = 1e-9;
};

// A single array field that differed, with a bounded sample of offending indices.
struct FieldDiff {
  std::string field;
  std::string note;
  std::int64_t count_a = 0;
  std::int64_t count_b = 0;
  std::int64_t num_diff = 0;
  struct Example {
    std::int64_t index;
    double a;
    double b;
  };
  std::vector<Example> examples;
};

struct SizeDiff {
  std::string name;
  std::int64_t a;
  std::int64_t b;
};

struct NameDiff {
  std::string objtype;
  int id;
  std::string a;
  std::string b;
};

// The full structural verdict: size ints, name tables, every array field, plus
// forward-kinematics invariants (xpos/xquat at qpos0).
struct DiffReport {
  std::vector<SizeDiff> sizes;
  std::vector<NameDiff> names;
  std::vector<FieldDiff> fields;
  std::vector<FieldDiff> invariants;
  bool sizes_equal = true;

  bool Differs() const {
    return !sizes.empty() || !names.empty() || !fields.empty() ||
           !invariants.empty();
  }

  // The field/size that first distinguishes the two models, for a compact
  // verdict (empty when identical). Prefers a size diff, then a name diff, then
  // an invariant, then an array field -- coarsest-cause-first.
  std::string FirstDivergence() const;
};

// Compare two compiled models. Mirrors mj_model_diff's original sequence:
// sizes, names, array fields (only when sizes match), then fk invariants. On a
// data-allocation failure during the invariant check, `err` is set non-empty
// and the returned report holds whatever was computed before it.
DiffReport DiffModels(const mjModel* a, const mjModel* b, const Tol& tol,
                      int max_examples, std::string& err);

}  // namespace ps::harness

#endif  // PROTOSPEC_HARNESS_MODEL_DIFF_LIB_H
