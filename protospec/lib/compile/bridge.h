// ProtoSpec compile boundary: Model -> mjModel + Binding (plan DR-5/DR-10/DR-11,
// milestone 5). One shipping implementation exists:
//   * XmlPath: WriteMjcf(model) -> register assets + the XML in an mjVFS ->
//     mj_loadXML -> name-based Binding.
// Auto resolves to XmlPath. report.taken always names the path that actually ran.
// NativePath is a defined enum value but is not supported; requesting it returns
// an error rather than falling back to the XML path.
//
// Purity (CDR-14): Compile / Recompile take const Model& end to end and never
// mutate the tree (no const_cast). Unnamed elements are auto-named only in the
// emitted compile-XML, via a serial-keyed override map -- the tree is untouched.
//
// Concurrency: concurrent Compile calls on the same const Model& are
// memory-safe (per-call state is stack-owned or thread-local). Warning capture
// borrows the process-global mju_user_warning hook only for the mj_loadXML
// window, under a process-global mutex that SERIALIZES ProtoSpec
// compiles: two ProtoSpec borrow windows never overlap, so the host's handler is
// always saved and restored cleanly. The handler is host-owned -- a host compile
// running concurrently with a ProtoSpec compile will briefly see our handler for
// the borrow window (an accepted, documented limitation; see public_api.md). Hook
// callbacks route to a thread-local sink, so captured warnings are only ever
// attributed to the borrowing thread and land in that compile's report. MuJoCo
// plugin registration must complete before concurrent compiles (the engine's own
// process-global rule).
//
// This header forward-declares mjModel/mjData and never includes mujoco.h.
#ifndef PROTOSPEC_COMPILE_BRIDGE_H
#define PROTOSPEC_COMPILE_BRIDGE_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "binding.h"
#include "report.h"
#include "types.h"

struct mjModel_;
typedef struct mjModel_ mjModel;
struct mjData_;
typedef struct mjData_ mjData;

namespace ps::mjcf {

// An in-memory asset injected into the compile VFS (mesh/texture/hfield bytes).
// `name` is matched by MuJoCo case-insensitively on its basename, so it should
// be the file name referenced by the model (directory part is ignored by the
// engine's VFS). This is how a model with mesh/texture buffers compiles with no
// files on disk.
struct VfsAsset {
  std::string name;
  std::vector<std::uint8_t> bytes;
};

// Numeric tolerances. Reserved for the native path and the differential harness;
// the XML path defers all numeric compilation to MuJoCo, so these are a no-op
// there (kept in the contract so callers do not churn when native lands).
struct Tolerance {
  double rtol = 1e-9;
  double atol = 1e-9;
};

struct CompileOptions {
  CompilePath path = CompilePath::Auto;

  // Auto-naming (DR-10): unnamed bindable elements get a deterministic reserved
  // name `<auto_name_prefix><family>:<serial>` in the compile-XML so every
  // element is bindable. Serial-derived, hence stable across tree edits (DR-11).
  // Opt out for pristine name tables, at the cost of unnamed elements being
  // unbindable.
  bool auto_name = true;
  std::string auto_name_prefix = std::string(ps::kReservedNamePrefix);

  // Directory used as the model dir for resolving on-disk meshdir/texturedir
  // assets that are NOT supplied in `vfs_assets`. Empty = no on-disk assets.
  std::string base_dir;

  std::vector<VfsAsset> vfs_assets;
  Tolerance tolerance;

  // Opt-in: run the validator (all tiers) before serialization and fold its
  // findings into the report. Validate errors gate the compile (model == nullptr
  // plus the errors in report.errors); tier-3 warnings flow through. Default
  // false preserves today's behavior, where front-ends validate separately.
  bool validate = false;
};

// Deletes an mjModel via mj_deleteModel (out-of-line so consumers do not pull in
// mujoco.h just to hold a Compiled).
struct ModelDeleter {
  void operator()(mjModel* m) const;
};

struct Compiled {
  std::unique_ptr<mjModel, ModelDeleter> model;  // null on failure
  Binding binding;                               // empty on failure
  CompileReport report;

  bool ok() const { return model != nullptr && report.ok(); }
};

// Compile a Model to an mjModel + Binding. Never throws: failure is
// `model == nullptr` plus at least one error diagnostic in the report.
Compiled Compile(const Model& m, const CompileOptions& opts = {});

// Inspection: the exact MJCF string Compile would hand to MuJoCo (auto-names
// injected per opts). Pure and MuJoCo-free; useful for debugging and tests.
// Empty when the model cannot be serialized (see io::WriteMjcf in mjcf.h);
// Compile reports the same condition as serialize-pass errors.
std::string CompileToXml(const Model& m, const CompileOptions& opts = {});

// Recompile after a structural edit, migrating simulation state (DR-11).
//
// Using `prev` (the binding from the previous compile) and `d` (a live mjData
// on the previous model), per-element qpos/qvel/act/ctrl/mocap slices are
// stashed keyed by element identity (creation serial). The edited `m` is
// compiled fresh; a new mjData is allocated and each surviving element's state
// is written back at its new address. Absent elements fall back to qpos0/zeros;
// d->time is preserved. `d` is read only (const). The returned Compiled holds
// no mjData; the migrated state is returned through `out_data` (caller owns it,
// frees with mj_deleteData; null `out_data` discards the migrated state).
//
// Pure w.r.t. `m`: the edited tree is not mutated. Element identity is the
// serial recorded in `prev`, so surviving elements must keep their serials
// (edit the same tree instance in place, or preserve serials across the edit).
Compiled Recompile(const Model& m, const Compiled& prev, const mjData* d,
                   mjData** out_data, const CompileOptions& opts = {});

}  // namespace ps::mjcf

#endif  // PROTOSPEC_COMPILE_BRIDGE_H
