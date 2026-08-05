// Compile boundary implementation (XML path). Inside the MuJoCo quarantine
// zone: includes mujoco.h. See compile.h for the contract and purity guarantees.

#include "bridge.h"

#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <mujoco/mujoco.h>

#include "mjcf.h"
#ifdef PROTOSPEC_NATIVE
#include "native.h"   // attic/compile native compiler; built only when PROTOSPEC_NATIVE
#endif
#include "reflect.h"
#include "validate.h"
#include "visit.h"

namespace ps::mjcf {

void ModelDeleter::operator()(mjModel* m) const {
  if (m) mj_deleteModel(m);
}

namespace {

// --- Generic tree walk ---------------------------------------------------- //

// Reads an element's `name` field (if it has one) without recursing.
struct NameExtractor {
  bool has_name_field = false;
  const std::string* name = nullptr;   // non-null only when authored

  template <class T>
  void field(int, const char* fn, const T& v) {
    if constexpr (std::is_same_v<std::decay_t<T>, ps::opt<std::string>>) {
      if (std::string_view(fn) == "name") {
        has_name_field = true;
        name = v.has_value() ? &v.value() : nullptr;
      }
    }
  }
  template <class T>
  void child(int, const char*, const std::vector<std::unique_ptr<T>>&) {}
  template <class U>
  void union_child(int, const char*, const std::vector<U>&) {}
};

// Asset / deformable families are never auto-named: MuJoCo derives an unnamed
// asset's name from its file (mesh/hfield/texture/skin), and other elements
// reference it by that derived name -- injecting an explicit name would break
// the reference. Unnamed materials/flexes are simply unreferenceable. So these
// families are recorded (bindable by any authored name) but never given a
// reserved auto-name.
bool AutoNameableFamily(int objtype) {
  switch (objtype) {
    case mjOBJ_MESH:
    case mjOBJ_HFIELD:
    case mjOBJ_TEXTURE:
    case mjOBJ_SKIN:
    case mjOBJ_MATERIAL:
    case mjOBJ_FLEX:
      return false;
    default:
      return true;
  }
}

// A pre-binding record: everything known about a bindable element before the
// mjModel exists. `name` is the effective compile-XML name (authored or auto).
struct PreEntry {
  const void* elem;
  std::uint64_t serial;
  ElementType etype;
  int objtype;
  std::string name;    // empty == unnameable, will not bind
};

// Walks the whole tree once: records every bindable element and, when
// auto-naming is on, assigns unnamed elements a serial-derived reserved name in
// the AutoNames override map (the tree is never mutated).
class Collector {
 public:
  Collector(const CompileOptions& opts, std::vector<PreEntry>& pre,
            io::AutoNames& names)
      : opts_(opts), pre_(pre), names_(names) {}

  // Entry point. The Model root is not itself bindable; its `worldbody` list
  // holds the implicit world body, which is emitted as <worldbody> and cannot
  // carry a name -- so the world body is skipped (not recorded, not auto-named)
  // while its children are collected normally. Everything else routes through
  // the generic per-element walk.
  void operator()(const Model& m) {
    TopRecurse top{this};
    Visit(m, top);
  }

  template <class E>
  void operator()(const E& e) {
    // Default-class subtrees hold real Geom/Joint/actuator structs as *templates*
    // (they set class defaults; MJCF forbids a name attribute on them and they
    // are never compiled to objects). Prune the whole subtree: neither the
    // templates nor any nested subclass is recorded or auto-named.
    if constexpr (std::is_same_v<E, Default>) {
      (void)e;
      return;
    }
    const ElementType et = element_type_of<E>::value;
    const int objtype = ObjTypeOf(et);
    if (objtype != mjOBJ_UNKNOWN) {
      NameExtractor ne;
      Visit(e, ne);
      PreEntry pe;
      pe.elem = &e;
      pe.serial = e.serial;
      pe.etype = et;
      pe.objtype = objtype;
      if (ne.name != nullptr) {
        pe.name = *ne.name;
      } else if (ne.has_name_field && opts_.auto_name &&
                 AutoNameableFamily(objtype)) {
        pe.name = opts_.auto_name_prefix + std::string(FamilyToken(et)) + ":" +
                  std::to_string(e.serial);
        names_[e.serial] = pe.name;
      }
      pre_.push_back(std::move(pe));
    }
    // Recurse into children regardless: non-bindable containers (Frame, the
    // section wrappers) still hold bindable descendants.
    Recurse rec{this};
    Visit(e, rec);
  }

 private:
  struct Recurse {
    Collector* c;
    template <class T>
    void field(int, const char*, const T&) {}
    template <class T>
    void child(int, const char*, const std::vector<std::unique_ptr<T>>& l) {
      for (const auto& p : l)
        if (p) (*c)(*p);
    }
    template <class U>
    void union_child(int, const char*, const std::vector<U>& l) {
      for (const auto& item : l)
        std::visit([&](const auto& p) { if (p) (*c)(*p); }, item.node);
    }
  };

  // Recurse the Model's section child lists, special-casing worldbody.
  struct TopRecurse {
    Collector* c;
    template <class T>
    void field(int, const char*, const T&) {}
    template <class T>
    void child(int, const char* name, const std::vector<std::unique_ptr<T>>& l) {
      if constexpr (std::is_same_v<T, Body>) {
        if (std::string_view(name) == "worldbody") {
          for (const auto& b : l)
            if (b) c->RecurseChildrenOnly(*b);   // skip the world body itself
          return;
        }
      }
      for (const auto& p : l)
        if (p) (*c)(*p);
    }
    template <class U>
    void union_child(int, const char*, const std::vector<U>& l) {
      for (const auto& item : l)
        std::visit([&](const auto& p) { if (p) (*c)(*p); }, item.node);
    }
  };

  // Records/auto-names an element's descendants but not the element itself.
  void RecurseChildrenOnly(const Body& b) {
    Recurse rec{this};
    Visit(b, rec);
  }

  const CompileOptions& opts_;
  std::vector<PreEntry>& pre_;
  io::AutoNames& names_;
};

// Compiler flags that can legitimately drop a named element from the model.
struct DropFlags {
  bool discardvisual = false;
  bool fusestatic = false;
  bool any() const { return discardvisual || fusestatic; }
};

DropFlags ReadDropFlags(const Model& m) {
  DropFlags f;
  for (const auto& c : m.compilers) {
    if (!c) continue;
    if (c->discardvisual.has_value()) f.discardvisual = c->discardvisual.value();
    if (c->fusestatic.has_value()) f.fusestatic = c->fusestatic.value();
  }
  return f;
}

// --- mj_loadXML warning capture ------------------------------------------- //
// MuJoCo surfaces load-window warnings on two channels: parse-time warnings
// (e.g. "XML contains a 'NaN'") go through the process-global mju_user_warning
// hook, while compile-time warnings are intercepted by the engine's own
// thread-local log handler and come back in mj_loadXML's error buffer when the
// load nonetheless succeeds. The hook has no context pointer, so a thread_local
// sink routes hook callbacks to the installing thread's collector; warnings
// raised on a thread with no active sink are dropped, never misattributed.
thread_local std::vector<std::string>* t_warning_sink = nullptr;

void CollectWarning(const char* msg) {
  if (t_warning_sink) t_warning_sink->push_back(msg ? msg : "");
}

// Host-ownership of the warning hook (Wave 4 #5, decided). `mju_user_warning` is
// a single process-global function pointer we do not own -- ProtoSpec lives
// inside a host process (Studio) that may install its own handler. So a compile
// only BORROWS it, for the duration of one mj_loadXML / mj_compile call, under
// this process-global mutex. The mutex SERIALIZES ProtoSpec compiles: no two
// borrow windows overlap, so the save/restore can never interleave and leave the
// host's handler pointing at a collector that has already returned. The borrow is
// still visible to the host for that window -- a host compile running concurrently
// with a ProtoSpec compile will briefly see our handler (an accepted, documented
// limitation; the real fix is an upstream context-local warning callback). Warning
// capture is otherwise unchanged: the thread_local sink routes callbacks to the
// borrowing thread's collector, and captured text still lands in the report.
std::mutex& WarningHookMutex() {
  static std::mutex m;
  return m;
}

// RAII borrow of the process-global mju_user_warning hook for one compile window.
// Locks WarningHookMutex (serializing ProtoSpec compiles), saves the host's
// handler, installs CollectWarning routed to `sink`, and restores BOTH the host
// handler and the thread-local sink on scope exit -- on every path, including
// exceptions. Never chains onto our own collector as the "previous" handler (a
// stale sink from an aborted window would otherwise be restored).
class WarningHookBorrow {
 public:
  explicit WarningHookBorrow(std::vector<std::string>* sink)
      : lock_(WarningHookMutex()), prev_(mju_user_warning) {
    if (prev_ == CollectWarning) prev_ = nullptr;
    t_warning_sink = sink;
    mju_user_warning = CollectWarning;
  }
  ~WarningHookBorrow() {
    mju_user_warning = prev_;
    t_warning_sink = nullptr;
  }
  WarningHookBorrow(const WarningHookBorrow&) = delete;
  WarningHookBorrow& operator=(const WarningHookBorrow&) = delete;

 private:
  std::lock_guard<std::mutex> lock_;
  void (*prev_)(const char*);
};

// mj_loadXML through an in-memory VFS: registers the compile-XML plus every
// injected asset, then loads. `base_dir` becomes the model directory MuJoCo uses
// to resolve on-disk assets not supplied in the VFS. Warnings MuJoCo emits
// during the load (both channels above) are appended to `warnings`.
mjModel* LoadFromVfs(const std::string& xml, const CompileOptions& opts,
                     std::string& err, std::vector<std::string>& warnings) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  const std::string kXmlBase = "_ps_compile.xml";
  std::string xml_path =
      opts.base_dir.empty() ? kXmlBase : opts.base_dir + "/" + kXmlBase;

  bool ok = true;
  if (mj_addBufferVFS(&vfs, kXmlBase.c_str(), xml.data(),
                      static_cast<int>(xml.size())) != 0) {
    err = "mj_addBufferVFS failed for the compile XML";
    ok = false;
  }
  for (const VfsAsset& a : opts.vfs_assets) {
    if (!ok) break;
    if (mj_addBufferVFS(&vfs, a.name.c_str(), a.bytes.data(),
                        static_cast<int>(a.bytes.size())) != 0) {
      err = "mj_addBufferVFS failed for asset '" + a.name + "'";
      ok = false;
    }
  }

  mjModel* m = nullptr;
  if (ok) {
    char errbuf[1024] = {0};
    // Borrow the process-global warning hook for just the load window (RAII;
    // serialized by WarningHookMutex, host handler restored on scope exit).
    {
      WarningHookBorrow borrow(&warnings);
      m = mj_loadXML(xml_path.c_str(), &vfs, errbuf, sizeof(errbuf));
    }
    if (!m) {
      err = errbuf[0] ? errbuf : "mj_loadXML failed (no message)";
    } else if (errbuf[0]) {
      // Successful load with a non-empty buffer: the compile warning text.
      warnings.push_back(errbuf);
    }
  }
  mj_deleteVFS(&vfs);
  return m;
}

ps::Diagnostic MakeError(std::string source, std::string msg,
                         ps::SourceLoc loc = {}) {
  ps::Diagnostic d;
  d.source = std::move(source);
  d.message = std::move(msg);
  d.loc = std::move(loc);
  return d;
}

// Build the Binding for a compiled model by resolving every effective name
// through mj_name2id. Shared by the XML and native paths: the native path emits
// byte-identical name tables (differential-verified), so name-based lookup is a
// correct construction of the same Binding on both paths (CDR-4). Warns when a
// named element did not bind and a drop flag could explain it.
void BuildNameBinding(const Model& model, mjModel* m, const CompileOptions& opts,
                      Compiled& out) {
  std::vector<PreEntry> pre;
  io::AutoNames auto_names;
  Collector collect(opts, pre, auto_names);
  collect(model);

  const DropFlags drop = ReadDropFlags(model);
  detail::BindingBuilder builder(out.binding);
  builder.SetContext(&model, m);
  for (const PreEntry& pe : pre) {
    Binding::Entry e;
    e.elem = pe.elem;
    e.serial = pe.serial;
    e.objtype = pe.objtype;
    e.etype = pe.etype;
    e.name = pe.name;
    e.id = pe.name.empty() ? -1 : mj_name2id(m, pe.objtype, pe.name.c_str());
    builder.Add(e);

    if (e.id < 0 && !pe.name.empty() && drop.any()) {
      const char* flag = drop.discardvisual ? "discardvisual" : "fusestatic";
      ps::Diagnostic d;
      d.severity = ps::Diagnostic::Severity::Warning;
      d.source = "bind";
      d.message = "element '" + pe.name +
                  "' did not bind; likely removed by compiler flag '" + flag +
                  "'";
      out.report.warnings.push_back(std::move(d));
    }
  }
}

}  // namespace

std::string CompileToXml(const Model& model, const CompileOptions& opts) {
  std::vector<PreEntry> pre;
  io::AutoNames auto_names;
  Collector collect(opts, pre, auto_names);
  collect(model);
  return io::WriteMjcf(model, auto_names);
}

Compiled Compile(const Model& model, const CompileOptions& opts) {
  Compiled out;
  out.report.requested = opts.path;

  // Opt-in pre-compile validation (default off keeps today's behavior, where
  // the front-ends validate separately). Errors gate the compile per validate's
  // own severity; warnings (tier-3 semantic lint) flow through into the report.
  // The standalone validate::Validate entry point is unchanged for callers that
  // drive validation themselves.
  if (opts.validate) {
    bool gated = false;
    for (ps::Diagnostic& d : validate::Validate(model)) {
      if (d.severity == ps::Diagnostic::Severity::Error) {
        out.report.errors.push_back(std::move(d));
        gated = true;
      } else {
        out.report.warnings.push_back(std::move(d));
      }
    }
    if (gated) return out;
  }

#ifdef PROTOSPEC_NATIVE
  // NativePath is forced: run the native compiler and never fall back to XML.
  // Today it always returns null with UnsupportedNatively (attic/compile/native.cc);
  // when NC1 lands stages it will return a model + native-constructed Binding.
  // Auto never routes here -- it resolves to the XML path; the native compiler
  // stays parked in attic and is only reached when explicitly forced.
  if (opts.path == CompilePath::NativePath) {
    mjModel* nm = compile::NativeCompile(model, opts, out.report);
    if (nm) {
      out.model.reset(nm);
      BuildNameBinding(model, nm, opts, out);  // CDR-4 (name-based construction)
    }
    return out;
  }
#else
  // The native compiler lives in attic/ and is not built (PROTOSPEC_NATIVE off).
  // NativePath (forced) reports UnsupportedNatively and does NOT fall back,
  // honoring its "never silently fall back" contract.
  if (opts.path == CompilePath::NativePath) {
    out.report.taken = CompilePath::NativePath;
    FallbackReason nr;
    nr.feature = "native.not_built";
    nr.count = 1;
    out.report.fallback_reasons.push_back(nr);
    ps::Diagnostic d;
    d.severity = ps::Diagnostic::Severity::Error;
    d.source = "gate";
    d.message =
        "native compile unsupported (UnsupportedNatively): the native compiler "
        "is not built (PROTOSPEC_NATIVE off; sources parked in attic/)";
    out.report.errors.push_back(std::move(d));
    return out;
  }
#endif  // PROTOSPEC_NATIVE

  // XmlPath (forced) and Auto both resolve to the XML path.
  out.report.taken = CompilePath::XmlPath;

  // Collect bindable elements + auto-names (single const walk; tree untouched).
  std::vector<PreEntry> pre;
  io::AutoNames auto_names;
  Collector collect(opts, pre, auto_names);
  collect(model);

  // Serialize with auto-name injection, then compile through the VFS.
  std::vector<ps::Diagnostic> write_errors;
  const std::string xml = io::WriteMjcf(model, auto_names, &write_errors);
  if (xml.empty()) {
    for (ps::Diagnostic& d : write_errors)
      out.report.errors.push_back(MakeError("serialize", std::move(d.message),
                                            d.loc));
    if (out.report.errors.empty())
      out.report.errors.push_back(MakeError("serialize",
                                            "WriteMjcf produced no output"));
    return out;
  }
  std::string load_err;
  std::vector<std::string> load_warnings;
  mjModel* m = LoadFromVfs(xml, opts, load_err, load_warnings);
  for (std::string& w : load_warnings) {
    out.report.warnings.push_back(ps::Diagnostic{
        ps::Diagnostic::Severity::Warning, "load", std::move(w), {}});
  }
  if (!m) {
    out.report.errors.push_back(MakeError("load", std::move(load_err)));
    return out;
  }
  out.model.reset(m);

  // Build the Binding by resolving every effective name through mj_name2id.
  BuildNameBinding(model, m, opts, out);

  return out;
}

// --- Recompile (DR-11) ---------------------------------------------------- //
namespace {

struct JointState {
  std::vector<mjtNum> qpos;
  std::vector<mjtNum> qvel;
};
struct ActState {
  bool has_ctrl = false;
  mjtNum ctrl = 0;
  std::vector<mjtNum> act;
};
struct MocapState {
  mjtNum pos[3];
  mjtNum quat[4];
};

int QposWidth(const mjModel* m, int jid) {
  const int njnt = static_cast<int>(m->njnt);
  const int start = m->jnt_qposadr[jid];
  const int end = (jid + 1 < njnt) ? m->jnt_qposadr[jid + 1]
                                   : static_cast<int>(m->nq);
  return end - start;
}
int DofWidth(const mjModel* m, int jid) {
  const int njnt = static_cast<int>(m->njnt);
  const int start = m->jnt_dofadr[jid];
  const int end = (jid + 1 < njnt) ? m->jnt_dofadr[jid + 1]
                                   : static_cast<int>(m->nv);
  return end - start;
}

}  // namespace

Compiled Recompile(const Model& model, const Compiled& prev, const mjData* d,
                   mjData** out_data, const CompileOptions& opts) {
  if (out_data) *out_data = nullptr;

  // 1. Stash previous state keyed by element serial (identity), using the
  //    previous binding + model; never dereferences the tree.
  const mjModel* pm = prev.model.get();
  std::unordered_map<std::uint64_t, JointState> joints;
  std::unordered_map<std::uint64_t, ActState> acts;
  std::unordered_map<std::uint64_t, MocapState> mocaps;
  const mjtNum stashed_time = d ? d->time : 0;

  if (pm && d) {
    for (const Binding::Entry& e : prev.binding.entries()) {
      if (e.id < 0) continue;
      if (e.objtype == mjOBJ_JOINT) {
        JointState js;
        const int qadr = pm->jnt_qposadr[e.id], qw = QposWidth(pm, e.id);
        const int vadr = pm->jnt_dofadr[e.id], vw = DofWidth(pm, e.id);
        js.qpos.assign(d->qpos + qadr, d->qpos + qadr + qw);
        js.qvel.assign(d->qvel + vadr, d->qvel + vadr + vw);
        joints.emplace(e.serial, std::move(js));
      } else if (e.objtype == mjOBJ_ACTUATOR) {
        ActState as;
        as.has_ctrl = true;
        as.ctrl = d->ctrl[e.id];
        const int aadr = pm->actuator_actadr[e.id];
        const int an = pm->actuator_actnum[e.id];
        if (aadr >= 0 && an > 0)
          as.act.assign(d->act + aadr, d->act + aadr + an);
        acts.emplace(e.serial, std::move(as));
      } else if (e.objtype == mjOBJ_BODY) {
        const int mid = pm->body_mocapid[e.id];
        if (mid >= 0) {
          MocapState ms;
          for (int k = 0; k < 3; ++k) ms.pos[k] = d->mocap_pos[3 * mid + k];
          for (int k = 0; k < 4; ++k) ms.quat[k] = d->mocap_quat[4 * mid + k];
          mocaps.emplace(e.serial, ms);
        }
      }
    }
  }

  // 2. Compile the edited tree fresh (pure w.r.t. model).
  Compiled next = Compile(model, opts);
  if (!next.ok()) return next;
  mjModel* nm = next.model.get();

  // 3. Fresh mjData (qpos0/zeros defaults), then migrate surviving state.
  mjData* nd = mj_makeData(nm);
  if (!nd) {
    next.report.warnings.push_back(
        ps::Diagnostic{ps::Diagnostic::Severity::Warning, "recompile",
                       "mj_makeData failed on the recompiled model; no state "
                       "migrated",
                       {}});
    return next;
  }

  for (const Binding::Entry& e : next.binding.entries()) {
    if (e.id < 0) continue;
    if (e.objtype == mjOBJ_JOINT) {
      auto it = joints.find(e.serial);
      if (it == joints.end()) continue;
      const int qadr = nm->jnt_qposadr[e.id], qw = QposWidth(nm, e.id);
      const int vadr = nm->jnt_dofadr[e.id], vw = DofWidth(nm, e.id);
      // Only migrate when the joint kept its shape (type unchanged); otherwise
      // leave qpos0/zeros for that joint.
      if (static_cast<int>(it->second.qpos.size()) == qw)
        for (int k = 0; k < qw; ++k) nd->qpos[qadr + k] = it->second.qpos[k];
      if (static_cast<int>(it->second.qvel.size()) == vw)
        for (int k = 0; k < vw; ++k) nd->qvel[vadr + k] = it->second.qvel[k];
    } else if (e.objtype == mjOBJ_ACTUATOR) {
      auto it = acts.find(e.serial);
      if (it == acts.end()) continue;
      if (it->second.has_ctrl) nd->ctrl[e.id] = it->second.ctrl;
      const int aadr = nm->actuator_actadr[e.id];
      const int an = nm->actuator_actnum[e.id];
      if (aadr >= 0 && static_cast<int>(it->second.act.size()) == an)
        for (int k = 0; k < an; ++k) nd->act[aadr + k] = it->second.act[k];
    } else if (e.objtype == mjOBJ_BODY) {
      const int mid = nm->body_mocapid[e.id];
      if (mid < 0) continue;
      auto it = mocaps.find(e.serial);
      if (it == mocaps.end()) continue;
      for (int k = 0; k < 3; ++k) nd->mocap_pos[3 * mid + k] = it->second.pos[k];
      for (int k = 0; k < 4; ++k)
        nd->mocap_quat[4 * mid + k] = it->second.quat[k];
    }
  }

  nd->time = stashed_time;
  if (out_data) *out_data = nd;
  else mj_deleteData(nd);
  return next;
}

}  // namespace ps::mjcf
