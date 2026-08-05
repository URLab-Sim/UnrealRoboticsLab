// ProtoSpec Binding: element identity -> compiled mjModel id (DR-10 / CDR-14).
//
// A Binding is produced by Compile() immediately after a model is compiled. It
// maps every named (or auto-named) ProtoSpec tree element to the id MuJoCo
// assigned, by resolving names through mj_name2id -- the supported, stable way
// to recover ids from a compiled model. Index prediction is unsafe
// (discardvisual/fusestatic compact ids); name lookup is immune, and an element
// that was compiled away simply reports "not found".
//
// This header forward-declares mjModel (never includes mujoco.h): consumers can
// hold and query a Binding without linking the engine. binding.cc is inside the
// MuJoCo quarantine zone.
//
// Lifetime contract: a Binding is a SNAPSHOT of one compile. Field-VALUE edits
// to the tree (e.g. `body.pos = ...`) never invalidate it -- ids do not move.
// STRUCTURAL edits (adding, deleting, or reordering elements) invalidate the id
// mappings; after a structural edit the Binding must be discarded and a fresh
// one obtained via Compile/Recompile. This is a documented contract, not a
// runtime check: the object model is plain data -- `body.pos = ...` and direct
// child-vector mutation cannot bump any counter, so partial automatic staleness
// detection would be false confidence.
#ifndef PROTOSPEC_BRIDGE_BINDING_H
#define PROTOSPEC_BRIDGE_BINDING_H

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "types.h"

struct mjModel_;
typedef struct mjModel_ mjModel;

namespace ps::mjcf {

namespace detail {
class BindingBuilder;
}

// The mjtObj type an element family compiles to (mjOBJ_UNKNOWN == not a bindable
// object, e.g. Frame/Compiler/Option). Defined in binding.cc; declared here so
// the auto-naming walker in compile.cc shares the single trait table.
int ObjTypeOf(ElementType type);

// The short family token used in auto-generated reserved names
// ("<prefix><family>:<serial>"), e.g. "geom", "joint". Empty for non-bindable
// families.
std::string_view FamilyToken(ElementType type);

class Binding {
 public:
  Binding() = default;

  // Identity lookup: any generated element type -> its mjModel id. Empty when
  // the element was not bound (compiled away, or not a bindable family). The
  // lookup key is the element pointer, so this works for the element you hold in
  // the tree you compiled.
  template <class E>
  std::optional<int> Id(const E& elem) const {
    return IdByPtr(&elem);
  }

  // Reverse lookup: mjModel id -> tree element, typed per family. Returns
  // nullptr when no element is bound at that id, or when the element bound there
  // is a different concrete type than requested (e.g. JointAt on a free joint's
  // id yields nullptr; use ElementAt for the type-erased pointer).
  const Body* BodyAt(int id) const;
  const Geom* GeomAt(int id) const;
  const Joint* JointAt(int id) const;
  const Site* SiteAt(int id) const;
  const Camera* CameraAt(int id) const;
  const Light* LightAt(int id) const;
  const Mesh* MeshAt(int id) const;
  const void* ElementAt(int objtype, int id) const;

  // Address sugar over the id maps. Empty when the element is unbound.
  std::optional<int> QposAdr(const Joint& j) const;   // jnt_qposadr
  std::optional<int> DofAdr(const Joint& j) const;     // jnt_dofadr
  std::optional<int> SensorAdr(const SensorAny& s) const;  // sensor_adr
  std::optional<int> ActId(const ActuatorAny& a) const;    // actuator id
  std::optional<int> ActAdr(const ActuatorAny& a) const;   // actuator_actadr

  // Pattern queries for macro-generated elements (composite/replicate/etc.),
  // reachable only by name in the XML path. glob supports '*' and '?'.
  std::vector<int> Find(int objtype, std::string_view glob) const;

  // --- Internal iteration (used by Recompile state migration + diagnostics) - //
  // One recorded element and where it landed. id < 0 == unbound. `name` is the
  // effective compile-XML name (authored, or the auto-generated reserved name);
  // it doubles as the stable loggable identity (impl-plan I1) and lets tooling
  // cross-check binding coverage without re-walking the tree.
  struct Entry {
    const void* elem = nullptr;
    std::uint64_t serial = 0;
    int objtype = 0;         // mjtObj
    ElementType etype = ElementType::Model;
    int id = -1;
    std::string name;
  };
  const std::vector<Entry>& entries() const { return entries_; }

 private:
  friend class detail::BindingBuilder;

  std::optional<int> IdByPtr(const void* elem) const;
  const Entry* EntryAt(int objtype, int id) const;

  const Model* model_ = nullptr;
  const mjModel* m_ = nullptr;               // non-owning; Compiled owns it

  std::vector<Entry> entries_;
  std::unordered_map<const void*, std::size_t> by_ptr_;
  std::unordered_map<std::int64_t, std::size_t> by_id_;   // (objtype<<32)|id
};

namespace detail {

// Populates a Binding during compile. The single writer of Binding's internals;
// befriended above so the Binding surface stays read-only for consumers.
class BindingBuilder {
 public:
  explicit BindingBuilder(Binding& b) : b_(b) {}

  void SetContext(const Model* model, const mjModel* m);
  void Add(const Binding::Entry& e);

 private:
  Binding& b_;
};

}  // namespace detail

}  // namespace ps::mjcf

#endif  // PROTOSPEC_BRIDGE_BINDING_H
