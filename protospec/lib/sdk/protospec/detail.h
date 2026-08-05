// ProtoSpec SDK internals: genuinely-private helpers, used ONLY within the SDK
// headers themselves. Nothing here is part of the public surface (see the
// sibling headers builders.h / traversal.h / refs.h / classes.h / attach.h),
// and no in-tree consumer outside protospec/lib/sdk refers to these private
// symbols.
//
// The generic tree machinery that the SDK's public verbs AND the in-tree native
// compiler both program against (the whole-tree walk, name access, field access
// by id, the ref prefixer) is NOT here: it lives in model_core.h under
// ps::sdk::internal, a named shared seam with its own contract. This header
// carries the reflection-derived pieces the SDK keeps to itself: the union / ref
// target descriptors, the name-category folding and the dynamic-keyword table +
// its drift guard.
//
// For continuity, the moved ps::sdk::internal names the SDK headers still spell
// as `detail::` are re-exported into this namespace below -- a spelling
// convenience for in-tree SDK/test code, not a widening of the surface.
#ifndef PROTOSPEC_SDK_DETAIL_H
#define PROTOSPEC_SDK_DETAIL_H

#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "protospec/core.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
#include "reflect.h"

namespace ps::sdk::detail {

namespace mj = ps::mjcf;

// --- Shared-core re-exports ----------------------------------------------- //
// The generic machinery that lives in ps::sdk::internal (model_core.h),
// re-exported under the `detail::` spelling the SDK headers and in-tree tests
// already use. The canonical home is model_core.h; these are aliases, not
// definitions.
using internal::ApplyToField;
using internal::ClearField;
using internal::Contains;
using internal::ConstFieldGrab;
using internal::Create;
using internal::EnsureSection;
using internal::FieldAt;
using internal::FieldIdByName;
using internal::GetEnumField;
using internal::has_name_v;
using internal::HasNameField;
using internal::MutFieldGrab;
using internal::NameOf;
using internal::PrefixRefs;
using internal::ReadField;
using internal::SetEnumField;
using internal::SetFixedField;
using internal::SetName;
using internal::SetScalarField;
using internal::SetSeqField;
using internal::SetStrField;
using internal::UnionMemberTypes;
using internal::WalkModelAll;
using internal::WalkModelLive;
using internal::WalkTree;

// --- Reference target sets ------------------------------------------------ //
// The element types a reference field may name, read off the schema's own
// descriptors: one type for a concrete target, every member of the union for a
// namespace with several declarers (a `joint` ref names <joint> or <freejoint>,
// an `actuator` ref names any actuator spelling). Nothing here hand-lists a
// membership.
inline std::vector<mj::ElementType> ResolveTargetName(std::string_view type_name) {
  for (std::size_t i = 0; i < mj::reflect::UnionCount(); ++i) {
    const mj::reflect::UnionDescriptor& u = mj::reflect::UnionAt(i);
    if (u.name == type_name) return {u.members, u.members + u.member_count};
  }
  if (const mj::reflect::ElementDescriptor* d =
          mj::reflect::DescribeByName(type_name)) {
    return {d->type};
  }
  return {};
}

// True when element type `e` is a valid target of the reference field described
// by `fd`. The runtime form of the compile-time check SetRef performs.
inline bool RefAcceptsType(const mj::reflect::FieldDescriptor& fd,
                           mj::ElementType e) {
  return Contains(ResolveTargetName(fd.type_name), e);
}

// The reference target set of the field at reflect id `field_id` on element type
// `t`, or empty when that field is not a typed reference. The runtime form of
// what the profile's reference scan hands out alongside each slot.
inline std::vector<mj::ElementType> RefTargetsAt(mj::ElementType t,
                                                 int field_id) {
  const mj::reflect::ElementDescriptor& d = mj::reflect::Describe(t);
  if (field_id < 0 || field_id >= static_cast<int>(d.field_count)) return {};
  const mj::reflect::FieldDescriptor& fd = d.fields[field_id];
  if (fd.kind != mj::reflect::FieldKind::Ref) return {};
  return ResolveTargetName(fd.type_name);
}

// --- Type-erased element handle ------------------------------------------- //
// (pointer, runtime element type) pair, for parent maps and diagnostics where
// heterogeneous elements share one container. The pointer is the profile's node
// identity, opaque here.
struct Handle {
  const void* ptr = nullptr;
  mj::ElementType type{};
  bool operator==(const Handle& o) const {
    return ptr == o.ptr && type == o.type;
  }
  explicit operator bool() const { return ptr != nullptr; }
};

template <class P, class E>
Handle MakeHandle(const E& e) {
  return Handle{&e, ElementTypeOf<P, E>};
}

// --- Name categories (shared MuJoCo namespaces) --------------------------- //
// MuJoCo names are unique within an object *category*, not per element type.
// Most element types are their own category; the spelling families share one
// namespace each: the two joint spellings, the two tendon spellings, and every
// actuator / sensor / equality spelling. Collision checks, rename rejection and
// clone re-uniquing key on the category so e.g. a FreeJoint "j" does collide
// with a Joint "j".
inline bool InUnionNamespace(std::string_view union_name, mj::ElementType t) {
  const mj::reflect::UnionDescriptor& u = mj::reflect::DescribeUnion(union_name);
  for (std::size_t i = 0; i < u.member_count; ++i)
    if (u.members[i] == t) return true;
  return false;
}

inline int NameCategory(mj::ElementType t) {
  if (t == mj::ElementType::Joint || t == mj::ElementType::FreeJoint) return -1;
  if (InUnionNamespace("TendonAny", t)) return -2;
  if (InUnionNamespace("ActuatorAny", t)) return -3;
  if (InUnionNamespace("SensorAny", t)) return -4;
  if (InUnionNamespace("EqualityAny", t)) return -5;
  return static_cast<int>(t);
}

// The keyword -> target-type table backing dynamic references. Keywords follow
// MuJoCo's mju_str2Type object names; the union-valued arms (tendon / actuator /
// sensor) read their member sets from the union descriptors so they never drift
// from the schema. Built once. The table is also the single set the coverage
// guard (DynRefKeywordGaps) checks against, so keyword lookup and drift
// detection cannot disagree.
inline const std::vector<
    std::pair<std::string_view, std::vector<mj::ElementType>>>&
DynRefTable() {
  using ET = mj::ElementType;
  static const std::vector<
      std::pair<std::string_view, std::vector<mj::ElementType>>>
      table = [] {
        std::vector<std::pair<std::string_view, std::vector<mj::ElementType>>> t;
        t.push_back({"body", {ET::Body}});
        t.push_back({"xbody", {ET::Body}});
        t.push_back({"joint", UnionMemberTypes("JointAny")});
        t.push_back({"geom", {ET::Geom}});
        t.push_back({"site", {ET::Site}});
        t.push_back({"camera", {ET::Camera}});
        t.push_back({"light", {ET::Light}});
        t.push_back({"flex", UnionMemberTypes("FlexAny")});
        t.push_back({"mesh", {ET::Mesh}});
        t.push_back({"skin", {ET::Skin}});
        t.push_back({"hfield", {ET::Hfield}});
        t.push_back({"texture", {ET::Texture}});
        t.push_back({"material", {ET::Material}});
        t.push_back({"tendon", UnionMemberTypes("TendonAny")});
        t.push_back({"actuator", UnionMemberTypes("ActuatorAny")});
        t.push_back({"sensor", UnionMemberTypes("SensorAny")});
        t.push_back({"numeric", {ET::Numeric}});
        t.push_back({"text", {ET::Text}});
        t.push_back({"tuple", {ET::Tuple}});
        t.push_back({"key", {ET::Keyframe}});
        t.push_back({"plugin", {ET::PluginInstance}});
        return t;
      }();
  return table;
}

// The element types a DYNAMIC reference can name, given the runtime keyword its
// sibling type field holds (objtype="body" -> Body, ...). An unknown or empty
// keyword returns the empty set: the name is not scannable, and callers must
// treat it as opaque rather than guess a namespace.
inline std::vector<mj::ElementType> DynRefTargetTypes(std::string_view keyword) {
  for (const auto& [k, v] : DynRefTable())
    if (k == keyword) return v;
  return {};
}

// Drift guard for the dynamic keyword table. Every element type that some typed
// reference field in the schema names AND that is a MuJoCo runtime object must
// be reachable through at least one dynamic keyword; otherwise a frame sensor
// could name that object by objtype but the SDK's rename / delete / referrer
// scan would silently skip it. Returns the reachable-but-orphaned target types,
// empty when the table covers every referenceable runtime object.
//
// Excluded (authoring-time reference kinds, never runtime mjOBJ objects, so no
// objtype keyword names them -- none of "class", "model" or "frame" is an
// mju_str2Type keyword): Default (a class/childclass naming a <default>),
// ModelAsset (a `model` naming an attached submodel) and Frame (an <attach
// frame=> naming the <frame> it splices into, resolved at attach time).
//
// Limitation: the authoritative keyword universe is MuJoCo's mju_str2Type, which
// this MuJoCo-independent layer cannot enumerate. This guard therefore catches a
// new referenceable family added to the SCHEMA; a keyword added to mju_str2Type
// with no schema ref target is out of its reach and must be caught where MuJoCo
// is linked (attic/compile/native.cc consumes str2Type directly).
inline std::vector<mj::ElementType> DynRefKeywordGaps() {
  std::vector<mj::ElementType> reachable;
  for (const auto& [k, v] : DynRefTable())
    for (mj::ElementType t : v)
      if (!Contains(reachable, t)) reachable.push_back(t);

  std::vector<mj::ElementType> gaps;
  for (std::size_t i = 0; i < mj::reflect::ElementCount(); ++i) {
    const mj::reflect::ElementDescriptor& e = mj::reflect::ElementAt(i);
    for (std::size_t f = 0; f < e.field_count; ++f) {
      const mj::reflect::FieldDescriptor& fd = e.fields[f];
      if (fd.kind != mj::reflect::FieldKind::Ref) continue;
      for (mj::ElementType t : ResolveTargetName(fd.type_name)) {
        if (t == mj::ElementType::Default || t == mj::ElementType::ModelAsset ||
            t == mj::ElementType::Frame)
          continue;
        if (!Contains(reachable, t) && !Contains(gaps, t)) gaps.push_back(t);
      }
    }
  }
  return gaps;
}

}  // namespace ps::sdk::detail

#endif  // PROTOSPEC_SDK_DETAIL_H
