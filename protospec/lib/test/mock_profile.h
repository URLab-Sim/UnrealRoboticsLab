// A second emission profile, built to be hostile.
//
// The point of this profile is to fail loudly if any algorithm in the SDK or io
// still assumes the plain profile's storage. A mock that mirrors the plain
// profile proves nothing, so this one differs along every trait axis at once:
//
//   presence     MOpt<T>: IsSet()/Ref()/Reset(), no operator*, no has_value()
//   strings      MStr/MView: Len()/Chars(), no operator==, no conversion to
//                std::string_view
//   references   MRef with PRIVATE storage; the only ways in are the profile's
//                shape adapter and its move-only slot proxy
//   children     one flat vector of base pointers per node, deliberately stored
//                in REVERSE insertion order, so document order exists only
//                through the adapter's (slot, sibling index) sort
//   root         the document is a node whose sections are children, not named
//                members
//   fixed arity  MVec3 is three named doubles with no operator[], no data(), no
//                size(); MQuat stores X,Y,Z,W while MJCF authors [w,x,y,z], so
//                code that indexed a fixed-arity value generically would not
//                merely fail to compile -- it would transpose every quaternion
//   sequences    MList/MFixedList: Count()/Grow()/Item()
//   identity     Id() over a private field; Duplicate() instead of a free Clone
//   ownership    an arena, so the owner handle is a bare pointer rather than
//                std::unique_ptr
//
// It models a SUBSET of the schema -- eleven element types, and only the fields
// the tests need -- but every field it does model carries the SCHEMA's field id,
// because that is the contract a real second profile owes: same schema, same
// ids, different storage. Everything schema-shaped (ElementType, the reflect
// tables, the XML binding, the keyword maps) is shared, as it must be.
#ifndef PROTOSPEC_TEST_MOCK_PROFILE_H
#define PROTOSPEC_TEST_MOCK_PROFILE_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "keywords.h"
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
#include "reflect.h"
#include "types.h"

namespace mock {

namespace mj = ps::mjcf;
namespace sdk = ps::sdk;
using mj::ElementType;

struct Mock;

// --- Presence -------------------------------------------------------------- //
// Nothing here spells has_value() or operator*, so any algorithm that reached
// for either fails to compile.

template <class T>
class MOpt {
 public:
  MOpt() = default;
  bool IsSet() const { return set_; }
  const T& Ref() const { return v_; }
  T& Ref() { return v_; }
  void Put(T v) {
    v_ = std::move(v);
    set_ = true;
  }
  void Reset() {
    v_ = T{};
    set_ = false;
  }

 private:
  T v_{};
  bool set_ = false;
};

// --- Strings --------------------------------------------------------------- //

class MStr {
 public:
  MStr() = default;
  explicit MStr(std::string_view s) : s_(s) {}
  std::size_t Len() const { return s_.size(); }
  const char* Chars() const { return s_.c_str(); }
  bool Same(const MStr& o) const { return s_ == o.s_; }
  // Deliberately absent: operator==, operator std::string_view.

 private:
  std::string s_;
};

// A borrowed range of characters. No implicit conversion in either direction.
class MView {
 public:
  MView() = default;
  MView(const char* p, std::size_t n) : p_(p), n_(n) {}
  explicit MView(const MStr& s) : p_(s.Chars()), n_(s.Len()) {}
  std::size_t Len() const { return n_; }
  const char* Chars() const { return p_; }

 private:
  const char* p_ = "";
  std::size_t n_ = 0;
};

inline std::string_view Utf8(MView v) {
  return std::string_view(v.Chars(), v.Len());
}

template <class T>
bool operator==(const MOpt<T>& a, const MOpt<T>& b) {
  if (a.IsSet() != b.IsSet()) return false;
  if (!a.IsSet()) return true;
  if constexpr (std::is_same_v<T, MStr>) {
    return a.Ref().Same(b.Ref());
  } else {
    return a.Ref() == b.Ref();
  }
}

struct MStrPolicy {
  using string = MStr;
  using view = MView;

  static view View(const string& s) { return view(s); }
  static string FromUtf8(std::string_view u) { return string(u); }
  static std::string ToUtf8(view v) { return std::string(Utf8(v)); }
  static bool Equals(view a, view b) { return Utf8(a) == Utf8(b); }
  static bool EqualsUtf8(view a, std::string_view b) { return Utf8(a) == b; }
  static bool Empty(view v) { return v.Len() == 0; }
  static bool StartsWith(view v, std::string_view p) {
    return Utf8(v).starts_with(p);
  }
  static string Concat(view a, view b) {
    std::string out;
    out.reserve(a.Len() + b.Len());
    out.append(Utf8(a));
    out.append(Utf8(b));
    return string(out);
  }
  static std::size_t Hash(view v) {
    return std::hash<std::string_view>{}(Utf8(v));
  }
};

// --- Typed references ------------------------------------------------------ //
// The stored name is private and the profile's own policies are the only
// friends, so an algorithm cannot reach it except through a slot.

class MRef {
 public:
  MRef() = default;
  friend bool operator==(const MRef& a, const MRef& b) {
    return a.name_.Same(b.name_);
  }

 private:
  friend struct MRefPolicy;
  friend struct MShape;
  MStr name_;
};

// --- Fixed arity ----------------------------------------------------------- //

struct MVec3 {
  double a = 0, b = 0, c = 0;
  friend bool operator==(const MVec3&, const MVec3&) = default;
};

class MQuat {
 public:
  MQuat() = default;
  MQuat(double x, double y, double z, double w) : x_(x), y_(y), z_(z), w_(w) {}
  // Storage order is X, Y, Z, W. MJCF authors [w, x, y, z].
  double X() const { return x_; }
  double Y() const { return y_; }
  double Z() const { return z_; }
  double W() const { return w_; }
  friend bool operator==(const MQuat&, const MQuat&) = default;

 private:
  double x_ = 0, y_ = 0, z_ = 0, w_ = 0;
};

// --- Sequences ------------------------------------------------------------- //

template <class T, std::size_t N>
class MFixedList {
 public:
  static constexpr std::size_t cap = N;
  std::size_t Count() const { return n_; }
  void Grow(std::size_t n) { n_ = n < N ? n : N; }
  T& Item(std::size_t i) { return v_[i]; }
  const T& Item(std::size_t i) const { return v_[i]; }
  friend bool operator==(const MFixedList& a, const MFixedList& b) {
    if (a.n_ != b.n_) return false;
    for (std::size_t i = 0; i < a.n_; ++i)
      if (!(a.v_[i] == b.v_[i])) return false;
    return true;
  }

 private:
  T v_[N]{};
  std::size_t n_ = 0;
};

template <class T>
class MList {
 public:
  std::size_t Count() const { return v_.size(); }
  void Grow(std::size_t n) { v_.resize(n); }
  T& Item(std::size_t i) { return v_[i]; }
  const T& Item(std::size_t i) const { return v_[i]; }
  friend bool operator==(const MList& a, const MList& b) { return a.v_ == b.v_; }

 private:
  std::vector<T> v_;
};

// --- Nodes ----------------------------------------------------------------- //

class MNode {
 public:
  explicit MNode(ElementType t) : type_(t), id_(NextId()) {}
  virtual ~MNode() = default;

  ElementType Type() const { return type_; }
  std::uint64_t Id() const { return id_; }
  void SetId(std::uint64_t v) { id_ = v; }
  ps::SourceLoc Where() const { return loc_; }
  void SetWhere(ps::SourceLoc l) { loc_ = std::move(l); }

  // The name slot. `GetName`/`SetName` are what the SDK reaches through the
  // profile; `NameSlot` exists only so the profile's own Visit can hand the
  // slot to the reader and writer, exactly as every profile must.
  MOpt<MStr>& NameSlot() { return name_; }
  const MOpt<MStr>& NameSlot() const { return name_; }
  void SetNameFrom(MView v) { name_.Put(MStr(Utf8(v))); }

  MNode* Parent() const { return parent_; }
  int Slot() const { return slot_; }
  int SiblingIndex() const { return sibling_; }
  std::vector<MNode*>& Kids() { return kids_; }
  const std::vector<MNode*>& Kids() const { return kids_; }

  void Link(MNode* parent, int slot, int sibling) {
    parent_ = parent;
    slot_ = slot;
    sibling_ = sibling;
  }
  void SetSibling(int s) { sibling_ = s; }

  static std::uint64_t NextId() {
    static std::uint64_t counter = 0;
    return ++counter;
  }

 private:
  MOpt<MStr> name_;
  ElementType type_;
  std::uint64_t id_;
  ps::SourceLoc loc_;
  MNode* parent_ = nullptr;
  int slot_ = -1;
  int sibling_ = 0;
  std::vector<MNode*> kids_;
};

// Every node the mock ever creates lives here, so the owner handle a profile
// hands around is a bare pointer rather than std::unique_ptr.
inline std::vector<std::unique_ptr<MNode>>& Arena() {
  static std::vector<std::unique_ptr<MNode>> a;
  return a;
}

template <class T>
T* NewNode() {
  auto p = std::make_unique<T>();
  T* raw = p.get();
  Arena().push_back(std::move(p));
  return raw;
}

// --- Element types --------------------------------------------------------- //
// Field ids below are the SCHEMA's. A field the mock does not model is simply
// never visited; the reader and writer skip it.

struct MModel : MNode {
  MModel() : MNode(ElementType::Model) {}
  MOpt<MStr> model;
};

struct MAsset : MNode {
  MAsset() : MNode(ElementType::Asset) {}
};

struct MSensorSection : MNode {
  MSensorSection() : MNode(ElementType::Sensor) {}
};

struct MMesh : MNode {
  MMesh() : MNode(ElementType::Mesh) {}
  MOpt<MStr> file;
  MOpt<MVec3> scale;
};

struct MMaterial : MNode {
  MMaterial() : MNode(ElementType::Material) {}
  MOpt<bool> texuniform;
};

struct MLayer : MNode {
  MLayer() : MNode(ElementType::MaterialLayer) {}
  MOpt<MRef> texture;
  MOpt<MStr> role;
};

struct MBody : MNode {
  MBody() : MNode(ElementType::Body) {}
  MOpt<MRef> childclass;
  MOpt<MVec3> pos;
  MOpt<MQuat> quat;
};

struct MGeom : MNode {
  MGeom() : MNode(ElementType::Geom) {}
  MOpt<MRef> dclass;
  MOpt<mj::GeomType> type;
  MOpt<MFixedList<double, 3>> size;
  MOpt<MRef> material;
  MOpt<MVec3> pos;
  MOpt<MQuat> quat;
  MOpt<MRef> mesh;
  MOpt<MList<double>> user;
};

struct MJoint : MNode {
  MJoint() : MNode(ElementType::Joint) {}
  MOpt<MRef> dclass;
  MOpt<mj::JointType> type;
  MOpt<MVec3> axis;
};

// A <default> class: its identity is `dclass`, so the profile's name accessor
// reports the base name slot at the schema's `dclass` id -- the same "the name
// is not always called name" case the plain profile has.
struct MDefault : MNode {
  MDefault() : MNode(ElementType::Default) {}
};

struct MSensorPlugin : MNode {
  MSensorPlugin() : MNode(ElementType::SensorPlugin) {}
  MOpt<MStr> objtype;
  MOpt<MStr> objname;
};

// --- Visit ----------------------------------------------------------------- //

template <class V>
void MVisit(MModel& e, V& v) {
  v.field(0, "model", e.model);
}
template <class V>
void MVisit(const MModel& e, V& v) {
  v.field(0, "model", e.model);
}
template <class V>
void MVisit(MAsset&, V&) {}
template <class V>
void MVisit(const MAsset&, V&) {}
template <class V>
void MVisit(MSensorSection&, V&) {}
template <class V>
void MVisit(const MSensorSection&, V&) {}

template <class V>
void MVisit(MMesh& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(3, "file", e.file);
  v.field(10, "scale", e.scale);
}
template <class V>
void MVisit(const MMesh& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(3, "file", e.file);
  v.field(10, "scale", e.scale);
}

template <class V>
void MVisit(MMaterial& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(3, "texuniform", e.texuniform);
}
template <class V>
void MVisit(const MMaterial& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(3, "texuniform", e.texuniform);
}

template <class V>
void MVisit(MLayer& e, V& v) {
  v.field(0, "texture", e.texture);
  v.field(1, "role", e.role);
}
template <class V>
void MVisit(const MLayer& e, V& v) {
  v.field(0, "texture", e.texture);
  v.field(1, "role", e.role);
}

template <class V>
void MVisit(MBody& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(1, "childclass", e.childclass);
  v.field(2, "pos", e.pos);
  v.field(3, "quat", e.quat);
}
template <class V>
void MVisit(const MBody& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(1, "childclass", e.childclass);
  v.field(2, "pos", e.pos);
  v.field(3, "quat", e.quat);
}

template <class V>
void MVisit(MGeom& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(1, "dclass", e.dclass);
  v.field(2, "type", e.type);
  v.field(8, "size", e.size);
  v.field(9, "material", e.material);
  v.field(22, "pos", e.pos);
  v.field(23, "quat", e.quat);
  v.field(25, "mesh", e.mesh);
  v.field(30, "user", e.user);
}
template <class V>
void MVisit(const MGeom& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(1, "dclass", e.dclass);
  v.field(2, "type", e.type);
  v.field(8, "size", e.size);
  v.field(9, "material", e.material);
  v.field(22, "pos", e.pos);
  v.field(23, "quat", e.quat);
  v.field(25, "mesh", e.mesh);
  v.field(30, "user", e.user);
}

template <class V>
void MVisit(MJoint& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(1, "dclass", e.dclass);
  v.field(2, "type", e.type);
  v.field(5, "axis", e.axis);
}
template <class V>
void MVisit(const MJoint& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(1, "dclass", e.dclass);
  v.field(2, "type", e.type);
  v.field(5, "axis", e.axis);
}

template <class V>
void MVisit(MDefault& e, V& v) {
  v.field(0, "dclass", e.NameSlot());
}
template <class V>
void MVisit(const MDefault& e, V& v) {
  v.field(0, "dclass", e.NameSlot());
}

template <class V>
void MVisit(MSensorPlugin& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(4, "objtype", e.objtype);
  v.field(5, "objname", e.objname);
}
template <class V>
void MVisit(const MSensorPlugin& e, V& v) {
  v.field(0, "name", e.NameSlot());
  v.field(4, "objtype", e.objtype);
  v.field(5, "objname", e.objname);
}

// --- Schema identity ------------------------------------------------------- //
// The map between a mock element type and its schema ElementType, both ways.
// Free functions rather than members so the slot tables below can use them
// while the profile tag is still incomplete.

template <class E>
constexpr ElementType MTypeOf() {
  using X = std::remove_const_t<E>;
  if constexpr (std::is_same_v<X, MModel>) return ElementType::Model;
  else if constexpr (std::is_same_v<X, MAsset>) return ElementType::Asset;
  else if constexpr (std::is_same_v<X, MSensorSection>) return ElementType::Sensor;
  else if constexpr (std::is_same_v<X, MMesh>) return ElementType::Mesh;
  else if constexpr (std::is_same_v<X, MMaterial>) return ElementType::Material;
  else if constexpr (std::is_same_v<X, MLayer>) return ElementType::MaterialLayer;
  else if constexpr (std::is_same_v<X, MBody>) return ElementType::Body;
  else if constexpr (std::is_same_v<X, MGeom>) return ElementType::Geom;
  else if constexpr (std::is_same_v<X, MJoint>) return ElementType::Joint;
  else if constexpr (std::is_same_v<X, MDefault>) return ElementType::Default;
  else if constexpr (std::is_same_v<X, MSensorPlugin>) return ElementType::SensorPlugin;
  else return ElementType::Model;
}

template <ElementType E>
struct MElementOf {
  using type = MModel;
};
template <> struct MElementOf<ElementType::Asset> { using type = MAsset; };
template <> struct MElementOf<ElementType::Sensor> { using type = MSensorSection; };
template <> struct MElementOf<ElementType::Mesh> { using type = MMesh; };
template <> struct MElementOf<ElementType::Material> { using type = MMaterial; };
template <> struct MElementOf<ElementType::MaterialLayer> { using type = MLayer; };
template <> struct MElementOf<ElementType::Body> { using type = MBody; };
template <> struct MElementOf<ElementType::Geom> { using type = MGeom; };
template <> struct MElementOf<ElementType::Joint> { using type = MJoint; };
template <> struct MElementOf<ElementType::Default> { using type = MDefault; };
template <> struct MElementOf<ElementType::SensorPlugin> { using type = MSensorPlugin; };

// --- Child slots ----------------------------------------------------------- //
// The storage slots each element admits, as (schema child index, child type).
// The index is the schema's child-list index, which is what lets the reader and
// writer recover the contextual XML tag without knowing the storage.

template <class E, class Fn>
void MSlots(Fn& fn);

template <class Fn>
void MSlots_(const MModel*, Fn& fn) {
  fn(5, sdk::TypeTag<MDefault>{});
  fn(7, sdk::TypeTag<MAsset>{});
  fn(8, sdk::TypeTag<MBody>{});
  fn(14, sdk::TypeTag<MSensorSection>{});
}
template <class Fn>
void MSlots_(const MAsset*, Fn& fn) {
  fn(0, sdk::TypeTag<MMesh>{});
  fn(4, sdk::TypeTag<MMaterial>{});
}
template <class Fn>
void MSlots_(const MMaterial*, Fn& fn) {
  fn(0, sdk::TypeTag<MLayer>{});
}
template <class Fn>
void MSlots_(const MBody*, Fn& fn) {
  fn(0, sdk::TypeTag<MBody>{});
  fn(0, sdk::TypeTag<MGeom>{});
  fn(0, sdk::TypeTag<MJoint>{});
}
template <class Fn>
void MSlots_(const MDefault*, Fn& fn) {
  fn(0, sdk::TypeTag<MDefault>{});
  fn(4, sdk::TypeTag<MGeom>{});
}
template <class Fn>
void MSlots_(const MSensorSection*, Fn& fn) {
  fn(0, sdk::TypeTag<MSensorPlugin>{});
}
template <class Fn>
void MSlots_(const MMesh*, Fn&) {}
template <class Fn>
void MSlots_(const MLayer*, Fn&) {}
template <class Fn>
void MSlots_(const MGeom*, Fn&) {}
template <class Fn>
void MSlots_(const MJoint*, Fn&) {}
template <class Fn>
void MSlots_(const MSensorPlugin*, Fn&) {}

// The slot a child of type `child` occupies under a parent of type `parent`, or
// -1. Derived from the slot tables above, so it cannot disagree with them.
inline int SlotFor(ElementType parent, ElementType child) {
  int found = -1;
  auto probe = [&](int slot, auto tag) {
    using T = typename decltype(tag)::type;
    if (MTypeOf<T>() == child && found < 0) found = slot;
  };
  switch (parent) {
    case ElementType::Model: MSlots_(static_cast<const MModel*>(nullptr), probe); break;
    case ElementType::Asset: MSlots_(static_cast<const MAsset*>(nullptr), probe); break;
    case ElementType::Material: MSlots_(static_cast<const MMaterial*>(nullptr), probe); break;
    case ElementType::Body: MSlots_(static_cast<const MBody*>(nullptr), probe); break;
    case ElementType::Default: MSlots_(static_cast<const MDefault*>(nullptr), probe); break;
    case ElementType::Sensor: MSlots_(static_cast<const MSensorSection*>(nullptr), probe); break;
    default: break;
  }
  return found;
}

// Recover a node's concrete type. The eleven-way switch is this profile's
// business, not the algorithms'.
template <class Fn>
void MDispatch(MNode& n, Fn&& fn) {
  switch (n.Type()) {
    case ElementType::Model: fn(static_cast<MModel&>(n)); break;
    case ElementType::Asset: fn(static_cast<MAsset&>(n)); break;
    case ElementType::Sensor: fn(static_cast<MSensorSection&>(n)); break;
    case ElementType::Mesh: fn(static_cast<MMesh&>(n)); break;
    case ElementType::Material: fn(static_cast<MMaterial&>(n)); break;
    case ElementType::MaterialLayer: fn(static_cast<MLayer&>(n)); break;
    case ElementType::Body: fn(static_cast<MBody&>(n)); break;
    case ElementType::Geom: fn(static_cast<MGeom&>(n)); break;
    case ElementType::Joint: fn(static_cast<MJoint&>(n)); break;
    case ElementType::Default: fn(static_cast<MDefault&>(n)); break;
    case ElementType::SensorPlugin: fn(static_cast<MSensorPlugin&>(n)); break;
    default: break;
  }
}

template <class Fn>
void MDispatch(const MNode& n, Fn&& fn) {
  MDispatch(const_cast<MNode&>(n),
            [&](auto& e) { fn(static_cast<const std::decay_t<decltype(e)>&>(e)); });
}

// Children in DOCUMENT order: (slot, sibling index). Storage order is the
// reverse of insertion, so anything that trusted `Kids()` order is wrong here.
inline std::vector<MNode*> OrderedKids(const MNode& n) {
  std::vector<MNode*> out(n.Kids().begin(), n.Kids().end());
  std::stable_sort(out.begin(), out.end(), [](const MNode* a, const MNode* b) {
    if (a->Slot() != b->Slot()) return a->Slot() < b->Slot();
    return a->SiblingIndex() < b->SiblingIndex();
  });
  return out;
}

// --- Ident ----------------------------------------------------------------- //

MNode* MCloneNode(const MNode& src);

struct MIdent {
  using serial_t = std::uint64_t;

  template <class E>
  static serial_t Serial(const E& e) {
    return e.Id();
  }
  template <class E>
  static void SetSerial(E& e, serial_t s) {
    e.SetId(s);
  }
  template <class E>
  static ps::SourceLoc Loc(const E& e) {
    return e.Where();
  }
  template <class E>
  static void SetLoc(E& e, ps::SourceLoc l) {
    e.SetWhere(std::move(l));
  }
  template <class E>
  static E* New() {
    return NewNode<E>();
  }
  // Deep copy with fresh ids, reproducing the source's document order under
  // ForEachChild -- the ordering guarantee the profile owes.
  template <class E>
  static E* Clone(const E& e) {
    return static_cast<E*>(MCloneNode(e));
  }
};

// --- Shape ----------------------------------------------------------------- //

namespace shape_detail {

template <class T>
struct is_mopt : std::false_type {};
template <class T>
struct is_mopt<MOpt<T>> : std::true_type {
  using inner = T;
};

template <class T, bool = is_mopt<T>::value>
struct inner_of {
  using type = T;
};
template <class T>
struct inner_of<T, true> {
  using type = typename is_mopt<T>::inner;
};

template <class T>
struct is_fixed_list : std::false_type {};
template <class T, std::size_t N>
struct is_fixed_list<MFixedList<T, N>> : std::true_type {};

template <class T>
struct is_list : std::false_type {};
template <class T>
struct is_list<MList<T>> : std::true_type {};

template <class I>
constexpr sdk::Shape KindOf() {
  if constexpr (std::is_same_v<I, MRef>) {
    return sdk::Shape::Ref;
  } else if constexpr (std::is_same_v<I, MStr>) {
    return sdk::Shape::Str;
  } else if constexpr (std::is_same_v<I, bool>) {
    return sdk::Shape::Bool;
  } else if constexpr (std::is_enum_v<I>) {
    return sdk::Shape::Enum;
  } else if constexpr (std::is_same_v<I, MVec3> || std::is_same_v<I, MQuat>) {
    return sdk::Shape::Fixed;
  } else if constexpr (is_fixed_list<I>::value) {
    return sdk::Shape::Range;
  } else if constexpr (is_list<I>::value) {
    return sdk::Shape::Unbounded;
  } else if constexpr (std::is_arithmetic_v<I>) {
    return sdk::Shape::Scalar;
  } else {
    return sdk::Shape::Unknown;
  }
}

}  // namespace shape_detail

struct MShape {
  template <class T>
  static constexpr bool optional_v = shape_detail::is_mopt<std::decay_t<T>>::value;

  template <class T>
  using inner_t = typename shape_detail::inner_of<std::decay_t<T>>::type;

  template <class T>
  static bool IsSet(const T& slot) {
    if constexpr (optional_v<T>) {
      return slot.IsSet();
    } else {
      (void)slot;
      return true;
    }
  }
  template <class T>
  static const inner_t<T>& Read(const T& slot) {
    if constexpr (optional_v<T>) {
      return slot.Ref();
    } else {
      return slot;
    }
  }
  template <class T>
  static void Author(T& slot, inner_t<T> v) {
    if constexpr (optional_v<T>) {
      slot.Put(std::move(v));
    } else {
      slot = std::move(v);
    }
  }
  template <class T>
  static void Reset(T& slot) {
    if constexpr (optional_v<T>) slot.Reset();
  }

  template <class I>
  static constexpr sdk::Shape kind_v = shape_detail::KindOf<std::decay_t<I>>();

  template <class I>
  static std::string_view StrGet(const I& v) {
    return std::string_view(v.Chars(), v.Len());
  }
  template <class I>
  static I StrMake(std::string_view v) {
    return I(v);
  }
  template <class I>
  static std::string_view RefGet(const I& v) {
    return std::string_view(v.name_.Chars(), v.name_.Len());
  }
  template <class I>
  static I RefMake(std::string_view v) {
    I r;
    r.name_ = MStr(v);
    return r;
  }

  template <class I>
  static I EnumMake(int index) {
    return static_cast<I>(index);
  }
  template <class I>
  static int EnumIndex(const I& v) {
    return static_cast<int>(v);
  }
  template <class I>
  static bool EnumFromMjcf(std::string_view kw, I& out) {
    return mj::FromMjcf(kw, out);
  }
  template <class I>
  static std::string_view EnumToMjcf(const I& v) {
    return mj::ToMjcf(v);
  }

  // Whole-array transfer in MJCF component order, per type. MQuat proves the
  // point: its storage is X,Y,Z,W and the wire is [w,x,y,z].
  template <class I>
  struct fixed;

  template <class I>
  struct seq;

  template <class I>
  struct reflist;
};

template <>
struct MShape::fixed<MVec3> {
  using scalar = double;
  static constexpr std::size_t size = 3;
  static void Load(const MVec3& v, double* out) {
    out[0] = v.a;
    out[1] = v.b;
    out[2] = v.c;
  }
  static MVec3 Make(const double* in) { return MVec3{in[0], in[1], in[2]}; }
};

template <>
struct MShape::fixed<MQuat> {
  using scalar = double;
  static constexpr std::size_t size = 4;
  static void Load(const MQuat& v, double* out) {
    out[0] = v.W();
    out[1] = v.X();
    out[2] = v.Y();
    out[3] = v.Z();
  }
  static MQuat Make(const double* in) {
    return MQuat(in[1], in[2], in[3], in[0]);
  }
};

template <class T, std::size_t N>
struct MShape::seq<MFixedList<T, N>> {
  using scalar = T;
  static constexpr std::size_t max_size = N;
  static std::size_t Size(const MFixedList<T, N>& v) { return v.Count(); }
  static void Load(const MFixedList<T, N>& v, T* out) {
    for (std::size_t i = 0; i < v.Count(); ++i) out[i] = v.Item(i);
  }
  static MFixedList<T, N> Make(const T* in, std::size_t n) {
    MFixedList<T, N> out;
    out.Grow(n);
    for (std::size_t i = 0; i < n; ++i) out.Item(i) = in[i];
    return out;
  }
};

template <class T>
struct MShape::seq<MList<T>> {
  using scalar = T;
  static constexpr std::size_t max_size = 0;
  static std::size_t Size(const MList<T>& v) { return v.Count(); }
  static void Load(const MList<T>& v, T* out) {
    for (std::size_t i = 0; i < v.Count(); ++i) out[i] = v.Item(i);
  }
  static MList<T> Make(const T* in, std::size_t n) {
    MList<T> out;
    out.Grow(n);
    for (std::size_t i = 0; i < n; ++i) out.Item(i) = in[i];
    return out;
  }
};

// --- Ref policy ------------------------------------------------------------ //

struct MRefPolicy {
  using slot = sdk::RefSlot<MView>;

  static const slot::Ops* RefOps() {
    static const slot::Ops ops{
        [](void* o) {
          auto* s = static_cast<MOpt<MRef>*>(o);
          return s->IsSet() && s->Ref().name_.Len() != 0;
        },
        [](void* o) -> MView {
          auto* s = static_cast<MOpt<MRef>*>(o);
          return s->IsSet() ? MView(s->Ref().name_) : MView();
        },
        [](void* o, MView v) {
          auto* s = static_cast<MOpt<MRef>*>(o);
          if (v.Len() == 0) {
            s->Reset();
          } else {
            MRef r;
            r.name_ = MStr(Utf8(v));
            s->Put(r);
          }
        },
        [](void* o) { static_cast<MOpt<MRef>*>(o)->Reset(); },
    };
    return &ops;
  }

  static const slot::Ops* StrOps() {
    static const slot::Ops ops{
        [](void* o) {
          auto* s = static_cast<MOpt<MStr>*>(o);
          return s->IsSet() && s->Ref().Len() != 0;
        },
        [](void* o) -> MView {
          auto* s = static_cast<MOpt<MStr>*>(o);
          return s->IsSet() ? MView(s->Ref()) : MView();
        },
        [](void* o, MView v) {
          auto* s = static_cast<MOpt<MStr>*>(o);
          if (v.Len() == 0)
            s->Reset();
          else
            s->Put(MStr(Utf8(v)));
        },
        [](void* o) { static_cast<MOpt<MStr>*>(o)->Reset(); },
    };
    return &ops;
  }

  // The visitor that finds a ref field by id, or scans them all. Every ref
  // reaches the caller as a slot handed over by value.
  template <class Fn>
  struct RefScan {
    Fn* on;
    ElementType type;
    template <class U>
    void field(int id, const char* name, U& v) {
      if constexpr (std::is_same_v<std::decay_t<U>, MOpt<MRef>>) {
        static_assert(!std::is_const_v<U>,
                      "the reference scan hands out live slots and needs a "
                      "mutable element");
        if (v.IsSet() && v.Ref().name_.Len() != 0) {
          (*on)(id, name, slot{&v, RefOps()},
                sdk::detail::RefTargetsAt(type, id));
        }
      }
    }
    template <class C>
    void child(int, const char*, C&) {}
    template <class C>
    void union_child(int, const char*, C&) {}
  };

  struct RefGrab {
    int target;
    void* out = nullptr;
    template <class U>
    void field(int id, const char*, U& v) {
      if (id != target) return;
      if constexpr (std::is_same_v<std::decay_t<U>, MOpt<MRef>> &&
                    !std::is_const_v<U>) {
        out = &v;
      }
    }
    template <class C>
    void child(int, const char*, C&) {}
    template <class C>
    void union_child(int, const char*, C&) {}
  };

  struct ConstRefName {
    int target;
    MView out;
    template <class U>
    void field(int id, const char*, const U& v) {
      if (id != target) return;
      if constexpr (std::is_same_v<std::decay_t<U>, MOpt<MRef>>) {
        if (v.IsSet()) out = MView(v.Ref().name_);
      }
    }
    template <class C>
    void child(int, const char*, const C&) {}
    template <class C>
    void union_child(int, const char*, const C&) {}
  };

  struct StrGrab {
    int target;
    void* out = nullptr;
    template <class U>
    void field(int id, const char*, U& v) {
      if (id != target) return;
      if constexpr (std::is_same_v<std::decay_t<U>, MOpt<MStr>> &&
                    !std::is_const_v<U>) {
        out = &v;
      }
    }
    template <class C>
    void child(int, const char*, C&) {}
    template <class C>
    void union_child(int, const char*, C&) {}
  };

  template <class E, class OnRef>
  static void ScanTyped(E& e, OnRef&& on) {
    RefScan<std::remove_reference_t<OnRef>> v{&on, MTypeOf<E>()};
    MVisit(e, v);
  }

  template <class E>
  static slot SlotAt(E& e, int field_id) {
    if (field_id < 0) return slot{};
    RefGrab g{field_id};
    MVisit(e, g);
    return g.out != nullptr ? slot{g.out, RefOps()} : slot{};
  }

  template <class E>
  static MView NameAt(const E& e, int field_id) {
    if (field_id < 0) return MView();
    ConstRefName g{field_id, MView()};
    MVisit(e, g);
    return g.out;
  }

  template <class E>
  static slot DynSlot(E& e, int field_id) {
    if (field_id < 0) return slot{};
    StrGrab g{field_id};
    MVisit(e, g);
    return g.out != nullptr ? slot{g.out, StrOps()} : slot{};
  }

  template <class Pred>
  struct Clearer {
    Pred* pred;
    ElementType type;
    template <class U>
    void field(int id, const char*, U& v) {
      if constexpr (std::is_same_v<std::decay_t<U>, MOpt<MRef>> &&
                    !std::is_const_v<U>) {
        if (v.IsSet() && v.Ref().name_.Len() != 0 &&
            (*pred)(MView(v.Ref().name_), sdk::detail::RefTargetsAt(type, id))) {
          v.Reset();
        }
      }
    }
    template <class C>
    void child(int, const char*, C&) {}
    template <class C>
    void union_child(int, const char*, C&) {}
  };

  template <class E, class Pred>
  static void ClearTypedIf(E& e, Pred&& pred) {
    Clearer<std::remove_reference_t<Pred>> c{&pred, MTypeOf<E>()};
    MVisit(e, c);
  }
};

// --- Tree ------------------------------------------------------------------ //

struct MTree {
  template <class T>
  using owner = T*;

  template <class E, class Fn>
  static void ForEachChild(E& parent, Fn&& fn) {
    for (MNode* k : OrderedKids(parent)) {
      if constexpr (std::is_const_v<E>) {
        MDispatch(*static_cast<const MNode*>(k), [&](const auto& c) { fn(c); });
      } else {
        MDispatch(*k, [&](auto& c) { fn(c); });
      }
    }
  }

  template <class E, class Fn>
  static void ForEachChildAt(E& parent, Fn&& fn) {
    for (MNode* k : OrderedKids(parent)) {
      const int slot = k->Slot();
      if constexpr (std::is_const_v<E>) {
        MDispatch(*static_cast<const MNode*>(k),
                  [&](const auto& c) { fn(slot, c); });
      } else {
        MDispatch(*k, [&](auto& c) { fn(slot, c); });
      }
    }
  }

  template <class E, class Fn>
  static void ForEachChildSlot(E&, Fn&& fn) {
    MSlots_(static_cast<const std::remove_const_t<E>*>(nullptr), fn);
  }

  template <class T, class E, class Fn>
  static void ForEachChildOfType(E& parent, Fn&& fn) {
    const ElementType want = MTypeOf<T>();
    for (MNode* k : OrderedKids(parent)) {
      if (k->Type() != want) continue;
      if constexpr (std::is_const_v<E>) {
        fn(static_cast<const T&>(*k));
      } else {
        fn(static_cast<T&>(*k));
      }
    }
  }

  template <class T, class E>
  static auto FirstChildOfType(E& parent)
      -> std::conditional_t<std::is_const_v<E>, const T*, T*> {
    const ElementType want = MTypeOf<T>();
    for (MNode* k : OrderedKids(parent))
      if (k->Type() == want) return static_cast<T*>(k);
    return nullptr;
  }

  template <class T, class E>
  static void ClearChildrenOfType(E& parent) {
    const ElementType want = MTypeOf<T>();
    auto& kids = parent.Kids();
    kids.erase(std::remove_if(kids.begin(), kids.end(),
                              [&](MNode* k) { return k->Type() == want; }),
               kids.end());
  }

  // Link an owned child at `index` among the siblings sharing its slot; storage
  // order stays the reverse of insertion, so only the sibling numbering carries
  // document order.
  template <class T, class E>
  static T& Adopt(E& parent, std::size_t index, owner<T> child) {
    const int slot = SlotFor(parent.Type(), MTypeOf<T>());
    std::vector<MNode*> peers;
    for (MNode* k : OrderedKids(parent))
      if (k->Slot() == slot) peers.push_back(k);
    const std::size_t at = index >= peers.size() ? peers.size() : index;
    peers.insert(peers.begin() + static_cast<std::ptrdiff_t>(at), child);
    child->Link(&parent, slot, 0);
    for (std::size_t i = 0; i < peers.size(); ++i)
      peers[i]->SetSibling(static_cast<int>(i));
    parent.Kids().insert(parent.Kids().begin(), child);  // reverse storage order
    return *child;
  }

  template <class Root>
  static bool Remove(Root& root, const void* target) {
    bool done = false;
    Walk(root, [&](MNode& n) {
      if (done) return;
      auto& kids = n.Kids();
      for (auto it = kids.begin(); it != kids.end(); ++it) {
        if (static_cast<const void*>(*it) == target) {
          kids.erase(it);
          done = true;
          return;
        }
      }
    });
    return done;
  }

  template <class Root>
  static void* CloneAsNextSibling(Root& root, const void* target) {
    MNode* found = nullptr;
    Walk(root, [&](MNode& n) {
      if (!found && static_cast<const void*>(&n) == target) found = &n;
    });
    if (found == nullptr || found->Parent() == nullptr) return nullptr;
    MNode* parent = found->Parent();
    MNode* clone = MCloneNode(*found);
    const int slot = found->Slot();
    std::vector<MNode*> peers;
    for (MNode* k : OrderedKids(*parent))
      if (k->Slot() == slot) peers.push_back(k);
    std::size_t at = peers.size();
    for (std::size_t i = 0; i < peers.size(); ++i)
      if (peers[i] == found) at = i + 1;
    peers.insert(peers.begin() + static_cast<std::ptrdiff_t>(at), clone);
    clone->Link(parent, slot, 0);
    for (std::size_t i = 0; i < peers.size(); ++i)
      peers[i]->SetSibling(static_cast<int>(i));
    parent->Kids().insert(parent->Kids().begin(), clone);
    return clone;
  }

  template <class Root, class Pred>
  static void PruneIf(Root& root, Pred&& pred) {
    Walk(root, [&](MNode& n) {
      auto& kids = n.Kids();
      kids.erase(std::remove_if(kids.begin(), kids.end(),
                                [&](MNode* k) {
                                  bool rm = false;
                                  MDispatch(*k, [&](auto& c) { rm = pred(c); });
                                  return rm;
                                }),
                 kids.end());
    });
  }

  // Movable: any child of a body's interleaved slot. Target: a body, or null
  // for the world body.
  template <class Root>
  static sdk::MoveStatus Reparent(Root& root, const void* elem,
                                  const void* new_parent) {
    MNode* node = nullptr;
    MNode* target = nullptr;
    Walk(root, [&](MNode& n) {
      if (static_cast<const void*>(&n) == elem) node = &n;
      if (new_parent != nullptr && static_cast<const void*>(&n) == new_parent)
        target = &n;
    });
    if (node == nullptr || node->Parent() == nullptr ||
        node->Parent()->Type() != ElementType::Body) {
      return sdk::MoveStatus::NotMovable;
    }
    if (new_parent == nullptr) {
      target = FirstChildOfType<MBody>(root);
      if (target == nullptr) return sdk::MoveStatus::BadTarget;
    } else if (target == nullptr || target->Type() != ElementType::Body) {
      return sdk::MoveStatus::BadTarget;
    }
    bool cycle = false;
    Walk(*node, [&](MNode& n) {
      if (&n == target) cycle = true;
    });
    if (cycle) return sdk::MoveStatus::Cycle;

    auto& src = node->Parent()->Kids();
    src.erase(std::remove(src.begin(), src.end(), node), src.end());
    MDispatch(*node, [&](auto& c) {
      using T = std::decay_t<decltype(c)>;
      Adopt<T>(static_cast<MBody&>(*target), sdk::kAppend, &c);
    });
    return sdk::MoveStatus::Ok;
  }

 private:
  template <class E, class Fn>
  static void Walk(E& n, Fn&& fn) {
    fn(static_cast<MNode&>(n));
    for (MNode* k : OrderedKids(n)) Walk(*k, fn);
  }
};

// --- Doc ------------------------------------------------------------------- //

struct MDoc {
  using doc_type = MModel;
  using root_type = MModel;
  using node_ptr = void*;

  template <class E>
  static constexpr bool is_root = std::is_same_v<std::remove_const_t<E>, MModel>;

  template <class D, class Fn>
  static void ForEachLiveSection(D& doc, Fn&& fn) {
    MTree::ForEachChild(doc, [&](auto& section) {
      if constexpr (!std::is_same_v<std::decay_t<decltype(section)>, MDefault>)
        fn(section);
    });
  }

  static std::optional<MView> Name(const MModel& m) {
    if (!m.model.IsSet()) return std::nullopt;
    return MView(m.model.Ref());
  }
};

// --- The profile ----------------------------------------------------------- //

struct Mock {
  using Doc = MDoc;
  using Str = MStrPolicy;
  using Tree = MTree;
  using Ref = MRefPolicy;
  using Ident = MIdent;
  using Shape = MShape;

  template <class E, class V>
  static void Visit(E& e, V&& v) {
    MVisit(e, v);
  }

  template <class E>
  static constexpr ElementType type_of = MTypeOf<E>();

  template <ElementType E>
  using element_t = typename MElementOf<E>::type;

  template <class E>
  static constexpr bool has_name = true;

  template <class E>
  static std::optional<MView> Name(const E& e) {
    if (!e.NameSlot().IsSet()) return std::nullopt;
    return MView(e.NameSlot().Ref());
  }

  template <class E>
  static void SetName(E& e, MView name) {
    e.SetNameFrom(name);
  }

  // The mock authors nothing implicitly: its documents carry exactly what was
  // read or set, so the lowest-priority defaults layer is empty.
  template <class E>
  static void ApplyDefault(E&) {}

  template <class E>
  static const E& Defaults() {
    static const E defs{};
    return defs;
  }
};

// A deep copy with fresh ids that reproduces the source's document order.
inline MNode* MCloneNode(const MNode& src) {
  MNode* out = nullptr;
  MDispatch(src, [&](const auto& c) {
    using T = std::decay_t<decltype(c)>;
    T* dst = NewNode<T>();
    const std::uint64_t id = dst->Id();
    *dst = c;        // copies fields, name, and the raw child pointer vector
    dst->SetId(id);  // ... but identity is minted fresh
    dst->Kids().clear();
    dst->Link(nullptr, -1, 0);
    for (MNode* k : OrderedKids(c)) {
      MNode* kc = MCloneNode(*k);
      MDispatch(*kc, [&](auto& kk) {
        using K = std::decay_t<decltype(kk)>;
        MTree::Adopt<K>(*dst, sdk::kAppend, &kk);
      });
    }
    out = dst;
  });
  return out;
}

}  // namespace mock

#endif  // PROTOSPEC_TEST_MOCK_PROFILE_H
