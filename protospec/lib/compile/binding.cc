// Binding implementation: the element -> mjtObj trait table and the id maps.
// This translation unit is inside the MuJoCo quarantine zone (it includes
// mujoco.h for mjOBJ_* and mjModel address fields).

#include "binding.h"

#include <cstdint>
#include <string>
#include <variant>

#include <mujoco/mujoco.h>

#include "reflect.h"

namespace ps::mjcf {
namespace {

// Structural (non-union) families: element type -> mjtObj. Union members are
// filled from the reflect UnionDescriptor tables so the ~50 sensor spellings and
// the actuator/equality/tendon variants need no manual list here.
struct FamilyRow {
  ElementType type;
  int objtype;
};

constexpr FamilyRow kStructural[] = {
    {ElementType::Body, mjOBJ_BODY},        {ElementType::Joint, mjOBJ_JOINT},
    {ElementType::FreeJoint, mjOBJ_JOINT},  {ElementType::Geom, mjOBJ_GEOM},
    {ElementType::Site, mjOBJ_SITE},        {ElementType::Camera, mjOBJ_CAMERA},
    {ElementType::Light, mjOBJ_LIGHT},      {ElementType::Mesh, mjOBJ_MESH},
    {ElementType::Hfield, mjOBJ_HFIELD},    {ElementType::Skin, mjOBJ_SKIN},
    {ElementType::Texture, mjOBJ_TEXTURE},  {ElementType::Material, mjOBJ_MATERIAL},
    {ElementType::Flex, mjOBJ_FLEX},        {ElementType::Pair, mjOBJ_PAIR},
    {ElementType::Exclude, mjOBJ_EXCLUDE},  {ElementType::Numeric, mjOBJ_NUMERIC},
    {ElementType::Text, mjOBJ_TEXT},        {ElementType::Tuple, mjOBJ_TUPLE},
    {ElementType::Key, mjOBJ_KEY},          {ElementType::PluginInstance, mjOBJ_PLUGIN},
};

struct UnionRow {
  const char* union_name;
  int objtype;
};

constexpr UnionRow kUnions[] = {
    {"ActuatorAny", mjOBJ_ACTUATOR},
    {"SensorAny", mjOBJ_SENSOR},
    {"TendonAny", mjOBJ_TENDON},
    {"EqualityAny", mjOBJ_EQUALITY},
};

// The trait table, built once. Index by ElementType; value is the mjtObj, or
// mjOBJ_UNKNOWN for families that do not compile to a bindable object. Sized
// from reflect::ElementCount() so it tracks the schema without a hardcoded max.
const std::vector<int>& ObjTypeTable() {
  static const std::vector<int> table = [] {
    std::vector<int> t(reflect::ElementCount(), mjOBJ_UNKNOWN);
    for (const FamilyRow& r : kStructural) {
      t[static_cast<std::size_t>(r.type)] = r.objtype;
    }
    for (const UnionRow& u : kUnions) {
      const reflect::UnionDescriptor& ud = reflect::DescribeUnion(u.union_name);
      for (std::size_t i = 0; i < ud.member_count; ++i) {
        t[static_cast<std::size_t>(ud.members[i])] = u.objtype;
      }
    }
    return t;
  }();
  return table;
}

std::int64_t IdKey(int objtype, int id) {
  return (static_cast<std::int64_t>(objtype) << 32) |
         (static_cast<std::int64_t>(static_cast<std::uint32_t>(id)));
}

// A tiny '*'/'?' glob matcher (case-sensitive), used by Binding::Find.
bool GlobMatch(std::string_view pat, std::string_view s) {
  std::size_t p = 0, t = 0, star = std::string_view::npos, mark = 0;
  while (t < s.size()) {
    if (p < pat.size() && (pat[p] == '?' || pat[p] == s[t])) {
      ++p;
      ++t;
    } else if (p < pat.size() && pat[p] == '*') {
      star = p++;
      mark = t;
    } else if (star != std::string_view::npos) {
      p = star + 1;
      t = ++mark;
    } else {
      return false;
    }
  }
  while (p < pat.size() && pat[p] == '*') ++p;
  return p == pat.size();
}

// Extract the concrete held pointer from a union wrapper.
const void* ConcretePtr(const ActuatorAny& a) {
  const void* out = nullptr;
  std::visit([&](const auto& p) { out = p.get(); }, a.node);
  return out;
}
const void* ConcretePtr(const SensorAny& s) {
  const void* out = nullptr;
  std::visit([&](const auto& p) { out = p.get(); }, s.node);
  return out;
}

}  // namespace

int ObjTypeOf(ElementType type) {
  const std::size_t i = static_cast<std::size_t>(type);
  const auto& t = ObjTypeTable();
  return i < t.size() ? t[i] : mjOBJ_UNKNOWN;
}

std::string_view FamilyToken(ElementType type) {
  switch (ObjTypeOf(type)) {
    case mjOBJ_BODY: return "body";
    case mjOBJ_JOINT: return "joint";
    case mjOBJ_GEOM: return "geom";
    case mjOBJ_SITE: return "site";
    case mjOBJ_CAMERA: return "cam";
    case mjOBJ_LIGHT: return "light";
    case mjOBJ_MESH: return "mesh";
    case mjOBJ_HFIELD: return "hfield";
    case mjOBJ_SKIN: return "skin";
    case mjOBJ_TEXTURE: return "tex";
    case mjOBJ_MATERIAL: return "mat";
    case mjOBJ_FLEX: return "flex";
    case mjOBJ_PAIR: return "pair";
    case mjOBJ_EXCLUDE: return "exclude";
    case mjOBJ_EQUALITY: return "eq";
    case mjOBJ_TENDON: return "tendon";
    case mjOBJ_ACTUATOR: return "act";
    case mjOBJ_SENSOR: return "sensor";
    case mjOBJ_NUMERIC: return "numeric";
    case mjOBJ_TEXT: return "text";
    case mjOBJ_TUPLE: return "tuple";
    case mjOBJ_KEY: return "key";
    case mjOBJ_PLUGIN: return "plugin";
    default: return "";
  }
}

// --- Builder (declared in binding.h; used by compile.cc) ------------------ //
namespace detail {

void BindingBuilder::SetContext(const Model* model, const mjModel* m) {
  b_.model_ = model;
  b_.m_ = m;
}

void BindingBuilder::Add(const Binding::Entry& e) {
  const std::size_t idx = b_.entries_.size();
  b_.entries_.push_back(e);
  b_.by_ptr_.emplace(e.elem, idx);
  if (e.id >= 0) b_.by_id_.emplace(IdKey(e.objtype, e.id), idx);
}

}  // namespace detail

// --- Binding accessors ---------------------------------------------------- //

std::optional<int> Binding::IdByPtr(const void* elem) const {
  auto it = by_ptr_.find(elem);
  if (it == by_ptr_.end()) return std::nullopt;
  const int id = entries_[it->second].id;
  if (id < 0) return std::nullopt;
  return id;
}

const Binding::Entry* Binding::EntryAt(int objtype, int id) const {
  auto it = by_id_.find(IdKey(objtype, id));
  if (it == by_id_.end()) return nullptr;
  return &entries_[it->second];
}

const void* Binding::ElementAt(int objtype, int id) const {
  const Entry* e = EntryAt(objtype, id);
  return e ? e->elem : nullptr;
}

// Typed reverse: return the element only if the concrete type matches (the id
// slot may hold a different family member, e.g. a free joint at a joint id).
#define PS_REVERSE(FnName, MjObj, CppType, EnumType)                        \
  const CppType* Binding::FnName(int id) const {                            \
    const Entry* e = EntryAt(MjObj, id);                                    \
    if (!e || e->etype != ElementType::EnumType) return nullptr;            \
    return static_cast<const CppType*>(e->elem);                            \
  }
PS_REVERSE(BodyAt, mjOBJ_BODY, Body, Body)
PS_REVERSE(GeomAt, mjOBJ_GEOM, Geom, Geom)
PS_REVERSE(JointAt, mjOBJ_JOINT, Joint, Joint)
PS_REVERSE(SiteAt, mjOBJ_SITE, Site, Site)
PS_REVERSE(CameraAt, mjOBJ_CAMERA, Camera, Camera)
PS_REVERSE(LightAt, mjOBJ_LIGHT, Light, Light)
PS_REVERSE(MeshAt, mjOBJ_MESH, Mesh, Mesh)
#undef PS_REVERSE

std::optional<int> Binding::QposAdr(const Joint& j) const {
  auto id = Id(j);
  if (!id) return std::nullopt;
  return m_->jnt_qposadr[*id];
}

std::optional<int> Binding::DofAdr(const Joint& j) const {
  auto id = Id(j);
  if (!id) return std::nullopt;
  return m_->jnt_dofadr[*id];
}

std::optional<int> Binding::SensorAdr(const SensorAny& s) const {
  auto it = by_ptr_.find(ConcretePtr(s));
  if (it == by_ptr_.end()) return std::nullopt;
  const int id = entries_[it->second].id;
  if (id < 0) return std::nullopt;
  return m_->sensor_adr[id];
}

std::optional<int> Binding::ActId(const ActuatorAny& a) const {
  auto it = by_ptr_.find(ConcretePtr(a));
  if (it == by_ptr_.end()) return std::nullopt;
  const int id = entries_[it->second].id;
  if (id < 0) return std::nullopt;
  return id;
}

std::optional<int> Binding::ActAdr(const ActuatorAny& a) const {
  auto id = ActId(a);
  if (!id) return std::nullopt;
  return m_->actuator_actadr[*id];
}

std::vector<int> Binding::Find(int objtype, std::string_view glob) const {
  std::vector<int> out;
  if (m_ == nullptr) return out;
  // Enumerate ids of this object type and glob-match their names. MuJoCo's n*
  // counts are mjtSize; narrow explicitly (models never exceed int here).
  std::int64_t n = 0;
  switch (objtype) {
    case mjOBJ_BODY: n = m_->nbody; break;
    case mjOBJ_JOINT: n = m_->njnt; break;
    case mjOBJ_GEOM: n = m_->ngeom; break;
    case mjOBJ_SITE: n = m_->nsite; break;
    case mjOBJ_CAMERA: n = m_->ncam; break;
    case mjOBJ_LIGHT: n = m_->nlight; break;
    case mjOBJ_MESH: n = m_->nmesh; break;
    case mjOBJ_HFIELD: n = m_->nhfield; break;
    case mjOBJ_SKIN: n = m_->nskin; break;
    case mjOBJ_TEXTURE: n = m_->ntex; break;
    case mjOBJ_MATERIAL: n = m_->nmat; break;
    case mjOBJ_FLEX: n = m_->nflex; break;
    case mjOBJ_PAIR: n = m_->npair; break;
    case mjOBJ_EXCLUDE: n = m_->nexclude; break;
    case mjOBJ_EQUALITY: n = m_->neq; break;
    case mjOBJ_TENDON: n = m_->ntendon; break;
    case mjOBJ_ACTUATOR: n = m_->nu; break;
    case mjOBJ_SENSOR: n = m_->nsensor; break;
    case mjOBJ_NUMERIC: n = m_->nnumeric; break;
    case mjOBJ_TEXT: n = m_->ntext; break;
    case mjOBJ_TUPLE: n = m_->ntuple; break;
    case mjOBJ_KEY: n = m_->nkey; break;
    case mjOBJ_PLUGIN: n = m_->nplugin; break;
    default: n = 0; break;
  }
  for (int id = 0; id < static_cast<int>(n); ++id) {
    const char* name = mj_id2name(m_, objtype, id);
    if (name != nullptr && GlobMatch(glob, name)) out.push_back(id);
  }
  return out;
}

}  // namespace ps::mjcf
