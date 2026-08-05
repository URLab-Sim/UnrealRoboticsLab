#include "validate.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

#include "keywords.h"
#include "reflect.h"
#include "sizes.h"
#include "types.h"
#include "visit.h"

namespace ps::mjcf::validate {
namespace {

using reflect::ArityKind;
using reflect::Cardinality;
using reflect::ChildDescriptor;
using reflect::FieldDescriptor;
using reflect::FieldKind;

// --- MuJoCo object namespaces (name uniqueness + ref resolution) ---------- //
// A ProtoSpec element maps to the MuJoCo object type whose namespace owns its
// name. Several ProtoSpec element spellings share one namespace (all actuator
// spellings -> Actuator, all sensor spellings -> Sensor, all equality spellings
// -> Equality, Spatial/Fixed -> Tendon), matching how MuJoCo assigns and checks
// names by mjtObj. Names are unique within a namespace; empties are allowed and
// never registered (as MuJoCo).
enum class Ns {
  Body, Geom, Joint, Site, Camera, Light, Mesh, Material, Texture, Hfield,
  Skin, Flex, Pair, Exclude, Equality, Tendon, Actuator, Sensor, Numeric,
  Text, Tuple, Key, Frame, PluginInstance, ModelAsset, Class, COUNT
};

const char* NsLabel(Ns ns) {
  switch (ns) {
    case Ns::Body: return "body";
    case Ns::Geom: return "geom";
    case Ns::Joint: return "joint";
    case Ns::Site: return "site";
    case Ns::Camera: return "camera";
    case Ns::Light: return "light";
    case Ns::Mesh: return "mesh";
    case Ns::Material: return "material";
    case Ns::Texture: return "texture";
    case Ns::Hfield: return "hfield";
    case Ns::Skin: return "skin";
    case Ns::Flex: return "flex";
    case Ns::Pair: return "pair";
    case Ns::Exclude: return "exclude";
    case Ns::Equality: return "equality";
    case Ns::Tendon: return "tendon";
    case Ns::Actuator: return "actuator";
    case Ns::Sensor: return "sensor";
    case Ns::Numeric: return "numeric";
    case Ns::Text: return "text";
    case Ns::Tuple: return "tuple";
    case Ns::Key: return "key";
    case Ns::Frame: return "frame";
    case Ns::PluginInstance: return "plugin instance";
    case Ns::ModelAsset: return "model asset";
    case Ns::Class: return "default class";
    case Ns::COUNT: break;
  }
  return "?";
}

std::optional<Ns> ElemNamespace(ElementType et) {
  switch (et) {
    case ElementType::Body: return Ns::Body;
    case ElementType::Geom: return Ns::Geom;
    case ElementType::Joint:
    case ElementType::FreeJoint: return Ns::Joint;
    case ElementType::Site: return Ns::Site;
    case ElementType::Camera: return Ns::Camera;
    case ElementType::Light: return Ns::Light;
    case ElementType::Mesh: return Ns::Mesh;
    case ElementType::Material: return Ns::Material;
    case ElementType::Texture: return Ns::Texture;
    case ElementType::Hfield: return Ns::Hfield;
    case ElementType::Skin: return Ns::Skin;
    case ElementType::Flex: return Ns::Flex;
    case ElementType::Pair: return Ns::Pair;
    case ElementType::Exclude: return Ns::Exclude;
    case ElementType::Connect:
    case ElementType::Weld:
    case ElementType::EqualityJoint:
    case ElementType::EqualityTendon:
    case ElementType::EqualityFlex:
    case ElementType::Flexvert:
    case ElementType::Flexstrain: return Ns::Equality;
    case ElementType::Spatial:
    case ElementType::Fixed: return Ns::Tendon;
    case ElementType::ActuatorGeneral:
    case ElementType::Motor:
    case ElementType::Position:
    case ElementType::Velocity:
    case ElementType::IntVelocity:
    case ElementType::OrientationActuator:
    case ElementType::Pid:
    case ElementType::Damper:
    case ElementType::Cylinder:
    case ElementType::Muscle:
    case ElementType::Adhesion:
    case ElementType::DcMotor:
    case ElementType::ActuatorPlugin: return Ns::Actuator;
    case ElementType::Numeric: return Ns::Numeric;
    case ElementType::Text: return Ns::Text;
    case ElementType::Tuple: return Ns::Tuple;
    case ElementType::Key: return Ns::Key;
    case ElementType::Frame: return Ns::Frame;
    case ElementType::PluginInstance: return Ns::PluginInstance;
    case ElementType::ModelAsset: return Ns::ModelAsset;
    default: break;
  }
  // Every remaining element is a sensor spelling (SensorAny member) or an
  // unnamed container/block; only sensors carry a namespaced name.
  switch (et) {
    case ElementType::Touch: case ElementType::Accelerometer:
    case ElementType::Velocimeter: case ElementType::Gyro:
    case ElementType::Force: case ElementType::Torque:
    case ElementType::Magnetometer: case ElementType::Camprojection:
    case ElementType::Rangefinder: case ElementType::Jointpos:
    case ElementType::Jointvel: case ElementType::Tendonpos:
    case ElementType::Tendonvel: case ElementType::Actuatorpos:
    case ElementType::Actuatorvel: case ElementType::Actuatorfrc:
    case ElementType::Jointactuatorfrc: case ElementType::Tendonactuatorfrc:
    case ElementType::Ballquat: case ElementType::Ballangvel:
    case ElementType::Jointlimitpos: case ElementType::Jointlimitvel:
    case ElementType::Jointlimitfrc: case ElementType::Tendonlimitpos:
    case ElementType::Tendonlimitvel: case ElementType::Tendonlimitfrc:
    case ElementType::Framepos: case ElementType::Framequat:
    case ElementType::Framexaxis: case ElementType::Frameyaxis:
    case ElementType::Framezaxis: case ElementType::Framelinvel:
    case ElementType::Frameangvel: case ElementType::Framelinacc:
    case ElementType::Frameangacc: case ElementType::Subtreecom:
    case ElementType::Subtreelinvel: case ElementType::Subtreeangmom:
    case ElementType::Insidesite: case ElementType::Distance:
    case ElementType::Normal: case ElementType::Fromto:
    case ElementType::SensorContact: case ElementType::EPotential:
    case ElementType::EKinetic: case ElementType::Clock:
    case ElementType::Tactile: case ElementType::SensorUser:
    case ElementType::SensorPlugin: return Ns::Sensor;
    default: return std::nullopt;
  }
}

// Ref target element type -> the namespace that must contain the referent.
// Undefined primary template forces a specialization for every Ref<T> target:
// a new ref target that lands in the schema fails to compile here (a drift
// signal), never silently resolves against the wrong table.
template <class T>
struct RefNs;
template <> struct RefNs<Default> { static constexpr Ns value = Ns::Class; };
template <> struct RefNs<Joint> { static constexpr Ns value = Ns::Joint; };
template <> struct RefNs<JointAny> { static constexpr Ns value = Ns::Joint; };
template <> struct RefNs<Site> { static constexpr Ns value = Ns::Site; };
template <> struct RefNs<Geom> { static constexpr Ns value = Ns::Geom; };
template <> struct RefNs<Body> { static constexpr Ns value = Ns::Body; };
template <> struct RefNs<Frame> { static constexpr Ns value = Ns::Frame; };
template <> struct RefNs<Camera> { static constexpr Ns value = Ns::Camera; };
template <> struct RefNs<Material> { static constexpr Ns value = Ns::Material; };
template <> struct RefNs<Texture> { static constexpr Ns value = Ns::Texture; };
template <> struct RefNs<Mesh> { static constexpr Ns value = Ns::Mesh; };
template <> struct RefNs<Hfield> { static constexpr Ns value = Ns::Hfield; };
template <> struct RefNs<Flex> { static constexpr Ns value = Ns::Flex; };
template <> struct RefNs<FlexAny> { static constexpr Ns value = Ns::Flex; };
template <> struct RefNs<TendonAny> { static constexpr Ns value = Ns::Tendon; };
template <> struct RefNs<ActuatorAny> {
  static constexpr Ns value = Ns::Actuator;
};
template <> struct RefNs<PluginInstance> {
  static constexpr Ns value = Ns::PluginInstance;
};
template <> struct RefNs<ModelAsset> {
  static constexpr Ns value = Ns::ModelAsset;
};

// The two namespaces MuJoCo seeds with an implicit member: the world body and
// the top-level "main" default class. References to them resolve even when no
// explicit element declares them, so they never dangle.
bool IsImplicit(Ns ns, const std::string& name) {
  return (ns == Ns::Body && name == "world") ||
         (ns == Ns::Class && name == "main");
}

// The namespace a DYNAMIC reference resolves in, given the runtime keyword its
// sibling type field holds (mju_str2Type object names). nullopt = unknown or
// unmapped keyword; the name is then not checkable and is left alone.
std::optional<Ns> DynNs(const std::string& keyword) {
  if (keyword == "body" || keyword == "xbody") return Ns::Body;
  if (keyword == "joint") return Ns::Joint;
  if (keyword == "geom") return Ns::Geom;
  if (keyword == "site") return Ns::Site;
  if (keyword == "camera") return Ns::Camera;
  if (keyword == "light") return Ns::Light;
  if (keyword == "flex") return Ns::Flex;
  if (keyword == "mesh") return Ns::Mesh;
  if (keyword == "skin") return Ns::Skin;
  if (keyword == "hfield") return Ns::Hfield;
  if (keyword == "texture") return Ns::Texture;
  if (keyword == "material") return Ns::Material;
  if (keyword == "tendon") return Ns::Tendon;
  if (keyword == "actuator") return Ns::Actuator;
  if (keyword == "sensor") return Ns::Sensor;
  if (keyword == "numeric") return Ns::Numeric;
  if (keyword == "text") return Ns::Text;
  if (keyword == "tuple") return Ns::Tuple;
  if (keyword == "key") return Ns::Key;
  if (keyword == "plugin") return Ns::PluginInstance;
  return std::nullopt;
}

// The `opt<string>` field at reflect field id `id`, via the generated Visit
// hook (validate stays independent of the SDK's probe helpers).
template <class T>
struct StringGrab {
  int want;
  const ps::opt<std::string>* out = nullptr;
  template <class U>
  void field(int id, const char*, const U& v) {
    if constexpr (std::is_same_v<std::decay_t<U>, ps::opt<std::string>>) {
      if (id == want) out = &v;
    }
  }
  template <class C>
  void child(int, const char*, const C&) {}
  template <class C>
  void union_child(int, const char*, const C&) {}
};
template <class T>
const ps::opt<std::string>* StringFieldAt(const T& e, int id) {
  StringGrab<T> g{id};
  Visit(e, g);
  return g.out;
}

// --- Field-shape traits --------------------------------------------------- //
template <class> struct is_opt : std::false_type {};
template <class T> struct is_opt<ps::opt<T>> : std::true_type {
  using inner = T;
};
template <class> struct is_inline_vec : std::false_type {};
template <class T, std::size_t N>
struct is_inline_vec<ps::InlineVec<T, N>> : std::true_type {};
template <class> struct is_ref : std::false_type {};
template <class T> struct is_ref<ps::Ref<T>> : std::true_type {
  using target = T;
};
template <class> struct is_variant : std::false_type {};
template <class... Ts> struct is_variant<std::variant<Ts...>> : std::true_type {};
template <class> struct is_std_vector : std::false_type {};
template <class T> struct is_std_vector<std::vector<T>> : std::true_type {
  using elem = T;
};

// Member-presence detection for the tier-3 limit/user lint (no per-type code).
template <class T, class = void> struct has_name : std::false_type {};
template <class T>
struct has_name<T, std::void_t<decltype(std::declval<const T&>().name)>>
    : std::true_type {};

// A name the schema marks required is stored as a plain string, an optional one
// as ps::opt; both read the same through this, with "" for absent.
template <class T>
const std::string& NameOf(const T& e) {
  static const std::string kNone;
  if constexpr (std::is_same_v<std::decay_t<decltype(e.name)>, std::string>) {
    return e.name;
  } else {
    return e.name ? *e.name : kNone;
  }
}
template <class T, class = void> struct has_user : std::false_type {};
template <class T>
struct has_user<T, std::void_t<decltype(std::declval<const T&>().user)>>
    : std::true_type {};
template <class T, class = void> struct has_ctrl : std::false_type {};
template <class T>
struct has_ctrl<T, std::void_t<decltype(std::declval<const T&>().ctrllimited),
                               decltype(std::declval<const T&>().ctrlrange)>>
    : std::true_type {};
template <class T, class = void> struct has_force : std::false_type {};
template <class T>
struct has_force<T, std::void_t<decltype(std::declval<const T&>().forcelimited),
                                decltype(std::declval<const T&>().forcerange)>>
    : std::true_type {};
template <class T, class = void> struct has_act : std::false_type {};
template <class T>
struct has_act<T, std::void_t<decltype(std::declval<const T&>().actlimited),
                              decltype(std::declval<const T&>().actrange)>>
    : std::true_type {};

bool RangeNontrivial(const std::array<double, 2>& r) {
  return !(r[0] == 0.0 && r[1] == 0.0);
}

// --- Shared validation context -------------------------------------------- //
struct NuserLimits {
  int body = -1, jnt = -1, geom = -1, site = -1, cam = -1, tendon = -1,
      actuator = -1, sensor = -1;
};

struct Ctx {
  TierMask tiers;
  std::vector<ps::Diagnostic> out;
  std::vector<std::string> path;
  std::array<std::unordered_map<std::string, ps::SourceLoc>,
             static_cast<std::size_t>(Ns::COUNT)>
      symbols;
  ModelSizes sizes;
  bool autolimits = true;  // MuJoCo compiler default
  NuserLimits nuser;
  // Ref resolution is suppressed inside <default> templates (MuJoCo resolves
  // refs on the concrete elements a class is applied to, never on the template
  // itself) and whenever the model contains procedural macros
  // (composite/flexcomp/replicate/attach), whose referent names MuJoCo mints at
  // compile and ProtoSpec cannot know pre-compile (DR-7).
  bool in_default = false;
  bool has_macros = false;

  std::string PathStr() const {
    std::string s;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (i) s += '/';
      s += path[i];
    }
    return s;
  }

  void Emit(Tier tier, Severity sev, std::string msg, const ps::SourceLoc& loc) {
    EmitAt(tier, sev, std::move(msg), loc, PathStr());
  }
  void EmitAt(Tier tier, Severity sev, std::string msg, const ps::SourceLoc& loc,
              std::string path_override) {
    ps::Diagnostic d;
    d.severity = sev;
    d.source = "validate";
    d.message = std::move(msg);
    d.loc = loc;
    d.tag = std::move(path_override);
    d.tier = tier;
    out.push_back(std::move(d));
  }

  bool Resolvable(Ns ns, const std::string& name) const {
    return symbols[static_cast<std::size_t>(ns)].count(name) != 0 ||
           IsImplicit(ns, name);
  }
};

// --- Generic tree walk ---------------------------------------------------- //
template <class T>
std::string MakeSeg(ElementType et, const T& e, int idx) {
  std::string tag(reflect::Describe(et).xml);
  std::string name;
  if constexpr (has_name<T>::value) {
    name = NameOf(e);
  } else if constexpr (std::is_same_v<T, Default>) {
    if (e.dclass) name = *e.dclass;
  }
  if (name.empty()) return tag + "[#" + std::to_string(idx) + "]";
  return tag + "[" + name + "]";
}

template <class Pass, class T>
void Walk(Pass& pass, const T& e, int idx);

template <class Pass>
struct ChildRecurser {
  Pass& pass;
  template <class U>
  void field(int, const char*, const U&) {}
  template <class U>
  void child(int, const char*, const std::vector<std::unique_ptr<U>>& list) {
    int i = 0;
    for (const auto& u : list) {
      if (u) Walk(pass, *u, i);
      ++i;
    }
  }
  template <class U>
  void union_child(int, const char*, const std::vector<U>& list) {
    int i = 0;
    for (const auto& item : list) {
      std::visit([&](const auto& ptr) { if (ptr) Walk(pass, *ptr, i); },
                 item.node);
      ++i;
    }
  }
};

template <class Pass, class T>
void Walk(Pass& pass, const T& e, int idx) {
  constexpr ElementType et = element_type_of<T>::value;
  pass.ctx.path.push_back(MakeSeg<T>(et, e, idx));
  pass.Enter(e, et);
  ChildRecurser<Pass> rec{pass};
  Visit(e, rec);
  pass.Exit(e, et);
  pass.ctx.path.pop_back();
}

// The basename of a file path without its directory or extension -- MuJoCo's
// implicit asset name when an asset with a `file` carries no explicit name.
std::string Stem(const std::string& path) {
  std::size_t slash = path.find_last_of("/\\");
  std::string base = slash == std::string::npos ? path : path.substr(slash + 1);
  std::size_t dot = base.find_last_of('.');
  return dot == std::string::npos ? base : base.substr(0, dot);
}

// MuJoCo derives an unnamed asset's name from its file basename. Meshes,
// hfields, skins, nested-model assets and file textures all carry a `file`.
template <class T>
std::string DerivedAssetName(const T& e) {
  if constexpr (std::is_same_v<T, Mesh> || std::is_same_v<T, Hfield> ||
                std::is_same_v<T, Skin> || std::is_same_v<T, ModelAsset> ||
                std::is_same_v<T, Texture>) {
    if (e.file && !e.file->empty()) return Stem(*e.file);
  }
  return std::string();
}

// --- Pass 1: symbol collection (tier 2 name uniqueness) ------------------- //
struct CollectPass {
  Ctx& ctx;
  int default_depth = 0;

  template <class T>
  void Enter(const T& e, ElementType et) {
    if constexpr (std::is_same_v<T, Default>) {
      ++default_depth;  // sub-elements below are class templates, not instances
    } else if constexpr (has_name<T>::value) {
      if (default_depth > 0) return;  // inside a default: template, not instance
      std::optional<Ns> ns = ElemNamespace(et);
      if (!ns) return;
      const std::string& nm = NameOf(e);
      if (!nm.empty()) {
        CheckReserved(nm, e.loc);
        Register(*ns, nm, e.loc);
      } else {
        // Implicit file-derived asset name: resolvable but not dup-checked (a
        // stem collision surfaces as MuJoCo's own asset error, and we avoid a
        // false positive from path stripping).
        std::string d = DerivedAssetName(e);
        if (!d.empty()) {
          ctx.symbols[static_cast<std::size_t>(*ns)].emplace(d, e.loc);
        }
      }
    }
  }
  template <class T>
  void Exit(const T&, ElementType) {
    if constexpr (std::is_same_v<T, Default>) --default_depth;
  }

  // The `_ps:` prefix is reserved for the compile bridge's auto-generated
  // binding names (which are injected only into compile-XML, never persisted
  // to a document), so an authored name inside it is always a user error and
  // could collide with an auto-name.
  void CheckReserved(const std::string& name, const ps::SourceLoc& loc) {
    if ((ctx.tiers & kTierReferential) &&
        std::string_view(name).starts_with(ps::kReservedNamePrefix)) {
      ctx.Emit(Tier::Referential, Severity::Error,
               "name '" + name + "' uses the reserved prefix '" +
                   std::string(ps::kReservedNamePrefix) + "'",
               loc);
    }
  }

  void Register(Ns ns, const std::string& name, const ps::SourceLoc& loc) {
    auto& tbl = ctx.symbols[static_cast<std::size_t>(ns)];
    auto it = tbl.find(name);
    if (it != tbl.end()) {
      if (ctx.tiers & kTierReferential) {
        ctx.Emit(Tier::Referential, Severity::Error,
                 "duplicate " + std::string(NsLabel(ns)) + " name '" + name +
                     "' (first at " + it->second.file + ":" +
                     std::to_string(it->second.line) + ")",
                 loc);
      }
      return;
    }
    tbl.emplace(name, loc);
  }
};

// Default class names live in their own tree (model.defaults), collected
// directly rather than through the generic walk so instance-name gating and
// class-name uniqueness stay separate concerns.
void CollectClassNames(const Default& d, Ctx& ctx) {
  if (d.dclass && !d.dclass->empty()) {
    // Same reservation as instance names: `_ps:` belongs to auto-names.
    if ((ctx.tiers & kTierReferential) &&
        std::string_view(*d.dclass).starts_with(ps::kReservedNamePrefix)) {
      ctx.EmitAt(Tier::Referential, Severity::Error,
                 "name '" + *d.dclass + "' uses the reserved prefix '" +
                     std::string(ps::kReservedNamePrefix) + "'",
                 d.loc, "default[" + *d.dclass + "]");
    }
    auto& tbl = ctx.symbols[static_cast<std::size_t>(Ns::Class)];
    auto it = tbl.find(*d.dclass);
    if (it != tbl.end()) {
      if (ctx.tiers & kTierReferential) {
        ctx.EmitAt(Tier::Referential, Severity::Error,
                   "duplicate default class '" + *d.dclass + "' (first at " +
                       it->second.file + ":" + std::to_string(it->second.line) +
                       ")",
                   d.loc, "default[" + *d.dclass + "]");
      }
    } else {
      tbl.emplace(*d.dclass, d.loc);
    }
  }
  for (const auto& sub : d.subclasses) {
    if (sub) CollectClassNames(*sub, ctx);
  }
}

// --- Pass 2 field checks (tier 1 structural + tier 2 refs) ---------------- //
template <class V>
void CheckInner(Ctx& ctx, const FieldDescriptor& fd, const V& v,
                const ps::SourceLoc& loc) {
  if constexpr (std::is_enum_v<V>) {
    if ((ctx.tiers & kTierStructural) && ToMjcf(v).empty()) {
      ctx.Emit(Tier::Structural, Severity::Error,
               "illegal enum value " +
                   std::to_string(static_cast<long long>(v)) + " for '" +
                   std::string(fd.name) + "'",
               loc);
    }
  } else if constexpr (is_inline_vec<V>::value) {
    if ((ctx.tiers & kTierStructural) && fd.arity == ArityKind::Range) {
      int n = static_cast<int>(v.size());
      if (n < fd.arity_min || n > fd.arity_max) {
        ctx.Emit(Tier::Structural, Severity::Error,
                 "'" + std::string(fd.name) + "' has " + std::to_string(n) +
                     " value(s), expected " + std::to_string(fd.arity_min) +
                     ".." + std::to_string(fd.arity_max),
                 loc);
      }
    }
  } else if constexpr (is_ref<V>::value) {
    if ((ctx.tiers & kTierReferential) && !v.empty() && !ctx.in_default &&
        !ctx.has_macros) {
      constexpr Ns ns = RefNs<typename is_ref<V>::target>::value;
      if (!ctx.Resolvable(ns, v.name)) {
        ctx.Emit(Tier::Referential, Severity::Error,
                 "unresolved reference '" + v.name + "' in '" +
                     std::string(fd.name) + "' (no such " +
                     std::string(NsLabel(ns)) + ")",
                 loc);
      }
    }
  } else if constexpr (is_variant<V>::value) {
    if ((ctx.tiers & kTierStructural) && v.valueless_by_exception()) {
      ctx.Emit(Tier::Structural, Severity::Error,
               "variant '" + std::string(fd.name) + "' holds no alternative",
               loc);
    }
  } else if constexpr (is_std_vector<V>::value) {
    using E = typename is_std_vector<V>::elem;
    if constexpr (std::is_enum_v<E>) {
      if (ctx.tiers & kTierStructural) {
        for (const auto& x : v) {
          if (ToMjcf(x).empty()) {
            ctx.Emit(Tier::Structural, Severity::Error,
                     "illegal enum value in '" + std::string(fd.name) + "'",
                     loc);
          }
        }
      }
    } else if constexpr (is_ref<E>::value) {
      // ref<T>[]: every entry resolves independently, same conditions as a
      // scalar ref (referential tier, outside <default>, no macro expansion).
      if ((ctx.tiers & kTierReferential) && !ctx.in_default &&
          !ctx.has_macros) {
        constexpr Ns ns = RefNs<typename is_ref<E>::target>::value;
        for (const auto& r : v) {
          if (!r.empty() && !ctx.Resolvable(ns, r.name)) {
            ctx.Emit(Tier::Referential, Severity::Error,
                     "unresolved reference '" + r.name + "' in '" +
                         std::string(fd.name) + "' (no such " +
                         std::string(NsLabel(ns)) + ")",
                     loc);
          }
        }
      }
    }
  }
  (void)ctx;
  (void)fd;
  (void)v;
  (void)loc;
}

template <class U>
void CheckFieldValue(Ctx& ctx, const FieldDescriptor& fd, const U& value,
                     const ps::SourceLoc& loc) {
  if constexpr (is_opt<U>::value) {
    if (!value.has_value()) {
      if ((ctx.tiers & kTierStructural) && !fd.optional) {
        ctx.Emit(Tier::Structural, Severity::Error,
                 "required field '" + std::string(fd.name) + "' is unset", loc);
      }
      return;
    }
    CheckInner(ctx, fd, *value, loc);
  } else {
    CheckInner(ctx, fd, value, loc);  // plain (required) scalar
  }
}

struct FieldVisitor {
  Ctx& ctx;
  const reflect::ElementDescriptor& d;
  const ps::SourceLoc& loc;

  template <class U>
  void field(int id, const char*, const U& value) {
    CheckFieldValue(ctx, d.fields[id], value, loc);
  }
  template <class U>
  void child(int id, const char*, const std::vector<std::unique_ptr<U>>& list) {
    Card(id, list.size());
  }
  template <class U>
  void union_child(int id, const char*, const std::vector<U>& list) {
    Card(id, list.size());
  }
  void Card(int id, std::size_t n) {
    if (!(ctx.tiers & kTierStructural)) return;
    const ChildDescriptor& cd = d.children[id];
    if (cd.card == Cardinality::One && n != 1) {
      ctx.Emit(Tier::Structural, Severity::Error,
               "child list '" + std::string(cd.name) + "' requires exactly one",
               loc);
    } else if (cd.card == Cardinality::ZeroOrOne && n > 1) {
      ctx.Emit(Tier::Structural, Severity::Error,
               "child list '" + std::string(cd.name) + "' allows at most one",
               loc);
    }
  }
};

// --- Pass 2 tier-3 element-local lint ------------------------------------- //
void CheckLimitPair(Ctx& ctx, const ps::SourceLoc& loc, const char* entity,
                    const char* attr, const ps::opt<TriState>& limited,
                    const ps::opt<std::array<double, 2>>& range, bool is_free) {
  // Mirror MuJoCo checklimited (user_objects.cc:173-191): the ambiguity only
  // exists when autolimits is off, `limited` is AUTO (unset defaults to AUTO),
  // and a non-trivial range was authored. Free joints force limited=false.
  if (is_free) return;
  if (ctx.autolimits) return;
  TriState lim = limited ? *limited : TriState::auto_;
  if (lim != TriState::auto_) return;
  if (!range || !RangeNontrivial(*range)) return;
  ctx.Emit(Tier::Semantic, Severity::Warning,
           std::string(entity) + " has `" + attr + "range` but not `" + attr +
               "limited`; set compiler autolimits=\"true\" or specify `" + attr +
               "limited` explicitly",
           loc);
}

int NuserFor(ElementType et, const NuserLimits& n) {
  switch (et) {
    case ElementType::Body: return n.body;
    case ElementType::Geom: return n.geom;
    case ElementType::Joint: return n.jnt;
    case ElementType::Site: return n.site;
    case ElementType::Camera: return n.cam;
    case ElementType::Spatial:
    case ElementType::Fixed: return n.tendon;
    default: break;
  }
  if (ElemNamespace(et) == Ns::Actuator) return n.actuator;
  if (ElemNamespace(et) == Ns::Sensor) return n.sensor;
  return -1;
}

template <class T>
void CheckNuser(Ctx& ctx, const T& e, ElementType et, const ps::SourceLoc& loc) {
  if (!e.user) return;
  int limit = NuserFor(et, ctx.nuser);
  if (limit < 0) return;  // nuser_* = -1 (auto): MuJoCo sizes it to the max used
  int len = static_cast<int>(e.user->size());
  if (len > limit) {
    ctx.Emit(Tier::Semantic, Severity::Warning,
             "user array length " + std::to_string(len) + " exceeds nuser_* (" +
                 std::to_string(limit) + ")",
             loc);
  }
}

template <class T>
void CheckSemanticLocal(Ctx& ctx, const T& e, ElementType et) {
  const ps::SourceLoc& loc = e.loc;

  if constexpr (std::is_same_v<T, Joint>) {
    JointType jt = e.type ? *e.type : JointType::hinge;
    bool is_free = jt == JointType::free;
    if ((jt == JointType::hinge || jt == JointType::slide) && e.axis) {
      const auto& a = *e.axis;
      if (a[0] == 0.0 && a[1] == 0.0 && a[2] == 0.0) {
        ctx.Emit(Tier::Semantic, Severity::Warning,
                 "hinge/slide joint has a zero-length axis", loc);
      }
    }
    CheckLimitPair(ctx, loc, "joint", "", e.limited, e.range, is_free);
    CheckLimitPair(ctx, loc, "joint", "actuatorfrc", e.actuatorfrclimited,
                   e.actuatorfrcrange, is_free);
  } else if constexpr (std::is_same_v<T, Spatial> ||
                       std::is_same_v<T, Fixed>) {
    CheckLimitPair(ctx, loc, "tendon", "", e.limited, e.range, false);
    CheckLimitPair(ctx, loc, "tendon", "actuatorfrc", e.actuatorfrclimited,
                   e.actuatorfrcrange, false);
  } else if constexpr (std::is_same_v<T, Camera>) {
    // R1 (docs/plan_canonicalization.md Section 2): camera intrinsic parameters
    // (focal/principal, in length or pixel units) require a positive sensorsize,
    // a compile error in MuJoCo (user_objects.cc:4426-4435). Mirrored as a
    // tier-3 lint on the authored form so the CameraIntrinsics dissolution (six
    // plain fields, R1) keeps MuJoCo's error surface. `fovy` XOR `sensorsize` on
    // the same element is the separate reader error (CameraIntrinsicExclusion).
    auto nz2 = [](const auto& v) { return v && ((*v)[0] || (*v)[1]); };
    bool has_intrinsic = nz2(e.focal) || nz2(e.focalpixel) ||
                         nz2(e.principal) || nz2(e.principalpixel);
    bool has_sensorsize =
        e.sensorsize && (*e.sensorsize)[0] > 0 && (*e.sensorsize)[1] > 0;
    if (has_intrinsic && !has_sensorsize) {
      ctx.Emit(Tier::Semantic, Severity::Warning,
               "camera focal/principal require a positive `sensorsize`", loc);
    }
  } else if constexpr (std::is_same_v<T, Config>) {
    if (e.key.empty()) {
      ctx.Emit(Tier::Semantic, Severity::Warning,
               "plugin config entry has an empty key", loc);
    }
  } else if constexpr (std::is_same_v<T, PluginRef>) {
    if ((!e.plugin || e.plugin->empty()) && (!e.instance || e.instance->empty())) {
      ctx.Emit(Tier::Semantic, Severity::Warning,
               "plugin reference names neither a plugin nor an instance", loc);
    }
  }

  if constexpr (has_ctrl<T>::value) {
    CheckLimitPair(ctx, loc, "actuator", "ctrl", e.ctrllimited, e.ctrlrange,
                   false);
  }
  if constexpr (has_force<T>::value) {
    CheckLimitPair(ctx, loc, "actuator", "force", e.forcelimited, e.forcerange,
                   false);
  }
  if constexpr (has_act<T>::value) {
    CheckLimitPair(ctx, loc, "actuator", "act", e.actlimited, e.actrange, false);
  }
  if constexpr (has_user<T>::value) {
    CheckNuser(ctx, e, et, loc);
  }
  (void)ctx;
  (void)e;
  (void)et;
  (void)loc;
}

struct CheckPass {
  Ctx& ctx;
  int default_depth = 0;

  template <class T>
  void Enter(const T& e, ElementType et) {
    ctx.in_default = default_depth > 0;
    if (ctx.tiers & (kTierStructural | kTierReferential)) {
      FieldVisitor fv{ctx, reflect::Describe(et), e.loc};
      Visit(e, fv);
      CheckDynRefs(e, et);
    }
    if (ctx.tiers & kTierSemantic) CheckSemanticLocal(ctx, e, et);
    if constexpr (std::is_same_v<T, Default>) ++default_depth;
  }

  // Dynamic refs (target_from): a string field whose target type is the
  // runtime keyword of a sibling field. The typed field walk above cannot see
  // them (they are plain strings there); resolve keyword -> namespace and run
  // the same referential check a scalar ref gets. An unknown keyword is left
  // alone -- objtype's own legality is a separate (structural) concern.
  template <class T>
  void CheckDynRefs(const T& e, ElementType et) {
    if (!(ctx.tiers & kTierReferential) || ctx.in_default || ctx.has_macros) {
      return;
    }
    const reflect::ElementDescriptor& d = reflect::Describe(et);
    for (int i = 0; i < static_cast<int>(d.field_count); ++i) {
      const reflect::FieldDescriptor& fd = d.fields[i];
      if (fd.target_from.empty()) continue;
      int sib = -1;
      for (int j = 0; j < static_cast<int>(d.field_count); ++j) {
        if (d.fields[j].name == fd.target_from) {
          sib = j;
          break;
        }
      }
      if (sib < 0) continue;
      const ps::opt<std::string>* kw = StringFieldAt(e, sib);
      const ps::opt<std::string>* nm = StringFieldAt(e, i);
      if (!kw || !kw->has_value() || !nm || !nm->has_value() ||
          (*nm)->empty()) {
        continue;
      }
      std::optional<Ns> ns = DynNs(**kw);
      if (!ns) continue;
      if (!ctx.Resolvable(*ns, **nm)) {
        ctx.Emit(Tier::Referential, Severity::Error,
                 "unresolved reference '" + **nm + "' in '" +
                     std::string(fd.name) + "' (no such " +
                     std::string(NsLabel(*ns)) + ")",
                 e.loc);
      }
    }
  }
  template <class T>
  void Exit(const T&, ElementType) {
    if constexpr (std::is_same_v<T, Default>) --default_depth;
  }
};

// --- tier-3 whole-model checks -------------------------------------------- //
void CheckKeyframeLen(Ctx& ctx, const Key& k, const char* field,
                      const ps::opt<std::vector<double>>& v, int expected,
                      bool expected_valid) {
  if (!v || v->empty() || !expected_valid) return;
  int n = static_cast<int>(v->size());
  if (n != expected) {
    std::string name = (k.name && !k.name->empty()) ? *k.name : std::string("?");
    ctx.EmitAt(Tier::Semantic, Severity::Warning,
               std::string("keyframe ") + field + " length " +
                   std::to_string(n) + " != model " + field + " size " +
                   std::to_string(expected),
               k.loc, "keyframe/key[" + name + "]");
  }
}

void RunKeyframeChecks(Ctx& ctx, const Model& model) {
  const ModelSizes& s = ctx.sizes;
  for (const auto& kf : model.keyframes) {
    if (!kf) continue;
    for (const auto& key : kf->keys) {
      if (!key) continue;
      CheckKeyframeLen(ctx, *key, "qpos", key->qpos, s.nq, s.exact);
      CheckKeyframeLen(ctx, *key, "qvel", key->qvel, s.nv, s.exact);
      CheckKeyframeLen(ctx, *key, "ctrl", key->ctrl, s.nu, s.exact);
      CheckKeyframeLen(ctx, *key, "act", key->act, s.na,
                       s.exact && s.na_exact);
      CheckKeyframeLen(ctx, *key, "mpos", key->mpos, 3 * s.nmocap, s.exact);
      CheckKeyframeLen(ctx, *key, "mquat", key->mquat, 4 * s.nmocap, s.exact);
    }
  }
}

// A mocap body must be a static child of the world (MuJoCo user_model.cc): no
// enclosing kinematic body, and no joints of its own. Frames/replicate are
// transparent groupings, so "child of world" means "no Body ancestor".
bool SubtreeHasJoint(const std::vector<BodyChildAny>& subtree) {
  for (const auto& item : subtree) {
    if (item.kind() == BodyChildAny::Kind::Joint ||
        item.kind() == BodyChildAny::Kind::FreeJoint) {
      return true;
    }
  }
  return false;
}

void CheckMocapSubtree(Ctx& ctx, const std::vector<BodyChildAny>& subtree,
                       bool has_body_ancestor, const std::string& parent_path);

void CheckMocapBody(Ctx& ctx, const Body& b, bool has_body_ancestor,
                    const std::string& path) {
  if (b.mocap && *b.mocap) {
    if (has_body_ancestor) {
      ctx.EmitAt(Tier::Semantic, Severity::Warning,
                 "mocap body must be a direct (static) child of the world body",
                 b.loc, path);
    }
    if (SubtreeHasJoint(b.subtree)) {
      ctx.EmitAt(Tier::Semantic, Severity::Warning,
                 "mocap body cannot have joints", b.loc, path);
    }
  }
  CheckMocapSubtree(ctx, b.subtree, true, path);
}

void CheckMocapSubtree(Ctx& ctx, const std::vector<BodyChildAny>& subtree,
                       bool has_body_ancestor, const std::string& parent_path) {
  int i = 0;
  for (const auto& item : subtree) {
    std::string p = parent_path + "/#" + std::to_string(i++);
    switch (item.kind()) {
      case BodyChildAny::Kind::Body:
        CheckMocapBody(ctx, *std::get<std::unique_ptr<Body>>(item.node),
                       has_body_ancestor, p);
        break;
      case BodyChildAny::Kind::Frame:
        CheckMocapSubtree(
            ctx, std::get<std::unique_ptr<Frame>>(item.node)->subtree,
            has_body_ancestor, p);
        break;
      case BodyChildAny::Kind::Replicate:
        CheckMocapSubtree(
            ctx, std::get<std::unique_ptr<Replicate>>(item.node)->subtree,
            has_body_ancestor, p);
        break;
      default:
        break;
    }
  }
}

void RunMocapChecks(Ctx& ctx, const Model& model) {
  // `model.worldbody` holds the <worldbody> element(s): each is the world root,
  // not a kinematic body, so its direct subtree children are children OF the
  // world (has_body_ancestor = false). A mocap body one level deeper (inside a
  // real body) is what MuJoCo rejects.
  for (const auto& wb : model.worldbody) {
    if (wb) CheckMocapSubtree(ctx, wb->subtree, false, "worldbody");
  }
}

// --- setup helpers -------------------------------------------------------- //
bool SubtreeHasMacro(const std::vector<BodyChildAny>& subtree) {
  for (const auto& item : subtree) {
    switch (item.kind()) {
      case BodyChildAny::Kind::Composite:
      case BodyChildAny::Kind::Flexcomp:
      case BodyChildAny::Kind::Replicate:
      case BodyChildAny::Kind::Attach:
        return true;
      case BodyChildAny::Kind::Body:
        if (SubtreeHasMacro(std::get<std::unique_ptr<Body>>(item.node)->subtree))
          return true;
        break;
      case BodyChildAny::Kind::Frame:
        if (SubtreeHasMacro(
                std::get<std::unique_ptr<Frame>>(item.node)->subtree))
          return true;
        break;
      default:
        break;
    }
  }
  return false;
}

bool ModelHasMacros(const Model& model) {
  for (const auto& b : model.worldbody) {
    if (b && SubtreeHasMacro(b->subtree)) return true;
  }
  return false;
}

void PrepareContext(Ctx& ctx, const Model& model) {
  ctx.has_macros = ModelHasMacros(model);
  // autolimits: last authored value wins across merged compiler blocks; unset
  // defaults to true (MuJoCo compiler default).
  for (const auto& c : model.compilers) {
    if (c && c->autolimits) ctx.autolimits = *c->autolimits;
  }
  // nuser_*: last authored value per field across size blocks; unset = -1 auto.
  for (const auto& sz : model.sizes) {
    if (!sz) continue;
    if (sz->nuser_body) ctx.nuser.body = *sz->nuser_body;
    if (sz->nuser_jnt) ctx.nuser.jnt = *sz->nuser_jnt;
    if (sz->nuser_geom) ctx.nuser.geom = *sz->nuser_geom;
    if (sz->nuser_site) ctx.nuser.site = *sz->nuser_site;
    if (sz->nuser_cam) ctx.nuser.cam = *sz->nuser_cam;
    if (sz->nuser_tendon) ctx.nuser.tendon = *sz->nuser_tendon;
    if (sz->nuser_actuator) ctx.nuser.actuator = *sz->nuser_actuator;
    if (sz->nuser_sensor) ctx.nuser.sensor = *sz->nuser_sensor;
  }
  ctx.sizes = ComputeSizes(model);
}

}  // namespace

std::vector<ps::Diagnostic> Validate(const Model& model, TierMask tiers) {
  Ctx ctx;
  ctx.tiers = tiers;
  PrepareContext(ctx, model);

  // Pass 1: collect the symbol tables (name uniqueness is reported here too).
  if (tiers & kTierReferential) {
    for (const auto& d : model.defaults) {
      if (d) CollectClassNames(*d, ctx);
    }
  }
  {
    CollectPass cp{ctx};
    Walk(cp, model, 0);
  }

  // Pass 2: field-level tier 1/2 checks + element-local tier 3 lint.
  {
    CheckPass chk{ctx};
    Walk(chk, model, 0);
  }

  // Whole-model tier 3 lint that needs cross-element context.
  if (tiers & kTierSemantic) {
    RunKeyframeChecks(ctx, model);
    RunMocapChecks(ctx, model);
  }

  return std::move(ctx.out);
}

}  // namespace ps::mjcf::validate
