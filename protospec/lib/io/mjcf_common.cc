// MJCF reader/writer: the profile-independent half (mjcf_common.h).
//
// Compiled once, whatever the profile. Nothing in this file touches how a
// document stores a value: it is keyed on ElementType, on raw XML and on
// diagnostics only.

#include "mjcf_common.h"

#include <string>
#include <string_view>
#include <unordered_set>

namespace ps::mjcf::io::common {
namespace {

// A bundle rendered as MuJoCo renders it: 'a' for a single attribute,
// ('a', 'b') for a group, comma-joined (BundleList, xml_util.cc:393-411).
std::string BundleList(const reflect::ElementDescriptor& d,
                       const reflect::ConstraintDescriptor& c) {
  std::string out;
  for (std::size_t i = 0; i < c.bundle_count; ++i) {
    const reflect::ConstraintBundle& b = c.bundles[i];
    if (i) out += ", ";
    if (b.field_count > 1) out += '(';
    for (std::size_t j = 0; j < b.field_count; ++j) {
      if (j) out += ", ";
      out += '\'';
      out += d.fields[b.fields[j]].xml;
      out += '\'';
    }
    if (b.field_count > 1) out += ')';
  }
  return out;
}

}  // namespace

// The supported families: every element ProtoSpec models as first-class data --
// Model-level blocks, the body tree, defaults, assets, the contact/equality/
// tendon/actuator sections, sensors, custom/keyframe/extension, and the
// deformable/macro pass-throughs. Everything else is a well-formed but
// unsupported element (skip signal), never a malformed input.
bool IsSupported(ElementType t) {
  switch (t) {
    case ElementType::Model:
    case ElementType::Compiler:
    case ElementType::LengthRange:
    case ElementType::Option:
    case ElementType::Flag:
    case ElementType::Size:
    case ElementType::Visual:
    case ElementType::VisualGlobal:
    case ElementType::VisualQuality:
    case ElementType::VisualHeadlight:
    case ElementType::VisualMap:
    case ElementType::VisualScale:
    case ElementType::VisualRgba:
    case ElementType::Statistic:
    case ElementType::Body:
    case ElementType::Inertial:
    case ElementType::Joint:
    case ElementType::FreeJoint:
    case ElementType::Geom:
    case ElementType::Site:
    case ElementType::Camera:
    case ElementType::Light:
    case ElementType::Frame:
    // Defaults (family c): the class tree and its per-family sub-elements.
    case ElementType::Default:
    case ElementType::Pair:
    case ElementType::EqualityDefault:
    case ElementType::TendonDefault:
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
    // Contact / equality / tendon sections (family e).
    case ElementType::Contact:
    case ElementType::Exclude:
    case ElementType::Equality:
    case ElementType::Connect:
    case ElementType::Weld:
    case ElementType::EqualityJoint:
    case ElementType::EqualityTendon:
    case ElementType::EqualityFlex:
    case ElementType::Flexvert:
    case ElementType::Flexstrain:
    case ElementType::Tendon:
    case ElementType::Spatial:
    case ElementType::SpatialSite:
    case ElementType::SpatialGeom:
    case ElementType::Pulley:
    case ElementType::Fixed:
    case ElementType::FixedJoint:
    // Actuator section (family f).
    case ElementType::Actuator:
    case ElementType::ActuatorPlugin:
    case ElementType::Config:
    // Assets (family d).
    case ElementType::Asset:
    case ElementType::Mesh:
    case ElementType::Hfield:
    case ElementType::Skin:
    case ElementType::SkinBone:
    case ElementType::Texture:
    case ElementType::Material:
    case ElementType::MaterialLayer:
    case ElementType::ModelAsset:
    // Sensors (family g): the full <sensor> section as an ordered union.
    case ElementType::Sensor:
    case ElementType::Touch:
    case ElementType::Accelerometer:
    case ElementType::Velocimeter:
    case ElementType::Gyro:
    case ElementType::Force:
    case ElementType::Torque:
    case ElementType::Magnetometer:
    case ElementType::Camprojection:
    case ElementType::Rangefinder:
    case ElementType::Jointpos:
    case ElementType::Jointvel:
    case ElementType::Tendonpos:
    case ElementType::Tendonvel:
    case ElementType::Actuatorpos:
    case ElementType::Actuatorvel:
    case ElementType::Actuatorfrc:
    case ElementType::Jointactuatorfrc:
    case ElementType::Tendonactuatorfrc:
    case ElementType::Ballquat:
    case ElementType::Ballangvel:
    case ElementType::Jointlimitpos:
    case ElementType::Jointlimitvel:
    case ElementType::Jointlimitfrc:
    case ElementType::Tendonlimitpos:
    case ElementType::Tendonlimitvel:
    case ElementType::Tendonlimitfrc:
    case ElementType::Framepos:
    case ElementType::Framequat:
    case ElementType::Framexaxis:
    case ElementType::Frameyaxis:
    case ElementType::Framezaxis:
    case ElementType::Framelinvel:
    case ElementType::Frameangvel:
    case ElementType::Framelinacc:
    case ElementType::Frameangacc:
    case ElementType::Subtreecom:
    case ElementType::Subtreelinvel:
    case ElementType::Subtreeangmom:
    case ElementType::Insidesite:
    case ElementType::Distance:
    case ElementType::Normal:
    case ElementType::Fromto:
    case ElementType::SensorContact:
    case ElementType::EPotential:
    case ElementType::EKinetic:
    case ElementType::Clock:
    case ElementType::Tactile:
    case ElementType::SensorUser:
    case ElementType::SensorPlugin:
    // Custom + keyframe + extension (family h).
    case ElementType::Custom:
    case ElementType::Numeric:
    case ElementType::Text:
    case ElementType::Tuple:
    case ElementType::TupleElement:
    case ElementType::Keyframe:
    case ElementType::Key:
    case ElementType::Extension:
    case ElementType::PluginDef:
    case ElementType::PluginInstance:
    case ElementType::PluginRef:
    // Macros + deformable (family i): first-class pass-through.
    case ElementType::Composite:
    case ElementType::CompositeJoint:
    case ElementType::CompositeSkin:
    case ElementType::CompositeGeom:
    case ElementType::CompositeSite:
    case ElementType::Flexcomp:
    case ElementType::FlexcompEdge:
    case ElementType::FlexElasticity:
    case ElementType::FlexContact:
    case ElementType::FlexcompPin:
    case ElementType::Attach:
    case ElementType::Replicate:
    case ElementType::Deformable:
    case ElementType::Flex:
    case ElementType::FlexEdge:
      return true;
    default:
      return false;
  }
}

// The MuJoCo name-uniqueness namespace an element's name lives in, as a stable
// label, or nullptr for elements that carry no namespaced name. This MIRRORS the
// authoritative grouping in validate.cc (ElemNamespace/NsLabel) so the parse-time
// duplicate-name WARNING matches the validator's ERROR tier: merged families
// (all actuator spellings -> "actuator", all sensor spellings -> "sensor",
// equality spellings -> "equality", Spatial/Fixed -> "tendon", Joint/FreeJoint
// -> "joint") share one namespace; every other named element is its own. An
// element this function does not classify simply gets no early warning -- the
// referential validator stays authoritative, so under-classification is safe
// (never a false error, never a cross-namespace false positive).
const char* NameNsLabel(ElementType et) {
  switch (et) {
    case ElementType::Body: return "body";
    case ElementType::Geom: return "geom";
    case ElementType::Joint:
    case ElementType::FreeJoint: return "joint";
    case ElementType::Site: return "site";
    case ElementType::Camera: return "camera";
    case ElementType::Light: return "light";
    case ElementType::Mesh: return "mesh";
    case ElementType::Material: return "material";
    case ElementType::Texture: return "texture";
    case ElementType::Hfield: return "hfield";
    case ElementType::Skin: return "skin";
    case ElementType::Flex: return "flex";
    case ElementType::Pair: return "pair";
    case ElementType::Exclude: return "exclude";
    case ElementType::Connect:
    case ElementType::Weld:
    case ElementType::EqualityJoint:
    case ElementType::EqualityTendon:
    case ElementType::EqualityFlex:
    case ElementType::Flexvert:
    case ElementType::Flexstrain: return "equality";
    case ElementType::Spatial:
    case ElementType::Fixed: return "tendon";
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
    case ElementType::ActuatorPlugin: return "actuator";
    case ElementType::Numeric: return "numeric";
    case ElementType::Text: return "text";
    case ElementType::Tuple: return "tuple";
    case ElementType::Key: return "key";
    case ElementType::Frame: return "frame";
    case ElementType::PluginInstance: return "plugin instance";
    case ElementType::ModelAsset: return "model asset";
    default: break;
  }
  // Every remaining element is a sensor spelling (a SensorAny member) or an
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
    case ElementType::SensorPlugin: return "sensor";
    default: return nullptr;
  }
}

Resolver LookupResolver(std::string_view name) {
  if (name == "orientation") return Resolver::Orientation;
  if (name == "materiallayer") return Resolver::MaterialLayer;
  if (name == "inertia") return Resolver::Inertia;
  if (name == "springlength") return Resolver::Springlength;
  if (name == "lighttype") return Resolver::LightType;
  if (name == "cylinderarea") return Resolver::CylinderArea;
  return Resolver::None;
}

// Provenance: elements spliced from an included file carry that file's
// path and line via the pre-pass map; top-level elements take theirs from
// tinyxml2 against the model filename.
ps::SourceLoc ReaderBase::Loc(const XMLElement* e) const {
  if (provenance_) {
    auto it = provenance_->find(e);
    if (it != provenance_->end()) return it->second;
  }
  return ps::SourceLoc{filename_, e->GetLineNum()};
}

void ReaderBase::Err(const XMLElement* e, std::string msg) {
  ps::Diagnostic d;
  d.source = "parse";
  d.kind = ps::Diagnostic::Kind::MalformedInput;
  d.message = std::move(msg);
  d.loc = Loc(e);
  errors_.push_back(std::move(d));
}
void ReaderBase::Unsupported(const XMLElement* e, std::string tag) {
  ps::Diagnostic d;
  d.source = "parse";
  d.kind = ps::Diagnostic::Kind::UnsupportedElement;
  d.message = "unsupported element '" + tag + "'";
  d.loc = Loc(e);
  errors_.push_back(std::move(d));
}
void ReaderBase::Warn(const XMLElement* e, std::string msg) {
  ps::Diagnostic d;
  d.severity = ps::Diagnostic::Severity::Warning;
  d.source = "parse";
  d.kind = ps::Diagnostic::Kind::MalformedInput;
  d.message = std::move(msg);
  d.loc = Loc(e);
  warnings_.push_back(std::move(d));
}

// Parse-time duplicate-name diagnostic: at first sight of a name
// that repeats within a MuJoCo name namespace, emit a WARNING carrying BOTH
// source locations (the first occurrence in the message, this one in the loc).
// The referential validator remains the authoritative error tier; this only
// surfaces the collision early, matching stock MuJoCo's fail-at-read shape.
// Cross-namespace repeats (a body and a geom both named "x") never fire: the
// key is (namespace-label, name), so only same-namespace repeats collide.
void ReaderBase::CheckDuplicateName(const XMLElement* xml, ElementType et) {
  const char* label = NameNsLabel(et);
  if (!label) return;
  const char* name = xml->Attribute("name");
  if (!name || name[0] == '\0') return;
  std::string key = std::string(label);
  key.push_back('\x1f');  // unit separator: label and name cannot alias
  key += name;
  ps::SourceLoc here = Loc(xml);
  auto [it, inserted] = name_first_loc_.emplace(std::move(key), here);
  if (inserted) return;
  const ps::SourceLoc& first = it->second;
  std::string first_at = first.file;
  if (first.line > 0) first_at += ":" + std::to_string(first.line);
  Warn(xml, "duplicate " + std::string(label) + " name '" +
                std::string(name) + "' (first defined at " + first_at + ")");
}

void ReaderBase::ReportNumError(XMLElement* xml, const AttrBinding& ab, num::Status st) {
  if (st == num::Status::Overflow) {
    Err(xml, "number is too large in attribute '" + std::string(ab.attr) +
                 "'");
  } else {
    Err(xml, "bad number in attribute '" + std::string(ab.attr) + "'");
  }
}

// Read exactly n doubles from an attribute (verbatim, no unit conversion).
bool ReaderBase::ReadDoubleArr(XMLElement* xml, const char* attr, int n,
                               double* out) {
  AttrBinding ab{};
  ab.attr = attr;
  return ReadNumberN(xml, ab, xml->Attribute(attr), out, n, n) == n;
}

// The schema's exclusive / together / requires / oneof rows, evaluated on the
// raw XML exactly as mjXSchema does (xml_util.cc:416-482), down to the wording:
// a bundle is "any" when one of its attributes is present and "all" when they
// all are; exclusive admits at most one any-bundle, oneof demands at least one
// all-bundle, together is all-or-none across every listed attribute, and
// requires ties the first bundle's head to the second's. Reading them from the
// generated tables is what lets the reader carry no per-element hook for them.
void ReaderBase::CheckConstraints(XMLElement* xml, ElementType type) {
  const reflect::ElementDescriptor& d = reflect::Describe(type);
  for (std::size_t ci = 0; ci < d.constraint_count; ++ci) {
    const reflect::ConstraintDescriptor& c = d.constraints[ci];
    int n_any = 0, n_all = 0, n_attr = 0, n_present = 0;
    for (std::size_t bi = 0; bi < c.bundle_count; ++bi) {
      const reflect::ConstraintBundle& b = c.bundles[bi];
      bool any = false, all = true;
      for (std::size_t j = 0; j < b.field_count; ++j) {
        bool present = Has(xml, d.fields[b.fields[j]].xml);
        any |= present;
        all &= present;
        ++n_attr;
        n_present += present;
      }
      n_any += any;
      n_all += all;
    }
    switch (c.kind) {
      case reflect::ConstraintKind::Exclusive:
        if (n_any > 1) {
          Err(xml, "at most one of " + BundleList(d, c) + " can be specified");
          return;
        }
        break;
      case reflect::ConstraintKind::Together:
        if (n_present != 0 && n_present != n_attr) {
          Err(xml, "attributes " + BundleList(d, c) +
                       " must be specified together");
          return;
        }
        break;
      case reflect::ConstraintKind::Requires: {
        std::string_view head = d.fields[c.bundles[0].fields[0]].xml;
        std::string_view need = d.fields[c.bundles[1].fields[0]].xml;
        if (Has(xml, head) && !Has(xml, need)) {
          Err(xml, "attribute '" + std::string(head) + "' requires attribute '" +
                       std::string(need) + "'");
          return;
        }
        break;
      }
      case reflect::ConstraintKind::OneOf:
        if (n_all == 0) {
          Err(xml, "one of " + BundleList(d, c) + " must be specified");
          return;
        }
        break;
    }
  }
}

// ---- children ----------------------------------------------------------- //
void ReaderBase::CheckUnknownAttributes(XMLElement* xml, const ElementBinding& b) {
  std::unordered_set<std::string_view> known;
  for (std::size_t i = 0; i < b.attr_count; ++i) known.insert(b.attrs[i].attr);
  // Read-only input aliases are accepted spellings canonicalized on read:
  // euler/axisangle/xyaxes/zaxis, fullinertia and the
  // Wave B scalar/slot aliases (directional->type, diameter->area, material
  // texture->layer).
  for (std::size_t i = 0; i < b.input_alias_count; ++i)
    known.insert(b.input_aliases[i].attr);
  // An aliased tag (<frame>, <replicate>) is validated against the element it
  // aliases, and mjXSchema reaches that check through its recursive branch --
  // which never calls Check on the aliased element itself, so the engine
  // silently ignores whatever attributes it carries (a stray prefix= on
  // <replicate> in the corpus is dropped, not rejected). Rejecting them would
  // make the reader stricter than MuJoCo and turn loadable models into
  // parse errors, so they are surfaced as a warning instead.
  const bool validated = b.alias.empty();
  for (const tinyxml2::XMLAttribute* a = xml->FirstAttribute(); a;
       a = a->Next()) {
    if (known.count(a->Name())) continue;
    std::string msg = "unknown attribute '" + std::string(a->Name()) +
                      "' in element '" + std::string(b.tag) + "'";
    if (validated) {
      Err(xml, std::move(msg));
    } else {
      Warn(xml, msg + " (ignored, as MuJoCo does)");
    }
  }
}

// fullinertia carries the inertial frame in its eigenvectors, so an authored
// inertial orientation (quat/euler/...) alongside it is a conflict, matching
// MuJoCo (user_objects.cc:2705-2716). Checked on the raw XML before the
// orientation collect, so the evidence is intact.
void ReaderBase::InertialOrientExclusion(XMLElement* xml) {
  if (!Has(xml, "fullinertia")) return;
  int orient = Has(xml, "quat") + Has(xml, "axisangle") + Has(xml, "xyaxes") +
               Has(xml, "zaxis") + Has(xml, "euler");
  if (orient > 0) {
    Err(xml, "fullinertia and inertial orientation cannot both be specified");
  }
}

// An actuator has at most one transmission target, and the
// slidercrank-only (cranklength/slidersite) and site-only (refsite)
// attributes require the matching transmission (xml_native_reader.cc:
// 2415-2470). trntype is undefined with no target, slidercrank with
// cranksite, site with site.
void ReaderBase::ActuatorTransmission(XMLElement* xml) {
  int cnt = Has(xml, "joint") + Has(xml, "jointinparent") +
            Has(xml, "tendon") + Has(xml, "cranksite") + Has(xml, "site") +
            Has(xml, "body");
  if (cnt > 1) {
    Err(xml, "actuator can have at most one of transmission target");
    return;
  }
  bool cranklength = Has(xml, "cranklength");
  bool slidersite = Has(xml, "slidersite");
  if ((cranklength || slidersite) && cnt == 1 && !Has(xml, "cranksite")) {
    Err(xml,
        "cranklength and slidersite can only be used in slidercrank "
        "transmission");
    return;
  }
  if (Has(xml, "refsite") && cnt == 1 && !Has(xml, "site")) {
    Err(xml, "refsite can only be used with site transmission");
  }
}

// contact sensor value rules the presence rows cannot state: `num` must be
// positive and the `data` keywords must be authored in strict enum order
// (xml_native_reader.cc:4505-4560). The source exclusivity beside them is a
// schema constraint.
void ReaderBase::ContactSensorData(XMLElement* xml) {
  if (const char* num = xml->Attribute("num")) {
    // Tokenize first: ParseInt rejects surrounding whitespace, so a padded
    // value ("  -2 ") would otherwise slip past the positivity gate.
    auto toks = num::Tokens(std::string_view(num));
    int n = 0;
    if (toks.size() == 1 &&
        num::ParseInt<int>(toks[0], n) == num::Status::Ok && n <= 0) {
      Err(xml, "'num' must be positive in sensor");
    }
  }
  // #9: the contact sensor's `data` keywords must be authored in strict enum
  // order (xml_native_reader.cc:4517-4530) -- unlike camera/rangefinder keyword
  // sets (order-insensitive), the contact reader rejects any other order, so
  // enum order is the sole legal spelling. Checked on the raw input before the
  // reader's keyword-set canonicalization sorts it.
  if (const char* data = xml->Attribute("data")) {
    int prev = -1;
    for (std::string_view tok : num::Tokens(std::string_view(data))) {
      ContactData v{};
      if (!FromMjcf(tok, v)) break;  // invalid keyword: reported by the field path
      int cur = static_cast<int>(v);
      if (cur <= prev) {
        Err(xml,
            "data attributes must be in order: found, force, torque, dist, "
            "pos, normal, tangent");
        return;
      }
      prev = cur;
    }
  }
}

// plugin sensor: objtype/objname and reftype/refname pair up (:4607-4626).
void ReaderBase::PluginSensorPairing(XMLElement* xml) {
  bool objtype = Has(xml, "objtype"), objname = Has(xml, "objname");
  if (objtype && !objname) {
    Err(xml, "objtype is specified but objname is not");
  } else if (objname && !objtype) {
    Err(xml, "objname is specified but objtype is not");
  }
  bool reftype = Has(xml, "reftype"), refname = Has(xml, "refname");
  if (reftype && !refname) {
    Err(xml, "reftype is specified but refname is not");
  } else if (refname && !reftype) {
    Err(xml, "refname is specified but reftype is not");
  }
}

// A plugin/instance element's <config> keys must be unique
// (ReadPluginConfigs, xml_native_reader.cc:135-141).
void ReaderBase::ConfigKeyUnique(XMLElement* xml) {
  std::unordered_set<std::string> seen;
  for (XMLElement* c = xml->FirstChildElement("config"); c;
       c = c->NextSiblingElement("config")) {
    const char* key = c->Attribute("key");
    if (key && !seen.insert(key).second) {
      Err(c, "duplicate config key: " + std::string(key));
    }
  }
}

// Does the schema fold this element's orientation spellings into one canonical
// quat? True exactly when the binding marks a field with the orientation
// resolver.
bool FoldsOrientation(const ElementBinding& b) {
  for (std::size_t i = 0; i < b.attr_count; ++i)
    if (b.attrs[i].resolver == "orientation") return true;
  return false;
}

// The reflect field id the named resolver owns on this element, or -1. The
// binding's attribute rows carry both the resolver name and the field id, so the
// canonical destination of a deferred or aliased read is a schema lookup rather
// than a hand-written overload per element type.
int ResolverFieldId(const ElementBinding& b, std::string_view resolver) {
  for (std::size_t i = 0; i < b.attr_count; ++i)
    if (b.attrs[i].resolver == resolver) return b.attrs[i].field_id;
  return -1;
}

}  // namespace ps::mjcf::io::common

namespace ps::mjcf::io {

bool ErrorsUnsupportedOnly(const std::vector<ps::Diagnostic>& errors) {
  bool any_unsupported = false;
  for (const auto& e : errors) {
    if (e.kind == ps::Diagnostic::Kind::MalformedInput) return false;
    if (e.kind == ps::Diagnostic::Kind::UnsupportedElement) any_unsupported = true;
  }
  return any_unsupported;
}

}  // namespace ps::mjcf::io
