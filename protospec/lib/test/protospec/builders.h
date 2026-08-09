// ProtoSpec SDK: typed builders.
//
// Ergonomic Add* helpers that construct an element, link it into the correct
// child list or ordered union subtree (Section 6 interleave), and return a
// reference to the live element for further authoring. Structural parameters
// (a geom's type, a joint's type) default to the same value the schema records
// as the compiler default, so `AddGeom(body)` yields a sphere. In keeping with
// Because defaults are never silently written into models, builders do NOT
// stamp the full default set onto an element -- only what you pass is authored.
// Call `ps::mjcf::ApplyDefault(elem)` explicitly to seed the rest.
//
// The `name` argument is always optional; unnamed elements are still bindable
// through their creation serial, so a name is only for your own refs.
//
// Every verb names its element by schema identity (`element_t<ET::Geom>`) and
// links it through the profile's tree adapter, so construction and linkage are
// one step -- which is what a component profile requires and what the plain
// profile is happy to provide.
#ifndef PROTOSPEC_SDK_BUILDERS_H
#define PROTOSPEC_SDK_BUILDERS_H

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include "protospec/detail.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "protospec/refs.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

namespace detail {

// Construct a child of the element type `E` under `parent`, naming it when a
// name is given.
template <class P, mj::ElementType E, class Parent>
ElementOf<P, E>& AddNamed(Parent& parent, ViewOf<P> name) {
  ElementOf<P, E>& child = Create<P, ElementOf<P, E>>(parent);
  if (!P::Str::Empty(name)) SetName<P>(child, name);
  return child;
}

}  // namespace detail

// --- Body tree ------------------------------------------------------------ //

// The single world body, created (unnamed) if the document has none.
template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Body>& World(DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Body>>(model);
}

// `Parent` is any body-context element carrying a subtree (Body, Frame,
// Replicate). Each helper appends into that ordered list.
template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Body>& AddBody(Parent& parent,
                                             ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Body>(parent, name);
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Geom>& AddGeom(
    Parent& parent, mj::GeomType type = mj::GeomType::sphere,
    ViewOf<P> name = {}) {
  auto& g = detail::AddNamed<P, mj::ElementType::Geom>(parent, name);
  detail::SetEnumField<P>(g, detail::FieldIdByName(mj::ElementType::Geom, "type"),
                          static_cast<int>(type));
  return g;
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Joint>& AddJoint(
    Parent& parent, mj::JointType type = mj::JointType::hinge,
    ViewOf<P> name = {}) {
  auto& j = detail::AddNamed<P, mj::ElementType::Joint>(parent, name);
  detail::SetEnumField<P>(
      j, detail::FieldIdByName(mj::ElementType::Joint, "type"),
      static_cast<int>(type));
  return j;
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::FreeJoint>& AddFreeJoint(Parent& parent,
                                                       ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::FreeJoint>(parent, name);
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Site>& AddSite(Parent& parent,
                                             ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Site>(parent, name);
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Camera>& AddCamera(Parent& parent,
                                                 ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Camera>(parent, name);
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Light>& AddLight(Parent& parent,
                                               ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Light>(parent, name);
}

template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Frame>& AddFrame(Parent& parent,
                                               ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Frame>(parent, name);
}

// The inertial element is a distinct (non-union) child of a body/frame.
template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Inertial>& AddInertial(Parent& parent) {
  return detail::Create<P, ElementOf<P, mj::ElementType::Inertial>>(parent);
}

// --- Assets --------------------------------------------------------------- //

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Asset>& EnsureAsset(DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Asset>>(model);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Mesh>& AddMesh(DocOf<P>& model,
                                             ViewOf<P> name = {},
                                             ViewOf<P> file = {}) {
  auto& m = detail::AddNamed<P, mj::ElementType::Mesh>(EnsureAsset<P>(model), name);
  if (!P::Str::Empty(file)) {
    detail::SetStrField<P>(m,
                           detail::FieldIdByName(mj::ElementType::Mesh, "file"),
                           P::Str::ToUtf8(file));
  }
  return m;
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Material>& AddMaterial(DocOf<P>& model,
                                                     ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Material>(EnsureAsset<P>(model),
                                                        name);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Texture>& AddTexture(DocOf<P>& model,
                                                   ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Texture>(EnsureAsset<P>(model),
                                                       name);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Hfield>& AddHfield(DocOf<P>& model,
                                                 ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Hfield>(EnsureAsset<P>(model),
                                                      name);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Skin>& AddSkin(DocOf<P>& model,
                                             ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Skin>(EnsureAsset<P>(model), name);
}

// --- Actuators ------------------------------------------------------------ //

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Actuator>& EnsureActuatorSection(
    DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Actuator>>(model);
}

// Add an actuator of a specific spelling (Position, Motor, Velocity, ...). When
// the spelling has a joint transmission and `joint` is non-empty it is set as
// the target; other transmissions (site/tendon/body) are set on the returned
// reference.
template <class A, class P = plain::Plain>
A& AddActuator(DocOf<P>& model, ViewOf<P> joint = {}, ViewOf<P> name = {}) {
  A& a = detail::Create<P, A>(EnsureActuatorSection<P>(model));
  if (!P::Str::Empty(name)) detail::SetName<P>(a, name);
  if (!P::Str::Empty(joint)) {
    const int id = detail::FieldIdByName(ElementTypeOf<P, A>, "joint");
    if (id >= 0) SetRefByField<P>(a, id, joint);
  }
  return a;
}

// --- Sensors -------------------------------------------------------------- //

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Sensor>& EnsureSensorSection(DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Sensor>>(model);
}

template <class S, class P = plain::Plain>
S& AddSensor(DocOf<P>& model, ViewOf<P> name = {}) {
  S& s = detail::Create<P, S>(EnsureSensorSection<P>(model));
  if (!P::Str::Empty(name)) detail::SetName<P>(s, name);
  return s;
}

// --- Contact / equality / tendon ------------------------------------------ //

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Contact>& EnsureContact(DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Contact>>(model);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Pair>& AddPair(DocOf<P>& model,
                                             ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Pair>(EnsureContact<P>(model), name);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Exclude>& AddExclude(DocOf<P>& model,
                                                   ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Exclude>(EnsureContact<P>(model),
                                                       name);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Equality>& EnsureEqualitySection(
    DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Equality>>(model);
}

template <class Q, class P = plain::Plain>
Q& AddEquality(DocOf<P>& model, ViewOf<P> name = {}) {
  Q& q = detail::Create<P, Q>(EnsureEqualitySection<P>(model));
  if (!P::Str::Empty(name)) detail::SetName<P>(q, name);
  return q;
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Tendon>& EnsureTendonSection(DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Tendon>>(model);
}

template <class TN, class P = plain::Plain>
TN& AddTendon(DocOf<P>& model, ViewOf<P> name = {}) {
  TN& t = detail::Create<P, TN>(EnsureTendonSection<P>(model));
  if (!P::Str::Empty(name)) detail::SetName<P>(t, name);
  return t;
}

// --- Keyframes ------------------------------------------------------------ //

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Keyframe>& EnsureKeyframe(DocOf<P>& model) {
  return detail::EnsureSection<P, ElementOf<P, mj::ElementType::Keyframe>>(model);
}

template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Key>& AddKey(DocOf<P>& model,
                                           ViewOf<P> name = {}) {
  return detail::AddNamed<P, mj::ElementType::Key>(EnsureKeyframe<P>(model), name);
}

// --- Defaults ------------------------------------------------------------- //

// The root `main` default, created if absent.
template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Default>& RootDefault(DocOf<P>& model) {
  using DefaultT = ElementOf<P, mj::ElementType::Default>;
  DefaultT* found = nullptr;
  P::Tree::template ForEachChildOfType<DefaultT>(model, [&](DefaultT& d) {
    if (found) return;
    std::optional<ViewOf<P>> n = detail::NameOf<P>(d);
    if (!n || P::Str::Empty(*n) || P::Str::EqualsUtf8(*n, "main")) found = &d;
  });
  if (found) return *found;
  DefaultT& d = detail::Create<P, DefaultT>(model);
  detail::SetName<P>(d, P::Str::View(P::Str::FromUtf8("main")));
  return d;
}

// A new named class nested under `main`.
template <class P = plain::Plain>
ElementOf<P, mj::ElementType::Default>& AddDefault(DocOf<P>& model,
                                                   ViewOf<P> name) {
  return detail::AddNamed<P, mj::ElementType::Default>(RootDefault<P>(model),
                                                       name);
}

// --- Primitive sizing ----------------------------------------------------- //

// Stamp a compilable default `size` onto a primitive geom from its type: a
// size-0 geom is a compile error for every non-mesh primitive, so `AddGeom`
// (which authors only what you pass) leaves it unset and this fills it.
// Only `size` is written -- no other default is stamped. Mesh/hfield/sdf geoms
// take their extent from the referenced asset and are left untouched.
template <class P = plain::Plain>
void SeedPrimitiveSize(ElementOf<P, mj::ElementType::Geom>& g) {
  const int type_id = detail::FieldIdByName(mj::ElementType::Geom, "type");
  const int size_id = detail::FieldIdByName(mj::ElementType::Geom, "size");
  const auto type = static_cast<mj::GeomType>(detail::GetEnumField<P>(
      g, type_id, static_cast<int>(mj::GeomType::sphere)));
  switch (type) {
    case mj::GeomType::sphere: {
      const double v[] = {0.1};
      detail::SetSeqField<P>(g, size_id, v, 1);
      break;
    }
    case mj::GeomType::capsule:
    case mj::GeomType::cylinder: {
      const double v[] = {0.05, 0.1};
      detail::SetSeqField<P>(g, size_id, v, 2);
      break;
    }
    case mj::GeomType::ellipsoid:
    case mj::GeomType::box: {
      const double v[] = {0.1, 0.1, 0.1};
      detail::SetSeqField<P>(g, size_id, v, 3);
      break;
    }
    case mj::GeomType::plane: {
      const double v[] = {1, 1, 0.1};
      detail::SetSeqField<P>(g, size_id, v, 3);
      break;
    }
    default:
      break;  // mesh / hfield / sdf: geometry comes from the asset
  }
}

// Add a geom already carrying a compilable size for its primitive type -- the
// one-call "give me a box I can simulate" a bare AddGeom deliberately does not.
template <class P = plain::Plain, class Parent>
ElementOf<P, mj::ElementType::Geom>& AddPrimitive(Parent& parent,
                                                  mj::GeomType type,
                                                  ViewOf<P> name = {}) {
  auto& g = AddGeom<P>(parent, type, name);
  SeedPrimitiveSize<P>(g);
  return g;
}

// --- Appearance: material layers + texture sources ------------------------ //

// A material binds textures through an ordered <layer> list (one texture per
// role). These author that list without the consumer touching the storage.
template <class P = plain::Plain>
ElementOf<P, mj::ElementType::MaterialLayer>& AddMaterialLayer(
    ElementOf<P, mj::ElementType::Material>& mat, ViewOf<P> role = {},
    ViewOf<P> texture = {}) {
  auto& layer =
      detail::Create<P, ElementOf<P, mj::ElementType::MaterialLayer>>(mat);
  const std::string r = P::Str::Empty(role) ? std::string("rgb")
                                            : P::Str::ToUtf8(role);
  detail::SetStrField<P>(
      layer, detail::FieldIdByName(mj::ElementType::MaterialLayer, "role"), r);
  if (!P::Str::Empty(texture)) {
    SetRefByField<P>(
        layer, detail::FieldIdByName(mj::ElementType::MaterialLayer, "texture"),
        texture);
  }
  return layer;
}

template <class P = plain::Plain>
void SetLayerTexture(ElementOf<P, mj::ElementType::MaterialLayer>& layer,
                     ViewOf<P> texture) {
  SetRefByField<P>(
      layer, detail::FieldIdByName(mj::ElementType::MaterialLayer, "texture"),
      texture);
}

template <class P = plain::Plain>
void SetLayerRole(ElementOf<P, mj::ElementType::MaterialLayer>& layer,
                  ViewOf<P> role) {
  detail::SetStrField<P>(
      layer, detail::FieldIdByName(mj::ElementType::MaterialLayer, "role"),
      P::Str::ToUtf8(role));
}

template <class P = plain::Plain>
bool RemoveMaterialLayer(ElementOf<P, mj::ElementType::Material>& mat,
                         std::size_t index) {
  using Layer = ElementOf<P, mj::ElementType::MaterialLayer>;
  std::size_t i = 0;
  const void* target = nullptr;
  P::Tree::template ForEachChildOfType<Layer>(mat, [&](Layer& l) {
    if (i++ == index) target = &l;
  });
  if (target == nullptr) return false;
  return P::Tree::Remove(mat, target);
}

// Point a texture element at an on-disk file, or at a procedural builtin. The
// two are sibling attributes MuJoCo reads independently, and a file wins over a
// builtin, so each setter clears the other spelling.
template <class P = plain::Plain>
void SetTextureFile(ElementOf<P, mj::ElementType::Texture>& tex,
                    ViewOf<P> file) {
  detail::SetStrField<P>(
      tex, detail::FieldIdByName(mj::ElementType::Texture, "file"),
      P::Str::ToUtf8(file));
  detail::ClearField<P>(
      tex, detail::FieldIdByName(mj::ElementType::Texture, "builtin"));
}

template <class P = plain::Plain>
void SetTextureBuiltin(ElementOf<P, mj::ElementType::Texture>& tex,
                       mj::TextureBuiltin builtin) {
  detail::SetEnumField<P>(
      tex, detail::FieldIdByName(mj::ElementType::Texture, "builtin"),
      static_cast<int>(builtin));
  detail::ClearField<P>(
      tex, detail::FieldIdByName(mj::ElementType::Texture, "file"));
}

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_BUILDERS_H
