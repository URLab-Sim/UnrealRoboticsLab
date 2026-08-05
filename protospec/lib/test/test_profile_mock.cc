// The Phase 2 exit gate: the SDK, the MJCF reader and the MJCF writer run over a
// SECOND emission profile.
//
// The profile (mock_profile.h) is deliberately hostile -- it shares nothing with
// the plain profile except the schema itself. What this file asserts is that the
// algorithms do not notice: the same verbs, the same reader, the same writer,
// and, for the io half, byte-identical output to what the plain profile produces
// from the same source document. That last assertion is the strongest available,
// because it proves the two profiles are semantically interchangeable at the io
// boundary rather than merely both compiling.

#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "mjcf.h"
#include "mock_profile.h"
#include "protospec/attach.h"
#include "protospec/builders.h"
#include "protospec/classes.h"
#include "protospec/plain_profile.h"
#include "protospec/refs.h"
#include "protospec/traversal.h"

namespace sdk = ps::sdk;
namespace mj = ps::mjcf;
namespace io = ps::mjcf::io;
using mock::Mock;
using mock::MView;

// Declared by the mock's own reader/writer instantiation TU.
namespace mock {
io::ParseResultOf<Mock> ParseMock(const std::string& xml,
                                  const std::string& filename);
std::string WriteMock(const MModel& model);
}  // namespace mock

static int g_failed = 0;
static int g_checks = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    ++g_checks;                                                   \
    if (!(cond)) {                                                \
      ++g_failed;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

static MView V(std::string_view s) { return MView(s.data(), s.size()); }
static std::string S(MView v) { return std::string(mock::Utf8(v)); }

// A one-argument spelling, so a template argument list's comma cannot be read
// as a macro-argument separator inside CHECK.
template <class T>
static T* FindM(mock::MModel& m, std::string_view name) {
  return sdk::Find<T, Mock>(m, V(name));
}

// --------------------------------------------------------------------------- //
// The reference slot's aliasing invariant                                      //
// --------------------------------------------------------------------------- //
// A proxy that copied instead of aliasing would make Rename silently stop
// updating referrers, with no compile error and no failure on a small model.
// Move-only is what turns that mistake into a build break, in every profile.
static_assert(!std::is_copy_constructible_v<sdk::RefSlotAny<Mock>>,
              "a reference slot must not be copyable: a by-value capture would "
              "detach the write from storage");
static_assert(!std::is_copy_assignable_v<sdk::RefSlotAny<Mock>>);
static_assert(!std::is_copy_constructible_v<sdk::RefSlotAny<ps::sdk::plain::Plain>>);

// --------------------------------------------------------------------------- //
// SDK verbs                                                                     //
// --------------------------------------------------------------------------- //

static mock::MModel& BuildTree() {
  mock::MModel& m = *mock::NewNode<mock::MModel>();
  mock::MBody& world = sdk::World<Mock>(m);
  mock::MBody& torso = sdk::AddBody<Mock>(world, V("torso"));
  mock::MGeom& g1 = sdk::AddGeom<Mock>(torso, mj::GeomType::box, V("shell"));
  sdk::AddJoint<Mock>(torso, mj::JointType::hinge, V("hip"));
  mock::MBody& shin = sdk::AddBody<Mock>(torso, V("shin"));
  sdk::AddGeom<Mock>(shin, mj::GeomType::capsule, V("shin_geom"));

  mock::MMaterial& steel = sdk::AddMaterial<Mock>(m, V("steel"));
  sdk::AddMaterialLayer<Mock>(steel, V("rgb"), V("grid"));
  sdk::AddMesh<Mock>(m, V("hull"), V("hull.obj"));

  const int mat_id = sdk::detail::FieldIdByName(mj::ElementType::Geom, "material");
  sdk::SetRefByField<Mock>(g1, mat_id, V("steel"));
  return m;
}

static void TestSdkVerbs() {
  mock::MModel& m = BuildTree();

  // Walks see document order, not the mock's reversed storage order.
  std::vector<std::string> names;
  sdk::ForEachElement<Mock>(m, [&](const auto& e) {
    if (auto n = Mock::Name(e)) names.push_back(S(*n));
  });
  // Document order is the schema's child-slot order then sibling index: the
  // asset section (slot 7) precedes the world body (slot 8), and within a body
  // the interleaved children keep the order they were added in.
  CHECK(names.size() == 7);
  CHECK(names.size() == 7 && names[0] == "hull" && names[1] == "steel" &&
        names[2] == "torso" && names[3] == "shell" && names[4] == "hip" &&
        names[5] == "shin" && names[6] == "shin_geom");

  // Find + name access.
  mock::MGeom* shell = FindM<mock::MGeom>(m, "shell");
  CHECK(shell != nullptr);
  CHECK(FindM<mock::MBody>(m, "torso") != nullptr);
  CHECK(FindM<mock::MGeom>(m, "nope") == nullptr);

  // FindReferrers sees the material reference.
  auto refs = sdk::FindReferrers<Mock>(m, V("steel"), mj::ElementType::Material);
  CHECK(refs.size() == 1);
  CHECK(refs.size() == 1 && refs[0].field == "material");

  // Rename rewrites the referrer through the slot -- the contract change.
  auto ren = sdk::Rename<Mock>(m, *FindM<mock::MMaterial>(m, "steel"),
                               V("iron"));
  CHECK(ren.ok);
  CHECK(ren.updated == 1);
  const int mat_id = sdk::detail::FieldIdByName(mj::ElementType::Geom, "material");
  CHECK(S(Mock::Ref::NameAt(*shell, mat_id)) == "iron");

  // Rename rejects a collision inside the shared name namespace.
  auto bad = sdk::Rename<Mock>(m, *FindM<mock::MBody>(m, "shin"),
                               V("torso"));
  CHECK(!bad.ok);
  CHECK(!bad.reason.empty());

  // UniqueName folds on the MuJoCo name category.
  CHECK(S(Mock::Str::View(sdk::UniqueName<Mock>(m, mj::ElementType::Body,
                                                V("torso")))) == "torso_1");
  CHECK(S(Mock::Str::View(sdk::UniqueName<Mock>(m, mj::ElementType::Body,
                                                V("free")))) == "free");

  // Duplicate re-uniques names and remaps the clone's internal references.
  auto dup = sdk::Duplicate<Mock>(m, FindM<mock::MBody>(m, "shin"));
  CHECK(dup.ok);
  CHECK(FindM<mock::MBody>(m, "shin_1") != nullptr);
  CHECK(FindM<mock::MGeom>(m, "shin_geom_1") != nullptr);

  // Reparent moves a body-context child; a cycle is rejected.
  mock::MBody* torso = FindM<mock::MBody>(m, "torso");
  mock::MBody* shin1 = FindM<mock::MBody>(m, "shin_1");
  CHECK(!sdk::Reparent<Mock>(m, torso, shin1).ok);
  CHECK(sdk::Reparent<Mock>(m, shin1, nullptr).ok);

  // DeleteSubtree reports and cascades danglers.
  auto del = sdk::DeleteSubtree<Mock>(
      m, FindM<mock::MMaterial>(m, "iron"), /*cascade=*/true);
  CHECK(del.removed);
  CHECK(del.dangling.size() == 1);
  CHECK(del.cascaded);
  CHECK(!Mock::Ref::SlotAt(*shell, mat_id).IsSet());
}

static void TestAttachAndClasses() {
  mock::MModel& host = BuildTree();
  mock::MModel& src = BuildTree();

  // Attach prefixes every name and every internal reference of the clone.
  auto res = sdk::Attach<Mock>(host, sdk::World<Mock>(host),
                               *FindM<mock::MBody>(src, "torso"),
                               V("arm/"));
  CHECK(res.ok);
  CHECK(FindM<mock::MBody>(host, "arm/torso") != nullptr);
  mock::MGeom* shell = FindM<mock::MGeom>(host, "arm/shell");
  CHECK(shell != nullptr);
  const int mat_id = sdk::detail::FieldIdByName(mj::ElementType::Geom, "material");
  CHECK(shell != nullptr && S(Mock::Ref::NameAt(*shell, mat_id)) == "arm/steel");

  // A collision against the host blocks the splice and leaves it untouched.
  auto clash = sdk::Attach<Mock>(host, sdk::World<Mock>(host),
                                 *FindM<mock::MBody>(src, "torso"),
                                 V("arm/"));
  CHECK(!clash.ok);
  CHECK(!clash.collisions.empty());

  // Default classes: a class partial fills an element's unauthored field.
  mock::MModel& m = *mock::NewNode<mock::MModel>();
  mock::MBody& world = sdk::World<Mock>(m);
  mock::MGeom& g = sdk::AddGeom<Mock>(world, mj::GeomType::sphere, V("g"));
  mock::MDefault& cls = sdk::AddDefault<Mock>(m, V("big"));
  mock::MGeom& partial = sdk::detail::Create<Mock, mock::MGeom>(cls);
  const int size_id = sdk::detail::FieldIdByName(mj::ElementType::Geom, "size");
  const double sz[] = {0.25};
  sdk::detail::SetSeqField<Mock>(partial, size_id, sz, 1);
  const int dclass_id = sdk::detail::FieldIdByName(mj::ElementType::Geom, "dclass");
  sdk::SetRefByField<Mock>(g, dclass_id, V("big"));

  sdk::EffectiveContext<Mock> ctx(m);
  mock::MOpt<mock::MFixedList<double, 3>> size;
  CHECK(sdk::EffectiveField<Mock>(ctx, g, size_id, size));
  CHECK(size.IsSet() && size.Ref().Count() == 1);
  CHECK(size.IsSet() && size.Ref().Count() == 1 && size.Ref().Item(0) == 0.25);

  // The whole-element form agrees with the per-field form.
  mock::MGeom* eff = sdk::Effective<Mock>(ctx, g, /*apply_idl_defaults=*/false);
  CHECK(eff->size.IsSet() && eff->size.Ref().Item(0) == 0.25);

  // FlattenDefaults bakes the class in and drops the class tree.
  sdk::FlattenDefaults<Mock>(m);
  CHECK(g.size.IsSet() && g.size.Ref().Item(0) == 0.25);
  CHECK(!Mock::Ref::SlotAt(g, dclass_id).IsSet());
  CHECK(Mock::Tree::FirstChildOfType<mock::MDefault>(m) == nullptr);

  // ExtractClass factors a shared authored value back out.
  mock::MGeom& g2 = sdk::AddGeom<Mock>(world, mj::GeomType::sphere, V("g2"));
  sdk::detail::SetSeqField<Mock>(g2, size_id, sz, 1);
  std::vector<mock::MGeom*> fam{&g, &g2};
  mock::MDefault* made = sdk::ExtractClass<Mock>(m, fam, V("shared"));
  CHECK(made != nullptr);
  CHECK(!g.size.IsSet());
  CHECK(S(Mock::Ref::NameAt(g, dclass_id)) == "shared");
}

static void TestDynamicRef() {
  // A dynamic (target_from) reference: objname is typed by the runtime keyword
  // its objtype sibling holds. Nothing about it is element-specific -- the
  // schema's marking is what drives it.
  mock::MModel& m = *mock::NewNode<mock::MModel>();
  mock::MBody& world = sdk::World<Mock>(m);
  sdk::AddBody<Mock>(world, V("pelvis"));
  mock::MSensorSection& sec =
      sdk::detail::EnsureSection<Mock, mock::MSensorSection>(m);
  mock::MSensorPlugin& s = sdk::detail::Create<Mock, mock::MSensorPlugin>(sec);
  s.objtype.Put(mock::MStr("body"));
  s.objname.Put(mock::MStr("pelvis"));

  auto refs = sdk::FindReferrers<Mock>(m, V("pelvis"), mj::ElementType::Body);
  CHECK(refs.size() == 1);
  CHECK(refs.size() == 1 && refs[0].field == "objname");

  auto ren = sdk::Rename<Mock>(m, *FindM<mock::MBody>(m, "pelvis"),
                               V("root"));
  CHECK(ren.ok && ren.updated == 1);
  CHECK(s.objname.IsSet() && s.objname.Ref().Same(mock::MStr("root")));
}

// --------------------------------------------------------------------------- //
// io                                                                            //
// --------------------------------------------------------------------------- //

// Only attributes and elements the mock models; everything else would be a
// legitimate "unknown element" from the mock's point of view.
static const char* kXml = R"(<mujoco model="hostile">
  <default>
    <default class="big">
      <geom size="0.3"/>
    </default>
  </default>
  <asset>
    <mesh name="hull" file="hull.obj" scale="2 2 2"/>
    <material name="steel">
      <layer role="rgb" texture="grid"/>
    </material>
  </asset>
  <worldbody>
    <body name="torso" pos="0 0 1" euler="0 0 90">
      <joint name="hip" type="hinge" axis="0 0 1"/>
      <geom name="shell" type="box" size="0.1 0.2 0.3" material="steel" quat="0 1 0 0"/>
      <body name="shin" pos="0 0 -1">
        <geom name="shin_geom" type="capsule" size="0.05 0.1" mesh="hull" user="1 2 3"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <plugin name="s0" objtype="body" objname="torso"/>
  </sensor>
</mujoco>)";

static void TestReaderWriter() {
  auto parsed = mock::ParseMock(kXml, "hostile.xml");
  for (const auto& e : parsed.errors) std::printf("  %s\n", e.Render().c_str());
  CHECK(parsed.ok());
  if (!parsed.ok()) return;
  mock::MModel& m = *parsed.model;

  // Element count and document order, against the mock's reversed storage.
  int elems = 0;
  sdk::ForEachElement<Mock>(m, [&](const auto&) { ++elems; });
  CHECK(elems == 15);

  std::vector<std::string> order;
  sdk::ForEachElement<Mock>(m, [&](const auto& e) {
    if (auto n = Mock::Name(e)) order.push_back(S(*n));
  });
  // Document order over the mock's reversed storage: the class tree, the asset
  // section, the world body, then each body's interleaved children in the order
  // the XML declared them (the joint before the geom).
  CHECK(order.size() == 9);
  CHECK(order.size() == 9 && order[0] == "big" && order[1] == "hull" &&
        order[2] == "steel" && order[3] == "torso" && order[4] == "hip" &&
        order[5] == "shell" && order[6] == "shin" && order[7] == "shin_geom" &&
        order[8] == "s0");

  // References read through the profile's slots.
  mock::MGeom* shell = FindM<mock::MGeom>(m, "shell");
  CHECK(shell != nullptr);
  const int mat_id = sdk::detail::FieldIdByName(mj::ElementType::Geom, "material");
  CHECK(shell && S(Mock::Ref::NameAt(*shell, mat_id)) == "steel");

  // The deferred orientation fold wrote through the profile's presence plumbing
  // AND through the fixed-arity adapter, which permutes: the wire's
  // quat="0 1 0 0" is w=0,x=1 and the mock stores X,Y,Z,W.
  CHECK(shell && shell->quat.IsSet());
  CHECK(shell && shell->quat.IsSet() && shell->quat.Ref().W() == 0.0 &&
        shell->quat.Ref().X() == 1.0);
  // euler="0 0 90" folded to a quaternion, so the body's quat is authored even
  // though no quat attribute appears in the source.
  mock::MBody* torso = FindM<mock::MBody>(m, "torso");
  CHECK(torso && torso->quat.IsSet());
  CHECK(torso && torso->pos.IsSet() && torso->pos.Ref().c == 1.0);

  // The material layer read as a child, front-inserted position and all.
  mock::MMaterial* steel = FindM<mock::MMaterial>(m, "steel");
  CHECK(steel != nullptr);
  mock::MLayer* layer =
      steel ? Mock::Tree::FirstChildOfType<mock::MLayer>(*steel) : nullptr;
  CHECK(layer != nullptr);
  CHECK(layer && S(Mock::Ref::NameAt(*layer, 0)) == "grid");

  // Range and unbounded sequences.
  mock::MGeom* shin = FindM<mock::MGeom>(m, "shin_geom");
  CHECK(shin && shin->size.IsSet() && shin->size.Ref().Count() == 2);
  CHECK(shin && shin->user.IsSet() && shin->user.Ref().Count() == 3);

  // The strongest assertion available: the mock's writer output is byte-identical
  // to the plain profile's, from the same source document. If the two profiles
  // agreed only structurally this would fail on ordering, on the quaternion
  // permutation, or on presence.
  const std::string mock_xml = mock::WriteMock(m);
  io::ParseResult plain = io::ParseMjcfString(kXml, "hostile.xml");
  CHECK(plain.ok());
  const std::string plain_xml = plain.ok() ? io::WriteMjcf(*plain.model) : "";
  // Guard against a vacuous comparison of two empty strings, and pin the one
  // value the fixed-arity adapter had to permute on the way in and back out.
  CHECK(mock_xml.rfind("<mujoco", 0) == 0);
  CHECK(mock_xml.find("quat=\"0 1 0 0\"") != std::string::npos);
  CHECK(mock_xml == plain_xml);
  if (mock_xml != plain_xml) {
    std::printf("--- mock ---\n%s\n--- plain ---\n%s\n", mock_xml.c_str(),
                plain_xml.c_str());
  }

  // Round-trip fixpoint on the mock: re-reading its own output reproduces it.
  auto again = mock::ParseMock(mock_xml, "hostile.xml");
  CHECK(again.ok());
  if (again.ok()) CHECK(mock::WriteMock(*again.model) == mock_xml);
}

int main() {
  TestSdkVerbs();
  TestAttachAndClasses();
  TestDynamicRef();
  TestReaderWriter();
  std::printf("test_profile_mock: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
