// Property tests for the ProtoSpec SDK (milestone 6).
//
// Builds the milestone-exit robot programmatically through the typed builders,
// round-trips it through the MJCF writer/reader as a fixpoint check, then
// exercises the ergonomic layer: traversal + parent map + path, typed-reference
// resolve / find-referrers / rename / recursive-delete, the default-class
// operations (Effective / FlattenDefaults / ExtractClass), and namespaced
// attach. MuJoCo-free: correctness is asserted against the reader fixpoint and
// against the generated value-equality, not against a live compile.

#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "mjcf.h"
#include "protospec/sdk.h"
#include "protospec/save.h"
#include "types.h"

using namespace ps::mjcf;
namespace sdk = ps::sdk;

static int g_failed = 0;
static int g_checks = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    ++g_checks;                                                         \
    if (!(cond)) {                                                      \
      ++g_failed;                                                       \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
    }                                                                   \
  } while (0)

// --- The milestone-6 exit robot, built entirely through the builders ------ //
//
// world -> torso (freejoint + geom)
//            -> thigh  (childclass "leg"): hinge "hip",  geom "thigh_geom"
//                 -> shin (childclass inherited): hinge "knee", geom "shin_geom"
// assets:   material "steel"
// defaults: class "leg" -> geom{size,rgba}, joint{damping}
// actuators: position "act_hip" -> joint "hip"; position "act_knee" -> "knee"
// sensors:   jointpos "sens_hip" -> "hip"; jointvel "sens_knee" -> "knee"
static std::unique_ptr<Model> BuildRobot() {
  auto m = std::make_unique<Model>();
  m->model = "robot";

  // Default class "leg": shared geom size/rgba + joint damping.
  Default& leg = sdk::AddDefault(*m, "leg");
  {
    auto g = std::make_unique<Geom>();
    g->size = ps::InlineVec<double, 3>{0.05};
    g->rgba = std::array<float, 4>{{1.0f, 0.0f, 0.0f, 1.0f}};
    leg.geom.push_back(std::move(g));
    auto j = std::make_unique<Joint>();
    j->damping = ps::InlineVec<double, 3>{0.5};
    leg.joint.push_back(std::move(j));
  }

  sdk::AddMaterial(*m, "steel");

  Body& world = sdk::World(*m);
  Body& torso = sdk::AddBody(world, "torso");
  sdk::AddFreeJoint(torso);
  sdk::AddGeom(torso, GeomType::capsule, "torso_geom");

  Body& thigh = sdk::AddBody(torso, "thigh");
  thigh.childclass = ps::Ref<Default>("leg");
  sdk::AddJoint(thigh, JointType::hinge, "hip");
  Geom& thigh_geom = sdk::AddGeom(thigh, GeomType::capsule, "thigh_geom");
  thigh_geom.material = ps::Ref<Material>("steel");

  Body& shin = sdk::AddBody(thigh, "shin");
  sdk::AddJoint(shin, JointType::hinge, "knee");
  sdk::AddGeom(shin, GeomType::capsule, "shin_geom");

  Position& act_hip = sdk::AddActuator<Position>(*m, "hip", "act_hip");
  act_hip.kp = 100.0;
  Position& act_knee = sdk::AddActuator<Position>(*m, "knee", "act_knee");
  act_knee.kp = 80.0;

  Jointpos& s_hip = sdk::AddSensor<Jointpos>(*m, "sens_hip");
  s_hip.joint = ps::Ref<JointAny>("hip");
  Jointvel& s_knee = sdk::AddSensor<Jointvel>(*m, "sens_knee");
  s_knee.joint = ps::Ref<JointAny>("knee");

  return m;
}

// --- builders link elements into the right lists -------------------------- //
static void TestBuilders() {
  auto m = BuildRobot();

  // Body tree: world holds torso, torso holds a freejoint, a geom, and thigh.
  Body& world = sdk::World(*m);
  CHECK(world.subtree.size() == 1);  // torso
  Body* torso = sdk::Find<Body>(*m, "torso");
  CHECK(torso != nullptr);
  // torso subtree: FreeJoint, Geom, Body(thigh) in authored order.
  CHECK(torso->subtree.size() == 3);
  CHECK(torso->subtree[0].kind() == BodyChildAny::Kind::FreeJoint);
  CHECK(torso->subtree[1].kind() == BodyChildAny::Kind::Geom);
  CHECK(torso->subtree[2].kind() == BodyChildAny::Kind::Body);

  // Actuators + sensors landed in their single ordered union sections.
  CHECK(m->actuators.size() == 1);
  CHECK(m->actuators.front()->actuators.size() == 2);
  CHECK(m->actuators.front()->actuators[0].kind() ==
        ActuatorAny::Kind::Position);
  CHECK(m->sensors.front()->sensors.size() == 2);

  // Asset + default class present.
  CHECK(m->assets.front()->materials.size() == 1);
  CHECK(!m->defaults.empty());

  // Structural default mirrors the IDL: AddJoint default type is hinge.
  Body dummy_parent;
  Joint& j = sdk::AddJoint(dummy_parent);
  CHECK(j.type.has_value() && j.type.value() == JointType::hinge);
  // ...but no other IDL default is silently stamped on (DR-1).
  CHECK(!j.damping.has_value());

  // Equality builder lands in the section's ordered union list.
  Weld& weld = sdk::AddEquality<Weld>(*m, "weld1");
  CHECK(weld.name.has_value() && weld.name.value() == "weld1");
  CHECK(m->equalitys.size() == 1);
  CHECK(m->equalitys.front()->equalities.size() == 1);
  CHECK(m->equalitys.front()->equalities[0].kind() ==
        EqualityAny::Kind::Weld);
}

// --- writer/reader fixpoint ----------------------------------------------- //
static void TestFixpoint() {
  auto m = BuildRobot();
  std::string s1 = io::WriteMjcf(*m);
  io::ParseResult parsed = io::ParseMjcfString(s1, "<built>");
  if (!parsed.ok()) {
    for (const auto& e : parsed.errors) std::printf("  parse: %s\n",
                                                     e.Render().c_str());
  }
  CHECK(parsed.ok());
  if (parsed.ok()) {
    std::string s2 = io::WriteMjcf(*parsed.model);
    CHECK(s1 == s2);  // deterministic write -> read -> write fixpoint
  }
}

// --- traversal: find, parent map, path, ForEach --------------------------- //
static void TestTraversal() {
  auto m = BuildRobot();

  Joint* hip = sdk::Find<Joint>(*m, "hip");
  CHECK(hip != nullptr);
  CHECK(sdk::Find<Joint>(*m, "nope") == nullptr);

  sdk::ParentMap pm(*m);
  const void* parent = pm.ParentOf(*hip);
  CHECK(parent != nullptr);
  const sdk::ParentMap<>::Node* pn = pm.Lookup(parent);
  CHECK(pn && pn->type == ElementType::Body && pn->name == "thigh");

  std::string path = pm.PathTo(*hip);
  CHECK(path.find("thigh") != std::string::npos);
  CHECK(path.find("[hip]") != std::string::npos);

  // Recursive geom count under the world body: torso/thigh/shin geoms = 3.
  Body& world = sdk::World(*m);
  int geoms = 0;
  sdk::ForEachGeom(world, /*recursive=*/true, [&](Geom&) { ++geoms; });
  CHECK(geoms == 3);
  // Shallow: the world body directly holds no geoms (they are in torso).
  int shallow = 0;
  sdk::ForEachGeom(world, /*recursive=*/false, [&](Geom&) { ++shallow; });
  CHECK(shallow == 0);
  // torso directly holds exactly one geom, shallow.
  Body* torso = sdk::Find<Body>(*m, "torso");
  int torso_geoms = 0;
  sdk::ForEachGeom(*torso, false, [&](Geom&) { ++torso_geoms; });
  CHECK(torso_geoms == 1);
}

// --- refs: resolve + find referrers --------------------------------------- //
static void TestResolveAndReferrers() {
  auto m = BuildRobot();

  // Resolve the actuator's joint transmission to the actual joint. The joint
  // namespace has two declarers (<joint> and <freejoint>), so the reference is
  // a union ref and resolves to a handle naming the spelling it found.
  Position* act = sdk::Find<Position>(*m, "act_hip");
  CHECK(act && act->joint.has_value());
  auto h = sdk::Resolve(*m, *act->joint);
  CHECK(h && h.type == ElementType::Joint);
  CHECK(h.ptr == sdk::Find<Joint>(*m, "hip"));

  // Referrers of "hip": the position actuator and the jointpos sensor.
  Joint* hipJ = sdk::Find<Joint>(*m, "hip");
  auto refs = sdk::FindReferrers(*m, *hipJ);
  CHECK(refs.size() == 2);
  bool from_act = false, from_sensor = false;
  for (const auto& r : refs) {
    CHECK(r.refname == "hip");
    CHECK(r.field == "joint");
    if (r.element.type == ElementType::Position) from_act = true;
    if (r.element.type == ElementType::Jointpos) from_sensor = true;
  }
  CHECK(from_act && from_sensor);

  // A material referrer: the thigh geom.
  Material* steel = sdk::Find<Material>(*m, "steel");
  auto mrefs = sdk::FindReferrers(*m, *steel);
  CHECK(mrefs.size() == 1);
  CHECK(mrefs.front().element.type == ElementType::Geom);
}

// --- rename updates every referrer ---------------------------------------- //
static void TestRename() {
  auto m = BuildRobot();
  Joint* hip = sdk::Find<Joint>(*m, "hip");

  sdk::RenameResult renamed = sdk::Rename(*m, *hip, "hip_renamed");
  CHECK(renamed.ok);
  CHECK(renamed.updated == 2);  // actuator + sensor
  CHECK(hip->name.value() == "hip_renamed");

  // Old name resolves to nothing; new name resolves; referrers followed it.
  CHECK(sdk::Find<Joint>(*m, "hip") == nullptr);
  CHECK(sdk::Find<Joint>(*m, "hip_renamed") == hip);
  Position* act = sdk::Find<Position>(*m, "act_hip");
  CHECK(act->joint->name == "hip_renamed");
  CHECK(sdk::FindReferrers(*m, "hip", ElementType::Joint).empty());
  CHECK(sdk::FindReferrers(*m, "hip_renamed", ElementType::Joint).size() == 2);

  // Renaming to the same name is a no-op.
  CHECK(sdk::Rename(*m, *hip, "hip_renamed") == 0);
}

// --- delete reports (and can cascade) dangling referrers ------------------ //
static void TestDelete() {
  auto m = BuildRobot();

  // Delete the thigh subtree: removes hip + shin + knee and their geoms. The
  // hip/knee actuators and sensors are left dangling.
  Body* thigh = sdk::Find<Body>(*m, "thigh");
  CHECK(thigh != nullptr);
  auto report = sdk::DeleteRecursive(*m, *thigh, /*cascade=*/false);
  CHECK(report.removed);
  CHECK(!report.cascaded);
  // 4 dangling refs: act_hip, act_knee, sens_hip, sens_knee.
  CHECK(report.dangling.size() == 4);
  CHECK(sdk::Find<Body>(*m, "thigh") == nullptr);
  CHECK(sdk::Find<Joint>(*m, "knee") == nullptr);
  // Not cascaded: the actuator still names the now-absent joint.
  Position* act = sdk::Find<Position>(*m, "act_hip");
  CHECK(act && act->joint.has_value());

  // Now delete an already-removed pointer target: a fresh model, cascade mode.
  auto m2 = BuildRobot();
  Body* thigh2 = sdk::Find<Body>(*m2, "thigh");
  auto rep2 = sdk::DeleteRecursive(*m2, *thigh2, /*cascade=*/true);
  CHECK(rep2.removed && rep2.cascaded);
  CHECK(rep2.dangling.size() == 4);
  // Cascade cleared the dangling transmissions.
  Position* act2 = sdk::Find<Position>(*m2, "act_hip");
  CHECK(act2 && !act2->joint.has_value());
  CHECK(sdk::FindReferrers(*m2, "hip", ElementType::Joint).empty());
}

// --- Effective / FlattenDefaults round-trip ------------------------------- //
static void TestEffectiveAndFlatten() {
  auto m = BuildRobot();

  // The thigh geom authors neither size nor rgba; they come from class "leg".
  Geom* tg = sdk::Find<Geom>(*m, "thigh_geom");
  CHECK(tg && !tg->size.has_value() && !tg->rgba.has_value());
  auto eff = sdk::Effective(*m, *tg);
  CHECK(eff->size.has_value() && eff->size->size() == 1 &&
        (*eff->size)[0] == 0.05);
  CHECK(eff->rgba.has_value() && (*eff->rgba)[0] == 1.0f);
  // material stayed authored on the element itself.
  CHECK(eff->material.has_value() && eff->material->name == "steel");

  // The hip joint inherits damping from class "leg" via the thigh childclass.
  Joint* hip = sdk::Find<Joint>(*m, "hip");
  auto heff = sdk::Effective(*m, *hip);
  CHECK(heff->damping.has_value() && (*heff->damping)[0] == 0.5);

  // Snapshot every live geom/joint's effective value, flatten, recompute:
  // effective values must be preserved (semantic equality) even though the
  // authored attributes moved from class to element and classes are gone.
  Body& world = sdk::World(*m);
  std::vector<Geom*> geoms;
  std::vector<Joint*> joints;
  sdk::ForEachGeom(world, true, [&](Geom& g) { geoms.push_back(&g); });
  sdk::ForEachJoint(world, true, [&](Joint& j) { joints.push_back(&j); });

  std::vector<std::unique_ptr<Geom>> before_g;
  for (Geom* g : geoms) before_g.push_back(sdk::Effective(*m, *g));
  std::vector<std::unique_ptr<Joint>> before_j;
  for (Joint* j : joints) before_j.push_back(sdk::Effective(*m, *j));

  sdk::FlattenDefaults(*m);

  // Classes and childclass references are gone; the values are baked in.
  CHECK(m->defaults.empty());
  Body* thigh = sdk::Find<Body>(*m, "thigh");
  CHECK(thigh && !thigh->childclass.has_value());
  CHECK(tg->size.has_value() && (*tg->size)[0] == 0.05);   // baked from class
  CHECK(tg->rgba.has_value() && (*tg->rgba)[0] == 1.0f);
  CHECK(hip->damping.has_value() && (*hip->damping)[0] == 0.5);

  // Effective values are unchanged across the flatten (semantic equality).
  for (std::size_t i = 0; i < geoms.size(); ++i) {
    auto after = sdk::Effective(*m, *geoms[i]);
    CHECK(*after == *before_g[i]);
  }
  for (std::size_t i = 0; i < joints.size(); ++i) {
    auto after = sdk::Effective(*m, *joints[i]);
    CHECK(*after == *before_j[i]);
  }
}

// --- ExtractClass factors shared authored values ------------------------- //
static void TestExtractClass() {
  auto m = std::make_unique<Model>();
  m->model = "extract";
  Body& world = sdk::World(*m);
  Body& b = sdk::AddBody(world, "b");

  // Three geoms sharing rgba (blue) and contype, but with distinct names/sizes.
  std::vector<Geom*> gs;
  for (int i = 0; i < 3; ++i) {
    Geom& g = sdk::AddGeom(b, GeomType::box, "g" + std::to_string(i));
    g.rgba = std::array<float, 4>{{0.0f, 0.0f, 1.0f, 1.0f}};
    g.contype = 2;
    g.size = ps::InlineVec<double, 3>{static_cast<double>(i) + 1.0};  // differs
    gs.push_back(&g);
  }

  Default* cls = sdk::ExtractClass(*m, gs, "blue");
  CHECK(cls != nullptr);
  // Shared fields moved to the class and cleared on each element.
  for (Geom* g : gs) {
    CHECK(!g->rgba.has_value());
    CHECK(!g->contype.has_value());
    CHECK(g->size.has_value());  // distinct value stayed on the element
    CHECK(g->dclass.has_value() && g->dclass->name == "blue");
  }
  CHECK(!cls->geom.empty());
  CHECK(cls->geom.front()->rgba.has_value() &&
        (*cls->geom.front()->rgba)[2] == 1.0f);
  CHECK(cls->geom.front()->contype.has_value() &&
        cls->geom.front()->contype.value() == 2);

  // Effective restores the extracted values through the class.
  auto eff = sdk::Effective(*m, *gs[0]);
  CHECK(eff->rgba.has_value() && (*eff->rgba)[2] == 1.0f);
  CHECK(eff->contype.has_value() && eff->contype.value() == 2);
  CHECK(eff->size.has_value() && (*eff->size)[0] == 1.0);
}

// --- multiple top-level <default> blocks merge into one `main` ------------- //
//
// After an <include> merges two files, Model.defaults holds several top-level
// <default> blocks. MuJoCo has one root default (`main`) and feeds every
// top-level section to it in document order: root-level partials merge into
// `main` (later blocks overwrite per field), nested classes share one flat
// namespace, and a class snapshots `main` as it stood when parsed. The builders
// only ever produce a single `main` block, so these fixtures are assembled by
// hand. Expected values below were cross-checked against mujoco 3.10.0
// mj_loadXML on the equivalent MJCF.

static Default& AddTopBlock(Model& m) {
  m.defaults.push_back(std::make_unique<Default>());
  return *m.defaults.back();
}
static Default& AddSubclass(Default& parent, const std::string& name) {
  auto d = std::make_unique<Default>();
  d->dclass = name;
  parent.subclasses.push_back(std::move(d));
  return *parent.subclasses.back();
}
static Geom& DefGeom(Default& d) {
  d.geom.push_back(std::make_unique<Geom>());
  return *d.geom.back();
}
static Joint& DefJoint(Default& d) {
  d.joint.push_back(std::make_unique<Joint>());
  return *d.joint.back();
}

static void TestMultiBlockDefaults() {
  // Two top-level blocks with distinct root partials and distinct classes.
  //   block0: geom{rgba=red}          class "a": geom{size=0.1}
  //   block1: joint{damping=0.5}      class "b": geom{size=0.2}
  // mj_loadXML: ga.rgba=red size=0.1, gb.rgba=red size=0.2, groot.rgba=red,
  // and the class-free joint gets damping 0.5 -- i.e. both blocks live in main.
  {
    auto m = std::make_unique<Model>();
    m->model = "multi";
    Default& b0 = AddTopBlock(*m);
    DefGeom(b0).rgba = std::array<float, 4>{{1.0f, 0.0f, 0.0f, 1.0f}};
    DefGeom(AddSubclass(b0, "a")).size = ps::InlineVec<double, 3>{0.1};
    Default& b1 = AddTopBlock(*m);
    DefJoint(b1).damping = ps::InlineVec<double, 3>{0.5};
    DefGeom(AddSubclass(b1, "b")).size = ps::InlineVec<double, 3>{0.2};

    Body& world = sdk::World(*m);
    Body& body = sdk::AddBody(world, "body");
    Joint& j0 = sdk::AddJoint(body, JointType::hinge, "j0");
    Geom& ga = sdk::AddGeom(body, GeomType::sphere, "ga");
    ga.dclass = ps::Ref<Default>("a");
    Geom& gb = sdk::AddGeom(body, GeomType::sphere, "gb");
    gb.dclass = ps::Ref<Default>("b");
    Geom& groot = sdk::AddGeom(body, GeomType::box, "groot");

    auto ega = sdk::Effective(*m, ga);
    CHECK(ega->rgba.has_value() && (*ega->rgba)[0] == 1.0f &&
          (*ega->rgba)[2] == 0.0f);              // red, from block0's main merge
    CHECK(ega->size.has_value() && (*ega->size)[0] == 0.1);  // from class "a"

    auto egb = sdk::Effective(*m, gb);
    CHECK(egb->rgba.has_value() && (*egb->rgba)[0] == 1.0f);  // block0 -> main
    CHECK(egb->size.has_value() && (*egb->size)[0] == 0.2);   // from class "b"

    auto egr = sdk::Effective(*m, groot);
    CHECK(egr->rgba.has_value() && (*egr->rgba)[0] == 1.0f);  // class-free -> main

    auto ej = sdk::Effective(*m, j0);
    CHECK(ej->damping.has_value() && (*ej->damping)[0] == 0.5);  // block1 -> main
  }

  // Unnamed + unnamed: disjoint root partials both land in main.
  //   block0: geom{rgba=red}   block1: geom{size=0.3}
  // mj_loadXML: class-free geom -> rgba=red, size=0.3.
  {
    auto m = std::make_unique<Model>();
    Default& b0 = AddTopBlock(*m);
    DefGeom(b0).rgba = std::array<float, 4>{{1.0f, 0.0f, 0.0f, 1.0f}};
    Default& b1 = AddTopBlock(*m);
    DefGeom(b1).size = ps::InlineVec<double, 3>{0.3};

    Body& body = sdk::AddBody(sdk::World(*m), "body");
    Geom& g = sdk::AddGeom(body, GeomType::sphere, "g");
    auto e = sdk::Effective(*m, g);
    CHECK(e->rgba.has_value() && (*e->rgba)[0] == 1.0f);  // block0
    CHECK(e->size.has_value() && (*e->size)[0] == 0.3);   // block1
  }

  // Same field in two blocks: the LATER block wins in main (the XML reader
  // overwrites the shared main default in document order).
  //   block0: geom{rgba=red, size=0.05}   block1: geom{rgba=blue}
  // mj_loadXML: class-free geom -> rgba=blue, size=0.05.
  {
    auto m = std::make_unique<Model>();
    Default& b0 = AddTopBlock(*m);
    Geom& g0 = DefGeom(b0);
    g0.rgba = std::array<float, 4>{{1.0f, 0.0f, 0.0f, 1.0f}};
    g0.size = ps::InlineVec<double, 3>{0.05};
    Default& b1 = AddTopBlock(*m);
    DefGeom(b1).rgba = std::array<float, 4>{{0.0f, 0.0f, 1.0f, 1.0f}};

    Body& body = sdk::AddBody(sdk::World(*m), "body");
    Geom& g = sdk::AddGeom(body, GeomType::sphere, "g");
    auto e = sdk::Effective(*m, g);
    CHECK(e->rgba.has_value() && (*e->rgba)[2] == 1.0f &&
          (*e->rgba)[0] == 0.0f);                            // blue (block1 wins)
    CHECK(e->size.has_value() && (*e->size)[0] == 0.05);     // block0's field
  }

  // Parse-time snapshot: a class defined in block0 does NOT inherit a root-level
  // field authored by a LATER block1 (mjCModel::AddDefault -> CopyWithoutChildren
  // snapshots main when the class is parsed).
  //   block0: class "a": geom{size=0.05}   block1: geom{rgba=green}
  // mj_loadXML: geom class="a" -> rgba is the compiler default gray, NOT green.
  // At the SDK class layer (IDL defaults off) rgba stays unset.
  {
    auto m = std::make_unique<Model>();
    Default& b0 = AddTopBlock(*m);
    DefGeom(AddSubclass(b0, "a")).size = ps::InlineVec<double, 3>{0.05};
    Default& b1 = AddTopBlock(*m);
    DefGeom(b1).rgba = std::array<float, 4>{{0.0f, 1.0f, 0.0f, 1.0f}};

    Body& body = sdk::AddBody(sdk::World(*m), "body");
    Geom& g = sdk::AddGeom(body, GeomType::sphere, "g");
    g.dclass = ps::Ref<Default>("a");
    auto e = sdk::Effective(*m, g, /*apply_idl_defaults=*/false);
    CHECK(e->size.has_value() && (*e->size)[0] == 0.05);  // from class "a"
    CHECK(!e->rgba.has_value());  // block1's root rgba never reaches class "a"
  }

  // Duplicate class name across blocks: MuJoCo rejects this model ("repeated
  // default class name"); the SDK does not validate and deterministically keeps
  // the first occurrence.
  //   block0: class "a": geom{size=0.1}   block1: class "a": geom{size=0.2}
  {
    auto m = std::make_unique<Model>();
    Default& b0 = AddTopBlock(*m);
    DefGeom(AddSubclass(b0, "a")).size = ps::InlineVec<double, 3>{0.1};
    Default& b1 = AddTopBlock(*m);
    DefGeom(AddSubclass(b1, "a")).size = ps::InlineVec<double, 3>{0.2};

    Body& body = sdk::AddBody(sdk::World(*m), "body");
    Geom& g = sdk::AddGeom(body, GeomType::sphere, "g");
    g.dclass = ps::Ref<Default>("a");
    auto e = sdk::Effective(*m, g);
    CHECK(e->size.has_value() && (*e->size)[0] == 0.1);  // first "a" kept
  }
}

// --- attach: namespaced deep-clone splice --------------------------------- //
static void TestAttach() {
  auto m = BuildRobot();

  // Build a small standalone gripper subtree to graft onto the shin.
  Body gripper;
  gripper.name = "palm";
  sdk::AddJoint(gripper, JointType::slide, "slide");
  Geom& gg = sdk::AddGeom(gripper, GeomType::box, "pad");
  (void)gg;
  Body& finger = sdk::AddBody(gripper, "finger");
  sdk::AddGeom(finger, GeomType::box, "tip");

  Body* shin = sdk::Find<Body>(*m, "shin");
  CHECK(shin != nullptr);
  auto result = sdk::Attach(*m, *shin, gripper, "g1_");
  CHECK(result.ok);
  CHECK(result.collisions.empty());
  CHECK(result.attached != nullptr);

  // Every name in the graft is prefixed; the source is untouched (deep clone).
  CHECK(sdk::Find<Body>(*m, "g1_palm") != nullptr);
  CHECK(sdk::Find<Body>(*m, "g1_finger") != nullptr);
  CHECK(sdk::Find<Geom>(*m, "g1_pad") != nullptr);
  CHECK(sdk::Find<Geom>(*m, "g1_tip") != nullptr);
  CHECK(sdk::Find<Joint>(*m, "g1_slide") != nullptr);
  CHECK(gripper.name.value() == "palm");  // source unchanged

  // Attaching the same subtree again with the same prefix collides.
  auto dup = sdk::Attach(*m, *shin, gripper, "g1_");
  CHECK(!dup.ok);
  CHECK(!dup.collisions.empty());
  // A different prefix succeeds.
  auto ok2 = sdk::Attach(*m, *shin, gripper, "g2_");
  CHECK(ok2.ok);
  CHECK(sdk::Find<Joint>(*m, "g2_slide") != nullptr);

  // An internal reference inside a cloned subtree is namespaced too: give the
  // gripper geom a material ref and attach; the clone's ref is prefixed.
  Body refbody;
  refbody.name = "rb";
  Geom& rg = sdk::AddGeom(refbody, GeomType::sphere, "rgeom");
  rg.material = ps::Ref<Material>("mat");
  auto r3 = sdk::Attach(*m, *shin, refbody, "p_");
  CHECK(r3.ok);
  Geom* cloned = sdk::Find<Geom>(*m, "p_rgeom");
  CHECK(cloned && cloned->material.has_value() &&
        cloned->material->name == "p_mat");
}

// --- save: Save round-trip + SaveAs asset externalization ----------------- //

namespace fs = std::filesystem;

// A fresh, unique temp directory for one save test (removed at scope exit).
struct TempDir {
  fs::path path;
  explicit TempDir(const char* tag) {
    std::error_code ec;
    fs::path base = fs::temp_directory_path(ec);
    path = base / ("ps_sdk_save_" + std::string(tag) + "_" +
                   std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    fs::create_directories(path, ec);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

// Save(model) -> ParseMjcf(file) -> deep-equal + deterministic-bytes fixpoint.
static void TestSaveRoundtrip() {
  auto m = BuildRobot();
  TempDir tmp("roundtrip");
  const fs::path out = tmp.path / "robot.xml";

  CHECK(sdk::Save(*m, out));
  CHECK(fs::exists(out));

  io::ParseResult reloaded = io::ParseMjcfFile(out.string());
  if (!reloaded.ok()) {
    for (const auto& e : reloaded.errors)
      std::printf("  parse: %s\n", e.Render().c_str());
  }
  CHECK(reloaded.ok());
  if (reloaded.ok()) {
    // Deep equality (generated operator==) between the built tree and the tree
    // read back off disk, plus writer-byte determinism.
    CHECK(*m == *reloaded.model);
    CHECK(io::WriteMjcf(*m) == io::WriteMjcf(*reloaded.model));
  }
}

// SaveAs with in-memory mesh bytes: the bytes land on disk under the model's
// meshdir, the assets vector is drained, and the reloaded model resolves the
// mesh to that on-disk file (the precondition the compiler's base_dir/meshdir
// resolution needs -- full mjModel compile of on-disk-mesh models is exercised
// by the bridge corpus suite and the ASan corpus pass).
static void TestSaveAsExternalizesAssets() {
  auto m = std::make_unique<Model>();
  m->model = "meshbot";
  // Author a meshdir so assets externalize into a subdirectory, exercising the
  // ModelAssetDir resolution rather than the beside-the-xml default.
  {
    auto c = std::make_unique<Compiler>();
    c->meshdir = "assets";
    m->compilers.push_back(std::move(c));
  }
  const std::string basename = "cube.stl";
  sdk::AddMesh(*m, "cube", basename);
  Body& world = sdk::World(*m);
  Body& b = sdk::AddBody(world, "cube_body");
  Geom& g = sdk::AddGeom(b, GeomType::mesh, "cube_geom");
  g.mesh = ps::Ref<Mesh>("cube");

  // Representative binary asset payload (opaque to the SDK; byte-exact on disk).
  std::vector<std::uint8_t> bytes;
  for (int i = 0; i < 256; ++i) bytes.push_back(static_cast<std::uint8_t>(i));
  std::vector<sdk::InMemoryAsset> assets{sdk::InMemoryAsset{basename, bytes}};

  TempDir tmp("saveas");
  const fs::path out = tmp.path / "meshbot.xml";
  CHECK(sdk::SaveAs(*m, out, &assets));

  // Assets drained once externalized.
  CHECK(assets.empty());

  // The bytes landed under <xmldir>/assets/cube.stl, byte-for-byte.
  const fs::path asset = tmp.path / "assets" / basename;
  CHECK(fs::exists(asset));
  {
    std::ifstream in(asset, std::ios::binary);
    std::vector<std::uint8_t> on_disk((std::istreambuf_iterator<char>(in)),
                                      std::istreambuf_iterator<char>());
    CHECK(on_disk == bytes);
  }

  // ModelAssetDir agrees with where SaveAs put the file.
  CHECK(sdk::ModelAssetDir(*m, out) == (tmp.path / "assets"));

  // Reload: the model parses and the mesh resolves to the on-disk basename.
  io::ParseResult reloaded = io::ParseMjcfFile(out.string());
  if (!reloaded.ok()) {
    for (const auto& e : reloaded.errors)
      std::printf("  parse: %s\n", e.Render().c_str());
  }
  CHECK(reloaded.ok());
  if (reloaded.ok()) {
    Mesh* rm = sdk::Find<Mesh>(*reloaded.model, "cube");
    CHECK(rm != nullptr && rm->file.has_value() && *rm->file == basename);
    CHECK(*m == *reloaded.model);
  }

  // SaveAs with a null asset list is exactly Save (no throw, file written).
  const fs::path out2 = tmp.path / "meshbot2.xml";
  CHECK(sdk::SaveAs(*m, out2, nullptr));
  CHECK(fs::exists(out2));
}

// --- runtime-typed (pointer-keyed) rename / delete ------------------------ //
static void TestRuntimeRenameDelete() {
  // Rename keyed on a type-erased element pointer follows referrers just like
  // the static-typed template, without instantiating it per element type.
  {
    auto m = BuildRobot();
    Joint* hip = sdk::Find<Joint>(*m, "hip");
    const void* p = hip;
    sdk::RenameResult renamed = sdk::Rename(*m, p, "hip2");
    CHECK(renamed.ok);
    CHECK(renamed.updated == 2);  // act_hip + sens_hip followed
    CHECK(hip->name.value() == "hip2");
    CHECK(sdk::Find<Joint>(*m, "hip") == nullptr);
    Position* act = sdk::Find<Position>(*m, "act_hip");
    CHECK(act && act->joint->name == "hip2");
    sdk::RenameResult noop = sdk::Rename(*m, p, "hip2");  // same name: no-op
    CHECK(noop.ok && noop.updated == 0);
    int not_in_model = 0;
    CHECK(!sdk::Rename(*m, &not_in_model, "x").ok);  // unknown pointer
  }

  // DeleteSubtree reports dangling referrers and (with cascade) clears them.
  {
    auto m = BuildRobot();
    Body* thigh = sdk::Find<Body>(*m, "thigh");
    auto rep = sdk::DeleteSubtree(*m, static_cast<const void*>(thigh),
                                  /*cascade=*/false);
    CHECK(rep.removed && !rep.cascaded);
    CHECK(rep.dangling.size() == 4);  // act_hip/knee + sens_hip/knee
    CHECK(sdk::Find<Body>(*m, "thigh") == nullptr);
    CHECK(sdk::Find<Joint>(*m, "knee") == nullptr);
    Position* act = sdk::Find<Position>(*m, "act_hip");
    CHECK(act && act->joint.has_value());  // not cascaded: still dangling

    int not_in_model = 0;
    auto rep2 = sdk::DeleteSubtree(*m, &not_in_model, false);
    CHECK(!rep2.removed && rep2.dangling.empty());
  }
  {
    auto m = BuildRobot();
    Body* thigh = sdk::Find<Body>(*m, "thigh");
    auto rep = sdk::DeleteSubtree(*m, static_cast<const void*>(thigh),
                                  /*cascade=*/true);
    CHECK(rep.removed && rep.cascaded && rep.dangling.size() == 4);
    Position* act = sdk::Find<Position>(*m, "act_hip");
    CHECK(act && !act->joint.has_value());  // cascade cleared the transmission
    CHECK(sdk::FindReferrers(*m, "hip", ElementType::Joint).empty());
  }
}

// --- Duplicate: deep clone as next sibling, re-uniqued + ref-remapped ------ //
static void TestDuplicate() {
  auto m = BuildRobot();
  Body* thigh = sdk::Find<Body>(*m, "thigh");
  CHECK(thigh != nullptr);
  const std::uint64_t thigh_serial = thigh->serial;

  auto* clone = sdk::Duplicate(*m, thigh).As<Body>();
  CHECK(clone != nullptr);
  CHECK(clone->serial != thigh_serial);  // fresh serials (generated Clone)

  // The clone is the immediate next sibling of the original in the torso subtree.
  Body* torso = sdk::Find<Body>(*m, "torso");
  std::size_t bodies = 0, clone_idx = 0, thigh_idx = 0, idx = 0;
  for (auto& item : torso->subtree) {
    if (auto* p = std::get_if<std::unique_ptr<Body>>(&item.node)) {
      if (p->get() == thigh) thigh_idx = idx;
      if (p->get() == clone) clone_idx = idx;
      ++bodies;
    }
    ++idx;
  }
  CHECK(bodies == 2);                    // thigh + its clone
  CHECK(clone_idx == thigh_idx + 1);     // clone directly follows the original

  // Names inside the clone are re-uniqued (the whole subtree), the originals kept.
  CHECK(sdk::Find<Body>(*m, "thigh") == thigh);
  CHECK(sdk::Find<Body>(*m, "thigh_1") == clone);
  CHECK(sdk::Find<Joint>(*m, "hip") != nullptr);
  CHECK(sdk::Find<Joint>(*m, "hip_1") != nullptr);   // cloned hip re-uniqued
  CHECK(sdk::Find<Body>(*m, "shin_1") != nullptr);   // nested body re-uniqued
  CHECK(sdk::Find<Geom>(*m, "shin_geom_1") != nullptr);

  // A reference INTERNAL to the clone is remapped to the clone's new name;
  // a reference OUT of the clone is preserved. The thigh_geom -> "steel"
  // material ref points outside the clone, so the clone still names "steel".
  Geom* clone_geom = sdk::Find<Geom>(*m, "thigh_geom_1");
  CHECK(clone_geom && clone_geom->material.has_value() &&
        clone_geom->material->name == "steel");  // external ref preserved

  // Original actuators/sensors still name the ORIGINAL joints (refs outside the
  // clone were not touched, and the clone's joints got fresh names).
  Position* act = sdk::Find<Position>(*m, "act_hip");
  CHECK(act && act->joint->name == "hip");

  CHECK(!sdk::Duplicate(*m, nullptr).ok);  // unknown pointer
}

// --- Duplicate remaps an intra-subtree reference to the clone's new name --- //
static void TestDuplicateInternalRef() {
  auto m = std::make_unique<Model>();
  m->model = "dup";
  Body& world = sdk::World(*m);
  Body& rig = sdk::AddBody(world, "rig");
  Body& watched = sdk::AddBody(rig, "watched");
  sdk::AddGeom(watched, GeomType::sphere, "ball");
  Camera& eye = sdk::AddCamera(rig, "eye");
  eye.target = ps::Ref<Body>("watched");  // internal: targets a body in the rig

  auto* clone = sdk::Duplicate(*m, &rig).As<Body>();
  CHECK(clone != nullptr);
  CHECK(sdk::Find<Body>(*m, "rig_1") == clone);
  Body* watched_1 = sdk::Find<Body>(*m, "watched_1");
  Camera* eye_1 = sdk::Find<Camera>(*m, "eye_1");
  CHECK(watched_1 != nullptr && eye_1 != nullptr);

  // The clone's camera target was remapped to the clone's renamed body -- the
  // reference is INTERNAL to the subtree, so it follows the re-unique.
  CHECK(eye_1->target.has_value() && eye_1->target->name == "watched_1");
  // The original camera still targets the original body (untouched).
  CHECK(eye.target.has_value() && eye.target->name == "watched");

  // Editing the clone does not touch the original (deep, independent copy).
  clone->pos = std::array<double, 3>{1, 2, 3};
  CHECK(!rig.pos.has_value());
}

// --- Reparent: pure-tree move, cycle/target rejection --------------------- //
static void TestReparent() {
  auto m = BuildRobot();
  Body* torso = sdk::Find<Body>(*m, "torso");
  Body* thigh = sdk::Find<Body>(*m, "thigh");
  Body* shin = sdk::Find<Body>(*m, "shin");
  CHECK(torso && thigh && shin);

  // Move `shin` (currently under thigh) up to the world body.
  Body& world = sdk::World(*m);
  auto r = sdk::Reparent(*m, shin, &world);
  CHECK(r.ok && r.reason.empty());
  // shin is no longer under thigh; it is now a direct child of world.
  bool shin_under_thigh = false, shin_under_world = false;
  sdk::ForEachBody(*thigh, /*recursive=*/false,
                   [&](Body& b) { if (&b == shin) shin_under_thigh = true; });
  sdk::ForEachBody(world, /*recursive=*/false,
                   [&](Body& b) { if (&b == shin) shin_under_world = true; });
  CHECK(!shin_under_thigh);
  CHECK(shin_under_world);
  // Pure tree op: the moved element's authored local pose is left exactly as it
  // was (no compile-aware world-pose fixup) -- shin authored no pos, still none.
  CHECK(!shin->pos.has_value());

  // Reject: reparent `torso` into its own descendant `thigh` (a cycle).
  auto cyc = sdk::Reparent(*m, torso, thigh);
  CHECK(!cyc.ok && !cyc.reason.empty());

  // Reject: a non-body/frame target (a Geom is not a container).
  Geom* g = sdk::Find<Geom>(*m, "thigh_geom");
  auto bad = sdk::Reparent(*m, thigh, g);
  CHECK(!bad.ok);

  // Reject: an element that is not a body-context child (a model-level material).
  Material* steel = sdk::Find<Material>(*m, "steel");
  auto notmov = sdk::Reparent(*m, steel, &world);
  CHECK(!notmov.ok);

  // nullptr target == the world body: move thigh to world too.
  auto r2 = sdk::Reparent(*m, thigh, nullptr);
  CHECK(r2.ok);
}

// Schema-drift guards: these must stay empty as the schema evolves. A failure
// means the generated family table missed a defaultable family (class-merge
// would silently skip it) or the schema grew a referenceable runtime family with
// no dynamic keyword (rename/delete/referrer scan would silently skip it). Plus
// the type-erased lookup / prune verbs and the SetRef target-type contract.
static void TestSchemaInvariants() {
  CHECK(sdk::DefaultFamilyCoverageGaps().empty());
  CHECK(sdk::detail::DynRefKeywordGaps().empty());

  std::unique_ptr<Model> m = BuildRobot();

  // FindBySerial round-trips a live element's serial to its pointer + type.
  Geom* shin = sdk::Find<Geom>(*m, "shin_geom");
  CHECK(shin != nullptr);
  const std::uint64_t s = shin->serial;
  CHECK(sdk::FindBySerial(*m, s) == static_cast<void*>(shin));
  sdk::Located loc = sdk::FindBySerialTyped(*m, s);
  CHECK(loc.ptr == static_cast<void*>(shin) && loc.type == ElementType::Geom);
  CHECK(sdk::FindBySerial(*m, 0) == nullptr);          // sentinel never matches
  CHECK(sdk::FindBySerial(*m, ~std::uint64_t{0}) == nullptr);  // absent

  // PruneSubtrees drops selected elements (subtrees included) with no ref fixup.
  int before = 0;
  sdk::ForEachOfType<Geom>(*m, [&](Geom&) { ++before; });
  sdk::PruneSubtrees(*m, [](const auto& e) {
    std::optional<std::string_view> n = sdk::Name(e);
    return n && *n == "shin_geom";
  });
  int after = 0;
  sdk::ForEachOfType<Geom>(*m, [&](Geom&) { ++after; });
  CHECK(after == before - 1);
  CHECK(sdk::Find<Geom>(*m, "shin_geom") == nullptr);

  // SetRef by target element: a concrete-type target sets the ref by name.
  Geom* thigh = sdk::Find<Geom>(*m, "thigh_geom");
  Material* steel = sdk::Find<Material>(*m, "steel");
  CHECK(thigh != nullptr && steel != nullptr);
  CHECK(sdk::SetRef(thigh->material, *steel));
  CHECK(thigh->material.has_value() && thigh->material->name == "steel");
}

// --- Promoted-from-editor helpers now in the SDK -------------------------- //
// The editor's generic model helpers (non-root walk, serial<->ptr<->name, unique
// naming, serial-preserving clone) are now public SDK verbs; the editor consumes
// these copies. Guard their contracts here.
static void TestEditorPromotions() {
  auto m = BuildRobot();

  // ForEachElement visits every element but NEVER the Model root.
  int roots = 0, elems = 0;
  sdk::ForEachElement(*m, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (std::is_same_v<E, Model>) ++roots;
    ++elems;
  });
  CHECK(roots == 0 && elems > 0);

  Geom* shin = sdk::Find<Geom>(*m, "shin_geom");
  CHECK(shin != nullptr);
  const std::uint64_t s = shin->serial;

  // SerialOf is the inverse of FindBySerial; the root and null map to 0.
  CHECK(sdk::SerialOf(*m, shin) == s);
  CHECK(sdk::SerialOf(*m, nullptr) == 0);
  CHECK(sdk::SerialOf(*m, m.get()) == 0);

  // FindBySerialAs<T> is the typed slice of the serial walk.
  CHECK(sdk::FindBySerialAs<Geom>(*m, s) == shin);
  CHECK(sdk::FindBySerialAs<Body>(*m, s) == nullptr);  // wrong type

  // NameOfSerial recovers an element's authored name from its serial.
  CHECK(sdk::NameOfSerial(*m, s) == "shin_geom");
  CHECK(sdk::NameOfSerial(*m, 0).empty());

  // UniqueName namespaces by MuJoCo category: a free base is returned as-is; a
  // taken one is suffixed; joint spellings share one namespace.
  CHECK(sdk::UniqueName(*m, ElementType::Geom, "brand_new") == "brand_new");
  CHECK(sdk::UniqueName(*m, ElementType::Geom, "shin_geom") == "shin_geom_1");
  CHECK(sdk::UniqueName(*m, ElementType::FreeJoint, "hip") == "hip_1");

  // CloneModelWithSerials reproduces every serial (generated Clone mints fresh
  // ones) and preserves structure -- the undo-snapshot contract.
  std::unique_ptr<Model> snap = sdk::CloneModelWithSerials(*m);
  Geom* shin_c = sdk::Find<Geom>(*snap, "shin_geom");
  CHECK(shin_c != nullptr);
  CHECK(shin_c->serial == s);   // serial preserved
  CHECK(shin_c != shin);        // distinct object
  std::vector<std::uint64_t> a, b;
  sdk::ForEachElement(*m, [&](auto& e) {
    if constexpr (requires { e.serial; }) a.push_back(e.serial);
  });
  sdk::ForEachElement(*snap, [&](auto& e) {
    if constexpr (requires { e.serial; }) b.push_back(e.serial);
  });
  CHECK(a.size() == b.size() && a == b);  // bijection of serials
}

// --- Rename result-object contract (error-convention alignment) ----------- //
static void TestRenameResult() {
  auto m = BuildRobot();
  Joint* hip = sdk::Find<Joint>(*m, "hip");
  CHECK(hip != nullptr);

  // Success: ok, updated == referrer count, empty reason. Deprecated int/bool
  // conversions still hold for one-release source compat.
  sdk::RenameResult r = sdk::Rename(*m, *hip, "hip2");
  CHECK(r.ok && r.updated == 2 && r.reason.empty());
  CHECK(static_cast<bool>(r));
  CHECK(static_cast<int>(r) == 2);

  // Rejection: onto a name held by another element of the joint category.
  Joint* knee = sdk::Find<Joint>(*m, "knee");
  CHECK(knee != nullptr);
  sdk::RenameResult bad = sdk::Rename(*m, *knee, "hip2");
  CHECK(!bad.ok && !bad.reason.empty());
  CHECK(static_cast<int>(bad) == -1);        // legacy -1 sentinel preserved
  CHECK(knee->name.value() == "knee");        // model untouched

  // Runtime-pointer form: a pointer not in the model reports ok=false.
  int not_in_model = 0;
  sdk::RenameResult nf =
      sdk::Rename(*m, static_cast<const void*>(&not_in_model), "x");
  CHECK(!nf.ok && !nf.reason.empty());
}

// --- SetRef: positives and the negatives validation (not SetRef) catches --- //
static void TestSetRefNegatives() {
  auto m = BuildRobot();
  Geom* thigh = sdk::Find<Geom>(*m, "thigh_geom");
  CHECK(thigh != nullptr);

  // By name (string_view overload): a name is stored verbatim (no lookup here);
  // an EMPTY name clears the field.
  sdk::SetRef(thigh->material, std::string_view("steel"));
  CHECK(thigh->material.has_value() && thigh->material->name == "steel");
  CHECK(sdk::ResolveTo<Material>(*m, *thigh->material) != nullptr);
  sdk::SetRef(thigh->material, std::string_view{});
  CHECK(!thigh->material.has_value());

  // By name to a NON-EXISTENT target: SetRef sets it (it does no lookup); Resolve
  // reports nothing -- the negative the validator, not SetRef, is meant to catch.
  sdk::SetRef(thigh->material, std::string_view("no_such_material"));
  CHECK(thigh->material.has_value());
  CHECK(sdk::ResolveTo<Material>(*m, *thigh->material) == nullptr);
  sdk::ClearRef(thigh->material);
  CHECK(!thigh->material.has_value());

  // By target element: an UNNAMED target returns false and leaves the field
  // untouched (an unnamed element cannot be named in MJCF).
  Material& unnamed = sdk::AddMaterial(*m);  // no name
  CHECK(!sdk::SetRef(thigh->material, unnamed));
  CHECK(!thigh->material.has_value());

  // By target element: a valid named target returns true and sets the ref.
  Material* steel = sdk::Find<Material>(*m, "steel");
  CHECK(steel != nullptr);
  CHECK(sdk::SetRef(thigh->material, *steel));
  CHECK(thigh->material->name == "steel");

  // The mismatched-target overload -- e.g. sdk::SetRef(thigh->material, someBody)
  // -- is a COMPILE error (static_assert ref_accepts_target), documented in
  // public_api.md and therefore not exercisable at runtime.
}

int main() {
  TestBuilders();
  TestEditorPromotions();
  TestRenameResult();
  TestSetRefNegatives();
  TestFixpoint();
  TestSaveRoundtrip();
  TestSaveAsExternalizesAssets();
  TestTraversal();
  TestResolveAndReferrers();
  TestRename();
  TestDelete();
  TestRuntimeRenameDelete();
  TestDuplicate();
  TestDuplicateInternalRef();
  TestReparent();
  TestEffectiveAndFlatten();
  TestExtractClass();
  TestMultiBlockDefaults();
  TestAttach();
  TestSchemaInvariants();

  std::printf("%d checks, %d failures\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
