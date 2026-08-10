// Standalone (MuJoCo-free) property tests for the generated object model.
//
// Milestone 2 exit criteria: generated code compiles standalone; reflection walk
// enumerates every field; clone + equality property tests. A small robot tree is
// built programmatically and exercised for presence semantics, variant
// exclusivity, union-list interleave, deep clone with independent stable
// identity, field-wise equality, full reflection enumeration, and a visitor walk.

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_set>
#include <variant>
#include <vector>

#include "defaults.h"
#include "keywords.h"
#include "reflect.h"
#include "types.h"
#include "visit.h"

using namespace ps::mjcf;

static int g_failed = 0;
static int g_checks = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    ++g_checks;                                                            \
    if (!(cond)) {                                                         \
      ++g_failed;                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
    }                                                                      \
  } while (0)

// A geom with a couple of authored fields, for reuse across tests.
static std::unique_ptr<Geom> MakeGeom(const std::string& name, double density) {
  auto g = std::make_unique<Geom>();
  g->name = name;
  g->type = GeomType::capsule;
  g->density = density;
  g->pos = std::array<double, 3>{{1.0, 2.0, 3.0}};
  return g;
}

// Body/Frame children are one ordered `subtree : BodyChildAny *` list
// (document order is id-semantic). These wrap the push/query the tests used to
// do against per-type lists.
template <class Child>
static void PushChild(std::vector<BodyChildAny>& subtree,
                      std::unique_ptr<Child> c) {
  BodyChildAny item;
  item.node = std::move(c);
  subtree.push_back(std::move(item));
}

template <class Child>
static Child* FirstChild(const std::vector<BodyChildAny>& subtree) {
  for (const auto& item : subtree) {
    if (auto* p = std::get_if<std::unique_ptr<Child>>(&item.node)) {
      return p->get();
    }
  }
  return nullptr;
}

template <class Child>
static int CountChild(const std::vector<BodyChildAny>& subtree) {
  int n = 0;
  for (const auto& item : subtree) {
    if (std::holds_alternative<std::unique_ptr<Child>>(item.node)) ++n;
  }
  return n;
}

// Build: world -> torso(body) { geom, joint, link(body){ geom, joint },
// wrist_frame{ geom, joint } }, plus an actuator section with an interleaved
// union child list. The link body nests a Body directly (body-in-body
// recursion, MJCF's R type-code self-reference); the wrist frame exercises the
// parallel body-context path through Frame.
static std::unique_ptr<Model> BuildRobot() {
  auto m = std::make_unique<Model>();
  m->model = "robot";

  auto torso = std::make_unique<Body>();
  torso->name = "torso";
  PushChild(torso->subtree, MakeGeom("torso_geom", 1000.0));
  {
    auto j = std::make_unique<Joint>();
    j->name = "torso_joint";
    j->type = JointType::free;
    PushChild(torso->subtree, std::move(j));
  }

  // Nest a child Body directly inside torso: Body.bodies is the body-in-body
  // recursion wired from the MJCF body row's R self-reference.
  auto link = std::make_unique<Body>();
  link->name = "link";
  PushChild(link->subtree, MakeGeom("link_geom", 500.0));
  {
    auto j = std::make_unique<Joint>();
    j->name = "link_joint";
    j->type = JointType::hinge;
    j->axis = std::array<double, 3>{{0.0, 1.0, 0.0}};
    PushChild(link->subtree, std::move(j));
  }
  PushChild(torso->subtree, std::move(link));

  // Also nest through a frame (the schema's persistent grouping element),
  // keeping the body-context-through-Frame path covered.
  auto frame = std::make_unique<Frame>();
  frame->name = "wrist_frame";
  PushChild(frame->subtree, MakeGeom("wrist_geom", 250.0));
  {
    auto j = std::make_unique<Joint>();
    j->name = "wrist_joint";
    j->type = JointType::hinge;
    j->axis = std::array<double, 3>{{1.0, 0.0, 0.0}};
    PushChild(frame->subtree, std::move(j));
  }
  PushChild(torso->subtree, std::move(frame));
  m->worldbody.push_back(std::move(torso));

  // Actuator section: interleave Motor / Position / Motor to exercise the
  // ordered heterogeneous union child list (Section 6).
  auto act = std::make_unique<Actuator>();
  {
    auto motor = std::make_unique<Motor>();
    motor->name = "m0";
    motor->joint = ps::Ref<JointAny>("link_joint");
    ActuatorAny item;
    item.node = std::move(motor);
    act->actuators.push_back(std::move(item));
  }
  {
    auto pos = std::make_unique<Position>();
    pos->name = "p0";
    pos->kp = 42.0;
    pos->joint = ps::Ref<JointAny>("link_joint");
    ActuatorAny item;
    item.node = std::move(pos);
    act->actuators.push_back(std::move(item));
  }
  {
    auto motor = std::make_unique<Motor>();
    motor->name = "m1";
    ActuatorAny item;
    item.node = std::move(motor);
    act->actuators.push_back(std::move(item));
  }
  m->actuators.push_back(std::move(act));
  return m;
}

// --- presence: unset vs set-to-default distinguishable ------------------- //
static void TestPresence() {
  Geom g;
  CHECK(!g.contype.has_value());  // fresh: unset
  CHECK(!g.type.has_value());
  CHECK(!g.mass.has_value());

  // Explicitly authoring a field to its IDL default value is still "present"
  // and distinguishable from unset.
  g.contype = 1;  // 1 is also the IDL default
  CHECK(g.contype.has_value());
  CHECK(g.contype.value() == 1);

  // ApplyDefault populates only defaulted fields; non-defaulted stay unset.
  // The layer is both sources: the schema's `=` rows and, where the schema is
  // silent, the value MuJoCo's own mjs_defaultGeom leaves in the struct. Only
  // an attribute MuJoCo COMPUTES (mass, from density and volume) has no value
  // at this layer, and it stays unset rather than reading as zero.
  Geom d;
  ApplyDefault(d);
  CHECK(d.contype.has_value() && d.contype.value() == 1);
  CHECK(d.type.has_value() && d.type.value() == GeomType::sphere);
  CHECK(d.density.has_value() && d.density.value() == 1000.0);
  CHECK(d.margin.has_value() && d.margin.value() == 0.0);
  CHECK(!d.mass.has_value());
}

// --- geom shape spellings are independent presence-tracked fields --------- //
static void TestGeomShape() {
  // MJCF's two ways to shape a geom are sibling attributes, each optional; the
  // schema states their exclusivity as a constraint, not as a sum type, so the
  // object model tracks each on its own.
  Geom g;
  CHECK(!g.size.has_value() && !g.fromto.has_value());

  g.size = ps::InlineVec<double, 3>{0.1, 0.2};
  CHECK(g.size.has_value() && g.size->size() == 2 && !g.fromto.has_value());

  g.fromto = std::array<double, 6>{{0, 0, 0, 1, 1, 1}};
  g.size.reset();
  CHECK(g.fromto.has_value() && !g.size.has_value());
  CHECK((*g.fromto)[3] == 1.0);
}

// --- union child list preserves interleaved order ------------------------- //
static void TestUnionOrder() {
  auto m = BuildRobot();
  const auto& acts = m->actuators.front()->actuators;
  CHECK(acts.size() == 3);
  CHECK(acts[0].kind() == ActuatorAny::Kind::Motor);
  CHECK(acts[1].kind() == ActuatorAny::Kind::Position);
  CHECK(acts[2].kind() == ActuatorAny::Kind::Motor);

  // Element pointers inside the union items are real and typed.
  const auto& p = std::get<std::unique_ptr<Position>>(acts[1].node);
  CHECK(p && p->kp.has_value() && p->kp.value() == 42.0);
}

// --- deep clone: independent, stable, value-equal, fresh serials ---------- //
static void TestClone() {
  auto m = BuildRobot();
  auto c = Clone(*m);

  // Value-equal despite being a separate tree.
  CHECK(*m == *c);

  // Distinct storage / stable independent identity.
  Body* orig_torso = m->worldbody.front().get();
  Body* clone_torso = c->worldbody.front().get();
  CHECK(orig_torso != clone_torso);

  // Serials are re-minted on clone (no duplicate serials within one Model),
  // yet equality ignores serial + provenance.
  CHECK(orig_torso->serial != clone_torso->serial);
  CHECK(*orig_torso == *clone_torso);

  // Mutating the clone does not touch the original (true deep copy).
  FirstChild<Geom>(clone_torso->subtree)->density = 7.0;
  CHECK(FirstChild<Geom>(m->worldbody.front()->subtree)->density.value() == 1000.0);
  CHECK(!(*m == *c));
}

// --- equality detects a single-field change ------------------------------- //
static void TestEquality() {
  auto a = MakeGeom("g", 100.0);
  auto b = Clone(*a);
  CHECK(*a == *b);

  b->density = 101.0;          // one scalar field differs
  CHECK(!(*a == *b));

  b->density = a->density;     // restore
  CHECK(*a == *b);

  b->rgba = std::array<float, 4>{{1, 0, 0, 1}};  // a previously-unset field
  CHECK(!(*a == *b));          // presence difference is detected
}

// --- reflection enumerates every field of Geom ---------------------------- //
static void TestReflection() {
  const auto& d = reflect::Describe(ElementType::Geom);
  CHECK(d.name == "Geom");
  CHECK(d.xml == "geom");
  CHECK(d.field_count == reflect::kFieldCount_Geom);
  CHECK(d.field_count == 31);

  // Every descriptor slot is populated and enumerable without the struct type.
  std::unordered_set<std::string> names;
  for (std::size_t i = 0; i < d.field_count; ++i) {
    names.insert(std::string(d.fields[i].name));
  }
  CHECK(names.size() == d.field_count);
  CHECK(names.count("type") == 1);
  CHECK(names.count("friction") == 1);
  CHECK(names.count("fromto") == 1);     // the geom-shape alternative
  CHECK(names.count("material") == 1);   // ref field

  // Field kinds/arities are reported correctly.
  auto find = [&](const char* n) -> const reflect::FieldDescriptor* {
    for (std::size_t i = 0; i < d.field_count; ++i)
      if (d.fields[i].name == n) return &d.fields[i];
    return nullptr;
  };
  const auto* type = find("type");
  CHECK(type && type->kind == reflect::FieldKind::Enum);
  CHECK(type->type_name == "GeomType");
  const auto* fric = find("friction");
  CHECK(fric && fric->arity == reflect::ArityKind::Range);
  CHECK(fric->arity_min == 1 && fric->arity_max == 3);
  const auto* mat = find("material");
  CHECK(mat && mat->kind == reflect::FieldKind::Ref &&
        mat->type_name == "Material");
  const auto* dclass = find("dclass");
  CHECK(dclass && dclass->xml == "class");  // name divergence recorded

  // Presence accessor works generically via void* + field id.
  Geom g;
  const int type_id = static_cast<int>(find("type") - d.fields);
  CHECK(!d.present(&g, type_id));
  g.type = GeomType::box;
  CHECK(d.present(&g, type_id));
  d.clear(&g, type_id);
  CHECK(!d.present(&g, type_id));

  // The whole element table is walkable.
  CHECK(reflect::ElementCount() >= 140);
  const auto* by_name = reflect::DescribeByName("Body");
  CHECK(by_name && by_name->type == ElementType::Body);

  // Union descriptor lists member element types.
  const auto& u = reflect::DescribeUnion("ActuatorAny");
  CHECK(u.member_count == 13);
  CHECK(u.members[0] == ElementType::ActuatorGeneral);

  // ActuatorAny actually carries the ActuatorGeneral variant column count too.
  const auto& path = reflect::DescribeUnion("PathItemAny");
  CHECK(path.member_count == 3);
}

// --- visitor walks the whole tree ----------------------------------------- //
struct TreeWalker {
  int fields = 0;
  int elements = 0;

  template <class T>
  void field(int, const char*, T&) {
    ++fields;
  }
  template <class T>
  void child(int, const char*, std::vector<std::unique_ptr<T>>& list) {
    for (auto& u : list) {
      if (u) {
        ++elements;
        Visit(*u, *this);
      }
    }
  }
  template <class U>
  void union_child(int, const char*, std::vector<U>& list) {
    for (auto& item : list) {
      ++elements;
      Visit(item, *this);
    }
  }
};

static void TestVisitor() {
  auto m = BuildRobot();
  TreeWalker w;
  Visit(*m, w);

  // Tree has: torso, torso_geom, torso_joint, link, link_geom, link_joint,
  // wrist_frame, wrist_geom, wrist_joint, the actuator container, and 3 union
  // actuators = 13 non-root elements.
  CHECK(w.elements == 13);
  // At least the Model.model field plus every visited element's fields.
  CHECK(w.fields > 20);
}

// --- body-in-body recursion (MJCF R type-code self-reference) ------------- //
static void TestBodyNesting() {
  auto m = BuildRobot();
  Body* torso = m->worldbody.front().get();

  // torso holds a directly-nested child Body via Body.bodies.
  CHECK(CountChild<Body>(torso->subtree) == 1);
  Body* link = FirstChild<Body>(torso->subtree);
  CHECK(link && link->name.has_value() && link->name.value() == "link");
  CHECK(CountChild<Joint>(link->subtree) == 1);
  CHECK(CountChild<Geom>(link->subtree) == 1);

  // Recursion is unbounded: nest another Body under the child.
  auto hand = std::make_unique<Body>();
  hand->name = "hand";
  PushChild(hand->subtree, std::make_unique<Body>());
  FirstChild<Body>(hand->subtree)->name = "finger";
  PushChild(link->subtree, std::move(hand));
  CHECK(CountChild<Body>(link->subtree) == 1);
  CHECK(FirstChild<Body>(FirstChild<Body>(link->subtree)->subtree)->name.value() == "finger");

  // Deep clone copies the whole nested body chain independently.
  auto c = Clone(*m);
  Body* c_torso = c->worldbody.front().get();
  Body* c_link = FirstChild<Body>(c_torso->subtree);
  CHECK(FirstChild<Body>(FirstChild<Body>(c_link->subtree)->subtree)->name.value() == "finger");
  CHECK(c_link != link);  // distinct storage

  // Frame mirrors the body-context set: a Frame can hold nested bodies too.
  Frame frame;
  PushChild(frame.subtree, std::make_unique<Body>());
  FirstChild<Body>(frame.subtree)->name = "in_frame";
  CHECK(FirstChild<Body>(frame.subtree)->name.value() == "in_frame");
}

// --- keyword tables round-trip -------------------------------------------- //
static void TestKeywords() {
  CHECK(ToMjcf(GeomType::sphere) == "sphere");
  CHECK(ToMjcf(GeomType::capsule) == "capsule");
  GeomType out{};
  CHECK(FromMjcf("box", out) && out == GeomType::box);
  CHECK(!FromMjcf("not_a_geom", out));
  // Sanitized enumerator, original MJCF keyword preserved.
  CHECK(ToMjcf(TriState::auto_) == "auto");
}

// --- serials are unique at construction ----------------------------------- //
static void TestSerials() {
  Geom a, b, c;
  CHECK(a.serial != b.serial);
  CHECK(b.serial != c.serial);
  CHECK(a.serial != c.serial);
}

int main() {
  TestPresence();
  TestGeomShape();
  TestUnionOrder();
  TestClone();
  TestEquality();
  TestReflection();
  TestVisitor();
  TestBodyNesting();
  TestKeywords();
  TestSerials();

  std::printf("%d checks, %d failures\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
