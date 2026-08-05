// Battery for the three-tier validator (plan Section 9 exit criterion): per-tier
// positives and negatives, plus a curated broken-model suite that exercises
// every tier-3 rule and asserts the diagnostic carries file:line provenance.
//
// Structural/referential negatives are built programmatically (they need SDK
// shapes the reader would reject, e.g. an out-of-range enum cast). Tier-3 cases
// are parsed from small MJCF strings so the SourceLoc is real and file:line can
// be asserted end to end.

#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "mjcf.h"
#include "sizes.h"
#include "types.h"
#include "validate.h"

using namespace ps::mjcf;
using ps::Diagnostic;
using validate::Severity;
using validate::Tier;

static int g_failed = 0;
static int g_checks = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    ++g_checks;                                                        \
    if (!(cond)) {                                                     \
      ++g_failed;                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
    }                                                                  \
  } while (0)

// Count diagnostics of a tier whose message contains `needle`.
static int CountTier(const std::vector<Diagnostic>& ds, Tier t,
                     const std::string& needle = "") {
  int n = 0;
  for (const auto& d : ds) {
    if (d.tier == t &&
        (needle.empty() || d.message.find(needle) != std::string::npos)) {
      ++n;
    }
  }
  return n;
}

static const Diagnostic* Find(const std::vector<Diagnostic>& ds, Tier t,
                              const std::string& needle) {
  for (const auto& d : ds) {
    if (d.tier == t && d.message.find(needle) != std::string::npos) return &d;
  }
  return nullptr;
}

static int TierErrors(const std::vector<Diagnostic>& ds) {
  return CountTier(ds, Tier::Structural) + CountTier(ds, Tier::Referential);
}

// A minimal well-formed body holding a geom, reused across tests.
static void AddGeom(Body& b, const std::string& name) {
  auto g = std::make_unique<Geom>();
  g->name = name;
  g->type = GeomType::sphere;
  g->size = ps::InlineVec<double, 3>{1.0};
  BodyChildAny item;
  item.node = std::move(g);
  b.subtree.push_back(std::move(item));
}

// ------------------------------------------------------------------ tier 1 //
static void TestStructuralPositive() {
  Model m;
  auto b = std::make_unique<Body>();
  b->name = "torso";
  AddGeom(*b, "g0");
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierStructural);
  CHECK(CountTier(ds, Tier::Structural) == 0);
}

static void TestStructuralEnumIllegal() {
  Model m;
  auto b = std::make_unique<Body>();
  auto g = std::make_unique<Geom>();
  g->type = static_cast<GeomType>(99);  // not a legal keyword
  BodyChildAny item;
  item.node = std::move(g);
  b->subtree.push_back(std::move(item));
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierStructural);
  CHECK(CountTier(ds, Tier::Structural, "illegal enum value") == 1);
}

static void TestStructuralArity() {
  Model m;
  auto b = std::make_unique<Body>();
  auto g = std::make_unique<Geom>();
  g->friction = ps::InlineVec<double, 3>{};  // present but empty: min is 1
  BodyChildAny item;
  item.node = std::move(g);
  b->subtree.push_back(std::move(item));
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierStructural);
  CHECK(CountTier(ds, Tier::Structural, "friction") == 1);

  // Filling it to a legal count clears the finding.
  auto& g2 = *std::get<std::unique_ptr<Geom>>(m.worldbody[0]->subtree[0].node);
  g2.friction = ps::InlineVec<double, 3>{1.0, 0.5};
  auto ds2 = validate::Validate(m, validate::kTierStructural);
  CHECK(CountTier(ds2, Tier::Structural, "friction") == 0);
}

// ------------------------------------------------------------------ tier 2 //
static void TestReferentialResolves() {
  Model m;
  auto asset = std::make_unique<Asset>();
  auto mat = std::make_unique<Material>();
  mat->name = "steel";
  asset->materials.push_back(std::move(mat));
  m.assets.push_back(std::move(asset));

  auto b = std::make_unique<Body>();
  auto g = std::make_unique<Geom>();
  g->material = ps::Ref<Material>("steel");  // resolves
  BodyChildAny item;
  item.node = std::move(g);
  b->subtree.push_back(std::move(item));
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential) == 0);
}

static void TestReferentialDangling() {
  Model m;
  auto b = std::make_unique<Body>();
  auto g = std::make_unique<Geom>();
  g->material = ps::Ref<Material>("missing");
  g->mesh = ps::Ref<Mesh>("nope");
  BodyChildAny item;
  item.node = std::move(g);
  b->subtree.push_back(std::move(item));
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential, "unresolved reference 'missing'") == 1);
  CHECK(CountTier(ds, Tier::Referential, "unresolved reference 'nope'") == 1);
}

static void TestReferentialImplicitWorldAndMain() {
  Model m;
  // A weld referencing the implicit "world" body, and a geom on class "main"
  // with no explicit <default> -- both must resolve.
  auto eq = std::make_unique<Equality>();
  auto w = std::make_unique<Weld>();
  w->body1 = ps::Ref<Body>("world");
  w->body2 = ps::Ref<Body>("free");
  EqualityAny ei;
  ei.node = std::move(w);
  eq->equalities.push_back(std::move(ei));
  m.equalitys.push_back(std::move(eq));

  auto b = std::make_unique<Body>();
  b->name = "free";
  auto g = std::make_unique<Geom>();
  g->dclass = ps::Ref<Default>("main");
  BodyChildAny item;
  item.node = std::move(g);
  b->subtree.push_back(std::move(item));
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential) == 0);
}

static void TestReferentialDuplicateName() {
  Model m;
  auto b1 = std::make_unique<Body>();
  b1->name = "dup";
  auto b2 = std::make_unique<Body>();
  b2->name = "dup";
  m.worldbody.push_back(std::move(b1));
  m.worldbody.push_back(std::move(b2));

  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential, "duplicate body name 'dup'") == 1);

  // A geom named "dup" is a different namespace -> no collision with the body.
  auto b3 = std::make_unique<Body>();
  AddGeom(*b3, "dup");
  m.worldbody.push_back(std::move(b3));
  auto ds2 = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds2, Tier::Referential, "duplicate") == 1);  // still just body
}

static void TestReferentialEmptyNamesAllowed() {
  Model m;
  m.worldbody.push_back(std::make_unique<Body>());  // unnamed
  m.worldbody.push_back(std::make_unique<Body>());  // unnamed
  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential) == 0);
}

static void TestReferentialDefaultClasses() {
  Model m;
  auto root = std::make_unique<Default>();  // implicit main (unnamed)
  auto sub = std::make_unique<Default>();
  sub->dclass = "arm";
  root->subclasses.push_back(std::move(sub));
  m.defaults.push_back(std::move(root));

  auto b = std::make_unique<Body>();
  auto g1 = std::make_unique<Geom>();
  g1->dclass = ps::Ref<Default>("arm");  // resolves
  auto g2 = std::make_unique<Geom>();
  g2->dclass = ps::Ref<Default>("leg");  // dangling
  BodyChildAny i1, i2;
  i1.node = std::move(g1);
  i2.node = std::move(g2);
  b->subtree.push_back(std::move(i1));
  b->subtree.push_back(std::move(i2));
  m.worldbody.push_back(std::move(b));

  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential, "unresolved reference 'leg'") == 1);
  CHECK(CountTier(ds, Tier::Referential, "'arm'") == 0);
}

static void TestReferentialDuplicateClass() {
  Model m;
  auto root = std::make_unique<Default>();
  auto a = std::make_unique<Default>();
  a->dclass = "x";
  auto b = std::make_unique<Default>();
  b->dclass = "x";
  root->subclasses.push_back(std::move(a));
  root->subclasses.push_back(std::move(b));
  m.defaults.push_back(std::move(root));

  auto ds = validate::Validate(m, validate::kTierReferential);
  CHECK(CountTier(ds, Tier::Referential, "duplicate default class 'x'") == 1);
}

// ------------------------------------------------------------------ sizes //
static void TestSizes() {
  Model m;
  auto b = std::make_unique<Body>();
  {
    auto j = std::make_unique<Joint>();
    j->type = JointType::free;  // 7/6
    BodyChildAny it;
    it.node = std::move(j);
    b->subtree.push_back(std::move(it));
  }
  auto child = std::make_unique<Body>();
  {
    auto j = std::make_unique<Joint>();
    j->type = JointType::hinge;  // 1/1
    BodyChildAny it;
    it.node = std::move(j);
    child->subtree.push_back(std::move(it));
  }
  auto mocap = std::make_unique<Body>();
  mocap->mocap = true;  // nmocap += 1
  b->subtree.push_back([&] {
    BodyChildAny it;
    it.node = std::move(child);
    return it;
  }());
  m.worldbody.push_back(std::move(b));
  m.worldbody.push_back(std::move(mocap));

  auto sizes = validate::ComputeSizes(m);
  CHECK(sizes.nq == 8);
  CHECK(sizes.nv == 7);
  CHECK(sizes.nmocap == 1);
  CHECK(sizes.exact);

  // Actuators: a motor (na 0) + an intvelocity (na 1) -> nu 2, na 1.
  auto act = std::make_unique<Actuator>();
  {
    ActuatorAny a;
    a.node = std::make_unique<Motor>();
    act->actuators.push_back(std::move(a));
  }
  {
    ActuatorAny a;
    a.node = std::make_unique<IntVelocity>();
    act->actuators.push_back(std::move(a));
  }
  m.actuators.push_back(std::move(act));
  auto sizes2 = validate::ComputeSizes(m);
  CHECK(sizes2.nu == 2);
  CHECK(sizes2.na == 1);
}

// ---------------------------------------------------- tier 3 (file:line) //
// Every tier-3 case parses a real MJCF string so the diagnostic loc is a true
// file:line, which we assert.
static std::unique_ptr<Model> Parse(const std::string& xml,
                                    const std::string& fname) {
  io::ParseResult r = io::ParseMjcfString(xml, fname);
  if (!r.ok()) {
    std::printf("PARSE FAIL (%s): ", fname.c_str());
    for (const auto& d : r.errors) std::printf("%s; ", d.Render().c_str());
    std::printf("\n");
    return nullptr;
  }
  return std::move(r.model);
}

static void ExpectLoc(const Diagnostic* d, const std::string& fname) {
  CHECK(d != nullptr);
  if (!d) return;
  CHECK(d->loc.file == fname);
  CHECK(d->loc.line > 0);
  CHECK(!d->tag.empty());
}

static void TestSemanticKeyframe() {
  const std::string f = "key.xml";
  auto m = Parse(
      "<mujoco>\n"
      "  <worldbody>\n"
      "    <body>\n"
      "      <joint type=\"hinge\"/>\n"
      "      <geom type=\"sphere\" size=\"1\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "  <keyframe>\n"
      "    <key name=\"home\" qpos=\"0 0\"/>\n"
      "  </keyframe>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kTierSemantic);
  const Diagnostic* d = Find(ds, Tier::Semantic, "keyframe qpos length 2");
  ExpectLoc(d, f);
  CHECK(CountTier(ds, Tier::Semantic, "!= model qpos size 1") == 1);
}

static void TestSemanticLimitedAutolimitsFalse() {
  const std::string f = "limit.xml";
  auto m = Parse(
      "<mujoco>\n"
      "  <compiler autolimits=\"false\"/>\n"
      "  <worldbody>\n"
      "    <body>\n"
      "      <joint type=\"hinge\" range=\"-1 1\"/>\n"
      "      <geom type=\"sphere\" size=\"1\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kTierSemantic);
  const Diagnostic* d = Find(ds, Tier::Semantic, "`range` but not `limited`");
  ExpectLoc(d, f);

  // With autolimits on (the default), the same range is silently fine.
  auto m2 = Parse(
      "<mujoco>\n"
      "  <worldbody>\n"
      "    <body>\n"
      "      <joint type=\"hinge\" range=\"-1 1\"/>\n"
      "      <geom type=\"sphere\" size=\"1\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      "ok.xml");
  CHECK(m2 != nullptr);
  if (m2) {
    auto ds2 = validate::Validate(*m2, validate::kTierSemantic);
    CHECK(CountTier(ds2, Tier::Semantic, "limited") == 0);
  }
}

static void TestSemanticZeroAxis() {
  const std::string f = "axis.xml";
  auto m = Parse(
      "<mujoco>\n"
      "  <worldbody>\n"
      "    <body>\n"
      "      <joint type=\"hinge\" axis=\"0 0 0\"/>\n"
      "      <geom type=\"sphere\" size=\"1\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kTierSemantic);
  const Diagnostic* d = Find(ds, Tier::Semantic, "zero-length axis");
  ExpectLoc(d, f);
}

static void TestSemanticMocap() {
  const std::string f = "mocap.xml";
  // "m" has a Body ancestor -> not a world child; "j" is a mocap body with a
  // joint. Two distinct tier-3 findings.
  auto m = Parse(
      "<mujoco>\n"
      "  <worldbody>\n"
      "    <body name=\"parent\">\n"
      "      <geom type=\"sphere\" size=\"1\"/>\n"
      "      <body name=\"m\" mocap=\"true\">\n"
      "        <geom type=\"sphere\" size=\"1\"/>\n"
      "      </body>\n"
      "    </body>\n"
      "    <body name=\"j\" mocap=\"true\">\n"
      "      <joint type=\"free\"/>\n"
      "      <geom type=\"sphere\" size=\"1\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kTierSemantic);
  ExpectLoc(Find(ds, Tier::Semantic, "must be a direct (static) child"), f);
  ExpectLoc(Find(ds, Tier::Semantic, "cannot have joints"), f);
}

static void TestSemanticNuser() {
  const std::string f = "nuser.xml";
  auto m = Parse(
      "<mujoco>\n"
      "  <size nuser_geom=\"1\"/>\n"
      "  <worldbody>\n"
      "    <body>\n"
      "      <geom type=\"sphere\" size=\"1\" user=\"1 2 3\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kTierSemantic);
  const Diagnostic* d = Find(ds, Tier::Semantic, "exceeds nuser_*");
  ExpectLoc(d, f);
}

static void TestSemanticPluginConfig() {
  Model m;
  auto ext = std::make_unique<Extension>();
  auto pd = std::make_unique<PluginDef>();
  pd->plugin = "mujoco.elasticity.cable";
  auto inst = std::make_unique<PluginInstance>();
  inst->name = "cable0";
  auto cfg = std::make_unique<Config>();
  cfg->key = "";  // empty key
  cfg->value = "1";
  inst->config.push_back(std::move(cfg));
  pd->pluginInstances.push_back(std::move(inst));
  ext->pluginDefs.push_back(std::move(pd));
  m.extensions.push_back(std::move(ext));

  auto ds = validate::Validate(m, validate::kTierSemantic);
  CHECK(CountTier(ds, Tier::Semantic, "empty key") == 1);
}

// R1 (docs/plan_canonicalization.md Wave B): camera focal/principal intrinsics
// require a positive sensorsize (a MuJoCo compile error, mirrored as a tier-3
// lint on the authored form). A focal without sensorsize warns; one with a valid
// sensorsize (frustum.xml pattern) does not.
static void TestSemanticCameraIntrinsics() {
  const std::string f = "cam.xml";
  auto m = Parse(
      "<mujoco>\n"
      "  <worldbody>\n"
      "    <camera name=\"bad\" focal=\"8e-3 8e-3\" resolution=\"640 480\"/>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kTierSemantic);
  const Diagnostic* d =
      Find(ds, Tier::Semantic, "focal/principal require a positive `sensorsize`");
  ExpectLoc(d, f);

  auto ok = Parse(
      "<mujoco>\n"
      "  <worldbody>\n"
      "    <camera name=\"good\" focalpixel=\"1600 1600\" resolution=\"1920 1200\"\n"
      "            sensorsize=\"9.6e-3 6e-3\"/>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      "camok.xml");
  CHECK(ok != nullptr);
  if (!ok) return;
  auto ds2 = validate::Validate(*ok, validate::kTierSemantic);
  CHECK(CountTier(ds2, Tier::Semantic, "sensorsize") == 0);
}

// A curated model that is clean at tiers 1-2 must produce zero errors even with
// all tiers requested (only tier-3 warnings, if any).
static void TestCleanModelNoErrors() {
  const std::string f = "clean.xml";
  auto m = Parse(
      "<mujoco>\n"
      "  <asset>\n"
      "    <material name=\"steel\"/>\n"
      "  </asset>\n"
      "  <worldbody>\n"
      "    <body name=\"torso\">\n"
      "      <joint name=\"j\" type=\"hinge\" axis=\"0 0 1\"/>\n"
      "      <geom type=\"sphere\" size=\"1\" material=\"steel\"/>\n"
      "    </body>\n"
      "  </worldbody>\n"
      "</mujoco>\n",
      f);
  CHECK(m != nullptr);
  if (!m) return;
  auto ds = validate::Validate(*m, validate::kAllTiers);
  CHECK(TierErrors(ds) == 0);
}

int main() {
  TestStructuralPositive();
  TestStructuralEnumIllegal();
  TestStructuralArity();

  TestReferentialResolves();
  TestReferentialDangling();
  TestReferentialImplicitWorldAndMain();
  TestReferentialDuplicateName();
  TestReferentialEmptyNamesAllowed();
  TestReferentialDefaultClasses();
  TestReferentialDuplicateClass();

  TestSizes();

  TestSemanticKeyframe();
  TestSemanticLimitedAutolimitsFalse();
  TestSemanticZeroAxis();
  TestSemanticMocap();
  TestSemanticNuser();
  TestSemanticPluginConfig();
  TestSemanticCameraIntrinsics();

  TestCleanModelNoErrors();

  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
