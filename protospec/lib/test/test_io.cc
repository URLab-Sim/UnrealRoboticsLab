// MJCF IO tests: fixpoint round-trips plus a quirk-by-quirk battery over
// embedded snippets and the matching negative cases. The reader/writer are
// exercised end to end; the differential-against-MuJoCo dimension belongs to the
// harness (protospec/lib/harness/, separate owner).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>

#include "keywords.h"
#include "mjcf.h"
#include "numeric.h"
#include "types.h"

using namespace ps::mjcf;
using namespace ps::mjcf::io;

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

static bool Near(double a, double b) { return std::fabs(a - b) < 1e-9; }

static ParseResult Parse(const std::string& xml) {
  return ParseMjcfString(xml, "<test>");
}

// The world Body of a parsed model (worldbody[0]).
static const Body* World(const Model& m) {
  return m.worldbody.empty() ? nullptr : m.worldbody.front().get();
}

// A body's children (geoms, joints, sites, sub-bodies, frames, macros, ...)
// live in one ordered BodyChildAny list to preserve document order. NthOf<T>
// returns the n-th child of the requested element type, or null.
template <class T>
static const T* NthOf(const Body& parent, std::size_t n) {
  for (const auto& item : parent.subtree) {
    if (auto* p = std::get_if<std::unique_ptr<T>>(&item.node)) {
      if (n-- == 0) return p->get();
    }
  }
  return nullptr;
}
template <class T>
static const T* FirstOf(const Body& parent) {
  return NthOf<T>(parent, 0);
}
static const Body* FirstBody(const Body& parent) { return FirstOf<Body>(parent); }

// Count children of a given element type in a body's subtree.
template <class T>
static std::size_t CountOf(const Body& parent) {
  std::size_t c = 0;
  for (const auto& item : parent.subtree) {
    if (std::holds_alternative<std::unique_ptr<T>>(item.node)) ++c;
  }
  return c;
}

// --- fixpoint: Read(Write(Read(x))) == Read(x); Write is deterministic ----- //
static void TestFixpoint() {
  const std::string xml = R"(<mujoco model="rig">
  <compiler angle="degree" eulerseq="xyz" meshdir="meshes"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast">
    <flag contact="disable" energy="enable"/>
  </option>
  <size memory="1M" nkey="2"/>
  <visual>
    <global fovy="50" offwidth="640"/>
    <map force="0.01"/>
    <rgba haze="1 1 1 1"/>
  </visual>
  <statistic meansize="0.05" extent="2"/>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
    <body name="torso" pos="0 0 1" euler="90 0 0">
      <inertial pos="0 0 0" mass="5" diaginertia="0.1 0.1 0.1"/>
      <joint name="hinge" type="hinge" axis="0 1 0" range="-90 90" ref="10"/>
      <geom name="cap" type="capsule" fromto="0 0 0 0 0 0.5" size="0.05"
            friction="1 0.5"/>
      <site name="s0" pos="0 0 0.5" quat="1 0 0 0"/>
      <camera name="eye" pos="0 -1 0" fovy="45"/>
      <body name="link" pos="0 0 0.5">
        <joint name="slide" type="slide" axis="0 0 1" range="0 0.3"/>
        <geom name="g2" type="sphere" size="0.03"/>
        <frame name="f0" pos="0 0 0.1">
          <geom name="g3" type="box" size="0.02 0.02 0.02"/>
        </frame>
      </body>
    </body>
  </worldbody>
</mujoco>)";

  ParseResult a = Parse(xml);
  CHECK(a.ok());
  if (!a.ok()) {
    for (const auto& e : a.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  std::string out1 = WriteMjcf(*a.model);
  ParseResult b = ParseMjcfString(out1, "<rt>");
  CHECK(b.ok());
  if (!b.ok()) {
    for (const auto& e : b.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  // Read(Write(Read(x))) == Read(x) under generated deep equality.
  CHECK(*a.model == *b.model);
  // Write is deterministic: Write(Read(Write(Read(x)))) is byte-identical.
  std::string out2 = WriteMjcf(*b.model);
  CHECK(out1 == out2);
}

// --- Q-ANGLE: values and compiler unit are preserved verbatim -------------- //
// MuJoCo converts joint range only for LIMITED hinge/ball and ref/springref
// only for hinge, resolving those conditions per consuming element at compile
// (user_objects.cc:3207-3282); a shared default class can feed elements that
// resolve them differently, so no read-time conversion is correct. ProtoSpec
// therefore stores every angle exactly as authored and round trips the compiler
// unit, deferring the conversion to MuJoCo.
static void TestAngle() {
  auto r = Parse(R"(<mujoco>
    <compiler angle="degree"/>
    <worldbody>
      <body name="b" euler="90 0 0">
        <joint name="h" type="hinge" range="-90 180" ref="90" springref="45"/>
        <joint name="s" type="slide" range="0 2" ref="1"/>
      </body>
    </worldbody>
  </mujoco>)");
  CHECK(r.ok());
  const Body* b = FirstBody(*World(*r.model));
  // Q-ORIENT: the body's euler is canonicalized to quat at read (angle="degree",
  // eulerseq "xyz"). euler="90 0 0" deg -> rotate +90 deg about x ->
  // (cos45, sin45, 0, 0) = (sqrt2/2, sqrt2/2, 0, 0). Joint angle FIELDS below stay
  // authored (Q-ANGLE): angle-field preservation is unaffected by Q-ORIENT.
  CHECK(b->quat.has_value());
  CHECK(Near((*b->quat)[0], 0.7071067811865476));
  CHECK(Near((*b->quat)[1], 0.7071067811865476));
  CHECK(Near((*b->quat)[2], 0.0) && Near((*b->quat)[3], 0.0));

  const Joint& h = *NthOf<Joint>(*b, 0);
  CHECK(Near((*h.range)[0], -90.0) && Near((*h.range)[1], 180.0));
  CHECK(Near(*h.ref, 90.0));
  CHECK(Near(*h.springref, 45.0));
  const Joint& s = *NthOf<Joint>(*b, 1);
  CHECK(Near((*s.range)[0], 0.0) && Near((*s.range)[1], 2.0));
  CHECK(Near(*s.ref, 1.0));

  // The authored compiler unit round trips verbatim: no pinning to radian.
  std::string out = WriteMjcf(*r.model);
  CHECK(out.find("angle=\"degree\"") != std::string::npos);
  CHECK(out.find("angle=\"radian\"") == std::string::npos);

  // A radian-authored model likewise keeps its unit and values verbatim.
  auto rad = Parse(R"(<mujoco>
    <compiler angle="radian"/>
    <worldbody><body euler="1.5707963 0 0"/></worldbody>
  </mujoco>)");
  CHECK(rad.ok());
  // angle="radian": euler is interpreted verbatim (no deg->rad conversion), so
  // the half-angle is 1.5707963/2. Computing the expected quat with the same math
  // demonstrates the radian interpretation exactly.
  const Body* b2 = FirstBody(*World(*rad.model));
  CHECK(b2->quat.has_value());
  const double half = 1.5707963 / 2;
  CHECK(Near((*b2->quat)[0], std::cos(half)));
  CHECK(Near((*b2->quat)[1], std::sin(half)));
  CHECK(Near((*b2->quat)[2], 0.0) && Near((*b2->quat)[3], 0.0));
  std::string rad_out = WriteMjcf(*rad.model);
  CHECK(rad_out.find("angle=\"radian\"") != std::string::npos);
}

// --- Q-ORIENT: each encoding + multiple-specifier and zero-quat errors ----- //
static void TestOrient() {
  auto ok = Parse(R"(<mujoco><worldbody>
    <geom name="g" type="sphere" size="1" axisangle="0 0 1 90"/>
  </worldbody></mujoco>)");
  CHECK(ok.ok());
  const auto& g = *FirstOf<Geom>(*World(*ok.model));
  // Q-ORIENT: axisangle="0 0 1 90" (deg default) canonicalizes to a +90 deg
  // rotation about z -> (cos45, 0, 0, sin45) = (sqrt2/2, 0, 0, sqrt2/2).
  CHECK(g.quat.has_value());
  CHECK(Near((*g.quat)[0], 0.7071067811865476));
  CHECK(Near((*g.quat)[1], 0.0) && Near((*g.quat)[2], 0.0));
  CHECK(Near((*g.quat)[3], 0.7071067811865476));

  auto multi = Parse(R"(<mujoco><worldbody>
    <geom type="sphere" size="1" quat="1 0 0 0" euler="0 0 0"/>
  </worldbody></mujoco>)");
  CHECK(!multi.ok());
  CHECK(multi.errors[0].message.find("multiple orientation") != std::string::npos);
  CHECK(multi.errors[0].loc.line > 0);

  auto zero = Parse(R"(<mujoco><worldbody>
    <geom type="sphere" size="1" quat="0 0 0 0"/>
  </worldbody></mujoco>)");
  CHECK(!zero.ok());
  CHECK(zero.errors[0].message.find("zero quaternion") != std::string::npos);
}

// --- Q-FROMTO: routes into GeomShape; size stays a plain field ------------- //
static void TestFromto() {
  auto r = Parse(R"(<mujoco><worldbody>
    <geom name="c" type="capsule" fromto="0 0 0 0 0 1" size="0.1"/>
    <geom name="s" type="sphere" size="0.2"/>
  </worldbody></mujoco>)");
  CHECK(r.ok());
  const auto& c = *NthOf<Geom>(*World(*r.model), 0);
  CHECK(c.fromto.has_value() && Near((*c.fromto)[5], 1.0));
  CHECK(c.size.has_value() && Near((*c.size)[0], 0.1));
  // No fromto -> the attribute stays unset; size is present either way.
  const auto& s = *NthOf<Geom>(*World(*r.model), 1);
  CHECK(!s.fromto.has_value());
  CHECK(s.size.has_value() && Near((*s.size)[0], 0.2));
}

// --- Q-INERTIA: full vs diagonal; fullinertia + orientation exclusion ------ //
static void TestInertia() {
  auto full = Parse(R"(<mujoco><worldbody><body>
    <inertial pos="0 0 0" mass="1" fullinertia="1 2 3 0.1 0.2 0.3"/>
  </body></worldbody></mujoco>)");
  CHECK(full.ok());
  const auto& body = *FirstBody(*World(*full.model));
  CHECK(!body.inertial.empty());
  const auto& in = *body.inertial.front();
  // Q-INERTIA: fullinertia is eigendecomposed at read into diaginertia (principal
  // moments, descending) + the inertial-frame quat. The exact eigenvalues are
  // hard to hand-write, but two invariants pin them: their sum equals the trace
  // (xx+yy+zz = 1+2+3 = 6) and they are positive and sorted descending.
  CHECK(in.diaginertia.has_value());
  CHECK(in.quat.has_value());
  const auto& di = *in.diaginertia;
  CHECK(di[0] >= di[1] && di[1] >= di[2] && di[2] > 0.0);
  CHECK(Near(di[0] + di[1] + di[2], 6.0));

  auto both = Parse(R"(<mujoco><worldbody><body>
    <inertial pos="0 0 0" mass="1" fullinertia="1 2 3 0 0 0" euler="0 0 0"/>
  </body></worldbody></mujoco>)");
  CHECK(!both.ok());
  CHECK(both.errors[0].message.find("fullinertia and inertial orientation") !=
        std::string::npos);
}

// --- Q-ORIENT: parse-end resolution is document-order independent ---------- //
static void TestOrientDocumentOrder() {
  // A <compiler> block AFTER <worldbody> still governs orientation resolution:
  // the reader folds the effective compiler context at parse end, so document
  // order does not matter. angle="radian" means the euler is read verbatim; a
  // (wrong) mid-parse resolve would use the default degree and give a different
  // quat. euler 1.5707963 rad about x -> (cos(h), sin(h), 0, 0), h = angle/2.
  auto r = Parse(R"(<mujoco>
    <worldbody><body euler="1.5707963 0 0"/></worldbody>
    <compiler angle="radian"/>
  </mujoco>)");
  CHECK(r.ok());
  const Body* b = FirstBody(*World(*r.model));
  CHECK(b->quat.has_value());
  const double half = 1.5707963 / 2;
  CHECK(Near((*b->quat)[0], std::cos(half)));
  CHECK(Near((*b->quat)[1], std::sin(half)));
  CHECK(Near((*b->quat)[2], 0.0) && Near((*b->quat)[3], 0.0));
}

// --- Q-ORIENT: a default class stores the canonical quat (inheritance) ------ //
static void TestOrientClassInheritance() {
  // A default class authoring euler stores the resolved quat, exactly like an
  // element (canonicalization runs per authored site, classes included). class
  // euler="0 0 90" deg -> +90 about z -> (cos45, 0, 0, sin45). Inheritance is
  // atomic over quat: an element in the class with no own orientation inherits it.
  auto r = Parse(R"(<mujoco>
    <default><default class="c"><geom euler="0 0 90"/></default></default>
    <worldbody><geom class="c" type="sphere" size="1"/></worldbody>
  </mujoco>)");
  CHECK(r.ok());
  // The class geom stores the canonical quat (not the authored euler).
  const auto& root = *r.model->defaults.front();
  const auto& c = *root.subclasses.front();
  CHECK(!c.geom.empty());
  const auto& cg = *c.geom.front();
  CHECK(cg.quat.has_value());
  CHECK(Near((*cg.quat)[0], 0.7071067811865476));
  CHECK(Near((*cg.quat)[3], 0.7071067811865476));
  // The element authored no orientation of its own -> its quat stays unset and
  // it inherits the class quat through Effective() (atomic field inheritance).
  const auto& g = *FirstOf<Geom>(*World(*r.model));
  CHECK(!g.quat.has_value());
}

// --- Q-ARITY: fewer-than-max OK, more-than-max errors ---------------------- //
static void TestArity() {
  auto r = Parse(R"(<mujoco><worldbody>
    <geom name="g" type="sphere" size="1" friction="1.5"/>
  </worldbody></mujoco>)");
  CHECK(r.ok());
  const auto& g = *FirstOf<Geom>(*World(*r.model));
  CHECK(g.friction.has_value() && g.friction->size() == 1);  // filled count = 1
  CHECK(Near((*g.friction)[0], 1.5));

  auto too_many = Parse(R"(<mujoco><worldbody>
    <geom type="sphere" size="1" friction="1 2 3 4"/>
  </worldbody></mujoco>)");
  CHECK(!too_many.ok());
  CHECK(too_many.errors[0].message.find("too much data") != std::string::npos);
}

// --- Q-NUM: inf, memory suffix, overflow, NaN warning ---------------------- //
static void TestNumeric() {
  auto r = Parse(R"(<mujoco>
    <size memory="2M"/>
    <worldbody><body>
      <joint name="j" type="hinge" armature="inf"/>
    </body></worldbody>
  </mujoco>)");
  CHECK(r.ok());
  CHECK(r.model->sizes.front()->memory.has_value());
  CHECK(*r.model->sizes.front()->memory == std::to_string(2ull << 20));
  const auto& j = *FirstOf<Joint>(*FirstBody(*World(*r.model)));
  CHECK(j.armature.has_value() && std::isinf(*j.armature));

  // Integer overflow is an error.
  auto over = Parse(R"(<mujoco><size nkey="99999999999999999999"/></mujoco>)");
  CHECK(!over.ok());
  CHECK(over.errors[0].message.find("too large") != std::string::npos);

  // NaN parses (MuJoCo parity) but raises a non-fatal warning.
  auto nan = Parse(R"(<mujoco><worldbody><body>
    <joint type="hinge" armature="nan"/>
  </body></worldbody></mujoco>)");
  CHECK(nan.ok());
  CHECK(!nan.warnings.empty());

  // A leading '+' is accepted: std::from_chars rejects it, MuJoCo's strtod does
  // not, and models in the wild write it (menagerie's go1 has pos="0 +0.04 0").
  auto plus = Parse(R"(<mujoco><worldbody><body>
    <geom type="sphere" size="+0.1" pos="0 +0.04 -0.5"/>
  </body></worldbody></mujoco>)");
  CHECK(plus.ok());
  const auto& g = *FirstOf<Geom>(*FirstBody(*World(*plus.model)));
  CHECK(g.pos.has_value());
  CHECK((*g.pos)[1] > 0.039 && (*g.pos)[1] < 0.041);
  CHECK((*g.pos)[2] < -0.499 && (*g.pos)[2] > -0.501);

  // A bare sign is still not a number.
  auto bare = Parse(R"(<mujoco><worldbody><body>
    <geom type="sphere" size="+"/>
  </body></worldbody></mujoco>)");
  CHECK(!bare.ok());

  // memory canonical-byte parsing in isolation.
  std::uint64_t bytes = 0;
  CHECK(num::ParseMemory("1K", bytes) == num::MemStatus::Ok && bytes == 1024);
  CHECK(num::ParseMemory("-1", bytes) == num::MemStatus::Unset);
  CHECK(num::ParseMemory("1Q", bytes) == num::MemStatus::Bad);
}

// --- presence (DR-1): only authored attributes set fields ------------------ //
static void TestPresence() {
  auto r = Parse(R"(<mujoco><worldbody>
    <geom name="g" type="box" size="1 1 1"/>
  </worldbody></mujoco>)");
  CHECK(r.ok());
  const auto& g = *FirstOf<Geom>(*World(*r.model));
  CHECK(g.type.has_value() && *g.type == GeomType::box);
  CHECK(g.size.has_value());
  CHECK(!g.mass.has_value());       // unauthored -> unset (no default written)
  CHECK(!g.density.has_value());
  CHECK(!g.rgba.has_value());
  CHECK(!g.contype.has_value());
  // The writer emits only authored attributes.
  std::string out = WriteMjcf(*r.model);
  CHECK(out.find("mass=") == std::string::npos);
  CHECK(out.find("density=") == std::string::npos);
  CHECK(out.find("type=\"box\"") != std::string::npos);
}

// --- unknown attribute / unknown element: malformed (not unsupported) ------ //
static void TestMalformed() {
  auto attr = Parse(R"(<mujoco><worldbody>
    <geom type="sphere" size="1" bogus="3"/>
  </worldbody></mujoco>)");
  CHECK(!attr.ok());
  CHECK(!attr.unsupported_only());
  CHECK(attr.errors[0].kind == ps::Diagnostic::Kind::MalformedInput);
  CHECK(attr.errors[0].message.find("unknown attribute 'bogus'") !=
        std::string::npos);
  CHECK(attr.errors[0].loc.line > 0);

  auto elem = Parse(R"(<mujoco><worldbody>
    <notathing/>
  </worldbody></mujoco>)");
  CHECK(!elem.ok());
  CHECK(!elem.unsupported_only());
  CHECK(elem.errors[0].message.find("unknown element 'notathing'") !=
        std::string::npos);
}

// --- support boundary: the whole MJCF surface is covered after wave 3 ------- //
static void TestUnsupported() {
  // Families that were unsupported skip signals before wave 3 now parse: the
  // <sensor> section and a geom's <plugin> child are first-class.
  auto r = Parse(R"(<mujoco>
    <worldbody><geom type="sphere" size="1"/></worldbody>
    <sensor/>
  </mujoco>)");
  CHECK(r.ok());

  auto plug = Parse(R"(<mujoco><worldbody>
    <geom type="sphere" size="1"><plugin plugin="x"/></geom>
  </worldbody></mujoco>)");
  CHECK(plug.ok());

  // A genuinely unknown tag is malformed input (not an unsupported-element
  // skip): it maps to no child list at all.
  auto bogus = Parse(R"(<mujoco><worldbody>
    <notanelement/>
  </worldbody></mujoco>)");
  CHECK(!bogus.ok());
  CHECK(!bogus.unsupported_only());
  bool unknown = false;
  for (const auto& e : bogus.errors) {
    if (e.kind == ps::Diagnostic::Kind::MalformedInput &&
        e.message.find("unknown element") != std::string::npos)
      unknown = true;
  }
  CHECK(unknown);
}

// --- root must be <mujoco> ------------------------------------------------- //
static void TestRoot() {
  auto r = Parse("<notmujoco/>");
  CHECK(!r.ok());
  CHECK(r.errors[0].message.find("root element must be <mujoco>") !=
        std::string::npos);

  auto bad = Parse("<mujoco><worldbody></mujoco>");  // malformed XML
  CHECK(!bad.ok());
}

// --- Defaults (family c): classes are DATA, read verbatim, resolve nothing - //
static void TestDefaults() {
  const std::string xml = R"(<mujoco>
  <default>
    <geom rgba="1 0 0 1" friction="1 0.1 0.1"/>
    <joint damping="0.5"/>
    <position kp="100" ctrlrange="-1 1"/>
    <pair friction="1 1 0.01 0.01 0.01"/>
    <equality solref="0.02 1"/>
    <mesh scale="2 2 2"/>
    <material specular="0.5"/>
    <default class="sub">
      <geom rgba="0 1 0 1"/>
      <default class="leaf">
        <joint damping="2"/>
      </default>
    </default>
  </default>
  <worldbody>
    <geom type="sphere" size="1" class="sub"/>
  </worldbody>
</mujoco>)";
  ParseResult r = Parse(xml);
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  CHECK(r.model->defaults.size() == 1);
  const Default& root = *r.model->defaults.front();
  // The root class is unnamed (implicitly "main"); sub-elements are stored as
  // partial specs with only their authored fields present.
  CHECK(!root.dclass.has_value());
  CHECK(root.geom.size() == 1 && root.geom.front()->rgba.has_value());
  CHECK(root.joint.size() == 1 &&
        Near((*root.joint.front()->damping)[0], 0.5));
  CHECK(root.position.size() == 1 && Near(*root.position.front()->kp, 100));
  CHECK(root.pair.size() == 1);
  CHECK(root.equality.size() == 1);
  CHECK(root.mesh.size() == 1);
  CHECK(root.material.size() == 1);
  // Nested class tree preserved verbatim; nothing resolved into the geom.
  CHECK(root.subclasses.size() == 1);
  const Default& sub = *root.subclasses.front();
  CHECK(sub.dclass.has_value() && *sub.dclass == "sub");
  CHECK(sub.subclasses.size() == 1 && *sub.subclasses.front()->dclass == "leaf");
  // The worldbody geom keeps its class ref by name; no default is applied.
  const Geom& g = *FirstOf<Geom>(*World(*r.model));
  CHECK(g.dclass.has_value() && g.dclass->name == "sub");
  CHECK(!g.rgba.has_value());  // class value is NOT merged into the element

  // Fixpoint: the whole default tree round trips under deep equality.
  std::string out1 = WriteMjcf(*r.model);
  ParseResult b = ParseMjcfString(out1, "<rt>");
  CHECK(b.ok());
  CHECK(*r.model == *b.model);
  CHECK(out1 == WriteMjcf(*b.model));
}

// --- Q-AUTO/defaults: root class name rules (xml_native_reader.cc:3041-3055) //
static void TestDefaultClassNames() {
  // Top level may be unnamed or exactly "main".
  CHECK(Parse(R"(<mujoco><default><geom size="1"/></default></mujoco>)").ok());
  CHECK(Parse(R"(<mujoco><default class="main"/></mujoco>)").ok());

  auto renamed = Parse(R"(<mujoco><default class="root"/></mujoco>)");
  CHECK(!renamed.ok());
  CHECK(renamed.errors[0].message.find(
            "top-level default class 'main' cannot be renamed") !=
        std::string::npos);
  CHECK(renamed.errors[0].loc.line > 0);

  // A nested default must name a non-empty class.
  auto empty = Parse(R"(<mujoco><default>
    <default><geom size="1"/></default>
  </default></mujoco>)");
  CHECK(!empty.ok());
  CHECK(empty.errors[0].message.find("empty class name") != std::string::npos);
}

// --- Unknown class reference: NOT a read-time error (deferred to tier 2) ---- //
// MuJoCo resolves classes during parse and errors immediately on a dangling
// name (xml_native_reader.cc:4705-4719). ProtoSpec instead stores every ref by
// name and resolves none at read (DR-8, plan Section 9 tier 2), exactly as it
// treats mesh/material/target refs -- referential validation, not IO, reports
// dangling names with provenance later. This is harness-neutral: a genuinely
// dangling class makes MuJoCo reject the model at compile on both sides.
static void TestUnknownClassRef() {
  auto r = Parse(R"(<mujoco><worldbody>
    <geom type="sphere" size="1" class="does_not_exist"/>
  </worldbody></mujoco>)");
  CHECK(r.ok());
  const Geom& g = *FirstOf<Geom>(*World(*r.model));
  CHECK(g.dclass.has_value() && g.dclass->name == "does_not_exist");
}

// --- Assets (family d): mesh/hfield/material/skin read as data -------------- //
static void TestAssets() {
  const std::string xml = R"(<mujoco>
  <asset>
    <mesh name="m" file="parts/arm.obj" scale="0.1 0.1 0.1" refpos="0 0 1"/>
    <mesh name="verts" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"/>
    <hfield name="h" file="terrain.png" nrow="4" ncol="4" size="1 1 1 0.1"/>
    <material name="mat" specular="0.3" rgba="1 1 1 1">
      <layer texture="t_rough" role="rgb"/>
    </material>
    <skin name="s" file="skin.skn" inflate="0.01"/>
    <model name="nested" file="submodel.xml"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="m"/>
  </worldbody>
</mujoco>)";
  ParseResult r = Parse(xml);
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  CHECK(r.model->assets.size() == 1);
  const Asset& a = *r.model->assets.front();
  CHECK(a.meshs.size() == 2);
  const Mesh& m0 = *a.meshs.front();
  // Asset file paths are DATA: stored verbatim, contents never loaded (DR-7).
  CHECK(m0.file.has_value() && *m0.file == "parts/arm.obj");
  CHECK(m0.scale.has_value() && Near((*m0.scale)[0], 0.1));
  CHECK(a.meshs[1]->vertex.has_value() && a.meshs[1]->vertex->size() == 9);
  CHECK(a.hfields.size() == 1 && *a.hfields.front()->nrow == 4);
  CHECK(a.materials.size() == 1);
  const Material& mat = *a.materials.front();
  CHECK(mat.layers.size() == 1 && !mat.layers.front()->role.empty());
  CHECK(a.skins.size() == 1 && *a.skins.front()->file == "skin.skn");
  CHECK(a.modelAssets.size() == 1 && *a.modelAssets.front()->file == "submodel.xml");

  std::string out1 = WriteMjcf(*r.model);
  ParseResult b = ParseMjcfString(out1, "<rt>");
  CHECK(b.ok());
  CHECK(*r.model == *b.model);
  CHECK(out1 == WriteMjcf(*b.model));
}

// --- Q-TEX: a texture's source (builtin vs file) --------------------------- //
static void TestTextureSource() {
  auto tex = Parse(R"(<mujoco><asset>
    <texture name="grid" type="2d" builtin="checker" width="8" height="8"/>
    <texture name="img" type="2d" file="wood.png"/>
  </asset></mujoco>)");
  CHECK(tex.ok());
  const Asset& a = *tex.model->assets.front();
  CHECK(a.textures.size() == 2);
  const Texture& t0 = *a.textures[0];
  CHECK(!t0.file.has_value() && t0.builtin == TextureBuiltin::checker);
  const Texture& t1 = *a.textures[1];
  CHECK(!t1.builtin.has_value() && t1.file == "wood.png");

  std::string out = WriteMjcf(*tex.model);
  CHECK(out.find("builtin=\"checker\"") != std::string::npos);
  CHECK(out.find("file=\"wood.png\"") != std::string::npos);
  // MuJoCo reads builtin and file as independent attributes, so both survive
  // verbatim and the compiler decides which wins.
  auto both = Parse(R"(<mujoco><asset>
    <texture name="x" type="2d" builtin="none" file="a.png"/>
  </asset></mujoco>)");
  CHECK(both.ok());
  const Texture& tb = *both.model->assets.front()->textures.front();
  CHECK(tb.file == "a.png" && tb.builtin == TextureBuiltin::none);
}

// --- Include (Q-INC): splice-in-place, provenance, once-per-file globally --- //
namespace {
int g_tmp_counter = 0;
std::filesystem::path TempDir() {
  auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  std::filesystem::path d =
      std::filesystem::temp_directory_path() /
      ("ps_io_inc_" + std::to_string(stamp) + "_" +
       std::to_string(++g_tmp_counter));
  std::filesystem::remove_all(d);
  std::filesystem::create_directories(d);
  return d;
}
void WriteText(const std::filesystem::path& p, const std::string& s) {
  std::filesystem::create_directories(p.parent_path());
  std::ofstream(p) << s;
}
}  // namespace

static void TestInclude() {
  std::filesystem::path dir = TempDir();
  WriteText(dir / "main.xml", R"(<mujoco model="m">
  <include file="defs.xml"/>
  <worldbody>
    <include file="sub/body.xml"/>
    <geom name="floor" type="plane" size="1 1 1"/>
  </worldbody>
</mujoco>)");
  WriteText(dir / "defs.xml", R"(<mujoco>
  <default><default class="vis"><geom rgba="1 0 0 1"/></default></default>
</mujoco>)");
  WriteText(dir / "sub" / "body.xml", R"(<mujocoinclude>
  <body name="b" pos="0 0 1">
    <geom name="g" type="sphere" size="0.1" class="vis"/>
  </body>
</mujocoinclude>)");

  ParseResult r = ParseMjcfFile((dir / "main.xml").string());
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
  } else {
    // The <default> from defs.xml and the <body> from sub/body.xml are spliced
    // in place; the include elements vanish (flattened tree).
    CHECK(r.model->defaults.size() == 1);
    CHECK(r.model->defaults.front()->subclasses.size() == 1);
    const Body* w = World(*r.model);
    CHECK(CountOf<Geom>(*w) == 1 && *FirstOf<Geom>(*w)->name == "floor");
    CHECK(CountOf<Body>(*w) == 1 && *FirstBody(*w)->name == "b");
    // Provenance (DR-9): the spliced body's SourceLoc names the INCLUDED file.
    const Body& b = *FirstBody(*w);
    CHECK(b.loc.file.find("body.xml") != std::string::npos);
    CHECK(b.loc.line == 2);
  }
  std::filesystem::remove_all(dir);
}

static void TestIncludeErrors() {
  // Self-include is caught by the once-per-file rule (seeded with the top file).
  {
    std::filesystem::path dir = TempDir();
    WriteText(dir / "c.xml", R"(<mujoco><include file="c.xml"/></mujoco>)");
    ParseResult r = ParseMjcfFile((dir / "c.xml").string());
    CHECK(!r.ok());
    CHECK(r.errors[0].message.find("already included") != std::string::npos);
    std::filesystem::remove_all(dir);
  }
  // A duplicate include of the same file anywhere is rejected globally.
  {
    std::filesystem::path dir = TempDir();
    WriteText(dir / "m.xml", R"(<mujoco>
      <include file="a.xml"/><include file="a.xml"/>
    </mujoco>)");
    WriteText(dir / "a.xml", R"(<mujoco><compiler/></mujoco>)");
    ParseResult r = ParseMjcfFile((dir / "m.xml").string());
    CHECK(!r.ok());
    CHECK(r.errors[0].message.find("already included") != std::string::npos);
    std::filesystem::remove_all(dir);
  }
  // Missing file attribute.
  {
    std::filesystem::path dir = TempDir();
    WriteText(dir / "m.xml", R"(<mujoco><include/></mujoco>)");
    ParseResult r = ParseMjcfFile((dir / "m.xml").string());
    CHECK(!r.ok());
    CHECK(r.errors[0].message.find("missing file attribute") !=
          std::string::npos);
    std::filesystem::remove_all(dir);
  }
  // Include element with children.
  {
    std::filesystem::path dir = TempDir();
    WriteText(dir / "m.xml",
              R"(<mujoco><include file="a.xml"><geom/></include></mujoco>)");
    WriteText(dir / "a.xml", R"(<mujoco><compiler/></mujoco>)");
    ParseResult r = ParseMjcfFile((dir / "m.xml").string());
    CHECK(!r.ok());
    CHECK(r.errors[0].message.find("cannot have children") != std::string::npos);
    std::filesystem::remove_all(dir);
  }
  // Unresolvable file.
  {
    std::filesystem::path dir = TempDir();
    WriteText(dir / "m.xml", R"(<mujoco><include file="nope.xml"/></mujoco>)");
    ParseResult r = ParseMjcfFile((dir / "m.xml").string());
    CHECK(!r.ok());
    CHECK(r.errors[0].message.find("cannot open") != std::string::npos);
    std::filesystem::remove_all(dir);
  }
}

// Fetch a union member pointer of type M from an item, or nullptr.
template <class M, class U>
static const M* Member(const U& item) {
  auto* p = std::get_if<std::unique_ptr<M>>(&item.node);
  return p ? p->get() : nullptr;
}

// Round-trip a whole model under deep equality + deterministic output.
static void Fixpoint(const ParseResult& a) {
  CHECK(a.ok());
  if (!a.ok()) {
    for (const auto& e : a.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  std::string out1 = WriteMjcf(*a.model);
  ParseResult b = ParseMjcfString(out1, "<rt>");
  CHECK(b.ok());
  if (!b.ok()) {
    for (const auto& e : b.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  CHECK(*a.model == *b.model);
  CHECK(out1 == WriteMjcf(*b.model));
}

// --- Contact (family e): pair 5-value friction + solreffriction, exclude ---- //
static void TestContact() {
  auto r = Parse(R"(<mujoco>
    <contact>
      <pair name="p" geom1="a" geom2="b" condim="6"
            friction="1 1 0.01 0.02 0.03" solreffriction="0.01 1" gap="0.001"/>
      <exclude name="e" body1="t" body2="l"/>
    </contact>
  </mujoco>)");
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  const Contact& c = *r.model->contacts.front();
  CHECK(c.pairs.size() == 1 && c.excludes.size() == 1);
  const Pair& p = *c.pairs.front();
  CHECK(*p.condim == 6);
  CHECK(p.friction.has_value() && p.friction->size() == 5);
  CHECK(Near((*p.friction)[4], 0.03));
  CHECK(p.solreffriction.has_value() && p.solreffriction->size() == 2);
  CHECK(p.geom1->name == "a" && p.geom2->name == "b");
  CHECK(c.excludes.front()->body1.name == "t");
  Fixpoint(r);

  // 3-value pair friction (Q-ARITY, fewer than the 5 max) round trips.
  auto few = Parse(R"(<mujoco><contact>
    <pair geom1="a" geom2="b" friction="1 1 0.01"/>
  </contact></mujoco>)");
  CHECK(few.ok());
  CHECK(few.model->contacts.front()->pairs.front()->friction->size() == 3);
  Fixpoint(few);
}

// --- Equality (family e): every spelling, document order, alt attr sets ----- //
static void TestEquality() {
  auto r = Parse(R"(<mujoco>
    <equality>
      <connect name="cb" body1="b1" body2="b2" anchor="0 0 1"/>
      <weld name="w" body1="b1" relpose="0 0 0 1 0 0 0" torquescale="2"/>
      <joint name="ej" joint1="j1" joint2="j2" polycoef="0 1 0 0 0"/>
      <tendon name="et" tendon1="t1" tendon2="t2"/>
      <connect name="cs" site1="s1" site2="s2"/>
      <flex name="ef" flex="f1"/>
      <flexvert name="fv" flex="f1"/>
      <flexstrain name="fs" flex="f1" cell="0 1 2"/>
    </equality>
  </mujoco>)");
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  const Equality& eq = *r.model->equalitys.front();
  CHECK(eq.equalities.size() == 8);
  // Union child list keeps document order across spellings (Section 6).
  using K = EqualityAny::Kind;
  CHECK(eq.equalities[0].kind() == K::Connect);
  CHECK(eq.equalities[1].kind() == K::Weld);
  CHECK(eq.equalities[2].kind() == K::EqualityJoint);
  CHECK(eq.equalities[3].kind() == K::EqualityTendon);
  CHECK(eq.equalities[4].kind() == K::Connect);
  CHECK(eq.equalities[5].kind() == K::EqualityFlex);
  CHECK(eq.equalities[6].kind() == K::Flexvert);
  CHECK(eq.equalities[7].kind() == K::Flexstrain);
  // Body-form connect keeps anchor+bodies; site-form connect keeps sites.
  const Connect* cb = Member<Connect>(eq.equalities[0]);
  CHECK(cb && cb->anchor.has_value() && cb->body1->name == "b1");
  CHECK(!cb->site1.has_value());
  const Connect* cs = Member<Connect>(eq.equalities[4]);
  CHECK(cs && cs->site1->name == "s1" && !cs->anchor.has_value());
  const Weld* w = Member<Weld>(eq.equalities[1]);
  CHECK(w && Near(*w->torquescale, 2.0) && w->relpose.has_value());
  const Flexstrain* fs = Member<Flexstrain>(eq.equalities[7]);
  CHECK(fs && fs->cell.has_value() && Near((*fs->cell)[2], 2.0));
  Fixpoint(r);
}

static void TestEqualityNegatives() {
  // connect: body and site semantics cannot be mixed.
  auto mixed = Parse(R"(<mujoco><equality>
    <connect body1="b" anchor="0 0 0" site1="s"/>
  </equality></mujoco>)");
  CHECK(!mixed.ok());
  CHECK(mixed.errors[0].message.find(
            "at most one of ('site1', 'site2'), ('body1', 'body2', 'anchor')") !=
        std::string::npos);

  // connect: neither complete body form nor complete site form.
  auto neither = Parse(R"(<mujoco><equality>
    <connect body1="b"/>
  </equality></mujoco>)");
  CHECK(!neither.ok());
  CHECK(neither.errors[0].message.find(
            "one of ('site1', 'site2'), ('body1', 'anchor') must be specified") !=
        std::string::npos);

  // weld: site1 without site2 is not a complete site form (and no body form).
  auto weld = Parse(R"(<mujoco><equality>
    <weld site1="s"/>
  </equality></mujoco>)");
  CHECK(!weld.ok());
  CHECK(weld.errors[0].message.find(
            "one of ('site1', 'site2'), 'body1' must be specified") !=
        std::string::npos);
}

// --- Tendon (family e): spatial path interleave (THE ordering test) + fixed - //
static void TestTendon() {
  auto r = Parse(R"(<mujoco>
    <tendon>
      <spatial name="sp" width="0.005" limited="true" range="0 1"
               stiffness="10" damping="0.1" springlength="0.2">
        <site site="s0"/>
        <geom geom="g0" sidesite="sd"/>
        <pulley divisor="2"/>
        <site site="s1"/>
      </spatial>
      <fixed name="fx">
        <joint joint="j0" coef="1"/>
        <joint joint="j1" coef="-1"/>
      </fixed>
    </tendon>
  </mujoco>)");
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  const Tendon& t = *r.model->tendons.front();
  CHECK(t.tendons.size() == 2);
  CHECK(t.tendons[0].kind() == TendonAny::Kind::Spatial);
  CHECK(t.tendons[1].kind() == TendonAny::Kind::Fixed);
  const Spatial* sp = Member<Spatial>(t.tendons[0]);
  CHECK(sp && sp->path.size() == 4);
  // The interleaved site/geom/pulley order is preserved exactly (the schema was
  // redesigned around this: a per-type list would lose site-geom-pulley-site).
  using PK = PathItemAny::Kind;
  CHECK(sp->path[0].kind() == PK::SpatialSite);
  CHECK(sp->path[1].kind() == PK::SpatialGeom);
  CHECK(sp->path[2].kind() == PK::Pulley);
  CHECK(sp->path[3].kind() == PK::SpatialSite);
  CHECK(Member<SpatialGeom>(sp->path[1])->sidesite->name == "sd");
  CHECK(Near(Member<Pulley>(sp->path[2])->divisor.value(), 2.0));
  CHECK(sp->stiffness->size() == 1 && Near((*sp->stiffness)[0], 10.0));
  const Fixed* fx = Member<Fixed>(t.tendons[1]);
  CHECK(fx && fx->fixedJoints.size() == 2);
  CHECK(fx->fixedJoints[0]->joint.name == "j0" &&
        Near(*fx->fixedJoints[0]->coef, 1.0));
  Fixpoint(r);
}

// --- Actuators (family f): id order, typed-stays-typed, params, plugin ------ //
static void TestActuators() {
  auto r = Parse(R"(<mujoco>
    <actuator>
      <motor name="m" joint="j0" gear="2 0 0 0 0 0" ctrllimited="true"
             ctrlrange="-1 1"/>
      <position name="p" joint="j0" kp="100" kv="5" inheritrange="1"/>
      <general name="g" joint="j1" dyntype="filter" gaintype="fixed"
               biastype="affine" gainprm="35" biasprm="0 -35 0" actdim="1"/>
      <velocity name="v" joint="j1" kv="7"/>
      <intvelocity name="iv" joint="j0" kp="3" actrange="-2 2"/>
      <damper name="d" joint="j1" kv="4" ctrlrange="0 1"/>
      <cylinder name="cy" joint="j0" timeconst="0.1" area="2" bias="1 0 0"/>
      <muscle name="mu" joint="j1" timeconst="0.01 0.04" force="500" scale="200"/>
      <adhesion name="ad" body="b0" gain="1" ctrlrange="0 1"/>
      <dcmotor name="dc" joint="j0" motorconst="0.5 0" resistance="1.2"
               controller="1 2 3 4 5 6" thermal="1 2 3 4 5 6" input="velocity"/>
      <plugin name="pl" joint="j1" plugin="mujoco.pid" actdim="1">
        <config key="kp" value="10"/>
        <config key="ki" value="1"/>
      </plugin>
    </actuator>
  </mujoco>)");
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  const Actuator& a = *r.model->actuators.front();
  CHECK(a.actuators.size() == 11);
  // Document order == compile id order; each spelling stays its own type
  // (DR-3/Q-ACT: no lowering to <general> in IO).
  using K = ActuatorAny::Kind;
  const K expect[] = {K::Motor,       K::Position, K::ActuatorGeneral,
                      K::Velocity,    K::IntVelocity, K::Damper,
                      K::Cylinder,    K::Muscle,   K::Adhesion,
                      K::DcMotor,     K::ActuatorPlugin};
  for (int i = 0; i < 11; ++i) CHECK(a.actuators[i].kind() == expect[i]);

  const Motor* m = Member<Motor>(a.actuators[0]);
  CHECK(m && m->joint->name == "j0" && m->gear->size() == 6);
  CHECK(*m->ctrllimited == TriState::true_);  // tri-state stays a tri-state
  const Position* p = Member<Position>(a.actuators[1]);
  CHECK(p && Near(*p->kp, 100) && Near(*p->kv, 5) && Near(*p->inheritrange, 1));
  const ActuatorGeneral* g = Member<ActuatorGeneral>(a.actuators[2]);
  CHECK(g && *g->dyntype == DynType::filter && *g->biastype == BiasType::affine);
  CHECK(g->gainprm->size() == 1 && g->biasprm->size() == 3);
  const Muscle* mu = Member<Muscle>(a.actuators[7]);
  CHECK(mu && mu->timeconst->size() == 2 && Near(*mu->force, 500));
  const DcMotor* dc = Member<DcMotor>(a.actuators[9]);
  CHECK(dc && dc->motorconst->size() == 2 && dc->controller->size() == 6 &&
        dc->thermal->size() == 6 && *dc->input == DcMotorInput::velocity);
  const ActuatorPlugin* pl = Member<ActuatorPlugin>(a.actuators[10]);
  CHECK(pl && *pl->plugin == "mujoco.pid" && pl->config.size() == 2);
  CHECK(pl->config[0]->key == "kp" && *pl->config[1]->value == "1");
  Fixpoint(r);
}

static void TestActuatorNegatives() {
  // At most one transmission target.
  auto two = Parse(R"(<mujoco><actuator>
    <motor joint="j0" site="s0"/>
  </actuator></mujoco>)");
  CHECK(!two.ok());
  CHECK(two.errors[0].message.find("at most one of transmission") !=
        std::string::npos);

  // refsite requires a site transmission.
  auto refsite = Parse(R"(<mujoco><actuator>
    <position joint="j0" refsite="r"/>
  </actuator></mujoco>)");
  CHECK(!refsite.ok());
  CHECK(refsite.errors[0].message.find("refsite can only be used") !=
        std::string::npos);

  // slidersite requires a slidercrank transmission.
  auto slider = Parse(R"(<mujoco><actuator>
    <motor joint="j0" slidersite="s"/>
  </actuator></mujoco>)");
  CHECK(!slider.ok());
  CHECK(slider.errors[0].message.find("slidercrank transmission") !=
        std::string::npos);
}

// --- Union interleave preserved through Write->Read (both directions) ------- //
static void TestUnionOrderRoundTrip() {
  // A spelling sequence with repeats; the written MJCF must re-read to the same
  // ordered union, and re-writing must be byte-identical.
  auto r = Parse(R"(<mujoco><actuator>
    <position name="a" joint="j"/>
    <motor name="b" joint="j"/>
    <position name="c" joint="j"/>
    <velocity name="d" joint="j"/>
    <motor name="e" joint="j"/>
  </actuator></mujoco>)");
  CHECK(r.ok());
  const auto& list = r.model->actuators.front()->actuators;
  CHECK(list.size() == 5);
  std::string out1 = WriteMjcf(*r.model);
  // The five spellings appear in the written order motor/position after position.
  CHECK(out1.find("name=\"a\"") < out1.find("name=\"b\""));
  CHECK(out1.find("name=\"b\"") < out1.find("name=\"c\""));
  ParseResult b = ParseMjcfString(out1, "<rt>");
  CHECK(b.ok());
  const auto& list2 = b.model->actuators.front()->actuators;
  CHECK(list2.size() == 5);
  for (std::size_t i = 0; i < list.size(); ++i)
    CHECK(list[i].kind() == list2[i].kind());
  CHECK(out1 == WriteMjcf(*b.model));
}

// --- (g) sensors: ordered union, per-tag slot routing, fixpoint ------------ //
static void TestSensors() {
  auto r = Parse(R"(<mujoco><sensor>
    <accelerometer name="a" site="s0" cutoff="5" noise="0.1"/>
    <gyro name="g" site="s0" interval="0.01"/>
    <rangefinder name="rf" site="s0" data="dist normal"/>
    <jointpos name="jp" joint="j0"/>
    <framepos name="fp" objtype="site" objname="s0" reftype="body" refname="b0"/>
    <framelinacc name="fa" objtype="body" objname="b0"/>
    <distance name="d" geom1="g0" body2="b0"/>
    <contact name="ct" subtree1="b0" geom2="g0" num="3" data="found force"
             reduce="mindist"/>
    <clock name="ck"/>
    <user name="u" objtype="body" objname="b0" datatype="axis" needstage="vel"
          dim="3"/>
    <plugin name="pp" plugin="mujoco.sensor.touch_grid" objtype="site"
            objname="s0"/>
  </sensor></mujoco>)");
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  const auto& list = r.model->sensors.front()->sensors;
  CHECK(list.size() == 11);
  using K = SensorAny::Kind;
  CHECK(list[0].kind() == K::Accelerometer && list[2].kind() == K::Rangefinder);
  CHECK(list[10].kind() == K::SensorPlugin);
  // A `interval` shorter than 2 (Q-ARITY exact=false) is accepted.
  const Gyro* g = Member<Gyro>(list[1]);
  CHECK(g && g->interval && g->interval->size() == 1);
  const Rangefinder* rf = Member<Rangefinder>(list[2]);
  CHECK(rf && rf->data && rf->data->size() == 2 &&
        (*rf->data)[0] == RayData::dist && (*rf->data)[1] == RayData::normal);
  const Framepos* fp = Member<Framepos>(list[4]);
  CHECK(fp && fp->objtype == FrameObject::site &&
        *fp->reftype == FrameObject::body && *fp->refname == "b0");
  const SensorContact* ct = Member<SensorContact>(list[7]);
  CHECK(ct && ct->subtree1->name == "b0" && *ct->num == 3 &&
        ct->data->size() == 2 && *ct->reduce == ContactReduce::mindist);
  Fixpoint(r);
}

static void TestSensorNegatives() {
  // rangefinder: exactly one of site/camera (here: both).
  auto rf = Parse(R"(<mujoco><sensor>
    <rangefinder site="s0" camera="c0"/></sensor></mujoco>)");
  CHECK(!rf.ok());
  CHECK(rf.errors[0].message.find(
            "at most one of 'site', 'camera' can be specified") !=
        std::string::npos);

  // distance: exactly one of (geom1, body1).
  auto dist = Parse(R"(<mujoco><sensor>
    <distance geom1="g0" body1="b0" geom2="g1"/></sensor></mujoco>)");
  CHECK(!dist.ok());
  CHECK(dist.errors[0].message.find(
            "at most one of 'geom1', 'body1' can be specified") !=
        std::string::npos);

  // contact: at most one first source.
  auto ct = Parse(R"(<mujoco><sensor>
    <contact site="s0" geom1="g0"/></sensor></mujoco>)");
  CHECK(!ct.ok());
  CHECK(ct.errors[0].message.find(
            "at most one of 'geom1', 'body1', 'subtree1', 'site'") !=
        std::string::npos);

  // frame sensor: refname without reftype.
  auto fr = Parse(R"(<mujoco><sensor>
    <framepos objtype="site" objname="s0" refname="b0"/></sensor></mujoco>)");
  CHECK(!fr.ok());
  CHECK(fr.errors[0].message.find(
            "attributes 'reftype', 'refname' must be specified together") !=
        std::string::npos);

  // user sensor: objname without objtype.
  auto us = Parse(R"(<mujoco><sensor>
    <user objname="b0" dim="1"/></sensor></mujoco>)");
  CHECK(!us.ok());
  CHECK(us.errors[0].message.find(
            "attributes 'objtype', 'objname' must be specified together") !=
        std::string::npos);
}

// --- (h) custom + keyframe + extension ------------------------------------- //
static void TestCustomKeyframeExtension() {
  auto r = Parse(R"(<mujoco>
    <extension>
      <plugin plugin="mujoco.elasticity.cable">
        <instance name="inst0">
          <config key="twist" value="1e6"/>
          <config key="bend" value="1e9"/>
        </instance>
      </plugin>
    </extension>
    <custom>
      <numeric name="n0" size="5" data="1 2 3"/>
      <numeric name="n1" data="7 8"/>
      <text name="t0" data="hello"/>
      <tuple name="tp">
        <element objtype="body" objname="b0" prm="0.5"/>
        <element objtype="geom" objname="g0"/>
      </tuple>
    </custom>
    <keyframe>
      <key name="home" time="0.5" qpos="0 0 1 1 0 0 0" ctrl="1 2"/>
    </keyframe>
  </mujoco>)");
  CHECK(r.ok());
  if (!r.ok()) {
    for (const auto& e : r.errors) std::printf("  err: %s\n", e.Render().c_str());
    return;
  }
  const Custom& c = *r.model->customs.front();
  CHECK(c.numerics.size() == 2 && c.texts.size() == 1 && c.tuples.size() == 1);
  // Numeric data is authored text the engine parses (the schema types it
  // `string`), stored verbatim alongside the authored `size`.
  CHECK(c.numerics[0]->data == "1 2 3" && c.numerics[0]->size &&
        *c.numerics[0]->size == 5);
  CHECK(c.numerics[1]->data == "7 8" && !c.numerics[1]->size);
  // Tuple element children: parallel objtype/objname/optional prm rows.
  const Tuple& tp = *c.tuples.front();
  CHECK(tp.tupleElements.size() == 2);
  CHECK(tp.tupleElements[0]->objtype == "body" && Near(*tp.tupleElements[0]->prm, 0.5));
  CHECK(!tp.tupleElements[1]->prm.has_value());
  // Keyframe vectors stored verbatim, no size validation at read.
  const Key& k = *r.model->keyframes.front()->keys.front();
  CHECK(Near(*k.time, 0.5) && k.qpos->size() == 7 && k.ctrl->size() == 2);
  // Extension: plugin/instance/config nesting.
  const PluginInstance& inst =
      *r.model->extensions.front()->pluginDefs.front()->pluginInstances.front();
  CHECK(inst.name == "inst0" && inst.config.size() == 2);
  Fixpoint(r);

  // Negative: duplicate config key (Q-PLUGIN, ReadPluginConfigs).
  auto dup = Parse(R"(<mujoco><extension>
    <plugin plugin="p"><instance name="i">
      <config key="k" value="1"/>
      <config key="k" value="2"/>
    </instance></plugin></extension></mujoco>)");
  CHECK(!dup.ok());
  CHECK(dup.errors[0].message.find("duplicate config key: k") != std::string::npos);

  // Negative: text field cannot be empty is a MuJoCo compile check, but the
  // reader still round-trips an empty-string data as authored -- verify a
  // non-empty text stays exact.
  auto txt = Parse(R"(<mujoco><custom>
    <text name="t" data="abc"/></custom></mujoco>)");
  CHECK(txt.ok() && txt.model->customs.front()->texts.front()->data == "abc");
}

// --- (i) macros + deformable: verbatim pass-through + subtree ordering ------ //
static void TestMacrosDeformable() {
  // Composite with per-kind sub-defaults and a plugin child, written verbatim.
  auto comp = Parse(R"(<mujoco><worldbody><body>
    <composite prefix="C" type="grid" count="4 4 1" offset="0 0 1">
      <joint kind="main" damping="0.1"/>
      <geom type="sphere" size="0.02" rgba="1 0 0 1"/>
      <skin texcoord="true" material="m" inflate="0.01"/>
    </composite>
  </body></worldbody></mujoco>)");
  CHECK(comp.ok());
  if (!comp.ok())
    for (const auto& e : comp.errors) std::printf("  err: %s\n", e.Render().c_str());
  const Body* cb = FirstBody(*World(*comp.model));
  const Composite* cc = FirstOf<Composite>(*cb);
  CHECK(cc && *cc->prefix == "C" && cc->count && cc->count->size() == 3);
  CHECK(cc->compositeJoints.size() == 1 && cc->compositeGeoms.size() == 1);
  CHECK(cc->compositeSkins.size() == 1 &&
        *cc->compositeSkins.front()->texcoord == true);
  Fixpoint(comp);

  // Flexcomp with edge/elasticity/contact/pin children, written verbatim.
  auto fc = Parse(R"(<mujoco><worldbody><body>
    <flexcomp name="F" type="grid" count="3 3 3" spacing="0.1 0.1 0.1"
              dim="3" mass="1">
      <edge equality="true" stiffness="100"/>
      <elasticity young="1e5" poisson="0.3"/>
      <contact internal="true" selfcollide="bvh" passive="false"/>
      <pin id="0 1 2"/>
    </flexcomp>
  </body></worldbody></mujoco>)");
  CHECK(fc.ok());
  if (!fc.ok())
    for (const auto& e : fc.errors) std::printf("  err: %s\n", e.Render().c_str());
  const Flexcomp* ff = FirstOf<Flexcomp>(*FirstBody(*World(*fc.model)));
  CHECK(ff && ff->count && ff->count->size() == 3 && ff->element.has_value() == false);
  CHECK(ff->flexcompEdges.size() == 1 && ff->flexcompPins.size() == 1);
  CHECK(ff->flexcompPins.front()->id->size() == 3);
  Fixpoint(fc);

  // Deformable flex, written verbatim (element is int connectivity, node text).
  auto df = Parse(R"(<mujoco>
    <deformable>
      <flex name="fx" dim="2" body="b0 b1 b2" vertex="0 0 0 1 0 0 0 1 0"
            element="0 1 2">
        <contact selfcollide="none"/>
        <edge stiffness="5"/>
      </flex>
    </deformable>
  </mujoco>)");
  CHECK(df.ok());
  if (!df.ok())
    for (const auto& e : df.errors) std::printf("  err: %s\n", e.Render().c_str());
  const Flex& fx = *df.model->deformables.front()->flexs.front();
  // <flex body=> is a space-separated name list the engine splits at compile;
  // the schema types it as one string, so it round-trips verbatim.
  CHECK(fx.body == "b0 b1 b2" && fx.element.size() == 3);
  Fixpoint(df);

  // Body-subtree document order: a geom, then a frame holding a geom, then a
  // sub-body -- all one ordered union, so the interleave survives round-trip.
  auto ord = Parse(R"(<mujoco><worldbody>
    <geom name="g0" type="sphere" size="1"/>
    <frame name="fr"><geom name="g1" type="box" size="1 1 1"/></frame>
    <body name="b1"/>
    <geom name="g2" type="sphere" size="1"/>
  </worldbody></mujoco>)");
  CHECK(ord.ok());
  const auto& sub = World(*ord.model)->subtree;
  CHECK(sub.size() == 4);
  CHECK(std::holds_alternative<std::unique_ptr<Geom>>(sub[0].node));
  CHECK(std::holds_alternative<std::unique_ptr<Frame>>(sub[1].node));
  CHECK(std::holds_alternative<std::unique_ptr<Body>>(sub[2].node));
  CHECK(std::holds_alternative<std::unique_ptr<Geom>>(sub[3].node));
  std::string out = WriteMjcf(*ord.model);
  CHECK(out.find("g0") < out.find("fr") && out.find("fr") < out.find("b1") &&
        out.find("b1") < out.find("g2"));
  Fixpoint(ord);
}

// --- Wave B canonicalizations (docs/plan_canonicalization.md #4-#10) -------- //
// Each: the reader accepts the legacy spelling and stores the canonical form;
// the writer emits only the canonical form; hand-verified goldens.
static void TestWaveBCanonicalization() {
  // #1 light directional -> type. directional="true" => directional, "false" =>
  // spot (xml_native_reader.cc:2126); the writer emits type=, never directional.
  auto lt = Parse(R"(<mujoco><worldbody>
    <light name="ld" directional="true"/>
    <light name="ls" directional="false"/>
    <light name="lp" type="point"/>
  </worldbody></mujoco>)");
  CHECK(lt.ok());
  const Body* w = World(*lt.model);
  const Light* ld = NthOf<Light>(*w, 0);
  const Light* ls = NthOf<Light>(*w, 1);
  const Light* lp = NthOf<Light>(*w, 2);
  CHECK(ld && ld->type && *ld->type == LightType::directional);
  CHECK(ls && ls->type && *ls->type == LightType::spot);
  CHECK(lp && lp->type && *lp->type == LightType::point);
  std::string lout = WriteMjcf(*lt.model);
  CHECK(lout.find("directional=") == std::string::npos);
  CHECK(lout.find("type=\"directional\"") != std::string::npos);
  Fixpoint(lt);
  // type + directional on one element is a read error.
  auto lboth = Parse(R"(<mujoco><worldbody>
    <light directional="true" type="spot"/></worldbody></mujoco>)");
  CHECK(!lboth.ok());
  CHECK(lboth.errors[0].message.find("type and directional") !=
        std::string::npos);

  // #5 tendon springlength: a lone value duplicates into the pair; two values
  // are kept (xml_native_reader.cc:2372-2374). The writer emits both.
  auto sl = Parse(R"(<mujoco><tendon>
    <spatial name="one" springlength="0.5"><site site="a"/><site site="b"/></spatial>
    <spatial name="two" springlength="0 0.7"><site site="a"/><site site="b"/></spatial>
  </tendon></mujoco>)");
  CHECK(sl.ok());
  const Spatial* s1 = Member<Spatial>(sl.model->tendons.front()->tendons[0]);
  const Spatial* s2 = Member<Spatial>(sl.model->tendons.front()->tendons[1]);
  CHECK(s1 && s1->springlength && Near((*s1->springlength)[0], 0.5) &&
        Near((*s1->springlength)[1], 0.5));
  CHECK(s2 && s2->springlength && Near((*s2->springlength)[0], 0.0) &&
        Near((*s2->springlength)[1], 0.7));
  CHECK(WriteMjcf(*sl.model).find("springlength=\"0.5 0.5\"") !=
        std::string::npos);
  Fixpoint(sl);

  // #6 cylinder diameter -> area = pi/4 d^2 (mjs_setToCylinder). diameter=0.5 =>
  // area = pi/4 * 0.25 = 0.19634954084936207. The writer emits area, not diameter.
  auto cyl = Parse(R"(<mujoco><actuator>
    <cylinder name="c" joint="j" diameter="0.5"/>
    <cylinder name="d" joint="j" area="2"/>
  </actuator></mujoco>)");
  CHECK(cyl.ok());
  const Cylinder* c0 =
      Member<Cylinder>(cyl.model->actuators.front()->actuators[0]);
  const Cylinder* c1 =
      Member<Cylinder>(cyl.model->actuators.front()->actuators[1]);
  CHECK(c0 && c0->area && Near(*c0->area, 3.14159265358979323846 / 4 * 0.25));
  CHECK(c1 && c1->area && Near(*c1->area, 2.0));
  std::string cout = WriteMjcf(*cyl.model);
  CHECK(cout.find("diameter=") == std::string::npos &&
        cout.find("area=") != std::string::npos);
  Fixpoint(cyl);

  // #7 material texture attr -> canonical <layer role="rgb">; the writer emits
  // the layer, never the texture attr.
  auto mat = Parse(R"(<mujoco><asset>
    <material name="m" texture="grid" specular="0.3"/>
  </asset></mujoco>)");
  CHECK(mat.ok());
  const Material& m = *mat.model->assets.front()->materials.front();
  CHECK(m.layers.size() == 1 && m.layers.front()->role == "rgb" &&
        m.layers.front()->texture->name == "grid");
  std::string mout = WriteMjcf(*mat.model);
  CHECK(mout.find("texture=\"grid\"") != std::string::npos);  // on the layer
  CHECK(mout.find("<layer") != std::string::npos);
  Fixpoint(mat);
  // texture attr + <layer> child is an error.
  auto mmix = Parse(R"(<mujoco><asset>
    <material name="m" texture="grid"><layer texture="t" role="normal"/></material>
  </asset></mujoco>)");
  CHECK(!mmix.ok());
  CHECK(mmix.errors[0].message.find("cannot have layer") != std::string::npos);

  // #8 numeric size and data are both authored fields, stored and written back
  // verbatim; MuJoCo materializes the array from them at compile.
  auto num = Parse(R"(<mujoco><custom>
    <numeric name="pad" size="5" data="1 2 3"/>
    <numeric name="reserve" size="3"/>
    <numeric name="bare" data="7 8"/>
  </custom></mujoco>)");
  CHECK(num.ok());
  const Custom& cst = *num.model->customs.front();
  CHECK(*cst.numerics[0]->size == 5 && cst.numerics[0]->data == "1 2 3");
  CHECK(*cst.numerics[1]->size == 3 && !cst.numerics[1]->data);
  CHECK(!cst.numerics[2]->size && cst.numerics[2]->data == "7 8");
  CHECK(WriteMjcf(*num.model).find("size=\"5\"") != std::string::npos);
  Fixpoint(num);

  // #9 keyword-set canonicalization: camera output / rangefinder data are
  // order-insensitive bitmasks; the reader stores them in enum-declaration order.
  auto kw = Parse(R"(<mujoco>
    <worldbody><camera name="c" output="segmentation rgb depth"/></worldbody>
    <sensor><rangefinder name="rf" site="s" data="normal dist"/></sensor>
  </mujoco>)");
  CHECK(kw.ok());
  const Camera* cam = NthOf<Camera>(*World(*kw.model), 0);
  CHECK(cam && cam->output && cam->output->size() == 3 &&
        (*cam->output)[0] == CameraOutput::rgb &&
        (*cam->output)[1] == CameraOutput::depth &&
        (*cam->output)[2] == CameraOutput::segmentation);
  const Rangefinder* rf =
      Member<Rangefinder>(kw.model->sensors.front()->sensors[0]);
  CHECK(rf && rf->data && (*rf->data)[0] == RayData::dist &&
        (*rf->data)[1] == RayData::normal);
  CHECK(WriteMjcf(*kw.model).find("output=\"rgb depth segmentation\"") !=
        std::string::npos);
  Fixpoint(kw);
  // contact sensor data must be authored in enum order (strict).
  auto cbad = Parse(R"(<mujoco><sensor>
    <contact name="ct" body1="b" data="normal force"/></sensor></mujoco>)");
  CHECK(!cbad.ok());
  CHECK(cbad.errors[0].message.find("must be in order") != std::string::npos);
}

// Every schema `resolver="..."` name carried by the generated binding metadata
// must have a handler in the reader's resolver registry. This guards the Task-2
// contract: a new resolver added to the schema without a matching reader function
// fails here loudly, instead of silently no-opping at parse time.
static void TestResolverRegistryCoverage() {
  std::vector<std::string> missing;
  CHECK(ResolverRegistryComplete(&missing));
  if (!missing.empty()) {
    std::printf("  unregistered resolver(s):");
    for (const auto& m : missing) std::printf(" %s", m.c_str());
    std::printf("\n");
  }
}

// Wave 4 #3: element nesting is capped (200 levels) so a hostile deep document
// fails with a clear diagnostic instead of overflowing the native stack.
static void TestNestingDepthCap() {
  auto nested = [](int n) {
    std::string s = "<mujoco>\n<worldbody>\n";
    for (int i = 0; i < n; ++i) s += "<body>";
    for (int i = 0; i < n; ++i) s += "</body>";
    s += "\n</worldbody>\n</mujoco>\n";
    return s;
  };
  auto has_cap_err = [](const ParseResult& r) {
    for (const auto& d : r.errors)
      if (d.message.find("element nesting depth exceeds") != std::string::npos)
        return true;
    return false;
  };
  // 197 nested bodies -> reader recursion depth 199 (<=200): parses.
  ParseResult under = ParseMjcfString(nested(197), "<nest>");
  CHECK(under.ok());
  CHECK(!has_cap_err(under));
  // 199 nested bodies -> reader recursion depth 201 (>200): rejected loudly.
  ParseResult over = ParseMjcfString(nested(199), "<nest>");
  CHECK(!over.ok());
  CHECK(has_cap_err(over));
  // The diagnostic names the element and the depth.
  bool named = false;
  for (const auto& d : over.errors)
    if (d.message.find("<body>") != std::string::npos &&
        d.message.find("depth 201") != std::string::npos)
      named = true;
  CHECK(named);
}

// Wave 4 #4: a name repeated within a namespace raises a parse-time WARNING (both
// locations) without failing the parse; cross-namespace repeats never fire.
static void TestDuplicateNameWarning() {
  auto has_warn = [](const ParseResult& r, const char* sub) {
    for (const auto& d : r.warnings)
      if (d.message.find(sub) != std::string::npos) return true;
    return false;
  };
  // Two bodies "dup": warning carries both the first and second locations.
  ParseResult dup = ParseMjcfString(
      "<mujoco><worldbody>\n<body name='dup'/>\n<body name='dup'/>\n"
      "</worldbody></mujoco>\n",
      "m.xml");
  CHECK(dup.ok());  // warning-tier: parse still succeeds
  CHECK(has_warn(dup, "duplicate body name 'dup'"));
  CHECK(has_warn(dup, "first defined at m.xml:2"));
  // Cross-namespace repeat (a body and a geom both "x"): no warning.
  ParseResult cross = ParseMjcfString(
      "<mujoco><worldbody>\n"
      "<body name='x'><geom name='x' type='sphere' size='1'/></body>\n"
      "</worldbody></mujoco>\n",
      "m.xml");
  CHECK(cross.ok());
  CHECK(!has_warn(cross, "duplicate"));
  // Merged namespace: two actuator spellings sharing a name still collide.
  ParseResult act = ParseMjcfString(
      "<mujoco><worldbody>"
      "<body name='b'><joint name='j' type='hinge'/>"
      "<geom type='sphere' size='1'/></body></worldbody>"
      "<actuator>\n<motor name='a' joint='j'/>\n"
      "<position name='a' joint='j'/>\n</actuator></mujoco>\n",
      "m.xml");
  CHECK(has_warn(act, "duplicate actuator name 'a'"));
}

// Wave 4 #6: an <include> whose resolved path escapes the root model's directory
// tree is rejected by default and permitted under allow_external_includes.
static void TestIncludeTraversal() {
  std::filesystem::path dir = TempDir();
  std::filesystem::path root = dir / "root";
  WriteText(root / "inc.xml", R"(<mujocoinclude>
  <body name="inc"><geom type="sphere" size="0.1"/></body>
</mujocoinclude>)");
  WriteText(dir / "outside" / "secret.xml", R"(<mujocoinclude>
  <body name="secret"><geom type="sphere" size="0.1"/></body>
</mujocoinclude>)");
  WriteText(root / "uses_inside.xml", R"(<mujoco><worldbody>
  <include file="inc.xml"/>
</worldbody></mujoco>)");
  WriteText(root / "uses_outside.xml", R"(<mujoco><worldbody>
  <include file="../outside/secret.xml"/>
</worldbody></mujoco>)");
  auto has_escape_err = [](const ParseResult& r) {
    for (const auto& d : r.errors)
      if (d.message.find("resolves outside the model directory tree") !=
          std::string::npos)
        return true;
    return false;
  };
  // Inside-tree include: accepted.
  ParseResult in = ParseMjcfFile((root / "uses_inside.xml").string());
  CHECK(in.ok());
  CHECK(!has_escape_err(in));
  // Escaping include: rejected by default, and the file is never opened.
  ParseResult esc = ParseMjcfFile((root / "uses_outside.xml").string());
  CHECK(!esc.ok());
  CHECK(has_escape_err(esc));
  // Opt-in permits it.
  ParseOptions opt;
  opt.allow_external_includes = true;
  ParseResult ok = ParseMjcfFile((root / "uses_outside.xml").string(), opt);
  CHECK(ok.ok());
  CHECK(!has_escape_err(ok));
  std::filesystem::remove_all(dir);
}

int main() {
  TestResolverRegistryCoverage();
  TestNestingDepthCap();
  TestDuplicateNameWarning();
  TestIncludeTraversal();
  TestFixpoint();
  TestAngle();
  TestOrient();
  TestFromto();
  TestInertia();
  TestOrientDocumentOrder();
  TestOrientClassInheritance();
  TestArity();
  TestNumeric();
  TestPresence();
  TestMalformed();
  TestUnsupported();
  TestRoot();
  TestDefaults();
  TestDefaultClassNames();
  TestUnknownClassRef();
  TestAssets();
  TestTextureSource();
  TestInclude();
  TestIncludeErrors();
  TestContact();
  TestEquality();
  TestEqualityNegatives();
  TestTendon();
  TestActuators();
  TestActuatorNegatives();
  TestUnionOrderRoundTrip();
  TestSensors();
  TestSensorNegatives();
  TestCustomKeyframeExtension();
  TestMacrosDeformable();
  TestWaveBCanonicalization();

  std::printf("%d checks, %d failures\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
