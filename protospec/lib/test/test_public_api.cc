// Public-API self-sufficiency test.
//
// This translation unit includes ONLY the curated public umbrella headers under
// <protospec/...> -- never a generated, io/, bridge/, validate/, or sdk-detail
// header. If it compiles and runs, the public surface is self-contained: a
// consumer (the studio editor today, the UE plugin tomorrow) needs nothing
// internal to load, edit, validate, compile, step, and save a model.
//
// <mujoco/mujoco.h> is the simulation engine the CONSUMER brings; ProtoSpec's
// own headers forward-declare mjModel/mjData and never include it. Stepping the
// compiled mjModel is the consumer's job, shown here for the full round trip.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include <mujoco/mujoco.h>

// The public surface -- these six headers are the whole ProtoSpec include set a
// consumer touches. Nothing below reaches past them.
#include "protospec/model.h"
#include "protospec/io.h"
#include "protospec/validate.h"
#include "protospec/compile.h"
#include "protospec/reflect.h"
#include "protospec/sdk.h"
#include "protospec/save.h"

namespace io = ps::mjcf::io;
namespace validate = ps::mjcf::validate;
namespace sdk = ps::sdk;
namespace mj = ps::mjcf;

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

// The MJCF the consumer starts from -- a ground plane, nothing else.
static const char* kBaseXml = R"(<mujoco model="hello">
  <worldbody>
    <body name="ground">
      <geom name="floor" type="plane" size="1 1 0.1"/>
    </body>
  </worldbody>
</mujoco>)";

int main() {
  // --- 1. LOAD ------------------------------------------------------------ //
  io::ParseResult parsed = io::ParseMjcfString(kBaseXml, "hello.xml");
  CHECK(parsed.ok());
  if (!parsed.ok()) {
    for (const auto& e : parsed.errors) std::printf("  %s\n", e.Render().c_str());
    return 1;
  }
  mj::Model& model = *parsed.model;

  // --- 2. EDIT (SDK only) ------------------------------------------------- //
  // A falling box: new body under world, free joint, sized primitive geom.
  mj::Body& world = sdk::World(model);
  mj::Body& box = sdk::AddBody(world, "box");
  box.pos = std::array<double, 3>{0, 0, 1};
  sdk::AddFreeJoint(box, "box_free");
  mj::Geom& g = sdk::AddPrimitive(box, mj::GeomType::box, "box_geom");
  CHECK(g.size.has_value());  // AddPrimitive seeded a compilable size

  // A textured material, wired through the SDK appearance verbs, then bound to
  // the geom by reference -- no raw ps::Ref, no raw layer vector.
  mj::Texture& tex = sdk::AddTexture(model, "grid");
  sdk::SetTextureBuiltin(tex, mj::TextureBuiltin::checker);
  tex.type = mj::TextureType::twod;
  tex.width = 64;
  tex.height = 64;
  mj::Material& mat = sdk::AddMaterial(model, "grid_mat");
  sdk::AddMaterialLayer(mat, "rgb", "grid");
  sdk::SetRef(g.material, mat);  // set a typed ref from the target element
  CHECK(g.material.has_value() && g.material->name == "grid_mat");

  // Identity + lookup through the public SDK (no ps::sdk::detail).
  CHECK(sdk::Name(box) && *sdk::Name(box) == "box");
  CHECK(sdk::TypeOf(g) == mj::ElementType::Geom);
  CHECK(sdk::Find<mj::Geom>(model, "box_geom") == &g);

  int geom_count = 0;
  sdk::ForEachOfType<mj::Geom>(model, [&](mj::Geom&) { ++geom_count; });
  CHECK(geom_count == 2);  // floor + box_geom

  // Reflection surface is reachable and describes the model.
  CHECK(std::string(mj::reflect::Describe(mj::ElementType::Geom).xml) == "geom");

  // --- 3. VALIDATE -------------------------------------------------------- //
  auto diags = validate::Validate(model);
  int errors = 0;
  for (const auto& d : diags)
    if (d.severity == validate::Severity::Error) ++errors;
  CHECK(errors == 0);

  // --- 4. COMPILE --------------------------------------------------------- //
  mj::Compiled compiled = mj::Compile(model);
  CHECK(compiled.ok());
  if (!compiled.ok()) {
    for (const auto& e : compiled.report.errors)
      std::printf("  %s\n", e.Render().c_str());
    return 1;
  }
  mjModel* m = compiled.model.get();
  CHECK(m != nullptr);

  // The Binding maps our tree element back to the compiled id.
  auto gid = compiled.binding.Id(g);
  CHECK(gid.has_value());
  CHECK(compiled.binding.GeomAt(*gid) == &g);

  // --- 5. STEP (consumer's engine) --------------------------------------- //
  mjData* d = mj_makeData(m);
  CHECK(d != nullptr);
  for (int i = 0; i < 10; ++i) mj_step(m, d);
  CHECK(d->time > 0.0);
  CHECK(std::isfinite(d->qpos[2]));  // box height stayed finite under gravity

  mj_deleteData(d);

  // --- 5c. STRUCTURAL SDK VERBS (public, runtime-typed) ------------------- //
  // Duplicate / Rename / Reparent / DeleteSubtree keyed on runtime pointers --
  // the binding is a snapshot, so these follow the compile (they invalidate it).
  auto* box_copy = sdk::Duplicate(model, &box).As<mj::Body>();
  CHECK(box_copy != nullptr);
  CHECK(sdk::Find<mj::Body>(model, "box_1") == box_copy);  // re-uniqued name
  CHECK(sdk::Rename(model, box_copy, "box_copy").ok);      // runtime rename
  CHECK(sdk::Find<mj::Body>(model, "box_copy") == box_copy);
  CHECK(sdk::Reparent(model, box_copy, &world).ok);        // pure-tree move
  auto del = sdk::DeleteSubtree(model, box_copy, /*cascade=*/true);
  CHECK(del.removed);
  CHECK(sdk::Find<mj::Body>(model, "box_copy") == nullptr);

  // --- 6. SAVE + reload --------------------------------------------------- //
  std::filesystem::path out =
      std::filesystem::temp_directory_path() / "protospec_public_api_hello.xml";
  CHECK(sdk::Save(model, out));
  io::ParseResult reloaded = io::ParseMjcfFile(out.string());
  CHECK(reloaded.ok());
  CHECK(reloaded.ok() && sdk::Find<mj::Geom>(*reloaded.model, "box_geom"));
  std::error_code ec;
  std::filesystem::remove(out, ec);

  std::printf("test_public_api: %d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
