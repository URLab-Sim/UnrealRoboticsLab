// Public-API self-sufficiency test.
//
// This translation unit includes ONLY the curated public umbrella headers under
// <protospec/...> -- never a generated, io/ or sdk-detail header. If it compiles
// and runs, the public surface is self-contained: a consumer needs nothing
// internal to load, edit and save a model.

#include <cstdio>
#include <filesystem>
#include <string>

// The public surface -- these five headers are the whole ProtoSpec include set a
// consumer touches. Nothing below reaches past them.
#include "protospec/model.h"
#include "protospec/io.h"
#include "protospec/reflect.h"
#include "protospec/sdk.h"
#include "protospec/save.h"

namespace io = ps::mjcf::io;
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

  // --- 3. STRUCTURAL SDK VERBS (public, runtime-typed) ------------------- //
  // Duplicate / Rename / Reparent / DeleteSubtree keyed on runtime pointers.
  auto* box_copy = sdk::Duplicate(model, &box).As<mj::Body>();
  CHECK(box_copy != nullptr);
  CHECK(sdk::Find<mj::Body>(model, "box_1") == box_copy);  // re-uniqued name
  CHECK(sdk::Rename(model, box_copy, "box_copy").ok);      // runtime rename
  CHECK(sdk::Find<mj::Body>(model, "box_copy") == box_copy);
  CHECK(sdk::Reparent(model, box_copy, &world).ok);        // pure-tree move
  auto del = sdk::DeleteSubtree(model, box_copy, /*cascade=*/true);
  CHECK(del.removed);
  CHECK(sdk::Find<mj::Body>(model, "box_copy") == nullptr);

  // --- 4. SAVE + reload --------------------------------------------------- //
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
