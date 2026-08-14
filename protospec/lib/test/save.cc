// ProtoSpec SDK save surface. See protospec/save.h.

#include "protospec/save.h"

#include <fstream>
#include <system_error>

#include "mjcf.h"

namespace ps::sdk {

namespace fs = std::filesystem;
namespace io = ps::mjcf::io;

std::filesystem::path ModelAssetDir(const mj::Model& model,
                                    const std::filesystem::path& xml_path) {
  fs::path dir = xml_path.parent_path();
  if (dir.empty()) dir = fs::path(".");
  // Last authored <compiler meshdir> wins, mirroring the compiler.
  std::string meshdir;
  for (const auto& c : model.compilers) {
    if (c && c->meshdir) meshdir = *c->meshdir;
  }
  if (!meshdir.empty()) dir /= meshdir;
  return dir;
}

bool WriteAssetFile(const std::filesystem::path& dir, const std::string& name,
                    const std::uint8_t* bytes, std::size_t size) {
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) return false;
  std::ofstream out(dir / name, std::ios::binary);
  if (!out) return false;
  if (size != 0) {
    out.write(reinterpret_cast<const char*>(bytes),
              static_cast<std::streamsize>(size));
  }
  out.close();
  return static_cast<bool>(out);
}

std::size_t ExternalizeAssets(const mj::Model& model,
                              const std::filesystem::path& xml_path,
                              const std::vector<InMemoryAsset>& assets) {
  if (assets.empty()) return 0;
  const fs::path dir = ModelAssetDir(model, xml_path);
  std::size_t written = 0;
  for (const InMemoryAsset& a : assets) {
    if (WriteAssetFile(dir, a.name, a.bytes.data(), a.bytes.size())) ++written;
  }
  return written;
}

bool Save(const mj::Model& model, const std::filesystem::path& path) {
  const std::string mjcf = io::WriteMjcf(model);
  // Empty means the model holds a value MJCF cannot represent (see mjcf.h);
  // do not produce an empty file.
  if (mjcf.empty()) return false;
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out.write(mjcf.data(), static_cast<std::streamsize>(mjcf.size()));
  out.close();
  return static_cast<bool>(out);
}

bool SaveAs(const mj::Model& model, const std::filesystem::path& path,
            std::vector<InMemoryAsset>* assets) {
  if (assets && !assets->empty()) {
    if (ExternalizeAssets(model, path, *assets) != assets->size()) return false;
    assets->clear();
  }
  return Save(model, path);
}

}  // namespace ps::sdk
