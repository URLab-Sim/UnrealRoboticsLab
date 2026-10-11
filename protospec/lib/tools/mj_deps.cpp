// Isolate MuJoCo's fatal dependency-scanner errors from the Unreal Editor.
// The scanner itself is upstream; this program only frames its completed result.

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace {

void WriteU32(std::ofstream& out, std::uint32_t value) {
  for (int i = 0; i != 4; ++i) {
    out.put(static_cast<char>((value >> (8 * i)) & 0xff));
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: urlab_mj_deps <source.xml> <output.bin>\n";
    return 2;
  }

  mjStringVec paths;
  // mju_error terminates this process on a missing/invalid recursive XML
  // dependency. Never call it inside the editor or replace its global handler.
  mju_getXMLDependencies(argv[1], &paths);
  std::sort(paths.begin(), paths.end());
  if (paths.size() > std::numeric_limits<std::uint32_t>::max()) return 3;
  for (const std::string& path : paths) {
    if (path.size() > std::numeric_limits<std::uint32_t>::max()) return 3;
  }

  std::ofstream out(argv[2], std::ios::binary | std::ios::trunc);
  if (!out) {
    std::cerr << "cannot open dependency output\n";
    return 3;
  }
  out.write("URLDEP01", 8);
  WriteU32(out, static_cast<std::uint32_t>(paths.size()));
  for (const std::string& path : paths) {
    WriteU32(out, static_cast<std::uint32_t>(path.size()));
    out.write(path.data(), static_cast<std::streamsize>(path.size()));
  }
  out.close();
  if (!out) {
    std::cerr << "cannot finish dependency output\n";
    return 3;
  }
  return 0;
}
