// std::getenv is the portable env read; MSVC's _CRT deprecation of it is a
// false positive for this read-only use.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "plugin_registry.h"

#include <cstdlib>
#include <filesystem>
#include <string>

#include <mujoco/mujoco.h>

#ifndef PROTOSPEC_DEFAULT_PLUGIN_DIR
#define PROTOSPEC_DEFAULT_PLUGIN_DIR ""
#endif

namespace ps::plugin {
namespace {

// The first-party plugin libraries prebuilt beside mujoco.dll. Bare filenames;
// the resolved directory is prepended. Together they cover every plugin the
// corpus references: elasticity (cable/solid), sdf (torus/gear/nut/bolt/bowl),
// sensor (touch_grid) and actuator (pid).
constexpr const char* kPluginLibs[] = {
    "elasticity",
    "sdf_plugin",
    "sensor",
    "actuator",
};

#if defined(_WIN32)
constexpr const char* kLibExt = ".dll";
#elif defined(__APPLE__)
constexpr const char* kLibExt = ".dylib";
#else
constexpr const char* kLibExt = ".so";
#endif

std::string ResolveDir(const std::string& dir) {
  if (!dir.empty()) return dir;
  if (const char* env = std::getenv("PROTOSPEC_PLUGIN_DIR"); env && *env)
    return env;
  return PROTOSPEC_DEFAULT_PLUGIN_DIR;
}

}  // namespace

void RegisterFirstPartyPlugins(const std::string& dir) {
  static bool done = false;
  if (done) return;
  done = true;

  const std::string base = ResolveDir(dir);
  if (base.empty()) return;

  std::error_code ec;
  const std::filesystem::path root(base);
  for (const char* lib : kPluginLibs) {
    // Windows builds these beside mujoco.dll as `<name>.dll`; on Linux/macOS
    // CMake emits the `lib<name>.so`/`.dylib` convention. Try both.
    const std::filesystem::path bare = root / (std::string(lib) + kLibExt);
    const std::filesystem::path prefixed =
        root / ("lib" + std::string(lib) + kLibExt);
    if (std::filesystem::exists(bare, ec))
      mj_loadPluginLibrary(bare.string().c_str());
    else if (std::filesystem::exists(prefixed, ec))
      mj_loadPluginLibrary(prefixed.string().c_str());
  }
}

}  // namespace ps::plugin
