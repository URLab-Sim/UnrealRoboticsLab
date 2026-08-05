// First-party MuJoCo engine-plugin registration, shared by every tool that
// loads plugin-bearing corpus models (mj_model_diff, ps_native_diff, ps_compile).
//
// MuJoCo's plugin registry is process-global: mj_loadPluginLibrary(dll) runs the
// library's mjPLUGIN_LIB_INIT, which calls mjp_registerPlugin into that global
// table. So a single call per process, before the first mj_loadXML of a plugin
// model, makes mujoco.elasticity.*, mujoco.sdf.*, mujoco.sensor.touch_grid and
// mujoco.pid resolvable on every subsequent load. This helper is that call.
//
// The plugin DLLs are the ones prebuilt alongside the vendored mujoco.dll
// (build/bin/Release/{elasticity,sdf_plugin,sensor,actuator}.dll); they import
// mujoco.dll by name, so they bind to the process's already-loaded mujoco.dll
// and register into its table.
#ifndef PROTOSPEC_HARNESS_PLUGIN_REGISTRY_H
#define PROTOSPEC_HARNESS_PLUGIN_REGISTRY_H

#include <string>

namespace ps::plugin {

// Load the first-party plugin libraries into MuJoCo's global registry, once per
// process (idempotent: repeat calls are no-ops). The directory holding the
// plugin DLLs is resolved in this order:
//   1. `dir` argument, when non-empty (a tool's --plugin-dir);
//   2. the PROTOSPEC_PLUGIN_DIR environment variable, when set;
//   3. the compile-time default (the vendored build/bin/Release beside mujoco.dll).
// Missing DLLs are skipped silently -- a checkout without a given plugin simply
// leaves those models unloadable, exactly as before.
void RegisterFirstPartyPlugins(const std::string& dir = {});

}  // namespace ps::plugin

#endif  // PROTOSPEC_HARNESS_PLUGIN_REGISTRY_H
