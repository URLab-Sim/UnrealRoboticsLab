# Changelog

Notable changes to UnrealRoboticsLab, newest first.

URLab is in **alpha**: the public API and on-disk formats are not yet stable, and
a milestone can include breaking changes. A first **beta** will follow once the
feature set settles.

## v0.5.0-alpha (2026-06-14)

The render-pipeline, codegen, and tooling milestone.

### Added

- **Coherent render snapshot.** The physics thread now publishes one consistent
  snapshot per step, and body, geom, and flexcomp transforms are driven from it.
  This removes the tearing that came from reading `mjData` mid-step on the render
  thread.
- **Engine command queue.** Mocap updates and external wrenches are routed into
  the physics thread through a single ordered queue instead of touching `mjData`
  directly.
- **Camera intrinsics on import.** `fovy` is derived from a camera's `focal`,
  `focalpixel`, or `sensorsize` when present, so MJCF cameras authored with
  real-sensor intrinsics import with the correct field of view.
- **MuJoCo thread pool.** `mju_threadpool` is exposed as an opt-in property on the
  physics engine, with the worker count auto-detected on Windows and Linux, and
  toggled at runtime through `set_sim_options`.

### Changed

- **Codegen rebuilt.** The component generator is now driven by three snapshots
  (the `mjxmacro` field tables, the MJCF schema, and a clang-AST introspection
  pass) rather than hand-maintained lists. The pass also splits the generated code
  into smaller modules and adds a correctness sweep, including a fix for `mjtBool`
  fields that were silently typed as `mjtNum*` and a loud diagnostic for any
  unmapped type.
- **MuJoCo 3.8.1 to upstream `main`.** Bumped 3.8.1 -> 3.9.0, then pinned to the
  latest `main` (3.10.0-dev). The vendored `mjspecmacro.h` was removed and is now
  read from the MuJoCo install.
- **Epic clang-format adopted** across the repo, with a codegen format step so
  generated files match, plus a `.git-blame-ignore-revs` entry for the reformat.
- **Documentation redesigned** from the ground up, with new Roadmap and Changelog
  pages.

### Fixed

- Inline-mesh OBJ import, MJCF 3.x layered materials, and ORM / normal-map wiring.
- Geom `rgba` and geom `type` now resolve through the full default class chain.
- Material-instance names are stripped of path separators that broke asset
  creation.
- Mesh normal cleanup is guarded behind a `networkx` import check instead of
  hard-failing when the optional dependency is missing.

## v0.4.0-alpha (2026-05-27)

The Python bridge and remote-control milestone.

### Added

- **Bridge dispatcher and transport layer.** External clients connect over ZMQ
  (REQ/REP step channel, PUB/SUB state channel) or shared memory, with an
  in-editor bridge server subsystem managing the lifecycle.
- **Extended RPC surface** for discovery, stepping, control, and simulation
  options, exposed to the `urlab_bridge` Python package.
- **Assets in the handshake.** A client can opt in to receiving the compiled MJCF
  and VFS assets as bytes on connect, so it can build a matching local model.
- **`set_sim_options`** with forward-op and raw enable / disable flags.
- **Python API reference and networking documentation.**

### Changed

- **Data-driven MJCF round-trip** for import and export, which also fixed the
  Robotiq 2F-85 gripper.
- **MuJoCo bumped 3.7.0 -> 3.8.1.**

### Fixed

- Direct and puppet steps no longer block on the realtime pacer.
- Imported geoms inherit their type from class defaults and respect user
  `RelativeScale3D` on export.
- Case-correct MuJoCo include paths; libzmq resolved by glob rather than a
  hardcoded toolset suffix.

## v0.3.0-alpha (2026-04-29)

The Linux support milestone.

### Added

- **Linux build and runtime.** The plugin compiles and `dlopen`s on UE Linux,
  built against Unreal's bundled clang and libc++ for ABI compatibility.
  Third-party libraries are staged into `Binaries/Linux/` so `$ORIGIN` RPATH
  resolution finds them at runtime.
- **`Scripts/build_and_test_linux.sh`** for headless build-and-test.
- **Linux setup and installation guides.**

### Fixed

- `TCHAR_TO_UTF8` conversions kept alive across `mjs_*` calls, which otherwise
  freed the strings mid-call.
- `mju_user_*` callbacks direct-assigned on Linux instead of going through the DLL
  handle.
- CoACD `_WIN32` assumptions patched through a custom overlay.

## v0.2.0-alpha (2026-04-19)

The visualization and MJCF-coverage milestone.

### Added

- **Debug visualization.** Body-island and segmentation overlay shaders, spatial
  tendons and activation-driven muscle tubes, a capsule primitive, and flexcomp
  runtime visualization via dynamic meshes.
- **Mouse-driven body perturbation** for poking the simulation interactively.
- **Per-camera capture modes** (Real, Depth, Segmentation) with an in-editor
  preview.
- **flexcomp import and spec registration.**
- **Explicit Python interpreter setup dialog** replacing the silent auto-install.
- **GitHub issue and pull-request templates** and the `build_and_test` helper
  scripts.

### Changed

- **MuJoCo bumped to 3.7.0**, adding the dcmotor actuator and the flex equality
  types.
- **Third-party dependencies pinned as git submodules** at exact commits, with
  Build.cs drift guards that detect a stale install.
- **Minimum Unreal Engine version raised to 5.7.**

### Fixed

- Physics-thread stability: a MuJoCo error hook, several race fixes, and camera
  cleanup on teardown.
- Material crash when browsing imported mesh assets.
- Mesh scale defined in MJCF `<default>` blocks; tendon and muscle MJCFs now
  compile.
- Component names synced from the MJCF `name=` attribute on import and on
  Blueprint compile.

## v0.1.0-alpha (2026-04-06)

The initial release.

### Added

- **MuJoCo physics embedded in Unreal Engine 5.** Import MJCF models into the
  Content Browser, simulate with MuJoCo on a dedicated thread, and render in
  Unreal with accurate contacts and photorealistic output. Articulations, geoms,
  joints, and actuators map to Unreal components, with CoACD convex decomposition
  for collision and libzmq available for external messaging.
