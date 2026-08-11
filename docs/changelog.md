# Changelog

Notable changes to UnrealRoboticsLab, newest first.

URLab is in **beta**: the public API and on-disk formats are close to stable,
but a milestone can still include breaking changes. The last alpha is preserved
on the `alpha` branch and the `v0.5.0-alpha` tag.

## v0.6.0-beta (2026-08-11)

The beta. Components are the model.

**Breaking: MjArticulation assets from any alpha will not open.** Their
components are the previous generation's classes and nothing migrates them.
Re-import the MJCF, or stay on the `alpha` branch.

### Added

- **ProtoSpec.** MJCF is now an object model generated from MuJoCo's own
  `mjcf.schema`, so a version bump regenerates the element tree instead of being
  hand-followed. Every attribute is stored as a `TOptional`, where unset means
  the document did not author it and the default class answers instead.
- **Stock parity.** An imported model compiles to exactly what `mj_loadXML`
  produces, checked field by field against every robot in a MuJoCo Menagerie
  checkout (`URLab.Parity.StockDiffMenagerie`, opt-in via `URLAB_MENAGERIE`).
- **MuJoCo engine plugins.** The first-party plugin libraries are built,
  installed and loaded, so a model using `mujoco.pid`, cable or shell
  elasticity, touch grids or SDF shapes compiles and runs.
- **Attach conflict policy.** `AMjArticulation` exposes MuJoCo's `mjtConflict`
  so a robot joining a scene can refuse to have its `<option>` replaced.
- **Convex decomposition, finished.** Right-click a mesh geom in the Blueprint
  editor's component tree. Threshold and CoACD's extrude are offered where the
  action is invoked, hulls become real `<mesh>` elements and Unreal assets
  beside the articulation's own, and the geom references them.

### Changed

- **The XML compile path is gone.** Components are read straight into an
  `mjSpec`; nothing serialises MJCF text to reach the compiler any more.
- **MuJoCo 3.11.1.**
- **An imported model's `<option>` reaches the scene.** The scene's
  `<compiler conflict>` is `merge`, not MuJoCo's `warning` default: the attach
  target is a scene nobody authored, so under `warning` every field an import
  brought lost to a default. Editable on the manager, and per-robot
  `AttachConflict` still overrides.
- **Breaking (RPC): the control-owner override is `control_owner`.** It was
  `source`, which `set_control_source` already uses for `"zmq"` | `"ui"` -- so
  that op could never pass its own ownership check. Requests that do not set it
  are unaffected; the owner still falls back to `session_id`.
- Line endings are LF in the repository on every platform, and the formatter
  no longer fights the code generator over the generated tree.

### Fixed

- **`multiccd` follows MuJoCo again.** The scene manager was disabling it on
  every scene. That mirrored MuJoCo's default when the flag was opt-in; MuJoCo
  has since moved it to the disable family, where it defaults on, so URLab was
  overriding it. Convex-convex pairs generated one contact point where MuJoCo
  generates several, and any scene with mesh or primitive contacts stepped
  differently from `mj_loadXML` of the same document.
- **Control values keep their precision.** The step RPC narrowed `ctrl` to
  `float` on the way to the actuator slots, so a `float64` setpoint arrived
  rounded and the sim diverged from stock MuJoCo given identical input. The
  path is `double` end to end, which is what `mjtNum` is.
- **The PD controller computes in `mjtNum`.** Its law read `d->qpos` and wrote
  `d->ctrl` through `float` locals, rounding the target and the state it was
  given. Gains stay `float`, which is how they are authored.
- **Convex decomposition finds its mesh.** It looked for a child
  `UStaticMeshComponent`, which is the visualiser's output rather than the
  authored geometry, and does not exist at all on the Blueprint template the
  right-click menu runs against. It resolves the geom's `<mesh>` element now,
  the same source the geom is drawn from.
- Assets whose visual and collision meshes share a basename no longer collide
  in MuJoCo's VFS, which silently gave collision geometry the visual mesh.
- A document naming its root default class `main` explicitly no longer loses
  every class nested inside it.
- The plugin builds again without the editor, so a game can be packaged.

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
