# Roadmap

Where UnrealRoboticsLab is headed. This is direction, not a commitment or a
schedule, and priorities shift. If you want to take one on, open an issue or
discussion so we can point you at the code.

- **Real screenshots and media.** Replace the placeholder images across the docs
  with real captures, and add short clips of the editor workflow.
- **Build and release automation.** Add CI that builds the plugin, runs the
  URLab test suite, and checks formatting on pull requests. Ship precompiled
  Windows and Linux binaries so the plugin can be installed without building
  MuJoCo, CoACD, and libzmq from source.
- **Camera render fixes.** Make the non-RGB capture modes correct: depth should
  honour the configured near and far planes, and segmentation should render flat
  (unlit, one flat colour per object/class) rather than through the tone-mapped
  path. Also finish wiring the imported camera intrinsics into the Unreal
  capture: principal-point offset, sensor-size-driven focal length, and
  orthographic projection (only field of view is applied today).
- **Sensor device library.** A collection of preconfigured, drop-in real-world
  sensors as concrete instances: cameras such as the Intel RealSense D435i, and
  lidars such as Ouster. Adding a new device should be a small subclass that sets
  resolution, intrinsics, and rates. Cameras build on the existing capture modes
  and intrinsics; lidar first needs an Unreal-side raycast sensor (the rangefinder
  is MuJoCo-passthrough only today), then concrete lidar types on top.
- **Native policies in packaged builds.** Run trained policies natively inside a
  cooked/packaged game (on-device inference), so a shipped build runs
  self-contained without the Python bridge in the loop.
- **Native ROS 2 (rclcpp) integration.** The Python `urlab_bridge` already
  re-broadcasts URLab data to ROS 2; this is a native, in-engine alternative.
  Link rclcpp into the plugin so a packaged URLab build can publish and subscribe
  to ROS 2 directly, with no separate bridge process. The sensor and control
  abstractions should make the wiring straightforward; the bulk of the work is on
  the build and third-party side, bringing ROS 2 into the Unreal build.
- **Load models over the wire.** Let a client send an MJCF plus its mesh and
  texture bytes for URLab to import, so a model can be loaded with no files on
  the host. Shipping a compiled model and assets to a client already works; this
  is the inbound direction.
- **Multi-client render server.** Grow the single-client bridge into a headless
  render server that several clients can attach to, drive, and stream cameras
  from: client discovery, a multi-client transport with an optional driver lease
  so one client owns the integrator, and per-camera render streams.
- **Domain randomisation.** Automated randomisation of materials and textures,
  lighting, and physics parameters, scriptable over the bridge for sim-to-real
  transfer and dataset generation.
- **VR teleoperation** for collecting demonstrations and direct control.
- **Runtime camera-mode switching over the bridge.** Capture mode (RGB / depth /
  segmentation) is set at design time today; expose it as a runtime op.
- **Keybinds aligned with MuJoCo `simulate`.** Rework the in-editor hotkeys to
  mirror MuJoCo's `simulate` application (pause, reset, step, speed, and so on)
  so the controls feel familiar to MuJoCo users.
- **Simulate widget overhaul.** Rework the in-editor simulation control panel
  (`UMjSimulateWidget`) into a cleaner, more capable UI.
- **Codegen: flexcomp sub-elements.** Bring flexcomp's `<contact>`, `<edge>`,
  `<elasticity>`, and `<pin>` sub-elements under the codegen; they are hand-rolled
  today pending per-sub-element property support.
- **Raw control-path cleanup.** Simplify the raw control mode's `NetworkValue`
  indirection and dual-write.
- **Script consolidation.** Merge the overlapping Linux build/test scripts and
  keep the helper scripts as cross-platform pairs.

## Known issues

- Mesh import loses fine-grained normals on UE 5.7 (the GLTF importer lacks
  `bWeldVertices`).
- The build-time third-party drift check self-disables on non-git checkouts,
  which can hide the most common setup failure (forgetting to build the
  dependencies).
