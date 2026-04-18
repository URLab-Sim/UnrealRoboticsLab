# Features

## Core Architecture

- Full MuJoCo C API accessible (`mjModel*`, `mjData*`) -- users can call any MuJoCo function directly
- Async physics thread with configurable timestep
- Component-based design matching MJCF element hierarchy
- Singleton `MjManager` orchestrates compilation, stepping, and state

## MJCF Import

Drag-and-drop XML import into Content Browser. Auto-generates Unreal Blueprints with full component hierarchy, mesh conversion, material import, and default class inheritance.

See [MuJoCo Import](guides/mujoco_import.md) for details.

## Quick Convert

- Convert any Unreal Static Mesh to a MuJoCo physics body with one click
- CoACD convex decomposition with adjustable threshold (0.01-1.0)
- Visual/collision mesh separation (group 2 visual, group 3 collision)
- Static, dynamic, or mocap-driven modes
- Friction, solver parameters configurable per component

## Heightfield Terrain

- Convert Unreal Landscapes to MuJoCo heightfield collision
- Configurable sample resolution (2-512)
- Complex trace mode for accurate mesh sampling
- Actor whitelist filtering
- Binary cache for fast recompilation
- Editor grid preview visualization

## Physics Simulation

- Solvers: PGS, CG, Newton
- Integrators: Euler, RK4, Implicit, ImplicitFast
- Sim speed control (5-100%)
- Pause, resume, reset, single-step
- Snapshot capture and restore (full state)
- Keyframe reset and hold
- Sleep support with per-body policies
- Multi-CCD collision detection
- Gravity, wind, density, viscosity

## Actuators (8 types)

Motor, Position, Velocity, IntVelocity, Damper, Cylinder, Muscle, Adhesion

- All transmission types: joint, tendon, site, body, slider-crank
- Runtime control via `SetControl()` (BlueprintCallable)
- Gain/bias parameter arrays
- Control/force/activation range limits

## Sensors (40+ types)

Site-attached: Touch, Accelerometer, Velocimeter, Gyro, Force, Torque, Magnetometer, RangeFinder, CamProjection

Joint/tendon/actuator: Pos, Vel, Frc for each

Ball joint: Quat, AngVel

Frame: Pos, Quat, XYZ axes, LinVel, AngVel, LinAcc, AngAcc

Subtree: COM, LinVel, AngMom

Geometric: InsideSite, GeomDist, GeomNormal, GeomFromTo

Global: Contact, EPotential, EKinetic, Clock

- All readable via `GetSensorReading()` / `GetSensorScalar()` on the articulation (BlueprintCallable)

## Controllers

PD, passthrough, keyframe, and twist controllers for articulations. Keyframe controller includes built-in presets for Franka, UR5e, Spot, ANYmal, and Go2.

See [Controller Framework](guides/controller_framework.md) for details.

## Debug Visualization

Hotkey-driven toggles during PIE for contact forces, collision wireframes, joint axes + limit arcs, visual-mesh visibility, constraint-island colouring, instance + semantic segmentation shaders with sleep modulation, and smooth muscle/tendon tubes with activation-driven colour and radius.

See [Debug Visualization](guides/debug_visualization.md) for the full hotkey table and per-mode details.

## Camera System

- **MjCamera**: scene capture attached to MuJoCo sites; per-camera `CaptureMode` for photoreal RGB, depth, semantic segmentation, or instance segmentation; streams via ZMQ; respects post-process volumes
- **MjOrbitCamera**: auto-orbiting cinematic camera with target detection, height oscillation, configurable focal length
- **MjKeyframeCamera**: waypoint-based camera path with smooth interpolation, `O` key play/pause

See [Camera Capture Modes](guides/camera_capture_modes.md) for per-mode behaviour and the simulate-widget preview, and [Blueprint Reference](guides/blueprint_reference.md#mjkeyframecameraactor) for camera API details.

## Networking (ZMQ)

Binary pub/sub protocol broadcasting joint state, sensors, and camera images. Receives actuator targets and PD gains. Prefix-based filtering for multi-articulation scenes.

See [ZMQ Networking](guides/zmq_networking.md) for details.

## Python Integration (urlab_bridge)

- **Policy GUI**: DearPyGui dashboard for policy selection, articulation targeting, force twist override, motion/gait preset selection
- **UnrealEnv**: RoboJuDo-compatible environment over ZMQ with auto-detection, forward kinematics, born-place alignment
- Pre-integrated policies: Unitree G1 locomotion (12/29 DOF), BeyondMimic (dance/violin/waltz/jump), Walk-These-Ways Go2, AMO, H2H, Smooth, ASAP
- **ROS 2 bridge**: republishes ZMQ streams to standard ROS 2 topics

## Replay System

Record and play back full simulation state. Supports CSV trajectory import, snapshot capture/restore, and multi-session management.

See [Blueprint Reference](guides/blueprint_reference.md) for recording/replay API.

## Blueprint API

40+ BlueprintCallable functions for simulation control, state queries, actuator commands, snapshots, and replay.

See [Scripting with Blueprints](guides/blueprint_reference.md) for the full API reference.

## MjSimulate Widget

- In-editor dashboard replicating MuJoCo's simulate UI
- Physics options: timestep, solver, integrator, iterations, sim speed
- Per-actuator control sliders
- Joint state readouts
- Sensor value display
- Camera feed thumbnails
- Articulation selector dropdown
- Record/replay/snapshot controls
- Possess button for spring-arm camera follow

## Forces

- **MjImpulseLauncher**: velocity-based object launching (matching MuJoCo's object_launcher_plugin approach)
- Direct mode (arrow direction + speed) or targeted mode (ballistic arc toward target actor)
- Reset-and-fire for iterative testing during PIE
- `F` key fires all launchers in scene

## Simulation Hotkeys

See [Hotkeys](guides/blueprint_reference.md#hotkeys) for the complete list of keyboard shortcuts.

## Mesh Preparation Tool

- `Scripts/clean_meshes.py`: XML-aware mesh converter
- Parses MJCF, detects GLB stem conflicts, renames automatically
- Converts OBJ/STL to GLB with UV preservation
- Strips embedded textures to prevent Interchange import conflicts
- Outputs updated `_ue.xml` ready for drag-and-drop import
