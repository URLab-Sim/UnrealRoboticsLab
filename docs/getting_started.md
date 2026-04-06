# Getting Started

## Prerequisites

- Unreal Engine 5.4+
- Windows 10/11
- Python 3.11+ (optional, for external policy control)

## Installation

1. Clone the repository into your Unreal project's `Plugins/` directory:
   ```bash
   cd "YourProject/Plugins"
   git clone https://github.com/URLab-Sim/UnrealRoboticsLab.git
   ```
2. Build the third-party dependencies (one-time):
   ```bash
   cd UnrealRoboticsLab/third_party
   # Run third_party/build_all.ps1
   ```
   Pre-built binaries for Windows are included in `third_party/install/` (MuJoCo, libzmq, CoACD).
3. Open your `.uproject` in Unreal Engine -- the plugin loads automatically.
4. (Optional) Set up the Python bridge for external control:
   ```bash
   cd UnrealRoboticsLab/urlab_bridge
   pip install uv
   uv sync
   ```

## Import Your First Robot

### From MJCF XML

1. Get a robot XML (e.g., from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)).
2. Drag the XML into the Unreal Content Browser. The importer auto-runs `Scripts/clean_meshes.py` to convert meshes to GLB (falls back gracefully if Python is missing).
3. A Blueprint is auto-generated with all joints, bodies, actuators, and sensors as components.

### Quick Convert (Static Meshes)

1. Place static mesh actors in your level (furniture, props, etc.).
2. Add an `MjQuickConvertComponent` to each actor.
3. Set to **Dynamic** for physics bodies or **Static** for fixed collision.
4. Enable `ComplexMeshRequired` for non-convex shapes (uses CoACD decomposition).

## Set Up the Scene

1. Place an `MjManager` actor in your level (one per level).
2. Place your imported robot Blueprint(s) in the level.
3. Hit Play -- physics simulation starts automatically.
4. The MjSimulate widget appears (if `bAutoCreateSimulateWidget` is enabled on the Manager).

## Control a Robot

### From the Dashboard

- Use the actuator sliders in the MjSimulate widget to move joints.
- Set control source to UI (on manager or per-articulation) to use dashboard sliders instead of ZMQ.

### From Python (ZMQ)

1. `cd` into `urlab_bridge/`.
2. Install: `uv sync` (or `pip install -e .`).
3. Run a policy: `uv run src/run_policy.py --policy unitree_12dof`
4. Or use the GUI: `uv run src/urlab_policy/policy_gui.py`
5. Select your articulation and policy, click Start.

### From Blueprint

```cpp
// Set actuator control value
MyArticulation->SetActuatorControl("left_hip", 0.5f);

// Read joint state
float Angle = MyArticulation->GetJointAngle("left_knee");

// Read sensor data
float Touch = MyArticulation->GetSensorScalar("foot_contact");
TArray<float> Force = MyArticulation->GetSensorReading("wrist_force");
```

All functions are `BlueprintCallable`.

## Debug Visualization

See [Hotkeys](guides/blueprint_reference.md#hotkeys) for keyboard shortcuts.

## Next Steps

- [features.md](features.md) -- complete feature reference
- [MJCF Import](guides/mujoco_import.md) -- import pipeline details
- [Blueprint Reference](guides/blueprint_reference.md) -- all Blueprint-callable functions and hotkeys
- [ZMQ Networking](guides/zmq_networking.md) -- protocol, topics, and Python examples
- [Policy Bridge](guides/policy_bridge.md) -- RL policy deployment
- [Developer Tools](guides/developer_tools.md) -- schema tracking, debug XML, build/test skills
