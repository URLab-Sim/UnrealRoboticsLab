# Getting Started

## Prerequisites

* **Unreal Engine 5.7+** — The C++ plugin code and third-party libraries should compile on earlier UE5 versions, but the bundled `.uasset` files (UI widgets, materials, input mappings) were serialized in 5.7 and are not backwards compatible. The core simulation will still work on older versions but the dashboard UI and some editor features will be missing. If there is demand for older version support, we can look into providing compatible assets.
* **Windows 10/11**
* **C++ Project:** This plugin contains source code and cannot be used in a Blueprints-only project.
* **Visual Studio 2022 / 2025** (with "Game development with C++" workload).
* **Python 3.11+** (optional, for external policy control)

## Installation

1. **Clone the Plugin:** Navigate to your project's `Plugins/` directory and clone the repository:
```bash
cd "YourProject/Plugins"
git clone https://github.com/URLab-Sim/UnrealRoboticsLab.git
```

2. **Build Third-Party Dependencies:** Before opening the engine, you must fetch and build the MuJoCo dependencies. Open **PowerShell** and run:
```powershell
cd UnrealRoboticsLab/third_party
.\build_all.ps1
```
*(If you encounter a compiler stack overflow error here, see the [Troubleshooting](#troubleshooting) section below).*

3. **Compile:** Right-click your `.uproject` file, select **Generate Visual Studio project files**, then open the solution and **Build** your project in your IDE.

4. **Show Assets:** In the Unreal Content Browser, click the **Settings** (gear icon) and check **Show Plugin Content**. This is required to see the UI widgets and plugin assets.

5. **(Optional) C++ Integration:** If you want to use URLab types directly in your own C++ code (e.g., `#include "MuJoCo/Core/AMjManager.h"`, casting to `AMjArticulation*`), add `"URLab"` to your project's `.Build.cs`:
```csharp
PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "URLab" });
```
This is **not required** if you only use the plugin through the editor, Blueprints, or ZMQ.

6. **(Optional) Python Bridge:** The companion [urlab_bridge](https://github.com/URLab-Sim/urlab_bridge) package provides Python middleware for external control, RL policy deployment, and ROS 2 bridging. See its README for setup instructions.

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

* Use the actuator sliders in the MjSimulate widget to move joints.
* Set control source to UI (on manager or per-articulation) to use dashboard sliders instead of ZMQ.

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

See [Hotkeys](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/blueprint_reference.md#hotkeys) for keyboard shortcuts.

## Next Steps

* [features.md](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/features.md) -- complete feature reference
* [MJCF Import](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/mujoco_import.md) -- import pipeline details
* [Blueprint Reference](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/blueprint_reference.md) -- all Blueprint-callable functions and hotkeys
* [ZMQ Networking](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/zmq_networking.md) -- protocol, topics, and Python examples
* [Policy Bridge](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/policy_bridge.md) -- RL policy deployment
* [Developer Tools](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/developer_tools.md) -- schema tracking, debug XML, build/test skills

---

## Troubleshooting

### Build Error: MSVC Stack Overflow (0xC00000FD)
If the `build_all.ps1` script fails with code `-1073741571`, your compiler has run out of internal memory while processing MuJoCo's complex sensor templates. 
* **Fix:** Update Visual Studio to the latest version of **VS 2022 (17.10+)** or **VS 2025** (the MuJoCo CI reference).
* **Workaround:** Force a larger stack size by running:
  `cmake -B build ... -DCMAKE_CXX_FLAGS="/F10000000"`

### UI: "Simulate" Dashboard Not Appearing
The UI is context-sensitive and requires specific conditions:
* Ensure an `MjManager` actor is present in the level.
* In the `MjManager` settings, verify `bAutoCreateSimulateWidget` is enabled.
* Ensure you have followed the **"Show Assets"** step in the Installation guide to make the UI widgets visible to the engine.

### Older UE Versions: Content Assets Won't Load
The bundled `.uasset` files (UI widgets, materials, input mappings) were saved in UE 5.7 and won't load in earlier versions. The C++ plugin code compiles and the core simulation runs, but the dashboard UI and some editor features will be missing.

We strongly recommend upgrading to UE 5.7 as that is the only version we test and support. If that is not possible and you have UE 5.7 installed alongside your older version, you can recreate the assets by copy-pasting their contents:
1. Open the plugin project in **UE 5.7** and open the widget/material/input asset you need.
2. Select all nodes in the editor graph (Ctrl+A) and copy (Ctrl+C).
3. In your **older UE version**, create a new asset of the same type (e.g., a Widget Blueprint parented to `MjSimulateWidget`).
4. Paste (Ctrl+V) — the nodes and hierarchy transfer across versions.
5. Save the new asset. It is now compatible with your engine version.

This works for UMG widget Blueprints, material graphs, and input mapping assets.

### Simulation: Robot is Static
* **Control Source:** Check if the **Control Source** on the `MjManager` or `MjArticulation` is set to **UI**. If set to **ZMQ**, UI sliders will be ignored.
* **Physics State:** Ensure the `MjManager` is not paused and that the robot is not set to `Static` in its component settings.
