# Getting Started

## Prerequisites

* **Unreal Engine 5.7+** — The C++ plugin code and third-party libraries should compile on earlier UE5 versions, but the bundled `.uasset` files (UI widgets, materials, input mappings) were serialized in 5.7 and are not backwards compatible. The core simulation will still work on older versions but the dashboard UI and some editor features will be missing. If there is demand for older version support, we can look into providing compatible assets.
* **Windows 10/11**
* **C++ Project:** This plugin contains source code and cannot be used in a Blueprints-only project.
* **Visual Studio 2022 / 2025** (with "Game development with C++" workload).
* **Python 3.11+** (optional, for external policy control)

## Installation

1. **Clone the Plugin with Submodules:** MuJoCo, CoACD, and libzmq sources are tracked as git submodules pinned to the exact commits URLab builds against, so you must clone recursively (or init the submodules afterwards). Navigate to your project's `Plugins/` directory and run:
```bash
cd "YourProject/Plugins"
git clone --recurse-submodules https://github.com/URLab-Sim/UnrealRoboticsLab.git
```
If you already cloned without `--recurse-submodules`, the build scripts will init the submodules for you on first run. You can also do it manually:
```bash
cd UnrealRoboticsLab
git submodule update --init --recursive
```

2. **Build Third-Party Dependencies:** Before opening the engine, build the three vendored deps. Open **PowerShell** and run:
```powershell
cd UnrealRoboticsLab/third_party
.\build_all.ps1
```
This will, for each of MuJoCo / CoACD / libzmq:
- sync the submodule under `third_party/<dep>/src/` to the SHA URLab expects (overwriting local edits there),
- wipe `third_party/install/<dep>/` and rebuild from scratch,
- record the source SHA in `third_party/install/<dep>/INSTALLED_SHA.txt` so URLab's `Build.cs` can detect drift on the next UE build.

*(If you encounter a compiler stack overflow error here, see the [Troubleshooting](#troubleshooting) section below).*

If you are iterating on a dep locally and want to keep your changes in `third_party/<dep>/src/`, pass the opt-out flag (see [Updating & Iterating on Deps](#updating--iterating-on-deps)).

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
2. Drag the XML into the Unreal Content Browser. On first import, the editor will prompt to install the required Python packages (`trimesh`, `numpy`, `scipy`) — by default these are installed to UE's bundled Python, so no external setup is needed. You can also choose your own Python interpreter if preferred.
3. A Blueprint is auto-generated with all joints, bodies, actuators, and sensors as components.
4. See the [Articulation Builder Guide](guides/articulation_builder.md) for editing and building articulations.

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

See the [Debug Visualization guide](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/debug_visualization.md) for the full set of PIE overlays — contact forces, collision wireframes, joint axes, constraint islands, instance/semantic segmentation, and muscle/tendon tubes — plus the hotkeys that drive them.

## Updating & Iterating on Deps

### After `git pull`

URLab bumps its pinned submodule SHAs from time to time (e.g. MuJoCo 3.6 → 3.7). When this happens, a plain `git pull` updates URLab's code and its *pointer* to the submodules, but it does **not** check out new submodule sources or rebuild them. The full update sequence is:

```powershell
git pull
git submodule update --init --recursive --force  # sync src/ to the new pinned SHA
cd third_party; .\build_all.ps1; cd ..           # rebuild any dep whose SHA moved
```

> `--force` is needed because `third_party/CoACD/build.ps1` overlays a custom `CMakeLists.txt` onto the CoACD submodule's working tree on every build, which makes a plain `git submodule update` refuse to check out a new SHA ("Your local changes would be overwritten"). Since those working-tree modifications are always reapplied by the next build, discarding them is safe. If you're iterating on submodule source manually, see [Iterating on a dep locally](#iterating-on-a-dep-locally) below — don't rely on `git submodule update` preserving your edits.

URLab's `Build.cs` cross-checks these on every UE build. If you forget a step, the build fails fast with a message telling you exactly which command to run — see [Troubleshooting](#troubleshooting).

### Iterating on a dep locally

If you are modifying a submodule's source (say, patching CoACD), you don't want every build to snap your edits back to the pinned SHA. Pass the opt-out flag:

```powershell
# Skip all three submodule syncs for this run
.\third_party\build_all.ps1 -NoSubmoduleSync

# Or a single dep
.\third_party\MuJoCo\build.ps1 -NoSubmoduleSync
```

On Linux/macOS the equivalent is `--no-submodule-sync`.

Once you are done iterating, you will still hit the drift checks on the next UE build (because your submodule HEAD no longer matches URLab's pointer, and the install SHA no longer matches your working tree). To bypass the checks entirely — for long-running local work — flip the constant at the top of [Source/URLab/URLab.Build.cs](/URLab-Sim/UnrealRoboticsLab/blob/main/Source/URLab/URLab.Build.cs):

```csharp
private static readonly bool SkipThirdPartyDriftChecks = true;
```

Remember to flip it back before committing; otherwise your teammates lose the safety net.

## Next Steps

* [features.md](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/features.md) -- complete feature reference
* [MJCF Import](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/mujoco_import.md) -- import pipeline details
* [Blueprint Reference](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/blueprint_reference.md) -- all Blueprint-callable functions and hotkeys
* [Debug Visualization](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/debug_visualization.md) -- PIE overlays, segmentation, island colouring, muscle/tendon tubes
* [ZMQ Networking](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/zmq_networking.md) -- protocol, topics, and Python examples
* [Policy Bridge](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/policy_bridge.md) -- RL policy deployment
* [Developer Tools](/URLab-Sim/UnrealRoboticsLab/blob/main/docs/guides/developer_tools.md) -- schema tracking, debug XML, build/test skills

---

## Troubleshooting

### Build Error: `<dep> submodule drift`

If the UE build fails with a message like:

```
MuJoCo submodule drift: URLab expects SHA 72cb2b2... but third_party/MuJoCo/src/ is at aef0589...
From the plugin root run:
  git submodule update --init --recursive --force
then re-run third_party/MuJoCo/build.ps1
```

…you've pulled URLab but the submodule under `third_party/MuJoCo/src/` still points at the old commit. This is URLab's **Layer A** check. Do exactly what the message says:

```powershell
git submodule update --init --recursive --force
cd third_party; .\MuJoCo\build.ps1; cd ..
```

The same applies to CoACD and libzmq — the message names the specific dep.

### Build Error: `<dep> install is stale`

If the UE build fails with a message like:

```
MuJoCo install is stale: built from SHA 0000... but third_party/MuJoCo/src/ is now at 72cb2b2...
Re-run third_party/MuJoCo/build.ps1
```

…you've updated the submodule source but haven't rebuilt the dep, so `third_party/install/<dep>/` is out of sync with the `src/`. This is URLab's **Layer B** check. Fix:

```powershell
cd third_party; .\MuJoCo\build.ps1; cd ..
```

(Both layers can be bypassed by setting `SkipThirdPartyDriftChecks = true` in `URLab.Build.cs` — see [Iterating on a dep locally](#iterating-on-a-dep-locally).)

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
