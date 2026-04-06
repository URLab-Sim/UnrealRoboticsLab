# Scripting with Blueprints & C++

Every MuJoCo component in Unreal Robotics Lab is a regular Unreal component. Articulations are actors. You interact with them the same way you would with any other actor or component — get references, call functions, bind events. No need to touch MuJoCo directly unless you want to.

For the full class-by-class API listing, see the [auto-generated API Reference](../api/index.md).

---

## Getting References

An `AMjArticulation` is just an actor. Get a reference any way you normally would:

**Blueprint:** Get All Actors of Class, cast from a hit result, or store a reference variable.

**C++:**
```cpp
AAMjManager* Manager = AAMjManager::GetManager();
AMjArticulation* Robot = Manager->GetArticulation("MyRobot");
```

From the manager you can look up articulations by name or get all of them:

**Blueprint:** **Get Manager** → **Get Articulation** (name) / **Get All Articulations**

**C++:**
```cpp
TArray<AMjArticulation*> All = Manager->GetAllArticulations();
```

---

## Working with Components

Once you have an articulation, its MuJoCo components are just child components. Access them by name or as arrays:

**Blueprint:** **Get Actuator** (name), **Get Joint** (name), **Get Sensor** (name) — or **Get Actuators**, **Get Joints**, **Get Sensors**, **Get Bodies** for arrays. You can also drag directly from the component tree in your Blueprint.

**C++:**
```cpp
UMjActuator* Act = Robot->GetActuator("shoulder");
TArray<UMjJoint*> Joints = Robot->GetJoints();
UMjBody* Body = Robot->GetBodyByMjId(3);  // by compiled ID
```

---

## Controlling Actuators

**Blueprint:** **Set Actuator Control** (Name, Value) on the articulation. Wire **Get Game Time in Seconds** → **Sin** → **Set Actuator Control** into Event Tick for a simple sine wave.

Use **Get Actuator Range** to get a `Vector2D` (min, max) for clamping.

**C++:**
```cpp
Robot->SetActuatorControl("shoulder", 1.57f);
FVector2D Range = Robot->GetActuatorRange("shoulder");
```

---

## Reading Sensors

**Blueprint:**

- **Get Sensor Scalar** (name) → Float — for 1D sensors (touch, joint pos, clock)
- **Get Sensor Reading** (name) → Array of Float — for vector sensors (force, accelerometer)
- **Get Joint Angle** (name) → Float — shortcut for joint position

**C++:**
```cpp
float Touch = Robot->GetSensorScalar("fingertip_touch");
TArray<float> Force = Robot->GetSensorReading("wrist_force");
float Angle = Robot->GetJointAngle("elbow");
```

---

## Reacting to Collisions

`AMjArticulation` has an **On Collision** event dispatcher. Fires with: `SelfGeom` (UMjGeom*), `OtherGeom` (UMjGeom*), `ContactPos` (FVector).

**Blueprint:** Select the articulation reference → **Assign On Collision** → wire into your logic.

**C++:**
```cpp
Robot->OnCollision.AddDynamic(this, &AMyActor::HandleCollision);

void AMyActor::HandleCollision(UMjGeom* SelfGeom, UMjGeom* OtherGeom, FVector ContactPos)
{
    // Stop gripper, play effect, etc.
}
```

---

## Simulation Lifecycle

All on the manager:

| Node | What it does |
|------|-------------|
| **Set Paused** (bool) | Pause/resume the physics thread |
| **Reset Simulation** | Zero positions, velocities, time |
| **Step Sync** (N) | Advance N steps synchronously (RL-style loops) |
| **Get Sim Time** | Current simulation clock |

**C++:**
```cpp
Manager->SetPaused(true);
Manager->StepSync(10);
Manager->ResetSimulation();
float Time = Manager->GetSimTime();
```

---

## Snapshots

Save and restore the full simulation state:

**Blueprint:** **Capture Snapshot** → store in a variable → **Restore Snapshot** later.

**C++:**
```cpp
UMjSimulationState* Snap = Manager->CaptureSnapshot();
// ... try something ...
Manager->RestoreSnapshot(Snap);
```

Hold multiple snapshots for A/B testing, checkpointing, or undo.

---

## Keyframe API

On `AMjArticulation`:

| Node | What it does |
|------|-------------|
| `ResetToKeyframe(Name)` | Teleports to a named keyframe (sets qpos/qvel/ctrl) |
| `HoldKeyframe(Name)` | Continuously maintains a keyframe pose |
| `StopHoldKeyframe()` | Releases the held keyframe |
| `GetKeyframeNames()` | Returns names of all keyframes on this articulation |
| `IsHoldingKeyframe()` | Returns true if currently holding a pose |

The MjSimulate widget provides a keyframe dropdown and Reset/Hold/Stop buttons for interactive use.

---

## Recording and Replay

With an `AMjReplayManager` in the level:

**Blueprint:** **Start Recording** / **Stop Recording** / **Start Replay** / **Stop Replay** / **Save Recording to File** / **Load Recording from File**

**C++:**
```cpp
Replay->StartRecording();
// ... simulation runs ...
Replay->StopRecording();
Replay->SaveRecordingToFile("C:/data/experiment.dat");
Replay->StartReplay();
```

---

## Switching Control Source

Toggle between dashboard and external ZMQ control:

**Blueprint:** **Get Manager** → **Set Control Source** → `UI` or `ZMQ`

**C++:**
```cpp
Manager->SetControlSource(EControlSource::ZMQ);
```

Per-articulation override available on `AMjArticulation::ControlSource`.

---

## MjKeyframeController

Cycles through named poses on an articulation with ease-in-out blending.

| Node | Description |
|------|-------------|
| **LoadPreset** (name) | Load a built-in pose sequence |
| **Play** / **Stop** | Start or pause playback |
| **GoToKeyframe** (index) | Jump to a specific keyframe |
| **GetPresetNames** | Returns all available preset names |

For full details on the keyframe controller, presets, and FMjKeyframePose struct, see [Controller Framework](controller_framework.md).

---

## MjKeyframeCameraActor

`AMjKeyframeCameraActor` is a cinematic camera that smoothly interpolates through a list of waypoints (`FMjCameraWaypoint`: Position, Rotation, Time). It uses a `UCineCameraComponent` and displays a spline preview of the path in the editor.

**Key functions:**

| Node | Description |
|------|-------------|
| **Play** | Start waypoint playback |
| **Pause** | Freeze at current position |
| **TogglePlayPause** | Toggle (also bound to **O** key) |
| **Reset** | Return to first waypoint |
| **CaptureCurrentView** | (Editor only) Snapshot the viewport camera as a new waypoint |

**Properties:** `bAutoPlay`, `bAutoActivate` (sets as player view target), `bLoop`, `StartDelay`, `bSmoothInterp` (cubic vs linear).

---

## MjImpulseLauncher

`AMjImpulseLauncher` applies a velocity-based impulse to an `MjBody`. Two modes:

- **Direct** — launches along the actor's forward vector (or `DirectionOverride`).
- **Targeted** — set `LaunchTarget` to an actor and it computes a ballistic arc toward it. `ArcHeight` controls the lob.

| Node | Description |
|------|-------------|
| **FireImpulse** | Apply the impulse once |
| **ResetAndFire** | Teleport projectile back to launcher position and fire (also bound to **F** key) |

**Properties:** `TargetActor`, `TargetBodyName` (optional, defaults to first MjBody), `LaunchSpeed` (m/s), `LaunchTarget`, `ArcHeight`, `bAutoFire`, `AutoFireDelay`.

---

## Hotkeys

Handled by `AAMjManager::Tick`. Active during PIE:

| Key | Action |
|-----|--------|
| **1** | Toggle debug contacts |
| **2** | Toggle visual meshes |
| **3** | Toggle articulation collision wireframes |
| **4** | Toggle debug joints |
| **5** | Toggle quick-convert collision wireframes |
| **P** | Pause / resume simulation |
| **R** | Reset simulation |
| **O** | Toggle orbit camera orbit + keyframe camera play/pause |
| **F** | Reset and fire all impulse launchers |

---

## Per-Articulation Control Source

Each `AMjArticulation` has a `ControlSource` field (`0` = ZMQ, `1` = UI) that overrides the manager-level `EControlSource` setting. This lets you run some robots from the dashboard sliders while others receive external ZMQ commands in the same scene.

**Blueprint:** Set `ControlSource` on the articulation reference in Details or via Set node.

**C++:**
```cpp
Robot->ControlSource = 1; // UI control for this robot
```

---

## Advanced: Direct MuJoCo Access

For users who need low-level access, every `UMjComponent` exposes **Get Mj ID** (compiled integer ID), **Get Mj Name** (prefixed name), and **Is Bound** (compilation status). Use these to index directly into `mjData` from C++. For most workflows, the API above is all you need.
