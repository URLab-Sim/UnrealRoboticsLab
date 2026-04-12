# Architecture

## Overview

UnrealRoboticsLab (URLab) integrates MuJoCo physics into Unreal Engine 5 as an editor plugin. `AAMjManager` is the top-level coordinator actor, but delegates core responsibilities to four `UActorComponent` subsystems: `UMjPhysicsEngine` (simulation), `UMjDebugVisualizer` (debug rendering), `UMjNetworkManager` (ZMQ discovery), and `UMjInputHandler` (hotkeys). The component system mirrors the MJCF element hierarchy -- each XML element type maps to a `UMjComponent` subclass attached to an `AMjArticulation` Blueprint. Physics runs on a dedicated async thread; the game thread reads results for rendering. ZMQ networking provides external control and sensor broadcasting.

---

## Subsystem Architecture

`AAMjManager` delegates responsibilities to four `UActorComponent` subsystems, created via `CreateDefaultSubobject` in the constructor:

```
AAMjManager (thin coordinator)
├── UMjPhysicsEngine       — simulation core
├── UMjDebugVisualizer      — debug rendering
├── UMjNetworkManager       — ZMQ discovery & cameras
└── UMjInputHandler         — hotkey processing
```

### UMjPhysicsEngine

**File:** `Source/URLab/Public/MuJoCo/Core/MjPhysicsEngine.h`

Owns all simulation state: `m_spec`, `m_vfs`, `m_model`, `m_data`, `CallbackMutex`. Handles the full compile pipeline (`PreCompile`/`Compile`/`PostCompile`/`ApplyOptions`). Runs the async physics loop (`RunMujocoAsync`). Provides callback registration (`RegisterPreStepCallback`, `RegisterPostStepCallback`) so other subsystems and ZMQ components can hook into the step loop without direct coupling. Also provides `StepSync`, `Reset`, `Snapshot`/`Restore`.

### UMjDebugVisualizer

**File:** `Source/URLab/Public/MuJoCo/Core/MjDebugVisualizer.h`

Owns `DebugData` (`FMuJoCoDebugData` from `MjDebugTypes.h`) and `DebugMutex`. `CaptureDebugData` is registered as a post-step callback on `UMjPhysicsEngine` — it copies contact data under the mutex on the physics thread. `TickComponent` renders contact force arrows/points on the game thread. Provides toggle methods for each debug visualization mode.

### UMjNetworkManager

**File:** `Source/URLab/Public/MuJoCo/Net/MjNetworkManager.h`

Owns `ZmqComponents` discovery and camera registration/streaming. `DiscoverZmqComponents()` is called during BeginPlay to collect all `UMjZmqComponent` subcomponents on the manager actor.

### UMjInputHandler

**File:** `Source/URLab/Public/MuJoCo/Input/MjInputHandler.h`

Processes hotkeys in `TickComponent`. Dispatches to `UMjPhysicsEngine` (pause/reset), `UMjDebugVisualizer` (debug toggles), and scene actors (mesh visibility, collision wireframes, cameras).

### Communication Pattern

Subsystems communicate through:
- **Callbacks:** `UMjPhysicsEngine` exposes `RegisterPreStepCallback`/`RegisterPostStepCallback`. Other subsystems register lambdas during BeginPlay.
- **`GetOwner()` traversal:** Subsystems access siblings via `GetOwner()->FindComponentByClass<T>()` (e.g., `UMjInputHandler` finds `UMjPhysicsEngine` to call `Reset()`).
- **Direct access:** External code accesses subsystem state directly via `Manager->PhysicsEngine->Options`, `Manager->DebugVisualizer->bShowDebug`, etc. `AAMjManager` has no duplicate properties — it is a pure coordinator. Blueprint-callable convenience methods like `SetPaused()`, `StepSync()`, and `ResetSimulation()` delegate to `PhysicsEngine`.

---

## Module Initialization

**File:** `Source/URLab/Private/URLab.cpp` -- `FURLabModule::StartupModule()`

DLL load order (sequential, each must succeed):

1. `mujoco.dll` (from `third_party/install/MuJoCo/bin/`) — obj/stl decoders are compiled into this DLL since MuJoCo 3.7.0; the standalone `obj_decoder.dll` / `stl_decoder.dll` shipped in earlier versions should not be loaded (plugin-registration collision).
2. `libzmq-v143-mt-4_3_6.dll` (from `third_party/install/libzmq/bin/`)
3. `lib_coacd.dll` (from `third_party/install/CoACD/bin/`)

Search path strategy: plugin `third_party/install/{SubDir}/bin/` first (editor/development builds), then `FPlatformProcess::GetModulesDirectory()` (packaged builds where DLLs are staged next to the executable).

In editor builds, the module also registers a "MuJoCo" asset category via `IAssetTools::RegisterAdvancedAssetCategory()`.

---

## Scene Lifecycle

### BeginPlay

**File:** `Source/URLab/Private/MuJoCo/Core/AMjManager.cpp` -- `AAMjManager::BeginPlay()`

1. **Singleton enforcement.** If `AAMjManager::Instance` is already set to a different actor, this instance logs an error and returns. Only one `AAMjManager` per level is supported.
2. **Subsystem creation.** The four subsystems (`UMjPhysicsEngine`, `UMjDebugVisualizer`, `UMjNetworkManager`, `UMjInputHandler`) are created via `CreateDefaultSubobject` in the constructor.
3. **Auto-create ZMQ components.** If no `UMjZmqComponent` subclasses exist on the manager actor, it creates a `UZmqSensorBroadcaster` (tcp://*:5555) and `UZmqControlSubscriber` (tcp://127.0.0.1:5556).
4. **ZMQ discovery.** `UMjNetworkManager::DiscoverZmqComponents()` collects all ZMQ components on the manager actor.
5. **Auto-create replay manager.** If no `AMjReplayManager` exists in the level, one is spawned.
6. **`Compile()`** -- delegates to `UMjPhysicsEngine` for spec construction and `mj_compile()` (see below).
7. **Callback registration.** Registers `UMjDebugVisualizer::CaptureDebugData` as a post-step callback on `UMjPhysicsEngine`.
8. **`UpdateCameraStreamingState()`** -- applies global camera broadcast toggle.
9. **`RunMujocoAsync()`** -- delegates to `UMjPhysicsEngine` to launch the physics thread.
10. **Auto-create MjSimulate widget.** Loads `/UnrealRoboticsLab/WBP_MjSimulate` Blueprint and adds it to the viewport (controlled by `bAutoCreateSimulateWidget`).

### Spec Construction (PreCompile)

**File:** `UMjPhysicsEngine::PreCompile()`

1. Creates a fresh `mjSpec` via `mj_makeSpec()` (radians mode: `compiler.degree = false`).
2. Initializes the VFS via `mj_defaultVFS()`.
3. Discovers all actors in the level via `UGameplayStatics::GetAllActorsOfClass()`.
4. Processing order within the discovery loop:
   - **Quick Convert:** Any actor with `UMjQuickConvertComponent` calls `Setup(spec, vfs)`.
   - **Articulations:** Any `AMjArticulation` calls `Setup(spec, vfs)`.
   - **Heightfields:** Any `AMjHeightfieldActor` calls `Setup(spec, vfs)`.
5. ZMQ component discovery is handled separately by `UMjNetworkManager::DiscoverZmqComponents()` during BeginPlay.

Note: The discovery loop iterates all actors once. Quick Convert components and Articulations on the same actor are both processed (they are not mutually exclusive in the loop logic, though in practice they exist on separate actors).

### Articulation Setup

**File:** `Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp` -- `AMjArticulation::Setup()`

This is the most complex part of spec construction. Each articulation creates an isolated child spec that is later merged into the root.

1. **Create child spec.** `mj_makeSpec()` for isolation. Radians mode. Articulation-level `SimOptions` applied via `ApplyToSpec()`.
2. **Create wrapper.** `FMujocoSpecWrapper` wraps the child spec and VFS for convenient element creation.
3. **Prefix.** Set to `{ActorName}_` -- all elements in this articulation will be prefixed after `mjs_attach()`.
4. **Processing order is critical:**
   1. **Defaults** (must be first). `GetComponents<UMjDefault>()` then `m_Wrapper->AddDefault()` for each. Other elements reference these default classes.
   2. **WorldBody traversal.** Finds the `UMjWorldBody` component, iterates its `GetAttachChildren()`. For each child:
      - `UMjBody` (non-default) -> `Setup(nullptr, nullptr, wrapper)` (recursive)
      - `UMjFrame` (non-default) -> `Setup(nullptr, nullptr, wrapper)`
   3. **Tendons.** `GetComponents<UMjTendon>()` then `RegisterToSpec()`. Must come after bodies so joint names exist.
   4. **Sensors.** `GetComponents<UMjSensor>()` then `RegisterToSpec()`.
   5. **Actuators.** `GetComponents<UMjActuator>()` then `RegisterToSpec()`.
   6. **Contact pairs.** `GetComponents<UMjContactPair>()` then `RegisterToSpec()`.
   7. **Contact excludes.** `GetComponents<UMjContactExclude>()` then `RegisterToSpec()`.
   8. **Equalities.** `GetComponents<UMjEquality>()` then `RegisterToSpec()`.
   9. **Keyframes.** `GetComponents<UMjKeyframe>()` then `RegisterToSpec()`.
5. **Attach.** `mjs_attach(attachmentFrame->element, childSpec->element, prefix, "")` merges the child spec into the root world body via an `mjsFrame`. The frame's pos/quat are set from the articulation actor's world transform (converted to MuJoCo coordinates). After attach, `m_ChildSpec` is set to nullptr (ownership transferred to root spec).

### Body Traversal (Recursive)

**File:** `Source/URLab/Private/MuJoCo/Components/Bodies/MjBody.cpp` -- `UMjBody::Setup()`

`UMjBody::Setup()` creates an `mjsBody` via the wrapper, then iterates `GetAttachChildren()`:

- Child `UMjBody` -> recursive `Setup()`
- Child `UMjFrame` -> `Frame::Setup()`
- Child `UMjGeom`, `UMjJoint`, `UMjSensor`, `UMjSite`, etc. -> `RegisterToSpec()`

Mesh preparation (CoACD convex decomposition via `PrepareMeshForMuJoCo`) happens during geom registration.

### Compilation

**File:** `UMjPhysicsEngine::Compile()`

1. Calls `PreCompile()`.
2. `mj_compile(m_spec, &m_vfs)` produces `mjModel*`.
3. **On failure:** retrieves error via `mjs_getError(m_spec)`, logs it, shows editor dialog (`FMessageDialog`). Returns without creating data.
4. **On success:**
   - If `bSaveDebugXml` is true: saves `scene_compiled.xml` and `scene_compiled.mjb` to `Saved/URLab/`, with file paths relativized.
   - Creates `mjData` via `mj_makeData(m_model)`.
   - Calls `ApplyOptions()` (applies manager-level option overrides to `m_model->opt`).
   - Calls `PostCompile()`.

### PostCompile (Binding)

**File:** `UMjPhysicsEngine::PostCompile()`

1. Calls `PostSetup(model, data)` on each `UMjQuickConvertComponent`.
2. Builds `m_ArticulationMap` (name -> `AMjArticulation*`) for O(1) lookup.
3. Calls `PostSetup(model, data)` on each `AMjArticulation`.
4. Calls `PostSetup(model, data)` on each `AMjHeightfieldActor`.

**Inside `AMjArticulation::PostSetup()`:**

Each component calls `Bind(model, data, prefix)`. The `Bind()` method on `UMjComponent` calls `BindToView<T>()` which:

1. **Path 1 (ID-based):** If `m_SpecElement` is set, tries `mjs_getId(m_SpecElement)`. Validates the ID is within model bounds (guards against stale IDs after `mjs_attach`).
2. **Path 2 (Name fallback):** Constructs `{Prefix}{MjName}` (or `{Prefix}{GetName()}` if MjName is empty), calls `mj_name2id()`.

This creates View structs (`BodyView`, `GeomView`, `JointView`, `ActuatorView`, `SensorView`, `TendonView`, `SiteView`) that cache raw pointers into `mjModel`/`mjData` arrays.

PostSetup also populates component-name maps (`ActuatorComponentMap`, `JointComponentMap`, etc.) and MuJoCo-ID maps (`BodyIdMap`, `GeomIdMap`, etc.) for O(1) runtime access.

---

## Physics Loop (Async Thread)

**File:** `Source/URLab/Private/MuJoCo/Core/MjPhysicsEngine.cpp` -- `UMjPhysicsEngine::RunMujocoAsync()`

Runs on a dedicated thread via `Async(EAsyncExecution::Thread, ...)`. Stored in `AsyncPhysicsFuture`.

Each iteration (under `CallbackMutex` lock, owned by `UMjPhysicsEngine`):

1. **Check `bPendingReset`** -> `mj_resetData()` + `mj_forward()`. Broadcasts `OnSimulationReset` on game thread.
2. **Check `bPendingRestore`** -> `mj_setState()` with `PendingStateVector` + `mj_forward()`.
3. **Registered pre-step callbacks** (replaces direct ZMQ PreStep calls). ZMQ components register via `RegisterPreStepCallback()`.
4. **ApplyControls** on each articulation (writes `d->ctrl` from actuator values).
5. **Physics step:**
   - If `bIsPaused`: skip.
   - If `CustomStepHandler` is bound: call it instead of `mj_step()` (used by replay playback).
   - Otherwise: `mj_step(m_model, m_data)`.
6. **Registered post-step callbacks** (replaces direct ZMQ PostStep and debug capture calls). ZMQ components and `UMjDebugVisualizer::CaptureDebugData` register via `RegisterPostStepCallback()`.
7. **`OnPostStep` delegate** (replay recording captures state here).

**Timing:** After releasing the mutex, the loop spin-waits (`FPlatformProcess::YieldThread()`) until `TargetInterval / SpeedFactor` has elapsed. `SimSpeedPercent` (5-100) controls the speed factor.

---

## Game Thread (Tick)

**File:** `AAMjManager::Tick()`

1. **Backward-compat pointer sync.** Copies `m_model`/`m_data` from `UMjPhysicsEngine` so legacy callers that read `AAMjManager::m_model` still work.
2. Hotkey processing and debug drawing are delegated to `UMjInputHandler` and `UMjDebugVisualizer` respectively (both tick via their own `TickComponent`).

### UMjInputHandler::TickComponent

Processes hotkeys each frame and dispatches to the appropriate subsystem:

- `1` -> `DebugVisualizer->ToggleContactForces()`
- `2` -> Toggle visual mesh visibility (scene actors)
- `3` -> Toggle articulation collision wireframes (scene actors)
- `4` -> Toggle joint debug axes (scene actors)
- `5` -> Toggle Quick Convert collision wireframes (scene actors)
- `P` -> `PhysicsEngine->TogglePause()`
- `R` -> `PhysicsEngine->Reset()`
- `O` -> Toggle orbit and keyframe cameras (scene actors)
- `F` -> Fire impulse launchers (scene actors)

### UMjDebugVisualizer::TickComponent

If debug is enabled, reads `DebugData` (protected by `DebugMutex` on the visualizer), draws contact force arrows and points via `ULineBatchComponent` / `DrawDebugLine`.

**Transform sync** happens in `UMjBody::TickComponent()`:

- If `bDrivenByUnreal`: writes UE world transform to MuJoCo mocap body (`d->mocap_pos`, `d->mocap_quat`).
- Otherwise: reads MuJoCo `xpos`/`xquat` from `BodyView` and sets UE world transform via `SetWorldLocationAndRotation()`.

---

## Thread Safety

| Mutex / Mechanism | Owner | Protects | Used By |
|---|---|---|---|
| `CallbackMutex` (`FCriticalSection`) | `UMjPhysicsEngine` | `m_model`, `m_data` during physics stepping | Physics thread (main lock), `StepSync()` |
| `DebugMutex` (`FCriticalSection`) | `UMjDebugVisualizer` | `DebugData` (contact visualization buffer) | Physics thread writes (via post-step callback), game thread reads (TickComponent) |
| `CameraMutex` (`FCriticalSection`) | `UMjNetworkManager` | `ActiveCameras` array | Register/Unregister from any thread |
| `bPendingReset` (`std::atomic<bool>`) | `UMjPhysicsEngine` | Reset signal | Game thread sets, physics thread reads/clears |
| `bPendingRestore` (`std::atomic<bool>`) | `UMjPhysicsEngine` | Restore signal | Game thread sets, physics thread reads/clears |
| `bIsPaused` (non-atomic, but written only from game thread) | `UMjPhysicsEngine` | Pause state | Read by physics thread |
| `bShouldStopTask` (`std::atomic<bool>`) | `UMjPhysicsEngine` | Shutdown signal | Game thread sets in `EndPlay`, physics thread checks |
| `UMjActuator::InternalValue` / `NetworkValue` | `UMjActuator` | Per-actuator control values | Atomic reads/writes across threads |

---

## Component System

### Base: UMjComponent

**File:** `Source/URLab/Public/MuJoCo/Components/MjComponent.h`

Inherits `USceneComponent` + `IMjSpecElement`. All MuJoCo element components derive from this.

Key members:

- `m_SpecElement` (`mjsElement*`) -- set during `RegisterToSpec()`, used for ID resolution in `Bind()`.
- `m_ID` (`int`) -- MuJoCo object ID, resolved during `Bind()`.
- `m_Model`, `m_Data` -- cached pointers, set during `Bind()`.
- `MjName` (`FString`) -- original MJCF name, stable for cross-referencing.
- `bIsDefault` (`bool`) -- if true, this component is a default template, skipped by runtime discovery.

Key methods:

- `RegisterToSpec(FMujocoSpecWrapper&, mjsBody*)` -- creates the spec element. Subclasses override.
- `Bind(mjModel*, mjData*, Prefix)` -- resolves ID and caches model/data pointers.
- `BindToView<T>(Prefix)` -- template method that creates a View struct. Tries `mjs_getId()` first (with bounds validation), falls back to `mj_name2id()`.
- `ResolveDefault(mjSpec*, ClassName)` -- static helper to find `mjsDefault` by class name. Falls back to global default.
- `FindEditorDefault()` -- editor-time default resolution without requiring an mjSpec.
- `SetSpecElementName()` -- assigns unique name to spec element via wrapper deduplication.

### View Structs

**File:** `Source/URLab/Public/MuJoCo/Utils/MjBind.h`

Lightweight structs that cache raw pointers into `mjModel`/`mjData` arrays for zero-overhead runtime access. Each has a `static constexpr mjtObj obj_type` for template dispatch.

| Struct | obj_type | Key Pointers |
|---|---|---|
| `BodyView` | `mjOBJ_BODY` | `xpos`, `xquat`, `xfrc_applied`, `mass`, `mocap_id` |
| `GeomView` | `mjOBJ_GEOM` | `xpos`, `xmat`, `size`, `type`, `contype`, `conaffinity`, `dataid` |
| `JointView` | `mjOBJ_JOINT` | `qpos`, `qvel`, `qacc`, `xanchor`, `xaxis`, `range`, `stiffness`, `stiffnesspoly`, `damping`, `dampingpoly` |
| `ActuatorView` | `mjOBJ_ACTUATOR` | `ctrl`, `force`, `length`, `ctrlrange`, `gainprm`, `biasprm` |
| `SensorView` | `mjOBJ_SENSOR` | `data` (pointer into `sensordata`), `dim`, `adr`, `type` |
| `TendonView` | `mjOBJ_TENDON` | `length`, `velocity`, `stiffness`, `stiffnesspoly`, `damping`, `dampingpoly`, `range` |
| `SiteView` | `mjOBJ_SITE` | `xpos`, `xmat`, `size`, `type`, `body_id` |

`BodyView` also provides traversal methods: `Bodies()`, `Geoms()`, `Joints()` for walking the compiled model hierarchy.

A standalone template function `bind<T>(model, data, name)` provides name-based lookup.

### Default System

`UMjDefault` components store template properties mirroring the MJCF `<default>` hierarchy. Parent-child class chain is built via `FMujocoSpecWrapper::AddDefault()`. During articulation setup, `ExportTo(mjsDefault*)` iterates child components and writes their properties to the spec default. At registration time, components resolve their default class via `ResolveDefault()` which calls `mjs_findDefault()`.

---

## Coordinate System

MuJoCo uses right-handed Z-up coordinates in meters. Unreal uses left-handed Z-up in centimeters.

**Position conversion** (`MjUtils::MjToUEPosition` / `UEToMjPosition`):
- X -> X, Y -> -Y, Z -> Z
- Scale: MuJoCo meters * 100 = Unreal centimeters

**Rotation conversion** (`MjUtils::MjToUERotation` / `UEToMjRotation`):
- MuJoCo quaternion `[w, x, y, z]` -> `FQuat` with X and Z components negated (to handle handedness flip).

All conversion utilities are in `Source/URLab/Public/MuJoCo/Utils/MjUtils.h`.

---

## Imported vs User-Built Articulations

There are two ways to create an `AMjArticulation`:

**1. Imported from MJCF XML (drag-and-drop):**
- Uses `UMujocoImportFactory` -> `UMujocoGenerationAction`
- All components auto-generated from XML parsing
- Default class hierarchy preserved as `UMjDefault` components
- `MuJoCoXMLFile` property stores the source XML path
- Mesh assets imported into Unreal Content Browser

**2. User-built from scratch (right-click -> New MuJoCo Articulation):**
- Uses `UMjArticulationFactory` -> `UMujocoGenerationAction::SetupEmptyArticulation()`
- Creates organizational hierarchy (Definitions, Defaults, Actuators, Sensors, etc.)
- User manually adds components in the Blueprint editor
- No XML file — components are authored directly

**Both paths produce the same result:** an `AMjArticulation` Blueprint with `UMjComponent` subclasses. At runtime, both go through the same Setup -> Compile -> Bind pipeline. The spec system doesn't know or care how the components were created.

### Special-Case Flags

| Flag | On | Purpose |
|------|------|---------|
| `bIsDefault` | `UMjComponent` | Marks template components inside `<default>` blocks. Excluded from runtime discovery (`GetRuntimeComponents`), excluded from `RegisterToSpec` during body traversal. |
| `bIsQuickConverted` | `UMjBody` | Set by Quick Convert. Enables mesh pivot offset correction during tick sync (UE meshes may have off-center pivots that MuJoCo doesn't know about). |
| `bDrivenByUnreal` | `UMjBody` | Enables one-way coupling: UE transform drives MuJoCo mocap body. Used for kinematic objects where Unreal is authoritative. |
| `bFromToResolvedHalfLength` | `UMjGeom` | Set during import when a `fromto` attribute was resolved to pos/quat/size. Controls how `ExportTo` writes the size — only the half-length slot is written, leaving radius to come from the default chain. |

---

## Import Pipeline

**Files:** `Source/URLabEditor/Private/MujocoImportFactory.cpp`, `Source/URLabEditor/Private/MjArticulationFactory.cpp`

### Steps

1. User drags an `.xml` file into the Content Browser.
2. `UMujocoImportFactory::FactoryCreateFile()` triggers.
3. **Mesh preprocessing:** Auto-runs `Scripts/clean_meshes.py` (Python subprocess).
   - Detects Python installation, auto-installs `trimesh` if missing.
   - Parses the MJCF XML to find all mesh assets.
   - Detects GLB stem conflicts (e.g., `link1.obj` and `link1.stl` both producing `link1.glb`).
   - Renames conflicting files with counter suffix, updates XML.
   - Converts all meshes to GLB (preserving UVs, stripping embedded textures).
   - Produces `_ue.xml` with updated mesh references.
   - Graceful fallback at every step — if Python/trimesh missing, uses raw XML.
4. Creates an `AMjArticulation` Blueprint.
5. `UMujocoGenerationAction::GenerateForBlueprint()` parses the XML in four passes:
   - **Pass 1:** Parse assets (meshes with scales, textures with file paths, materials with RGBA/texture refs).
   - **Pass 2:** Parse defaults (class hierarchy — creates `UMjDefault` components with child geom/joint/actuator templates).
   - **Pass 3:** Import worldbody recursively (bodies, geoms, joints, sites become `UMjComponent` subclasses attached in the Blueprint hierarchy).
   - **Pass 4:** Import actuators, sensors, tendons, equalities, keyframes, contact pairs/excludes.
6. Mesh import uses Unreal's asset tools with format priority: FBX > GLB > OBJ. Meshes saved to `/Meshes/` subfolder to avoid texture name collisions.
7. Materials created as shared instances keyed by XML material name (`MI_white` shared by all geoms referencing `"white"`).
8. Textures imported to asset folder, applied to material instances with `bUseTexture` flag.
9. Final Blueprint compile via `FKismetEditorUtilities`.

### Import Normalization

The importer normalizes several MJCF constructs for consistency:

**FromTo resolution:** All `fromto` attributes on geoms and sites are decomposed at import time into explicit `pos`, `quat`, and `size`:
- `pos` = midpoint of the fromto segment
- `quat` = rotation aligning local +Z with the fromto direction
- `size[1]` = half-length (for capsule/cylinder), `size[2]` for box/ellipsoid
- `size[0]` (radius) left untouched — inherited from defaults
- `bFromToResolvedHalfLength` flag set so ExportTo knows to only write the half-length slot
- The component's relative transform is set for accurate editor visualization

**Default class visual transforms:** When a geom inherits a transform from a default class (e.g., `class="visual_zflip"` with `quat="0 0 0 1"`), the importer walks the default hierarchy to find the transform and applies it as a visual offset to the `UStaticMeshComponent` child — NOT to the `UMjGeom` component itself (which would double-apply via the spec default system).

**Orientation handling:** All MJCF rotations (euler, axisangle, quat, xyaxes, zaxis) are converted to UE quaternions at import time using `MjOrientationUtils`. The `<compiler>` settings (angle="radian"/"degree", eulerseq) are respected.

**Mesh name deduplication:** `MjSpecWrapper::GetUniqueName()` appends `_1`, `_2`, etc. if a mesh/body/geom name already exists in the spec.

---

## Export Pipeline (Runtime)

Each component's `RegisterToSpec()` writes to an `mjsElement` via the `FMujocoSpecWrapper`. The component's `ExportTo()` method populates properties (type, size, pos, quat, friction, etc.).

### Override Flag System

Every exportable property has a corresponding `bOverride_*` bool. During `ExportTo()`:
- If `bOverride_X == true`: the property is explicitly written to the spec element.
- If `bOverride_X == false`: the property is NOT written, allowing MuJoCo's default system to provide the value.

This is critical for round-trip fidelity: an imported geom with `class="collision"` that inherits `group=3` from its default should NOT have `group=3` written explicitly — that would bypass the default if the user later changes the default.

### FromTo Export

Geoms with `bFromToResolvedHalfLength == true`:
- If `bIsDefault`: writes raw fromto to the default's geom (MuJoCo handles size resolution for defaults).
- If instance with `bOverride_Size == false`: writes only the half-length slot, letting the default provide the radius.
- If instance with `bOverride_Size == true`: writes all size slots explicitly.

### Spec Attachment

`mjs_attach()` merges the child spec into the root with the articulation's prefix, enabling multi-articulation scenes with unique namespaces. After attachment, all element names gain the prefix (e.g., `pelvis` becomes `g1_C_1_pelvis`). Runtime name lookups use this prefix.

---

## ZMQ Networking

**Files:** `Source/URLab/Private/MuJoCo/Net/ZmqSensorBroadcaster.cpp`, `ZmqControlSubscriber.cpp`

Binary pub/sub protocol for minimal latency.

**Topics:**
- `{prefix}/joint/{name}` -- joint state
- `{prefix}/sensor/{name}` -- sensor reading
- `{prefix}/base_state/` -- root body pose
- `{prefix}/twist` -- twist command velocity
- `{prefix}/control` -- actuator control values

**Timing:**
- Sensor broadcast: `PostStep()` on the physics thread.
- Control receive: `PreStep()` on the physics thread.

**Control source:** `EControlSource` enum (`ZMQ` or `UI`). Set globally on `AAMjManager::ControlSource`, with per-articulation override via `AMjArticulation::ControlSource`.

**Info broadcast:** JSON payload with actuator names, IDs, and ranges. Sent periodically for client discovery.

---

## Quick Convert

**File:** `Source/URLab/Public/MuJoCo/Components/QuickConvert/MjQuickConvertComponent.h`

`UMjQuickConvertComponent` attaches to any Static Mesh actor for one-click physics integration.

- At `Setup()` time, creates an `MjBody` + `MjGeom` on the root world body.
- Runs CoACD convex decomposition with configurable threshold.
- Separates visual mesh (group 2) from collision mesh (group 3).
- Supports static, dynamic, or mocap body modes.

---

## Heightfield

**File:** `Source/URLab/Public/MuJoCo/Components/QuickConvert/AMjHeightfieldActor.h`

`AMjHeightfieldActor` samples Unreal landscape geometry via raycasts.

- Configurable resolution, trace channel, and actor whitelist.
- `bTraceComplex` for accurate mesh sampling.
- Registers as `mjsHField` + static geom on the world body.
- Binary cache (`Saved/URLab/`) for fast recompilation.

---

## Replay System

**File:** `Source/URLab/Public/Replay/MjReplayManager.h`, `MjReplayTypes.h`

- **Recording:** `OnPostStep` callback captures `qpos`/`qvel` per joint per frame.
- **Playback:** `CustomStepHandler` replaces `mj_step()`, applies recorded `qpos` then calls `mj_forward()`.
- **CSV import:** Supports external trajectory data with automatic joint name matching.
- **Snapshot/Restore:** Full state capture via `mj_getState()` / `mj_setState()`. Game thread writes `PendingStateVector` then sets `bPendingRestore`; physics thread applies on next iteration.
- **Thread safety:** Playback time is `std::atomic`. The `CustomStepHandler` runs inside `CallbackMutex`.

---

## Controller System

**Files:** `Source/URLab/Public/MuJoCo/Components/Controllers/`

- `UMjArticulationController` -- abstract base class. Override `ComputeAndApply()` to implement control logic.
- `UMjPDController` -- PD servo controller with per-actuator Kp/Kv/torque limits.
- `UMjPassthroughController` -- passes control values directly to `d->ctrl`.
- `UMjKeyframeController` -- cycles through preset keyframe sequences with smooth interpolation.

**Discovery:** During `PostSetup()`, the articulation calls `FindComponentByClass<UMjArticulationController>()`.

**Bind:** Maps actuator IDs to `qpos`/`qvel` addresses via `FActuatorBinding` structs for efficient access.

**ApplyControls:** `AMjArticulation::ApplyControls()` delegates to the controller's `ComputeAndApply()` if one is bound; otherwise falls back to writing actuator values directly to `d->ctrl`.

---

## Input System

**Files:** `Source/URLab/Public/MuJoCo/Input/MjInputHandler.h`, `Source/URLab/Public/MuJoCo/Input/MjTwistController.h`, `Source/URLab/Public/MuJoCo/Net/MjInputMapping.h`

### MjInputHandler

`UMjInputHandler` is an `UActorComponent` subsystem on `AAMjManager`. It processes debug/simulation hotkeys in `TickComponent` and dispatches to the appropriate subsystem or scene actors. See "Game Thread (Tick)" above for the full key mapping. It accesses sibling subsystems via `GetOwner()->FindComponentByClass<T>()`.

### MjTwistController

Added automatically to articulations on Possess. Captures WASD/gamepad input and stores twist commands (`vx`, `vy`, `yaw_rate`) for broadcasting over ZMQ.

- Uses Unreal's Enhanced Input System (`UInputAction`, `UInputMappingContext`)
- Input context added in `PossessedBy()`, removed in `UnPossessed()`
- `GetTwist()` returns the current twist vector (thread-safe, read by ZMQ broadcaster on physics thread)
- Supports 10 action keys (mapped to a bitmask) for discrete commands
- `MaxVx`, `MaxVy`, `MaxYawRate` properties clamp the output

### MjInputMapping

Configurable actuator-to-input binding system. Maps Enhanced Input actions directly to actuator SetControl calls. Each `FMjInputBinding` specifies:
- Target actuator component
- Input action reference
- Scale multiplier
- The binding fires `SetControl(value * scale)` on the Triggered event

---

## Simulation Options

**File:** `Source/URLab/Public/MuJoCo/Core/MjSimOptions.h`

`FMuJoCoOptions` wraps MuJoCo's `mjOption` fields as UPROPERTYs (timestep, gravity, integrator, solver, iterations, etc.). Used in **two contexts with different semantics**:

### On AMjArticulation (child spec options)
- ALL fields are written to the child spec's `mjOption` before `mjs_attach()`
- The `bOverride_*` toggles are **ignored** — every value applies unconditionally
- This defines the "native" physics settings for that robot model

### On AAMjManager (post-compile overrides)
- Acts as **post-compile overrides** on `m_model->opt` via `ApplyOptions()`
- Only fields with `bOverride_* = true` are applied
- Fields with `bOverride_* = false` keep whatever the compiled model has (from the articulation specs or MJCF defaults)
- Called once after `mj_compile()` succeeds

### Resolution order
1. MuJoCo's built-in defaults (from `<option>` in XML or MuJoCo hardcoded defaults)
2. Articulation's `SimOptions` overwrites the child spec options during Setup
3. After compilation, Manager's `Options` selectively overwrites `m_model->opt` fields

This means: an articulation can set `timestep = 0.002` in its SimOptions, and the manager can override it to `timestep = 0.005` if `bOverride_Timestep = true` on the manager.

---

## Keyframes

**File:** `Source/URLab/Public/MuJoCo/Components/Keyframes/MjKeyframe.h`

`UMjKeyframe` components store named simulation state snapshots from the MJCF `<keyframe>` section:
- `Time` -- simulation time
- `Qpos` -- joint positions (array of doubles, size = nq)
- `Qvel` -- joint velocities (array of doubles, size = nv)
- `Act` -- actuator activations
- `Ctrl` -- control values
- `MocapPos` / `MocapQuat` -- mocap body transforms

Registered to the spec during Setup (after bodies/joints). At runtime, `AMjArticulation::ResetToKeyframe()` teleports the robot to a named keyframe, and `HoldKeyframe()` continuously maintains the pose (via ctrl or direct qpos injection). The MjSimulate widget exposes these via a keyframe dropdown and Reset/Hold/Stop buttons.

---

## Editor Tools

- **MjSimulate widget** (`WBP_MjSimulate`): Physics options, per-actuator sliders, debug visualization toggles, replay controls, possess button. Auto-created at BeginPlay.
- **ValidateSpec:** Blueprint compile hook on `AMjArticulation`. Creates a temporary spec, runs `mj_compile()`, and reports errors without affecting the running simulation.
- **MjComponentDetailCustomizations** (`Source/URLabEditor/`): Detail customizations for all MuJoCo component types — hides internal properties (DefaultClass pointers, synced names) and adds CoACD decomposition buttons for mesh geoms. Component-reference dropdowns (Target, Default Class, etc.) use `meta=(GetOptions)` on UPROPERTY declarations.
- **MjEditorStyle** (`Source/URLabEditor/`): Custom Slate icons for all MuJoCo component types in the Blueprint component tree. Icons are loaded from `Resources/Icons/` PNGs.
- **MuJoCo Outliner** (`SMjArticulationOutliner`): Dockable editor tab (Window menu) that shows a filtered, searchable tree of an articulation Blueprint's component hierarchy. Auto-detects open articulation BPs, provides type filter toggles and summary counts.

### Editor Module Hooks (`FURLabEditorModule`)

The editor module (`Source/URLabEditor/`) registers several hooks that improve the Blueprint editing workflow:

| Hook | Trigger | What It Does |
|------|---------|--------------|
| `OnObjectModified` (SCS) | Any SCS modification | **Auto-parenting:** defers to next tick, then scans all nodes and moves sensors/actuators/defaults/tendons/contacts/equalities under their organizational root folders. |
| `OnObjectModified` (USCS_Node) | Variable name rename | **Default ClassName sync:** keeps `UMjDefault::ClassName` in sync with the SCS variable name for user-created defaults. |
| `OnBlueprintPreCompile` | Blueprint Compile button | **FixupDefaultFlags:** walks everything under `DefaultsRoot` and marks all `UMjComponent` descendants as `bIsDefault = true`. Ensures user-added defaults are properly flagged before the spec is built. |

---

## Editor Workflow

### Blueprint Compile Validation

When a user compiles an `AMjArticulation` Blueprint in the editor, `ValidateSpec()` runs automatically (hooked via `OnBlueprintCompiled`). This creates a temporary mjSpec, runs the full Setup() pipeline on it, and attempts `mj_compile()`. If compilation fails, a dialog shows the MuJoCo error message. This catches spec errors (missing joints, invalid ranges, etc.) at edit time rather than at Play.

### MjSimulate Widget

Auto-created from `WBP_MjSimulate` Blueprint asset at BeginPlay. Provides:
- Physics options: timestep, solver, integrator, iterations, sim speed
- Per-actuator sliders (locked when control source is ZMQ)
- Joint/sensor readouts
- Articulation selector dropdown
- Debug toggles (per-articulation and global)
- Record/Replay/Snapshot controls
- Possess button (attaches spring arm camera)

### Detail Panel Customizations

All MuJoCo component types have registered `IDetailCustomization` classes (in `MjComponentDetailCustomizations.h/.cpp`) that hide internal properties like `DefaultClass` pointers and auto-synced string fields. Component-reference properties (e.g. `TargetName`, `MjClassName`, `Geom1`) use UE's native `meta=(GetOptions="FunctionName")` UPROPERTY specifier to render as dropdown combo boxes — the option lists are populated by `UMjComponent::GetSiblingComponentOptions()`, which scans the Blueprint's SCS tree. Mesh geom components additionally show "Decompose Mesh" and "Remove Decomposition" buttons for CoACD operations.

---

## Debug Visualization

**File:** `Source/URLab/Public/MuJoCo/Core/MjDebugVisualizer.h`

`UMjDebugVisualizer` is an `UActorComponent` subsystem on `AAMjManager`. It owns `DebugData` (`FMuJoCoDebugData`, defined in `MjDebugTypes.h`) and `DebugMutex`. Data capture runs on the physics thread via a registered post-step callback on `UMjPhysicsEngine`. Rendering runs on the game thread in `TickComponent`.

Hotkeys (processed by `UMjInputHandler::TickComponent()`, dispatched to `UMjDebugVisualizer`):

| Key | Toggle |
|---|---|
| `1` | Contact force arrows |
| `2` | Visual mesh visibility |
| `3` | Articulation collision wireframes |
| `4` | Joint axis/range arcs |
| `5` | Quick Convert collision wireframes |

Collision drawing logic: renders geoms in group 3 OR geoms where `contype != 0 && conaffinity != 0`.

Convex hull rendering uses `mesh_graph` data from MuJoCo (hull edges, not full mesh triangles).

Both per-articulation (`bDrawDebugCollision`, `bDrawDebugJoints`, `bDrawDebugSites`) and global toggles (`bGlobalDrawDebugCollision`, etc.) are supported.

---

## Cinematic Tools

**Files:** `Source/URLab/Public/Cinematics/`

- `AMjOrbitCameraActor` -- auto-orbiting camera that tracks detected robots. Toggled via `O` key.
- `AMjKeyframeCameraActor` -- waypoint-based camera path with smooth interpolation. Play/pause toggled via `O` key.

---

## Error Handling and Failure Modes

### Compile failure
If `mj_compile()` returns null, `mjs_getError(m_spec)` retrieves the error string. In editor builds, a dialog box shows the message. The error is also stored in `m_LastCompileError` (accessible via Blueprint). The simulation does not start — `m_model` and `m_data` remain null, and `RunMujocoAsync` checks for this before stepping.

### Mesh import failure
`ImportSingleMesh()` tries multiple paths: GLB (via Interchange), then raw OBJ/STL (via FBX factory), then FBX fallback. If all fail, the geom component is created but has no visual mesh — it still exists in the spec as a collision primitive. A warning is logged. The compile still succeeds since MuJoCo doesn't need visual meshes.

### ZMQ connection failure
ZMQ uses `bind()` not `connect()` — it listens on ports. If the port is already in use (e.g., previous PIE session didn't clean up), ZMQ will log an error but the simulation still runs. External clients simply can't connect. There is no retry logic — the user must restart PIE.

### Python mesh preprocessing failure
Every step of the auto-clean-meshes pipeline has a fallback:
- Python not found → uses raw XML
- trimesh not installed → attempts pip install, if that fails → uses raw XML
- Script errors → uses raw XML
- The import always proceeds; mesh preprocessing is best-effort

### ValidateSpec failure
Shows a dialog with the MuJoCo error. Does NOT prevent the Blueprint from being saved — it's informational. The user can fix the issue and recompile.

---

## Debug XML (Diagnostics)

When `bSaveDebugXml = true` on the AAMjManager, after successful compilation the system saves:
- `Saved/URLab/scene_compiled.xml` — the full MuJoCo spec as XML
- `Saved/URLab/scene_compiled.mjb` — the compiled binary model

The XML undergoes path relativization: absolute paths like `C:/Users/.../Saved/URLab/ConvertedMeshes/...` are stripped to `ConvertedMeshes/...` relative to the output directory. Double slashes and `../` traversals are cleaned.

**Diagnostic use:** Diff the original MJCF XML against `scene_compiled.xml` to identify import/export mismatches. Missing elements, wrong property values, or broken default inheritance will be visible in the diff. The compiled XML can also be loaded directly into native MuJoCo (`mj_loadXML`) to verify it produces the same behavior.

---

## Key File Reference

| Path | Purpose |
|---|---|
| `Source/URLab/Private/URLab.cpp` | Module startup, DLL loading |
| `Source/URLab/Public/MuJoCo/Core/AMjManager.h` | Manager actor header (thin coordinator) |
| `Source/URLab/Private/MuJoCo/Core/AMjManager.cpp` | Manager lifecycle, subsystem wiring |
| `Source/URLab/Public/MuJoCo/Core/MjPhysicsEngine.h` | Physics engine subsystem (spec, model, data, async loop) |
| `Source/URLab/Public/MuJoCo/Core/MjDebugVisualizer.h` | Debug visualization subsystem |
| `Source/URLab/Public/MuJoCo/Core/MjDebugTypes.h` | `FMuJoCoDebugData` shared struct |
| `Source/URLab/Public/MuJoCo/Net/MjNetworkManager.h` | Network/ZMQ discovery subsystem |
| `Source/URLab/Public/MuJoCo/Input/MjInputHandler.h` | Input/hotkey subsystem |
| `Source/URLab/Public/MuJoCo/Core/MjArticulation.h` | Articulation actor header |
| `Source/URLab/Private/MuJoCo/Core/MjArticulation.cpp` | Articulation setup, attach |
| `Source/URLab/Public/MuJoCo/Components/MjComponent.h` | Base component, BindToView |
| `Source/URLab/Public/MuJoCo/Utils/MjBind.h` | View structs (BodyView, GeomView, etc.) |
| `Source/URLab/Public/MuJoCo/Utils/MjUtils.h` | Coordinate conversions |
| `Source/URLab/Public/MuJoCo/Core/Spec/MjSpecWrapper.h` | Spec element creation wrapper |
| `Source/URLab/Public/MuJoCo/Components/Defaults/MjDefault.h` | Default system |
| `Source/URLabEditor/Private/MujocoImportFactory.cpp` | MJCF import pipeline |
| `Source/URLabEditor/Private/MjArticulationFactory.cpp` | Blueprint generation |
| `Source/URLab/Public/Replay/MjReplayTypes.h` | Replay data structures |
| `Source/URLab/Public/MuJoCo/Net/MjZmqComponent.h` | ZMQ base component |
