# Fast-path render server + server browser (WIP handoff)

TEMPORARY doc. Delete before merging `feat/mjb-fast-path`. Explains how the
packaged fast-path render server works and lists the small things still open.

## What it is

A MuJoCo **owner** steps a sim and streams it; one or more UE **render slaves**
mirror it with no physics of their own (puppet mode). A slave discovers owners
and joins them through a runtime **server browser**, so a cooked `url_proj.exe`
is a standalone render node.

```
owner (python, menagerie_swap.py)                 render slave (UE -game / packaged)
  steps MuJoCo + random control        bus       AMjbScene puppet mirrors transforms
  serves fastpath_hello (MJB bytes)  <------->    + copycat viewport camera
  advertises in the shared registry   fastpath   RPC fastpath_load for live swaps
                                       _hello
```

## Pieces

- **Owner** `URLab_Bridge/scripts/menagerie_swap.py`: Tk scene picker (list +
  "Browse file..." for any XML), random control, MuJoCo viewer, streams per-body
  transforms + camera + free-cam pose on a ZMQ bus, serves the MJB on
  `fastpath_hello`, and writes a registry entry. `FastPathOwner.update_model()`
  keeps the served MJB current across live swaps.
- **Discovery** `MjbOwnerDiscovery.{h,cpp}` (`URLabFastPath::DiscoverOwners`):
  scans the shared registry dir (`%LOCALAPPDATA%/URLab/registry`), 30s TTL.
- **Subsystem** `MjbRenderSlaveSubsystem` (GameInstance): owner list, cooked-level
  auto-list, JoinOwner (pull MJB via `AMjbScene::FetchModelFromOwner`, OpenLevel,
  pending-join across the transition), auto-join, live origin nudge, HUD.
- **UI** `SMjbRenderSlaveBrowser` (browser) + `SMjbSlaveHud` (in-slave: back to
  browser + live X/Y/Z spawn-origin nudge).
- **Builder** `AMjbScene::SpawnRenderSlave(...)`: one shared spawn for the -game
  launcher and the browser (lights unless base-level, framing camera, a
  high-quality de-grain cvar preset).

## Running it

Owner (browser mode, GUI picker):

```
URLAB_PICKER=1 URLAB_LAUNCH_LOCAL=0 uv run python scripts/menagerie_swap.py
```

Render slave (editor `-game` now; the cooked `url_proj.exe` takes the same flags):

```
url_proj.exe /Engine/Maps/Entry -URLabFastBrowser              # interactive browser
url_proj.exe /Engine/Maps/Entry -URLabFastAutoJoin[=scene]     # headless render node
```

Flags: `-URLabFastBrowser`, `-URLabFastAutoJoin[=scene]`, `-URLabFastLevel=/Game/...`,
`-URLabFastOrigin=X,Y,Z` (UE cm), `-URLabFastBaseLevel` (use the map's own lights),
`-URLabFastCameras`, `-URLabFastNoQuality` (skip the de-grain preset).

In the browser: pick an owner, pick an environment (Bare Plane or any cooked
`/Game` level), set Origin, Connect. In the HUD: nudge Origin live, or go back to
the browser without relaunching.

## Materials / PBR

Full PBR is forwarded: scalar metallic/roughness/specular/reflectance/emission,
plus map roles (rgb, roughness, metallic, normal, occlusion, opacity, emissive,
rgba) and a single packed **ORM** map (R=occlusion, G=roughness, B=metallic).
The master material forms `scalar * map`, so an unset scalar is passed through at
neutral 1.0 when a map (or ORM) supplies the value; otherwise metallic 0 /
roughness `1-shininess`. Fixed in both the fast path and the authoring/import path
so ORM-authored parts are no longer crushed to non-metallic. De-grain preset (Epic
scalability + film-grain/motion-blur off) is applied on every slave spawn.

## What's left

To merge this branch:
- **Packaged re-cook**: the current cooked exe predates the PBR fixes and only has
  the bare-plane environment. Re-cook to pick up all material fixes and to include
  custom `/Game` levels in the browser dropdown (GateBackyard-scale envs are heavy).
- **Delete this WIP doc** before merge.
- **PR / merge** into the target branch.

Tech debt (bigger, see memory `project_terminology_and_fastpath_bridge_debt`):
- **Terminology cleanup**: client / viewer / server / render / slave / owner /
  puppet are overloaded and mixing; needs a consistent vocabulary pass across code
  + docs.
- **Fast-path shadow-articulation bridge**: the shadow-model hack for RPC control
  (`URLabFastShadow::Build`, `UMjPhysicsEngine::InstallRawModel`) + the raw-mode
  ctrl NetworkValue dual-write want a cleaner control surface.
- **God-object extraction** (audit #1, task #21): pull `FMjbAssetBaker` +
  `FMjbTransportBus` + `FMjbDirectMode` out of the ~2k-line `MjbScene.cpp`.

Optional polish:
- Per-level default env + origin in the browser (pre-select with the right offset).
- Direct origin type-in in the HUD (currently nudge-only).
- Camera-feed viewer wired into the browser flow.
- Owner exits if its MuJoCo viewer window is closed (minor robustness).

Done this session (for reference): render server + browser; copycat cam; spawn
origin + base-level; texrepeat/texuniform ground plane; normal-map TC_Normalmap;
PBR scalar-vs-map (fast + authoring); render de-grain preset; aloha normal-import
(task #11, both smoothing + tabletop texture).

Parked / unrelated: full mjModel->articulation bake (task #13); packaged-path
investigation (task #24) is effectively done now.

## Tech-debt investigation results (2026-08-15, three read-only analyses)

### 1. Terminology
Worst collisions: **puppet** = Python authority / UE mirror / RPC step-mode (3);
**owner** = sim authority / control-claim / lease-holder / UObject (4); **server** =
BridgeServer / camera-producer / discoverable-sim / "step server" (4); **client** =
Python driver AND the UE SUB receiver (opposite wire ends). Canonical set: keep
Client/BridgeServer/Manager/Controller (qualified); ELIMINATE "slave" and "puppet"
from names; render mirror -> **Renderer** (`EMjbRunMode::Puppet`->`Mirror`); sim
authority -> **Authority** (not owner); control/lease owner -> **holder**; "render
server" -> **CameraServer**. Rename plan tiered: Tier 1 internal/safe (purge slave,
owner->holder); Tier 2 Blueprint enum + fast-path C++ symbols (keep wire strings);
Tier 3 public Python `StepMode.PUPPET` + wire (aliases, defer). KEEP all wire strings
(`puppet`/`geoms`/`viewer`/`fastpath_hello`/`fastpath_owner`) unless a protocol bump.

### 2. Fast-path shadow articulation
Why it exists: the RPC control/observation layer + Python client address everything
as an `AMjArticulation` by name, but `InstallRawModel` installs a raw mjModel with
ZERO articulations, so `URLabFastShadow::Build` spawns one geometry-less fake art
whose per-element components are name-bound to the raw model. Hacky because: (a)
parallel object graph duplicating the model as N runtime UMjNodeComponents; (b) the
handshake ALSO ships a separate `raw_actuators`/`raw_joints` description off mjModel
(model described twice); (c) the ctrl "dual-write" -- raw ctrl reaches `d->ctrl`
either directly OR via staged NetworkControl slots, decided at two independent sites,
with a latent clobber (a bSkipController=false pass on a raw art can zero the command).
Recommendation: **Design C first** (make raw ctrl flow through ONE writer, resolve
raw-ness once -- a correctness fix, cheap, independently shippable), **then Design A**
(data-driven raw articulation: keep one thin AMjArticulation as the ActorId/ownership
anchor but drop the per-element component spawn and read state off the raw model under
CallbackMutex). Design B (no articulation at all) rejected as too invasive. Main risk
in A: fence discipline (raw state reads vs ReloadFromBytes/UninstallRawModel).

### 3. MjbScene.cpp god-object extraction
Extract order (risk-minimizing): **(1) UMjbTransportBus** (smallest: StartBus/StopBus/
OnBusMessage + Rx buffer; decode stays in Tick via TryPopLatest). **(2) UMjbAssetBaker**
(biggest LOC: GetOrBuildStaticMesh/GetOrBuildTexture/BuildMesh/ApplyGeomMaterial/
BuildMeshArrays + caches + the WITH_EDITOR-vs-PMC split; leave BuildGeom in the scene
delegating to it). **(3) FMjbDirectMode** (most entangled: DirectManager/ShadowArt/
install-timer/ApplyFromSnapshot). KEY correction: baker + bus must be **UObjects**
(cached UTexture2D/UStaticMesh/Master/BusTransport are UPROPERTY(Transient) for GC
rooting -- plain F-structs would need FGCObject); direct mode can be a plain struct
(weak ptrs + PODs). Stays on the actor: Model/Data ownership, the scene graph
(GeomComps/CameraComps/BodyActors), `BuildGeom`, and `EnsureManager` (shared by Direct
AND Puppet). Hard constraint: preserve Teardown order (engine uninstall + shadow
teardown BEFORE freeing Model/Data).

Cross-cutting: the shadow (item 2 Design A) and FMjbDirectMode (item 3) both touch the
shadow, and the terminology rename (item 1) touches all fast-path symbols. Suggested
overall order: shadow Design C (correctness) -> god-object bus+baker (independent of
shadow) -> shadow Design A -> FMjbDirectMode -> terminology rename pass last.

## Clean-slate bridge abstraction redesign (2026-08-15, chosen direction)

Supersedes item 2 above (the incremental shadow Design C/A). After the three
read-only analyses the user redirected: no backwards-compat concern (wire protocol
and Python API are both free to change), prioritize a clean end-state over
incrementalism, drop "articulation" as the bridge addressing unit if it does not
earn its place. A fourth read-only agent produced this. NOT started in code; the
user is low on quota. Resume here.

### The load-bearing facts (verified)
- The Python client already owns a full local `mjModel`/`mjData` mirror
  (`URLabEntity.root_pos_w` reads `client.data.xpos[body_id]`; flat qpos/qvel +
  `mj(kind,name)` resolve against the client's own model). The server ships
  per-element metadata the client largely already has.
- The raw path already demonstrates component-free addressing: `FMjEntityRecord`
  (`AMjManager.h:82`) is `{MjId, Name, bHasFreeBase}` and the collector indexes
  `mjData` directly by id for the `entities` block. No `UMjNodeComponent` graph
  is involved for entities. Proof the per-element component graph is NOT required.
- The shadow exists only to satisfy an actor-shaped interface. `URLabFastShadow::Build`
  (`MjbShadowArticulation.cpp:52-108`) spawns a geometryless `AMjArticulation`,
  fabricates one `UMjNodeComponent` per joint/actuator, name-binds to the raw model,
  sizes control slots, and registers -- purely so `GetAllArticulations()`, the state
  collector's producer cache, and control ownership have an actor to key on.
- The model is described TWICE on the raw path: the shadow's components describe it,
  AND the handshake re-emits `raw_actuators`/`raw_joints` straight off `mjModel`
  (`RpcDispatcher.cpp:885-951`).
- Control has a dual-write with two decision sites: `ApplyStepCtrl` either writes
  `d->ctrl[id]` directly (raw) or stages `NetworkControl`
  (`RpcHandlers_Step.cpp:452-475`); then `AMjArticulation::ApplyControls` copies staged
  slots into `d->ctrl` for `OwnedActuatorIds` only (`MjArticulation.cpp:497-510`) -- the
  `OwnedActuatorIds` gate exists solely to stop one articulation's unset zeroes from
  clobbering another's ctrl. The direct handler independently rebuilds a `SkipController`
  map (`RpcHandlers_Step.cpp:1022-1036`).
- Four overlapping mode concepts: `EStepMode{Live,Direct,Puppet,Auto}`
  (`MjPhysicsEngine.h:66`), `EMjbRunMode{Puppet,Direct}` (`MjbScene.h:27`),
  `control_mode{raw,ue_controller}` (`enums.py:33`), `ControlSource{ZMQ,UI}`
  (`MjArticulation.h:410`). `EMjbRunMode.Direct` and `EStepMode.Direct` are the same
  idea twice.

### First principles: exactly four operations the layer must expose
1. Address controllable/observable groups within the one live model, by stable name.
2. Read state for an addressed group (qpos/qvel/sensordata/derived), coherently per step.
3. Write control for an addressed group into the one control store, with an optional
   transform.
4. Arbitrate who is allowed to write (ownership/lease).

None of the four needs a UObject per joint. There is one `mjModel` and one `mjData`;
addressing is fundamentally a name bound to a set of MuJoCo ids.

### Core abstraction: Entity (drop "articulation" from the bridge)
An Entity is a stable name bound to the set of MuJoCo ids (body/joint/actuator/sensor
slice) the client treats as one addressable unit. A robot owns actuators; a prop owns
none. There is no second kind. "Entity" is already the base concept in the Python client
(`URLabEntity`, with `URLabArticulation(URLabEntity)` as a thin subclass, plus a shipped
`entities` handshake block) -- this promotes the existing superclass and deletes the
subclass distinction. Server-side, the addressing unit becomes one POD, not a UObject:

```cpp
// Replaces both FMjEntityRecord AND AMjArticulation-as-addressing-unit.
struct FMjEntity
{
    FName    Name;            // stable public name (was ActorId/ArtSegment)
    int32    RootBodyId;      // root-link state + free-base detection
    bool     bFreeBase;
    TArray<int32> ActuatorIds;   // may be empty (a prop owns no actuators)
    TArray<int32> JointIds;
    TArray<int32> SensorIds;
    TArray<int32> BodyIds;
};
```

`AMjArticulation` stays, but demoted to the compiled path's authoring/rendering actor
only (spec tree, meshes, cameras, Blueprint API, debug draws). The bridge never sees it.
Optional rename to `AMjRobot`/`AMjModelActor`, orthogonal to this plan.

### Both paths present the SAME abstraction (shadow eliminated)
One builder, model-only, no UObjects, no per-element components:

```cpp
namespace MjEntityBuilder {
    // Partition the live model into entities by name prefix (compiled: participant
    // prefixes; raw: top-level body subtrees / a single "root").
    TArray<FMjEntity> Build(const mjModel* m, const FEntityPartition& How);
}
```

- Compiled path: after `InstallCompiledSpec`, `Build(m, {participant prefixes})`. The
  compiled `AMjArticulation` actors still exist for rendering; the bridge gets `FMjEntity`
  views keyed by the same prefixes (their `ActorId` becomes the entity `Name`).
- Raw/fast path: after `InstallRawModel`, `Build(m, {single root or body-subtree split})`.
  NO shadow actor, NO fabricated `UMjNodeComponent`s, NO `RegisterArticulation`.
  `MjbShadowArticulation.{h,cpp}` DELETED.
- Handshake is one code path: serialize `FMjEntity` + read names/ranges/gear/joint-types
  straight off `mjModel` for every entity, compiled or raw. The `raw_actuators`/
  `raw_joints` block and `bRawShadow` DISAPPEAR. Model described exactly once, the same
  way, both paths. Bonus: this fixes the compiled path's `actuator_types` gap (the MJB
  can't carry `<position>` vs `<general>`; shipping compiled gear/ctrlrange/trntype off
  the model is the honest fix and matches the client's local model).
- State collector loses its component-producer cache entirely: walk `FMjEntity[]`, index
  `mjData` by id. `FMjStateCollector`'s `FCachedArticulation`/`Producers`/
  `RebuildProducerCacheGameThread` (`MjStateCollector.h:76-114`) collapse; game-thread
  cache rebuilds and stale-weak-ptr invalidation go away (an id slice can't dangle like a
  `TWeakObjectPtr<UMjNodeComponent>`).
- Keep the compiled `UMjSensorRuntime`/`UMjJointRuntime` "DescribeState" logic ONLY as an
  optional ROS-typed-message enrichment keyed off the entity's sensor ids; it must NOT be
  on the msgpack/step-reply critical path (which reads `mjData` directly).

### Control: one store, one write path
One control store (`d->ctrl`), one writer (the step loop's pre-step drain under
`CallbackMutex`, modeled on `UMjPhysicsEngine::DrainCommands` which already works for
mocap/wrench). Replace the dual `NetworkControl`/`InternalControl` atomic slot arrays and
`ApplyControls` with a single engine-owned setpoint buffer sized to `nu`:

```cpp
struct FMjControlBuffer {
    TArray<double> Setpoint;   // size nu; last-write-wins per id
    TBitArray<>    Touched;    // only touched ids written -> no zero-clobber
};
```

Drain, single site, pre-step:
```
for each id in Touched:
    if entity_of[id].drive == Direct: d->ctrl[id] = Setpoint[id];
    else /* Controller */:            d->ctrl[id] = controller.transform(id, Setpoint[id], d);
clear Touched;   // unless "hold" latch (keyframes)
```

Kills: the dual-write (`ApplyStepCtrl`'s two branches become one setpoint parse; `Drive`
is a per-entity property read at drain time); the `OwnedActuatorIds` zero-clobber guard
(the `Touched` mask means an entity only writes ids it set); the `SkipController` rebuild;
`StageNetworkControl`/`StageInternalControl`/`ResolveDesiredControl`/`ClearStagedControl`
and the fixed atomic arrays (`MjArticulation.h:181-206, 485-488`). Compiled PD/keyframe
controllers become the `Controller` branch -- `UMjArticulationController::ComputeAndApply`
refactored from "write `d->ctrl` yourself" to "return the ctrl for your bound ids"; the
engine does the one write. Keyframe hold becomes a per-entity latch on the setpoint buffer
(`Touched` stays set) rather than an early-return inside `ApplyControls`.
`AMjArticulation::ApplyControls` (both overloads) DELETED; ownership of `d->ctrl` moves
entirely into `UMjPhysicsEngine`.

### Modes: three orthogonal axes + lease (replacing four tangled enums)
- Axis A -- Clock (who advances physics time; session-scoped). Replaces `EStepMode` AND
  `EMjbRunMode.Direct`: `engine` (UE free-runs `mj_step`, was Live) / `stepped` (client
  pulls `step(n)`, was Direct) / `external` (client integrates elsewhere + pushes
  qpos/qvel/ctrl, server runs `mj_forward`, was Puppet). `Auto` becomes a launch-time
  resolution policy, not an enum value the worker branches on.
- Axis B -- Drive (how a setpoint reaches `d->ctrl`; per-entity). Replaces `control_mode`:
  `direct` (was raw) / `controller` (was ue_controller). Resolved at drain, never at two
  sites again.
- Axis C -- Render source (where a renderer's poses come from; per render instance).
  Replaces `EMjbRunMode.Puppet` vs implicit "render my own snapshot": `local` (draw own
  stepped snapshot) / `mirror` (subscribe to another instance's transform stream, was
  `EMjbRunMode.Puppet`). Splitting Clock from RenderSource makes a mirror that ALSO steps
  expressible, and a `stepped` instance rendering locally needs no separate enum.
- NOT a mode: ownership/lease. `ControlSource{ZMQ,UI}` is the identity of a writer for
  arbitration. The `FMjControlOwnership` lease (`RpcHandlers_Control.cpp:101-189`, keyed
  by name) already does this; rekey it from `Art->GetName()` to `FMjEntity::Name`. Delete
  `EControlSource` and the per-articulation `ControlSource uint8`; UI is just another lease
  holder.

Resulting surface: one session enum (Clock, 3), one per-entity property (Drive, 2), one
per-render-instance property (RenderSource, 2), one lease. Down from
EStepMode(4) x EMjbRunMode(2) x control_mode(2) x ControlSource(2).

### Migration order (each step compiles/ships on its own)
1. Land `FMjEntity` + `MjEntityBuilder::Build` for the compiled path, built alongside the
   existing articulation registry (dual-run, assert equality in tests). No behavior change.
2. Repoint the state collector and handshake at `FMjEntity` (compiled path), reading
   `mjModel`/`mjData` directly. Delete the producer-cache graph + component `DescribeState`
   msgpack path (keep ROS enrichment behind the ROS encoder).
3. Introduce the engine control buffer + single drain; route `ApplyStepCtrl` into it;
   delete staged slots, `ApplyControls`, `SkipController`. Controllers -> `transform`.
   (HIGHEST RISK step: keyframe-hold ordering vs setpoints, controllers that read
   `ControlSource`.)
4. Build `FMjEntity` on the raw path from `mjModel`; delete `MjbShadowArticulation`, the
   shadow spawn/teardown in `MjbScene`, the `raw_*` handshake block, `bRawShadow`.
5. Collapse the modes: rename `EStepMode`->`Clock`; split `EMjbRunMode` into
   `Clock`+`RenderSource`; delete `EControlSource`; rekey the lease to `FMjEntity::Name`.
   Update Python `enums.py` (wire strings change -- allowed).
6. Python: merge `URLabArticulation` into `URLabEntity`; most of `_walk_model`/
   `_build_from_raw_handshake` (`articulation.py:797-989`) collapses into one
   prefix-bucketing pass over the local model driven by `FMjEntity` name+id slices.
   `client.articulations` becomes a filtered view over `client.entities` (those with
   actuators), or is dropped.

### Blast radius (honest -- large, deliberately not incremental-friendly)
- Deleted/gutted: `MjbShadowArticulation.{h,cpp}`; `AMjArticulation`'s control subsystem
  (staged slots, `ApplyControls`, owned-id gating, `ResetControlSlots`); `FMjStateCollector`'s
  producer cache; `EControlSource`; the `raw_*` handshake block; `EMjbRunMode`.
- Heavily rewritten: `RpcHandlers_Step.cpp`, `RpcHandlers_Control.cpp` (lease rekey),
  `RpcDispatcher.cpp` handshake, `MjPhysicsEngine` (control buffer + drain + entity
  registry), `MjbScene.cpp` (Direct no longer spawns a shadow), all three controllers.
- Wire + Python API changes (allowed): handshake `articulations`/`entities` unify; the
  `raw_*`/`bRawShadow`/`default_control_mode` semantics change; `control_mode`/step-mode
  strings rename; `URLabArticulation` disappears as a distinct type.
- Tests rewritten against `FMjEntity`: `MjStateCollectorTests`, `MjStepServerTests`,
  `MjControlOwnershipTests`, `MjUserChannelTests`.

Payoff: model described once, `d->ctrl` written from one place, one addressable concept
with one good name, and the shadow -- the clearest symptom the old abstraction didn't fit
the raw path -- gone entirely.

### Critical files
- `Source/URLab/Public/MuJoCo/Core/MjPhysicsEngine.h` (host `FMjEntity` registry + control
  buffer + pre-step drain)
- `Source/URLab/Private/Bridge/RpcHandlers_Step.cpp` (fold dual-write into one setpoint
  parse; retarget step strategies to `Clock`)
- `Source/URLab/Private/Bridge/RpcDispatcher.cpp` (single `FMjEntity` handshake serializer;
  delete the `raw_*` block)
- `Source/URLab/Public/State/MjStateCollector.h` + `Private/State/MjStateCollector.cpp`
  (collapse producer graph to id-slice iteration over `mjData`)
- `Plugins/URLab_Bridge/src/urlab_client/articulation.py` (merge `URLabArticulation` into
  a single `Entity`)
- DELETE: `Source/URLab/Private/MuJoCo/Fast/MjbShadowArticulation.cpp` + its header.

### Open decision for the user (before code starts)
- Name: lands on "Entity" (drop "articulation" from the bridge, keep it only as the render
  actor). Confirm "Entity" or pick another word.
- Scope: whole end-state vs. pull out step 3 (one control store / kill the dual-write)
  first as a standalone correctness fix.
