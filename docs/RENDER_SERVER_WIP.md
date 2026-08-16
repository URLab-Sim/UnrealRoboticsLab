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

### Investigation findings (2026-08-15, verified against code -- READ THIS)

A second read-only agent verified the redesign section above against the real code. The
proposal's DIRECTION survives, but several load-bearing citations were wrong and six holes
surfaced. Treat the corrections below as authoritative over the section above.

**Facts re-checked:**
- VERIFIED: the Python client owns a full local mjModel/mjData mirror (`client.py:336-337,
  2316, 2297`; `articulation.py:512, 1017`) -- this, not fact 2, is the real proof that
  id-slice addressing works. VERIFIED: the shadow (`MjbShadowArticulation.cpp:52-108`), the
  double model description (`RpcDispatcher.cpp:885-951`), the ctrl dual-write
  (`RpcHandlers_Step.cpp:452-475` + `MjArticulation.cpp:497-510` + SkipController
  `1022-1036`).
- WRONG -- fact 2 mis-attributed: `FMjEntityRecord` (`AMjManager.h:82`) is the COMPILED
  props table (bodies only, and it has a 4th field `TWeakObjectPtr<UMjBody> BodyComp`), NOT
  the raw path. The `entities` handshake block is `RpcDispatcher.cpp:1021-1055`, not
  MjStateCollector. Its cache is BUILT by walking the component graph
  (`AMjManager.cpp:178-179`); id-index only at read time. So it does not demonstrate
  "component-free" addressing.
- WRONG -- `EControlSource` is at `MjPhysicsEngine.h:44` (not `MjArticulation.h:410`, which
  is a bare `uint8`). It is NOT pure arbitration: it selects the ZMQ vs UI staged slot in
  `ResolveDesiredControl` (`MjArticulation.cpp:414-425`) and is passed into controllers
  (`MjPDController.cpp:72`, `MjPassthroughController.cpp:35`).
- WRONG -- `EMjbRunMode::Direct` is NOT the same as `EStepMode::Direct`: it spans engine-
  clock AND stepped-clock ("driven by the RPC layer or free-running", `MjbScene.h:31-33`).
  The 3-axis split still holds (it is actually vindicated by this), but the stated
  justification was false.

**Six design holes (ranked):**
1. HIGH -- the nu-sized ctrl setpoint buffer CANNOT represent keyframe qpos-hold.
   `ApplyControls` has two hold paths (`MjArticulation.cpp:452-489`): `bHoldViaQpos` writes
   `qpos`+zeroes `qvel` (not ctrl); only ctrl-hold writes `d->ctrl`. A ctrl-only buffer
   silently regresses qpos-hold. Needs a separate qpos/qvel injection channel (mocap-style;
   DrainCommands has mocap/wrench but not qpos).
2. HIGH -- deleting `EControlSource` changes controller semantics (ZMQ+UI can no longer both
   stage with per-art source selection -> becomes last-write-wins by lease) AND every
   controller's `ComputeAndApply(Source)` signature changes. Plus `SetNetworkControl` has
   FOUR ingress callers to re-home: `RpcHandlers_Step.cpp:473`, `ZmqSubscribeTransport.cpp:446`,
   `RosRpcTransport.cpp:450` + `:571` -- and a third control write exists at
   `MjPhysicsEngine.cpp:1238` (Live-mode `ApplyControls`).
3. HIGH -- the producer cache is NOT purely mjData id-slices. `FCachedArticulation`
   (`MjStateCollector.h:76-85`) also carries `TwistCtrl` (UMjTwistController),
   `InterfaceProducers` (user-channel IMjStateProducer), plus scene-scoped `SceneProducers`.
   "Walk FMjEntity[], index mjData" silently drops user channels + twist + scene producers.
   FMjEntity needs an explicit component-enrichment SIDE-CHANNEL; do not "delete the producer
   cache entirely."
4. MED -- deleting the shadow strands ~30 `GetAllArticulations` consumers on the raw path,
   none listed: `MjDebugVisualizer.cpp` (6), `ZmqSubscribeTransport.cpp` (control subscriber),
   `MjReplayManager.cpp:235`, `RosCameraInfoProvider.cpp:87`, `MjSimulateWidget.cpp`,
   `RosRpcTransport.cpp:262`. Each must be repointed at FMjEntity or excluded.
5. MED -- see the EMjbRunMode/EStepMode correction above.
6. LOW-MED -- step 1's "dual-run, assert equality" is only feasible for qpos/qvel/sensordata
   (there is golden-diff precedent in `MjParityGoldenTests`); the observation also includes
   user-channel/twist data FMjEntity won't produce, so scope the assert.

**Corrected blast radius (add to the section's list):** `MjActuatorRuntime.{h,cpp}`
(SetNetworkControl/ResolveDesiredControl statics); `RpcHandlers_SimOptions.cpp:410-468`
(HandleSetControlSource -- the whole `set_control_source` op); `RpcHandlers_ModelUpload.cpp:593`
+ `StepCommands.h:35` (PerArticulationControlMode); `MjPhysicsEngine.cpp:1174-1176, 1238`;
UI `SMjStepModeIndicator.cpp`, `MjPerturbation.cpp:79`; state producers
`AMjManager.cpp:538-566`; tests beyond the 4 named: `MjActuatorControlSlotTests`,
`MjPDControllerTests`, `MjRosLinkTests`, `MjThreadTests`, `MjSnapshotAccessorTests`,
`MjAttachPolicyTests`.

**Refined recommendation:**
- Name: "Entity" is defensible (already the Python base class) BUT collides with the existing
  `FMjEntityRecord` / `entities` handshake block (compiled props, bodies-only). Either rename
  the old struct or pick a distinct term (`FMjGroup` / `FMjAddressable`). Decide before code.
- Abstraction: FMjEntity is the right addressing + core-state unit, but keep an explicit
  component-enrichment side-channel (user channels, twist, scene producers, camera meta,
  debug draw); it does NOT subsume them.
- Scope: peel a NARROWED step 3 first -- unify the raw ctrl write so raw-ness resolves once
  and the owned-id/Touched invariant is explicit, WITHOUT yet deleting `EControlSource` or
  touching the qpos-hold path. That is exactly the cheap "Design C first" from item 2 above
  (line ~131) and de-risks holes 1+2 before the big rewrite commits to them. Then FMjEntity
  for compiled state (with side-channel) -> raw path + shadow deletion -> mode collapse.

**Future API A (load_model) -- MORE ready than the section assumes:**
- UE runtime already links the whole compile toolchain: `mj_parseXMLString`
  (`MjSpecWriteHooks.cpp:2034`), `mj_compile` (`MjSceneSpec.cpp:386`), `mjVFS`/`mj_addBufferVFS`
  (`MjSceneSpec.cpp:377-387`), `mj_saveModel` (`MjPhysicsEngine.cpp:1022`,
  `RpcHandlers_Fastpath.cpp:61`), `mj_loadModelBuffer` (`MjbScene.cpp:471`). So xml(+assets)->MJB
  is a recombination of EXISTING runtime calls, not a new dependency.
- The doc's "authoring import can't accept bytes" is OBSOLETE: `upload_model_manifest/chunk/commit`
  (`RpcHandlers_ModelUpload.cpp:448-512`) already materializes XML+asset bytes to a temp dir and
  drives import_xml + clean_meshes; client `URLabClient.upload_model(xml: str|bytes, assets=...)`
  (`client.py:1771`) already takes bytes. So `load_model(fast_path=False)` is mostly a facade
  over `upload_model`.
- mjz is the ONLY net-new piece (no support in either repo; codec only in vendored MuJoCo,
  maybe not compiled in).
- Recommended shape: explicit `format` field (auto-sniff optional), KEEP the `assets` map (do
  not force client-side zip), normalize UE-side via the calls above, map `fast_path` onto the
  Clock/RenderSource axes. Also add the missing client wrapper for `fastpath_load` (menagerie_swap
  hand-rolls the op dict at `menagerie_swap.py:251`).

**Future API B (articulation split) -- attach is the easy 10%, keyframes the hard 90%:**
- `mjs_attach(frame, spec, prefix, "")` is the live compose path (`MjSceneSpec.cpp:532-533`);
  cross-body data is first-class UE spec objects (`UMjPair`/`UMjExclude`/`UMjContact`/
  `UMjEquality`/`UMjTendon`/`UMjKey`) referencing partners by name. Anything wholly inside one
  child survives namespacing free.
- Genuinely hard: contact/exclude/equality/tendon spanning two children can't be expressed
  post-attach (no cross-participant prefixed-name resolution exists); keyframes are hardest -- a
  `<key>` is one flat qpos/ctrl vector over whole-model nq/nu (`UMjKey.Qpos`,
  `MjArticulation.cpp:814`, sliced `RpcHandlers_Scene.cpp:511`), and nothing fragments/recombines
  them across participants.
- Gate: DO NOT diff against `scene_compiled.xml` (known VFS-asset-ref reload problem, see
  [[project_scene_compiled_xml_not_loadable]]); use the compiled-mjModel FIELD diff
  (`model_diff_lib.h`) that `MjParityGoldenTests` (`.scene2` p0_/p1_ golden) already runs -- a
  split-then-link diff is a near-drop-in extension.

## Future API discussion points (2026-08-15, not scoped, user notes)

Two ideas the user raised to capture for later. Both are DISCUSSION ONLY, likely partly
wrong, meant to seed design when quota is back. Design goal stated by the user: keep the
RPC/endpoint surface as minimal and uncluttered as possible while accepting a wide range
of inputs.

### A. Unified `load_model()` RPC (one endpoint, many input formats)

Today loading is split across several paths (fast-path `fastpath_load` with MJB bytes vs
the authoring import factory that takes an XML path and runs `clean_meshes.py`). The user
wants a single `load_model()` on both the Python client and UE instances that swallows the
lot.

Proposed shape (rough):

```
load_model(payload: bytes, format: {mjb|xml|mjz}, fast_path: bool = False,
           assets: map<name,bytes> = {}, name: str = ...) -> handle
```

Behavior sketch:
- `fast_path = True` (renderer / raw-model path, always resolves to an MJB):
  - `mjb`  -> load the bytes directly (already the fast path).
  - `xml (+ assets)` -> feed XML + asset bytes into an `mjSpec` via `mjVFS`, compile,
    dump the MJB, load it.
  - `mjz`  -> unpack the self-contained archive, same `mjSpec` -> dump MJB -> load.
  - i.e. the fast path has ONE real requirement (an MJB); every other format is just a
    normalize-to-MJB step in front of it.
- `fast_path = False` (usual `AMjArticulation` authoring import):
  - Today this path takes a path/XML and cannot accept bytes. To fit the same endpoint it
    would stage the payload to a temp location (write the XML, expand an mjz's assets to a
    temp dir, or write asset bytes next to the XML) and reuse the existing import factory +
    `clean_meshes.py`. "Cannot accept bytes yet" is really "needs a bytes -> temp-files
    shim in front of the importer."

Initial thoughts / open questions (agent's, to argue about later):
- Format encoding: explicit `format` field vs sniffing magic bytes (MJB has a header, mjz
  is a zip `PK\x03\x04`, xml is text). Explicit is less magic and matches the "clean surface"
  goal; could allow `format=auto` as sugar that sniffs. Lean explicit.
- Collapsing the input set: `xml + asset bytes` and `mjz` are the SAME internal case once
  unpacked (an XML plus a VFS of named asset blobs). One simplification worth considering:
  drop the separate `assets` map and require "XML with assets" to arrive as an mjz (client
  zips it). Then the wire accepts only two blob kinds, `mjb` and `mjz`, plus bare
  asset-free `xml` text. Fewer fields, same reach. Counter-argument: forcing a client-side
  zip is friction for the common "I have an xml and a folder" case; the `assets` map may be
  nicer ergonomically. Present both.
- Where normalization runs: UE-side is the general answer (UE instances need this API too
  and may have no Python client; UE already has `mjSpec`). Client-side normalize-to-MJB
  would keep the wire trivially "always MJB for fast path" but couples the client to
  mujoco-python's compiler. Prefer UE-side `mjSpec` compile so the endpoint is uniform for
  all callers.
- Return shape: fast path returns a loaded-scene handle; the authoring import returns a
  Blueprint handle you then `spawn_actor`. Two different return types under one endpoint is
  a wart. Decide whether `load_model` always returns a uniform handle (and spawning is a
  separate step for both) or the two arms honestly return different things.
- Interaction with the Entity redesign: with the bridge addressing by `FMjEntity` (id-slice
  of the one compiled model), `fast_path` stops being an input-format concern and becomes
  purely "who renders / who owns the integrator" (the Clock/RenderSource axes above). The
  format normalization (anything -> MJB) is orthogonal to that. Worth designing
  `load_model` so `fast_path` maps onto the mode axes rather than being a fourth ad hoc
  flag.

### B. Blueprint action: split an `AMjArticulation` into per-body articulations

Motivation: a huge XML compiles to a huge Blueprint that is both hard to read and expensive
to instantiate (see [[project_blueprint_edit_lag_unresolved]]). The user wants a BP action
on an `AMjArticulation` that separates each top-level body (direct child of `<worldbody>`)
into its own `AMjArticulation`, duplicating the shared authoring settings, plus a
"linking" articulation that references them all back into one simulable model.

Sketch:
- Split: for each top-level body, emit a child articulation carrying that body's subtree
  plus copies of the shared authoring blocks it needs: `<compiler>`, `<option>`, `<size>`,
  the referenced `<default>` classes, and only the `<asset>` entries that subtree uses.
- Link: a parent/composition articulation that re-includes the children (mechanism:
  `mjSpec` attach with per-child prefixes, the same tool the protospec/mjspec work uses;
  see [[project_mjspec_migration]] / [[project_protospec_plan]]) and carries the
  cross-cutting, whole-model-scope data that does not belong to any single child.

The hard part (user already flagged it): an XML can contain data that is inherently
whole-model, not per-body: `<contact>` pairs/excludes, `<equality>` constraints, tendons,
and keyframes (a `<key>` qpos/ctrl vector spans the entire model's DoF order). These cannot
be cleanly pushed down into a single split child. Options to weigh:
- Keep all cross-body relations on the linking articulation and re-apply them after attach
  (needs stable name remapping so a `<pair>` still resolves to the right geoms once children
  are attached with prefixes).
- Keep keyframes only on the link (they are defined over the recombined model's DoF layout,
  so they only make sense there anyway).
- Detect and warn when a split would sever a constraint that spans two children.

Initial thoughts:
- This is an AUTHORING / instantiation optimization only. At sim time MuJoCo still compiles
  ONE model, so the children must recombine (via attach) into a single `mjModel`. So this
  does NOT create multiple simulations, and with the Entity redesign it does not change what
  the bridge sees either: the compiled model is still one model partitioned into
  `FMjEntity`s. The split is purely to shrink/organize the Blueprints and speed BP
  instantiation. Good news: the two ideas are compatible and even reinforcing (per-body BPs
  map naturally onto per-body entities).
- `mjSpec` attach with prefixes is almost exactly the "link them back" primitive; the real
  work is the cross-cutting-data bookkeeping and name remapping, not the attach itself.
- Round-trip risk: split then link should reproduce the original compiled model bit-for-bit
  (or provably-equivalent). A golden compiled-model diff test would be the honest gate
  before trusting it (compare against `scene_compiled.xml`, see
  [[project_scene_compiled_xml_not_loadable]]).

## Full-infrastructure leanness audit (2026-08-15, four-agent synthesis)

The user broadened the mandate from "clean the bridge addressing" to "audit the whole
control / observation / object-graph / modes / transport stack for leanness and redesign
main infrastructure where it earns it; it exists is not a reason to keep it." Four read-only
agents each owned one subsystem; this section reconciles them and is authoritative over the
proposal + investigation sections above where they conflict. Nothing here is started in code.

### The unifying principle
There is ONE runtime truth: the compiled `mjModel`/`mjData`, indexed by id. Every other
structure is exactly one of three supporting roles:
- AUTHORING: the spec tree, editor-only, its job ends at compile.
- RENDERING: an actor with meshes/cameras/draw.
- ADDRESSING: a flat id-slice partition naming which slots a caller may touch.
Control, observation, and addressing all reduce to "index `mjData` by id." Anything that is
neither authoring nor rendering nor an id-slice is either a genuine mailbox side-channel
(user channels) or spaghetti. This single test drives every verdict below.

### Model representations: six today, target four
Today (object-graph audit): (1) authoring tree `UMjModel:UMjNodeComponent:USceneComponent`
(`MjModel.gen.h:27`, `AMjArticulation::Spec` `MjArticulation.h:88`); (2) retained runtime
`mjSpec` `FMjBuiltSpec::Spec` (`MjSpecBuild.h:61`, engine `InstalledScene`
`MjPhysicsEngine.h:555`); (3) compiled `mjModel`/`mjData`; (4) per-articulation element index
+ control slots (`MjArticulation.h:466-488`); (5) props table `FMjEntityRecord`, bodies-only,
built by walking `UMjBody` (`AMjManager.h:82`); (6) raw-path shadow
(`MjbShadowArticulation.cpp:52-108`).

Target FOUR: authoring spec / compiled `mjModel`+`mjData` / addressing partition POD /
render actor. #4 folds into the control buffer + partition; #5 folds into the partition; #6
DELETES. #2 (runtime `mjSpec`) is a candidate FIFTH copy to free after `mj_compile` pending a
use-audit (kept today only because a composed scene spec references participant specs,
`MjSceneSpec.h:110-116`; if nothing edits the spec in-place after install, free it).

### The addressing unit (naming resolved)
A flat POD partition built FROM `mjModel` by name-prefix, carrying id slices only
(`{Name, RootBodyId, bFreeBase, JointIds, ActuatorIds, SensorIds, BodyIds}`). It replaces
`FMjEntityRecord` + `AMjArticulation`-as-addressing-unit + the shadow, identically for compiled
and raw paths. NAMING: do not call it `FMjEntity` while `FMjEntityRecord` + the `entities` wire
block + Python `URLabEntity` already mean "prop". Preferred: name the POD `FMjAddressable` or
`FMjGroup`, retire `FMjEntityRecord` into it, keep the wire word `entities` for the unified
list. If `FMjEntity` is insisted on, rename the old struct to `FMjPropRecord` in the same
change. This is a decision to make before code.

Crucially, the POD does NOT subsume everything. Three classes of non-id-sliceable data attach
via ONE explicit enrichment side-channel keyed by entity name (NOT a per-element producer
graph, NOT on the core critical path): (a) compiled-path camera + controller handshake metadata
(`actuator_types`, controller kind/params/schema, `camera_topics` at `RpcDispatcher.cpp:838-1015`
-- none of it is on `mjModel`, so the compiled handshake still walks the render actor for it);
(b) user channels (a real, tested mailbox feature, `MjUserChannelComponent.cpp:383-389`);
(c) the twist echo (ROS/UI only, below). Same registration `GetStateProducers` already does
(`AMjManager.cpp:563-567`), minus the per-articulation fan-through, typically 0..few entries.

### Control (audit 1): one engine-owned buffer, one pre-step drain
Today the same setpoint reaches `d->ctrl` through FOUR-PLUS uncoordinated writers: raw direct
write (`RpcHandlers_Step.cpp:465-470`), staged-then-`ApplyControls` (`MjArticulation.cpp:504-510`),
keyframe ctrl-hold (`:485`), Live-mode engine `ApplyControls` (`MjPhysicsEngine.cpp:1238`), Puppet
push (`MjPhysicsEngine.cpp:580`), `ResetToKeyframe` (`:763`), plus the two controllers. Target:
one `FMjControlBuffer{Setpoint[nu], Touched, HoldLatch}` owned by the engine, drained pre-step
under `CallbackMutex` (modeled on the existing `DrainCommands` that already serializes
mocap/wrench). Per touched id: `Drive==direct` writes the setpoint; `Drive==pd` applies the PD
law. This DELETES: the `NetworkControl`/`InternalControl` dual slots, `ApplyControls` (both),
`SkipController`, `OwnedActuatorIds`, and both `ApplyStepCtrl` branches.

Verdicts: `UMjPassthroughController` DELETE (its own header says it is identical to the default
path; `Drive=direct` IS passthrough). `UMjPDController` KEEP THE LAW, data-ify the packaging
(gains struct + one free function called in the drain, not a `UObject` + `FindComponentByClass`;
keep `MjPDControllerTests` pointed at the free function). `EControlSource` + per-art `ControlSource`
+ the `set_control_source` RPC DELETE -- but note this is a BEHAVIOR change, not a rename:
`ControlSource` is today an active 2-slot input selector (`MjArticulation.cpp:420`) fed into
controllers, so collapsing to the lease makes control last-write-wins by lease holder. The lease
(`FMjControlOwnership`) is solid and tested; KEEP, rekey to the entity name.

qpos-hold (redesign HIGH hole 1, confirmed real): a ctrl-only buffer CANNOT represent
`bHoldViaQpos` (`MjArticulation.cpp:454-478` writes `qpos` + zeroes `qvel`, not ctrl). Add a
SIBLING state-injection buffer `{Qpos, QposHold}` drained in the same pass, symmetric with
mocap. Fixes a latent bug for free: today's qpos-hold loops `njnt` scene-wide (`:458`), so in a
multi-participant scene one entity's hold clobbers others' qpos; the id slice fixes it.

Behavior fix to flag, not hide: Live mode hard-codes `bSkipController=false`
(`MjPhysicsEngine.cpp:1238`) and never reads `control_mode`, so raw vs ue_controller silently
means nothing in Live today (raw only works by accident). Unifying Drive fixes this.

### Observation (audit 2): walk entities, index mjData, derive-on-client
The producer graph exists only to serve the msgpack and ROS paths; the fast-path viewer/geom bus
already ships raw `{qpos, qvel}` and per-geom poses off `mjData` with ZERO producers
(`AMjManager.cpp:667-713`) and already renders robots. That is the existence proof. Target: the
collector walks the addressing partition and copies id slices straight from `mjData` (exactly
what the existing `entities` block already does, `MjStateCollector.cpp:556-587`). DELETE the
`FCachedArticulation` producer graph, `Producers` weak-ptr array, `DescribeElement` type-dispatch,
and the game-thread producer-cache rebuild.

CORRECTION (user, 2026-08-15): an earlier draft here claimed the server could STOP SENDING obs
because the Python client runs `mj_forward` locally. That is WRONG and inverts the architecture.
UE is the observation AUTHORITY in every mode that produces obs (Live, Direct). The Python
client keeping a local model + `mj_forward` (`client.py:2246-2276`) is at most a per-wrapper
convenience, NOT the contract, and a non-Python consumer would not have it. The design must NOT
be driven by client-side derivation, and must NOT be designed Python-API-first (the Python
client is a thin wrapper). The lean target for observation is a leaner UE-side producer, not a
thinner wire justified by the client re-deriving. The handshake-describes-model-5x and
WorldGeoms-off-the-per-step-path findings still stand on their own (they are UE-side redundancy);
the "derive on the client" reasoning is retracted.

Handshake describes the model up to FIVE times (`RpcDispatcher.cpp:773, 823-877, 885-951,
1027-1054, 795-817`). `raw_actuators`/`raw_joints` exists ONLY because of MJB version skew (the
client cannot load the fork MJB, `:879-884`); fix the skew (see [[project_mujoco_version_skew]])
and the whole block dies. WorldGeoms is recomputed every physics step (`MjStateCollector.cpp:529-551`)
but never encoded to msgpack -- it is ROS-only dead weight on the per-step path.

Producer verdicts: joint qpos/qvel KEEP-AS-CORE (data-ify the container to a flat slice). Sensors
+ actuator ctrl/act/force DATA-IFY (pure id reads; `Semantic`/`TargetJoint` are ROS-only, move
them). Bodies DELETE from wire (client derives). User channels KEEP-AS-SIDE-CHANNEL (real mailbox
feature). Twist: it writes ZERO ctrl and has ZERO msgpack consumers -- it is a control ECHO, not
observation; DELETE from the core wire, keep only for the ROS cmd_vel echo + UI slider.
WorldGeoms/RefPos/Semantic/TargetJoint move behind the ROS `IMjStateConsumer`, off the per-step
`Collect()`.

### Object graph (audit 3): thin the actor, delete the shadow
After control, controllers, and the lease move into the engine, EVERY remaining reason to hand
the bridge an `AMjArticulation` is rendering (debug draw `MjDebugVisualizer.cpp`; cameras
`RpcHandlers_Camera.cpp:210`, `RosCameraInfoProvider.cpp:87`; transform apply
`AMjManager.cpp:936`) plus the state side-channel (`MjStateCollector.cpp:274`). The raw path then
has ZERO actor consumers, so the shadow deletes cleanly. Verdicts: `UMjNodeComponent`
authoring-role KEEP (real editor infra), per-step-producer-role THIN-OUT; `UMjBody` KEEP (render
leaf: pose write-back, mocap, wrench, sleep); runtime element libraries
`UMj{Joint,Sensor,Actuator,Tendon}Runtime` KEEP as-is (already lean id-indexed stateless statics
-- and the control-runtime READ accessors are fine, only the WRITE statics collapse);
`UMjSceneSpec` + `UMj*` spec objects KEEP (authoring, do not persist a runtime mjSpec beside
them); `AMjArticulation` THIN-OUT hard (loses ~130 lines of control API + `ControlSource` +
`bRawShadow` + its collector-key role; keeps Spec, element index, cameras, keyframe/possession,
draw, `ApplyRenderState`; reasonably renamed `AMjModelActor`/`AMjRobot`).

### Modes + transports + lifecycle (audit 4)
Enum -> axis mapping (all verified): `EStepMode::{Live,Direct,Puppet}` -> Clock
`{engine,stepped,external}`; `Auto` is a resolution policy, NOT a value (collapsed to Live at
`MjPhysicsEngine.cpp:1363`; worker never branches on it) -- CONFIRMED. `EMjbRunMode` is PURELY a
RenderSource axis (`Puppet`->mirror, `Direct`->local spanning engine OR stepped clock,
`MjbScene.h:31-33`) -- refutes "same as EStepMode.Direct". `control_mode` -> Drive.
`EControlSource` -> lease (with the behavior-change caveat above). The 3-axis model gains a
genuine capability (a mirror that also steps, inexpressible today) and newly allows two harmless
nonsense combos (external-clock+local-render; Drive=controller on a 0-actuator prop) -- document,
do not guard.

Modes spaghetti: `EStepMode` is branched in ~19 sites / 4 switches and Clock is encoded THREE
redundant ways -- `ResolvedStepMode` atomic (`MjPhysicsEngine.cpp:1133`), `CustomStepHandler`
presence (Direct installs one `:1073`, Puppet installs none), and the strategy object. Renaming
`EStepMode->Clock` is NOT enough; the engine must branch on Clock alone and kill the
Direct-installs-handler / Puppet-installs-nothing asymmetry. That asymmetry is an active hazard:
`FPuppetStepMode` installs no handler yet `SetStepMode(Puppet)` unpauses the worker
(`MjPhysicsEngine.cpp:1368`), so a handler-less worker can free-run `mj_step` on idle wakes
(`:1255-1265`) while the client believes it owns the integrator -- semantic double-integration,
worth a live check. Step strategies `FLive/FDirect/FPuppet` themselves KEEP (clean, stateless);
just de-tangle the `bPublishersPaused` render toggle baked into `OnEnter`.

Transports: the RPC leg is ONE clean interface (`UURLabRpcTransport`, `RpcTransport.h:33`, all
backends funnel to one dispatcher) -- this is the model to copy, do NOT churn it. The Publish leg
is also unified. But RECEIVE is duplicated: the main `SubscribeTransport` (control-in) and the
fast-path `ClientSubscribe`/`ViewerSubscribe` transform bus (`MjbScene::BusTransport`) are
parallel hierarchies -- the fast-path bus duplicates the main transport role. Target: one
`Subscribe` base mirroring the RPC base, and the fast-path transform stream becomes a normal
Publish topic a Renderer subscribes to (3 role-interfaces Rpc/Publish/Subscribe x N backends, no
bespoke bus). Caveat: `ViewerSubscribe` internals unread; verify it is not a genuinely distinct
role before merging.

Lifecycle: the MAIN install/teardown paths are transactional and NOT fragile (build-before-retire,
worker-joined-before-free, shadow retired before model dies) -- do not churn them. But two
INDEPENDENT, memory-unsafe latent races should be banked NOW as standalone fixes, before any
redesign: (RACE 1, HIGH) the registry rebuild does `Empty()`+rebuild with NO `CallbackMutex`
(`InstallCompiledSpec:832-833`, `InstallRawModel:961-962`) while the RPC thread reads unlocked
(`GetArticulation:1433`, `GetAllArticulations:1469`) -> TMap-rehash / TArray-realloc race; (RACE 2)
`MjbScene::BeginDestroy:262-271` frees `Model`/`Data` without `UninstallRawModel` -> UAF vs the
worker if `Teardown` did not run. Both survive/precede the redesign.

### MjbScene god-object
`MjbScene.cpp` is ~2309 lines that reimplement the compiled component renderer
(`BuildBodies/BuildGeoms/BuildMesh/GetOrBuildTexture/GetOrBuildStaticMesh/BuildCameras`); only
`EnsureManager` is shared. The extraction (bus/baker/direct-mode, task #21) still stands, but the
deeper lean move is to SHARE the renderer between the fast path and the compiled path rather than
maintain two.

### Canonical vocabulary (adopt across code + docs)
Driver (the Python authority that steps in puppet/fast-path; retire owner/puppet/client-as-authority);
Renderer (the UE mirror; retire slave/render-slave/puppet); Registry (discovery layer);
Integrator (whoever advances `mj_step` = the Clock holder); ControlLease / Writer (arbitration;
retire EControlSource / "ZMQ vs UI" as an identity); and the addressing unit per the naming
resolution above.

### Consolidated sequencing
STANDALONE CORRECTNESS FIXES FIRST (independent of the rewrite, each its own small PR):
1. Race 1: fence the registry rebuild under `CallbackMutex` (memory-safety, HIGH).
2. Race 2: `MjbScene::BeginDestroy` must `UninstallRawModel` before freeing.
3. Live puppet free-run hazard: verify + fix the handler-less unpaused worker.
4. Move WorldGeoms off the per-step `Collect()` behind the ROS consumer (zero wire change).
5. Narrowed control unification (the doc's "Design C"): resolve raw-ness / Drive ONCE via a
   Touched-masked buffer, WITHOUT yet deleting `EControlSource` or touching qpos-hold. Also
   closes the Live-ignores-control_mode bug.

THEN THE REWRITE:
6. Addressing partition POD (named per the resolution) + the single enrichment side-channel;
   dual-run vs the actor registry, assert equality on qpos/qvel/sensordata only.
7. Observation collapse + derive-on-client (stop sending bodies + kinematic sensors; ROS fields
   to the side-channel).
8. Full control buffer: delete dual slots / ApplyControls / SkipController / OwnedActuatorIds /
   passthrough; data-ify PD; qpos-hold sibling buffer; twist off-core.
9. Raw path builds the partition from `mjModel`; DELETE the shadow + `raw_*` handshake block
   (requires the MJB version skew fixed first).
10. Mode collapse: Clock as the single source of truth (kill the three-way encoding + the
    handler-presence asymmetry); `EMjbRunMode`->RenderSource; `EControlSource`->lease.
11. Transport: unify Subscribe; fast-path bus becomes a Publish topic.
12. Vocabulary rename pass; MjbScene renderer sharing.

### Corrected mode + render-server model (code-grounded, 2026-08-15)
The user corrected the framing: the audits over-abstracted the modes into "3 axes" and
inverted the observation architecture. This subsection is the ground truth from the enums
themselves and supersedes the 3-axis framing where they differ. UE is the observation
AUTHORITY in every mode that produces obs; the client is a thin wrapper; design the CORE UE
code first, not the wire or the Python API. The "weird nuances" below are the cleanup
TARGETS of this audit, not constraints to preserve.

Two enums, and they collide by name (this naming IS half the confusion):
- `EStepMode` (on `UMjPhysicsEngine`, i.e. when UE owns a model; `MjPhysicsEngine.h:65-72`):
  - `Live` -- physics thread free-runs at the model timestep; publishers stream, control
    subscriber writes ctrl. UE owns everything.
  - `Direct` -- physics thread blocks on a step-request queue; RPC writes ctrl, calls
    `mj_step` n times, returns obs. UE owns the integrator; steps on request.
  - `Puppet` -- physics thread blocks on a push-state queue; RPC writes qpos/qvel, calls
    `mj_forward`, returns obs. The CLIENT owns the integrator (MJX/Jax rollouts). This is a
    THIRD, unrelated meaning of "puppet".
  - `Auto` -- starts Live, promotes to Direct/Puppet on first client connect.
- `EMjbRunMode` (on the fast-path renderer `AMjbScene`; `MjbScene.h:26-35`):
  - `Puppet` (default) -- NO physics; mirror an owner's per-geom transform stream over ZMQ.
  - `Direct` -- install this scene's OWN raw mjModel/mjData into the shared `UMjPhysicsEngine`
    and render the stepped state; a full sim a client can drive by RPC.

Mapping to the user's taxonomy: "Live" = `EStepMode::Live`; "Direct" = `EStepMode::Direct`;
"puppet / render server, 2 slightly different ways" = the renderer's `EMjbRunMode::Puppet`
(mirror) vs `EMjbRunMode::Direct` (own sim). "Render server" is NOT a mode -- it is any
renderer PLUS camera streaming (`-URLabFastCameras`). "A render server can slave to any other
mode" = a Puppet-mirror renderer can attach to a Live owner, a Direct owner, or a Python
puppet-client owner (per `docs/fast_path_render.md`: an OWNER is a Python puppet client or a
UE live/direct instance; a RENDERER mirrors it).

Nuances to CLEAN UP (the point of the audit; preserve functionality, kill the confusion):
1. "Puppet" means three different things (`EStepMode::Puppet` push-state / `EMjbRunMode::Puppet`
   transform-mirror / a Python "puppet client" owner) and "Direct" means two
   (`EStepMode::Direct` UE-steps-on-request / `EMjbRunMode::Direct` renderer-owns-a-sim).
   Rename to a single coherent vocabulary (Integrator-owner axis vs Renderer-sourcing axis).
2. `EMjbRunMode::Direct` installs a raw model into the SAME shared `UMjPhysicsEngine` the
   compiled path uses -- so a fast-path renderer in Direct is re-doing what a compiled UE sim
   already does, via the raw-model + shadow path. Two ways to "own a sim in UE".
3. `MjbScene.cpp` (~2300 lines) is a second renderer implementation parallel to the compiled
   component renderer (only `EnsureManager` shared). Two ways to "render a MuJoCo model in UE".
4. `EStepMode::Puppet` (client-integrated push-state) may fold into Direct (both: UE holds the
   model, advances on a client request; one pushes ctrl, the other pushes full state) -- verify
   it is still used before consolidating.

Lean target (functionality that MUST survive, without loss): (a) UE free-runs a sim (Live);
(b) a client steps UE deterministically and reads obs (Direct); (c) a client that integrates
externally pushes state and UE renders/serves it (push-state); (d) a lightweight renderer
mirrors an owner with no physics (transform stream); (e) a renderer runs its own sim and is
RPC-drivable; (f) any renderer can additionally stream cameras (render server); (g) a renderer
can slave to any owner. The redesign should express all seven with ONE mode vocabulary, ONE
"UE owns a sim" path (not compiled-vs-raw+shadow), and ideally ONE renderer implementation
(share the compiled renderer with the fast path) -- not by deleting any of a-g.

### Custom controllers -- direction (no MuJoCo recompile)
User constraint: must NOT require recompiling MuJoCo; the MuJoCo-plugin-style controller "may
be useful" but is uncertain. Grounding from the code: `mjplugin.h` is present in the plugin's
linked MuJoCo headers (`third_party/install/MuJoCo/include/mujoco/mjplugin.h`) and NOTHING in
`Source/` registers a plugin today (no `mjp_register*` usage). The plugin registry is exported
by the already-built `libmujoco`, so a custom actuator plugin authored as UE C++ callbacks and
registered at UE startup (`mjp_registerPlugin`) does NOT require recompiling MuJoCo -- the model
just declares `<plugin>`. So the plugin-controller is feasible under the constraint (exact
registration mechanics to confirm at implementation time). Two orthogonal facts to weigh in the
followup: (1) a plain PD loop is already NATIVE MuJoCo (`<position>`/`<general>` affine
gain/bias, kp/kv), so much of the current `UMjPDController` may need no custom code at all; a
plugin is only for laws native actuators can't express; (2) a plugin runs INSIDE `mj_step`, so
it composes uniformly across all modes (and a slaved renderer sees its effect through the normal
state stream), unlike the current UObject controllers that run around the step on the UE side.
Keyframe (game-thread animation) and twist (teleop input) are NOT control laws and should leave
the "controller" concept regardless of which route the actuator controllers take.

### Open decisions for the user (unchanged + refined)
- NAME of the addressing unit: `FMjAddressable`/`FMjGroup` (retire `FMjEntityRecord`) vs
  `FMjEntity` (then rename the old struct to `FMjPropRecord`). Decide before code.
- SCOPE / first PR: the recommendation from all four audits converges -- ship the standalone
  correctness fixes 1-5 first (they are real bugs, independent, and de-risk the rewrite), and
  treat 6-12 as the staged redesign after. In particular fixes 1 and 2 are memory-safety and
  worth doing regardless of whether the redesign ever proceeds.
