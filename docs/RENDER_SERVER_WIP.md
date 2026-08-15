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
