# URLab render roles & modes — authoritative reference

Code-verified against branch `feat/render-server-client` (UE plugin + `URLab_Bridge`
Python client). **The code is ground truth; this doc cites it.** When code and this
doc disagree, the code wins and this doc is the bug.

This exists because the render "modes/roles" are chronically conflated (owner vs
mirror vs render server vs viewer vs peek), which keeps reintroducing bugs. Read the
[Glossary](#glossary) first if any term is ambiguous.

Supersedes the role/mode content in `fast_path_render.md` (old `AMjbScene` /
`EMjbRunMode::Puppet` / "slave" vocabulary) and `RENDER_SERVER_WIP.md` (temporary
audit). The forward-looking design rationale lives in `core_redesign_target.md`; the
enums that doc proposed have now **landed** — this doc describes the landed reality.

---

## 1. The two axes (this is the whole model)

A render instance is described by **one pose-source axis** plus a **composable,
open set of capabilities**. There are no other "modes". New features are added as
capabilities, never as new modes.

### Axis: `EMjPoseSource` — where each frame's pose comes from
`Source/URLab/Public/MuJoCo/Entity/MjPoseSource.h:14-20`

| Value | Meaning | Runs `mj_step`? | Runs `mj_forward`? | Owner-capable? |
|---|---|---|---|---|
| `FreeRun` | UE steps its own `mjData` on its own clock | yes | — | yes |
| `Stepped` | UE steps its own `mjData` only on client step requests | yes | — | yes |
| `StatePushed` | client integrates elsewhere & pushes qpos/qvel; UE runs `mj_forward` | no | yes | yes |
| `Mirror` | apply an owner's streamed transforms; no physics at all | no | no | **no** |

- `AMjManager::StepMode` (owner side) is one of `FreeRun` / `Stepped` / `StatePushed`
  (`AMjManager.h:299`). `Mirror` is **never** a manager step mode.
- `AMjRenderer::RunMode` (fast-path renderer) is only ever `Mirror` (default) or
  `Stepped` (`MjRenderer.h:107`; "any other value renders as Mirror"). It is set in
  exactly one place: `SpawnRenderer(..., bStepped, ...)` →
  `RunMode = bStepped ? Stepped : Mirror` (`MjRenderer.cpp:2647`).
- Owner-ness is **derived**, not a separate axis: `FreeRun`/`Stepped`/`StatePushed`
  have authoritative live state so they can advertise & serve; `Mirror` is downstream
  and cannot.

**Wire strings differ from the C++ names** (deliberately kept for back-compat;
`RpcHandlers_Step.cpp:85-110`, Python `enums.py` `StepMode`):

| wire string (`set_mode`, `enums.py`) | C++ `EMjPoseSource` |
|---|---|
| `live` / `streaming` | `FreeRun` |
| `direct` | `Stepped` |
| `puppet` | `StatePushed` |
| `auto` | `FreeRun` (a promotion policy, not a real value) |

`Mirror` has **no** `set_mode` wire string — it is a renderer-only property set at
spawn, not over the step RPC.

### Axis: `EMjCapability` — composable, open bitflags
`MjPoseSource.h:28-35`

| flag | wire string | backing field on `AMjManager` | gates |
|---|---|---|---|
| `StreamCameras` | `stream_cameras` | `bStreamCameras` (default **true**) | camera RPCs (`RpcHandlers_Camera.cpp:65,266`), advertised caps (`RpcDispatcher.cpp:759`) |
| `AcceptInput` | `accept_input` | `bAcceptInput` (default **true**) | forwarded perturb (`RpcHandlers_Fastpath.cpp:243`), pushed wrenches (`RpcHandlers_Step.cpp:480`) |

- `HasCapability` maps flag → bool field (`AMjManager.cpp:1217-1225`). Both fields are
  `EditAnywhere` UPROPERTYs, **not** set by any command-line flag — they are
  editor/Blueprint-configured. Advertised in the handshake caps array as
  `stream_cameras` / `accept_input`.
- "Render server" = `StreamCameras`; "viewer" = `AcceptInput` — these are **labels for
  capability combinations, not types in the code**.

> **Do not confuse** `EMjCapability::StreamCameras` (`AMjManager::bStreamCameras`, the
> RPC gate) with `AMjRenderer::bEnableCameraStreaming` (set by `-URLabCaps=cameras`,
> formerly `-URLabFastCameras`, which actually *builds & binds* the camera components).
> One authorizes; the other
> constructs.

---

## 2. The buses & transports

### Two publish topics from an owner
Driven per physics step from `AMjManager::FanOutStateSnapshot` (`AMjManager.cpp:722-728`),
gated by `-URLabBroadcastViewers` (`:320`). Python `FastPathOwner` publishes the same
topics from its own step loop.

| topic | payload (msgpack) | producer | consumer |
|---|---|---|---|
| `geoms` | `{f, xpos[3·ngeom], xquat[4·ngeom wxyz], cxpos[3·ncam], cxquat[4·ncam], ...}` **or** `{f, bxpos[3·nbody], bxquat[4·nbody wxyz], ...}` | owner | `AMjRenderer` **Mirror** |
| `viewer` | `{t, qpos[nq], qvel[nv]}` | owner | Python `PeekViewer`, UE manager **viewer role** |

- The `geoms` topic carries **either** per-geom (`xpos`/`xquat`) **or** per-body
  (`bxpos`/`bxquat`) transforms, plus optional camera (`cxpos`/`cxquat`) and free/user
  camera (`ucpos`/`ucfwd`/`ucup`) fields, keyed by frame index `f`.
- **The two owners produce different `geoms` shapes** (see
  [inconsistencies](#code-inconsistencies--traps)): the UE owner sends **per-geom**
  (`AMjManager::PublishGeomFrame`, `AMjManager.cpp:670-720`); the Python
  `FastPathOwner` defaults to **per-body** (`publish_bodies`,
  `fastpath_owner.py:477-492`) but can also send per-geom (`publish_geoms`, `:506-515`).
  `AMjRenderer` accepts both — prefers `bxpos`/`bxquat`, falls back to `xpos`/`xquat`
  (`MjRenderer.cpp:1615-1619`, `2384-2398`).

### Transport selection — by URL scheme (UE) / explicit name (Python)
- **UE**: both client seams test the endpoint scheme. `grpc://…` → the dm_env_rpc gRPC
  backend **iff** the `URLabDmEnvRpc` module bound its hook; every other scheme
  (`tcp://…`) → ZMQ. `UURLabClientSubscribeTransport::Create` (`ClientSubscribeTransport.cpp:11-35`),
  `UURLabRpcClientTransport::Create` (`RpcClientTransport.cpp:11-34`). SHM transports
  exist for co-located RPC/publish (see `concepts/networking.md`).
- **Python**: transport is chosen by an explicit `"zmq"|"grpc"|"shm"` name
  (`transports/__init__.py` `make_transport`), or `--transport`, or
  `OwnerInfo.default_transport()` (prefers `grpc` when advertised, else `zmq`).
  **There is no `grpc://` auto-detection on the Python side** — `grpc://` appears only
  as the advertised bus string.
- Default ports: ZMQ step REQ/REP 5559, state PUB 5555, transform/control bus 5561/5571,
  camera base 5600, viewer bus `ViewerPort`; gRPC (dm_env_rpc) 50051.

### Discovery / registry
Owners write an atomic JSON entry to the shared registry dir
(`$URLAB_REGISTRY_DIR`, else platform cache `…/URLab/registry`) with `role =
"fastpath_owner"`, `capabilities`, `control`, `bus`, `transports`. UE: only when
`-URLabBroadcastViewers` (`InstanceRegistry.cpp:91-102`). Python: `FastPathOwner._write_registry`
(`fastpath_owner.py:243-273`, 10 s heartbeat). Consumers discover via the editor server
browser, `-URLabSourceFind=discover` (formerly `-URLabFastDiscover`), or Python `session.discover_owners`.

---

## 3. The roles

Each role below = a specific (pose-source, capabilities, process) combination.

### 3.1 OWNER
- **What**: the authority that holds a model, steps physics, and streams it. Serves the
  model, streams transforms, accepts perturbations. Any process — a UE instance
  (`FreeRun`/`Stepped`/`StatePushed`) or an external Python process. Feeds **many**
  mirrors/viewers at once (1→N fan-out).
- **Two implementations**:
  - **Python `FastPathOwner`** (`fastpath_owner.py`). Runs MuJoCo physics in the host
    process. Note: it does **not** itself own `mjModel`/`mjData` — the caller passes
    `model, data` into `apply_perturbations`/`publish_*`; it holds the compiled `mjb`
    bytes. Binds ZMQ REP control (`:control_port`, default 5571) + ZMQ PUB bus
    (`:bus_port`, default 5561); optionally a gRPC face (`owner_server.OwnerGrpcServer`,
    50051). Broadcasts `geoms` (per-body default) + `viewer`. Serves `fastpath_hello`
    (returns MJB bytes over ZMQ; xml/mjz + `vfs_assets` over gRPC) and `fastpath_perturb`.
  - **UE `AMjManager` owner** (`StepMode` ∈ {FreeRun,Stepped,StatePushed}, engine holds
    `mjModel`/`mjData`). Broadcasts `geoms` (per-geom) + `viewer` when
    `-URLabBroadcastViewers=1`; advertises `fastpath_owner`; answers `fastpath_hello`
    and RPCs on its bridge.
- **Physics / mjData / mjModel**: yes / yes / yes (Python: physics yes; model+data owned
  by the caller).
- **Sends**: `geoms` + `viewer` buses (ZMQ PUB / gRPC stream); serves model + RPC on the
  control channel. **Receives**: `fastpath_perturb`, step/control RPCs.
- **Selector**: Python — construct `FastPathOwner`. UE — `-URLabBroadcastViewers=1` (+
  the model boot/`set_mode`).
- **Model transport**: mjb (version-locked), xml+`vfs_assets`, or mjz — see [§5](#5-model-transport).

### 3.2 MIRROR / RENDERER (`AMjRenderer`, `RunMode = Mirror`)
- **What**: the fast-path lightweight render scene. One actor per MuJoCo body, per-geom
  mesh components, built from a compiled MJB (`mj_loadModel`), **no physics**. Draws an
  owner's streamed transforms. Process: UE packaged `-game` or editor PIE.
- **Physics / mjData / mjModel**: no / rest-pose only / **yes** (it loads the model to
  build geometry, plus a one-shot `mj_forward` for the rest pose — Mirror is *not*
  model-less).
- **Receives**: the `geoms` bus (`UMjRendererBus` subscribes topic `geoms`,
  `MjRendererBus.cpp:22-26`; applied on the game thread in `Tick` via
  `ApplyBodyTransforms` / `ApplyGeomTransforms`). **Sends**: nothing on the hot path;
  forwards perturbation intent on the control channel (see [§4](#4-perturbation)).
- **Transport**: `BusEndpoint` (ZMQ `tcp://` or gRPC `grpc://` per scheme rule).
- **Selector**: `-URLabDrive=stream:tcp://<endpoint>` (formerly `-URLabFastBus`),
  `-URLabDrive=stream:grpc://host:port` (formerly `-URLabFastGrpcJoin`),
  `-URLabDrive=await` (formerly `-URLabFastServe`), `-URLabSourceFind=discover` (formerly
  `-URLabFastDiscover`), a directed `-URLabDrive=stream:<control>` with no `-URLabModel`
  (formerly `-URLabFastConnect`), the server browser, or `fastpath_load` on a running
  server. Absence of
  `-URLabDrive=sim` (formerly `-URLabFastDirect`) ⇒ Mirror.
- **Capabilities**: `AcceptInput` lets it forward drag intent; `StreamCameras` +
  `-URLabCaps=cameras` (formerly `-URLabFastCameras`) makes it *also* a render server.
- **Manager**: a Mirror that **serves** (has a bus and is not pure subscribe-only) still
  calls `EnsureManager()` — a manager+bridge is needed in BOTH modes for `fastpath_load`
  scene swaps (`MjRenderer.h:534-537`, `MjRenderer.cpp:285-289`). The one exception is a
  pure subscribe-only client (`-URLabDrive=stream:grpc://...`, formerly `-URLabFastGrpcJoin`), which skips it (`:276-288`).

### 3.3 RENDER SERVER (`AMjRenderer` + cameras)
- **What**: a renderer that also builds & streams its cameras. It is **not** a separate
  class — it is any `AMjRenderer` with `bEnableCameraStreaming` on. Two regimes:
  - **Forced / eval** (`-URLabDrive=push`, formerly `-URLabFastForcedOnly`;
    `bForcedRenderOnly=true`): the `fastpath_render` RPC is the *sole* pose driver —
    cameras capture only on request, the transform bus is never connected, primary-view
    render disabled. Exact-fresh, lowest per-request latency. When the client loads the
    model over the wire instead of preloading it, boot empty with `-URLabDrive=await`
    (formerly `-URLabFastServe`) plus `-URLabCaps=serve,cameras` — `await` is
    async-capable and the client's `delay=0` still gives exact forced frames; `push` and
    `await` are mutually exclusive (Drive is one atomic axis). The client loads the model
    and pushes poses via `fastpath_render`.
    Client side: `RenderClient` / `RenderPool` (`render_client.py`, `render_pool.py`) —
    the Python *client* computes poses (`poses_from_mjdata` → `bxpos`/`bxquat`) and pulls
    frames synchronously over gRPC (50051). `RenderPool` fans cameras across many
    instances.
  - **Streaming / viewer** (`-URLabDrive=stream:<endpoint>` instead of `push`; formerly:
    omit `-URLabFastForcedOnly`): the bus is the driver; cameras auto-capture each frame
    and stream over ZMQ/SHM.
- **Physics / mjData / mjModel**: same as its `RunMode` (Mirror: no physics; or a
  `Stepped` server steps in-process).
- **Sends**: camera frames (ZMQ per-camera topics / SHM ring / synchronous
  `fastpath_render` reply). **Receives**: `fastpath_render` (forced) or the `geoms` bus
  (streaming).
- **Selector**: `-URLabCaps=cameras` (build cameras; formerly `-URLabFastCameras`) +
  `-URLabDrive=` — the value sets both regime and pose source: `push` (forced, formerly
  `-URLabFastForcedOnly`), `await` (client-driven), or `stream:<endpoint>` (the bus).
  Capability: `StreamCameras` must be on to publish.

### 3.4 STEPPED RENDERER / in-process sim (`AMjRenderer`, `RunMode = Stepped`)
- **What**: a fast-path scene that installs its own raw `mjModel` into the shared
  `UMjPhysicsEngine` and steps it, so the fast-path instance is a full sim a client
  drives over RPC. Renders the engine's thread-safe snapshot.
- **Physics / mjData / mjModel**: yes / yes / yes.
- **Selector**: `-URLabDrive=sim` (formerly `-URLabFastDirect`) ⇒ `SpawnRenderer(bStepped=true)` ⇒ `RunMode = Stepped`
  (`MjRendererLauncher.cpp:229`, `MjRenderer.cpp:2647`). Requires `EnsureManager()` +
  `Direct.Begin`.
- **Note**: `Stepped` is the same idea as the compiled manager's `Stepped` step mode
  (wire `direct`) — UE owns & steps the model, driven by client requests.

### 3.5 PEEK VIEWER (Python `PeekViewer`)
- **What**: a `mujoco.viewer` passive window bound to an owner's `viewer` bus. For
  eyeballing an owner's sim from Python. Process: Python.
- **Physics / mjData / mjModel**: **no step** / yes / yes. It loads its own model and
  calls `mj_forward` on each received `{t,qpos,qvel}` to place the scene
  (`peek.py:166-179`) — it does not integrate.
- **Receives**: the `viewer` topic (`start_viewer_stream`). **Sends**: raw-wrench
  perturbations back over the control channel (see [§4](#4-perturbation)).
- **Transport**: `--transport zmq` (SUB on the bus) or `grpc` (`subscribe_viewer` stream).
- **Selector**: `session join <target> --mode viewer`, or run `PeekViewer` directly.

### 3.6 UE VIEWER ROLE (`AMjManager`, `bIsViewerRole`)
- **What**: a **manager-based** read-only viewer — distinct from an `AMjRenderer` Mirror.
  The manager holds a compiled model with the engine **paused** (no physics), subscribes
  to the owner's `viewer` bus, applies each `{qpos,qvel}`, and pushes a render snapshot
  (`AMjManager.cpp:344-360`, via `UURLabViewerSubscribeTransport`).
- **Physics / mjData / mjModel**: no (engine paused) / yes / yes.
- **Receives**: the `viewer` topic (`{qpos,qvel}`). **Selector**: `-URLabDrive=stream:<endpoint>`
  (formerly `-URLabStateSource=<endpoint>`) (`BridgeServerConfigUtils.cpp:180-181`).
- **Contrast with Mirror**: a Mirror consumes the `geoms` bus (resolved transforms, no
  model needed to move) with a lightweight per-body render actor; the viewer role
  consumes the `viewer` bus (`qpos/qvel`) through a full manager + compiled model. Same
  intent ("show me the owner's sim"), different bus, different actor, different data.

### 3.7 VR / SPECTATOR VIEWER (`ADroneViewerPawn` + a render consumer)
- **What**: a free-fly "drone" camera pawn (WASD/QE/mouse, Shift boost) that renders
  **nothing itself** — it just flies around whatever render consumer is drawing the sim
  (`DroneViewerPawn.h:12-21`). Process: UE non-headless.
- **Selector**: `-URLabCaps=vr` (formerly `-URLabVrViewer`) — spawns + possesses the drone and suppresses the static
  framing camera (`MjRendererLauncher.cpp:75-81,291-323`; a bus-less VR viewer calls
  `EnsureManager()`, `MjRenderer.cpp:297`).
- **Pairs with**: a `Mirror` renderer (`-URLabDrive=stream:tcp://...` / `stream:grpc://...`,
  formerly `-URLabFastBus`/`-URLabFastGrpcJoin`) for interactive Ctrl+LMB drag (the drag
  path lives on `AMjRenderer`, Mirror only), or a manager viewer role
  (`-URLabDrive=stream:<endpoint>`, formerly `-URLabStateSource`) for a `qpos/qvel` peek.
  Holding Ctrl enters
  "grab mode": free-fly suspends and the cursor drives the mirror's drag-perturb
  (`DroneViewerPawn.h:53-56`).

---

## 4. Perturbation

Two shapes ride the **same** op `fastpath_perturb` on the owner **control** REQ/REP
channel (not the step channel). A third, unrelated path (`entity_xfrc`) rides the `step`
RPC.

### 4.1 Drag INTENT — `{select, active, localpos, refselpos}`
- **Producer**: `AMjRenderer` Mirror. `ProcessMirrorPerturbationInput`
  (`MjRenderer.cpp:2033`, gated `RunMode==Mirror && OwnerControlEndpoint set &&
  !bExternallyDriven`) captures a Ctrl+LMB grab, then `SendPerturbation`
  (`:1939-1973`) packs: `op="fastpath_perturb"`, `select` (body id), `active` (bool),
  `body` (back-compat alias = select), `localpos[3]` (grab point in the body's **local
  MuJoCo frame**, metres), `refselpos[3]` (drag target in **MuJoCo world frame**, metres).
  No `force`/`torque`, no `refquat`. Sent via a short-lived `UURLabRpcClientTransport`
  REQ to `OwnerControlEndpoint` (500 ms).
- **Why intent**: the mirror has no `mjData` (no velocities/mass matrix), so it cannot
  compute the force. The **owner** does.
- **Consumer = Python owner**: `fastpath_owner.py` `fastpath_perturb` (`:311`, gated on
  `accept_input`) → `submit_perturb` (`:343`); `apply_perturbations` (`:380`) sets
  `pert.localpos`/`pert.refselpos`, calls **`mjv_applyPerturbForce`** (`:432`) — the
  mass-scaled, critically-damped spring, same math as `simulate`'s Ctrl-drag — writing
  `data.xfrc_applied`. `active=false` clears the latched force (`:399-404`). gRPC face
  mirrors this (`owner_server.py:111-124`).
- **A UE owner does NOT interpret intent**: `HandleFastpathPerturb`
  (`RpcHandlers_Fastpath.cpp:234`) reads only `body`/`force`/`torque`, so a forwarded
  drag-intent resolves to a zero wrench. A UE owner's own interactive drag is computed
  **in-process** by `UMjPerturbation` (`MjPerturbation.cpp:134-136`, `mjv_applyPerturbForce`
  into `d->xfrc_applied` under `CallbackMutex`) and never crosses the wire. → The
  drag-intent wire shape is effectively a **UE-Mirror → Python-owner** contract.

### 4.2 Raw WRENCH — `{body, force, torque}`
- **Producers**: Python `PeekViewer` (converts a local `mjv` spring into an exact wrench:
  `_maybe_send_perturb` runs `mjv_applyPerturbForce` locally then sends
  `perturb_request(body, force, torque)`, `peek.py:182-194`); or any programmatic Python
  (`submit_perturb_force`). A UE owner over the wire only ever handles this shape.
- **Consumers**: Python owner writes `data.xfrc_applied[body]=wrench` before the drag
  spring (`fastpath_owner.py:390-392`). UE owner `HandleFastpathPerturb` builds
  `Xfrc[6]={force,torque}` → `PhysicsEngine->SubmitWrench` (`RpcHandlers_Fastpath.cpp:293`),
  drained into `d->xfrc_applied` pre-step (`MjPhysicsEngine.cpp:2003-2008`), persisting
  until overwritten/cleared. Gated on `AcceptInput`.
- **Difference**: raw wrench = exact `[force,torque]` applied verbatim; drag intent =
  owner-side geometric spring recomputed each step from the live body pose.

### 4.3 `entity_xfrc` (separate — step RPC, not fastpath)
`URLabEntity.apply_xfrc` buffers a root-body wrench attached to the **`step`** request
(`client.py:1178-1182`); server parses it into `d->xfrc_applied` one-shot, cleared by
`mj_step` (`RpcHandlers_Step.cpp:475-496`). Inert in `StatePushed` (puppet). This is the
compiled/RPC path, distinct from the fast-path control-channel perturbation above.

> **`xfrc_applied` layout trap**: the fast-path code writes **force-first**
> `[fx,fy,fz,tx,ty,tz]` (`RpcHandlers_Fastpath.cpp:293`, `fastpath_owner.py:366-369`),
> but comments at `MjPhysicsEngine.h:568` / `MjBody.cpp:311` describe it torque-first.
> The writes are correct; the comments are misleading.

---

## 5. Model transport

A model reaches a receiver in one of three formats, normalized to an `mjModel`:

| format | how it loads | version behavior |
|---|---|---|
| **mjb** | `mj_loadModelBuffer` directly (fastest) | **version-locked** to the receiver's `libmujoco` (3.11.1); a skewed MJB fails to load |
| **xml + assets** | fed into an `mjSpec` via `mjVFS`, `mj_compile`d in-engine | **version-independent** (receiver compiles with its own lib) |
| **mjz** | archive (root `model.xml`) decoded → `mjSpec` → `mj_compile` in-engine | **version-independent** |

- UE boot flag: `-URLabModel=<path.{mjb,xml,mjz}>` (format from extension; formerly
  `-URLabFastMjb`/`-URLabFastXml`/`-URLabFastMjz`)
  (`MjRendererLauncher.cpp:132-137`; xml/mjz compiled via `MjModelSource::CompileFileToMjb`).
- Over the wire: `fastpath_load` `{format, model(bytes), assets:{name:b64}}`
  (`render_client.py:230-243`); `fastpath_hello` returns `mjb` (ZMQ) or `xml`/`mjz` +
  `vfs_assets` (gRPC, `owner_server.py:142-152`).
- The transform bus (`geoms`) is version-independent regardless (indexed floats), so an
  owner may *step* with a slightly different `mujoco` as long as the MJB it *serves* was
  compiled by the matching lib.

---

## 6. Role × property matrix

| Role | Process | `EMjPoseSource` | Physics | mjData | mjModel | Sends | Receives | Transport | Selector | Caps used |
|---|---|---|---|---|---|---|---|---|---|---|
| **Owner (Python)** | Python | n/a (holds sim) | yes | via caller | mjb bytes | `geoms`(per-body)+`viewer`, model, RPC | `fastpath_perturb`, RPC | ZMQ PUB/REP + gRPC | `FastPathOwner(...)` | serves both |
| **Owner (UE)** | UE game/editor | FreeRun/Stepped/StatePushed | yes¹ | yes | yes | `geoms`(per-geom)+`viewer`, model, RPC | perturb, step/ctrl RPC | ZMQ + gRPC | `-URLabBroadcastViewers=1` | StreamCameras/AcceptInput |
| **Mirror / Renderer** | UE game/PIE | `Mirror` | no | rest only | yes | drag intent (fwd) | `geoms` bus | ZMQ/gRPC (`BusEndpoint`) | `-URLabDrive=stream:tcp://…` / `stream:grpc://…` / `-URLabDrive=await` / browser | AcceptInput (drag) |
| **Render server (forced)** | UE game | `Mirror` (usu.) | no | rest only | yes | camera frames (reply) | `fastpath_render` | gRPC 50051 | `-URLabDrive=await -URLabCaps=serve,cameras` (empty) or `-URLabDrive=push -URLabModel=… -URLabCaps=serve,cameras` (preloaded) | StreamCameras |
| **Render server (streaming)** | UE game | `Mirror` | no | rest only | yes | camera frames (PUB/SHM) | `geoms` bus | ZMQ/SHM + `BusEndpoint` | `-URLabCaps=cameras` + `-URLabDrive=stream:<endpoint>` | StreamCameras |
| **Stepped renderer** | UE game/PIE | `Stepped` | yes | yes | yes | snapshot / camera | step/ctrl RPC | manager bridge | `-URLabDrive=sim` | any |
| **Peek viewer** | Python | n/a (mj_forward only) | no | yes | yes | raw wrench (back) | `viewer` bus | ZMQ SUB / gRPC | `session join --mode viewer` | — |
| **UE viewer role** | UE game | n/a (engine paused) | no | yes | yes | — | `viewer` bus (`qpos/qvel`) | ZMQ/gRPC subscribe | `-URLabDrive=stream:<endpoint>` | — |
| **VR / spectator** | UE non-headless | (of its render consumer) | — | — | — | drag intent (via Mirror) | (renders paired consumer) | — | `-URLabCaps=vr` (+ `-URLabDrive=stream:…`) | AcceptInput |

¹ Python owner: physics runs in-process; `mjModel`/`mjData` are owned by the caller, not `FastPathOwner`.

---

## 7. Which roles pair together

- **Owner → many Mirrors**: one owner (Python or UE) fans the `geoms` bus to N
  `AMjRenderer` Mirror instances (render nodes / camera farms).
- **Mirror + cameras = render server**: add `StreamCameras` + `-URLabCaps=cameras`.
- **VR viewer + Mirror + owner**: `-URLabCaps=vr` drone flies around a Mirror renderer;
  Ctrl+drag forwards intent to the owner (Python owner applies it).
- **Peek viewer + owner**: Python `PeekViewer` subscribes an owner's `viewer` bus.
- **UE viewer role + owner**: `-URLabDrive=stream:<endpoint>` (formerly `-URLabStateSource`) manager subscribes the `viewer` bus.
- **RenderClient/RenderPool + forced render servers**: the Python client steps its own
  sim, pushes poses, and pulls frames from one or many forced render servers over gRPC.

---

## Glossary

Terms that get conflated — pin them down.

- **Owner** — a process with authoritative live sim state that advertises, serves its
  model, streams transforms, and accepts perturbations. Python `FastPathOwner` or a UE
  `AMjManager` in a stepping mode. NOT a UE-only concept.
- **Mirror** — a UE `AMjRenderer` with `RunMode=Mirror`: draws an owner's `geoms`-bus
  transforms with **no physics**. The lightweight fast-path render actor.
- **Renderer** — the class `AMjRenderer`. It is a Mirror *or* a Stepped in-process sim
  depending on `RunMode`. "Renderer" alone is ambiguous — say Mirror or Stepped.
- **Render server** — a Renderer with camera streaming on (`StreamCameras` +
  `-URLabCaps=cameras`). Not a distinct class; a capability combination. Comes in
  forced (`-URLabDrive=push`, or `-URLabDrive=await` for the empty/client-driven case)
  and streaming (`-URLabDrive=stream:<endpoint>`) regimes.
- **Viewer** — a read-only consumer of the `viewer` bus (`{t,qpos,qvel}`): the Python
  `PeekViewer` (peek) or the UE `AMjManager` viewer role (`-URLabDrive=stream:<endpoint>`,
  formerly `-URLabStateSource`). Consumes
  a **different bus** than a Mirror.
- **Peek / PeekViewer** — specifically the Python `mujoco.viewer` window on the `viewer`
  bus. A kind of viewer.
- **VR / spectator viewer** — the `ADroneViewerPawn` free-fly camera. Renders nothing
  itself; pairs with a Mirror or a viewer role.
- **`geoms` bus** — per-geom (`xpos/xquat`) or per-body (`bxpos/bxquat`) resolved
  transforms + camera poses. Feeds Mirrors. Cheap to draw, no model math needed.
- **`viewer` bus** — raw `{t,qpos,qvel}`. Feeds viewers, which reconstruct poses
  (`mj_forward` in Python peek; snapshot push in the UE viewer role).
- **Stepped (`direct`)** — UE owns & steps the model on client request. **StatePushed
  (`puppet`)** — client integrates, UE runs `mj_forward`. **FreeRun (`live`)** — UE
  free-runs its own clock. Note the C++ name ≠ wire string.
- **Capability** — `StreamCameras` / `AcceptInput`; composable flags, not modes.

---

## Code inconsistencies & traps

Places where the code contradicts itself, or naming misleads:

1. **UE owner sends per-geom, Python owner sends per-body** on the same `geoms` topic.
   `AMjManager::PublishGeomFrame` emits `xpos`/`xquat` (per-geom, `AMjManager.cpp:670-720`);
   `FastPathOwner.publish_bodies` emits `bxpos`/`bxquat` (per-body,
   `fastpath_owner.py:477-492`). Not a bug — `AMjRenderer` accepts both — but the header
   calls per-body the "preferred" stream and per-geom "legacy", while the *UE* owner only
   ever produces the "legacy" form.

2. **Wire strings never renamed with the C++ enum.** `EMjPoseSource` is
   `FreeRun/Stepped/StatePushed/Mirror`, but `set_mode`/`enums.py` still use
   `live/direct/puppet/auto` (`RpcHandlers_Step.cpp:85-110`, `enums.py`). Python
   `enums.py` also still ships `StepMode`/`ControlSource` with the old vocabulary.

3. **Two "camera streaming" switches with similar names.**
   `AMjManager::bStreamCameras` (`EMjCapability::StreamCameras`, the RPC authorization
   gate) vs `AMjRenderer::bEnableCameraStreaming` (set by `-URLabCaps=cameras`, formerly
   `-URLabFastCameras`, actually builds the cameras). Both are needed to publish frames;
   neither implies the other.

4. **`xfrc_applied` order comments are wrong.** Fast-path writes force-first
   `[fx,fy,fz,tx,ty,tz]`; `MjPhysicsEngine.h:568` / `MjBody.cpp:311` comments say
   torque-first. Code is correct; comments mislead.

5. **A UE owner silently drops forwarded drag intent.** `HandleFastpathPerturb` reads
   only `body`/`force`/`torque` (`RpcHandlers_Fastpath.cpp:234`), so a Mirror's
   `{select,active,localpos,refselpos}` forwarded to a UE owner yields a **zero wrench**.
   Interactive drag against a UE owner works only in-process (`UMjPerturbation`), never
   over the wire. The drag-intent path is a UE-Mirror → Python-owner contract only.

6. **"Viewer" spans two unrelated consumer mechanisms.** The Python peek and the UE
   `-URLabDrive=stream:<endpoint>` (formerly `-URLabStateSource`) manager both consume the `viewer` bus, but a Mirror (also loosely
   called a "viewer") consumes the `geoms` bus with a completely different actor. Always
   qualify which.

7. **Capabilities have no launch flag.** `bStreamCameras`/`bAcceptInput` default `true`
   and are only editor/Blueprint-settable — a headless server cannot currently turn a
   capability *off* from the command line.
</content>
</invoke>
