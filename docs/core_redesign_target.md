# URLab core UE architecture: target + plan

The end-state we are building toward, and how we get there. This is the FORWARD doc:
what the product looks like when it is clean. The backward doc is
[`RENDER_SERVER_WIP.md`](./RENDER_SERVER_WIP.md), which audits the current code, names the
spaghetti, and cites every file:line this plan rests on. Read this for the shape; read that
for the evidence.

Both are temporary and get deleted when the redesign lands.

Scope: the CORE UE code (how UE owns a sim, renders it, addresses it, and serves it),
because that is the product. The Python client is a thin wrapper and is designed last.
Controllers are deferred (see the WIP doc); this plan touches them only where the single
control write-path is load-bearing.

---

## 1. The one principle

There is exactly one MuJoCo model in a UE instance that has physics: a compiled
`mjModel`/`mjData` owned by one `UMjPhysicsEngine`, indexed by id. Everything else is a ROLE
layered over that one model:

- AUTHORING: the spec/component tree that builds the model. Editor-only, done at compile.
- SIMULATING: advancing `mjData`. One engine, one integrator.
- ADDRESSING: a flat id-slice partition that names which parts a caller talks to.
- RENDERING: actors + meshes + cameras that present the model's state.
- TRANSPORT: RPC / publish / subscribe carrying requests, state, and camera frames.

Every current wart is a place where one of these roles got a second, parallel
implementation (two ways to own a sim, two renderers, a shadow model, a five-times-described
handshake). The target removes the duplicates, not the capabilities.

UE is the authority for anything it owns. A client never re-derives what UE is responsible
for; the wire and the Python API are shaped by what the core produces, never the reverse.

---

## 2. The final product in one picture

```
                         ONE UE INSTANCE
   ┌───────────────────────────────────────────────────────────┐
   │  configured by three independent settings:                 │
   │                                                            │
   │   SimSource   = Owned | External                           │
   │   Drive       = FreeRun | Stepped | StatePushed  (if Owned)│
   │   Cameras     = off | streamed                             │
   │                                                            │
   │  ┌──────────────┐        ┌───────────────┐                 │
   │  │ UMjPhysics   │  state │  ONE Renderer │  frames         │
   │  │ Engine       │───────▶│ (actors,      │────────────────▶│ cameras
   │  │ (mjModel/    │        │  meshes,      │   (render server │  out
   │  │  mjData,     │        │  cameras)     │    when Cameras  │
   │  │  ONE install)│        └───────────────┘    = streamed)  │
   │  └──────┬───────┘                ▲                          │
   │         │ addressing partition   │ mirror stream            │
   │         │ (flat id-slices)       │ (SimSource=External)     │
   └─────────┼────────────────────────┼──────────────────────────┘
             │                        │
        Rpc / Publish            Subscribe (a Publish topic
        (obs, ctrl, step)         from some owner)
```

A UE instance is fully described by three orthogonal settings. That single table replaces
`EStepMode{Live,Direct,Puppet,Auto}` + `EMjbRunMode{Puppet,Direct}` +
`control_mode{raw,ue_controller}` + `EControlSource{ZMQ,UI}` and the three-meanings-of-"Puppet"
/ two-meanings-of-"Direct" tangle documented in the WIP doc.

---

## 3. The mode model, clean

### SimSource: does this instance own a sim?
- `Owned` — this instance's `UMjPhysicsEngine` holds and advances the `mjModel`. It renders
  its own sim and serves obs.
- `External` — no local `mjModel`. State arrives from an owner over the wire; the instance is
  a pure renderer (a mirror). This is today's fast-path puppet renderer.

SimSource alone decides what the renderer draws: an Owned instance draws its own sim's
snapshot; an External instance draws the mirrored stream. There is no separate "render
source" axis — render source IS SimSource.

### Drive: how an Owned sim is advanced (only meaningful when SimSource=Owned)
- `FreeRun` — UE advances `mj_step` in real time on its own thread; publishes state,
  subscribes ctrl. (was `EStepMode::Live`)
- `Stepped` — UE advances only on client step requests, writing ctrl and returning obs. UE
  owns the integrator. (was `EStepMode::Direct` AND the fast-path `EMjbRunMode::Direct` — these
  are THE SAME thing and unify here.)
- `StatePushed` — UE owns the `mjModel` but the client owns the integrator: the client pushes
  qpos/qvel, UE runs `mj_forward` and serves obs (MJX/Jax rollouts). (was `EStepMode::Puppet`.)
  Kept only if still used; otherwise folds into Stepped.

### Cameras: off or streamed
Any instance (Owned or External) can additionally spawn the model's cameras and stream their
frames over the transport. "Render server" is not a mode; it is `Cameras = streamed`.

### How this expresses every capability, with no duplication
- UE free-runs a sim: `Owned + FreeRun`.
- Client steps UE deterministically and reads obs: `Owned + Stepped`.
- Client integrates externally, UE renders/serves: `Owned + StatePushed`.
- Lightweight renderer mirrors an owner, no physics: `External`.
- Renderer runs its own sim and is RPC-drivable: `Owned + Stepped` (same as the second case —
  the duplication is gone).
- Any renderer also streams cameras: `Cameras = streamed`.
- A renderer slaves to any owner: an `External` instance points at any owner (Owned-anything,
  or a Python owner).

`Auto` (start FreeRun, promote on connect) stays a launch policy, not a mode value.

---

## 4. One "UE owns a sim" path

Today UE can own and step a model two ways: the compiled path (`AMjArticulation` actors + full
component graph) and the fast path (`AMjbScene` installs a raw `mjModel` into the shared engine
and stands up a shadow so the RPC layer can address it). The target has ONE path:

- `UMjPhysicsEngine` owns the compiled `mjModel`/`mjData`, installed once, whether the source
  was an authored spec or a wire MJB.
- Addressing is a flat partition (`FMjAddressable` / `FMjGroup`, name TBD, see WIP doc)
  built FROM the `mjModel` by name-prefix, identical for both origins. No shadow, no
  fabricated per-joint components, no second model description.
- The render actor is a presentation layer on top, never the thing the bridge addresses.

The shadow, the `raw_actuators`/`raw_joints` handshake block, and `bRawShadow` all disappear.
The prerequisite is fixing the MJB version skew (see `project_mujoco_version_skew`) so a wire
MJB loads directly instead of being re-described.

---

## 5. One renderer

Today `MjbScene.cpp` (~2300 lines) is a second renderer (lightweight per-body actors straight
from an MJB, no `mjData`) parallel to the compiled component renderer. The target is ONE
renderer that:

- builds its actor/mesh/camera representation from a compiled `mjModel`,
- is driven either by the local sim's thread-safe snapshot (SimSource=Owned) or by a
  subscribed transform stream (SimSource=External),
- exposes the "lightweight, one actor per body, no per-element component graph" property as a
  fidelity/perf option, not as a separate class.

This is the single biggest lean win and it gates the mode collapse (once there is one
own-a-sim path and one renderer, `EMjbRunMode` has nothing left to encode). The god-object
extraction already tracked as task #21 (asset baker / transport bus / direct mode out of
`MjbScene`) is the first cut of this, but the deeper move is sharing the compiled renderer
rather than maintaining two.

---

## 6. Addressing and observation

- One flat partition built from `mjModel` by prefix; the addressing unit carries id slices
  only (bodies/joints/actuators/sensors).
- UE produces obs by walking the partition and copying id slices straight from `mjData` on the
  physics thread — the pattern the existing `entities` block already uses. No per-element
  producer graph, no game-thread producer-cache rebuild.
- Non-id-sliceable data attaches through ONE explicit enrichment side-channel keyed by the
  addressable's name: compiled camera/controller handshake metadata, user channels, and the
  ROS-only fields (semantic tags, world geometry). This side-channel is off the per-step core
  path and is consumed by the ROS layer and opt-in clients.
- UE is the obs authority. The redundant handshake descriptions collapse to one. The client's
  local model mirror is a convenience of one wrapper, never a design assumption.

---

## 7. Control (kept minimal here; full detail in the WIP doc)

The load-bearing core property: one engine-owned control store and one pre-step write path
into `d->ctrl`, replacing today's four-plus uncoordinated writers and the dual staging slots.
Drive-per-addressable (direct vs a control law) is resolved once, at drain. Keyframe qpos-hold
gets a sibling state-injection buffer. The controller model itself (native MuJoCo actuators for
PD, an optional UE-C++ MuJoCo plugin for custom laws that needs no MuJoCo recompile, and
evicting keyframe/twist from "controller") is deferred per current priority.

---

## 8. Transport

Three role interfaces over N backends, mirroring the already-clean RPC layer:
- Rpc (request/reply): hello, step, push-state, perturb, scene ops. One dispatcher, backends
  ZMQ / SHM / ROS.
- Publish (state out, camera frames out): one base, backends ZMQ / SHM / ROS.
- Subscribe (state in for a mirror, ctrl in for FreeRun): ONE base. The fast-path transform bus
  becomes a normal Publish topic that an `External` renderer subscribes to, retiring the bespoke
  bus and the parallel `ClientSubscribe`/`ViewerSubscribe` hierarchies.

Do not churn the RPC base; it is the model the others copy.

---

## 9. Vocabulary (adopt across code + docs)

- Driver — the authority that advances a sim and drives renderers (retire owner / puppet /
  client-as-authority).
- Renderer — a UE instance presenting a model (retire slave / render slave / puppet).
- Registry — the discovery layer (registry JSON + hello).
- Integrator — whoever advances `mj_step`.
- ControlLease / Writer — write arbitration (retire `EControlSource` as an identity).
- Addressable (or Entity, name TBD) — the id-slice unit; resolve the collision with the
  existing bodies-only `FMjEntityRecord` first.

---

## 10. What each current wart becomes

| Today | Target |
|---|---|
| `EStepMode{Live,Direct,Puppet,Auto}` | `Drive{FreeRun,Stepped,StatePushed}` (Auto = policy) |
| `EMjbRunMode{Puppet,Direct}` | folded into `SimSource{External,Owned}` |
| compiled path + fast-path raw install + shadow | ONE `UMjPhysicsEngine` install + one partition |
| `MjbScene` second renderer (~2300 lines) | ONE renderer, lightweight as a fidelity option |
| `raw_actuators`/`raw_joints` + `bRawShadow` | deleted (needs MJB version-skew fix first) |
| per-element producer cache | walk the partition, index `mjData` + one side-channel |
| four-plus `d->ctrl` writers | one engine-owned store + one pre-step drain |
| `EControlSource` 2-slot selector | write lease; one setpoint per id |
| main Subscribe + fast-path bus + Client/ViewerSubscribe | one Subscribe base; bus = a Publish topic |
| overloaded owner/puppet/slave/server terms | Driver / Renderer / Registry / Integrator |

---

## 11. Functionality that MUST survive (no loss)

Every one of these keeps working through the redesign; the redesign only removes the second
implementation of each:
a. UE free-runs a sim in real time.
b. A client steps UE deterministically and reads obs.
c. A client integrates externally, pushes state, UE renders and serves it.
d. A lightweight renderer mirrors an owner with no physics.
e. A renderer runs its own sim and is RPC-drivable (= b).
f. Any renderer also streams cameras (render server).
g. A renderer slaves to any owner, cross-machine.

---

## 12. Plan / sequencing

Standalone correctness fixes FIRST (independent of the redesign, worth landing regardless;
detail + file:line in the WIP doc):
1. Fence the addressing-registry rebuild under `CallbackMutex` (memory-safety race).
2. `MjbScene::BeginDestroy` must uninstall the raw model before freeing it (UAF).
3. Verify and fix the puppet free-run hazard (handler-less worker double-integrating).
4. Move world-geometry collection off the per-step path (zero wire change).

Then the staged core redesign, each phase compiling on its own:
5. Addressing partition built from `mjModel` (compiled path), dual-run against the actor
   registry, asserting equality on qpos/qvel/sensordata.
6. Observation: walk the partition + index `mjData`; the enrichment side-channel; collapse the
   handshake to one description.
7. One "UE owns a sim" path: build the partition on the wire-MJB path too, delete the shadow +
   raw handshake block (needs the MJB version-skew fix).
8. One renderer: share the compiled renderer with the fast path; lightweight as an option;
   finish the `MjbScene` god-object extraction into it.
9. Mode collapse: `Drive` + `SimSource` + `Cameras`; single source of truth for the integrator
   state; retire `EStepMode` / `EMjbRunMode` / `EControlSource`.
10. Transport: one Subscribe base; fast-path bus becomes a Publish topic.
11. Vocabulary rename pass across code + docs.
12. Control write-path unification (single store + drain); controller model as a later,
    separate effort.

---

## 13. What we explicitly do NOT touch

- The RPC transport base (already clean; it is the pattern to copy).
- The transactional install/teardown ordering of `UMjPhysicsEngine` (correct; only add the
  registry fence).
- The authoring spec/component tree and the editor infrastructure on it.
- The lightweight per-body rendering technique itself (it stays; it just stops being a second
  renderer class).
- The one-collect / many-consumers state fan-out and the SI-unit IR convention.

---

## 14. Open decisions

- Name of the addressable unit (`FMjAddressable`/`FMjGroup` vs `FMjEntity` + rename the old
  struct). Blocks the addressing phase.
- Whether `StatePushed` (client-integrated) is still used; if not, fold it into Stepped.
- Whether the two renderers merge fully in one pass or the fast renderer is kept as a fidelity
  profile of the shared renderer. Recommend investigating this first — it is the largest lean
  win and it gates the mode collapse.
