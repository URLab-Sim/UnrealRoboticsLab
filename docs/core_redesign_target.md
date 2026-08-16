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

   SimSource    = Owned | Mirror
   Drive        = FreeRun | Stepped | StatePushed      (Owned only)
   Capabilities = any combination, OPEN set:
                    stream-cameras (model cams and/or own view),
                    accept-input (xfrc / wrench / drag / requests), ...

   Owned:                                Mirror:
   ┌──────────────┐   ┌────────────┐     ┌────────────┐
   │ UMjPhysics   │   │    ONE     │     │    ONE     │  draws a transform
   │ Engine       │──▶│  Renderer  │     │  Renderer  │◀─ stream, no physics,
   │ mjModel/Data │   │ (from the  │     │ (from the  │   no mj_forward
   │ ONE install  │   │  mjModel)  │     │  mjModel)  │
   └──────┬───────┘   └────────────┘     └────────────┘
          │ addressing partition                │
          │ (flat id-slices)                    │
     Rpc / Publish                         Subscribe (an owner's
     (obs, ctrl, step, perturb)             Publish topic)

   "render server" / "viewer" = shorthand for common capability
   combinations, NOT types in the code.
```

A UE instance is described by two axes plus a freely-composable, open set of capabilities. That
replaces `EStepMode{Live,Direct,Puppet,Auto}` + `EMjbRunMode{Puppet,Direct}` +
`control_mode{raw,ue_controller}` + `EControlSource{ZMQ,UI}` and the three-meanings-of-"Puppet"
/ two-meanings-of-"Direct" tangle documented in the WIP doc. Crucially, we do NOT replace those
enums with a new set of named bundles — "render server" and "viewer" are just labels for common
capability combinations, so new features are added AS capabilities, never as new modes (see the
design guard in section 3).

---

## 3. The mode model, clean

Two axes plus optional overlays.

### Axis 1 — SimSource: is there a local `mjModel`?
- `Owned` — this instance's `UMjPhysicsEngine` holds a real `mjModel`/`mjData`. It has the full
  simulation state locally: contacts, sensors, derived quantities, full obs.
- `Mirror` — no local `mjModel`. The instance receives already-resolved per-geom transforms
  from an owner and draws them. No `mj_step`, no `mj_forward`. It has geometry poses only. This
  is the cheap render / view path (today's fast-path puppet renderer).

SimSource decides both what the renderer draws and what data is available locally. Render
source IS SimSource; there is no separate render-source axis.

### Axis 2 — Drive: how an Owned sim advances (Owned only)
- `FreeRun` — UE integrates `mj_step` in real time on its own thread; publishes state,
  subscribes ctrl. (was `EStepMode::Live`)
- `Stepped` — UE integrates only on client step requests; the client sends ctrl, UE `mj_step`s
  n and returns obs. UE owns the integrator. (was `EStepMode::Direct` AND the fast-path
  `EMjbRunMode::Direct` — these are the same thing and unify here.)
- `StatePushed` — the CLIENT owns the integrator and `mj_step`s externally (e.g. MJX/Jax);
  it pushes qpos/qvel; UE runs `mj_forward` to reconstruct the full `mjData` (contacts, sensors,
  derived, xpos), renders, and returns obs. UE holds the model but does not integrate. (was
  `EStepMode::Puppet`.)

NOTE — the cheap sibling of `StatePushed` is NOT a Drive. If the client pushes ALREADY-RESOLVED
transforms and UE just draws them (no `mj_forward`, no local data), that is `SimSource = Mirror`,
because a Mirror has no local `mjModel` to drive. So the four compute profiles are: `FreeRun`
(mj_step, continuous), `Stepped` (mj_step, on request), `StatePushed` (mj_forward on pushed
state), and `Mirror` (no compute at all). The first three are Owned Drives; the fourth is the
other SimSource. See "Mirror vs StatePushed" below.

### Mirror vs StatePushed — keep both; they are different trades
Both have an external integrator, but they pay different costs and expose different data:
- `Mirror` receives RESOLVED transforms and pays nothing beyond drawing. No contacts, no
  sensors, no derived data. This is the pure render-server / viewer path — a render server or a
  VR viewer must NOT be forced to pay a per-update `mj_forward`.
- `Owned + StatePushed` receives qpos/qvel and pays one `mj_forward` per update to rebuild the
  full `mjData` locally, which gives the local instance the diverse data a mirror cannot see
  (contacts, derived quantities, local sensor reads, local queries). Keep this for clients that
  want that richness; it is a deliberate cost, not the default render path.

So the earlier draft was wrong to blur these: the render server is `Mirror` (cheap), and
`StatePushed` is a separate, heavier Owned mode you opt into for local data.

### Capabilities: orthogonal, composable, an OPEN set (the anti-lock-in guard)
On top of the two axes, an instance carries any COMBINATION of capabilities. These are not
types, not modes, and not mutually exclusive; they compose freely, and the set is meant to GROW
without ever adding a new mode:
- `stream-cameras` — publish frames from any cameras the instance has: the model's cameras
  AND/OR the instance's OWN view camera (e.g. a human or VR client streaming what it sees).
  Works on Owned or Mirror.
- `accept-input` — perturbations (xfrc, wrench, drag like MuJoCo `simulate`) and an extensible
  request set. Applied locally if Owned; forwarded to the owner if Mirror. Rides the fast-path
  RPC (`fastpath_perturb` is the seed).
- ...new capabilities slot in HERE, as more of the same, never as a new mode.

The GUARD (this is the point of the whole redesign, do not violate it): "render server" and
"viewer" are SHORTHAND for common combinations, not categories in the code. A headless camera
node is `Mirror` + `stream-cameras`. A VR client is `Mirror` + `accept-input`. A VR client that
ALSO streams its own view is `Mirror` + `accept-input` + `stream-cameras` — nothing special,
just another combination. Because these are capabilities and not bundles, wanting "a viewer that
also streams a camera" needs ZERO new code paths; it is already expressible. If a future feature
tempts us to add a named mode or a "type" enum, that is the smell we are removing — add it as a
capability instead. Keep the axes tiny (SimSource, Drive) and let the capability set carry the
growth.

### The owner role is uniform and symmetric
Any `Owned` instance (any Drive) can act as an OWNER: advertise in the registry, serve its
model, stream transforms, and accept perturbations. Any `Mirror` instance can attach to ANY
owner, whether that owner is a UE instance or an external Python client. So any UE instance can
mirror / render-serve / view any other UE instance or any Python owner, and vice versa.
Owner-ness is not a special mode; it is what an Owned instance exposes.

### How this expresses every capability, with no duplication
- UE free-runs a sim: `Owned + FreeRun`.
- Client steps UE deterministically and reads obs: `Owned + Stepped`.
- Client integrates externally, UE reconstructs + serves local data: `Owned + StatePushed`.
- Lightweight renderer mirrors an owner, no physics: `Mirror`.
- Renderer runs its own sim and is RPC-drivable: `Owned + Stepped` (the duplication is gone).
- Camera farm node: `Mirror` + `stream-cameras`.
- VR viewer: `Mirror` + `accept-input`.
- VR viewer that also streams its own view: `Mirror` + `accept-input` + `stream-cameras` — no
  new code path, just another combination.
- Any instance slaves to any owner, UE or Python, cross-machine.

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

### Model source over the wire: mjb, xml+assets, or mjz
Today the fast path pulls only a prebuilt MJB. The target accepts three source formats,
normalized to an `mjModel` by the RECEIVING instance:
- `mjb` — load directly (`mj_loadModelBuffer`). Fastest, but version-locked to the receiver's
  `libmujoco`.
- `xml + asset bytes` — feed into an `mjSpec` via `mjVFS`, `mj_compile` to an `mjModel`.
- `mjz` — MuJoCo's archive format, read straight into a spec, then compile to an `mjModel`.

Key benefit: when an owner ships `xml` or `mjz` instead of a prebuilt MJB, the RECEIVER compiles
with its OWN `libmujoco`, so there is NO version lock and NO skew. That removes the ROOT CAUSE of
the `raw_actuators`/`raw_joints` re-description (which exists today only because a version-skewed
MJB cannot be loaded downstream). The MJB path stays as the fast option when versions match.
`mjz` support is the one net-new piece to verify (whether the codec is compiled into the linked
lib). This is the SAME normalize-to-`mjModel` front end as the `load_model()` RPC discussed in
the WIP doc; they must be ONE mechanism, not two. UE runtime already links
`mj_parseXMLString` / `mj_compile` / `mjVFS`, so xml→mjModel is wiring existing calls.

---

## 5. One renderer (concrete, because the double-build is the real objection)

The honest objection (raised while iterating): the fast path already builds render actors FROM
the `mjModel`. The compiled/authoring path builds an `AMjArticulation` Blueprint component tree,
uses it to build the spec, compiles the `mjModel` — and if we THEN also build render actors from
that `mjModel`, we have instanced the geometry TWICE (the heavy authoring tree AND the
mjModel-derived tree). That is expensive and is a fair reason to distrust a naive "everything
renders from the mjModel".

Proposed resolution — separate authoring-time from render-time:
- EDITOR / authoring: the `AMjArticulation` component tree stays as the authoring + placement +
  spec-build surface. Unchanged. It is what a user edits, and it is what produces the spec.
- RUNTIME (a sim is installed, or a mirror is streaming): ONE renderer builds from the
  `mjModel`, for BOTH the compiled and the fast paths. The heavy authoring component tree is
  NOT instantiated as a render representation at runtime.

Why this does not double-instance, and likely gets FASTER:
- At runtime you build from the `mjModel` ONCE. You never stand up the authoring tree as a
  second render representation; the compiled path stops rendering its Blueprint tree at play.
- The authoring Blueprint tree is the known-EXPENSIVE object (huge XML -> huge BP -> slow to
  instantiate, see `project_blueprint_edit_lag_unresolved`). The mjModel-derived lightweight
  renderer (one actor per body, no per-element component graph) is CHEAPER. So moving
  compiled-path runtime rendering onto it should REDUCE cost: the "build from mjModel" work
  replaces the heavier BP-tree instantiation, it is not added on top.

What must be validated before committing (this is a proposal; we iterate):
1. FIDELITY PARITY — the mjModel-derived renderer must resolve the SAME imported meshes/materials
   the authoring path used, so the compiled scene looks identical. The compiled path imports
   those assets and the `mjModel` references them by name, so this should hold, but it is the
   first thing to prove.
2. AUTHORING-ONLY VISUALS — anything a user attaches in UE that is not in the `mjModel` (debug
   widgets, decorative components) needs an explicit bridge or is declared editor-only.
3. ONE-TIME PLAY COST — building from the `mjModel` at play-start is a cost the compiled path
   does not pay today; it must be no worse than the BP-tree instantiation it replaces (expected
   better, per the point above).

Net: "one renderer" means the `mjModel` is the SINGLE runtime render source for both paths, and
the authoring tree is editor-only. This is the biggest lean win and it gates the mode collapse
(with one own-a-sim path and one renderer, `EMjbRunMode` has nothing left to encode). It needs
the fidelity-parity proof (item 1) before we commit — that proof is the recommended first probe.
The god-object extraction tracked as task #21 (asset baker / transport bus / direct mode out of
`MjbScene`) is a first cut, but the deeper move is this shared runtime renderer.

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
| `EMjbRunMode{Puppet,Direct}` | folded into `SimSource{Mirror,Owned}` |
| compiled path + fast-path raw install + shadow | ONE `UMjPhysicsEngine` install + one partition |
| `MjbScene` second renderer (~2300 lines) | ONE runtime renderer from `mjModel`; authoring tree is editor-only |
| render-server = a bundled mode | two independent overlays: cameras out / interactive input |
| fast path takes only MJB | receiver normalizes `{mjb, xml+assets, mjz}` to `mjModel`; xml/mjz kills version skew |
| `raw_actuators`/`raw_joints` + `bRawShadow` | deleted (xml/mjz source removes the root cause) |
| per-element producer cache | walk the partition, index `mjData` + one side-channel |
| four-plus `d->ctrl` writers | one engine-owned store + one pre-step drain |
| `EControlSource` 2-slot selector | write lease; one setpoint per id |
| main Subscribe + fast-path bus + Client/ViewerSubscribe | one Subscribe base; bus = a Publish topic |
| overloaded owner/puppet/slave/server terms | Driver / Renderer / Registry / Integrator / Viewer |

---

## 11. Functionality that MUST survive (no loss)

Every one of these keeps working through the redesign; the redesign only removes the second
implementation of each:
a. UE free-runs a sim in real time.
b. A client steps UE deterministically and reads obs.
c. A client integrates externally, pushes state, UE runs `mj_forward` and serves the full local
   data (contacts, derived, sensors) — the deliberately heavier path, NOT the pure render path.
d. A lightweight renderer mirrors an owner with no physics and no `mj_forward` (the cheap path).
e. A renderer runs its own sim and is RPC-drivable (= b).
f. Any instance streams cameras — its model's cameras and/or its own view — as a capability.
g. Any instance slaves to any owner, UE or Python, cross-machine; and any Owned instance can BE
   an owner for others.
h. A viewer (e.g. VR) mirrors an owner and sends interactive input back (xfrc / wrench / drag /
   requests) as a capability; it can ALSO stream its own view at the same time — capabilities
   compose, no new mode.

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

## 14. Open decisions / iteration log

- Name of the addressable unit (`FMjAddressable`/`FMjGroup` vs `FMjEntity` + rename the old
  struct). Blocks the addressing phase.
- `StatePushed` is confirmed KEEP (gives the local instance contacts + derived data a mirror
  can't see); it is distinct from the cheap `Mirror` path and must not be collapsed into it.
- The unified renderer's FIDELITY PARITY (section 5, item 1) is the recommended FIRST probe:
  prove the mjModel-derived runtime renderer resolves the same imported meshes/materials as the
  authoring tree, so the compiled scene looks identical and the BP tree can go editor-only. This
  gates the whole mode collapse.
- `mjz` codec availability in the linked `libmujoco` — verify before relying on the mjz source
  format (mjb and xml+assets are already known-feasible).
- The `load_model()` RPC and the fast-path model source must be ONE normalize-to-`mjModel`
  mechanism, not two.

### Resolved during iteration (2026-08-15)
- Mirror (cheap, no `mj_forward`) and `StatePushed` (Owned, pays `mj_forward` for local data)
  are DIFFERENT and both stay. An earlier draft blurred them.
- "Render server" (cameras out) and "viewer" (interactive input in) are INDEPENDENT overlays,
  not one bundle; a VR viewer is `Mirror` + input, no cameras.
- Owner-ness is uniform: any Owned UE instance can be an owner; any Mirror can attach to any
  owner (UE or Python).
- Fast-path model source broadens from MJB-only to `{mjb, xml+assets, mjz}`; xml/mjz compiled by
  the receiver removes the version-skew that forces the `raw_*` re-description today.
