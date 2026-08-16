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
        (ALWAYS loads an mjModel to build its geometry from)

   PoseSource   = FreeRun | Stepped | StatePushed | Mirror
   Capabilities = any combination, OPEN set:
                    stream-cameras (model cams and/or own view),
                    accept-input (xfrc / wrench / drag / requests), ...

   ┌──────────────────────────────────────────────────────┐
   │  mjModel  ──build──▶  geometry  ──▶  ONE Renderer      │
   │                                                        │
   │  each frame the render pose comes from ONE PoseSource: │
   │    FreeRun      UE mj_step, own clock                  │
   │    Stepped      UE mj_step, on client request          │
   │    StatePushed  client pushes qpos/qvel; UE mj_forward │
   │                 (gets contacts / sensors / derived)    │
   │    Mirror       apply owner's streamed transforms      │
   │                 (no step, no forward)                  │
   └──────────────────────────────────────────────────────┘

   "render server" / "viewer" = shorthand for capability
   combinations, NOT types in the code.
```

A UE instance is described by ONE axis (PoseSource) plus a freely-composable, open set of
capabilities. That replaces `EStepMode{Live,Direct,Puppet,Auto}` + `EMjbRunMode{Puppet,Direct}`
+ `control_mode{raw,ue_controller}` + `EControlSource{ZMQ,UI}` and the
three-meanings-of-"Puppet" / two-meanings-of-"Direct" tangle documented in the WIP doc.
Crucially, we do NOT replace those enums with a new set of named bundles — "render server" and
"viewer" are just labels for common capability combinations, so new features are added AS
capabilities, never as new modes (see the design guard in section 3).

---

## 3. The mode model, clean

Ground truth from the code (this corrects an earlier draft): EVERY rendering instance loads an
`mjModel`. It needs one to build its geometry — the fast-path render server does
`mj_loadModelBuffer` on the wire MJB and builds actors from it (`MjbScene.cpp:471`), then a
one-shot `mj_forward` for the rest pose (`:486`). So "has a model vs not" is NOT the axis, and
Mirror is not model-less. The one axis that matters is where each frame's render POSE comes
from.

### The one axis — PoseSource
- `FreeRun` — UE steps its own `mjData` in real time on its own thread; publishes state,
  subscribes ctrl. (was `EStepMode::Live`)
- `Stepped` — UE steps its own `mjData` only on client step requests; client sends ctrl, UE
  `mj_step`s n, returns obs. (was `EStepMode::Direct`, AND the fast renderer's own-sim mode
  `EMjbRunMode::Direct` — the same thing, unified.)
- `StatePushed` — the client integrates externally (e.g. MJX/Jax) and pushes qpos/qvel; UE runs
  `mj_forward` to reconstruct the full `mjData` (contacts, sensors, derived), renders, returns
  obs. UE holds the model, does not integrate. (was `EStepMode::Puppet`.)
- `Mirror` — UE applies per-body/geom world transforms streamed from an owner and draws them; no
  `mj_step`, no `mj_forward`. It still HAS the `mjModel` (that is how it built the geometry) and
  a rest-pose `mjData`; it just does not advance physics. (was the fast-path
  `EMjbRunMode::Puppet` / render server.)

### Why one axis, not the "Owned vs Mirror + Drive" split I had wrong
Your instinct was right: `StatePushed` and `Mirror` are SIBLINGS — both take their state from
OUTSIDE the instance. The old two-axis split hid that and wrongly implied Mirror has no model.
They are just two points on the same axis. All four differ only in how much UE computes per
frame and therefore what local data exists:

| PoseSource   | what arrives          | UE computes | local data available       |
| ------------ | --------------------- | ----------- | -------------------------- |
| FreeRun      | (nothing; UE drives)  | mj_step     | everything                 |
| Stepped      | ctrl + step count     | mj_step     | everything                 |
| StatePushed  | qpos/qvel             | mj_forward  | contacts, sensors, derived |
| Mirror       | resolved transforms   | nothing     | geometry poses only        |

`FreeRun` / `Stepped` / `StatePushed` have authoritative live sim state, so they can act as an
OWNER (advertise, serve the model, stream transforms). `Mirror` is downstream and cannot. That
owner-ness is DERIVED from PoseSource, not a separate axis.

Verified against code (2026-08-15, anchor corrected by audit): Mirror applies `bxpos/bxquat` (or
legacy `xpos/xquat`) streamed transforms with no step (`MjbScene.cpp:1756/1766`); the fast
renderer's own-sim Direct renders the engine snapshot (`:1710`); StatePushed does `mj_setState` +
`mj_forward` on the pushed state at `MjPhysicsEngine.cpp:1218-1219` (driven by `bPendingRestore`),
plus `FPuppetStepMode::HandleStep`'s inline `mj_forward` (`RpcHandlers_Step.cpp:~884`). (An earlier
draft mis-cited `MjPhysicsEngine.cpp:560-571`, which is `RestoreState()`, a migration helper.) We
are designing a NEW plan, not ratifying these names — the code is evidence, not the target.

### Mirror vs StatePushed — keep both; different cost, different data
- `Mirror` receives already-resolved transforms and pays nothing beyond drawing. No contacts,
  no sensors, no derived data. The cheap render / view path — a render server or VR viewer must
  NOT be forced to pay a per-update `mj_forward`.
- `StatePushed` receives qpos/qvel and pays one `mj_forward` per update to rebuild the full
  `mjData` locally, giving contacts and derived quantities a Mirror cannot see. A deliberate
  cost you opt into, not the default render path.

Both load the same `mjModel` over the wire; the difference is only what is pushed each frame
(resolved transforms vs qpos/qvel) and whether UE runs `mj_forward`.

### Capabilities: orthogonal, composable, an OPEN set (the anti-lock-in guard)
On top of the one axis, an instance carries any COMBINATION of capabilities. These are not
types, not modes, and not mutually exclusive; they compose freely, and the set is meant to GROW
without ever adding a new mode:
- `stream-cameras` — publish frames from any cameras the instance has: the model's cameras
  AND/OR the instance's OWN view camera (e.g. a human or VR client streaming what it sees).
  Works on any PoseSource.
- `accept-input` — perturbations (xfrc, wrench, drag like MuJoCo `simulate`) and an extensible
  request set. Applied locally if the instance runs its own sim (FreeRun/Stepped/StatePushed);
  forwarded to the owner if Mirror. Rides the fast-path RPC (`fastpath_perturb` is the seed).
- ...new capabilities slot in HERE, as more of the same, never as a new mode.

The GUARD (this is the point of the whole redesign, do not violate it): "render server" and
"viewer" are SHORTHAND for common combinations, not categories in the code. A headless camera
node is `Mirror` + `stream-cameras`. A VR client is `Mirror` + `accept-input`. A VR client that
ALSO streams its own view is `Mirror` + `accept-input` + `stream-cameras` — nothing special,
just another combination. Because these are capabilities and not bundles, wanting "a viewer that
also streams a camera" needs ZERO new code paths; it is already expressible. If a future feature
tempts us to add a named mode or a "type" enum, that is the smell we are removing — add it as a
capability instead. Keep the axis tiny (one PoseSource) and let the capability set carry the
growth.

### The owner role: any process, one-to-many
An OWNER is any process with authoritative live sim state that advertises, serves its model,
streams transforms, and accepts perturbations. An owner is NOT UE-specific:
- a UE instance (`FreeRun` / `Stepped` / `StatePushed`) can be an owner;
- an external Python or other process can equally be an owner (it holds the sim; it has no UE
  PoseSource at all — PoseSource describes a UE instance, ownership is a process role).
An owner serves MANY mirrors at once (one-to-many fan-out): N UE renderers / viewers / camera
nodes can all attach to the same owner. Any UE instance can attach as a `Mirror` to any owner,
UE or external. So owner-ness is a process role derived from holding live state, decoupled from
whether the holder is UE; a single owner feeds an arbitrary fan-out of mirrors.

### How this expresses every capability, with no duplication
- UE free-runs a sim: `FreeRun`.
- Client steps UE deterministically and reads obs: `Stepped`.
- Client integrates externally, UE reconstructs + serves local data: `StatePushed`.
- Lightweight renderer mirrors an owner, no physics: `Mirror`.
- Renderer runs its own sim and is RPC-drivable: `Stepped` (the old compiled-vs-fast duplication
  is gone).
- Camera farm node: `Mirror` + `stream-cameras`.
- VR viewer: `Mirror` + `accept-input`.
- VR viewer that also streams its own view: `Mirror` + `accept-input` + `stream-cameras` — no
  new code path, just another combination.
- Any instance slaves to any owner, UE or Python, cross-machine.

`Auto` (start FreeRun, promote on connect) stays a launch policy, not a PoseSource value.

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

## 5. One renderer — RESOLVED: approach B (one lightweight structure + pluggable asset source)

Three read-only probes (2026-08-15) settled this, all citations verified. The double-work fear
does NOT materialize. The clean path is ONE `mjModel`-driven lightweight structure whose per-geom
ASSET is resolved from whichever cache already has it (imported for authored models, baked for
wire models) — no rebuild in either case.

### Why B, from the evidence
1. NO play-time rebuild to fear. The authoring path builds meshes ONCE at import and persists
   `/Game` `StaticMesh` assets (idempotent, reused not rebuilt — `MujocoMeshImporter.cpp:258-330`);
   at play it only instantiates components + `SetStaticMesh`-references them + writes transforms
   (`MjGeom.cpp:441-520, 490`). The only `BuildFromMeshDescriptions` in the codebase is the fast
   path. And the blueprint lag is EDIT-time (recompile + construction-script reconstruction, the
   code is written around it — `MjArticulation.cpp:1251-1275`), not a per-play cost. So rendering
   an authored model from the lightweight structure at play does not ADD a rebuild — it REPLACES
   the authoring path's heavier "instantiate hundreds of SCS components + re-resolve every geom
   through the default-class chain" pass with a lighter per-body build.
2. The main divergence is the ASSET SOURCE, a pluggable seam. Materials PARTLY converge (audit
   correction): BOTH paths use the SAME master `M_MuJoCo_Master` and the same parameter-NAME
   convention (`MjLoadMasterMaterial` `MjbScene.cpp:496`; `MjGeom.cpp:529`), and both share
   `MjBindNeutralMaterialTextures` (`:1493`) — BUT the fast path applies parameters via its OWN
   inline `ApplyGeomMaterial` (`MjbScene.cpp:1475-1592`) reading `mjModel` arrays, while the
   authoring path calls the shared `MjApplyMaterialParameters` (`MjGeom.cpp:567`) off a spec.
   So material APPLICATION is still a duplicated seam (a 4th seam, alongside mesh/texture/transform)
   — the unified resolver must converge it, not assume it converged. `UMjCamera` is shared by both.
3. A unified `Resolve(geomId) -> {StaticMesh, material}` is feasible with NO rebuild: authored
   resolves the spec element's stored `MeshAsset` (`MjAssetResolve.cpp:409`), wire resolves the
   content-hashed `SM_<id>` via `LoadObject` (`MjbScene.cpp:1227-1231`), bridged by the
   already-present `mj_id2name` (`:918`). Both terminal lookups are non-rebuilding.
4. Bridge/state/control already separates cleanly from rendering: fast Direct drives all of it
   through a geometry-less shadow `AMjArticulation` (`MjbShadowArticulation.cpp:52-108`). So
   addressing is not a reason to keep the authoring render tree. (In the target the shadow itself
   is replaced by the flat addressing partition, section 4/6; the decoupling point stands.)

Approach A (keep two render trees, share only helpers) is REJECTED: the duplication is four build
seams (mesh, texture, transform-apply, geom-type table), not a shared structure. Approach C
(author into the fast representation) is UNNECESSARY: the shadow already gives the fast path its
addressing without moving authoring into it, and C is the biggest change to the editor for no
benefit over B.

### What B is, concretely
- ONE lightweight renderer: one actor per body, per-geom primitive/mesh components, built from
  the `mjModel` structure (the fast path's `BuildBodies` / `BuildGeoms`), driven by a local
  snapshot or a mirror stream.
- ONE asset resolver `geom -> {UStaticMesh|proc, UMaterialInterface}`: imported UE assets when a
  spec is present (authored), baked-from-`mjModel` otherwise (wire), with a per-geom fallback to
  the baked path for inline meshes the import never produced.
- The authoring `AMjArticulation` component tree becomes EDITOR-ONLY (authoring, placement, spec
  build). It is not the play renderer.

### The enumerable bridges B needs (small, not structural)
- Give the renderer BOTH a `mjModel` (structure + baked fallback) and, for authored models, a
  spec ref, bridged by `mj_id2name`.
- `UMjGeom::OverrideMaterial` (a user pick, not in the model) needs an explicit per-geom override
  channel.
- User-attached decorative UE components / child actors under a body or geom need an editor-only
  declaration OR a reparent-onto-the-body-actor bridge so they follow the physics transform.
- Blueprint logic / possession / input attached to the `AMjArticulation` pawn
  (`MjArticulation.h:389-456`): if the pawn becomes editor-only, any gameplay behaviour a user
  hung on it needs an explicit home (or the lightweight render actor must be possess-able). Niche
  but real; declare the policy.
- Packaged path: the fast packaged build is a runtime `UProceduralMeshComponent` with NO asset, so
  a `UStaticMesh*`-typed resolver can't express it. Either cook the `/Game/URLabFastPath/SM_<id>`
  assets so packaged has real StaticMeshes, or make the resolver return at component level. Decide
  during implementation.

### Out of scope for parity (shared TODO, not a blocker)
Heightfield, SDF, skin/deformable, and site/tendon VISUAL geometry are not built by the fast path
today — but they are also not part of the authoring path's PLAY render (sites/tendons are
debug-draw on the manager; hfield authoring is an editor grid). So unifying does not regress them;
they are a shared future TODO for both renderers, not a reason to keep two.

### One remaining live check (validation, not a gate)
The probes bound the cost SHAPE from code (no rebuild) but not its milliseconds. Since B AVOIDS
the authoring path's instantiate-hundreds-of-SCS-components + per-geom-resolve pass, it is expected
to be at least as fast. Confirm with a live editor profile on a large model (aloha / a humanoid)
that the lightweight play build is not slower than what it replaces. Post-decision validation, not
a blocker on choosing B.

Net: the renderer unifies on ONE lightweight `mjModel`-driven structure with a pluggable asset
resolver; materials and cameras are already shared; the authoring tree goes editor-only; the
losses are three enumerable, bridgeable items. This unblocks the mode collapse (with one
own-a-sim path and one renderer, `EMjbRunMode` has nothing left to encode). The god-object
extraction (task #21) is orthogonal cleanup that feeds straight into this renderer.

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
| `EMjbRunMode{Puppet,Direct}` | folded into `PoseSource` (Puppet=`Mirror`, Direct=`Stepped`) |
| compiled path + fast-path raw install + shadow | ONE `UMjPhysicsEngine` install + one partition |
| `MjbScene` second renderer (~2300 lines) | ONE lightweight `mjModel`-driven renderer + pluggable asset resolver (approach B); authoring tree editor-only |
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
g. Any instance slaves to any owner, UE or Python, cross-machine; and any instance with live
   state (FreeRun/Stepped/StatePushed) can BE an owner for others.
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
8. One renderer (approach B, section 5): one lightweight `mjModel`-driven structure + a pluggable
   `geom -> {mesh, material}` resolver (imported assets for authored, baked for wire); authoring
   tree goes editor-only; bridge the 4 enumerable authoring-only items. The `MjbScene` god-object
   extraction (task #21) feeds this. One live profile confirms the play cost post-decision.
9. Mode collapse: one `PoseSource` axis + composable capabilities; single source of truth for the integrator
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
- ONE RENDERER — RESOLVED to approach B by the 2026-08-15 probes (section 5): one lightweight
  `mjModel`-driven structure + a pluggable `geom -> {mesh, material}` resolver. Confirmed: no
  play-time mesh rebuild (authoring builds meshes once at import; BP lag is edit-time); materials
  + `UMjCamera` already shared; only the asset SOURCE diverges (a pluggable seam). Remaining
  implementation sub-decisions: the packaged `ProceduralMesh` return type (cook `SM_<id>` vs
  component-level resolver); and one post-decision live profile that B's play build is not slower
  than the authoring SCS-instantiate pass it replaces.
- `mjz` codec availability in the linked `libmujoco` — verify before relying on the mjz source
  format (mjb and xml+assets are already known-feasible).
- The `load_model()` RPC and the fast-path model source must be ONE normalize-to-`mjModel`
  mechanism, not two.

### Resolved during iteration (2026-08-15)
- Mirror (cheap, no `mj_forward`) and `StatePushed` (pays `mj_forward` for local data) are
  DIFFERENT and both stay; they are SIBLINGS on the one PoseSource axis (both externally
  sourced), not on different axes. An earlier draft blurred them AND wrongly said Mirror has no
  local mjModel — it does load the wire MJB and build from it (`MjbScene.cpp:471`).
- "Render server" (cameras out) and "viewer" (interactive input in) are INDEPENDENT overlays,
  not one bundle; a VR viewer is `Mirror` + input, no cameras.
- Owner-ness is a PROCESS role, not UE-specific: a UE instance (FreeRun/Stepped/StatePushed) OR
  an external Python/other process can own; ownership is decoupled from PoseSource (PoseSource
  describes a UE instance). An owner serves MANY mirrors at once (one-to-many fan-out).
- Fast-path model source broadens from MJB-only to `{mjb, xml+assets, mjz}`; xml/mjz compiled by
  the receiver removes the version-skew that forces the `raw_*` re-description today.
- Mode model collapsed from two axes (SimSource + Drive) to ONE (PoseSource); every renderer has
  an mjModel, so the axis is where the render pose comes from, not whether a model exists.

---

## 15. Implementation detail (per phase)

Concrete enough to prototype and to audit. Anchors in `code font` with line numbers are VERIFIED
against the current branch; new types/signatures are marked PROPOSED. Paths are under
`Source/URLab/` unless noted. Each phase compiles and ships on its own.

### Phase 0 — standalone correctness fixes (independent of the redesign)
These are real bugs; land them first, on this branch, regardless of the rest.

0a. REGISTRY-REBUILD RACE (memory-unsafe). The registry is `Empty()`+rebuilt with no lock
(`MjPhysicsEngine.cpp` `InstallCompiledSpec:832-833`, `InstallRawModel:961-962`) while the RPC
thread reads it unlocked (`GetArticulation:1433`, `GetAllArticulations:1469`).
- FIX: take `CallbackMutex` around the rebuild, and make `GetArticulation`/`GetAllArticulations`
  either take the lock or return a copied snapshot. Prefer a snapshot (`TArray` copy under lock)
  so RPC reads never block the worker.
- TEST: a threaded stress test (install/uninstall in a loop while an RPC thread enumerates) under
  the existing `MjThreadTests` harness.

0b. `MjbScene::BeginDestroy` UAF: frees `Model`/`Data` without `UninstallRawModel`
(`MjbScene.cpp:262-271`).
- FIX: call the engine uninstall (join the worker) before freeing, mirroring `Teardown`
  (`:2262-2309`).
- TEST: destroy a Direct-mode `AMjbScene` while its worker is stepping (PIE end).

0c. PUPPET FREE-RUN HAZARD: `FPuppetStepMode` installs no step handler, yet `SetStepMode(Puppet)`
unpauses the worker (`MjPhysicsEngine.cpp:1368`), so a handler-less worker can free-run `mj_step`
on idle wakes (`:1255-1265`) while the client believes it owns the integrator.
- FIX: gate the worker's `mj_step` on the resolved clock being an integrating one; in the
  external/push-state clock the worker only `mj_forward`s pushed state, never `mj_step`s.
- TEST: assert `data->time` does not advance under Puppet without a push.

0d. WORLDGEOMS OFF THE PER-STEP PATH: `WorldGeomCache` is rebuilt every physics step
(`MjStateCollector.cpp:529-551`) but never encoded to msgpack (ROS-only).
- FIX: move the world-geom collection behind the ROS `IMjStateConsumer`, recomputed on the ROS
  publish tick (poses only). Zero wire change, zero Python change.
- TEST: existing `MjStateCollectorTests` still green; ROS providers still receive geometry.

### Phase 1 — addressing partition (compiled path), dual-run
NEW (PROPOSED): `Public/MuJoCo/Core/MjAddressable.h`
```cpp
struct FMjAddressable {                 // replaces FMjEntityRecord + AMjArticulation-as-addr-unit
    FName Name;                         // stable public name (compiled: participant prefix)
    int32 RootBodyId; bool bFreeBase;
    TArray<int32> BodyIds, JointIds, ActuatorIds, SensorIds;   // may be empty
};
namespace MjAddressableBuilder {
    TArray<FMjAddressable> Build(const mjModel* m, const FMjPartition& How);  // by name prefix
}
```
NEW: host `TArray<FMjAddressable> Addressables` on `UMjPhysicsEngine`, built in
`InstallCompiledSpec` right after compile. NAME the type per section 14 (retire/rename the
existing `FMjEntityRecord`, `AMjManager.h:82`, to `FMjPropRecord` in the same change).
DUAL-RUN: build alongside the existing articulation registry; a test asserts the partition's
name+id-slices match the actor registry (both derive from the same compiled `mjModel`).
TEST: new `MjAddressableTests` + an equality assert modeled on `MjParityGoldenTests`.
No behavior change this phase.

### Phase 2 — observation on the partition + one handshake
CHANGE: `FMjStateCollector` walks `Addressables` and copies id slices from `mjData` (the pattern
the `entities` block already uses, `RpcDispatcher.cpp:1021-1055`).
DELETE: `FCachedArticulation` / `Producers` / `RebuildProducerCacheGameThread`
(`MjStateCollector.h:76-114`) and `DescribeElement`'s type dispatch (`MjStateCollector.cpp:194-230`).
NEW (PROPOSED): one enrichment side-channel on the manager, keyed by addressable name:
```cpp
struct FMjEnrichment { FName Name; TWeakObjectPtr<UObject> Producer; EMjEnrichmentScope Scope; };
TArray<FMjEnrichment> Enrichments;      // twist ctrl, user-channel comp, scene producer, cam/ctrl meta
```
Consumed by the ROS `IMjStateConsumer` and by an OPT-IN msgpack `user` block only. Keep the
compiled `UMjSensorRuntime`/`UMjJointRuntime` DescribeState logic ONLY as ROS enrichment.
CHANGE: handshake serializes `FMjAddressable` + reads ranges/gear/types straight off `mjModel`,
one code path for compiled and raw. The camera/controller metadata (`RpcDispatcher.cpp:838-1015`)
comes from the enrichment side-channel (it is not on `mjModel`).
TEST: `MjStateCollectorTests`, `MjUserChannelTests` rewritten against the partition + side-channel.

### Phase 3 — one control store + one drain
NEW (PROPOSED): on `UMjPhysicsEngine`
```cpp
struct FMjControlBuffer   { TArray<double> Setpoint; TBitArray<> Touched, HoldLatch; };   // size nu
struct FMjStateInjection  { TArray<double> Qpos; TBitArray<> QposHold; };                 // keyframe qpos-hold
```
NEW: one pre-step drain in the worker, under `CallbackMutex`, modeled on the existing
`DrainCommands` (`MjPhysicsEngine.h:568-604`) that already serializes mocap/wrench. Per touched
id: `Drive==direct` writes `d->ctrl[id]=Setpoint[id]`; `Drive==controller` writes
`controller.transform(id, Setpoint[id], d)`. Then apply `FMjStateInjection` (qpos-hold, mocap-style).
CHANGE: `ApplyStepCtrl` (`RpcHandlers_Step.cpp:452-475`) becomes "parse setpoints into
Setpoint/Touched" — one branch. Re-home the other writers: the 4 `SetNetworkControl` callers
(`RpcHandlers_Step.cpp:473`, `ZmqSubscribeTransport.cpp:446`, `RosRpcTransport.cpp:450,571`), the
Live `ApplyControls` (`MjPhysicsEngine.cpp:1238`), the Puppet push (`:580`), and `ResetToKeyframe`
(`MjArticulation.cpp:763`) all feed the one buffer.
DELETE: `NetworkControl`/`InternalControl` slot arrays (`MjArticulation.h:181-206,485-488`),
`ApplyControls` both overloads (`MjArticulation.cpp:432,442`), the `OwnedActuatorIds` gate
(`:502-510`), `SkipController` (`RpcHandlers_Step.cpp:1022-1046`), `UMjPassthroughController`
(it IS `Drive=direct`), and `EControlSource` + `set_control_source` (`MjPhysicsEngine.h:44`,
`RpcHandlers_SimOptions.cpp:410-473`). DATA-IFY `UMjPDController`: keep the law as a free function
(`torque = Kp(target-pos) - Kv*vel`, `MjPDController.cpp:62-95`) + a `{Kp,Kv,limit}` gains struct,
drop the `UObject`. Keep `MjPDControllerTests` pointed at the free function.
FIRST, NARROW CUT (ship before the deletions): route the raw ctrl branch through the Touched-masked
buffer so raw-ness resolves ONCE, WITHOUT yet deleting `EControlSource` or touching qpos-hold —
this also fixes Live silently ignoring `control_mode` (`MjPhysicsEngine.cpp:1238`). This is the
highest-risk area; keyframe-hold ordering vs setpoints is the subtle spot.
TEST: `MjActuatorControlSlotTests`, `MjStepServerTests`, `MjPDControllerTests`, `MjRosLinkTests`.

### Phase 4 — raw path on the partition; delete the shadow
CHANGE: `InstallRawModel` builds `FMjAddressable` from the raw `mjModel` (single root or
body-subtree split).
DELETE: `MjbShadowArticulation.{h,cpp}` (`:52-108`), the shadow spawn/teardown in `MjbScene`
(`ShadowArt`, `MjbScene.h:307`; `BeginDirect`/`InstallIntoEngine`), the `raw_actuators`/`raw_joints`
handshake block + `bRawShadow` (`RpcDispatcher.cpp:885-951`). PREREQUISITE: the wire model source
(phase 7) or the version-skew fix, so the receiver has a loadable model without re-description.
REPOINT the ~30 `GetAllArticulations` consumers that resolved through the shadow on the raw path
(`MjDebugVisualizer.cpp`, `ZmqSubscribeTransport.cpp`, `MjReplayManager.cpp:235`,
`RosCameraInfoProvider.cpp:87`, `MjSimulateWidget.cpp`, `RosRpcTransport.cpp:262`) at the
partition or exclude them.
TEST: live RPC control against a raw/wire model with no shadow (the `MjStepServerTests` path).

### Phase 5 — one renderer (approach B, section 5)
NEW (PROPOSED): `IMjGeomAssetResolver { FMjGeomAssets Resolve(int32 GeomId) const; }` returning
`{UStaticMesh* Mesh /*or proc*/, UMaterialInterface* Material}`. Two impls:
- `FMjBakedAssetResolver` — wraps `GetOrBuildStaticMesh`/`GetOrBuildTexture`/`ApplyGeomMaterial`
  (`MjbScene.cpp:1212-1592`), keyed by mesh id + `ContentHash` (`:488-494,1223-1231`).
- `FMjImportedAssetResolver` — holds an `FSpecRef`; per geom does `mj_id2name(m, mjOBJ_MESH,
  geom_dataid)` (`MjbScene.cpp:918`) -> `MjResolveMesh(Spec, name).Asset` (`MjAssetResolve.cpp:409`)
  and `MjResolveTexture` (`:342-354`); FALLS BACK to the baked resolver per geom for inline meshes.
NEW: extract the fast path's structure builder (`BuildBodies:536-575`, `BuildGeoms:577-603`,
`BuildGeom:863-1050`, `BuildCameras:686-749`) into a reusable renderer that takes `(mjModel*,
IMjGeomAssetResolver&, poseSource)`. Materials already funnel to `MjApplyMaterialParameters` +
`M_MuJoCo_Master` in BOTH paths (`MjbScene.cpp:496,1493`; `MjGeom.cpp:529,565-567`) — unify the
value source (`FMjMaterialValues` from spec OR `mjModel`).
CHANGE: the compiled path renders through this renderer at play; `AMjArticulation`'s `UMjGeom`/
`UMjBody` tree becomes EDITOR-ONLY (authoring/placement/spec-build, `MjGeom.cpp:441-520`).
BRIDGES (enumerable, from the probe): a per-geom `OverrideMaterial` override channel
(`MjGeom.h:96-97`); a "keep user attachments" declaration reparenting decorative components onto
the body actor; a policy for Blueprint-on-pawn logic (`MjArticulation.h:389-456`); and the
packaged `ProceduralMesh` return type (cook `SM_<id>` assets, or widen `FMjGeomAssets` to a
component-level return — `BuildMesh:1324-1363` has no `UStaticMesh`).
LEAN-IN: task #21 (extract `FMjbAssetBaker` etc. from `MjbScene`) IS the first slice of this.
TEST: golden pixel/structure parity between the old authoring render and the new renderer on a
menagerie model; + the live play-cost profile (validation, not a gate).

### Phase 6 — model source over the wire ({mjb, xml+assets, mjz})
CHANGE: the receive path normalizes to `mjModel` using calls the runtime ALREADY links —
`mj_parseXMLString` (`MjSpecWriteHooks.cpp:2034`), `mj_compile` (`MjSceneSpec.cpp:386`), `mjVFS`
(`:377-387`), `mj_loadModelBuffer` (`MjbScene.cpp:471`). Reuse the existing bytes->temp-dir shim
(`upload_model_manifest/chunk/commit`, `RpcHandlers_ModelUpload.cpp:448-512`). Unify with the
`load_model()` RPC as ONE mechanism. `mjz`: VERIFY the codec is compiled into the linked lib
before relying on it (mjb + xml+assets are known-feasible). BENEFIT: xml/mjz compiled by the
receiver removes the version skew that forces phase 4's `raw_*` block.
TEST: `MjModelUploadTests` extended with format tags; a wire model loaded from xml with assets.

### Phase 7 — mode collapse (PoseSource) + transport
NEW (PROPOSED): `enum class EMjPoseSource : uint8 { FreeRun, Stepped, StatePushed, Mirror };`
replaces `EStepMode` (`MjPhysicsEngine.h:65-72`) AND `EMjbRunMode` (`MjbScene.h:26-35`). The clock
gets ONE source of truth (kill the three-way encoding: `ResolvedStepMode` atomic
`MjPhysicsEngine.cpp:1133`, `CustomStepHandler` presence `:1073`, and the strategy object).
Capabilities become an open flag set on the instance (`stream-cameras`, `accept-input`), NOT modes.
CHANGE (transport): one `Subscribe` base mirroring the clean `UURLabRpcTransport`
(`RpcTransport.h:33`); the fast-path bus (`MjbScene::BusTransport`, `MjbScene.h:286`) becomes a
normal Publish topic a `Mirror` renderer subscribes to; retire the parallel
`ClientSubscribe`/`ViewerSubscribe` hierarchies. (Caveat: read `ViewerSubscribe` internals first.)
TEST: `MjStepServerTests` + a mirror/renderer integration test across the unified Subscribe base.

### Phase 8 — vocabulary rename + Python wrapper (LAST)
Rename across code + docs: Driver / Renderer / Registry / Integrator / ControlLease / Addressable.
The Python client is a thin wrapper and is updated last to match the new wire; `enums.py` strings
change (allowed). This is mechanical once the C++ core lands.

### Cross-phase notes for the auditors
- Phase order is dependency-driven: 1->2 (partition before collector), 3 independent (control),
  4 needs 6-or-skew (wire model without shadow), 5 independent of 3/4 (renderer), 7 needs 1+4
  (partition + no shadow). Phases 0 and 6 can start immediately.
- Highest risk: phase 3 (control drain — keyframe-hold ordering) and phase 5 (renderer fidelity
  parity + packaged return type). Everything else is mechanical once the partition exists.
- Every deletion listed has a test that must be rewritten, not just removed:
  `MjStateCollectorTests`, `MjStepServerTests`, `MjControlOwnershipTests`, `MjUserChannelTests`,
  `MjActuatorControlSlotTests`, `MjPDControllerTests`, `MjRosLinkTests`.

---

## 16. Audit findings — required revisions BEFORE prototyping (2026-08-15)

Two independent adversarial auditors read the plan cold. Verdict: the DIRECTION is sound, but the
plan is NOT implementable as written. This section is authoritative over sections 3/5/7/15 where
they conflict. The central correction: the premise "addressing + observation + control all reduce
to index-mjData-by-id, so the actor + producer graph are deletable" is only ~70% true. The
missing 30% (sensor semantics, per-body FK, camera identity, ALL ROS control ingress) is keyed on
the live `AMjArticulation`/`UMjNodeComponent`/`AAMjManager`, NOT on ids, and is NET-NEW work the
plan never scoped. Phase 4 (delete the shadow) regresses ROS + cameras until that work exists.

### 16.1 Corrected anchors (were cited as VERIFIED, are wrong)
- StatePushed push: NOT `MjPhysicsEngine.cpp:560-571` (that is `RestoreState()`, migration). It is
  `mj_setState`+`mj_forward` at `:1218-1219` + `FPuppetStepMode::HandleStep` inline
  (`RpcHandlers_Step.cpp:~884`). (Fixed inline in section 3.)
- `CustomStepHandler` presence: NOT `MjPhysicsEngine.cpp:1073` (that is `ApplyOptions`). Real:
  `FDirectStepMode::OnEnter:734 -> InstallDirectHandler` vs `FPuppetStepMode::OnEnter:829-834`
  (installs none) in `RpcHandlers_Step.cpp`.
- Materials: `MjbScene.cpp:1493` is `MjBindNeutralMaterialTextures`, not `MjApplyMaterialParameters`;
  material APPLICATION is still duplicated (fixed inline in section 5).
- `PerArticulationControlMode`: `StepCommands.h:36` (`:35` is the comment); `RpcHandlers_ModelUpload.cpp:593`
  is `EStepMode Mode`, not that field.
- `NetworkControl`/`InternalControl` atomic arrays: `MjArticulation.h:485-488` (`:181-206` are the
  staging method decls).
- WorldGeom (`0d`): the cache is built once on the game thread; `MjStateCollector.cpp:529-551`
  recomputes the snapshot from it each step. The ROS-only / not-msgpack half is correct; the fix stands.

### 16.2 Design gaps that MUST be resolved before phases 3 / 5 / 7
1. DRAIN CADENCE (HIGH, blocks phase 3). Controllers recompute PER SUBSTEP today (Direct loops
   `ApplyControls` inside the n-step loop `RpcHandlers_Step.cpp:1038-1046`; Live before every
   `mj_step` `MjPhysicsEngine.cpp:1235-1257`). A `DrainCommands`-style ONCE-per-request drain makes
   a PD controller open-loop for substeps 2..n. Direct setpoints want set-once-persist; controllers
   want evaluate-every-substep. The single "one drain" abstraction conflates two cadences. DECIDE:
   the drain is a per-substep hook where direct setpoints persist (do not clear `Touched` each
   substep) and controllers re-evaluate each substep; specify exactly.
2. STATE-INJECTION CHANNEL (HIGH, blocks phase 3). `FMjStateInjection{Qpos, QposHold}` is
   insufficient for keyframe qpos-hold: today's `bHoldViaQpos` (`MjArticulation.cpp:454-478`) also
   (a) ZEROES qvel for held DoFs, (b) SKIPS free joints, (c) SUPPRESSES that entity's ctrl write
   (early-return `:488`). Add a qvel channel + free-joint rule + a "hold suppresses ctrl for this
   entity" rule.
3. RESOLVER RETURN TYPE (HIGH, blocks phase 5). `{UStaticMesh*, material}` is unrepresentable in
   PACKAGED builds — `GetOrBuildStaticMesh` is `#if WITH_EDITOR` (ends `MjbScene.cpp:1322`);
   packaged is a runtime `UProceduralMeshComponent` with no asset (`BuildMesh :1324-1363`). The
   seam MUST be at COMPONENT level (resolver populates a component), or every wire mesh must be
   cooked to `SM_<id>` (the current cook does not). Redefine `IMjGeomAssetResolver` accordingly.
4. PLAY RENDER PATH (blocks phase 5). Section 5's parity anchors (`RebuildVisualizer MjGeom.cpp:441`,
   `PostEditMove MjArticulation.cpp:1251`) may be EDITOR-preview/edit-time. Pin the ACTUAL play-time
   authoring render entry point before asserting parity. (Note `MujocoMeshImporter.cpp` is in
   `Source/URLabEditor/`, a cross-module dependency the "editor-only tree" split must handle.)
5. `ViewerSubscribe` IS NOT REDUNDANT (blocks phase 7). `UURLabViewerSubscribeTransport` owns a
   `ClientSubscribe` AND runs `mj_forward` (`ViewerSubscribeTransport.cpp:146`) — it is StatePushed
   delivered over subscribe, not a cheap Mirror. Do NOT fold it into "a Publish topic a Mirror
   subscribes to"; keep the forward-reconstruction role.
6. `EControlSource` DELETION IS A BEHAVIOR CHANGE (phase 3). It selects the ZMQ-vs-UI staged slot
   (`MjArticulation.cpp:420`) and is fed into every controller (`ComputeAndApply(Source)`). Deleting
   it makes ZMQ+UI co-drive last-write-wins-by-lease. Confirm acceptable; list under "must survive".

### 16.3 Net-new work the plan did NOT scope (the missing 30%)
Each is a NEW workstream, required before phase 4 can delete the shadow without regressing:
- ROS CONTROL INGRESS routing (HIGH). `RosRpcTransport` writes straight through the actor:
  `GetAllArticulations` (`:262`), per-write `GetArticulation` (`:436,485,533`),
  `GetActuators`+`SetNetworkControl` (`:444-451,545-571`), `FindComponentByClass<UMjTwistController>`
  (`:490`), and `GetStructureVersion()` to rebuild subs (`:238,366`). Deleting the shadow breaks
  `cmd_ctrl`/`cmd_vel`/`joint_command`/`claim_control`/user-channel input for wire models. Needs a
  shadowless command-routing target (a control-ingress interface keyed by addressable name).
- SENSOR-SEMANTIC model-only table (HIGH). `FMjSensorState::Semantic` comes from
  `UMjSensorRuntime::GetSemantic` (`MjStateCollector.cpp:146`) off the ProtoSpec schema, NOT
  `mjModel`. No `mjtSensor -> EMjSensorSemantic` table exists. `/tf` + `/odom` per-link poses come
  from `UMjBody::DescribeState` (`MjBody.cpp:375-395`), not an id-slice (which yields only the base
  pose). "Keep DescribeState as ROS enrichment" transitively keeps the shadow alive — build the
  model-only replacements (semantic table + per-body-id FK walk) or phase 4 can't land.
- CAMERA IDENTITY + egress (HIGH). `stream-cameras` is not a clean toggle: the RPC camera surface
  enumerates only via `GetAllArticulations` (`RpcHandlers_Camera.cpp:210`, can't see fast cameras);
  `ResolveCameraCanonical` does `Cast<AMjArticulation>` (`MjCamera.cpp:131-143`) so a lightweight
  body actor gets different topics/SHM stems/handshake keys; frame stamping lives only on
  `AAMjManager` (`MjCamera.cpp:790-801`); mirror camera poses ride a SEPARATE `cxpos/cxquat` stream
  (`MjbScene.cpp:1770-1797`) a body-transform-only mirror drops. There are FOUR camera egress paths
  (per-camera ZMQ PUB, per-camera SHM via `CameraShmWriter`, `FMjCameraFrameBus`, RPC reply). Needs
  a renderer-agnostic camera registry + canonical-name resolver decoupled from `AMjArticulation`.
- HANDSHAKE is a RE-PLUMB, not a collapse (MED-HIGH). `actuator_types`/`default_control_mode`/
  controller block are walked off the live actor (`RpcDispatcher.cpp:838-876`), not `mjModel`. If we
  stop shipping `mjb`/`mjcf_compiled`, the "one description" must CARRY that per-element metadata on
  the wire, captured from the actor phase 5 is demoting. Scope it as expansion.
- REPLAY is a third clock consumer (MED). `MjReplayManager` installs its own `SetCustomStepHandler`
  (`:385`), walks `GetAllArticulations` (`:235`), runs `mj_forward` (`:1279`). Add to "must survive"
  and to phase 7's handler model (Direct/Puppet/Replay, not two).
- LEVEL SAVE/RELOAD tags (MED). The render-server product saves actors tagged `MjbBody=`/`MjbGeom=`
  and rewires via `ReindexFromLevel` (`MjbScene.cpp:83-84,165-189,292`). The new renderer MUST
  preserve the tag scheme + reindex path.
- SHM has no Subscribe backend (MED); camera SHM is a bespoke `CameraShmWriter` not routed through
  `Publish`. The "one Subscribe base" must add an SHM subscribe or scope ZMQ-only.
- Python is REAL work, not a rename (HIGH for the bridge phase). `_walk_model` (`articulation.py:882-995`)
  is the entire source of joints/actuators; `set_control_source` is a public method
  (`runtime.py:191-208`); `enums.py` values ARE the wire strings and `coerce()` raises on unknowns
  (`:102`). Breaking-API, hard-raising, not silent. (This feeds the separate bridge redesign.)

### 16.4 Blast-radius gaps (consumers the plan missed)
- `GetAllArticulations` also at: `RpcHandlers_Step.cpp:178,295,366,430,558,1022`;
  `RpcHandlers_SimOptions.cpp:74,454,463,489`; `RpcHandlers_Scene.cpp:104,119,123,272`;
  `RpcHandlers_Control.cpp:128,168`; `AMjManager.cpp:128,936`; `MjStateCollector.cpp:274`;
  `MjSimulateWidget.cpp:274,799,1414`; `RosRpcTransport.cpp:436,485,533` + `GetStructureVersion :238,366`.
- `EControlSource` behavior consumers: `MjArticulation.cpp:420,429,493,508`;
  `MjSimulateWidget.cpp:981,995,1325,1412,1417,1472`; controllers `MjPDController.cpp:72` /
  `MjPassthroughController.cpp:35`; tests `MjActuatorControlSlotTests`, `MjActuatorTests`,
  `MjSpecInstallTests`, `MjPDControllerTests`.
- Mode-enum reach: ~151 refs across 22 files; switch logic in ~13 Source files incl. `MjCamera.cpp`
  (7 sites), `RpcHandlers_Step.cpp` (13). The mode collapse is NOT engine-confined.
- Race 2 (BeginDestroy) is LATENT/guarded on the normal path (play nulls in EndPlay; only
  abnormal destroy-without-EndPlay hits it), not a live UAF — still worth fixing, don't overstate.
- `mjz`: the pinned headers declare only `mj_encode`, NO `mj_decode`/archive-load. Treat as
  UNAVAILABLE and drop from the format list unless a decoder is confirmed registered in the lib.

### 16.5 Corrected dependency graph (concurrent vs serial)
Phases the doc called independent are not:
- Phase 3 FULL depends on Phase 1 (per-id Drive needs the partition). Only the NARROW cut is
  partition-free.
- Phase 5 actor-DEMOTION depends on 1+2+3 (the bridge addresses/collects/controls through the
  actor until those move off it). The renderer EXTRACTION is independent; the demotion is not.
- Phase 3 and Phase 7 both rewrite the SAME worker loop (`MjPhysicsEngine.cpp:1224-1266`; the
  `bSkipApplyControls=(Mode==Puppet)` branch `:1232` fuses control + clock). They serialize.
- Phase 0d and Phase 2 both rewrite `MjStateCollector::Collect()`.
- Phase 4 needs a shadowless ROS command-routing path (16.3) in addition to phase 6.

```
Serial spine:   1 -> 2 -> 3(full) -> 4 -> 7 -> 5-demote -> 8
                6 -> 4 ;  3 -> 7 (shared worker loop)
Concurrent from day 1: 0a, 0d, 6, renderer-EXTRACTION(#21)+resolvers,
                       ROS model-only replacements (semantic table, FK, ingress), camera registry
```

### 16.6 Shared-file contention (merge hotspots)
Four-way collisions block naive per-phase parallelism:
- `MjPhysicsEngine.*` — 0a, 0c, 1, 3, 4, 7 (nearly every phase).
- `RpcHandlers_Step.cpp` — 2, 3, 4, 7.
- `MjbScene.*` — 0b, 4, 5, 7.
Also 2-3 way: `RpcDispatcher.cpp` (2,4), `MjStateCollector.*` (0d,2,4), `MjArticulation.*` (3,5,7),
`MjCamera.cpp` (5,7), `RosRpcTransport.cpp` (3,4,7), `AMjManager.*` (1,2,4,5,7).
RULE: on the hotspot files, assign ONE owning agent across all colliding phases (partition work by
FILE there), and by FEATURE elsewhere.

### 16.7 Parallel decomposition (contract-first)
- STEP 0 (serial, coordinator, no logic): resolve the addressable-unit NAME first; land header-only
  compiling stubs for the frozen contracts — `MjAddressable.h` + builder, `FMjControlBuffer` +
  the FULL state-injection channel, `FMjEnrichment` + EMjEnrichmentScope, a COMMAND-INGRESS
  interface (not just state), `IMjGeomAssetResolver` at COMPONENT level, `EMjPoseSource` + capability
  flags, an `mjtSensor->EMjSensorSemantic` table interface, and a renderer-agnostic camera registry.
- STEP 1 (fan out, ~5 agents, no spine files): 0a/0b/0c lifecycle fixes; 6 wire-model normalize;
  renderer extraction (#21) + resolvers; ROS model-only replacements (semantic table + FK + ingress
  routing); 0d worldgeom (coordinate with the phase-2 owner).
- STEP 2 (serial spine, one owner per hotspot file): `1 -> 2 -> 3(narrow -> full) -> 4 -> 7 ->
  5-demote -> 8`, rebasing each on the last. Ship the phase-3 NARROW cut first (fixes
  Live-ignores-control_mode `MjPhysicsEngine.cpp:1232` independently).

### 16.8 Newly-blocking open decisions (add to section 14)
Drain cadence (16.2.1); full state-injection channel (16.2.2); component-level resolver (16.2.3);
the actual play render path (16.2.4); ViewerSubscribe's fate (16.2.5); EControlSource behavior
change accepted? (16.2.6); scope the missing-30% net-new work (16.3) into explicit phases; drop
`mjz` unless a decoder is confirmed; and the addressable-unit name (still open, blocks Step 0).

### Additions to "functionality that MUST survive" (section 11)
i. ROS control ingress (cmd_ctrl/cmd_vel/joint_command/claim_control/user-channel) to wire models.
j. Sensor-semantic ROS typing (Imu/Wrench/…) and per-link `/tf` + `/odom`.
k. Camera streaming from the unified renderer with STABLE canonical topics/SHM stems + frame stamps.
l. Replay (its own clock handler + forward).
m. Level save/reload of renderer actors via the `MjbBody=`/`MjbGeom=` tag + reindex.
n. UI+ZMQ co-drive semantics (or an accepted, documented behavior change).
