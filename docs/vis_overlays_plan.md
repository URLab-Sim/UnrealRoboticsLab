# Visualization overlays in UE — plan (studio/filament-parity, all modes)

Goal: reproduce MuJoCo's full "vis options" experience (contact points/forces, external
& perturbation forces, inertia, CoM, constraints, camera/light glyphs, frames, wrenches,
…) in Unreal, rendered *properly* (not `DrawDebug`), and make it work in **every render
role** — owner, in-process renderer, mirror, peek, VR — over **both gRPC and ZMQ**.

Grounded in two audits (Aug 2026):
- Upstream vis audit: `scratchpad/mujoco_vis_audit.md` (mjvGeom pipeline, scale formulas,
  filament rendering, data-locality).
- Roles/modes reference: [`render_roles.md`](./render_roles.md) (where `mjData` lives per role).

Joints are **excluded** — the plugin already renders them natively and better.

---

## 1. The one architectural decision: consume `mjvGeom`, don't re-derive

MuJoCo's renderer is two decoupled stages. Stage 1, `mjv_updateScene()`
(`engine_vis_visualize.c:3373`), walks `mjModel`+`mjData` and emits backend-agnostic
`mjvGeom` primitives (sphere/box/ellipsoid/capsule/cylinder/arrow/arrow1/arrow2/line/
linebox/label/triangle) in world coordinates. Stage 2 (classic GL, or studio's **filament**)
just draws them. **We adopt the same seam: keep stage 1 (call `mjv_updateScene`), replace only
stage 2 with proper UE rendering.** This gives every vis option — and every future one — for
free, with guaranteed parity. The plugin already links libmujoco and already states this 1:1
intent in `MjOverlayFlags.h`.

Every arrow/capsule/cylinder/line obeys one convention (`mjv_connector`,
`engine_vis_visualize.c:251-288`): a unit mesh whose **local +Z is its axis**, scaled
`(width, width, length=size[2])`, rotated by `mat`, placed at `pos`. That single rule maps
directly onto a UE instanced-mesh transform.

---

## 2. UE rendering subsystem (`UMjOverlayRenderer`, rewritten)

Replace the current `DrawDebug*` calls (`MjDebugVisualizer.cpp:83-99`,
`MjOverlayRenderer.cpp:211-213`, `MjPerturbation.cpp:488-529`) with:

- **One `UInstancedStaticMeshComponent` per unit primitive** — unit sphere, box,
  cylinder/capsule, single-head arrow (ARROW), double-head arrow (ARROW2), thin stick
  (ARROW1), wireframe box (LINEBOX), line. Each `mjvGeom` → one instance. O(1) draw calls,
  scales to hundreds of contacts (which per-actor DrawDebug cannot). Author arrow meshes along
  +Z, unit length/radius, base at origin; port studio's arrow proportions
  (`kArrowScale=1/6`, `kArrowHeadSize=1.75`, `renderable.cc:54-55`) for visual parity.
- **Per-instance color/alpha via `PerInstanceCustomData`** feeding one master material — no
  MID-per-instance. Master material is translucent-capable PBR (Metallic≈0, Roughness≈0.4,
  emissive from `mjvGeom.emission`) so decor catches scene lighting instead of looking flat.
- **Opaque vs translucent ISM split** — inertia boxes (α .6), frustums (α .2), BVH (α .5),
  transparent dynamic geoms need UE distance-sorted translucency; mirrors MuJoCo's back-to-front
  `geomorder` sort (`render_gl3.c:894-928`). Decor casts **no shadows** (matches `mjCAT_DECOR`).
- **Selection = custom-depth + outline post-process** (studio's jump-flood outliner analog),
  nicer than the classic emission glow.
- **Dynamic meshes** (`UDynamicMeshComponent`) only for flex faces, skins, camera-frustum
  triangles, tactile fields. **Labels** via pooled `UTextRenderComponent`s. **Niagara** reserved
  for future large point fields (tactile), not the standard options.
- Pool everything by high-water mark; hide extras by zero-scale, never reallocate. Bound by a
  `maxgeom` like MuJoCo's `scn->maxgeom`.

**Port these value→size formulas verbatim** (so magnitudes match upstream), reading
`vis.scale.*`/`vis.map.*`/`vis.rgba.*`/`stat.*` from the model rather than hardcoding:
- Force arrow length `= |force| · vis.map.force / stat.meanmass`; width `= vis.scale.forcewidth · stat.meansize`.
- Contact disk radius `= contactwidth·meansize`, half-height `= max(contactheight·meansize, -dist/2)`.
- Inertia box half-size `sz[k] = sqrt((ΣI − 2·I_k)/(2m)) · √3` (ellipsoid uses √5).

**Net-new vs upstream (optional):** a true 6-D **wrench** glyph. Upstream computes contact
torque (`confrc[3:6]`) and applied torque (`xfrc[3:6]`) but never draws them, though
`vis.rgba.contacttorque` exists. A curl/torus-about-axis glyph scaled by `vis.map.torque` (0.1)
would be a genuine improvement — the user explicitly wants wrenches.

---

## 3. Making it work in ALL modes — where the `mjvGeom`s come from

`mjv_updateScene` needs `mjData`. Per [`render_roles.md`](./render_roles.md), only some roles
have it. So the overlay source differs by role, but the **renderer (§2) is identical everywhere**:

| Role | Has `mjData`? | Overlay source |
|---|---|---|
| **Owner** (UE `AMjManager` / Python) | yes | call `mjv_updateScene` locally each step |
| **In-process / Stepped renderer** (`-URLabFastDirect`) | yes | `mjv_updateScene` locally |
| **Peek viewer** (Python, `mj_forward`) | yes (fwd only) | `mjv_updateScene` locally (contacts need a real step; `mj_forward` gives kinematics-only) |
| **Mirror** (`AMjRenderer`, transforms only) | **no** | **must receive a compact overlay stream** (§4) |
| **VR/spectator** | no (pairs with a mirror/viewer) | whatever its paired renderer uses |

Data-locality (from the vis audit §8):
- **[XFORM]** a transforms-only mirror can already draw natively: **frames** (body/geom/site),
  **camera & light glyphs** (if cam/light transforms are on the bus).
- **[MODEL]** derivable from streamed transforms + static `mjModel`: **inertia boxes**
  (principal moments are static; needs CoM transform), **BVH boxes**.
- **[OWNER]** impossible without extra streamed `mjData`: the entire **contact / force /
  constraint / sensor** family, **subtree CoM**, **actuator ctrl/act coloring**, **tendon wrap
  paths**, **skin/flex** deformation.

So an owner/local-mjData role gets the full experience immediately (§1–§2). A mirror gets the
[XFORM]/[MODEL] set for free and needs §4 to unlock the [OWNER] set.

---

## 4. Streaming capabilities: overlay data to mirrors/peekers (gRPC + ZMQ)

This is the "optionally add capabilities to stream contacts/forces" the user described. It is an
**additive payload + new capabilities**, not a new mode — consistent with the `EMjCapability`
model. Both transports carry the same msgpack fields, so it's transport-agnostic by construction
(the mirror subscribe stream already runs over gRPC *and* ZMQ).

**Design:**
- Two new capabilities alongside `stream_cameras` / `accept_input` (wire strings):
  `stream_contacts` and `stream_overlay_state`. Advertised in `fastpath_hello`, gated like the
  existing caps.
- **Append** optional keys to the existing transform frame (topic `geoms` / `view_frame`),
  so no new stream and no protocol fork:
  - `contacts`: packed array of `{pos[3], frame[9], dist, force[6], dim, g1, g2}` — unlocks
    contact point / force / split / friction / torque + contact frames. Highest value per byte.
  - `subtree_com`: `[3*nroot]` — CoM spheres.
  - `ctrl` / `act`: `[nu]` / `[na]` — actuator coloring.
  - `wrap_xpos` (+ `ten_wrapadr`): tendon wrap paths.
  - `xfrc_applied` is **already streamed** — PERTFORCE works on mirrors today once §2 lands.
- The owner computes these from `mjData` right after the step (it already computes a
  `RenderSnapshot`, `MjPhysicsEngine.cpp:1755`; Python owner already builds the frame in
  `fastpath_owner.publish_bodies`). Only serialize a key when its capability is granted, to keep
  the lean-mirror path cheap.
- The mirror, on receiving these, synthesizes the missing `mjvGeom`s itself (it has the static
  `mjModel`), or — simpler and more parity-safe — calls a small helper that appends the same
  primitives `mjv_addGeoms` would, using the streamed arrays. Either way it feeds the §2 renderer.

**Cost control:** contacts can be hundreds of entries; gate behind `stream_contacts`, cap count,
and only send on subscribers that asked. A peek that just wants qpos pays nothing.

---

## 5. Prerequisite fix: UE-owner drops forwarded drag intent (correctness)

Found in the modes audit: a **UE** owner's `HandleFastpathPerturb` reads only
`{body, force, torque}`, so a Mirror's `{select, active, localpos, refselpos}` drag intent →
zero wrench. Our new perturbation path therefore works Mirror→**Python**-owner but silently
no-ops Mirror→**UE**-owner. For "perturb in all modes" this must be fixed: teach the UE owner's
handler the intent shape and route it through `mjv_applyPerturbForce` (the UE owner already has
`UMjPerturbation` doing exactly this in-process). Small, self-contained; do it before/with the
overlay-streaming work since it's the same "works in all modes" theme.

Related cleanups the audit flagged (do opportunistically): the UE owner emits per-geom
`xpos/xquat` while Python emits per-body `bxpos/bxquat` (header calls per-body "preferred");
`xfrc_applied` comments say torque-first but code is force-first; capabilities have no launch
flag (can't disable from a headless command line).

---

## 6. Phasing

1. **Renderer core (§2)** — `UMjOverlayRenderer` rewrite: ISM pools + master materials +
   `mjv_updateScene` consumption on any local-`mjData` role (owner, in-process, peek). Delivers
   the full experience everywhere the data is already local. Replaces all `DrawDebug`.
2. **UE-owner perturb-intent fix (§5)** — closes the "perturb in all modes" gap.
3. **Mirror [XFORM]/[MODEL] overlays (§3)** — frames, camera/light, inertia, BVH on a
   transforms-only mirror (no new streaming).
4. **Streaming caps (§4)** — `stream_contacts` first (biggest unlock), then `subtree_com`,
   `ctrl/act`, `wrap_xpos`; unlocks the [OWNER] set on mirrors/peekers over gRPC and ZMQ.
5. **Wrench glyph (§2, optional)** — the net-new 6-D force+torque visualization.

Each phase is independently shippable and testable; phase 1 alone removes the ugly debug lines.
