# Fast-path render server: launch flags

An audit of every render-server command-line flag, what each does, and which you
actually need for the common cases. The old per-feature `-URLabFast*` scheme was
replaced by a compact **five-flag surface** (the "Render migration"); the former
names are kept below as `(formerly -URLabFast...)` lineage so migrating readers can
find them. Parsed in `MjRendererLauncher.cpp` (boot) and `MjRenderer.cpp` (runtime),
except the two editor-only browser flags noted at the end. SSOT for the grammar:
`Source/URLab/Public/MuJoCo/Fast/MjLauncherFlags.h`.

Since the render server now **spawns its renderer on demand** (the first
`fastpath_load` / client `load_xml` creates one when the booted level has none),
a model flag is no longer required. Boot a bare empty level and let the Python
client drive everything.

## Grammar

```
-URLabDrive=sim | stream:<endpoint> | push | await          # ONE value only (atomic axis)
-URLabSourceFind=discover[:scene] | browse
-URLabModel=<path.{mjb,xml,mjz}>                              # format from extension
-URLabCaps=serve,publish,cameras,input,vr                    # CSV; absent=lean; negatable -input
-URLabScene=level=<n>,origin=X;Y;Z,base,quality=off,cammax=N,camnear=<cm>,noexposure,overlay=<mask>,maxcontacts=<n>
-URLabNet=id=,index=,portbase=,stride=,step=,state=,bus=,cam=,grpc=,bind=
```

Plus the standard engine flags for a headless server: `-game -RenderOffScreen
-nosplash -unattended -stdout`.

## `-URLabDrive=` — pose-source axis (choose exactly ONE)

This axis is atomic: `push` and `await` are mutually exclusive, and only one
`-URLabDrive=` may appear.

| Value | Meaning | When you need it |
|-------|---------|------------------|
| `await` | Boot with **no** model: spawn a bare manager so the bridge + gRPC/ZMQ listeners come up immediately, then wait. The client connects, calls `load_*`, and the renderer is spawned on demand. Without this a model-less boot never starts a listener, so the client can't connect. **`serve` is not implicit here** — you MUST also pass `-URLabCaps=serve,...` or nothing binds `:50051`. (formerly `-URLabFastServe`.) | The client-driven flow (empty level, load over the wire). `await` is async-capable — the client's `delay=0` still yields exact forced frames. |
| `push` | Forced-render regime: cameras capture *only* on a `fastpath_render` request (manual capture), the primary view render is disabled, and frame-rate smoothing is off. Exact-fresh, lowest per-request latency. (formerly `-URLabFastForcedOnly`.) | Model-preloaded forced render server (eval / training frames). |
| `sim` | Step the model in-process (a full sim a client drives over RPC) instead of mirroring an owner's bus. (formerly `-URLabFastDirect`.) | In-process / stepped renderer. |
| `stream:<endpoint>` | Subscribe to an owner's bus and mirror it (one owner → many render/viewer instances). The endpoint scheme selects transport: `stream:tcp://host:port` for the ZMQ transform bus (formerly `-URLabFastBus=<endpoint>`), `stream:grpc://host:port` for gRPC (formerly `-URLabFastGrpcJoin=host:port`). The same form also boots a read-only **viewer** subscribing an owner's `viewer` bus (formerly `-URLabStateSource=<endpoint>` — keep the scheme the endpoint already has). | Mirror renderer or read-only viewer. |

## `-URLabSourceFind=` — owner discovery

| Value | Meaning |
|-------|---------|
| `discover[:scene]` | Headless farm node: auto-join the first (or scene-matching) advertised owner, no UI. (formerly `-URLabFastAutoJoin[=scene]` / `-URLabFastDiscover`.) |
| `browse` | Show the interactive server-browser UI (non-headless). (formerly `-URLabFastBrowser`.) |

## `-URLabModel=<path>` — model source (optional)

One `-URLabModel=`; format is taken from the extension. Omit it to boot an empty
level and load from the client instead (the recommended flow).

| Extension | Meaning |
|-----------|---------|
| `.xml` | Boot model from MJCF XML; compiled in-engine (immune to MJB version skew). (formerly `-URLabFastXml=<file>`.) |
| `.mjz` | Boot model from a `.mjz` archive (root `model.xml`), compiled in-engine. (formerly `-URLabFastMjz=<file>`.) |
| `.mjb` | Boot model from a compiled MJB (must match the server's MuJoCo version). (formerly `-URLabFastMjb=<file>`.) |

## `-URLabCaps=` — capability CSV (absent ⇒ lean; `-input` negates)

Merge every capability into this one CSV flag (e.g. cameras + vr →
`-URLabCaps=cameras,vr`).

| Token | Meaning | When you need it |
|-------|---------|------------------|
| `serve` | Bring up the bridge + gRPC/ZMQ listeners so clients can connect. **Required on the `await` path** (not implicit there); implicit for `push` / `stream` / `vr` but harmless — include it for clarity. | Any client-driven (`await`) render server. |
| `cameras` | Build + stream the camera components. Without it the renderer has no capturing cameras and `fastpath_render` returns nothing. (formerly `-URLabFastCameras`.) | **Always**, for any render server. |
| `vr` | Spawn + possess a free-fly **drone** camera (WASD + Q/E + mouse; Shift boosts) to fly around the viewed sim. Non-headless. (formerly `-URLabVrViewer`.) | VR / spectator viewer. |
| `publish` | Re-broadcast raw kinematics on the `viewer` bus so viewers can subscribe (the `StreamCameras`/broadcast side; cf. the owner's `-URLabBroadcastViewers`). | When viewers subscribe to this instance. |
| `input` | Accept forwarded drag / perturb intent (`AcceptInput`). Negate with `-input`. | Interactive drag against this instance. |

## `-URLabScene=` — scene tuning / placement CSV

Merge every scene key into this one CSV flag (e.g. cammax + base →
`-URLabScene=cammax=0,base`).

| Key | Meaning |
|-----|---------|
| `level=<n>` | Level to open (for the discover / auto-join node). (formerly `-URLabFastLevel=<level>`.) |
| `origin=X;Y;Z` | UE-cm world offset applied to all geoms/cameras/user-cam, for tiling several scenes in one level. **Note `;` separators** (was `,`). (formerly `-URLabFastOrigin=X,Y,Z`.) |
| `base` | The boot map is a curated scene (its own lights/sky/floor); skip importing the model's environment on top of it. (formerly `-URLabFastBaseLevel`.) |
| `quality=off` | Skip `ApplyRendererQuality` (leave engine scalability settings untouched). (formerly `-URLabFastNoQuality`.) |
| `cammax=<n>` | Cap each model camera's capture height (keeps aspect). Caps render cost; `0` = uncapped. Does not affect the `user` camera. (formerly `-URLabFastCamMaxHeight=<n>`.) |
| `camnear=<cm>` | Camera near-clip plane, in cm. (formerly `-URLabCamNearClip=<cm>`.) |
| `noexposure` | Disable auto-exposure. (formerly `-URLabDisableAutoExposure`.) |
| `overlay=<mask>` | Visualization-overlay bitmask (see `vis_overlays_plan.md`). |
| `maxcontacts=<n>` | Cap on contact points drawn. |

## `-URLabNet=` — networking CSV

Merge every net key into this one CSV flag.

| Key | Meaning |
|-----|---------|
| `grpc=<n>` | gRPC (dm_env_rpc) listen port. Default `50051`. Set a distinct value per instance to run **several render servers on one host** (the `RenderPool` case); pair with a distinct `index=` so the ZMQ ports don't collide either. Cross-host instances can leave it at the default. (formerly `-URLabDmEnvPort=<n>`.) |
| `index=<n>` | Instance index; offsets the ZMQ port block so co-hosted instances don't collide. (formerly `-URLabInstanceIndex=<n>`.) |
| `id=`, `portbase=`, `stride=`, `step=`, `state=`, `bus=`, `cam=`, `bind=` | Identity, ZMQ port-block base/stride, and the individual step/state/bus/cam port and bind-address overrides. |

## Owner re-broadcast

| Flag | Meaning |
|------|---------|
| `-URLabBroadcastViewers=1` | On an **owner**, re-broadcast raw kinematics (`{t,qpos,qvel}`) on the `viewer` bus (ZMQ + gRPC `subscribe_viewer`) so viewers can subscribe. (Unchanged by the migration.) |

## Joining an owner (discover / directed connect)

The `URLabEditor` server browser has GUI actions for these; the CLI equivalents are
the same runtime flags a headless renderer uses (the legacy `-URLabFast*` spellings
below are deleted):

- **Discover** an advertised owner in the registry: `-URLabSourceFind=discover[:scene]`
  (formerly `-URLabFastDiscover` / `-URLabFastAutoJoin`).
- **Connect** to a specific owner by its control endpoint: `-URLabDrive=stream:<control>`
  with no `-URLabModel` — the renderer pulls the model over the control channel
  (formerly `-URLabFastConnect=<control>`; see `MjLauncherFlags.h` / `MjLevelOps` where
  a bare `-URLabDrive=stream:<ctrl>` is documented as the `-URLabFastConnect` replacement).
- **Browse** (interactive picker): `-URLabSourceFind=browse` (formerly `-URLabFastBrowser`).

## Cheat sheet

Pure render server (forced / eval), empty level, client loads the model:

```
UnrealEditor URLabTest.uproject /Game/FastPath/FastPathRender -game \
  -URLabDrive=await -URLabCaps=serve,cameras \
  -RenderOffScreen -nosplash -unattended -stdout
```

Smooth viewer server (streaming): the same launch — `await` is async-capable — but
have the client pull frames with `delay>0` (show frame N-1 while N renders) instead
of `delay=0`.
