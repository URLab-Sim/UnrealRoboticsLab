# Fast-path render server: `-URLabFast*` launch flags

An audit of every `-URLabFast*` command-line flag the render server understands,
what each does, and which you actually need for the common cases. Parsed in
`MjRendererLauncher.cpp` (boot) and `MjRenderer.cpp` (runtime), except the two
editor-only browser flags noted at the end.

Since the render server now **spawns its renderer on demand** (the first
`fastpath_load` / client `load_xml` creates one when the booted level has none),
a model flag is no longer required. Boot a bare empty level and let the Python
client drive everything.

## The essentials

| Flag | Meaning | When you need it |
|------|---------|------------------|
| `-URLabFastServe` | Boot with **no** model: spawn a bare manager so the bridge + gRPC/ZMQ listeners come up immediately, then wait. The client connects, calls `load_*`, and the renderer is spawned on demand. Without it a model-less boot never starts a listener, so the client can't connect. | The client-driven flow (empty level, load over the wire). |
| `-URLabFastCameras` | Build + stream the camera components. Without it the renderer has no capturing cameras and `fastpath_render` returns nothing. | **Always**, for any render server. |
| `-URLabFastForcedOnly` | Forced-render regime (mode 1): cameras capture *only* on a `fastpath_render` request (manual capture), the primary view render is disabled, and frame-rate smoothing is off. Exact-fresh, lowest per-request latency. Omit it for the smooth streaming/viewer regime (mode 2). | Pure render server (eval / training frames). Omit for viewer mode. |

Plus the standard engine flags for a headless server: `-game -RenderOffScreen
-nosplash -unattended -stdout`.

## Model source (all optional now)

Provide **at most one**. Omit all three to boot an empty level and load from the
client instead (the recommended flow).

| Flag | Meaning |
|------|---------|
| `-URLabFastXml=<file>` | Boot model from MJCF XML; compiled in-engine (immune to MJB version skew). |
| `-URLabFastMjz=<file>` | Boot model from a `.mjz` archive (root `model.xml`). |
| `-URLabFastMjb=<file>` | Boot model from a compiled MJB (must match the server's MuJoCo version). |

## Tuning / placement (optional)

| Flag | Meaning |
|------|---------|
| `-URLabDmEnvPort=<n>` | gRPC (dm_env_rpc) listen port. Default `50051`. Set a distinct value per instance to run **several render servers on one host** (the `RenderPool` case); pair with a distinct `-URLabInstanceIndex=` so the ZMQ ports don't collide either. Cross-host instances can leave it at the default. |
| `-URLabFastCamMaxHeight=<n>` | Cap each model camera's capture height (keeps aspect). Caps render cost; `0` = uncapped. Does not affect the `user` camera. |
| `-URLabFastOrigin=X,Y,Z` | UE-cm world offset applied to all geoms/cameras/user-cam, for tiling several scenes in one level. |
| `-URLabFastBaseLevel` | The boot map is a curated scene (its own lights/sky/floor); skip importing the model's environment on top of it. |
| `-URLabFastNoQuality` | Skip `ApplyRendererQuality` (leave engine scalability settings untouched). |

## Other modes (not the Python-driven render server)

| Flag | Meaning |
|------|---------|
| `-URLabFastDirect` | Step the model in-process (a full sim a client drives over RPC) instead of mirroring an owner's bus. |
| `-URLabFastBus=<endpoint>` | Subscribe to an owner's transform bus and mirror it (one owner → many render/viewer instances). |
| `-URLabFastAutoJoin[=scene]` | Headless farm node: auto-join the first (or scene-matching) owner, no UI. |
| `-URLabFastLevel=<level>` | Level to open for `-URLabFastAutoJoin`. |
| `-URLabFastBrowser` | Show the interactive server-browser UI (non-headless). |

## Editor-only (not runtime)

These live in the `URLabEditor` module and drive the in-editor server browser;
they are not part of a packaged/headless render server:

- `-URLabFastConnect=<control>` / `-URLabFastDiscover` — CLI equivalents of the
  editor's server-browser connect/discover actions.

## Cheat sheet

Pure render server (mode 1, forced), empty level, client loads the model:

```
UnrealEditor URLabTest.uproject /Game/FastPath/FastPathRender -game \
  -URLabFastServe -URLabFastForcedOnly -URLabFastCameras \
  -RenderOffScreen -nosplash -unattended -stdout
```

Smooth viewer server (mode 2, streaming): drop `-URLabFastForcedOnly`, and have
the client pull frames with `delay>0`.
