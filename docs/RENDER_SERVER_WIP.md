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

## Small things still open

- **Packaged re-cook**: the current cooked exe predates the PBR fixes and only has
  the bare-plane environment. Re-cook to pick up the PBR fixes and to include any
  custom `/Game` levels in the browser dropdown (GateBackyard-scale envs are heavy).
- **Aloha normal-import smoothing** (task #11, still open): meshes import
  over-smoothed on the normal articulation route. `Scripts/clean_meshes.py`
  does crease-split per mesh (`split_normals_by_crease`, crease dot 0.8), so the
  likely cause is the split being skipped/lost, not missing logic: (a) `networkx`
  absent -> `clean_mesh` raises -> GLB skipped -> UE computes smooth normals, or
  (b) a stale GLB not regenerated. Needs a repro of the aloha import to confirm
  which. Tabletop-texture drop on that route is the paired half.
- **Owner robustness**: the owner exits if its MuJoCo viewer window is closed.
- **Optional polish**: per-level default env+origin in the browser; direct
  origin type-in in the HUD; a camera-feed viewer wired into the browser flow.
