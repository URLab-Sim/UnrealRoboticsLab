# MJB fast-path renderer + render server

> **STALE for roles/modes.** This page still uses the old `AMjbScene` /
> `EMjbRunMode::Puppet` / "slave" vocabulary. For the authoritative, code-verified
> role & mode model (owner / mirror / render server / viewer / peek / VR),
> the `EMjPoseSource` / `EMjCapability` axes, the bus wire formats, and the
> perturbation paths, see [`render_roles.md`](./render_roles.md). The run/demo
> mechanics below are kept for reference but the architecture section is superseded.

The fast path renders a MuJoCo scene in Unreal **straight from a compiled MJB**,
running **no physics**. An OWNER (a Python puppet client, or a UE instance in
live/direct mode) steps the sim and broadcasts per-geom world transforms; a
RENDERER (`AMjbScene`) loads the MJB, builds one lightweight actor per body, and
mirrors the stream. A renderer can also act as a **render server**, spawning the
MJB's cameras and streaming their frames over ZMQ / shared memory (and ROS).

- No `UMjArticulation` / ProtoSpec / Blueprint tree, no `mjData`, no stepping on
  the renderer.
- Zero-config connect: an owner **advertises** itself in a registry; the renderer
  **discovers** it and pulls the MJB **over the wire** (no shared file).

## Contents
1. [Architecture](#architecture)
2. [Prerequisites](#prerequisites)
3. [Build the plugin](#build-the-plugin)
4. [Build `mjbcompile`](#build-mjbcompile)
5. [Run the demo](#run-the-demo)
6. [CLI flags & the server browser](#cli-flags--the-server-browser)
7. [What you should see](#what-you-should-see)
8. [Troubleshooting](#troubleshooting)
9. [File map](#file-map)

---

## Architecture

```
 OWNER (steps physics)                         RENDERER (no physics)
 ─────────────────────                         ─────────────────────
 Python: run_fastpath_demo.py                  UE: AMjbScene
   or a UE live/direct instance                  discovers owner (registry)
                                                  pulls MJB over control channel
   registry entry  ── advertise ─────────────▶   loads mj_loadModelBuffer
   control REQ/REP (fastpath_hello) ──MJB────▶    builds per-body actors
   geoms PUB bus  ── xpos/xquat/cxpos/cxquat ─▶   applies transforms each frame
                                            ◀──   perturbation (force) REQ
                                                  cameras ── ZMQ/SHM/ROS ─▶ clients
```

- **Discovery**: owners write a JSON entry into a shared registry directory
  (`$URLAB_REGISTRY_DIR`, else `~/.cache/URLab/registry`) with a `fastpath_owner`
  capability, a `control` endpoint, and a `bus` endpoint.
- **Control channel** (ZMQ REQ/REP): the renderer sends `fastpath_hello`; the
  owner replies with the MJB bytes + the transform-bus endpoint. Perturbations go
  back the same channel (`fastpath_perturb`).
- **Transform bus** (ZMQ PUB, topic `geoms`): msgpack
  `{f, xpos[3*ngeom], xquat[4*ngeom], cxpos[3*ncam], cxquat[4*ncam]}` (camera
  fields optional). Applied on the game thread.
- **Version lock**: the MJB is version-locked to the plugin's `libmujoco`
  (3.11.1). The transform stream is version-independent (indexed floats), so the
  owner can *step* with pip `mujoco` 3.11.0 as long as the MJB it *serves* was
  compiled by the 3.11.1 lib (see [`mjbcompile`](#build-mjbcompile)).

---

## Prerequisites

- **Unreal Engine** source build with this plugin (Linux instructions here; the
  paths differ on other OSes).
- A UE **project** that enables the plugin (the dev setup uses `URLabTest`).
- **`uv`** (`pip install uv`) — the demo runs Python in throwaway envs.
- **MuJoCo menagerie** cloned as a sibling of the repos:
  `git clone https://github.com/google-deepmind/mujoco_menagerie`.
- A GPU — camera capture renders real frames (a headless `-nullrhi` run can't).

Assumed layout (override with env vars, see below):

```
<root>/
  UnrealRoboticsLab/     this plugin
  URLab_Bridge/          the Python bridge + demo scripts
  mujoco_menagerie/      scenes
  mjb_test/mjbcompile    the xml->mjb tool you build once (below)
  URLabTest/             a UE project that enables the plugin
```

---

## Build the plugin

Rebuild the editor target of your project after pulling this branch:

```bash
<UE>/Engine/Build/BatchFiles/Linux/Build.sh \
  URLabTestEditor Linux Development \
  -Project=<root>/URLabTest/URLabTest.uproject -WaitMutex
```

---

## Build `mjbcompile`

`mjbcompile` turns a `scene.xml` into a **version-matched MJB** using the
plugin's own `libmujoco`. You build it once per machine. Source:
`Scripts/fastpath/mjbcompile.cpp`.

The catch: `libmujoco.so` is built against **UE's clang libc++** (so it has
undefined `__cxa_*` / `std::bad_alloc` symbols) and carries a **stale RUNPATH**
into UE's toolchain lib64 (old glibc). So: link UE's clang **static** libc++ /
libc++abi, allow the shared-lib undefined symbols, and at **run** time put the
**system** libdir ahead of the stale RUNPATH.

```bash
# Point these at your machine. The clang toolchain dir version (v26_clang-...)
# will differ; find it under Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux.
UEC=<UE>/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/<clang-ver>/x86_64-unknown-linux-gnu
MJ=<root>/UnrealRoboticsLab/third_party/install/MuJoCo

"$UEC/bin/clang++" -O2 -std=c++17 \
  <root>/UnrealRoboticsLab/Scripts/fastpath/mjbcompile.cpp \
  -I"$MJ/include" -L"$MJ/lib" -lmujoco \
  "$UEC/lib64/libc++.a" "$UEC/lib64/libc++abi.a" \
  -Wl,--allow-shlib-undefined -Wl,-rpath,"$MJ/lib" \
  -o <root>/mjb_test/mjbcompile

# Test (system libdir FIRST so libmujoco gets a new-enough glibc):
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:"$MJ/lib" \
  <root>/mjb_test/mjbcompile <root>/UnrealRoboticsLab/Scripts/fastpath/../../mjb_test/primitives.xml /tmp/x.mjb
# -> ok: ... ver=3011001
```

`run_fastpath_demo.py` invokes `mjbcompile` with the right `LD_LIBRARY_PATH`
automatically; you only need it built and reachable (see env vars below).

---

## Run the demo

One command (from the bridge repo). Defaults to the camera-rich **aloha** scene:

```bash
bash <root>/URLab_Bridge/scripts/run_fastpath.sh
# or a specific scene:
bash <root>/URLab_Bridge/scripts/run_fastpath.sh <root>/mujoco_menagerie/<pkg>/scene.xml
```

It (1) starts the MuJoCo owner (native viewer + broadcast), (2) waits for it to
advertise, (3) opens an OpenCV camera-viewer, and (4) launches the UE editor as a
render server that auto-discovers the owner. **Then press PLAY in UE.**

### Machine-specific paths (env vars)

`run_fastpath.sh` auto-detects `<root>` as two levels above itself. Override any
of these if your layout differs:

| var | meaning | default |
|-----|---------|---------|
| `URLAB_ROOT` | the root holding all the sibling dirs | script's `../..` |
| `UE_EDITOR` | UnrealEditor binary | `$HOME/UnrealEngine/Engine/Binaries/Linux/UnrealEditor` |
| `UE_PROJECT` | `.uproject` that enables the plugin | `$URLAB_ROOT/URLabTest/URLabTest.uproject` |
| `URLAB_MJBCOMPILE` | the `mjbcompile` binary | `$URLAB_ROOT/mjb_test/mjbcompile` |
| `URLAB_MJLIB` | the plugin's MuJoCo `lib/` | `.../third_party/install/MuJoCo/lib` |
| `BUS_PORT` / `CAM_BASE` | transform-bus port / camera base port | `5561` / `5600` |

### Run it by hand (if you prefer)

```bash
# Owner (viewer + broadcast + serves the MJB):
cd <root>/URLab_Bridge
URLAB_ROOT=<root> uv run --no-project --with mujoco --with pyzmq --with msgpack --with numpy \
  --python 3.11 python scripts/run_fastpath_demo.py <root>/mujoco_menagerie/aloha/scene.xml

# UE renderer + render server (separate terminal):
<UE>/.../UnrealEditor <root>/URLabTest/URLabTest.uproject -URLabSourceFind=discover -URLabCaps=cameras

# Camera feeds (separate terminal, after pressing Play in UE):
cd <root>/URLab_Bridge
uv run --no-project --with opencv-python --with pyzmq --with numpy --python 3.11 \
  python scripts/show_fastpath_cameras.py --base-port 5600
```

---

## CLI flags & the server browser

UE editor launch flags (read by `UMjbFastPathEditorLauncher`):

| flag | effect |
|------|--------|
| `-URLabSourceFind=discover` | find the first advertised owner and connect. (formerly `-URLabFastDiscover`) |
| `-URLabDrive=stream:tcp://host:port` (no `-URLabModel`) | connect to a specific owner control endpoint and pull its model over the control channel. (formerly `-URLabFastConnect=tcp://host:port`) |
| `-URLabModel=/path.mjb -URLabDrive=stream:tcp://host:port` | connect with no discovery (explicit). (formerly `-URLabFastMjb=/path.mjb -URLabFastBus=tcp://host:port`) |
| `-URLabCaps=cameras` | spawn the MJB's cameras and stream them (render server). (formerly `-URLabFastCameras`) |

**Server browser** (interactive alternative to the CLI): in the editor, open the
**"Fast-Path Servers"** panel — the **"Fast-Path" toolbar button** (URLab
section, near Play) or **Window ▸ Fast-Path Servers**. It lists advertised owners
with a **Connect** button.

Streaming (transforms **and** cameras) is a **play-session** behaviour: the
editor shows a static rest-pose preview; press **Play (PIE)** to start the live
stream. The scene persists and is saveable; body/geom actors are tagged
(`MjbBody=<id>`, `MjbGeom=<id>`) so `ReindexFromLevel` can reconnect a saved
scene without rebuilding.

---

## What you should see

Running the aloha demo and pressing Play:

- The native **MuJoCo viewer** (owner, ground truth) with the arms moving.
- The **UE PIE viewport** puppeting the same motion, materials + textures from
  the MJB.
- **Six OpenCV windows** (aloha's cameras): `overhead_cam`, `worms_eye_cam`, two
  POV cams, and `wrist_cam_left`/`wrist_cam_right` — the wrist cams track the
  moving arms (proves camera-transform streaming).

Camera height is capped to 480px by default (`AMjbScene::CameraMaxHeight`) so many
full scene captures stay smooth; raise it on the actor for full resolution.

---

## Troubleshooting

- **PIE shows a static scene / no camera windows.** The renderer didn't rebuild
  on Play. Open **Window ▸ Output Log**, filter `[MjbScene]`, and read the
  `BeginPlay` line — it prints the MJB file/bytes, bus endpoint, and camera flag.
  A rebuild needs a reachable MJB (the wire MJB is cached to
  `Saved/URLab/fastpath_wire.mjb`).
- **`mj_loadModelBuffer failed (version-mismatched MJB?)`.** The owner served an
  MJB compiled by a different MuJoCo version. Rebuild `mjbcompile` against *this*
  plugin's `libmujoco` and make sure the owner uses it.
- **`mjbcompile` fails at runtime** with `undefined symbol: __cxa_*` or
  `GLIBC_2.29 not found`. You skipped the static libc++ archives, or didn't put
  the system libdir first at run time. Re-read [Build `mjbcompile`](#build-mjbcompile).
- **Owner never advertises.** Check `~/.cache/URLab/registry/fastpath_*.json`
  exists; is `uv` / `mujoco` installed? Set `URLAB_REGISTRY_DIR` if you use a
  custom location (same var on both sides).
- **Renderer can't reach the owner across machines.** The registry advertises the
  owner's hostname; ensure it resolves and the ports (control, bus, cameras) are
  reachable. Same-host, everything is `127.0.0.1`.
- **UE as the owner** (live/direct): launch it with `-URLabBroadcastViewers=1`;
  it then advertises `fastpath_owner`, answers `fastpath_hello`, and publishes the
  `geoms` bus, so a fast-path renderer can puppet from it exactly like from
  Python.

---

## File map

Plugin (`UnrealRoboticsLab`):
- `Source/URLab/{Public,Private}/MuJoCo/Fast/MjbScene.*` — the renderer actor
  (load, build, materials/textures, streaming, cameras, perturbation, re-index).
- `Source/URLab/Private/MuJoCo/Fast/MjbFastPathLauncher.cpp` — `-game` launcher.
- `Source/URLabEditor/Private/MjbFastPathEditorLauncher.cpp` — editor CLI launcher.
- `Source/URLabEditor/{Private,Public}/MjLevelOps.*` — discovery + connect ops.
- `Source/URLabEditor/{Private,Public}/SMjbServerBrowser.*` — the server-browser panel.
- `Source/URLab/Private/Bridge/RpcHandlers_Fastpath.cpp` — UE-owner `fastpath_hello`.
- `Source/URLab/Private/MuJoCo/Core/AMjManager.cpp` — UE-owner `geoms` broadcast.
- `Scripts/fastpath/mjbcompile.cpp` — the xml→mjb tool.

Bridge (`URLab_Bridge`):
- `src/urlab_client/fastpath_owner.py` — the `FastPathOwner` (advertise + serve + broadcast).
- `scripts/run_fastpath_demo.py` — the owner demo (sim + viewer + broadcast).
- `scripts/show_fastpath_cameras.py` — the OpenCV camera-feed viewer.
- `scripts/run_fastpath.sh` — one-command orchestration.
