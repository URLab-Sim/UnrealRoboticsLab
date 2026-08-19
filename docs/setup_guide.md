# URLab render — setup guide

Three ways to use the URLab render stack. Each is self-contained; start with the one
you need.

1. **Pure render server** — a cooked, headless Unreal server that renders MuJoCo
   cameras on demand, driven from Python. The main path.
2. **Mirror viewer** — the same server plus a live OpenCV window, for watching a scene.
3. **gRPC / dm_env transport** — route the server over DeepMind `dm_env_rpc` instead of
   ZMQ (Linux).

Detailed references: `docs/render_server_packaging.md` (cook + flags),
`docs/grpc_dmenv_setup.md` (gRPC), and the bridge's `docs/render_server.md` (client API).

---

## 1. Pure render server

Python owns the physics (a *puppet*); Unreal is a *mirror* that renders the pushed
state and returns camera images. Python never blocks on the engine except to wait for a
frame.

### 1.1 Cook the server (once per C++ change)

```powershell
& "$Engine\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun `
  -project="$Project\url_proj.uproject" -noP4 -platform=Win64 `
  -clientconfig=Development -cook -build -stage -pak -iostore -prereqs -nodebuginfo -utf8output
```
Output: `Saved/StagedBuilds/Windows/url_proj.exe`.

### 1.2 Compile a version-matched model

The server loads a `.mjb` whose MuJoCo version matches Unreal's. Run `mjbcompile` from
the scene directory so mesh/texture paths resolve:

```bash
mjbcompile scene.xml scene.mjb      # prints ver=3011001 — must match the UE MuJoCo
```
Camera resolution comes from `<camera resolution="W H">`; change it and recompile — no
recook (the launch uses `-URLabFastCamMaxHeight=0`, which honours the model).

### 1.3 Launch (headless)

```bash
url_proj.exe /Game/FirstPerson/Lvl_FirstPerson \
  -URLabFastMjb=scene.mjb -URLabFastCameras -URLabFastForcedOnly \
  -URLabFastCamMaxHeight=0 -RenderOffScreen -nosplash -abslog=server.log
```
| flag | meaning |
|------|---------|
| `<map>` | a **lit** level (SkyLight + reflection captures). Unlit maps (`/Engine/Maps/Entry`) leave metallics flat. |
| `-URLabFastMjb=<file>` | model to load |
| `-URLabFastForcedOnly` | forced/eval regime — the bridge serves `fastpath_render` |
| `-URLabFastCameras` | enable camera capture |
| `-URLabFastCamMaxHeight=N` | camera height cap; `0` = honour the model resolution |
| `-RenderOffScreen` | headless |

Wait for `camera server: N camera(s) streaming` in the log, then the bridge step port
(default 5559) is live.

### 1.4 Drive it from Python

```python
import mujoco
from urlab_client import RenderClient

model = mujoco.MjModel.from_xml_path("scene.xml")   # the model the server loaded
data = mujoco.MjData(model)
with RenderClient("tcp://127.0.0.1", step_port=5559) as rc:
    names = rc.camera_names()
    for _ in range(100):
        mujoco.mj_step(model, data)
        frames = rc.render_mjdata(model, data, cameras=[names[0]])   # fresh, 1 camera
        bgr = frames[names[0]].to_bgr()                              # HxWx3 for cv2/save
```

**Fresh vs delayed** — `render(..., delay=0)` blocks until the exact pushed state is
rendered (eval/exact-state). `delay=N` returns an N-substep-stale frame (faster; matches
real-camera latency for training data). Rendering **one** named camera is far cheaper
than all of them (each is a full scene capture).

---

## 2. Mirror viewer (live window)

The runnable example opens an OpenCV window that steps physics, pushes poses, force-
renders, and shows the camera live:

```bash
# launch the server as in 1.3, then, from the URLab_Bridge dir:
uv run python examples/render_client_example.py --xml scene.xml --camera 0 --show
uv run python examples/render_client_example.py --xml scene.xml --show --delay 2   # stale/fast
```

For a real-time-paced view, step the physics to wall-clock and apply your own control
before each render (see the example's loop). Two fidelity notes:

- **Use a lit level** (§1.3) or metallics look flat.
- **Do not force low scalability for viewing.** `-ExecCmds="sg.ShadowQuality 0;
  sg.PostProcessQuality 0"` (used only for comparable perf benchmarks) sets
  `sg.ReflectionQuality 0`, which turns Lumen reflections and SSR **off** — metallic PBR
  then reflects nothing and looks flat. For a viewer, run full scalability
  (`sg.PostProcessQuality 3; sg.ReflectionQuality 3; r.SSR.Quality 3`).

A fresh-blocking viewer is inherently slower than real time when render time ≥ timestep;
for a smooth real-time viewer use `delay>0` (show frame N-1 while N renders) or render
locally with the MuJoCo viewer.

---

## 3. gRPC / dm_env transport (Linux)

Route the bridge over DeepMind `dm_env_rpc` instead of ZMQ. The `URLabDmEnvRpc` module is
a normal transport-role backend: at startup it binds the control-RPC factory, so the
bridge speaks gRPC. Requests ride the proto's `extension` field (a `UrlabPacket` we
unpack ourselves) and forward through the same dispatch as ZMQ — so every op, including
`fastpath_render`, works over gRPC unchanged.

It is **Linux-only** (`PlatformAllowList: ["Linux"]`); the Windows editor/cook skip it.

### 3.1 Build gRPC (one time)

```bash
cd third_party/grpc && ./build.sh     # clones gRPC v1.62.0, builds static libs
                                      # into third_party/install/grpc  (~15-40 min)
```
(`build.ps1` is the Windows equivalent — the module is Linux-only, but the script builds
a `/MD` static gRPC for experimentation. The Windows module build is currently blocked on
a proto-enum / windows.h typedef clash; the Linux build is the supported path.)

### 3.2 Build + run on Linux

With `third_party/install/grpc` present, a normal Linux build compiles the module and
links the static gRPC. A company `dm_env_rpc` client then drives the render server over
gRPC with our payload tunnelled in the extension field.

See `docs/grpc_dmenv_setup.md` for detail. Native `step` mapping (transforms → action
Tensors, images → observation Tensors) is a planned enhancement; today everything routes
through the extension tunnel.
