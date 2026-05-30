# Python Getting Started

This guide walks through the typical URLab Python workflow end-to-end.
It assumes the URLab Unreal plugin is already installed (see
[Getting Started](../getting_started.md)) and you are ready to drive the
simulation from Python via [urlab_bridge](https://github.com/URLab-Sim/urlab_bridge).

## Minimal example

The shortest end-to-end use of the API. Connect to a running editor,
start PIE, step the sim, send control, read state, close cleanly:

```python
from urlab_client import URLabClient

# Connect to the editor's ZMQ step server (default tcp://localhost:5559).
# `with` ensures the transport closes even on exceptions.
with URLabClient("tcp://localhost", step_mode="direct") as client:
    client.discover()                       # handshake + load MJB
    client.sim.start()                      # enter PIE; raises URLabPIEError
                                            # if compile fails or PIE times out

    robot = client.articulations["robot"]   # keyed by MJCF model name
    for _ in range(1000):
        robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})
        client.step(n_steps=1)              # advance 1 physics step
        # State mirrors locally: art attributes refresh on every step.
        print(robot.root_pos_w, robot.dof_qpos[:2])

    client.sim.stop()                       # leave PIE
```

That's the entire happy path. Read the rest of this guide for what
each call actually does, what other namespaces (`scene`, `runtime`,
`outliner`, `recording`, `replay`) cover, and how to compose them.

For the full method-by-method reference, see
[Python API Reference](api.md).

---

## Contents

1. [Install](#install)
2. [Connect](#connect)
3. [Pick a transport](#pick-a-transport)
4. [Author a scene](#author-a-scene)
5. [Run PIE and discover](#run-pie-and-discover)
6. [Step the sim](#step-the-sim)
7. [Read state and sensors](#read-state-and-sensors)
8. [Send control](#send-control)
9. [Tune controllers](#tune-controllers)
10. [Cameras](#cameras)
11. [Recording and replay](#recording-and-replay)
12. [Add a new policy](#add-a-new-policy)
13. [Pick a step mode](#pick-a-step-mode)
14. [Common pitfalls](#common-pitfalls)

---

## Install

The bridge is a separate repository. Clone and install:

```bash
git clone https://github.com/URLab-Sim/urlab_bridge.git
cd urlab_bridge
uv sync                          # core install (pyzmq, numpy, mujoco, dearpygui)
uv sync --extra policy           # optional: torch, onnxruntime for pretrained policies
```

Or with pip:

```bash
pip install -e ./urlab_bridge
```

`mujoco` is required for state mirroring and any client-side IK / MPC.
The bridge pins it to a version that matches the URLab plugin build —
the handshake refuses on mismatch.

---

## Connect

The Unreal editor (or PIE / standalone) listens on `tcp://localhost:5559`
by default. Open a session:

```python
from urlab_client import URLabClient

with URLabClient("tcp://localhost") as client:
    client.discover()
    print(client.urlab_version, client.mujoco_version)
    print("manager_present:", client.manager_present)
```

Two states matter:

- **`manager_present is False`** — the editor is open but PIE is not
  running. Editor-time namespaces (`client.scene.*`,
  `client.outliner.*`, `client.debug.*`, `client.viewport.*`) are
  valid; `client.runtime.*` is not.
- **`manager_present is True`** — PIE (or standalone) is live. All
  namespaces are valid.

The transition between the two is `client.sim.start()` → see
[Run PIE](#run-pie-and-discover).

---

## Pick a transport

URLab ships two transports. The default works everywhere; the alternative
is faster on the same machine.

### ZMQ (default) — cross-host

```python
client = URLabClient("tcp://localhost", transport="zmq")
```

TCP REQ/REP plus PUB/SUB streams. Point `address` at any reachable host.
This is the only path that works between machines.

### SHM — same host, lower jitter

```python
client = URLabClient("tcp://localhost", transport="shm")
```

Shared-memory ring buffers with kernel-event signalling. The handshake
still goes over ZMQ (and so do oversize RPCs), but everything else moves
through `<ProjectSavedDir>/URLabShm/<session>/`. Tail latency is tighter
than ZMQ; the difference is most visible on camera streams.

### Recommendation

| | ZMQ | SHM |
|---|---|---|
| Different host from UE | yes | no |
| Same host as UE | works | **prefer this** |
| Camera-stream jitter | higher | noticeably lower |
| Setup | `transport="zmq"` (default) | `transport="shm"` |

If you're running Python and UE on the same workstation, use SHM. The
[Python API Reference](api.md#transports) covers the fallback
behaviour and the perf table is in
[Networking](../guides/networking.md#performance-ballpark).

---

## Author a scene

Editor-time, before PIE, you can build a level entirely from Python.
Each `URLabAsset` is one MJCF file — the bridge imports it as a Blueprint
and spawns one actor per asset.

```python
from urlab_client import URLabAsset

assets = [
    URLabAsset(
        actor_id="robot_0",
        xml="C:/path/to/robot.xml",
        location=(0, 0, 0),
    ),
    URLabAsset(
        actor_id="prop_0",
        xml="C:/path/to/prop.xml",
        location=(1.0, 0, 0.5),
        rotation_euler=(0, 0, 90),  # degrees
    ),
]

handles = client.scene.apply_scene("Level_Sim", assets, save=True)
print(handles["robot_0"].actor_path)
```

`apply_scene` is a composer for the underlying primitives. If you need
finer control, call them directly. `import_xml` returns a typed
`URLabBlueprint` that plugs straight into `spawn_actor`:

```python
client.scene.create_level("Level_Sim")    # or load_level if it already exists
bp = client.scene.import_xml("C:/path/to/robot.xml")
handle = client.scene.spawn_actor(
    blueprint=bp,                 # URLabBlueprint or class-path string
    actor_id="robot_0",
    location=(0, 0, 0),
)
client.scene.save_level()
```

`actor_id` is the **stable handle** the bridge uses to reconcile a UE
actor across PIE restarts. It survives renames; UE actor names don't.

### Lights

```python
light = client.scene.spawn_light(
    kind="directional",
    actor_id="key",
    rotation_euler=(-45, 30, 0),
    intensity=10000.0,
    color=(1.0, 0.95, 0.85),
)
```

Lights are UE actors only — they don't appear in `client.entities` after
PIE.

### Modify the scene

After spawning, you can move, transform, and destroy actors by their
`actor_id`:

```python
# Move
client.scene.set_actor_transform(
    "robot_0",
    location=(2, 0, 0),
    rotation_euler=(0, 0, 90),
)

# Or by UE actor name
client.scene.set_actor_transform("BP_Robot_C_0", by_name=True, location=(2, 0, 0))

# Remove
client.scene.destroy_actor("prop_0")
```

`apply_scene` is **idempotent** — calling it twice with the same
`actor_id` updates the existing actor's transform / light parameters in
place rather than spawning a duplicate. The
[`URLabSpawnHandle.was_existing`](api.md#urlabspawnhandle) flag
tells you whether a given call hit an existing actor or created a fresh
one. Multi-instance scenes still work because each instance picks its
own `actor_id`.

### Inspect the world

```python
for actor in client.outliner.list_actors():
    print(actor.name, actor.actor_class, actor.actor_id)

for bp in client.outliner.list_blueprints():
    print(bp.class_path)

client.outliner.select_actor("robot_0")     # highlights in the editor
```

Use `add_quick_convert` to convert a static-mesh actor into a MuJoCo
collidable on the fly — handy for set-dressing imported assets:

```python
client.outliner.add_quick_convert(
    "Pallet_BP_2",
    by_name=True,
    static=True,                # fixed in the world
    complex_mesh=False,         # CoACD decomposition off
    coacd_threshold=0.05,
    friction=(1.0, 1.0, 1.0),
)
```

---

## Run PIE and discover

```python
result = client.sim.start(timeout_s=30.0)   # raises URLabPIEError on failure
assert result.is_ready
```

`sim.start()` returns a typed `PIEStartResult`. By default it raises
`URLabPIEError` if PIE didn't enter `Playing` (compile error, timeout).
Pass `raise_on_failure=False` if you want to inspect the result yourself:

```python
from urlab_client import PIEState

result = client.sim.start(raise_on_failure=False)
match result.state:
    case PIEState.READY:
        ...   # client.articulations is populated; no need to discover() again
    case PIEState.COMPILE_FAILED:
        print(result.compile_error)
    case PIEState.TIMEOUT:
        ...
```

The `READY` reply embeds a fresh handshake the client has already
absorbed; you don't have to call `discover()` again.

Resolve handles to articulations after PIE is up:

```python
robot = handles["robot_0"].runtime(client)
assert robot is not None  # None if PIE not running or actor was removed
```

Or look up directly:

```python
robot = client.articulations_by_id["robot_0"]   # by actor_id
robot = client.articulations["robot"]            # by MJCF prefix
```

To stop PIE:

```python
client.sim.stop()
```

`client.sim.status()` is a cheap query for the current PIE state +
sim_time, without affecting it.

---

## Step the sim

The simplest control loop:

```python
client = URLabClient("tcp://localhost", step_mode="direct")
client.discover()
client.sim.start()
robot = client.articulations["robot"]

for t in range(1000):
    robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})
    client.step(n_steps=1)

client.close()
```

`step()` returns the raw step reply for the rare case you need raw
fields, but most of the time you read state through the articulation
attributes (next section) — `client.data`, `art.qpos_array`,
`art.get_sensors()`, etc. are all updated in place.

---

## Read state and sensors

After every `step()` (or `reset()`), the local state mirror updates.
Read it through articulation attributes — these are zero-copy views,
not RPCs:

```python
# Root link, world frame
print(robot.root_pos_w)         # (3,) [x, y, z]
print(robot.root_quat_xyzw)     # (4,) ROS/SciPy convention

# Per-DoF, free-base excluded
print(robot.dof_qpos)
print(robot.dof_qvel)

# Body-frame derived obs (handy for locomotion policies)
print(robot.projected_gravity_b)
print(robot.root_lin_vel_b)
print(robot.root_ang_vel_b)
```

Sensors come back as a name → ndarray dict:

```python
sensors = robot.get_sensors()
print(sensors["imu_gyro"])       # (3,) angular rate
print(sensors["foot_force_fl"])  # (1,) or (3,) per the MJCF
```

Or per-handle via the bookkeeping object:

```python
print(robot.sensors["imu_gyro"].latest)
```

For full state including per-body `xpos` / `xquat` and actuator forces,
discover with `observations="full"`:

```python
client.discover(observations="full")
# ...
client.step()
print(robot.bodies["hand"].xpos)
print(robot.actuators["shoulder"].force)
```

---

## Send control

Articulations control behaviour depends on `art.control_mode`:

- **`ControlMode.UE_CONTROLLER`** — `ctrl` values are routed through
  whichever `UMjArticulationController` is attached to the articulation.
  The controller maps your setpoints to actuator forces according to
  its kind (PD is the bundled default, but it can be a passthrough, a
  custom one, etc.). For the PD controller, setpoints are joint targets
  in the controller's units (radians for hinges, metres for slides).
- **`ControlMode.RAW`** — the server passes ctrl straight to MuJoCo. Your
  values are interpreted by the actuator's `<general>` gain block.

Two ways to send ctrl:

```python
# Bulk dict — partial updates allowed (missing keys keep last value)
robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})

# Per-actuator
robot.actuators["shoulder"].set_control(0.5)

# Or set the buffer directly if you know the order
robot.ctrl_array[:] = [0.5, -0.2, 0.0, ...]
```

The buffer ships with the next `step()`. To inspect what UE actually
applied last frame:

```python
print(robot.last_applied_ctrl)
```

### Twist control

If a robot has a `UMjTwistController` (driven by ROS-Twist topics in
production), set its target through `client.runtime`:

```python
client.runtime.set_twist("robot", linear=(0.5, 0, 0), angular=(0, 0, 0.3))
```

### Switching control source

Useful for handing the wheel between the Python client and the in-engine
debug UI:

```python
client.runtime.set_control_source("zmq")    # ZMQ (you) drives ctrl
client.runtime.set_control_source("ui")     # UE debug UI drives ctrl
```

Pass `articulation="robot"` to scope the change to one articulation.

---

## Tune controllers

Articulations with a server-side controller expose it as
`art.controller`. PD controllers support typed setters:

```python
ctrl = robot.controller    # URLabPDController
ctrl.set_gains(
    kp={"shoulder": 300.0, "elbow": 200.0},
    kv={"shoulder": 25.0, "elbow": 18.0},
    torque_limit={"shoulder": 40.0},
)
ctrl.set_defaults(kp=100.0, kv=5.0, torque_limit=50.0)
```

Partial-patch: joints not mentioned keep their current value. Read live
gains:

```python
print(dict(ctrl.kp))
print(dict(ctrl.kv))
```

For non-PD controllers, use the generic surface:

```python
ctrl.configure(some_param=42)
ctrl.refresh()    # re-pull from server
print(ctrl.params)
```

If `art.controller is None`, the articulation is in raw passthrough mode —
your ctrl values reach MuJoCo directly.

See [Controllers and Gains](controllers_and_gains.md) for the
full schema.

---

## Cameras

Cameras live on `art.cameras` (articulation-local) or
`client.global_cameras` (scene-level). Each is a `URLabCameraView`.

Capture on demand, in step:

```python
client.step(n_steps=1, include_cameras=True)
frame = robot.cameras["wrist"].latest_frame   # (H, W, 4) RGBA uint8
```

Continuous capture (live mode only):

```python
client.runtime.set_mode("live")
# Per-camera SUB threads start automatically; latest_frame updates
# in the background.
while running:
    frame = robot.cameras["wrist"].latest_frame
    ts = robot.cameras["wrist"].sim_time
```

Camera modes (set in the MJCF / handshake):

| Mode | Shape | Channels |
|---|---|---|
| `real` | `(H, W, 4)` | RGBA |
| `depth` | `(H, W)` | float32 metres |
| `semantic` / `instance` | `(H, W, 4)` | BGRA tint |

See [Camera Capture Modes](../guides/camera_capture_modes.md) for the full
per-mode story (rendering, post-process, segmentation IDs).

---

## Recording and replay

Server-side recording is a sequence of state snapshots saved to disk.
Useful for replaying a teleop or scripted session deterministically.

```python
client.recording.start(name="trial_01", max_duration_s=60.0)
# ... drive the robot ...
client.recording.stop()
path = client.recording.save()    # absolute path to the .urlab_rec file
client.recording.clear()
```

Playback:

```python
client.replay.play("C:/recordings/trial_01.urlab_rec", loop=True)
# ...
client.replay.stop()
```

Or load + select + start manually:

```python
session = client.replay.load("C:/recordings/trial_01.urlab_rec")
client.replay.set_active(session.name)
client.replay.start()
```

See [Recording and Replay](recording_replay.md) for the full
lifecycle.

---

## Add a new policy

There are two ways to drive a robot from Python:

1. **Write a control loop** against `URLabClient` directly — no
   registration needed. Best for one-off scripts, debugging,
   custom controllers.
2. **Add an entry to the policy registry** — for pretrained policies
   that live alongside the bundled set so the dashboard, launcher, and
   `run.py` can find them by name.

### Path 1: a custom control loop

Anything you can do in Python you can do here. There's no policy base
class to inherit from. Open a client, build state into your action,
push it.

```python
from urlab_client import URLabClient

with URLabClient("tcp://localhost", step_mode="direct") as client:
    client.discover()
    client.sim.start()
    robot = client.articulations["vx300s"]

    target_qpos = {"waist": 0.0, "shoulder": -0.3, "elbow": 0.6}
    for t in range(1000):
        # Read state
        qpos = {n: float(robot.get_qpos()[n]) for n in target_qpos}
        # Compute control (toy P controller)
        ctrl = {n: 5.0 * (target_qpos[n] - qpos[n]) for n in target_qpos}
        robot.set_ctrl(ctrl)
        client.step(n_steps=1)
```

Useful patterns to compose:

- `art.get_sensors()` for IMU / touch / contact-force inputs.
- `art.root_pos_w`, `art.root_quat_xyzw` for the world-frame base.
- `art.dof_qpos`, `art.dof_qvel` excluding the free-base root — the
  shape locomotion / manipulation policies usually expect.
- `client.runtime.set_twist(...)` instead of joint ctrl when the robot
  has a built-in `UMjTwistController` (locomotion).

### Path 2: register a pretrained policy

The bridge has two policy adapters today, each a folder under
`src/urlab_policy/adapters/`:

- [`adapters/robojudo/`](https://github.com/URLab-Sim/urlab_bridge/tree/main/src/urlab_policy/adapters/robojudo) — for [RoboJuDo](https://github.com/URLab-Sim/RoboJuDo)-shaped policies. Bundled policies (Unitree G1, Go2 WTW, BeyondMimic, AMO, H2H, …) all live here.
- [`adapters/mjlab/`](https://github.com/URLab-Sim/urlab_bridge/tree/main/src/urlab_policy/adapters/mjlab) — for mjlab-trained policies, driven via the framework-agnostic `PolicyRunner` + `TaskSpec` machinery.

Each adapter has its own `registry.py` contributing entries to the
top-level [`urlab_policy.registry.POLICIES`](https://github.com/URLab-Sim/urlab_bridge/blob/main/src/urlab_policy/registry.py).
Open the adapter that matches your backend, copy an existing entry as
a template, run with `urlab-policy --policy <name>`.

For the bundled-policy table, the registry merge mechanics, and
`required_step_mode` semantics, see
[Policy Registry](policy_registry.md).

---

## Pick a step mode

URLab supports three step modes, each suited to a different role:

### `direct` — deterministic rollouts, data collection

```python
client = URLabClient("tcp://localhost", step_mode="direct")
```

UE steps the sim `n_steps` times per RPC, applying the ctrl you sent.
The RPC returns when stepping is done. **Deterministic** — the same
seed + ctrl sequence yields the same trajectory. Use this when you
need repeatability: data collection, scripted demos, sim-to-sim
verification.

### `live` — low-latency control loops, real-time playback

```python
client = URLabClient("tcp://localhost", step_mode="live")
```

UE physics runs autonomously at its own rate. Each `step(n_steps=0)`
just stamps the latest ctrl and reads back current state. State also
streams continuously over a SUB socket — `client.data` is up-to-date
even between `step()` calls. **Not deterministic** (wall-clock dependent).

### `puppet` — MJX, MPC, custom integrators

```python
client = URLabClient("tcp://localhost", step_mode="puppet")
```

The client calls `mj_step` locally and pushes the resulting qpos/qvel to
UE for rendering. UE doesn't run physics in this mode. Use this when
your sim-side logic is in JAX / a different MuJoCo version / an MPC
loop. `n_steps=0` skips local stepping and just pushes whatever is in
`client.data` (handy for hand-authored states).

> `apply_xfrc` is inert in `puppet` mode — UE's `xfrc_applied` is
> overwritten by the bridge each step.

### Switching at runtime

```python
client.runtime.set_mode("puppet")
```

See [Networking](../guides/networking.md) for the full UE-side
architecture.

---

## Common pitfalls

### `editor_only_op` errors

You called a `client.scene.*` op while PIE was running, or a
`client.runtime.*` op before PIE started. Check `client.manager_present`.

### Names don't match my MJCF

UE blueprint uniqueness can rename joints / actuators (`shoulder_pan`
becomes `shoulder_pan_2` if there's a clash). Use `art.resolve_joint`
and `art.resolve_actuator` to map original-XML names to live keys, or
read `art.joints.keys()` / `art.actuators.keys()` to see the live set.

### State looks stale

You're reading from a thread that doesn't hold the data lock, or you're
in `live` mode and reading between SUB callbacks. Acquire the
lock for non-step-thread reads:

```python
with client._data_lock:
    qpos = robot.qpos_array.copy()
```

### Version mismatch on `discover()`

`URLabVersionMismatch` means the server's MuJoCo build differs from your
local one. Either bring them into sync, or pass
`mujoco_version_check=False` to the constructor (downgrade to a warning).

### `set_qpos` doesn't seem to land

Make sure the articulation is in a mode that respects external state
overrides. In `puppet` mode, `client.data` is canonical and pushed each
step — call `client.runtime.set_qpos` *and* update `client.data.qpos`
yourself, or use `art.qpos_array[...] = ...` before stepping.

### Camera frames are `None`

In `direct` / `auto` modes you must pass `include_cameras=True` to
`step()` to capture this frame. In `live` mode the SUB thread
populates `latest_frame` asynchronously — give it a frame to warm up
before reading.

---

## See also

- [Python API Reference](api.md) — every method, every argument,
  every reply field.
- [Networking](../guides/networking.md) — UE-side architecture
  and the three step modes.
- [Policy Registry](policy_registry.md) — bundled pretrained
  policies and how to add new ones.
- [Step Server Protocol](../reference/step_server.md) — wire-protocol
  reference.
- The `meta` RPC — `client._rpc("meta", {})` returns the live op
  catalogue. The bridge's
  [`tools/gen_stubs.py`](https://github.com/URLab-Sim/urlab_bridge/blob/main/tools/gen_stubs.py)
  regenerates the typed `.pyi` from this.
