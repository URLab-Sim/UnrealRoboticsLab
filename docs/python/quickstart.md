# Python Quickstart

Connect to a running editor, author a scene, enter PIE, send control,
and step the simulation. This is the end-to-end Python flow. For the
per-method contract, keep [API Reference](api.md) open alongside it.

Before you start, install the bridge (see [Overview](index.md#install-the-bridge)).

## The shortest loop

```python
from urlab_client import URLabClient

# `with` closes the transport even on exceptions.
with URLabClient("tcp://localhost", step_mode="direct") as client:
    client.discover()                       # handshake + load the model
    client.sim.start()                      # enter PIE

    robot = client.articulations["robot"]   # keyed by MJCF model name
    for _ in range(1000):
        robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})
        client.step(n_steps=1)              # advance one physics step
        print(robot.root_pos_w, robot.dof_qpos[:2])

    client.sim.stop()
```

The rest of this page explains each stage.

## Connect

The editor listens on `tcp://localhost:5559` by default.

```python
from urlab_client import URLabClient

with URLabClient("tcp://localhost") as client:
    client.discover()
    print("manager_present:", client.manager_present)
```

Two states matter:

- `manager_present is False` - the editor is open but PIE is not
  running. Editor-time namespaces (`client.scene.*`,
  `client.outliner.*`, `client.debug.*`, `client.viewport.*`) work;
  `client.runtime.*` does not.
- `manager_present is True` - PIE (or standalone) is live. Every
  namespace works.

`client.sim.start()` is the transition between the two.

### Transports

The default ZMQ transport works everywhere, including across hosts.
On the same machine, the shared-memory transport cuts tail latency,
especially on camera streams:

```python
client = URLabClient("tcp://localhost", transport="shm")
```

SHM still runs the handshake and any oversize op over ZMQ, then resumes
shared memory. Use ZMQ when Python and the editor are on different
machines. See [API Reference](api.md#transports).

## Author a scene

Before PIE, build a level from Python. Each `URLabAsset` is one MJCF
file; the bridge imports it as a Blueprint and spawns one actor.

```python
from urlab_client import URLabAsset

assets = [
    URLabAsset(actor_id="robot_0", xml="C:/path/to/robot.xml", location=(0, 0, 0)),
    URLabAsset(actor_id="prop_0",  xml="C:/path/to/prop.xml",  location=(1.0, 0, 0.5),
               rotation_euler=(0, 0, 90)),
]

handles = client.scene.apply_scene("Level_Sim", assets, save=True)
print(handles["robot_0"].actor_path)
```

`apply_scene` is a composer over the lower-level primitives. For finer
control, call them directly:

```python
client.scene.create_level("Level_Sim")          # or load_level if it exists
bp = client.scene.import_xml("C:/path/to/robot.xml")
handle = client.scene.spawn_actor(blueprint=bp, actor_id="robot_0", location=(0, 0, 0))
client.scene.save_level()
```

`actor_id` is the stable handle the bridge uses to reconcile a UE actor
across PIE restarts. It survives renames; UE actor names do not.

`apply_scene` is idempotent: calling it again with the same `actor_id`
updates the existing actor in place instead of duplicating it. Check
`handle.was_existing` to see which happened.

## Run PIE and discover

```python
result = client.sim.start(timeout_s=30.0)   # raises URLabPIEError on failure
assert result.is_ready
```

`sim.start()` returns a typed `PIEStartResult`. By default it raises
`URLabPIEError` if PIE did not reach `Playing` (compile error or
timeout). Pass `raise_on_failure=False` to inspect the result yourself:

```python
from urlab_client import PIEState

result = client.sim.start(raise_on_failure=False)
match result.state:
    case PIEState.READY:           ...   # articulations populated already
    case PIEState.COMPILE_FAILED:  print(result.compile_error)
    case PIEState.TIMEOUT:         ...
```

On `READY`, the reply embeds a fresh handshake the client has already
absorbed; you do not need to call `discover()` again. Resolve handles:

```python
robot = handles["robot_0"].runtime(client)        # via spawn handle
robot = client.articulations_by_id["robot_0"]      # by actor_id
robot = client.articulations["robot"]              # by MJCF prefix
```

Stop PIE with `client.sim.stop()`. `client.sim.status()` is a cheap
read of the current state plus sim time.

## Send control

Control behaviour depends on `art.control_mode`:

- `ControlMode.UE_CONTROLLER` - `ctrl` values route through the
  articulation's attached controller (PD by default), which converts
  your setpoints into actuator forces against live `qpos` / `qvel`.
- `ControlMode.RAW` - values pass straight to MuJoCo and are
  interpreted by the actuator's gain block.

```python
# Bulk dict; partial updates allowed (missing keys keep last value).
robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})

# Per-actuator.
robot.actuators["shoulder"].set_control(0.5)
```

The buffer ships with the next `step()`. Read what UE actually applied
with `robot.last_applied_ctrl`.

### Twist control

If a robot carries a `UMjTwistController`, drive it by twist instead of
joint ctrl:

```python
client.runtime.set_twist("robot", linear=(0.5, 0, 0), angular=(0, 0, 0.3))
```

## Step the sim

```python
for t in range(1000):
    robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})
    client.step(n_steps=1)
```

`step()` returns the raw reply, but you usually read state through the
articulation attributes, which update in place after every `step()` or
`reset()`:

```python
print(robot.root_pos_w)          # (3,) world-frame position
print(robot.root_quat_xyzw)      # (4,) SciPy / ROS convention
print(robot.dof_qpos)            # per-DoF, free base excluded
print(robot.dof_qvel)
print(robot.projected_gravity_b) # body-frame derived obs
```

Sensors come back as a name to ndarray dict:

```python
sensors = robot.get_sensors()
print(sensors["imu_gyro"])
```

For per-body `xpos` / `xquat` and actuator forces, discover with the
fuller observation level:

```python
client.discover(observations="full")
client.step()
print(robot.bodies["hand"].xpos)
print(robot.actuators["shoulder"].force)
```

## Tune controllers

Articulations with a server-side controller expose it as
`art.controller`. PD controllers take typed setters:

```python
art = client.articulations["robot"]
art.controller.set_gains(
    kp={"shoulder": 300.0, "elbow": 200.0},
    kv={"shoulder": 25.0,  "elbow": 18.0},
    torque_limit={"shoulder": 40.0},
)
art.controller.set_defaults(kp=100.0, kv=5.0, torque_limit=50.0)
```

!!! warning "set_gains lives on the controller"
    Gains are set through `art.controller.set_gains(...)`, not on
    `client.runtime`. If `art.controller is None`, the articulation is
    in raw passthrough mode and your ctrl reaches MuJoCo directly.

Partial-patch: joints you do not mention keep their current value. Read
live gains with `dict(art.controller.kp)`. See
[Controllers](api.md#controllers).

## Cameras

Cameras live on `art.cameras` (articulation-local) or
`client.global_cameras` (scene-level). Capture on demand in `direct`
or `puppet`:

```python
client.step(n_steps=1, include_cameras=True)
frame = robot.cameras["wrist"].latest_frame   # (H, W, 4) RGBA uint8
```

In `live` mode, per-camera SUB threads keep `latest_frame` fresh in the
background. See [API Reference](api.md#cameras) for the per-mode shapes.

## Record and replay

A recording is a sequence of state snapshots, saved to disk as a
`.json` file and replayed deterministically.

```python
client.recording.start(name="trial_01", max_duration_s=60.0)
# ... drive the robot ...
client.recording.stop()
path = client.recording.save()    # absolute path to the .json file
```

Playback (requires `direct` or `puppet`):

```python
client.replay.play("C:/recordings/trial_01.json", loop=True)
# ...
client.replay.stop()
```

## Pick a step mode

URLab supports three modes; pick at construction or switch at runtime
with `client.runtime.set_mode(...)`.

=== "direct"

    ```python
    client = URLabClient("tcp://localhost", step_mode="direct")
    ```

    UE steps the sim `n_steps` times per RPC against the ctrl you sent,
    and returns when stepping is done. Deterministic: the same scene,
    keyframe, and ctrl sequence reproduce the trajectory. Use for data
    collection and scripted demos.

=== "live"

    ```python
    client = URLabClient("tcp://localhost", step_mode="live")
    ```

    UE runs physics autonomously at its own rate. `step()` stamps the
    latest ctrl and reads current state; state also streams over a SUB
    socket. Not deterministic (wall-clock dependent). Use for
    low-latency control loops and real-time playback.

=== "puppet"

    ```python
    client = URLabClient("tcp://localhost", step_mode="puppet")
    ```

    The client calls `mj_step` locally and pushes qpos/qvel to UE for
    rendering; UE does not run physics. Use when your sim logic is in
    JAX, a different MuJoCo version, or an MPC loop.

    !!! note
        `apply_xfrc` is inert in `puppet` mode; UE's `xfrc_applied` is
        overwritten by the bridge each step.

## Common pitfalls

`not_in_editor` errors

: You called a `client.scene.*` op while the editor had no registered
  handler for the current state, or a `client.runtime.*` op before PIE
  started. Check `client.manager_present`.

Names do not match my MJCF

: UE Blueprint uniqueness can rename joints / actuators on a clash. Use
  `art.resolve_joint` / `art.resolve_actuator` to map original-XML
  names to live keys, or inspect `art.joints.keys()`.

Camera frames are `None`

: In `direct` / `puppet`, pass `include_cameras=True` to `step()`. In
  `live`, give the SUB thread a frame to warm up before reading.

## See also

- [API Reference](api.md) - every method and reply field.
- [Running Policies](policies.md) - bundled policies and the gym wrapper.
- [Protocol Reference](../reference/protocol.md) - the wire ops.
